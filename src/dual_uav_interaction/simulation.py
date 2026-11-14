"""One shared physics clock, isolated PX4 contexts, and artificial loads."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import Any, Sequence

import mujoco
import numpy as np
import numpy.typing as npt

from .fleet import (box_attachment_offsets, horizontal_distances, validate_fleet)
from .interaction import InteractionConfig, compute_pair_accelerations
from .payload import PayloadConfig, PayloadSystem
from .px4_bridge import (
    Px4Bridge,
    compute_hover_matched_total_max_thrust,
    equal_rotor_thrust_scales,
    read_px4_param,
)
from .scene import ASSETS, AircraftBindings, apply_accelerations, load_scene

FloatArray = npt.NDArray[np.float64]
AIRBORNE_PHASES = ("takeoff", "hover", "landing")


@dataclass
class Aircraft:
    """One independent control/feedback context within the common physics world."""

    bindings: AircraftBindings
    bridge: Any
    home: FloatArray
    target: FloatArray
    previous_velocity: FloatArray
    rotor_thrusts: FloatArray
    arm_hold: FloatArray
    streamed_target: FloatArray
    streamed_velocity: FloatArray
    phase: str = "disarmed"
    arm_pending: bool = False
    arm_started: float = 0
    landed_since: float | None = None
    landing_disarm_requested: bool = False


class DualSimulation:
    """Coordinate complete aircraft without advancing separate worlds.

    Args:
        separation_m: Initial horizontal center separation.
        takeoff_height_m: Height above each aircraft's initial root position.
        instance_base: First PX4 instance index.
        bridges: Optional socket-free bridge doubles for offline checks.
        payload_config: Shared cargo settings, or None for unloaded operation.
        aircraft_count: Two or four aircraft; low-level default retains two.
    """

    def __init__(
        self,
        separation_m: float = 2,
        takeoff_height_m: float = 2,
        instance_base: int = 0,
        bridges: Sequence[Any] | None = None,
        payload_config: PayloadConfig | None = None,
        aircraft_count: int = 2,
    ) -> None:
        if (
            not np.isfinite(takeoff_height_m)
            or not 0.5 <= takeoff_height_m <= 10
        ):
            raise ValueError("takeoff height must be in .5..10 m")
        validate_fleet(aircraft_count, instance_base)
        if bridges is not None and len(bridges) != aircraft_count:
            raise ValueError("one independent bridge per aircraft is required")
        self.aircraft_count = aircraft_count
        self.model, self.data, self.bindings = load_scene(separation_m, payload_config, aircraft_count)
        self.payload_config = payload_config
        self.payload_system = PayloadSystem(self.model, payload_config) if payload_config else None
        self._initial_gripper_offsets = np.array([
            self.data.site(binding.prefix + "endpoint").xpos - self.data.xpos[binding.root_id]
            for binding in self.bindings
        ])
        self.floor_id = self.model.geom("floor").id
        self.takeoff_height_m = takeoff_height_m
        self.interaction = InteractionConfig()
        self.aircraft: list[Aircraft] = []
        self.exit_requested = False
        self.takeoff_pending = False
        self.last_error = ""
        self.last_accelerations = np.zeros((aircraft_count, 3))
        hover = read_px4_param(
            ASSETS / "configs/X4-950.params", "MPC_THR_HOVER", 0.5
        )
        for index, binding in enumerate(self.bindings):
            instance = instance_base + index
            bridge = (
                bridges[index]
                if bridges is not None
                else Px4Bridge(
                    equal_rotor_thrust_scales(
                        compute_hover_matched_total_max_thrust(
                            binding.mass_kg, hover
                        )
                    ),
                    sim_tcp_port=4560 + instance,
                    command_udp_port=14580 + instance,
                    system_id=instance + 1,
                    simulator_system_id=200 + instance,
                    command_system_id=206 + instance,
                )
            )
            home = self.data.xpos[binding.root_id].copy()
            bridge.set_local_origin(home)
            self.aircraft.append(
                Aircraft(
                    bindings=binding,
                    bridge=bridge,
                    home=home,
                    target=home.copy(),
                    previous_velocity=np.zeros(3),
                    rotor_thrusts=np.zeros(4),
                    arm_hold=self.data.qpos[binding.arm_qpos].copy(),
                    streamed_target=home.copy(),
                    streamed_velocity=np.zeros(3),
                )
            )

    def connect(self) -> None:
        """Open all bridge endpoints, closing partial initialization on failure."""
        try:
            for item in self.aircraft:
                item.bridge.connect()
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        """Close only the bridge connections belonging to this application."""
        for item in self.aircraft:
            item.bridge.close()

    def set_interaction(
        self, mode: str, horizontal: float, vertical: float, length: float
    ) -> None:
        """Replace all settings together, retaining the old settings on error.

        Args:
            mode: off, horizontal, vertical, or both.
            horizontal: Horizontal peak acceleration in m/s².
            vertical: Downward peak acceleration in m/s².
            length: Positive horizontal half-amplitude distance in metres.
        """
        candidate = InteractionConfig(mode, horizontal, vertical, length)
        self.interaction = candidate
        print(
            f"Interaction settings: {asdict(candidate)} (artificial acceleration model)",
            flush=True,
        )

    def command(self, action: str, value: Any = None) -> None:
        """Apply one operator command to the fleet in the shared world.

        Args:
            action: arm, request_takeoff, takeoff, land, emergency, exit,
                separation, or move.
            value: Separation increment or common world-frame target offset.
        """
        now = time.perf_counter()
        if action in ("land", "emergency", "exit"):
            self.takeoff_pending = False
            for item in self.aircraft:
                item.arm_pending = False
        if action == "request_takeoff":
            if self.takeoff_pending:
                return
            if any(item.phase not in ("disarmed", "armed", "landed")
                   for item in self.aircraft):
                raise RuntimeError("takeoff requires all aircraft on the ground")
            self.command("arm")
            self.takeoff_pending = True
        elif action == "arm":
            if not all(
                item.bridge.heartbeat_is_recent() for item in self.aircraft
            ):
                raise RuntimeError(
                    "wait for fresh heartbeats from all PX4 instances"
                )
            for item in self.aircraft:
                item.arm_pending = True
                item.arm_started = now
                item.bridge.request_arm()
        elif action == "takeoff":
            if not all(item.bridge.armed for item in self.aircraft):
                raise RuntimeError(
                    "all aircraft must be confirmed armed before takeoff"
                )
            if any(
                item.phase not in ("disarmed", "armed", "landed")
                for item in self.aircraft
            ):
                raise RuntimeError(
                    "takeoff requires all aircraft on the ground"
                )
            for item in self.aircraft:
                item.target[2] = item.home[2] + self.takeoff_height_m
                item.phase = "takeoff"
                item.landing_disarm_requested = False
        elif action == "land":
            for item in self.aircraft:
                if item.phase in ("takeoff", "hover"):
                    # Position below the supported root height ensures contact,
                    # rather than an asymptotic near-floor hover.
                    item.target[2] = item.home[2] - 0.2
                    item.phase = "landing"
                    item.landed_since = None
                    item.landing_disarm_requested = False
        elif action == "emergency":
            for item in self.aircraft:
                item.arm_pending = False
                item.phase = "emergency"
                item.rotor_thrusts[:] = 0
                self.data.ctrl[item.bindings.thrust_ids] = 0
                item.bridge.request_disarm()
        elif action == "exit":
            self.exit_requested = True
        elif action == "separation":
            targets = np.array([item.target for item in self.aircraft])
            distances = horizontal_distances(targets)
            previous = float(distances[np.triu_indices(self.aircraft_count, 1)].min())
            desired = previous + float(value)
            if not np.isfinite(desired) or desired < 1.5:
                raise ValueError("target separation must be finite and >=1.5 m")
            center = targets[:, :2].mean(axis=0)
            targets[:, :2] = center + (targets[:, :2] - center) * desired / previous
            self._validate_target_positions(targets)
            for item, target in zip(self.aircraft, targets):
                item.target[:] = target
        elif action == "move":
            offset = np.asarray(value, dtype=float)
            if offset.shape != (3,) or not np.isfinite(offset).all():
                raise ValueError("move offset must be a finite 3-vector")
            targets = np.array([item.target for item in self.aircraft])
            for index, item in enumerate(self.aircraft):
                if item.phase in ("takeoff", "hover"):
                    targets[index] += offset
            self._validate_target_positions(targets)
            for item, target in zip(self.aircraft, targets):
                item.target[:] = target
        else:
            raise ValueError(f"unknown command: {action}")

    def move_aircraft(self, index: int, offset: Sequence[float]) -> None:
        """Move one aircraft target in world XYZ, preserving every peer target.

        Args:
            index: Zero-based aircraft index (A/B or A/B/C/D).
            offset: Finite world-frame target increment in metres.

        Raises:
            ValueError: Invalid index, vector or resulting horizontal spacing.
            RuntimeError: Selected aircraft is not armed and flying.
        """
        if (isinstance(index, bool) or not isinstance(index, (int, np.integer))
                or not 0 <= index < self.aircraft_count):
            raise ValueError(f"aircraft index must be in 0..{self.aircraft_count-1}")
        delta = np.asarray(offset, dtype=float)
        if delta.shape != (3,) or not np.isfinite(delta).all():
            raise ValueError("move offset must be a finite 3-vector")
        item = self.aircraft[index]
        if item.phase not in ("takeoff", "hover") or not item.bridge.armed:
            raise RuntimeError("selected aircraft must be armed and flying")
        targets = np.array([peer.target.copy() for peer in self.aircraft])
        with np.errstate(over="ignore", invalid="ignore"):
            targets[index] += delta
        self._validate_target_positions(targets)
        item.target[:] = targets[index]

    def _validate_target_positions(self, targets: FloatArray) -> None:
        """Reject an entire candidate formation before committing any target.

        Args:
            targets: Candidate world positions of all aircraft, shape (N, 3).

        Raises:
            ValueError: Nonfinite targets, insufficient separation or an
                infeasible horizontal rope span in the level target geometry.
        """
        if not np.isfinite(targets).all():
            raise ValueError("resulting target must be finite")
        distances = horizontal_distances(targets)
        pairs = distances[np.triu_indices(self.aircraft_count, 1)]
        maximum = 2.25 if self.payload_system and self.aircraft_count == 2 else 10.
        if pairs.min() < 1.5 or pairs.max() > maximum:
            raise ValueError(f"target separation must be in 1.5..{maximum:g} m")
        if self.payload_system and self.aircraft_count == 4:
            # Project fixed initial arm geometry at each root target. This is
            # an operator target constraint, not a dynamic stability guarantee.
            # endpoint coordinates are local to link6: recover the original
            # world pose rather than treating them as root-relative offsets.
            original = self._initial_gripper_offsets
            anchors = targets[:, :2] + original[:, :2]
            center = anchors.mean(axis=0)
            ends = center + box_attachment_offsets(4)[:, :2]
            spans = np.linalg.norm(anchors - ends, axis=1)
            if np.any(spans >= self.payload_config.rope.rest_length_m):
                raise ValueError("target formation exceeds horizontal rope reach")

    def _sensor(self, sensor_id: int) -> FloatArray:
        """Read a validated three-axis sensor from the current forward state."""
        address = self.model.sensor_adr[sensor_id]
        result = self.data.sensordata[address : address + 3].copy()
        if not np.isfinite(result).all():
            raise FloatingPointError("non-finite MuJoCo sensor")
        return result

    def _grounded(
        self, item: Aircraft, position: FloatArray, velocity: FloatArray
    ) -> bool:
        """Require floor contact and a settled vehicle near its starting height.

        Args:
            item: Vehicle whose subtree may contact the world floor.
            position: Current world root position in metres.
            velocity: Current world root velocity in m/s.

        Returns:
            Whether this aircraft is touching the floor at low speed.
        """
        if (
            abs(position[2] - item.home[2]) >= 0.05
            or np.linalg.norm(velocity) >= 0.2
        ):
            return False
        for contact in self.data.contact:
            if contact.dist > 0:
                continue
            if contact.geom1 == self.floor_id:
                other = contact.geom2
            elif contact.geom2 == self.floor_id:
                other = contact.geom1
            else:
                continue
            if self.model.geom_bodyid[other] in item.bindings.body_ids:
                return True
        return False

    def _update_aircraft(self, item: Aircraft) -> None:
        """Feed one vehicle's same-time state to its own flight controller."""
        binding = item.bindings
        position = self.data.xpos[binding.root_id].copy()
        velocity = self.data.qvel[binding.dof : binding.dof + 3].copy()
        quaternion = self.data.qpos[binding.qpos + 3 : binding.qpos + 7].copy()
        if not all(
            np.isfinite(vector).all()
            for vector in (position, velocity, quaternion)
        ):
            raise FloatingPointError("non-finite aircraft state")
        acceleration = (
            velocity - item.previous_velocity
        ) / self.model.opt.timestep
        item.previous_velocity = velocity
        item.bridge.send_state(
            position,
            velocity,
            acceleration,
            quaternion,
            self._sensor(binding.gyro_id),
            self._sensor(binding.accelerometer_id),
        )
        item.bridge.send_command_heartbeat()
        item.bridge.poll_status()
        if item.bridge.armed:
            item.arm_pending = False
            if item.phase in ("disarmed", "landed"):
                item.phase = "armed"
        elif item.phase == "landing":
            grounded = self._grounded(item, position, velocity)
            if not (grounded and item.landing_disarm_requested):
                raise RuntimeError(
                    f"{binding.prefix} unexpected disarm before confirmed landing"
                )
            item.phase = "landed"
        elif item.phase == "armed":
            item.phase = "disarmed"
        elif item.phase in ("takeoff", "hover"):
            raise RuntimeError(
                f"{binding.prefix} unexpected disarm while flying"
            )
        if item.arm_pending:
            if time.perf_counter() - item.arm_started > 10:
                item.arm_pending = False
                raise RuntimeError(
                    f"{binding.prefix} arm confirmation timed out: {item.bridge.last_statustext}"
                )
            item.bridge.request_arm()
        if item.phase in AIRBORNE_PHASES:
            # Loaded position ramp: .25 m/s and .1 m/s², with no
            # velocity/acceleration feed-forward or native PX4 changes.
            if self.payload_system:
                delta = item.target - item.streamed_target
                distance = float(np.linalg.norm(delta))
                timestep = self.model.opt.timestep
                # Braking-distance speed limit, with bounded change in speed.
                desired_velocity = delta / max(distance, 1e-12) * min(
                    .25, np.sqrt(2 * .1 * distance)
                )
                velocity_change = desired_velocity - item.streamed_velocity
                change_norm = float(np.linalg.norm(velocity_change))
                item.streamed_velocity += velocity_change * min(
                    1., .1 * timestep / max(change_norm, 1e-12)
                )
                motion = item.streamed_velocity * timestep
                if np.linalg.norm(motion) >= distance:
                    item.streamed_target[:] = item.target
                    item.streamed_velocity[:] = 0
                else:
                    item.streamed_target += motion
            else:
                item.streamed_target[:] = item.target
            item.bridge.send_position_target(item.streamed_target, 0.0)
            if not item.bridge.offboard:
                item.bridge.request_offboard()
            if (
                item.phase == "takeoff"
                and abs(position[2] - item.target[2]) < 0.15
            ):
                item.phase = "hover"
            if item.phase == "landing" and self._grounded(
                item, position, velocity
            ):
                if item.landed_since is None:
                    item.landed_since = float(self.data.time)
                if self.data.time - item.landed_since > 0.7:
                    item.landing_disarm_requested = True
                    item.bridge.request_disarm()
            else:
                item.landed_since = None
        thrusts = np.asarray(item.bridge.receive_rotor_thrusts(), dtype=float)
        if thrusts.shape != (4,) or not np.isfinite(thrusts).all():
            raise FloatingPointError("invalid rotor commands")
        item.rotor_thrusts = (
            thrusts
            if item.bridge.armed and item.phase != "emergency"
            else np.zeros(4)
        )
        if item.phase in AIRBORNE_PHASES and item.bridge.control_is_stale(0.5):
            raise RuntimeError(f"{binding.prefix} stale PX4 actuator stream")
        self.data.ctrl[binding.thrust_ids] = item.rotor_thrusts
        self.data.ctrl[binding.spin_ids] = (
            [180, -180, -180, 180] if item.rotor_thrusts.sum() > 0.001 else 0
        )
        self.data.ctrl[binding.arm_ids] = item.arm_hold

    def step(self) -> dict[str, Any]:
        """Compute all directed loads from one snapshot, then advance once.

        Returns:
            Log sample describing the same pre-step state and applied loads.
        """
        # mj_step's position/sensor caches lag its updated qpos. Refresh first.
        self.data.qfrc_applied[:] = 0
        mujoco.mj_forward(self.model, self.data)
        for item in self.aircraft:
            self._update_aircraft(item)
        if self.takeoff_pending and all(item.bridge.armed for item in self.aircraft):
            self.command("takeoff")
            self.takeoff_pending = False
        positions = np.array(
            [self.data.xpos[b.root_id] for b in self.bindings]
        )
        airborne = [
            item.phase in AIRBORNE_PHASES
            and item.bridge.armed
            and positions[index, 2] > item.home[2] + 0.15
            for index, item in enumerate(self.aircraft)
        ]
        source_active = [
            enabled and item.rotor_thrusts.sum() > 0.001
            for enabled, item in zip(airborne, self.aircraft)
        ]
        accelerations = compute_pair_accelerations(
            positions, self.interaction, source_active
        )
        accelerations[~np.array(airborne)] = 0
        apply_accelerations(
            self.model, self.data, self.bindings, accelerations
        )
        self.last_accelerations = accelerations
        payload, ropes = (
            self.payload_system.apply(self.data)
            if self.payload_system else (None, [])
        )
        distances = horizontal_distances(positions)
        minimum_distance = float(distances[np.triu_indices(self.aircraft_count, 1)].min())
        sample = {
            "aircraft_count": self.aircraft_count,
            "pairwise_separations_m": distances.tolist(),
            "min_separation_m": minimum_distance,
            "time_s": float(self.data.time),
            "interaction": asdict(self.interaction),
            "separation_m": minimum_distance,
            "aircraft": [],
            "payload": payload,
            "ropes": ropes,
        }
        for index, item in enumerate(self.aircraft):
            binding = item.bindings
            sample["aircraft"].append(
                {
                    "name": binding.prefix[0].upper(),
                    "system_id": (
                        item.bridge.command_target_system
                        if isinstance(item.bridge, Px4Bridge)
                        else index + 1
                    ),
                    "phase": item.phase,
                    "armed": bool(item.bridge.armed),
                    "offboard": bool(item.bridge.offboard),
                    "position": positions[index].tolist(),
                    "target": item.target.tolist(),
                    "position_setpoint": item.streamed_target.tolist(),
                    "quaternion_wxyz": self.data.qpos[
                        binding.qpos + 3 : binding.qpos + 7
                    ].tolist(),
                    "velocity": self.data.qvel[
                        binding.dof : binding.dof + 3
                    ].tolist(),
                    "rotor_thrust_n": item.rotor_thrusts.tolist(),
                    "disturbance_source_active": bool(source_active[index]),
                    "interaction_acceleration": accelerations[index].tolist(),
                    "interaction_force_n": (
                        binding.mass_kg * accelerations[index]
                    ).tolist(),
                    "interaction_moment_about_com_nm": [0.0, 0.0, 0.0],
                    "arm_deviation_rad": (
                        self.data.qpos[binding.arm_qpos] - item.arm_hold
                    ).tolist(),
                    "rope_force_n": ropes[index]["force_on_anchor_n"] if ropes else [0., 0., 0.],
                    "rope_moment_about_com_nm": (
                        ropes[index]["moment_about_aircraft_com_nm"]
                        if ropes else [0., 0., 0.]
                    ),
                }
            )
        mujoco.mj_step(self.model, self.data)
        return sample
