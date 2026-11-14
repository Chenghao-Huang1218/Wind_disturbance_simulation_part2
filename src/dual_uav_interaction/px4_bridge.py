"""MAVLink transport adapted from Wind_field_simulation; see docs/PROVENANCE.md."""
from __future__ import annotations
import time
from pathlib import Path
from typing import Any, Callable, Sequence
import numpy as np
import numpy.typing as npt
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
FloatArray = npt.NDArray[np.float64]
GRAVITY_M_S2 = 9.80665
BAROMETER_RANDOM_SEED = 0
BAROMETER_NOISE_STD_HPA = 0.01

def _validated_vector3(values: Sequence[float], name: str) -> FloatArray:
    """Returns a finite three-element floating-point vector.

    Args:
        values: Candidate vector values.
        name: Field name used in validation errors.

    Returns:
        A defensive NumPy copy with shape ``(3,)``.

    Raises:
        ValueError: If the vector has the wrong shape or non-finite values.
    """
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (3,):
        raise ValueError(f"{name} must have shape (3,)")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain finite values")
    return vector.copy()


def read_px4_param(path: Path, name: str, default: float) -> float:
    """Reads a numeric value from a PX4 parameter export.

    Args:
        path: PX4 ``.params`` file.
        name: Parameter name to locate.
        default: Value returned when the file or value is unavailable.

    Returns:
        Parsed parameter value or ``default``.
    """
    if not path.exists():
        return float(default)
    with path.open("r", encoding="utf-8", errors="replace") as param_file:
        for line in param_file:
            fields = line.strip().split()
            if len(fields) >= 5 and fields[2] == name:
                try:
                    return float(fields[3])
                except ValueError:
                    return float(default)
    return float(default)


def compute_hover_matched_total_max_thrust(
    mass_kg: float,
    hover_thrust: float,
    gravity_m_s2: float = GRAVITY_M_S2,
) -> float:
    """Computes full-scale thrust from PX4 normalized hover thrust.

    Args:
        mass_kg: Vehicle subtree mass in kilograms.
        hover_thrust: PX4 ``MPC_THR_HOVER`` normalized command.
        gravity_m_s2: Gravity magnitude in m/s^2.

    Returns:
        Total four-rotor thrust at normalized command one, in newtons.
    """
    values = np.array([mass_kg, hover_thrust, gravity_m_s2], dtype=np.float64)
    if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
        raise ValueError("mass, hover thrust, and gravity must be finite and positive")
    return float(mass_kg * gravity_m_s2 / hover_thrust)


def equal_rotor_thrust_scales(
    total_max_thrust: float,
    rotor_count: int = 4,
) -> FloatArray:
    """Returns equal normalized-output-to-thrust scales for all rotors."""
    if rotor_count <= 0:
        raise ValueError("rotor_count must be positive")
    scale = float(total_max_thrust) / float(rotor_count)
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError("total_max_thrust must be finite and positive")
    return np.full(rotor_count, scale, dtype=np.float64)


def motor_outputs_to_thrusts(
    normalized_outputs: Sequence[float],
    rotor_thrust_scales: Sequence[float],
) -> FloatArray:
    """Maps four PX4 normalized allocator outputs to thrust in newtons."""
    outputs = np.asarray(normalized_outputs, dtype=np.float64)
    scales = np.asarray(rotor_thrust_scales, dtype=np.float64)
    if outputs.shape != (4,):
        raise ValueError("normalized_outputs must have shape (4,)")
    if scales.shape != (4,):
        raise ValueError("rotor_thrust_scales must have shape (4,)")
    if not np.all(np.isfinite(outputs)) or not np.all(np.isfinite(scales)):
        raise ValueError("motor outputs and thrust scales must be finite")
    if np.any(scales <= 0.0):
        raise ValueError("rotor_thrust_scales must be positive")
    return np.clip(outputs, 0.0, 1.0) * scales


def standard_atmosphere_pressure_hpa(altitude_m: float) -> float:
    """Returns pressure using PX4's first-layer standard atmosphere model.

    Formula and constants follow ``src/lib/atmosphere/atmosphere.cpp`` and
    ``atmosphere.h`` (US Standard Atmosphere 1976, up to 11 km).

    Args:
        altitude_m: Altitude above mean sea level in metres.

    Returns:
        Ideal absolute pressure in hPa, as required by HIL_SENSOR.

    Raises:
        ValueError: If altitude is non-finite or outside the first layer.
    """
    if not np.isfinite(altitude_m) or altitude_m > 11_000.0:
        raise ValueError("barometer altitude must be finite and <= 11000 m")
    temperature_ratio = (288.15 - 0.0065 * altitude_m) / 288.15
    return float(1013.25 * temperature_ratio ** (GRAVITY_M_S2 / (0.0065 * 287.1)))


class Px4Bridge:
    """Independent MAVLink bridge between MuJoCo and PX4 SITL."""

    EARTH_RADIUS_M = 6_378_137.0
    HOME_LAT_DEG = 22.0
    HOME_LON_DEG = 114.0
    HOME_ALT_M = 100.0
    PX4_OFFBOARD_MAIN_MODE = 6
    POSITION_ONLY_TARGET_TYPE_MASK = 0b0000101111111000
    MUJOCO_TO_PX4_AXIS = np.array([1.0, -1.0, -1.0], dtype=np.float64)

    def __init__(
        self,
        rotor_thrust_scales: Sequence[float],
        sim_tcp_port: int = 4560,
        command_udp_port: int = 14580,
        system_id: int = 1,
        simulator_system_id: int = 200,
        command_system_id: int = 206,
        connection_factory: Callable[..., Any] | None = None,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        """Initializes PX4 endpoints and conversion parameters.

        Args:
            rotor_thrust_scales: Four normalized-output-to-thrust scales in N.
            sim_tcp_port: PX4 simulator MAVLink TCP port.
            command_udp_port: PX4 Offboard MAVLink UDP port.
            connection_factory: Injectable pymavlink connection constructor.
            clock: Injectable monotonic wall clock.
        """
        scales = np.asarray(rotor_thrust_scales, dtype=np.float64)
        if scales.shape != (4,) or not np.all(np.isfinite(scales)):
            raise ValueError("rotor_thrust_scales must be finite with shape (4,)")
        if np.any(scales <= 0.0):
            raise ValueError("rotor_thrust_scales must be positive")
        self.rotor_thrust_scales = scales.copy()
        self.sim_tcp_port = int(sim_tcp_port)
        self.command_udp_port = int(command_udp_port)
        self._connection_factory = (
            mavutil.mavlink_connection
            if connection_factory is None
            else connection_factory
        )
        self._clock = clock
        self.sim_link: Any | None = None
        self.command_link: Any | None = None
        self.command_target_system = int(system_id)
        self.simulator_system_id = int(simulator_system_id)
        self.command_system_id = int(command_system_id)
        if not all(1 <= value <= 255 for value in (system_id, simulator_system_id, command_system_id)):
            raise ValueError("MAVLink system IDs must be in 1..255")
        self.command_target_component = 1
        self.local_origin_world: FloatArray | None = None
        self.last_rotor_thrusts = np.zeros(4, dtype=np.float64)
        self.last_control_received_wall_time = 0.0
        self.last_heartbeat_wall_time = 0.0
        self.last_command_heartbeat_wall_time = 0.0
        self.last_px4_heartbeat_wall_time = 0.0
        self.last_offboard_request_wall_time = 0.0
        self.last_arm_request_wall_time: float | None = None
        self.request_interval_s = 0.25
        self.armed = False
        self.offboard = False
        self.shutdown_request_sent = False
        self.last_command_ack = "none"
        self.arm_ack_sequence = 0
        self.last_arm_ack_result: int | None = None
        self.last_statustext = "none"
        self._barometer_rng = np.random.default_rng(BAROMETER_RANDOM_SEED + self.command_target_system)
        self._statustext_chunks: dict[
            tuple[int, int, int], tuple[int, bytes, float]
        ] = {}

    @property
    def connected(self) -> bool:
        """Reports whether both MAVLink connection objects exist."""
        return self.sim_link is not None and self.command_link is not None

    def connect(self) -> None:
        """Opens simulator and Offboard MAVLink connections."""
        self.sim_link = self._connection_factory(
            f"tcpin:127.0.0.1:{self.sim_tcp_port}",
            source_system=self.simulator_system_id,
            source_component=1,
            autoreconnect=True,
        )
        self.command_link = self._connection_factory(
            f"udpout:127.0.0.1:{self.command_udp_port}",
            source_system=self.command_system_id,
            source_component=190,
        )
        # MAVLink 1 drops STATUSTEXT.id/chunk_seq extension fields. Use a
        # per-link MAVLink 2 codec without changing mavutil's global dialect.
        self.command_link.mav = mavlink2.MAVLink(
            self.command_link, srcSystem=self.command_system_id, srcComponent=190
        )
        self.command_link.mav.robust_parsing = True
        self.command_link.WIRE_PROTOCOL_VERSION = "2.0"
        self.command_link.first_byte = False

    def close(self) -> None:
        """Closes both MAVLink connections when supported."""
        for link in (self.sim_link, self.command_link):
            if link is not None and hasattr(link, "close"):
                link.close()
        self.sim_link = None
        self.command_link = None

    @classmethod
    def mujoco_world_to_px4_local(
        cls,
        vector_world: Sequence[float],
    ) -> FloatArray:
        """Converts a MuJoCo ENU-like world vector to PX4 NED."""
        return _validated_vector3(vector_world, "vector_world") * cls.MUJOCO_TO_PX4_AXIS

    @classmethod
    def mujoco_body_to_px4_body(
        cls,
        vector_body: Sequence[float],
    ) -> FloatArray:
        """Converts a MuJoCo FLU body vector to PX4 FRD."""
        return _validated_vector3(vector_body, "vector_body") * cls.MUJOCO_TO_PX4_AXIS

    def set_local_origin(self, position_world: Sequence[float]) -> None:
        """Sets the MuJoCo world position corresponding to PX4 local zero."""
        self.local_origin_world = _validated_vector3(
            position_world,
            "position_world",
        )

    def world_position_to_px4_local(
        self,
        position_world: Sequence[float],
    ) -> FloatArray:
        """Converts an absolute MuJoCo position to PX4 local NED."""
        position = _validated_vector3(position_world, "position_world")
        if self.local_origin_world is None:
            self.set_local_origin(position)
        assert self.local_origin_world is not None
        return self.mujoco_world_to_px4_local(position - self.local_origin_world)

    @staticmethod
    def _quat_wxyz_to_matrix(quaternion_wxyz: Sequence[float]) -> FloatArray:
        """Converts a normalized WXYZ quaternion to a rotation matrix."""
        quaternion = np.array(quaternion_wxyz, dtype=np.float64, copy=True)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError("quaternion_wxyz must be finite with shape (4,)")
        quaternion /= max(float(np.linalg.norm(quaternion)), np.finfo(float).eps)
        w_value, x_value, y_value, z_value = quaternion
        return np.array(
            [
                [
                    1.0 - 2.0 * (y_value**2 + z_value**2),
                    2.0 * (x_value * y_value - z_value * w_value),
                    2.0 * (x_value * z_value + y_value * w_value),
                ],
                [
                    2.0 * (x_value * y_value + z_value * w_value),
                    1.0 - 2.0 * (x_value**2 + z_value**2),
                    2.0 * (y_value * z_value - x_value * w_value),
                ],
                [
                    2.0 * (x_value * z_value - y_value * w_value),
                    2.0 * (y_value * z_value + x_value * w_value),
                    1.0 - 2.0 * (x_value**2 + y_value**2),
                ],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _matrix_to_quat_wxyz(rotation_matrix: FloatArray) -> FloatArray:
        """Converts a three-dimensional rotation matrix to WXYZ quaternion."""
        matrix = np.asarray(rotation_matrix, dtype=np.float64)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("rotation_matrix must be finite with shape (3, 3)")
        quaternion = np.empty(4, dtype=np.float64)
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = 2.0 * np.sqrt(trace + 1.0)
            quaternion[:] = [
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ]
        else:
            axis = int(np.argmax(np.diag(matrix)))
            next_axis = (axis + 1) % 3
            last_axis = (axis + 2) % 3
            scale = 2.0 * np.sqrt(
                max(
                    0.0,
                    1.0
                    + matrix[axis, axis]
                    - matrix[next_axis, next_axis]
                    - matrix[last_axis, last_axis],
                )
            )
            scale = max(scale, np.finfo(float).eps)
            quaternion[axis + 1] = 0.25 * scale
            quaternion[0] = (
                matrix[last_axis, next_axis] - matrix[next_axis, last_axis]
            ) / scale
            quaternion[next_axis + 1] = (
                matrix[axis, next_axis] + matrix[next_axis, axis]
            ) / scale
            quaternion[last_axis + 1] = (
                matrix[axis, last_axis] + matrix[last_axis, axis]
            ) / scale
        return quaternion / max(
            float(np.linalg.norm(quaternion)),
            np.finfo(float).eps,
        )

    @classmethod
    def mujoco_quat_to_px4_quat(
        cls,
        quaternion_wxyz: Sequence[float],
    ) -> FloatArray:
        """Converts MuJoCo world/body attitude to PX4 NED/FRD attitude."""
        axis_flip = np.diag(cls.MUJOCO_TO_PX4_AXIS)
        rotation_mujoco = cls._quat_wxyz_to_matrix(quaternion_wxyz)
        return cls._matrix_to_quat_wxyz(
            axis_flip @ rotation_mujoco @ axis_flip
        )

    def request_arm(self, force: bool = False) -> None:
        """Sends a rate-limited PX4 Arm command and no mode command."""
        if self.command_link is None:
            return
        wall_time = self._clock()
        if (
            not force
            and self.last_arm_request_wall_time is not None
            and wall_time - self.last_arm_request_wall_time
            < self.request_interval_s
        ):
            return
        self.last_arm_request_wall_time = wall_time
        self.command_link.mav.command_long_send(
            self.command_target_system,
            self.command_target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0,
            21196.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def request_disarm(self) -> None:
        """Sends only a PX4 Disarm command."""
        if self.command_link is None:
            return
        self.command_link.mav.command_long_send(
            self.command_target_system,
            self.command_target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0.0,
            21196.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def request_shutdown(self) -> None:
        """Sends one PX4 autopilot shutdown command when connected."""
        if self.command_link is None or self.shutdown_request_sent:
            return
        self.command_link.mav.command_long_send(
            self.command_target_system,
            self.command_target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self.shutdown_request_sent = True

    def request_offboard(self, force: bool = False) -> None:
        """Sends only a PX4 Offboard mode request."""
        if self.command_link is None:
            return
        wall_time = self._clock()
        if (
            not force
            and self.last_offboard_request_wall_time > 0.0
            and wall_time - self.last_offboard_request_wall_time
            < self.request_interval_s
        ):
            return
        self.last_offboard_request_wall_time = wall_time
        self.command_link.mav.command_long_send(
            self.command_target_system,
            self.command_target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            29.0,
            float(self.PX4_OFFBOARD_MAIN_MODE),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def send_simulator_heartbeat(self) -> None:
        """Sends the one-hertz simulator heartbeat."""
        if self.sim_link is None:
            return
        wall_time = self._clock()
        if wall_time - self.last_heartbeat_wall_time < 1.0:
            return
        self.last_heartbeat_wall_time = wall_time
        self.sim_link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def send_command_heartbeat(self) -> None:
        """Identifies the interactive control panel as a GCS at one hertz.

        PX4's streams/STATUSTEXT.hpp gates text on is_gcs_connected().
        An onboard-controller heartbeat alone therefore hides arm failures.
        """
        if self.command_link is None:
            return
        wall_time = self._clock()
        if wall_time - self.last_command_heartbeat_wall_time < 1.0:
            return
        self.last_command_heartbeat_wall_time = wall_time
        self.command_link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def send_state(
        self,
        position_world: Sequence[float],
        velocity_world: Sequence[float],
        acceleration_world: Sequence[float],
        quaternion_wxyz: Sequence[float],
        angular_rate_body: Sequence[float],
        specific_force_body: Sequence[float],
    ) -> None:
        """Sends sensors and paired float position/HIL state to PX4 SITL.

        The custom simulator consumes LOCAL_POSITION_NED immediately before
        HIL_STATE_QUATERNION when PX4_SIM_FLOAT_STATE=1. Both packets must
        share the sender, timestamp and setpoint origin. Legacy mode ignores
        the additional local-position packet.
        """
        if self.sim_link is None:
            return
        self.send_simulator_heartbeat()
        now_us = int(self._clock() * 1_000_000)
        position = _validated_vector3(position_world, "position_world")
        position_px4 = self.world_position_to_px4_local(position)
        velocity_px4 = self.mujoco_world_to_px4_local(velocity_world)
        acceleration_px4 = self.mujoco_world_to_px4_local(acceleration_world)
        quaternion_px4 = self.mujoco_quat_to_px4_quat(quaternion_wxyz)
        angular_rate_px4 = self.mujoco_body_to_px4_body(angular_rate_body)
        specific_force_px4 = self.mujoco_body_to_px4_body(specific_force_body)
        latitude = self.HOME_LAT_DEG + np.degrees(
            position_px4[0] / self.EARTH_RADIUS_M
        )
        longitude = self.HOME_LON_DEG + np.degrees(
            position_px4[1]
            / (
                self.EARTH_RADIUS_M
                * np.cos(np.radians(self.HOME_LAT_DEG))
            )
        )
        altitude = self.HOME_ALT_M - position_px4[2]
        # A constant, noiseless pressure trips DataValidator's STALE flag
        # despite fresh timestamps. Simulate a seeded measurement instead.
        pressure_hpa = standard_atmosphere_pressure_hpa(float(altitude))
        pressure_hpa += float(
            self._barometer_rng.normal(0.0, BAROMETER_NOISE_STD_HPA)
        )
        self.sim_link.mav.hil_sensor_send(
            now_us,
            *specific_force_px4.tolist(),
            *angular_rate_px4.tolist(),
            0.2,
            0.0,
            0.45,
            pressure_hpa,
            0.0,
            float(altitude),
            15.0 - 0.0065 * float(altitude),
            0x1FFF,
            0,
        )
        self.sim_link.mav.local_position_ned_send(
            (now_us // 1000) & 0xFFFFFFFF,
            *position_px4.tolist(),
            *velocity_px4.tolist(),
        )
        # HIL_STATE_QUATERNION has signed int16 velocity/acceleration fields.
        # Ground-contact impulses can exceed their range. Saturate only this
        # integer representation; float LOCAL_POSITION_NED and HIL_SENSOR
        # above retain their full values. Keep the existing receiver's /1000
        # acceleration convention (SimulatorMavlink.cpp), not a unit redesign.
        velocity_integer = np.clip(
            velocity_px4 * 100, -32768, 32767,
        ).astype(np.int16)
        acceleration_integer = np.clip(
            acceleration_px4 * 1000, -32768, 32767,
        ).astype(np.int16)
        self.sim_link.mav.hil_state_quaternion_send(
            now_us,
            quaternion_px4.tolist(),
            *angular_rate_px4.tolist(),
            int(latitude * 1e7),
            int(longitude * 1e7),
            int(altitude * 1000),
            *velocity_integer.tolist(),
            0,
            0,
            *acceleration_integer.tolist(),
        )

    def send_position_target(
        self,
        target_world: Sequence[float],
        target_yaw_rad: float,
    ) -> None:
        """Sends a position-only PX4 local-NED Offboard setpoint."""
        if self.command_link is None:
            return
        target_px4 = self.world_position_to_px4_local(target_world)
        self.command_link.mav.set_position_target_local_ned_send(
            int(self._clock() * 1000) & 0xFFFFFFFF,
            self.command_target_system,
            self.command_target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            self.POSITION_ONLY_TARGET_TYPE_MASK,
            *target_px4.tolist(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(target_yaw_rad),
            0.0,
        )

    def _receive_statustext(self, message: Any) -> None:
        """Reassembles ordered text chunks, retaining PX4's visible prefix.

        Args:
            message: A decoded MAVLink STATUSTEXT message. Chunk keys include
                sender identity and message ID; gaps and expired tails are
                discarded. PX4 starts IDs at zero, unlike the spec's suggested
                use of zero for standalone texts.
        """
        raw_text = getattr(message, "_text_raw", message.text)
        if isinstance(raw_text, str):
            raw_text = raw_text.encode("utf-8")
        chunk = raw_text.split(b"\x00", 1)[0]
        sequence = int(getattr(message, "chunk_seq", 0))
        key = (
            int(message.get_srcSystem()),
            int(message.get_srcComponent()),
            int(getattr(message, "id", 0)),
        )
        now = self._clock()
        self._statustext_chunks = {
            key_: value for key_, value in self._statustext_chunks.items()
            if now - value[2] <= 2.0
        }
        if sequence == 0:
            combined = chunk
        else:
            previous = self._statustext_chunks.pop(key, None)
            if previous is None or previous[0] + 1 != sequence:
                return
            combined = previous[1] + chunk
        if len(chunk) == 50:
            self._statustext_chunks[key] = (sequence, combined, now)
        else:
            self._statustext_chunks.pop(key, None)
        # PX4 sends exactly-50-byte messages without an extra terminating
        # chunk. Show the cumulative prefix so these remain visible too.
        self.last_statustext = combined.decode("utf-8", errors="replace").strip()
        if self.last_statustext:
            severity = int(getattr(message, "severity", -1))
            print(
                f"PX4 STATUSTEXT[{severity}]: {self.last_statustext}",
                flush=True,
            )

    def poll_status(self) -> None:
        """Polls PX4 heartbeat, command acknowledgement, and status text."""
        if self.command_link is None:
            return
        while True:
            message = self.command_link.recv_match(
                type=["COMMAND_ACK", "STATUSTEXT", "HEARTBEAT"],
                blocking=False,
            )
            if message is None:
                return
            if (message.get_srcSystem() != self.command_target_system
                    or message.get_srcComponent() != self.command_target_component):
                continue
            message_type = message.get_type()
            if message_type == "HEARTBEAT":
                base_mode = int(getattr(message, "base_mode", 0))
                custom_mode = int(getattr(message, "custom_mode", 0))
                main_mode = (custom_mode >> 16) & 0xFF
                if main_mode == 0:
                    main_mode = custom_mode
                self.armed = bool(
                    base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self.offboard = main_mode == self.PX4_OFFBOARD_MAIN_MODE
                self.last_px4_heartbeat_wall_time = self._clock()
            elif message_type == "COMMAND_ACK":
                command = int(getattr(message, "command", -1))
                result = int(getattr(message, "result", -1))
                self.last_command_ack = (
                    f"{command}:{result}"
                )
                if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.arm_ack_sequence += 1
                    self.last_arm_ack_result = result
            elif message_type == "STATUSTEXT":
                self._receive_statustext(message)

    def receive_rotor_thrusts(self) -> FloatArray:
        """Returns the most recent four PX4 rotor thrust commands."""
        if self.sim_link is None:
            return self.last_rotor_thrusts.copy()
        while True:
            message = self.sim_link.recv_match(
                type="HIL_ACTUATOR_CONTROLS",
                blocking=False,
            )
            if message is None:
                return self.last_rotor_thrusts.copy()
            if (message.get_srcSystem() != self.command_target_system
                    or message.get_srcComponent() != self.command_target_component):
                continue
            controls = np.asarray(message.controls, dtype=np.float64)
            if controls.size < 4 or not np.all(np.isfinite(controls[:4])):
                continue
            self.last_rotor_thrusts = motor_outputs_to_thrusts(
                controls[:4],
                self.rotor_thrust_scales,
            )
            self.last_control_received_wall_time = self._clock()

    def control_is_stale(self, max_age_s: float) -> bool:
        """Reports whether an established actuator stream exceeded its age limit."""
        if max_age_s <= 0.0 or not np.isfinite(max_age_s):
            raise ValueError("max_age_s must be finite and positive")
        if self.last_control_received_wall_time <= 0.0:
            return False
        return (
            self._clock() - self.last_control_received_wall_time
            > max_age_s
        )

    def heartbeat_is_recent(self, max_age_s: float = 2.0) -> bool:
        """Reports whether PX4 recently supplied a command-link heartbeat."""
        if self.last_px4_heartbeat_wall_time <= 0.0:
            return False
        return self._clock() - self.last_px4_heartbeat_wall_time <= max_age_s
