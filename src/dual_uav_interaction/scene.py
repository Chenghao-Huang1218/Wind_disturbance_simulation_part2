"""Build and bind two complete aircraft in one independent MuJoCo world."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np
import numpy.typing as npt

from .payload import PayloadConfig

ASSETS = Path(__file__).resolve().parent / "assets"
FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True)
class AircraftBindings:
    """Namespaced model IDs and joint addresses for a complete assembly."""

    prefix: str
    root_id: int
    qpos: int
    dof: int
    thrust_ids: npt.NDArray[np.int64]
    spin_ids: npt.NDArray[np.int64]
    arm_ids: npt.NDArray[np.int64]
    arm_qpos: npt.NDArray[np.int64]
    gyro_id: int
    accelerometer_id: int
    body_ids: npt.NDArray[np.int64]
    mass_kg: float

    @classmethod
    def resolve(cls, model: mujoco.MjModel, prefix: str) -> AircraftBindings:
        """Resolve the complete vehicle namespace and its subtree mass.

        Args:
            model: Compiled shared-world model.
            prefix: a/ or b/ namespace.

        Returns:
            Independent bindings for exactly one aircraft assembly.
        """
        root = model.body(prefix + "quadrotor_950_pos").id
        joint = model.joint(prefix + "main_joint").id
        descendants = []
        for body_id in range(1, model.nbody):
            ancestor = body_id
            while ancestor and ancestor != root:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor == root:
                descendants.append(body_id)
        return cls(
            prefix,
            root,
            int(model.jnt_qposadr[joint]),
            int(model.jnt_dofadr[joint]),
            np.array(
                [model.actuator(prefix + f"rotor{i}").id for i in range(4)]
            ),
            np.array(
                [
                    model.actuator(prefix + f"rotor_{name}_spin").id
                    for name in ("a00", "a10", "a20", "a30")
                ]
            ),
            np.array(
                [model.actuator(prefix + f"joint{i}").id for i in range(1, 7)]
            ),
            np.array(
                [
                    model.jnt_qposadr[model.joint(prefix + f"joint{i}").id]
                    for i in range(1, 7)
                ]
            ),
            model.sensor(prefix + "body_gyro").id,
            model.sensor(prefix + "body_linacc").id,
            np.array(descendants),
            float(model.body_subtreemass[root]),
        )


def load_scene(
    separation_m: float = 2.0,
    payload_config: PayloadConfig | None = None,
) -> tuple[
    mujoco.MjModel, mujoco.MjData, tuple[AircraftBindings, AircraftBindings]
]:
    """Attach two fully prefixed copies of the packaged X4 + Airbot scene.

    Args:
        separation_m: Initial horizontal center separation, at least 1.5 m.
        payload_config: Optional shared box and elastic-rope configuration.

    Returns:
        Shared model, state, and isolated bindings for A and B.
    """
    if not np.isfinite(separation_m) or separation_m < 1.5:
        raise ValueError(
            "initial separation must be finite and >=1.5 m (scene clearance setting)"
        )
    if payload_config is not None and separation_m > 2.25:
        raise ValueError("loaded separation must be in 1.5..2.25 m")
    ground_scene = ASSETS / "robot_models/scenes/ground.xml"
    world = mujoco.MjSpec.from_string(
        f"""<mujoco model="dual_x4">
      <option timestep="0.005" gravity="0 0 -9.80665" density="0" viscosity="0"/>
      <include file="{ground_scene}"/>
    </mujoco>"""
    )
    scene = ASSETS / "scenes/default/robot_quadrotor_950_with_arm.xml"
    for prefix, horizontal, color in (
        ("a/", -separation_m / 2, [0.1, 0.5, 1, 1]),
        ("b/", separation_m / 2, [1, 0.45, 0.1, 1]),
    ):
        child = mujoco.MjSpec.from_file(str(scene))
        child.delete(child.geom("floor"))
        child.body("quadrotor_950_pos").add_geom(
            name="identity_marker",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            pos=[0, 0, 0.2],
            size=[0.06, 0, 0],
            rgba=color,
            contype=0,
            conaffinity=0,
            mass=0,
        )
        child.body("quadrotor_950_pos").add_geom(
            name="heading_marker", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[.20, 0, .12, .43, 0, .12], size=[.025, 0, 0],
            rgba=[1, .85, .15, 1], contype=0, conaffinity=0, mass=0,
        )
        frame = world.worldbody.add_frame(pos=[0, horizontal, 0])
        world.attach(child, prefix=prefix, frame=frame)
    model = world.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if payload_config is not None:
        grippers = np.array([data.site(p + "endpoint").xpos for p in ("a/", "b/")])
        center = grippers.mean(axis=0)
        center[2] = payload_config.size_m[2] / 2
        body = world.worldbody.add_body(name="payload", pos=center)
        body.add_freejoint(name="payload_free")
        body.add_geom(name="payload_box", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=np.array(payload_config.size_m) / 2,
                      mass=payload_config.mass_kg, rgba=[.9, .65, .15, 1],
                      friction=[.8, .01, .001])
        for index, (label, side) in enumerate((("a", -1), ("b", 1))):
            local = np.array([0., side * .12, .075])
            initial_length = np.linalg.norm(center + local - grippers[index])
            if initial_length > payload_config.rope.rest_length_m:
                raise ValueError(
                    "initial ropes must be slack; increase rest length "
                    "or reduce separation"
                )
            body.add_site(name="payload_" + label, pos=local, size=[.016, 0, 0], rgba=[1, 1, 1, 1])
            tendon = world.add_tendon(name="rope_" + label, stiffness=0, damping=0,
                                      limited=False, width=.006, rgba=[.95, .9, .65, 1])
            tendon.wrap_site(label + "/endpoint")
            tendon.wrap_site("payload_" + label)
        model = world.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
    bindings = tuple(
        AircraftBindings.resolve(model, prefix) for prefix in ("a/", "b/")
    )
    return model, data, bindings


def apply_accelerations(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    bindings: Sequence[AircraftBindings],
    accelerations: FloatArray,
) -> None:
    """Distribute each acceleration by live body mass, with zero net COM moment.

    Args:
        model: Shared physics model.
        data: Current synchronized state; writes external loads only.
        bindings: Aircraft subtree definitions.
        accelerations: World acceleration per aircraft, shape (2, 3).
    """
    values = np.asarray(accelerations, dtype=np.float64)
    if values.shape != (len(bindings), 3) or not np.isfinite(values).all():
        raise ValueError(
            "accelerations must be finite with one vector per aircraft"
        )
    for item, acceleration in zip(bindings, values):
        forces = model.body_mass[item.body_ids, None] * acceleration
        if not np.isfinite(forces).all():
            raise ValueError("interaction force overflow")
        data.xfrc_applied[item.body_ids, :3] = forces
        data.xfrc_applied[item.body_ids, 3:] = 0
