"""Tension-only Kelvin–Voigt ropes, applied at the actual attachment sites.

The unilateral clipped spring/damper law follows the local single-UAV
uav_control_with_px4.py reference; coefficients are simulation assumptions.
Spatial tendons are drawing only; all rope loads use mj_applyFT in world axes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np
import numpy.typing as npt

from .fleet import AIRCRAFT_LABELS, validate_fleet

FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True)
class ElasticRopeConfig:
    """Massless straight rope parameters in SI units."""

    rest_length_m: float = 1.2
    stiffness_n_m: float = 100.0
    damping_ns_m: float = 3.0

    def __post_init__(self) -> None:
        values = (self.rest_length_m, self.stiffness_n_m, self.damping_ns_m)
        if not np.isfinite(values).all() or min(values[:2]) <= 0 or values[2] < 0:
            raise ValueError("rope length/stiffness must be positive; damping nonnegative")


@dataclass(frozen=True)
class PayloadConfig:
    """Shared box geometry and equal elastic ropes."""

    mass_kg: float = .5
    rope: ElasticRopeConfig = field(default_factory=ElasticRopeConfig)
    size_m: tuple[float, float, float] = (.4, .3, .15)

    def __post_init__(self) -> None:
        if not np.isfinite(self.mass_kg) or self.mass_kg <= 0:
            raise ValueError("payload mass must be finite and positive")
        if self.size_m != (.4, .3, .15):
            raise ValueError("this configuration uses a .40 x .30 x .15 m box")


@dataclass(frozen=True)
class RopeState:
    """One instantaneous tension and force at the anchor."""

    length_m: float
    extension_m: float
    extension_rate_m_s: float
    tension_n: float
    force_on_anchor_n: FloatArray


def compute_rope_force(
    anchor_position: FloatArray,
    payload_position: FloatArray,
    anchor_velocity: FloatArray,
    payload_velocity: FloatArray,
    config: ElasticRopeConfig,
) -> RopeState:
    """Compute unilateral elastic/damped tension, never compression.

    Args:
        anchor_position: World gripper position, metres.
        payload_position: World box attachment position, metres.
        anchor_velocity: World gripper site velocity, m/s.
        payload_velocity: World box site velocity, m/s.
        config: Spring rest length, stiffness and axial damping.

    Returns:
        Geometric diagnostics and world force on the gripper; the box gets
        its negative at its own site. Slack ropes have zero damping force.
    """
    vectors = np.asarray([anchor_position, payload_position,
                          anchor_velocity, payload_velocity], dtype=float)
    if vectors.shape != (4, 3) or not np.isfinite(vectors).all():
        raise ValueError("rope positions and velocities must be finite 3-vectors")
    delta = vectors[1] - vectors[0]
    length = float(np.linalg.norm(delta))
    direction = delta / length if length > 1e-12 else np.zeros(3)
    extension = max(0., length - config.rest_length_m)
    rate = float((vectors[3] - vectors[2]) @ direction)
    tension = 0.
    if extension > 0:
        tension = max(0., config.stiffness_n_m * extension
                      + config.damping_ns_m * rate)
    if not np.isfinite([length, extension, rate, tension]).all():
        raise FloatingPointError("non-finite rope geometry or force; reduce rope parameters")
    return RopeState(length, extension, rate, tension, tension * direction)


class PayloadSystem:
    """Resolve sites and write only the rope-owned generalized-force channel.

    Args:
        model: Compiled shared world with the payload and all gripper sites.
        config: Payload and rope settings used to build that world.
    """

    def __init__(self, model: mujoco.MjModel, config: PayloadConfig) -> None:
        self.model = model
        self.config = config
        self.body_id = model.body("payload").id
        self.geom_id = model.geom("payload_box").id
        self.floor_id = model.geom("floor").id
        labels = [label for label in AIRCRAFT_LABELS
                  if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                      label + "/endpoint") >= 0]
        validate_fleet(len(labels))
        self.anchor_sites = [model.site(label + "/endpoint").id for label in labels]
        self.payload_sites = [model.site("payload_" + label).id for label in labels]
        self.root_ids = [model.body(label + "/quadrotor_950_pos").id for label in labels]
        self.jacobian = np.zeros((3, model.nv))
        self.forces = np.zeros(model.nv)

    def _velocity(self, data: mujoco.MjData, site: int) -> FloatArray:
        """Read world site velocity including all upstream joint motion."""
        mujoco.mj_jacSite(self.model, data, self.jacobian, None, site)
        return self.jacobian @ data.qvel

    def apply(self, data: mujoco.MjData) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        """Accumulate each rope's equal/opposite point loads and same-time diagnostics.

        Args:
            data: Forwarded state. Replaces qfrc_applied, the rope-owned channel.

        Returns:
            Box pose/contact/clearance and all ropes' forces and COM moments.
        """
        self.forces.fill(0)
        ropes = []
        for anchor, end, root in zip(self.anchor_sites, self.payload_sites, self.root_ids):
            state = compute_rope_force(
                data.site_xpos[anchor], data.site_xpos[end],
                self._velocity(data, anchor), self._velocity(data, end),
                self.config.rope,
            )
            for site, sign in ((anchor, 1), (end, -1)):
                mujoco.mj_applyFT(self.model, data, sign * state.force_on_anchor_n,
                                 np.zeros(3), data.site_xpos[site],
                                 int(self.model.site_bodyid[site]), self.forces)
            ropes.append({
                "length_m": state.length_m, "extension_m": state.extension_m,
                "extension_rate_m_s": state.extension_rate_m_s,
                "tension_n": state.tension_n,
                "force_on_anchor_n": state.force_on_anchor_n.tolist(),
                "anchor_position": data.site_xpos[anchor].tolist(),
                "payload_attachment_position": data.site_xpos[end].tolist(),
                "moment_about_aircraft_com_nm": np.cross(
                    data.site_xpos[anchor] - data.subtree_com[root],
                    state.force_on_anchor_n,
                ).tolist(),
            })
        if not np.isfinite(self.forces).all():
            raise FloatingPointError("non-finite accumulated rope generalized force")
        data.qfrc_applied[:] = self.forces
        half_size = np.array(self.config.size_m) / 2
        rotation = data.xmat[self.body_id].reshape(3, 3)
        bottom = float(data.xpos[self.body_id, 2] - np.abs(rotation[2]) @ half_size)
        contact = any(
            c.dist <= 0 and {c.geom1, c.geom2} == {self.floor_id, self.geom_id}
            for c in data.contact
        )
        joint = int(self.model.body_jntadr[self.body_id])
        dof = int(self.model.jnt_dofadr[joint])
        qpos = int(self.model.jnt_qposadr[joint])
        payload = {"position": data.xpos[self.body_id].tolist(),
                   "quaternion_wxyz": data.qpos[qpos + 3:qpos + 7].tolist(),
                   "velocity": data.qvel[dof:dof + 3].tolist(),
                   "angular_velocity": data.qvel[dof + 3:dof + 6].tolist(),
                   "bottom_clearance_m": bottom, "floor_contact": bool(contact)}
        return payload, ropes
