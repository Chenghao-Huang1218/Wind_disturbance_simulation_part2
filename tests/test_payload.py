"""Elastic ropes act at live gripper sites, with no compression or double load."""

import mujoco
import numpy as np
import pytest

from dual_uav_interaction.payload import (
    ElasticRopeConfig, PayloadConfig, PayloadSystem, compute_rope_force,
)
from dual_uav_interaction.scene import load_scene


def test_tension_only_elastic_damped_law() -> None:
    config = ElasticRopeConfig()
    zero = np.zeros(3)
    assert compute_rope_force(zero, [0, 0, 1], zero, zero, config).tension_n == 0
    state = compute_rope_force(zero, [0, 0, 1.3], zero, [0, 0, 2], config)
    assert state.tension_n == pytest.approx(16)
    np.testing.assert_allclose(state.force_on_anchor_n, [0, 0, 16])
    assert compute_rope_force(zero, [0, 0, 1.3], zero, [0, 0, -20], config).tension_n == 0
    assert compute_rope_force(zero, zero, zero, zero, config).tension_n == 0


def test_overflow_rejected_before_applying_nonfinite_force() -> None:
    zero = np.zeros(3)
    with pytest.raises(FloatingPointError, match='rope'):
        compute_rope_force(zero, [3., 0., 0.], zero, zero,
                           ElasticRopeConfig(stiffness_n_m=1e308))


def test_payload_scene_and_point_force_jacobian() -> None:
    model, data, bindings = load_scene(payload_config=PayloadConfig())
    system = PayloadSystem(model, PayloadConfig())
    assert model.body('payload').mass[0] == pytest.approx(.5)
    np.testing.assert_allclose([b.mass_kg for b in bindings], 8.0004)
    np.testing.assert_allclose(data.xpos[bindings[1].root_id] - data.xpos[bindings[0].root_id], [0, 2, 0])
    payload, ropes = system.apply(data)
    assert all(row['tension_n'] == 0 for row in ropes)
    for binding in bindings:
        data.qpos[binding.qpos + 2] += 2
    mujoco.mj_forward(model, data)
    payload, ropes = system.apply(data)
    expected = np.zeros(model.nv)
    for anchor, end, row in zip(system.anchor_sites, system.payload_sites, ropes):
        force = np.array(row['force_on_anchor_n'])
        assert row['tension_n'] > 0
        for site, sign in ((anchor, 1), (end, -1)):
            jac = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jac, None, site)
            expected += jac.T @ (sign * force)
    np.testing.assert_allclose(data.qfrc_applied, expected, atol=1e-10)
    assert np.linalg.norm(expected[bindings[0].dof + 6:bindings[0].dof + 12]) > 0
    assert np.all(model.tendon_stiffness == 0)
    assert np.all(model.tendon_damping == 0)
    data.qpos[:] = model.qpos0
    mujoco.mj_forward(model, data)
    system.apply(data)
    np.testing.assert_allclose(data.qfrc_applied, 0)


def test_loaded_geometry_and_configuration_rejection() -> None:
    for value in (float('nan'), 0, -1):
        with pytest.raises(ValueError):
            PayloadConfig(mass_kg=value)
    with pytest.raises(ValueError):
        load_scene(2.5, PayloadConfig())
    with pytest.raises(ValueError, match='slack'):
        load_scene(2, PayloadConfig(rope=ElasticRopeConfig(rest_length_m=.1)))


def test_simulation_loaded_limit_and_snapshot_force_clearing() -> None:
    from test_simulation import Bridge
    from dual_uav_interaction.simulation import DualSimulation

    app = DualSimulation(bridges=[Bridge(), Bridge()], payload_config=PayloadConfig())
    with pytest.raises(ValueError):
        app.command('separation', .5)
    app.data.qfrc_applied[:] = 123
    sample = app.step()
    assert sample['payload'] is not None
    assert len(sample['ropes']) == 2
    np.testing.assert_array_equal(app.data.qfrc_applied, 0)


def test_loaded_setpoint_ramp_preserves_final_goal() -> None:
    from test_simulation import Bridge
    from dual_uav_interaction.simulation import DualSimulation

    app = DualSimulation(bridges=[Bridge(), Bridge()], payload_config=PayloadConfig())
    app.command('takeoff')
    before = app.aircraft[0].streamed_target.copy()
    sample = app.step()
    delta = app.aircraft[0].streamed_target - before
    assert np.linalg.norm(delta) == pytest.approx(.1 * app.model.opt.timestep ** 2)
    assert np.linalg.norm(app.aircraft[0].streamed_velocity) <= .25
    assert sample['aircraft'][0]['target'][2] == pytest.approx(2.48)
    assert sample['aircraft'][0]['position_setpoint'][2] < 1.


def test_fixed_grippers_support_payload_weight_in_equilibrium() -> None:
    model, data, bindings = load_scene(payload_config=PayloadConfig())
    system = PayloadSystem(model, PayloadConfig())
    fixed = data.qpos.copy()
    for binding in bindings:
        fixed[binding.qpos + 2] += 2
    for _ in range(4000):
        for binding in bindings:
            data.qpos[binding.qpos:binding.qpos + 7] = fixed[binding.qpos:binding.qpos + 7]
            data.qpos[binding.arm_qpos] = fixed[binding.arm_qpos]
            data.qvel[binding.dof:binding.dof + 6] = 0
            for index in range(1, 7):
                data.qvel[model.joint(binding.prefix + f'joint{index}').dofadr[0]] = 0
        data.qfrc_applied[:] = 0
        mujoco.mj_forward(model, data)
        payload, ropes = system.apply(data)
        mujoco.mj_step(model, data)
    assert payload['bottom_clearance_m'] > 1.
    upward_force = -sum(row['force_on_anchor_n'][2] for row in ropes)
    assert upward_force == pytest.approx(.5 * 9.80665, abs=1e-7)
