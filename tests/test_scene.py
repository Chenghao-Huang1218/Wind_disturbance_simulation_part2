"""Real MuJoCo checks for two isolated, complete X4 assemblies."""

import mujoco
import numpy as np

from dual_uav_interaction.scene import apply_accelerations, load_scene


def test_two_complete_namespaces_and_isolated_actuators() -> None:
    model, data, bindings = load_scene(2.0)
    assert model.nq > 30
    assert len(bindings) == 2
    np.testing.assert_allclose([b.mass_kg for b in bindings], 8.0004)
    assert set(bindings[0].thrust_ids).isdisjoint(bindings[1].thrust_ids)
    for item in bindings:
        assert len(item.arm_ids) == 6
        assert len(item.body_ids) > 10
        assert model.sensor_dim[item.gyro_id] == 3
    data.ctrl[bindings[0].thrust_ids] = 10
    np.testing.assert_array_equal(data.ctrl[bindings[1].thrust_ids], 0)
    np.testing.assert_allclose(
        data.xpos[bindings[1].root_id] - data.xpos[bindings[0].root_id],
        [0, 2, 0],
    )


def test_subtree_force_has_no_moment_at_live_assembly_com() -> None:
    model, data, bindings = load_scene(2.0)
    # Exercise nontrivial live arm geometry, not only the frozen initial COM.
    data.qpos[bindings[0].arm_qpos[1]] = -0.6
    mujoco.mj_forward(model, data)
    acceleration = np.array([[-0.3, 0.2, -0.4], [0.1, -0.2, -0.5]])
    apply_accelerations(model, data, bindings, acceleration)
    for item, target in zip(bindings, acceleration):
        forces = data.xfrc_applied[item.body_ids, :3]
        offsets = data.xipos[item.body_ids] - data.subtree_com[item.root_id]
        np.testing.assert_allclose(forces.sum(axis=0), item.mass_kg * target)
        np.testing.assert_allclose(
            np.cross(offsets, forces).sum(axis=0), 0, atol=1e-12
        )
        np.testing.assert_array_equal(data.xfrc_applied[item.body_ids, 3:], 0)
    apply_accelerations(model, data, bindings, np.zeros((2, 3)))
    np.testing.assert_array_equal(data.xfrc_applied, 0)
