"""Four-aircraft behavior: real geometry/physics and isolated controller state."""
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dual_uav_interaction.interaction import InteractionConfig, compute_pair_accelerations
from dual_uav_interaction.processes import SitlPair
from test_processes import Child, fake_runtime


SQUARE = np.array([[-1., -1., 2.48], [-1., 1., 2.48],
                   [1., -1., 2.48], [1., 1., 2.48]])


def test_square_sums_three_disturbance_sources_without_self_load() -> None:
    """Pin analytic adjacent/diagonal gains, source gating and permutation."""
    config = InteractionConfig('both', .5, .5, 2.)
    result = compute_pair_accelerations(SQUARE, config)
    component = .25 + .5 / (3 * np.sqrt(2))
    np.testing.assert_allclose(result[:, :2], SQUARE[:, :2] * component)
    np.testing.assert_allclose(result[:, 2], -2 / 3)
    np.testing.assert_allclose(result[:, :2].sum(axis=0), 0, atol=1e-14)
    one_source = compute_pair_accelerations(SQUARE, config, [True, False, False, False])
    np.testing.assert_array_equal(one_source[0], 0)
    np.testing.assert_allclose(one_source[1], [0, .25, -.25])
    np.testing.assert_allclose(one_source[3], [.5 / (3*np.sqrt(2))] * 2 + [-1/6])
    order = [3, 1, 0, 2]
    np.testing.assert_allclose(compute_pair_accelerations(SQUARE[order], config), result[order])
    np.testing.assert_array_equal(compute_pair_accelerations(SQUARE, InteractionConfig()), 0)


def test_four_child_processes_have_isolated_ids_and_cleanup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Actual launcher setup must allocate all four contexts and reap them."""
    monkeypatch.setattr('dual_uav_interaction.processes.check_ports', lambda *a, **kw: None)
    records: list[Any] = []
    children: list[Child] = []

    def spawn(args: list[str], **kwargs: Any) -> Child:
        records.append((args, kwargs))
        child = Child()
        children.append(child)
        return child

    with SitlPair(fake_runtime(tmp_path), tmp_path/'state', 4,
                  process_factory=spawn, aircraft_count=4):
        assert [row[0][-1] for row in records] == ['4', '5', '6', '7']
        assert [row[1]['cwd'].name for row in records] == list('abcd')
        assert len({row[1]['cwd'] for row in records}) == 4
    assert len(children) == 4 and all(child.stopped for child in children)


def test_fourth_child_failure_reaps_all_three_owned_children(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A late startup failure must not leak the previously launched instances."""
    monkeypatch.setattr('dual_uav_interaction.processes.check_ports', lambda *a, **kw: None)
    children: list[Child] = []

    def spawn(*args: Any, **kwargs: Any) -> Child:
        if len(children) == 3:
            raise OSError('fourth launch failed')
        child = Child()
        children.append(child)
        return child

    with pytest.raises(OSError, match='fourth'):
        with SitlPair(fake_runtime(tmp_path), tmp_path/'state', 4,
                      process_factory=spawn, aircraft_count=4):
            pass
    assert len(children) == 3 and all(child.stopped for child in children)
    with pytest.raises(ValueError):
        SitlPair(tmp_path/'runtime', tmp_path/'other', 7, aircraft_count=4)


def test_four_real_aircraft_have_same_heading_isolated_actuators_and_four_slack_ropes() -> None:
    """Missing/misbound C or D models or ropes breaks physical isolation."""
    import mujoco
    from dual_uav_interaction.payload import ElasticRopeConfig, PayloadConfig, PayloadSystem
    from dual_uav_interaction.scene import load_scene

    config = PayloadConfig(rope=ElasticRopeConfig(rest_length_m=1.6))
    model, data, bindings = load_scene(payload_config=config, aircraft_count=4)
    assert [b.prefix for b in bindings] == ['a/', 'b/', 'c/', 'd/']
    np.testing.assert_allclose([b.mass_kg for b in bindings], 8.0004)
    np.testing.assert_allclose([data.xpos[b.root_id] for b in bindings], SQUARE - [0, 0, 2])
    for b in bindings:
        np.testing.assert_allclose(data.qpos[b.qpos+3:b.qpos+7], [1, 0, 0, 0])
    assert len({int(i) for b in bindings for i in b.thrust_ids}) == 16
    data.ctrl[bindings[2].thrust_ids] = 17
    np.testing.assert_array_equal(data.ctrl[bindings[3].thrust_ids], 0)
    np.testing.assert_allclose([model.site('payload_'+s).pos for s in 'abcd'],
                               [[-.15,-.10,.075],[-.15,.10,.075],[.15,-.10,.075],[.15,.10,.075]])
    system = PayloadSystem(model, config)
    cargo, ropes = system.apply(data)
    assert len(ropes) == 4 and all(row['tension_n'] == 0 for row in ropes)
    assert cargo['bottom_clearance_m'] == pytest.approx(0)
    for b in bindings:
        data.qpos[b.qpos+2] += 2
    mujoco.mj_forward(model, data)
    _, ropes = system.apply(data)
    assert all(row['tension_n'] > 0 for row in ropes)
    cargo_dof = int(model.joint('payload_free').dofadr[0])
    total = data.qfrc_applied[cargo_dof:cargo_dof+3].copy()
    for b in bindings:
        total += data.qfrc_applied[b.dof:b.dof+3]
    np.testing.assert_allclose(total, 0, atol=1e-10)
    assert not np.any(model.tendon_stiffness) and not np.any(model.tendon_damping)


def test_c_d_moves_are_atomic_and_share_one_physics_step() -> None:
    """Independent targets must not mutate peers or reuse pair-only indices."""
    from dual_uav_interaction.payload import ElasticRopeConfig, PayloadConfig
    from dual_uav_interaction.simulation import DualSimulation
    from test_simulation import Bridge

    app = DualSimulation(aircraft_count=4, bridges=[Bridge() for _ in range(4)],
                         payload_config=PayloadConfig(rope=ElasticRopeConfig(rest_length_m=1.6)))
    app.command('takeoff')
    before = np.array([a.target.copy() for a in app.aircraft])
    app.move_aircraft(2, [0, .2, 0])
    app.move_aircraft(3, [0, 0, .2])
    expected = before.copy(); expected[2, 1] += .2; expected[3, 2] += .2
    np.testing.assert_allclose([a.target for a in app.aircraft], expected)
    for index, offset in [(3, [-.8, 0, 0]), (2, [5, 0, 0])]:
        with pytest.raises(ValueError):
            app.move_aircraft(index, offset)
        np.testing.assert_allclose([a.target for a in app.aircraft], expected)
    sample = app.step()
    assert app.data.time == pytest.approx(.005)
    assert len(sample['aircraft']) == 4 and len(sample['ropes']) == 4
    assert np.asarray(sample['pairwise_separations_m']).shape == (4, 4)
    assert sample['min_separation_m'] == pytest.approx(2)


def test_four_surface_sources_follow_fade_and_clear_independently() -> None:
    """A stale two-source array or truncated fourth mesh must fail."""
    import mujoco
    from dual_uav_interaction.disturbance_visualization import DisturbanceVisualization

    model = mujoco.MjModel.from_xml_string('<mujoco/>')
    scene = mujoco.MjvScene(model, maxgeom=16000)
    visual = DisturbanceVisualization()
    visual.update(0., SQUARE, [True]*4)
    assert visual.draw(scene) == 10176
    colors = {tuple(np.round(g.rgba[:3], 3)) for g in scene.geoms[:scene.ngeom]
              if g.type == mujoco.mjtGeom.mjGEOM_TRIANGLE}
    assert len(colors) == 4
    moved = SQUARE + [.2, 0., .5]
    visual.update(.1, moved, [True]*4)
    assert visual.draw(scene) == 10176
    for cache in visual._source_draws:
        np.testing.assert_allclose(cache.position, moved[cache.index])
        assert len(cache.geoms) == 2544
    visual.update(.8, moved, [False, True, False, True])
    visual.draw(scene)
    visual.update(1.5, moved, [False, True, False, True])
    assert visual.draw(scene) == 5088
    visual.set_enabled(False)
    assert visual.draw(scene) == 0 and scene.ngeom == 0


def test_all_twenty_four_direction_buttons_change_only_selected_target() -> None:
    """Exercise the actual canvas hit regions of the four-panel layout."""
    from dual_uav_interaction.panel import ControlPanel
    from dual_uav_interaction.payload import ElasticRopeConfig, PayloadConfig
    from dual_uav_interaction.simulation import DualSimulation
    from test_simulation import Bridge
    from test_panel import click

    app = DualSimulation(aircraft_count=4, bridges=[Bridge() for _ in range(4)],
                         payload_config=PayloadConfig(rope=ElasticRopeConfig(rest_length_m=1.6)))
    app.command('takeoff')
    panel = ControlPanel(app, lambda enabled:None)
    try:
        panel.update(app.step())
        panel.figure.canvas.draw()
        assert len(panel.direction_controls) == 24
        assert len(panel.aircraft_status) == 4
        assert panel.flight_buttons['Takeoff all'].label.get_text() == 'Takeoff all'
        for control in panel.direction_controls:
            before = np.array([a.target.copy() for a in app.aircraft])
            click(panel.figure, control.axes)
            assert not panel.error.get_text()
            expected = before.copy()
            expected[control.aircraft_index,control.axis_index] += control.direction * .2
            np.testing.assert_allclose([a.target for a in app.aircraft], expected)
            for a, target in zip(app.aircraft,before):a.target[:] = target
        click(panel.figure,panel.field_button.ax)
        assert panel.field_visible
        click(panel.figure,panel.field_button.ax)
        assert not panel.field_visible
        panel.figure.canvas.draw()
        assert panel._status_renderer._regions_safe
    finally:
        panel.close()


def test_default_cli_check_builds_four_and_two_mode_keeps_original_rope_length(capsys) -> None:
    """Run the public entrypoint, proving count-dependent defaults take effect."""
    import json
    from dual_uav_interaction.cli import main

    assert main(['--check','--mode','both']) == 0
    four = json.loads(capsys.readouterr().out)
    assert four['aircraft'] == 4
    assert four['payload']['rope']['rest_length_m'] == pytest.approx(1.6)
    assert main(['--check','--aircraft-count','2']) == 0
    two = json.loads(capsys.readouterr().out)
    assert two['aircraft'] == 2
    assert two['payload']['rope']['rest_length_m'] == pytest.approx(1.2)
