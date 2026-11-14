"""Independent target movement and paired acknowledgement-driven takeoff."""

import numpy as np
import pytest

from dual_uav_interaction.payload import PayloadConfig
from dual_uav_interaction.simulation import DualSimulation
from test_simulation import Bridge, airborne_app


@pytest.mark.parametrize('index', [0, 1])
@pytest.mark.parametrize('axis', [0, 1, 2])
def test_independent_world_axis_moves_only_selected_target(index: int, axis: int) -> None:
    app = airborne_app()
    before = np.array([a.target.copy() for a in app.aircraft])
    offset = np.zeros(3)
    offset[axis] = .1
    app.move_aircraft(index, offset)
    expected = before.copy()
    expected[index] += offset
    np.testing.assert_array_equal([a.target for a in app.aircraft], expected)


def test_loaded_spacing_rejection_is_atomic() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()], payload_config=PayloadConfig())
    app.command('takeoff')
    before = np.array([a.target.copy() for a in app.aircraft])
    with pytest.raises(ValueError, match='separation'):
        app.move_aircraft(0, [0, -1, 0])
    np.testing.assert_array_equal([a.target for a in app.aircraft], before)


def test_bad_index_offset_or_ground_phase_does_not_change_targets() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    before = np.array([a.target.copy() for a in app.aircraft])
    for index, offset in ((-1, [0, 0, 1]), (2, [0, 0, 1]),
                          (0, [0, float('nan'), 0]), (0, [1, 2])):
        with pytest.raises(ValueError):
            app.move_aircraft(index, offset)
    with pytest.raises(RuntimeError, match='flying'):
        app.move_aircraft(0, [0, 0, .1])
    np.testing.assert_array_equal([a.target for a in app.aircraft], before)


def test_finite_offsets_cannot_overflow_a_target() -> None:
    app = airborne_app()
    app.aircraft[0].target[2] = np.finfo(float).max
    before = np.array([a.target.copy() for a in app.aircraft])
    with pytest.raises(ValueError, match='finite'):
        app.move_aircraft(0, [0, 0, np.finfo(float).max])
    np.testing.assert_array_equal([a.target for a in app.aircraft], before)


def test_one_click_takeoff_waits_for_both_real_acks() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    for a in app.aircraft:
        a.bridge.armed = False
    app.command('request_takeoff')
    app.step()
    assert app.takeoff_pending
    assert not any(a.phase == 'takeoff' for a in app.aircraft)
    app.aircraft[0].bridge.armed = True
    app.step()
    assert app.takeoff_pending
    app.aircraft[1].bridge.armed = True
    app.step()
    assert not app.takeoff_pending
    assert all(a.phase == 'takeoff' for a in app.aircraft)


@pytest.mark.parametrize('action', ['land', 'emergency', 'exit'])
def test_stop_actions_cancel_pending_takeoff(action: str) -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    for a in app.aircraft:
        a.bridge.armed = False
    app.command('request_takeoff')
    app.command(action)
    for a in app.aircraft:
        a.bridge.armed = True
    app.step()
    assert not app.takeoff_pending
    assert not any(a.phase == 'takeoff' for a in app.aircraft)
    assert not any(a.arm_pending for a in app.aircraft)
