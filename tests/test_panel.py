"""Exercise real Matplotlib clicks, isolated targets and display-only toggles."""

from matplotlib.backend_bases import MouseEvent
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import mujoco
import numpy as np
import pytest

from dual_uav_interaction.disturbance_visualization import DisturbanceVisualization
from dual_uav_interaction.panel import ControlPanel
from dual_uav_interaction.payload import PayloadConfig
from dual_uav_interaction.simulation import DualSimulation
from test_simulation import Bridge, airborne_app


def click(figure: Figure, axes: Axes, x: float = .5,
          y: float = .5, button: int = 1) -> None:
    """Dispatch an actual canvas event at an axes-relative point."""
    pixels = axes.transAxes.transform((x, y))
    event = MouseEvent('button_press_event', figure.canvas, *pixels, button=button)
    figure.canvas.callbacks.process('button_press_event', event)
    release = MouseEvent('button_release_event', figure.canvas, *pixels, button=button)
    figure.canvas.callbacks.process('button_release_event', release)


def test_panel_supports_atomic_settings() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    panel = ControlPanel(app)
    try:
        assert tuple(label.get_text() for label in panel.modes.labels) == (
            'off', 'horizontal', 'vertical', 'both')
        panel.modes.set_active(3)
        for field, value in zip(panel.fields, ('1', '2', '3')):
            field.set_val(value)
        panel.apply_settings(None)
        assert (app.interaction.mode, app.interaction.length_m) == ('both', 3)
        old = app.interaction
        panel.fields[2].set_val('nan')
        panel.apply_settings(None)
        assert app.interaction is old
        assert panel.error.get_text()
    finally:
        panel.close()


def test_event_pump_dispatches_without_reshowing_or_raising_window(monkeypatch) -> None:
    from matplotlib import pyplot as plt

    app = DualSimulation(bridges=[Bridge(), Bridge()])
    panel = ControlPanel(app)
    calls = []

    def forbidden(*args, **kwargs) -> None:
        pytest.fail('refresh must not show/raise the control panel')

    try:
        monkeypatch.setattr(plt, 'pause', forbidden)
        monkeypatch.setattr(plt, 'show', forbidden)
        monkeypatch.setattr(panel.figure.canvas.manager, 'show', forbidden)
        monkeypatch.setattr(panel.figure.canvas, 'flush_events',
                            lambda: calls.append('flushed'))
        for _ in range(10):
            panel.process_events()
        assert calls == ['flushed'] * 10
    finally:
        panel.close()


def test_both_direction_pads_use_real_triangle_hit_testing_and_step() -> None:
    app = airborne_app()
    panel = ControlPanel(app)
    try:
        panel.figure.canvas.draw()
        assert len(panel.direction_controls) == 12
        assert panel.step_m == pytest.approx(.20)
        for label, step in (('Small', .10), ('Medium', .20), ('Large', .50)):
            click(panel.figure, panel.step_buttons[label].ax)
            assert panel.step_m == pytest.approx(step)
        click(panel.figure, panel.step_buttons['Large'].ax)
        assert panel.step_m == pytest.approx(.50)
        for control in panel.direction_controls:
            before = np.array([a.target.copy() for a in app.aircraft])
            click(panel.figure, control.axes, x=.02, y=.02)
            click(panel.figure, control.axes, button=3)
            np.testing.assert_array_equal([a.target for a in app.aircraft], before)
            click(panel.figure, control.axes)
            expected = before.copy()
            expected[control.aircraft_index, control.axis_index] += control.direction * .50
            np.testing.assert_allclose([a.target for a in app.aircraft], expected)
            for a, target in zip(app.aircraft, before):
                a.target[:] = target
    finally:
        panel.close()


def test_field_starts_disabled_then_button_toggles_without_changing_physics() -> None:
    app = airborne_app()
    visual = DisturbanceVisualization()
    scene = mujoco.MjvScene(app.model, maxgeom=1000)

    def set_visible(enabled: bool) -> None:
        visual.set_enabled(enabled)
        visual.draw(scene)

    panel = ControlPanel(app, set_visible)
    previous = app.interaction
    try:
        assert not panel.field_visible and not visual.enabled
        assert panel.field_button.label.get_text() == 'Start wind field'
        panel.figure.canvas.draw()
        click(panel.figure, panel.field_button.ax)
        assert panel.field_visible and visual.enabled
        assert panel.field_button.label.get_text() == 'Stop wind field'
        visual.update(0., np.array([[0, -1, 3], [0, 1, 3]]), [True, True])
        assert visual.draw(scene) > 0
        click(panel.figure, panel.field_button.ax)
        assert not visual.enabled and scene.ngeom == 0
        assert app.interaction is previous
    finally:
        panel.close()


def test_invalid_move_or_failed_display_callback_reports_error_without_commit() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()], payload_config=PayloadConfig())
    app.command('takeoff')

    def fail(enabled: bool) -> None:
        if enabled:
            raise RuntimeError('viewer unavailable')

    panel = ControlPanel(app, fail)
    try:
        panel.step_m = .25
        control = next(c for c in panel.direction_controls
                       if c.aircraft_index == 0 and c.axis_index == 1 and c.direction == -1)
        app.aircraft[0].target[1] -= .25
        before = np.array([a.target.copy() for a in app.aircraft])
        panel.figure.canvas.draw()
        click(panel.figure, control.axes)
        assert 'separation' in panel.error.get_text()
        np.testing.assert_array_equal([a.target for a in app.aircraft], before)
        click(panel.figure, panel.field_button.ax)
        assert 'viewer unavailable' in panel.error.get_text()
        assert not panel.field_visible
        assert panel.field_button.label.get_text() == 'Start wind field'
    finally:
        panel.close()


def test_takeoff_button_and_loaded_readouts_have_room() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()], payload_config=PayloadConfig())
    panel = ControlPanel(app, lambda enabled: None)
    try:
        panel.figure.canvas.draw()
        click(panel.figure, panel.flight_buttons['Takeoff both'].ax)
        assert app.takeoff_pending
        panel.update(app.step())
        assert 'Rope A/B' in panel.cargo_status.get_text()
        panel.figure.canvas.draw()
        renderer = panel.figure.canvas.get_renderer()
        assert panel._status_renderer._regions_safe
        artists = [panel.cargo_status, panel.error, panel.applied,
                   *panel.aircraft_status]
        for artist in artists:
            box = artist.get_window_extent(renderer)
            assert panel.figure.bbox.contains(box.x0, box.y0)
            assert panel.figure.bbox.contains(box.x1, box.y1)
            widget_axes = [panel.field_button.ax, panel.modes.ax,
                           *[button.ax for button in panel.flight_buttons.values()],
                           *[button.ax for button in panel.step_buttons.values()],
                           *[field.ax for field in panel.fields],
                           *[control.axes for control in panel.direction_controls]]
            for axes in widget_axes:
                assert not box.overlaps(axes.get_window_extent(renderer))
    finally:
        panel.close()


def test_status_update_preserves_controls_without_full_redraw(monkeypatch) -> None:
    app = airborne_app()
    panel = ControlPanel(app, lambda enabled: None)
    try:
        sample = app.step()
        panel.update(sample)
        panel.figure.canvas.draw()
        click(panel.figure, panel.step_buttons['Large'].ax)
        panel.fields[0].set_val('0.123')  # Keep an unapplied user draft.
        panel.figure.canvas.draw()
        before = np.asarray(panel.figure.canvas.buffer_rgba()).copy()

        def forbidden() -> None:
            pytest.fail('status refresh must not redraw the whole panel')

        monkeypatch.setattr(panel.figure.canvas, 'draw_idle', forbidden)
        original_blit = panel.figure.canvas.blit
        blits = []

        def track_blit(bbox) -> None:
            blits.append(bbox)
            original_blit(bbox)

        monkeypatch.setattr(panel.figure.canvas, 'blit', track_blit)
        sample['aircraft'][0]['position'][0] += .5
        panel.update(sample)
        assert len(blits) == 1
        after = np.asarray(panel.figure.canvas.buffer_rgba()).copy()
        assert np.any(before != after)
        # Lower settings controls are outside every dynamic status region.
        height = before.shape[0]
        y0, y1 = int(height * (1 - .21)), int(height * (1 - .14))
        np.testing.assert_array_equal(before[y0:y1], after[y0:y1])
        assert panel.fields[0].text == '0.123'
        assert panel.step_m == pytest.approx(.5)
        panel.figure.canvas.draw()
        np.testing.assert_array_equal(
            after, np.asarray(panel.figure.canvas.buffer_rgba())
        )
    finally:
        panel.close()


@pytest.mark.parametrize('size', [(6., 5.6), (5., 4.67), (4., 3.73)])
def test_resized_status_refresh_matches_complete_canvas(size: tuple[float, float]) -> None:
    app = airborne_app()
    panel = ControlPanel(app)
    try:
        panel.figure.set_size_inches(*size)
        panel.figure.canvas.draw()
        panel.update(app.step())
        partial = np.asarray(panel.figure.canvas.buffer_rgba()).copy()
        panel.figure.canvas.draw()
        np.testing.assert_array_equal(
            partial, np.asarray(panel.figure.canvas.buffer_rgba())
        )
    finally:
        panel.close()


def test_native_pixel_rounding_does_not_disable_partial_render(monkeypatch) -> None:
    app = airborne_app()
    panel = ControlPanel(app)
    try:
        panel.figure.set_size_inches(6., 5.59)
        panel.figure.canvas.draw()

        def forbidden() -> None:
            pytest.fail('one-pixel canvas rounding must retain partial redraw')

        monkeypatch.setattr(panel.figure.canvas, 'draw_idle', forbidden)
        panel.update(app.step())
        partial = np.asarray(panel.figure.canvas.buffer_rgba()).copy()
        panel.figure.canvas.draw()
        np.testing.assert_array_equal(
            partial, np.asarray(panel.figure.canvas.buffer_rgba())
        )
    finally:
        panel.close()


def test_narrow_window_removes_old_text_outside_status_region() -> None:
    app = airborne_app()
    panel = ControlPanel(app)
    try:
        panel.figure.set_size_inches(4., 5.6)
        sample = app.step()
        sample['aircraft'][0]['position'] = [-9., -1., 3.]
        panel.update(sample)
        panel.figure.canvas.draw()
        sample['aircraft'][0]['position'] = [1., -1., 3.]
        panel.update(sample)
        partial = np.asarray(panel.figure.canvas.buffer_rgba()).copy()
        panel.figure.canvas.draw()
        np.testing.assert_array_equal(
            partial, np.asarray(panel.figure.canvas.buffer_rgba())
        )
    finally:
        panel.close()
