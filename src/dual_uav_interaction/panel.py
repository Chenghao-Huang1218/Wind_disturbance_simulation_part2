"""Reference-style directional controls for two independent aircraft targets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.backend_bases import MouseEvent
from matplotlib.patches import Polygon, Rectangle
from matplotlib.widgets import Button, RadioButtons, TextBox
import numpy as np

from .interaction import MODES
from .simulation import DualSimulation
from .panel_rendering import StatusRenderer

BACKGROUND = "#f5f5f5"
STEP_OPTIONS_M = {"Small": .10, "Medium": .20, "Large": .50}


@dataclass
class DirectionControl:
    """One triangular hit region and its isolated movement command."""

    axes: Axes
    patch: Polygon
    aircraft_index: int
    axis_index: int
    direction: int
    base_color: str
    hover_color: str
    hovered: bool = False


class ControlPanel:
    """Provide per-aircraft XYZ pads and shared flight/display controls.

    Args:
        app: Existing shared-world simulation, without a separate physics clock.
        set_disturbance_visible: Display-only callback. Initialization sets False.
    """

    def __init__(
        self,
        app: DualSimulation,
        set_disturbance_visible: Callable[[bool], None] | None = None,
    ) -> None:
        self.app = app
        self.set_disturbance_visible = set_disturbance_visible
        self.step_m = STEP_OPTIONS_M["Medium"]
        self.field_visible = False
        self.figure = plt.figure(figsize=(6, 5.6), facecolor=BACKGROUND)
        self.figure.canvas.manager.set_window_title("Dual UAV Control Panel")
        self.buttons: list[Button] = []
        self.direction_controls: list[DirectionControl] = []
        self.aircraft_status = []
        for index, (shift, color) in enumerate(((0., "#2563a6"),
                                               (-.30, "#c76117"))):
            self.figure.add_artist(Rectangle(
                (.025, .695 + shift), .95, .285,
                transform=self.figure.transFigure, facecolor="#fafafa",
                edgecolor="#d1d5db", linewidth=1, zorder=-10,
            ))
            self.figure.text(.045, .94 + shift, f"UAV {'AB'[index]}",
                             fontsize=10, color=color, fontweight="bold")
            self.aircraft_status.append(self.figure.text(
                .045, .895 + shift, "Waiting for PX4...", va="top",
                fontsize=8, family="monospace", color="#303030",
            ))
            for label, axis, sign, pointer, x, y, base, hover in (
                ("X+", 0, 1, "up", .43, .845, "#dbeafe", "#bfdbfe"),
                ("X-", 0, -1, "down", .43, .715, "#dbeafe", "#bfdbfe"),
                ("Y+", 1, 1, "left", .61, .845, "#dcfce7", "#bbf7d0"),
                ("Y-", 1, -1, "right", .61, .715, "#dcfce7", "#bbf7d0"),
                ("Z+", 2, 1, "up", .79, .845, "#ffedd5", "#fed7aa"),
                ("Z-", 2, -1, "down", .79, .715, "#ffedd5", "#fed7aa"),
            ):
                self._add_direction(index, axis, sign, label, pointer,
                                    [x, y + shift, .13, .105], base, hover)
        self.flight_buttons = {}
        for label, action, x, color in (
            ("Takeoff both", "request_takeoff", .025, "#d7f0dc"),
            ("Land both", "land", .265, "#dbeafe"),
            ("Emergency", "emergency", .505, "#ffe2cf"),
            ("Exit", "exit", .745, "#dedede"),
        ):
            self.flight_buttons[label] = self._button(
                [x, .325, .23, .05], label,
                lambda _, key=action: self.run_command(key), color
            )
        self.figure.text(.045, .28, "Step (m)", fontsize=9,
                         va="center", fontweight="bold")
        self.step_buttons = {}
        for index, (label, step) in enumerate(STEP_OPTIONS_M.items()):
            self.step_buttons[label] = self._button(
                [.22 + index * .12, .257, .105, .045],
                f"{step:.2f}",
                lambda _, value=step: self._set_step(value), "#e2e2e2",
            )
        self._set_step(self.step_m)
        self.field_button = self._button(
            [.64, .257, .335, .045], "Start wind field", self.toggle_field, "#dbeafe"
        )
        self.status = self.figure.text(.045, .235, "",
                                       fontsize=8, color="#555555", va="top")
        self.cargo_status = self.figure.text(
            .045, .202, "Cargo: disabled", va="top", fontsize=8,
            family="monospace",
        )
        self.modes = RadioButtons(
            self.figure.add_axes([.025, .06, .235, .112], facecolor=BACKGROUND),
            MODES, active=MODES.index(app.interaction.mode),
        )
        for label in self.modes.labels:
            label.set_fontsize(8)
        self.fields = []
        for x, label, value in zip(
            (.33, .49, .65), ("Ah (m/s²)", "Az (m/s²)", "L (m)"),
            (app.interaction.horizontal_m_s2,
             app.interaction.vertical_m_s2, app.interaction.length_m),
        ):
            self.figure.text(x, .151, label, fontsize=8, va="bottom")
            self.fields.append(TextBox(
                self.figure.add_axes([x, .10, .12, .045]), "",
                initial=str(value),
            ))
        self._button([.81, .10, .165, .045], "Apply",
                     self.apply_settings, "#dbeafe")
        self.applied = self.figure.text(.29, .065, "", fontsize=7.5)
        self.error = self.figure.text(.035, .035, "", fontsize=7.5,
                                      color="firebrick", va="top", wrap=True)
        self.figure.canvas.mpl_connect("button_press_event", self._on_click)
        self.figure.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.figure.canvas.mpl_connect("close_event",
                                       lambda _: self.app.command("exit"))
        if set_disturbance_visible is not None:
            try:
                set_disturbance_visible(False)
            except RuntimeError as error:
                self.error.set_text(str(error))
        else:
            self.field_button.label.set_text("Field unavailable")
            self.field_button.color = "#e2e2e2"
            self.field_button.ax.set_facecolor(self.field_button.color)
        self._status_renderer = StatusRenderer(self.figure, [
            ((.035, .80, .39, .108), "#fafafa", [self.aircraft_status[0]]),
            ((.035, .50, .39, .108), "#fafafa", [self.aircraft_status[1]]),
            ((.035, .174, .94, .075), BACKGROUND, [self.status, self.cargo_status]),
            ((.025, .001, .95, .042), BACKGROUND, [self.error]),
            ((.28, .052, .695, .04), BACKGROUND, [self.applied]),
        ])

    def _button(
        self, box: list[float], label: str, callback: Callable[..., None],
        color: str,
    ) -> Button:
        """Create and retain one reference-style Matplotlib button."""
        button = Button(self.figure.add_axes(box), label, color=color,
                        hovercolor="#c5d4e0")
        button.label.set_fontsize(8)
        button.label.set_fontweight("bold")
        button.on_clicked(callback)
        self.buttons.append(button)
        return button

    def _add_direction(
        self, aircraft: int, axis: int, direction: int, label: str,
        pointer: str, box: list[float], base: str, hover: str,
    ) -> None:
        """Create a triangle with the reference's axis colors and hit shape."""
        vertices = {
            "up": [(.5, .9), (.12, .12), (.88, .12)],
            "down": [(.12, .88), (.88, .88), (.5, .1)],
            "left": [(.1, .5), (.88, .88), (.88, .12)],
            "right": [(.12, .88), (.9, .5), (.12, .12)],
        }
        axes = self.figure.add_axes(box, facecolor="#fafafa")
        axes.set(xlim=(0, 1), ylim=(0, 1))
        axes.set_axis_off()
        patch = Polygon(vertices[pointer], closed=True, facecolor=base,
                        edgecolor="#606060", linewidth=1.2,
                        transform=axes.transAxes)
        axes.add_patch(patch)
        axes.text(.5, .5, label, ha="center", va="center", fontsize=9,
                  fontweight="bold", color="#252525")
        self.direction_controls.append(DirectionControl(
            axes, patch, aircraft, axis, direction, base, hover
        ))

    def _on_click(self, event: MouseEvent) -> None:
        """Apply one isolated step only for a left click inside a triangle."""
        if event.button != 1:
            return
        for control in self.direction_controls:
            if event.inaxes is control.axes and control.patch.contains(event)[0]:
                offset = np.zeros(3)
                offset[control.axis_index] = control.direction * self.step_m
                try:
                    self.app.move_aircraft(control.aircraft_index, offset)
                    self.error.set_text("")
                except (ValueError, RuntimeError) as error:
                    self.error.set_text(str(error))
                self.figure.canvas.draw_idle()
                return

    def _on_motion(self, event: MouseEvent) -> None:
        """Highlight only the actual triangular hover area."""
        changed = False
        for control in self.direction_controls:
            hovered = (event.inaxes is control.axes
                       and control.patch.contains(event)[0])
            if hovered != control.hovered:
                control.hovered = hovered
                control.patch.set_facecolor(
                    control.hover_color if hovered else control.base_color
                )
                changed = True
        if changed:
            self.figure.canvas.draw_idle()

    def _set_step(self, step: float) -> None:
        """Select a shared increment while preserving independent targets."""
        self.step_m = step
        for label, button in self.step_buttons.items():
            button.color = "#b7d7f0" if STEP_OPTIONS_M[label] == step else "#e2e2e2"
            button.ax.set_facecolor(button.color)
        self.figure.canvas.draw_idle()

    def toggle_field(self, event: Any) -> None:
        """Toggle visualization only; commit the label after callback success."""
        if self.set_disturbance_visible is None:
            return
        desired = not self.field_visible
        try:
            self.set_disturbance_visible(desired)
        except (RuntimeError, ValueError) as error:
            self.error.set_text(str(error))
            self.figure.canvas.draw_idle()
            return
        self.field_visible = desired
        self.field_button.label.set_text("Stop wind field" if desired else "Start wind field")
        self.field_button.color = "#fbd2c4" if desired else "#dbeafe"
        self.field_button.ax.set_facecolor(self.field_button.color)
        self.error.set_text("")
        self.figure.canvas.draw_idle()

    def apply_settings(self, event: Any) -> None:
        """Atomically apply mode and all fields, preserving valid prior settings."""
        try:
            self.app.set_interaction(self.modes.value_selected,
                                     *[float(field.text) for field in self.fields])
            self.error.set_text("")
        except (ValueError, OverflowError) as error:
            self.error.set_text(str(error))
        self.figure.canvas.draw_idle()

    def run_command(self, action: str) -> None:
        """Display rejected flight commands without interrupting physics."""
        try:
            self.app.command(action)
            self.error.set_text("")
        except (ValueError, RuntimeError) as error:
            self.error.set_text(str(error))
        self.flight_buttons["Takeoff both"].label.set_text(
            "Arming both..." if self.app.takeoff_pending else "Takeoff both"
        )
        self.figure.canvas.draw_idle()

    def update(self, sample: dict[str, Any]) -> None:
        """Refresh actual/target positions and loads from one same-time sample."""
        for index, row in enumerate(sample["aircraft"]):
            connected = self.app.aircraft[index].bridge.heartbeat_is_recent()
            position = np.array(row["position"])
            target = np.array(row["target"])
            self.aircraft_status[index].set_text(
                f"{'connected' if connected else 'waiting'} | {row['phase']}\n"
                f"Pos: {np.array2string(position, precision=2, floatmode='fixed')}\n"
                f"Tgt: {np.array2string(target, precision=2, floatmode='fixed')}"
            )
        label = "Arming both..." if self.app.takeoff_pending else "Takeoff both"
        takeoff = self.flight_buttons["Takeoff both"].label
        flight_label_changed = takeoff.get_text() != label
        if flight_label_changed:
            takeoff.set_text(label)
        cargo = sample.get("payload")
        if cargo:
            tension = [round(row["tension_n"], 2) for row in sample["ropes"]]
            self.cargo_status.set_text(
                f"Cargo: {cargo['position'][2]:.2f} m | "
                f"Rope A/B: {tension} N"
            )
        config = sample["interaction"]
        self.applied.set_text(
            f"Applied: {config['mode']} | Ah={config['horizontal_m_s2']:g} "
            f"Az={config['vertical_m_s2']:g} L={config['length_m']:g}"
        )
        self.status.set_text(
            f"Distance: {sample['separation_m']:.2f} m"
        )
        if flight_label_changed:
            self.figure.canvas.draw_idle()
        else:
            self._status_renderer.draw()

    def process_events(self) -> None:
        """Dispatch canvas events without re-showing or raising this window.

        Tk's pyplot.pause calls show on every refresh, which can repeatedly
        toggle topmost and steal window focus. Initial show belongs to startup.
        """
        self.figure.canvas.flush_events()

    def close(self) -> None:
        """Close this panel and dispatch its normal exit request."""
        plt.close(self.figure)
