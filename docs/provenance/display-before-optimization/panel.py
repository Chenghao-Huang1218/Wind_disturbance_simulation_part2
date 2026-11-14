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
        self.figure = plt.figure(figsize=(12, 7.2), facecolor=BACKGROUND)
        self.figure.canvas.manager.set_window_title("Dual UAV Control Panel")
        self.figure.suptitle("Dual UAV Control Panel", fontsize=15,
                             fontweight="bold", y=.975)
        self.buttons: list[Button] = []
        self.direction_controls: list[DirectionControl] = []
        self.aircraft_status = []
        for index, (origin, color) in enumerate(((.04, "#2563a6"),
                                                (.36, "#c76117"))):
            self.figure.add_artist(Rectangle(
                (origin - .01, .30), .30, .63,
                transform=self.figure.transFigure, facecolor="#fafafa",
                edgecolor="#d1d5db", linewidth=1, zorder=-10,
            ))
            self.figure.text(origin + .01, .90, f"UAV {'AB'[index]}",
                             fontsize=12, color=color, fontweight="bold")
            self.aircraft_status.append(self.figure.text(
                origin + .01, .865, "Waiting for PX4...", va="top",
                fontsize=9, family="monospace", color="#303030",
            ))
            for label, axis, sign, pointer, x, y, base, hover in (
                ("X+", 0, 1, "up", .095, .625, "#dbeafe", "#bfdbfe"),
                ("X-", 0, -1, "down", .095, .445, "#dbeafe", "#bfdbfe"),
                ("Y+", 1, 1, "left", .005, .535, "#dcfce7", "#bbf7d0"),
                ("Y-", 1, -1, "right", .185, .535, "#dcfce7", "#bbf7d0"),
                ("Z+", 2, 1, "up", .005, .335, "#ffedd5", "#fed7aa"),
                ("Z-", 2, -1, "down", .185, .335, "#ffedd5", "#fed7aa"),
            ):
                self._add_direction(index, axis, sign, label, pointer,
                                    [origin + x, y, .075, .075], base, hover)
        self.flight_buttons = {}
        for label, action, box, color in (
            ("Takeoff both", "request_takeoff", [.69, .805, .13, .065], "#d7f0dc"),
            ("Land both", "land", [.835, .805, .13, .065], "#dbeafe"),
            ("Emergency", "emergency", [.69, .725, .13, .065], "#ffe2cf"),
            ("Exit", "exit", [.835, .725, .13, .065], "#dedede"),
        ):
            self.flight_buttons[label] = self._button(
                box, label, lambda _, key=action: self.run_command(key), color
            )
        self.figure.text(.69, .675, "Shared step (m)", fontsize=10,
                         fontweight="bold")
        self.step_buttons = {}
        for index, (label, step) in enumerate(STEP_OPTIONS_M.items()):
            self.step_buttons[label] = self._button(
                [.69 + index * .095, .605, .08, .055],
                f"{step:.2f}",
                lambda _, value=step: self._set_step(value), "#e2e2e2",
            )
        self._set_step(self.step_m)
        self.field_button = self._button(
            [.69, .505, .27, .065], "Start field", self.toggle_field, "#dbeafe"
        )
        self.cargo_status = self.figure.text(
            .69, .435, "Cargo: disabled", va="top", fontsize=9,
            family="monospace",
        )
        self.error = self.figure.text(.04, .272, "", fontsize=9,
                                      color="firebrick", va="top")
        self.figure.text(.04, .235, "Disturbance",
                         fontsize=10, fontweight="bold")
        self.modes = RadioButtons(
            self.figure.add_axes([.04, .065, .245, .145], facecolor=BACKGROUND),
            MODES, active=MODES.index(app.interaction.mode),
        )
        for label in self.modes.labels:
            label.set_fontsize(9)
        self.fields = []
        for x, label, value in zip(
            (.375, .54, .705), ("Ah (m/s²)", "Az (m/s²)", "L (m)"),
            (app.interaction.horizontal_m_s2,
             app.interaction.vertical_m_s2, app.interaction.length_m),
        ):
            self.fields.append(TextBox(
                self.figure.add_axes([x, .145, .085, .05]), label,
                initial=str(value),
            ))
        self._button([.83, .145, .13, .05], "Apply",
                     self.apply_settings, "#dbeafe")
        self.applied = self.figure.text(.325, .105, "", fontsize=9)
        self.status = self.figure.text(.69, .465, "",
                                       fontsize=9, color="#555555")
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

    def _button(
        self, box: list[float], label: str, callback: Callable[..., None],
        color: str,
    ) -> Button:
        """Create and retain one reference-style Matplotlib button."""
        button = Button(self.figure.add_axes(box), label, color=color,
                        hovercolor="#c5d4e0")
        button.label.set_fontsize(9)
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
        axes.text(.5, .5, label, ha="center", va="center", fontsize=11,
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
        self.field_button.label.set_text("Stop field" if desired else "Start field")
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
        self.flight_buttons["Takeoff both"].label.set_text(
            "Arming both..." if self.app.takeoff_pending else "Takeoff both"
        )
        cargo = sample.get("payload")
        if cargo:
            tension = [round(row["tension_n"], 2) for row in sample["ropes"]]
            self.cargo_status.set_text(
                f"Cargo: {cargo['position'][2]:.2f} m\n"
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
        self.figure.canvas.draw_idle()

    def process_events(self) -> None:
        """Dispatch canvas events without re-showing or raising this window.

        Tk's pyplot.pause calls show on every refresh, which can repeatedly
        toggle topmost and steal window focus. Initial show belongs to startup.
        """
        self.figure.canvas.flush_events()

    def close(self) -> None:
        """Close this panel and dispatch its normal exit request."""
        plt.close(self.figure)
