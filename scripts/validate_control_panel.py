"""Bounded native GUI / actual dual-SITL validation via real canvas clicks.

Example: MPLBACKEND=TkAgg xvfb-run -a python scripts/validate_control_panel.py
Starts only project-owned PX4 instances; never builds firmware.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from typing import Any

from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.backend_bases import MouseEvent
from matplotlib.figure import Figure
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dual_uav_interaction.disturbance_visualization import DisturbanceVisualization  # noqa: E402
from dual_uav_interaction.panel import ControlPanel  # noqa: E402
from dual_uav_interaction.payload import PayloadConfig  # noqa: E402
from dual_uav_interaction.processes import SitlPair  # noqa: E402
from dual_uav_interaction.simulation import DualSimulation  # noqa: E402
from dual_uav_interaction.viewer import ManagedPassiveViewer  # noqa: E402
from dual_uav_interaction.display import DisplaySchedule  # noqa: E402


def click(figure: Figure, axes: Axes) -> None:
    """Dispatch a left press/release at a real canvas control's centre."""
    point = axes.transAxes.transform((.5, .5))
    for event_name in ("button_press_event", "button_release_event"):
        event = MouseEvent(event_name, figure.canvas, *point, button=1)
        figure.canvas.callbacks.process(event_name, event)


def validate(output: Path, instance_base: int, timeout: float) -> bool:
    """Run takeoff, field toggle, six isolated moves and grounded landing.

    Args:
        output: Fresh evidence directory, rejected if already present.
        instance_base: First of two local PX4 instances.
        timeout: Maximum wall seconds for the complete validation.

    Returns:
        True only after all GUI-driven moves and grounded/disarmed landing.
    """
    output.mkdir(parents=True, exist_ok=False)
    app = DualSimulation(instance_base=instance_base, payload_config=PayloadConfig())
    app.set_interaction("both", .5, .5, 2.)
    visual = DisturbanceVisualization()
    visual.set_enabled(False)
    viewer = None
    panel = None
    pair = SitlPair(ROOT / "runtime/px4", output / "px4", instance_base)
    events: list[dict[str, Any]] = []
    success = False
    error = ""
    phase = "waiting"
    started = time.perf_counter()
    stage_started = 0.
    stable_since = None
    move_index = 0
    pending_move = None
    display_schedule = DisplaySchedule()
    last_sample = None
    moves = [(index, axis) for index in (0, 1) for axis in (0, 1, 2)]
    try:
        with pair:
            app.connect()
            viewer = ManagedPassiveViewer(app.model, app.data)
            viewer.cam.distance = 7
            viewer.cam.lookat[:] = [.3, 0, 1.5]

            def set_visible(enabled: bool) -> None:
                """Drive exactly the same viewer callback as the application."""
                with viewer.lock():
                    visual.set_enabled(enabled)
                    visual.draw(viewer.user_scn)
                viewer.sync()

            panel = ControlPanel(app, set_visible)
            plt.show(block=False)
            panel.figure.canvas.draw()
            with (output / "states.jsonl").open("w", buffering=1) as stream:
                iteration = 0
                while not app.exit_requested:
                    tick = time.perf_counter()
                    if tick - started > timeout:
                        raise RuntimeError(f"validation timed out in {phase}")
                    if any(child.poll() is not None for child in pair.children):
                        raise RuntimeError("owned PX4 exited early")
                    with viewer.lock():
                        sample = app.step()
                        sample["wall_elapsed_s"] = tick - started
                        last_sample = sample
                        visual.update(sample["time_s"], np.array([
                            row["position"] for row in sample["aircraft"]
                        ]), [row["disturbance_source_active"]
                             for row in sample["aircraft"]])
                    now = sample["time_s"]
                    if iteration % 4 == 0:
                        stream.write(json.dumps({**sample, "validation_phase": phase}) + "\n")
                    if phase == "waiting" and all(
                        a.bridge.heartbeat_is_recent() for a in app.aircraft
                    ):
                        click(panel.figure, panel.flight_buttons["Takeoff both"].ax)
                        assert app.takeoff_pending and not panel.error.get_text()
                        events.append({"event": "Takeoff both clicked", "time_s": now})
                        phase = "lifting"
                        print("GUI Takeoff both clicked; awaiting actual lift", flush=True)
                    elif phase == "lifting":
                        cargo = sample["payload"]
                        settled = (all(a["phase"] == "hover" for a in sample["aircraft"])
                                   and cargo["bottom_clearance_m"] > .15
                                   and np.linalg.norm(cargo["velocity"]) < .2)
                        stable_since = (
                            now if stable_since is None else stable_since
                        ) if settled else None
                        if stable_since is not None and now - stable_since >= 1:
                            click(panel.figure, panel.field_button.ax)
                            assert panel.field_visible and visual.enabled
                            stage_started = now
                            phase = "field"
                    elif phase == "field" and now - stage_started > .2:
                        with viewer.lock():
                            count = visual.draw(viewer.user_scn)
                        assert count > 0
                        click(panel.figure, panel.field_button.ax)
                        assert not panel.field_visible and viewer.user_scn.ngeom == 0
                        events.append({"event": "field start/draw/stop",
                                       "geoms": count, "time_s": now})
                        # Keep full-resolution fields on throughout the expensive
                        # moving/landing segment for display-performance validation.
                        click(panel.figure, panel.field_button.ax)
                        assert panel.field_visible
                        events.append({"event": "field restarted for moves", "time_s": now})
                        phase = "moves"
                    elif phase == "moves":
                        if pending_move is None:
                            index, axis = moves[move_index]
                            before = np.array([a.target.copy() for a in app.aircraft])
                            actual_before = np.array(sample["aircraft"][index]["position"])
                            control = next(c for c in panel.direction_controls
                                           if c.aircraft_index == index
                                           and c.axis_index == axis and c.direction == 1)
                            click(panel.figure, control.axes)
                            assert not panel.error.get_text()
                            expected = before.copy()
                            expected[index, axis] += panel.step_m
                            np.testing.assert_allclose([a.target for a in app.aircraft], expected)
                            pending_move = (index, axis, actual_before, expected)
                            stage_started = now
                        else:
                            index, axis, before, expected = pending_move
                            actual = np.array(sample["aircraft"][index]["position"])
                            if (now - stage_started >= 1.5
                                and actual[axis] - before[axis] > .6 * panel.step_m
                                and np.linalg.norm(actual - expected[index]) < .08):
                                events.append({"event": "independent move", "aircraft": index,
                                               "axis": axis, "target": expected.tolist(),
                                               "measured_axis_displacement_m": float(
                                                   actual[axis] - before[axis]),
                                               "time_s": now})
                                print(f"GUI UAV {'AB'[index]} {'XYZ'[axis]}+ verified", flush=True)
                                move_index += 1
                                pending_move = None
                                if move_index == len(moves):
                                    panel.update(sample)
                                    panel.figure.savefig(output / "native-panel.png", dpi=140)
                                    click(panel.figure, panel.flight_buttons["Land both"].ax)
                                    phase = "landing"
                                    stable_since = None
                    elif phase == "landing":
                        cargo = sample["payload"]
                        grounded = (all(a["phase"] == "landed" and not a["armed"]
                                        for a in sample["aircraft"])
                                    and cargo["floor_contact"]
                                    and np.linalg.norm(cargo["velocity"]) < .2
                                    and all(r["tension_n"] < .1 for r in sample["ropes"]))
                        stable_since = (
                            now if stable_since is None else stable_since
                        ) if grounded else None
                        if stable_since is not None and now - stable_since > .7:
                            success = True
                            events.append({"event": "grounded/disarmed", "time_s": now})
                            if panel.field_visible:
                                click(panel.figure, panel.field_button.ax)
                            click(panel.figure, panel.flight_buttons["Exit"].ax)
                    due = display_schedule.poll(time.perf_counter())
                    if due.field:
                        with viewer.lock():
                            visual.draw(viewer.user_scn)
                    if due.viewer:
                        if not viewer.is_running():
                            raise RuntimeError("viewer closed before completion")
                        viewer.sync()
                    if due.panel:
                        panel.update(sample)
                    if due.events:
                        panel.process_events()
                    iteration += 1
                    remaining = app.model.opt.timestep - (time.perf_counter() - tick)
                    if remaining > 0:
                        time.sleep(remaining)
    except (Exception, KeyboardInterrupt) as problem:
        error = f"{type(problem).__name__}: {problem}"
        print(error, flush=True)
    finally:
        app.close()
        if panel:
            panel.close()
        if viewer:
            viewer.close()
        summary = {"success": success and not error, "error": error,
                   "native_GUI": True, "backend": plt.get_backend(),
                   "actual_dual_PX4": True, "instance_base": instance_base,
                   "events": events, "phase": phase,
                   "elapsed_wall_s": time.perf_counter() - started,
                   "simulated_time_s": last_sample["time_s"] if last_sample else 0.,
                   "realtime_factor": (
                       last_sample["time_s"] / max(time.perf_counter() - started, 1e-9)
                       if last_sample else 0.
                   ),
                   "display_rates_hz": DisplaySchedule.rates_hz,
                   "owned_px4_stopped": all(p.poll() is not None for p in pair.children),
                   "final_payload": last_sample["payload"] if last_sample else None}
        (output / "summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary), flush=True)
    return success and not error


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/validation" /
                        ("control_panel_" + datetime.now().strftime("%Y%m%d_%H%M%S")))
    parser.add_argument("--instance-base", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=180.)
    args = parser.parse_args()
    raise SystemExit(0 if validate(args.output_dir.resolve(), args.instance_base,
                                  args.timeout) else 1)
