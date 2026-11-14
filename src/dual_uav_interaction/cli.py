"""Launch an independent fleet, graphical panel, or bounded headless checks."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import asdict
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import time
from typing import Any, Sequence

import numpy as np

from .interaction import InteractionConfig, MODES
from .processes import SitlPair
from .simulation import DualSimulation
from .payload import ElasticRopeConfig, PayloadConfig
from .transport import TransportSequence
from .display import DisplaySchedule

PROJECT_ROOT = Path(__file__).resolve().parents[2]


def parser() -> argparse.ArgumentParser:
    """Describe runtime settings and the four artificial interaction modes."""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--aircraft-count", type=int, choices=(2, 4), default=4,
                        help="number of aircraft; default four")
    result.add_argument("--panel-hz", type=float, default=None,
                        help="numeric readout Hz; defaults 1 for four aircraft, 2 for two")
    result.add_argument("--mode", choices=MODES, default="off")
    result.add_argument("--no-payload", action="store_true", help="run without cargo or ropes")
    result.add_argument("--payload-mass", type=float, default=.5, help="cargo mass (kg)")
    result.add_argument("--rope-length", type=float, default=None,
                        help="unstretched rope length (m); defaults 1.6 for four aircraft, 1.2 for two")
    result.add_argument("--rope-stiffness", type=float, default=100.,
                        help="axial spring stiffness (N/m)")
    result.add_argument("--rope-damping", type=float, default=3., help="axial damping (N s/m)")
    result.add_argument(
        "--horizontal",
        type=float,
        default=0.5,
        help="horizontal peak acceleration (m/s²)",
    )
    result.add_argument(
        "--vertical",
        type=float,
        default=0.5,
        help="downward peak acceleration (m/s²)",
    )
    result.add_argument(
        "--length",
        type=float,
        default=2.0,
        help="half-amplitude horizontal distance L (m)",
    )
    result.add_argument(
        "--separation",
        type=float,
        default=2.0,
        help="initial center separation (m), >=1.5",
    )
    result.add_argument(
        "--height",
        type=float,
        default=2.0,
        help="takeoff height above initial root (m)",
    )
    result.add_argument(
        "--instance-base",
        type=int,
        default=0,
        help="first consecutive PX4 instance; 0..6 for four, 0..8 for two",
    )
    result.add_argument(
        "--headless",
        action="store_true",
        help="disable both windows; requires --auto",
    )
    result.add_argument(
        "--auto",
        action="store_true",
        help="automatically take off, lift/translate cargo if fitted, hold, land, and exit",
    )
    result.add_argument(
        "--duration",
        type=float,
        default=12.0,
        help="automatic hover duration in wall seconds",
    )
    result.add_argument(
        "--timeout",
        type=float,
        default=80.0,
        help="overall automatic run timeout in wall seconds",
    )
    result.add_argument(
        "--check",
        action="store_true",
        help="offline model/config check, no PX4 or sockets",
    )
    result.add_argument(
        "--output-dir",
        type=Path,
        help="fresh run directory; existing directories are rejected",
    )
    return result


def run(arguments: argparse.Namespace) -> int:
    """Execute one owned multi-PX4 session and record an honest completion summary.

    Args:
        arguments: Validated CLI namespace.

    Returns:
        Zero for orderly exit, one for timeout/physics/communication failure.
    """
    settings = InteractionConfig(
        arguments.mode,
        arguments.horizontal,
        arguments.vertical,
        arguments.length,
    )
    if not all(
        math.isfinite(value) and value > 0
        for value in (arguments.duration, arguments.timeout)
    ):
        raise ValueError("duration and timeout must be finite and positive")
    if arguments.headless and not arguments.auto and not arguments.check:
        raise ValueError(
            "--headless requires --auto; use --check for an offline load check"
        )
    rope_length = arguments.rope_length
    if rope_length is None:
        rope_length = 1.6 if arguments.aircraft_count == 4 else 1.2
    payload_config = None if arguments.no_payload else PayloadConfig(
        mass_kg=arguments.payload_mass,
        rope=ElasticRopeConfig(rope_length, arguments.rope_stiffness,
                               arguments.rope_damping),
    )
    panel_hz = arguments.panel_hz
    if panel_hz is None:
        panel_hz = 1. if arguments.aircraft_count == 4 else 2.
    display_schedule = DisplaySchedule(panel_hz=panel_hz)
    app = DualSimulation(arguments.separation, arguments.height, arguments.instance_base,
                         payload_config=payload_config, aircraft_count=arguments.aircraft_count)
    app.interaction = settings
    if arguments.check:
        print(
            json.dumps(
                {
                    "aircraft": len(app.aircraft),
                    "mass_kg": [
                        item.bindings.mass_kg for item in app.aircraft
                    ],
                    "interaction": asdict(settings),
                    "timestep_s": app.model.opt.timestep,
                    "PX4_started": False,
                    "payload": asdict(payload_config) if payload_config else None,
                },
                indent=2,
            )
        )
        return 0
    if (
        not arguments.headless
        and os.name == "posix"
        and not os.environ.get("DISPLAY")
        and not os.environ.get("WAYLAND_DISPLAY")
    ):
        raise RuntimeError(
            "no graphical display; run --headless --auto or use a desktop terminal"
        )
    run_directory = arguments.output_dir or PROJECT_ROOT / "outputs/runs" / (
        datetime.now().strftime("%Y%m%d_%H%M%S_%f") + f"_{os.getpid()}"
    )
    run_directory = run_directory.resolve()
    run_directory.mkdir(parents=True, exist_ok=False)
    metadata = {
        "model": "artificial_distance_acceleration_v1",
        "physical_calibration": False,
        "initial_settings": asdict(settings),
        "instance_base": arguments.instance_base,
        "aircraft_count": arguments.aircraft_count,
        "interaction_combination": "sum_of_other_active_sources",
        "separation_m": arguments.separation,
        "takeoff_height_m": arguments.height,
        "mass_kg": [item.bindings.mass_kg for item in app.aircraft],
        "subtree_force_distribution": True,
        "disturbance_visualization": {
            "enabled_at_start": False,
            "control": "Start wind field / Stop wind field button",
            "illustrative_only": True,
            "style": "normalized_strength_surface_with_contours",
            "display_radius_m": 10.0,
            "radius_definition": "horizontal_graph_domain",
            "height_represents": "normalized_artificial_gain_not_potential",
        },
        "seed": list(range(arguments.instance_base + 1, arguments.instance_base + arguments.aircraft_count + 1)),
        "payload": asdict(payload_config) if payload_config else None,
        "rope_force_model": "tension_only_clipped_kelvin_voigt_at_live_sites",
        "rope_spatial_tendons_visual_only": True,
        "display_rates_hz": display_schedule.rates_hz if not arguments.headless else None,
        "loaded_position_ramp": {
            "speed_limit_m_s": .25,
            "acceleration_slew_m_s2": .1,
            "endpoint_clamped": True,
            "velocity_acceleration_feedforward": False,
        } if payload_config else None,
    }
    (run_directory / "metadata.json").write_text(
        json.dumps(metadata, indent=2)
    )
    print(f"Run output: {run_directory}", flush=True)
    print(
        "Artificial interaction: horizontal repulsion and downward acceleration; no aerodynamic calibration.",
        flush=True,
    )
    panel = None
    viewer = None
    visualization = None
    success = False
    error_text = ""
    samples: list[dict[str, Any]] = []
    completion = {"armed": False, "takeoff": False, "landed": False}
    transport = (
        TransportSequence(app.data.xpos[app.payload_system.body_id, :2])
        if app.payload_system else None
    )
    if transport:
        completion.update(transport.completion)
    last_sample = None
    start = time.perf_counter()
    phase = "waiting"
    hover_started = 0.0

    def request_shutdown(number: int, frame: Any) -> None:
        """Leave the loop safely even when Tk is dispatching a callback."""
        nonlocal error_text
        error_text = f"interrupted by signal {number}"
        app.command("exit")

    previous_handlers = {
        number: signal.signal(number, request_shutdown)
        for number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        with SitlPair(
            PROJECT_ROOT / "runtime/px4",
            run_directory / "px4",
            arguments.instance_base,
            aircraft_count=arguments.aircraft_count,
        ) as pair:
            app.connect()
            if not arguments.headless:
                from matplotlib import pyplot as plt
                from .panel import ControlPanel
                from .viewer import ManagedPassiveViewer
                from .disturbance_visualization import DisturbanceVisualization

                viewer = ManagedPassiveViewer(app.model, app.data)
                viewer.cam.distance = 8 if arguments.aircraft_count == 4 else 7
                viewer.cam.lookat[:] = [0, 0, 1.7]
                visualization = DisturbanceVisualization()
                visualization.set_enabled(False)

                def set_disturbance_visible(enabled: bool) -> None:
                    """Clear or redraw user geometry immediately on a UI toggle."""
                    with viewer.lock():
                        visualization.set_enabled(enabled)
                        visualization.draw(viewer.user_scn)
                    viewer.sync()
                    print(f"Strength surfaces visible: {enabled}", flush=True)

                panel = ControlPanel(app, set_disturbance_visible)
                plt.show(block=False)
            with (run_directory / "states.jsonl").open(
                "w", buffering=1
            ) as stream:
                iteration = 0
                while not app.exit_requested:
                    iteration_start = time.perf_counter()
                    elapsed = iteration_start - start
                    if any(
                        child.poll() is not None for child in pair.children
                    ):
                        raise RuntimeError(
                            "one of the owned PX4 instances exited; inspect per-aircraft px4/*/console.log"
                        )
                    if arguments.auto and elapsed > arguments.timeout:
                        raise RuntimeError(
                            f"automatic run timed out in phase {phase}"
                        )
                    lock = (
                        viewer.lock() if viewer is not None else nullcontext()
                    )
                    with lock:
                        sample = app.step()
                        sample["wall_elapsed_s"] = elapsed
                        last_sample = sample
                        if visualization is not None:
                            visualization.update(
                                sample["time_s"],
                                np.array([
                                    row["position"] for row in sample["aircraft"]
                                ]),
                                [
                                    row["disturbance_source_active"]
                                    for row in sample["aircraft"]
                                ],
                                length_m=app.interaction.length_m,
                            )
                    if iteration % 4 == 0:
                        stream.write(
                            json.dumps(sample, allow_nan=False) + "\n"
                        )
                    if arguments.auto:
                        if phase == "waiting" and all(
                            item.bridge.heartbeat_is_recent()
                            for item in app.aircraft
                        ):
                            print(
                                "All PX4 identities connected; arming.",
                                flush=True,
                            )
                            app.command("arm")
                            phase = "arming"
                        elif phase == "arming" and all(
                            item.bridge.armed for item in app.aircraft
                        ):
                            completion["armed"] = True
                            app.command("takeoff")
                            phase = "takeoff"
                            print("All armed; takeoff.", flush=True)
                        elif phase == "takeoff" and all(
                            item.phase == "hover" for item in app.aircraft
                        ):
                            completion["takeoff"] = True
                            hover_started = time.perf_counter()
                            phase = "lifting" if transport else "hover"
                            print(f"All at takeoff height; next phase: {phase}.",
                                  flush=True)
                        elif phase in ("lifting", "transferring"):
                            transition = transport.update(sample)
                            completion.update(transport.completion)
                            if transition == "move":
                                app.command("move", transport.offset)
                                phase = "transferring"
                                print("Cargo lifted; translating all aircraft "
                                      "+X 0.5 m.", flush=True)
                            elif transition == "hold":
                                phase = "hover"
                                hover_started = time.perf_counter()
                                print("Cargo translation measured and settled; "
                                      "holding before landing.", flush=True)
                        elif phase == "hover":
                            if transport:
                                transport.update(sample)
                            samples.append(sample)
                            if (
                                time.perf_counter() - hover_started
                                >= arguments.duration
                            ):
                                app.command("land")
                                phase = "landing"
                                if transport:
                                    transport.phase = "landing"
                                print("Landing all aircraft.", flush=True)
                        elif phase == "landing" and all(
                            item.phase == "landed" for item in app.aircraft
                        ):
                            completion["landed"] = True
                            done = transport.update(sample) == "done" if transport else True
                            if transport:
                                completion.update(transport.completion)
                            if done:
                                app.command("exit")
                                success = True
                                detail = " and cargo grounded" if transport else ""
                                print(f"All landed/disarmed{detail}; exiting.",
                                      flush=True)
                    if viewer is not None:
                        due = display_schedule.poll(time.perf_counter())
                        if due.field:
                            with viewer.lock():
                                visualization.draw(viewer.user_scn)
                        if due.viewer:
                            if not viewer.is_running():
                                app.command("exit")
                            viewer.sync()
                        if due.panel:
                            panel.update(sample)
                        if due.events:
                            panel.process_events()
                    iteration += 1
                    remaining = float(app.model.opt.timestep) - (
                        time.perf_counter() - iteration_start
                    )
                    if remaining > 0:
                        time.sleep(remaining)
                if not arguments.auto:
                    success = not error_text
    except KeyboardInterrupt:
        error_text = "interrupted by operator"
    except Exception as error:
        error_text = f"{type(error).__name__}: {error}"
        print(error_text, flush=True)
    finally:
        app.close()
        if panel is not None:
            panel.close()
        if viewer is not None:
            viewer.close()
        metrics = []
        for index in range(arguments.aircraft_count):
            if samples:
                positions = np.array(
                    [
                        sample["aircraft"][index]["position"]
                        for sample in samples
                    ]
                )
                targets = np.array(
                    [sample["aircraft"][index]["target"] for sample in samples]
                )
                accelerations = np.array(
                    [
                        sample["aircraft"][index]["interaction_acceleration"]
                        for sample in samples
                    ]
                )
                metrics.append(
                    {
                        "position_rmse_m": np.sqrt(
                            np.mean((positions - targets) ** 2, axis=0)
                        ).tolist(),
                        "mean_interaction_acceleration_m_s2": accelerations.mean(
                            axis=0
                        ).tolist(),
                    }
                )
        summary = {
            "success": success,
            "aircraft_count": arguments.aircraft_count,
            "error": error_text,
            "completion": completion,
            "initial_mode": settings.mode,
            "final_settings": asdict(app.interaction),
            "hover_samples": len(samples),
            "hover_metrics": metrics,
            "elapsed_wall_s": time.perf_counter() - start,
            "simulated_time_s": last_sample["time_s"] if last_sample else 0.,
            "realtime_factor": (
                last_sample["time_s"] / max(time.perf_counter() - start, 1e-9)
                if last_sample else 0.
            ),
            "transport_phase": transport.phase if transport else None,
            "final_payload": last_sample["payload"] if last_sample else None,
            "final_ropes": last_sample["ropes"] if last_sample else [],
            "payload_displacement_xy_m": (
                (np.array(last_sample["payload"]["position"][:2])
                 - transport.initial_xy).tolist()
                if transport and last_sample else None
            ),
            "owned_px4_stopped": (
                all(child.poll() is not None for child in pair.children)
                if "pair" in locals()
                else True
            ),
        }
        (run_directory / "summary.json").write_text(
            json.dumps(summary, indent=2)
        )
        print(json.dumps(summary), flush=True)
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
    return 0 if success else 1


def main(argv: Sequence[str] | None = None) -> int:
    """Validate arguments and launch without touching any original project state."""
    try:
        return run(parser().parse_args(argv))
    except (ValueError, OSError, RuntimeError) as error:
        print(f"Dual UAV startup error: {error}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
