"""Compare identical native display work before/after without PX4 or sockets."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
from typing import Any

from matplotlib import pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "tests")]

from dual_uav_interaction.panel import ControlPanel  # noqa: E402
from dual_uav_interaction.disturbance_visualization import DisturbanceVisualization  # noqa: E402
from dual_uav_interaction.payload import PayloadConfig  # noqa: E402
from dual_uav_interaction.simulation import DualSimulation  # noqa: E402
from dual_uav_interaction.viewer import ManagedPassiveViewer  # noqa: E402
from test_simulation import Bridge  # noqa: E402


def baseline_class(directory: Path, module: str, name: str) -> Any:
    """Load a read-only backed-up project module with working relative imports."""
    module_name = "dual_uav_interaction._baseline_" + module
    spec = importlib.util.spec_from_file_location(module_name, directory / (module + ".py"))
    loaded = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = loaded
    spec.loader.exec_module(loaded)
    return getattr(loaded, name)


def measure(panel_type: Any, visual_type: Any) -> dict[str, Any]:
    """Measure matched per-call native costs using socket-free bridge doubles."""
    app = DualSimulation(bridges=[Bridge(), Bridge()], payload_config=PayloadConfig())
    for aircraft in app.aircraft:
        app.data.qpos[aircraft.bindings.qpos + 2] += 2
        aircraft.phase = "hover"
        aircraft.target[2] += 2
    viewer = ManagedPassiveViewer(app.model, app.data)
    visual = visual_type()
    panel = panel_type(app, lambda enabled: visual.set_enabled(enabled))
    result = {}
    try:
        plt.show(block=False)
        panel.process_events()
        for enabled in (False, True):
            visual.set_enabled(enabled)
            timing = {key: [] for key in (
                "physics", "visual_update", "visual_draw", "viewer_sync",
                "panel_update", "panel_flush",
            )}
            for _ in range(20):
                start = time.perf_counter()
                with viewer.lock():
                    sample = app.step()
                timing["physics"].append(time.perf_counter() - start)
                start = time.perf_counter()
                visual.update(sample["time_s"], np.array([
                    row["position"] for row in sample["aircraft"]
                ]), [True, True])
                timing["visual_update"].append(time.perf_counter() - start)
                start = time.perf_counter()
                with viewer.lock():
                    count = visual.draw(viewer.user_scn)
                timing["visual_draw"].append(time.perf_counter() - start)
                start = time.perf_counter()
                viewer.sync()
                timing["viewer_sync"].append(time.perf_counter() - start)
                start = time.perf_counter()
                panel.update(sample)
                timing["panel_update"].append(time.perf_counter() - start)
                start = time.perf_counter()
                panel.process_events()
                timing["panel_flush"].append(time.perf_counter() - start)
            result["field_on" if enabled else "field_off"] = {
                "median_ms": {key: float(np.median(values) * 1000)
                              for key, values in timing.items()},
                "p95_ms": {key: float(np.percentile(values, 95) * 1000)
                           for key, values in timing.items()},
                "field_geoms": count,
            }
    finally:
        app.close()
        panel.close()
        viewer.close()
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    before = measure(baseline_class(args.baseline_dir, "panel", "ControlPanel"),
                     baseline_class(args.baseline_dir, "disturbance_visualization",
                                    "DisturbanceVisualization"))
    after = measure(ControlPanel, DisturbanceVisualization)
    report = {"backend": plt.get_backend(), "PX4_started": False,
              "protocol": "20 matched calls per field state, native MuJoCo and Tk",
              "before": before, "after": after}
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
