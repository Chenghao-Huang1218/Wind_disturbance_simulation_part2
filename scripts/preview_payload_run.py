"""Render logged SITL body/arm poses; no PX4 or physics stepping.

Usage: MUJOCO_GL=egl python scripts/preview_payload_run.py RUN_DIRECTORY
Rotating propeller phase is not logged; body, arm and cargo poses are replayed.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dual_uav_interaction.payload import ElasticRopeConfig, PayloadConfig  # noqa: E402
from dual_uav_interaction.scene import load_scene  # noqa: E402


def render_run(run_directory: Path, output_directory: Path) -> None:
    """Reconstruct measured poses and save a still, GIF and panel preview.

    Args:
        run_directory: Actual run containing metadata.json and states.jsonl.
        output_directory: Destination for the image artifacts.
    """
    metadata = json.loads((run_directory / "metadata.json").read_text())
    settings = metadata["payload"]
    config = PayloadConfig(settings["mass_kg"], ElasticRopeConfig(**settings["rope"]))
    model, data, bindings = load_scene(metadata["separation_m"], config, metadata.get("aircraft_count", 2))
    samples = [json.loads(line) for line in (run_directory / "states.jsonl").open()]
    output_directory.mkdir(parents=True, exist_ok=True)
    model.vis.global_.offwidth = 1000
    model.vis.global_.offheight = 700
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.35, 0, 1.35]
    camera.distance = 6.2
    camera.azimuth = 145
    camera.elevation = -17
    cargo_joint = model.joint("payload_free").qposadr[0]
    font = ImageFont.truetype("DejaVuSans.ttf", 15)
    frames = []
    selected_indices = np.linspace(0, len(samples) - 1, 100).astype(int)
    airborne = [i for i, s in enumerate(samples) if s["payload"]["bottom_clearance_m"] > .3]
    still_index = max(airborne, key=lambda i: samples[i]["payload"]["position"][0])
    with mujoco.Renderer(model, height=700, width=1000) as renderer:
        for index in sorted(set(selected_indices.tolist() + [still_index])):
            sample = samples[index]
            for binding, row in zip(bindings, sample["aircraft"]):
                data.qpos[binding.qpos:binding.qpos + 3] = row["position"]
                data.qpos[binding.qpos + 3:binding.qpos + 7] = row["quaternion_wxyz"]
                data.qpos[binding.arm_qpos] = row["arm_deviation_rad"]
            cargo = sample["payload"]
            data.qpos[cargo_joint:cargo_joint + 3] = cargo["position"]
            data.qpos[cargo_joint + 3:cargo_joint + 7] = cargo["quaternion_wxyz"]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            image = Image.fromarray(renderer.render())
            framed = Image.new("RGB", (1000, 760), "#162232")
            framed.paste(image, (0, 30))
            draw = ImageDraw.Draw(framed)
            tension = [round(r["tension_n"], 2) for r in sample["ropes"]]
            draw.text((12, 7), f"SITL pose replay | t={sample['time_s']:.2f} s | mode={metadata['initial_settings']['mode']}", font=font)
            draw.text((12, 737), f"Cargo bottom={cargo['bottom_clearance_m']:.2f} m | tension={tension} N | recorded poses, time compressed", font=font)
            if index == still_index:
                framed.save(output_directory / "payload-carry-sitl.png")
            if index in selected_indices:
                frames.append(framed)
    frames[0].save(output_directory / "payload-carry-sitl.gif", save_all=True,
                   append_images=frames[1:], duration=100, loop=0)
    print(f"Rendered {len(frames)} recorded pose frames from {run_directory}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "docs/media")
    arguments = parser.parse_args()
    render_run(arguments.run_directory.resolve(), arguments.output_dir.resolve())
