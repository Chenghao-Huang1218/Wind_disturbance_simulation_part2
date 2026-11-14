"""Render illustrative airborne poses without PX4, network, or physics steps.

Example:
    MUJOCO_GL=egl python scripts/preview_disturbance.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from dual_uav_interaction.disturbance_visualization import (  # noqa: E402
    DisturbanceVisualization,
)
from dual_uav_interaction.scene import load_scene  # noqa: E402


def caption(image: Image.Image) -> Image.Image:
    """Add clear preview scope labels above and below the rendered scene.

    Args:
        image: Actual MuJoCo RGB rendering.

    Returns:
        Framed rendering labelled as a non-flight, illustrative preview.
    """
    framed = Image.new("RGB", (image.width, image.height + 76), "#151c26")
    framed.paste(image, (0, 38))
    draw = ImageDraw.Draw(framed)
    font = ImageFont.truetype("DejaVuSans.ttf", 16)
    draw.text(
        (16, 9), "Dual UAV | Strength surfaces + contours | 10 m horizontal radius",
        fill="white", font=font,
    )
    draw.text(
        (16, image.height + 48),
        "Height = normalized artificial strength. "
        "Pose-only preview; no PX4 or physics stepping.",
        fill="#d5dce6", font=font,
    )
    return framed


def render_preview(output_dir: Path) -> None:
    """Save surface PNGs and a camera-orbit GIF of the packaged models.

    Args:
        output_dir: Directory to create or reuse for the generated previews.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    model, data, bindings = load_scene()
    # Poses only: no stepping, simulated flight or live controller.
    for binding in bindings:
        data.qpos[binding.qpos + 2] += 2.0
    mujoco.mj_forward(model, data)
    positions = np.array([data.xpos[item.root_id].copy() for item in bindings])
    model.vis.global_.offwidth = 1100
    model.vis.global_.offheight = 750
    camera = mujoco.MjvCamera()
    camera.lookat[:] = positions.mean(axis=0) - [0, 0, 0.55]
    camera.azimuth = 125
    camera.elevation = -20
    camera.distance = 9
    visual = DisturbanceVisualization()
    frames: list[Image.Image] = []
    with mujoco.Renderer(model, height=750, width=1100) as renderer:
        for frame_index in range(24):
            time_s = frame_index / 10
            camera.azimuth = 125 + 12 * np.sin(2 * np.pi * frame_index / 24)
            visual.update(time_s, positions, [True, True])
            renderer.update_scene(data, camera=camera)
            visual.draw(renderer.scene, append=True)
            frame = caption(Image.fromarray(renderer.render()))
            frames.append(frame)
            if frame_index == 6:
                frame.save(output_dir / "disturbance-surface-close.png")
        camera.distance = 26
        renderer.update_scene(data, camera=camera)
        count = visual.draw(renderer.scene, append=True)
        caption(Image.fromarray(renderer.render())).save(
            output_dir / "disturbance-surface-overview.png"
        )
    frames[0].save(
        output_dir / "disturbance-surface.gif",
        save_all=True, append_images=frames[1:], duration=100, loop=0,
    )
    print(f"Saved previews to {output_dir}; last frame visual geoms: {count}")


def main() -> None:
    """Parse preview destination without starting simulation processes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=PROJECT_ROOT / "docs/media"
    )
    arguments = parser.parse_args()
    render_preview(arguments.output_dir.resolve())


if __name__ == "__main__":
    main()
