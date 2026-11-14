"""Scalar-strength surface geometry and strict physics/display isolation."""

import mujoco
import numpy as np
import pytest

from dual_uav_interaction.disturbance_visualization import (
    DisturbanceVisualization,
    surface_height,
)
from test_simulation import airborne_app


POSITIONS = np.array([[-1.0, 0, 3], [1.0, 0, 3]])


def visual_scene(capacity: int = 12000) -> mujoco.MjvScene:
    """Allocate a real scene without GL, sockets or flight controllers."""
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    return mujoco.MjvScene(model, maxgeom=capacity)


def triangle_vertices(geom: mujoco.MjvGeom) -> np.ndarray:
    """Recover the three world vertices of MuJoCo's right-triangle primitive."""
    rotation = geom.mat.reshape(3, 3)
    return np.array([
        geom.pos,
        geom.pos + rotation[:, 0] * geom.size[0],
        geom.pos + rotation[:, 1] * geom.size[1],
    ])


def test_surface_height_encodes_gain_and_applied_length() -> None:
    np.testing.assert_allclose(
        surface_height(np.array([0.0, 2.0, 6.0]), 2.0),
        [-0.05, -0.725, -1.265],
    )
    values = surface_height(np.linspace(0, 10, 101), 2.0)
    assert np.all(np.diff(values) <= 0)
    assert surface_height(np.array([4.0]), 4.0)[0] == pytest.approx(-0.725)
    for length in (0, -1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            surface_height(np.array([1.0]), length)


def test_real_geoms_are_translucent_surfaces_and_contours_without_arrows() -> None:
    visual = DisturbanceVisualization()
    scene = visual_scene()
    visual.update(0, POSITIONS, [True, False])
    count = visual.draw(scene)
    assert 500 < count < 6000
    types = {scene.geoms[i].type for i in range(count)}
    assert types == {
        mujoco.mjtGeom.mjGEOM_TRIANGLE, mujoco.mjtGeom.mjGEOM_LINE
    }
    for index in range(count):
        geom = scene.geoms[index]
        assert 0 < geom.rgba[3] < 1
        assert geom.rgba[0] > geom.rgba[2]
        if geom.type == mujoco.mjtGeom.mjGEOM_TRIANGLE:
            vertices = triangle_vertices(geom)
            offsets = vertices - POSITIONS[0]
            assert np.all(offsets[:, 2] <= -0.0499)
            assert np.all(np.linalg.norm(offsets[:, :2], axis=1) <= 10.00001)
            np.testing.assert_allclose(
                geom.mat.reshape(3, 3).T @ geom.mat.reshape(3, 3),
                np.eye(3), atol=1e-6,
            )
            assert np.linalg.det(geom.mat.reshape(3, 3)) > 0.999
            assert geom.size[0] > 0 and geom.size[1] > 0


def test_surface_covers_center_and_full_azimuth_with_consistent_normals() -> None:
    visual = DisturbanceVisualization()
    scene = visual_scene()
    visual.update(0, POSITIONS, [True, False])
    visual.draw(scene)
    points = []
    projected_area = 0.0
    for index in range(scene.ngeom):
        geom = scene.geoms[index]
        if geom.type != mujoco.mjtGeom.mjGEOM_TRIANGLE:
            continue
        vertices = triangle_vertices(geom)
        points.extend(vertices)
        cross = np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0])
        assert cross[2] > 0
        projected_area += cross[2] / 2
    points = np.array(points) - POSITIONS[0]
    assert np.min(np.linalg.norm(points[:, :2], axis=1)) < 1e-5
    assert 310 < projected_area < 315  # Full disk, pi*10^2; no holes/overdraw.
    for axis in (0, 1):
        assert points[:, axis].min() < -9.9
        assert points[:, axis].max() > 9.9


@pytest.mark.parametrize("length_m", [0.1, 0.5, 1.0, 2.0, 8.0])
def test_contours_stay_above_rendered_mesh_for_applied_length(
    length_m: float,
) -> None:
    visual = DisturbanceVisualization()
    scene = visual_scene()
    visual.update(0, POSITIONS, [True, False], length_m=length_m)
    visual.draw(scene)
    faces = np.array([
        triangle_vertices(scene.geoms[i]) for i in range(scene.ngeom)
        if scene.geoms[i].type == mujoco.mjtGeom.mjGEOM_TRIANGLE
    ])
    first = faces[:, 0]
    edge1 = faces[:, 1] - first
    edge2 = faces[:, 2] - first
    determinant = edge1[:, 0] * edge2[:, 1] - edge1[:, 1] * edge2[:, 0]
    for index in range(scene.ngeom):
        geom = scene.geoms[index]
        if geom.type != mujoco.mjtGeom.mjGEOM_LINE:
            continue
        offset = geom.pos - first
        weight1 = (
            offset[:, 0] * edge2[:, 1] - offset[:, 1] * edge2[:, 0]
        ) / determinant
        weight2 = (
            edge1[:, 0] * offset[:, 1] - edge1[:, 1] * offset[:, 0]
        ) / determinant
        inside = (
            (weight1 >= -1e-5) & (weight2 >= -1e-5)
            & (weight1 + weight2 <= 1 + 1e-5)
        )
        assert inside.any()
        mesh_height = (
            first[:, 2] + weight1 * edge1[:, 2] + weight2 * edge2[:, 2]
        )[inside].max()
        assert geom.pos[2] > mesh_height + 0.005


def test_sources_follow_current_positions_and_fade_independently() -> None:
    visual = DisturbanceVisualization()
    scene = visual_scene()
    visual.update(0, POSITIONS, [True, True])
    both_count = visual.draw(scene)
    initial_position = scene.geoms[0].pos.copy()
    initial_alpha = scene.geoms[0].rgba[3]
    moved = POSITIONS + [20, 0, 0]
    visual.update(0.1, moved, [True, True])
    assert visual.draw(scene) == both_count
    np.testing.assert_allclose(scene.geoms[0].pos, initial_position + [20, 0, 0])
    visual.update(0.4, moved, [False, True])
    visual.draw(scene)
    assert scene.geoms[0].rgba[3] < initial_alpha
    visual.update(0.71, moved, [False, True])
    assert visual.draw(scene) == both_count // 2
    visual.update(2, moved, [False, False])
    assert visual.draw(scene) == 0


def test_length_edit_rebuilds_surface_and_capacity_preserves_physical_geoms() -> None:
    visual = DisturbanceVisualization()
    scene = visual_scene()
    visual.update(0, POSITIONS, [True, False], length_m=1.0)
    visual.draw(scene)
    old_heights = np.array([
        scene.geoms[i].pos[2] for i in range(scene.ngeom)
        if scene.geoms[i].type == mujoco.mjtGeom.mjGEOM_TRIANGLE
    ])
    visual.update(0.1, POSITIONS, [True, False], length_m=4.0)
    visual.draw(scene)
    new_heights = np.array([
        scene.geoms[i].pos[2] for i in range(scene.ngeom)
        if scene.geoms[i].type == mujoco.mjtGeom.mjGEOM_TRIANGLE
    ])
    assert np.all(new_heights >= old_heights - 1e-6)
    assert np.any(new_heights > old_heights + 0.1)
    small = visual_scene(7)
    small.ngeom = 3
    small.geoms[0].pos[:] = [123, 456, 789]
    assert visual.draw(small, append=True) == 4
    assert small.ngeom == 7
    np.testing.assert_array_equal(small.geoms[0].pos, [123, 456, 789])


def test_toggle_and_time_rewind_clear_fading_surfaces() -> None:
    visual = DisturbanceVisualization()
    scene = visual_scene()
    visual.update(5, POSITIONS, [True, True])
    assert visual.draw(scene) > 0
    visual.set_enabled(False)
    assert visual.draw(scene) == 0
    visual.update(5.1, POSITIONS, [True, True])
    assert visual.draw(scene) == 0
    visual.set_enabled(True)
    visual.update(5.2, POSITIONS, [False, False])
    assert visual.draw(scene) == 0
    visual.update(6, POSITIONS, [True, True])
    assert visual.draw(scene) > 0
    visual.update(0, POSITIONS, [False, False])
    assert visual.draw(scene) == 0


def test_visualization_leaves_physics_and_external_forces_identical() -> None:
    reference = airborne_app()
    displayed = airborne_app()
    reference.set_interaction("both", 0.5, 0.5, 2)
    displayed.set_interaction("both", 0.5, 0.5, 2)
    visual = DisturbanceVisualization()
    scene = visual_scene()
    for _ in range(20):
        expected = reference.step()
        sample = displayed.step()
        visual.update(
            sample["time_s"],
            np.array([row["position"] for row in sample["aircraft"]]),
            [True, True],
        )
        visual.draw(scene)
        assert sample == expected
        for attribute in ("qpos", "qvel", "ctrl", "xfrc_applied"):
            np.testing.assert_array_equal(
                getattr(displayed.data, attribute),
                getattr(reference.data, attribute),
            )


def test_owned_scene_refresh_reuses_geoms_and_matches_full_build(monkeypatch) -> None:
    cached = DisturbanceVisualization()
    scene = visual_scene()
    cached.update(0., POSITIONS, [True, True])
    cached.draw(scene)
    reference = DisturbanceVisualization()
    expected_scene = visual_scene()
    reference.update(0., POSITIONS, [True, True])
    # Translate both sources and fade just A, retaining the visible topology.
    moved = POSITIONS + [[.2, .1, .3], [-.3, .2, -.1]]
    cached.update(.2, moved, [False, True])
    reference.update(.2, moved, [False, True])
    reference.draw(expected_scene, append=True)

    def forbidden(*args, **kwargs) -> None:
        pytest.fail('cached refresh must not reinitialize static geometry')

    with monkeypatch.context() as patch:
        patch.setattr(mujoco, 'mjv_initGeom', forbidden)
        patch.setattr(mujoco, 'mjv_connector', forbidden)
        assert cached.draw(scene) == expected_scene.ngeom
    for index in range(scene.ngeom):
        actual, expected = scene.geoms[index], expected_scene.geoms[index]
        for attribute in ('pos', 'mat', 'size', 'rgba'):
            np.testing.assert_allclose(getattr(actual, attribute),
                                       getattr(expected, attribute), atol=2e-6)
        assert actual.type == expected.type
    # Explicit external clearing invalidates the owned-scene cache.
    scene.ngeom = 0
    assert cached.draw(scene) == expected_scene.ngeom
