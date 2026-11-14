"""Display-only scalar-strength surfaces; not potential energy or wind speed."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import mujoco
import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]
DISPLAY_RADIUS_M = 10.0  # Horizontal graph domain, matching distance in g(r).
# Packaged X4 blades start about .066 m above root. Keep the graph peak
# .05 m below root (about .116 m below the level blades), rather than .65 m.
SURFACE_OFFSET_M = 0.05
SURFACE_DEPTH_M = 1.35
FADE_OUT_S = 0.6
RADIAL_INTERVALS = 12
AZIMUTH_SECTORS = 48
SOURCE_COLORS = np.array([[0.98, 0.30, 0.12], [1.0, 0.60, 0.16],
                          [0.88, 0.25, 0.65], [0.95, 0.80, 0.20]])


def surface_height(radius_m: FloatArray, length_m: float) -> FloatArray:
    """Map normalized artificial strength to a height below the source plane.

    This is a graph of g(r)=1/(1+(r/L)^2), not a physical flow boundary.
    The offset and depth are layout settings with no aerodynamic calibration.

    Args:
        radius_m: Finite nonnegative horizontal distance(s) from the source.
        length_m: Finite positive half-strength distance L from the force model.

    Returns:
        Graph height relative to the source, in visual geometry metres.

    Raises:
        ValueError: If a distance or length is invalid.
    """
    distances = np.asarray(radius_m, dtype=np.float64)
    if not math.isfinite(length_m) or length_m <= 0:
        raise ValueError("surface length must be finite and positive")
    if not np.isfinite(distances).all() or np.any(distances < 0):
        raise ValueError("surface radii must be finite and nonnegative")
    gain = (length_m / np.hypot(distances, length_m)) ** 2
    return -SURFACE_OFFSET_M - SURFACE_DEPTH_M * (1 - gain)


@dataclass(frozen=True)
class SurfaceGeometry:
    """Cached local right-triangle frames and contour segments for one source."""

    origins: FloatArray
    sizes: FloatArray
    rotations: FloatArray
    opacity: FloatArray
    contours: FloatArray
    contour_opacity: FloatArray


@dataclass
class SourceDrawCache:
    """Static geom references and local positions for one native source graph."""

    index: int
    geoms: tuple[mujoco.MjvGeom, ...]
    local_positions: FloatArray
    base_alpha: FloatArray
    position: FloatArray
    opacity: float


def _edge_fade(radius_m: FloatArray) -> FloatArray:
    """Fade smoothly over the last two metres of the horizontal graph domain."""
    fraction = np.clip((DISPLAY_RADIUS_M - radius_m) / 2, 0, 1)
    return fraction**2 * (3 - 2 * fraction)


def _make_geometry(length_m: float) -> SurfaceGeometry:
    """Build a polar height graph using valid MuJoCo right-triangle primitives.

    Args:
        length_m: Validated half-strength distance controlling graph width.

    Returns:
        Local surface frames and closed contour segments, without world state.
    """
    radii = DISPLAY_RADIUS_M * np.linspace(0, 1, RADIAL_INTERVALS + 1) ** 1.5
    angles = np.linspace(0, 2 * math.pi, AZIMUTH_SECTORS + 1)
    vertices = np.empty((len(radii), len(angles), 3))
    vertices[:, :, 0] = radii[:, None] * np.cos(angles)
    vertices[:, :, 1] = radii[:, None] * np.sin(angles)
    vertices[:, :, 2] = surface_height(radii, length_m)[:, None]
    corners = [
        vertices[:-1, :-1].reshape(-1, 3),
        vertices[1:, :-1].reshape(-1, 3),
        vertices[1:, 1:].reshape(-1, 3),
        vertices[:-1, 1:].reshape(-1, 3),
    ]
    faces = np.concatenate([
        np.stack((corners[0], corners[1], corners[2]), axis=1),
        np.stack((corners[0], corners[2], corners[3]), axis=1),
    ])
    # mjGEOM_TRIANGLE is a RIGHT triangle with vertices (0,0,0),
    # (size[0],0,0), (0,size[1],0) in its orthonormal local frame.
    # Split arbitrary mesh triangles at the altitude to the longest edge;
    # its foot lies inside that edge, so both right triangles cover the face.
    edges = np.roll(faces, -1, axis=1) - faces
    longest = np.argmax(np.linalg.norm(edges, axis=2), axis=1)
    rows = np.arange(len(faces))
    first = faces[rows, longest]
    second = faces[rows, (longest + 1) % 3]
    third = faces[rows, (longest + 2) % 3]
    base = second - first
    base_lengths = np.linalg.norm(base, axis=1)
    base_units = np.divide(
        base, base_lengths[:, None], out=np.zeros_like(base),
        where=base_lengths[:, None] > 1e-12,
    )
    projection = np.sum((third - first) * base_units, axis=1)
    foot = first + projection[:, None] * base_units
    triangles = np.concatenate([
        np.stack((foot, second, third), axis=1),
        np.stack((foot, third, first), axis=1),
    ])
    first_leg = triangles[:, 1] - triangles[:, 0]
    second_leg = triangles[:, 2] - triangles[:, 0]
    first_length = np.linalg.norm(first_leg, axis=1)
    second_length = np.linalg.norm(second_leg, axis=1)
    valid = (first_length > 1e-9) & (second_length > 1e-9)
    triangles = triangles[valid]
    first_axis = first_leg[valid] / first_length[valid, None]
    second_axis = second_leg[valid] / second_length[valid, None]
    normal = np.cross(first_axis, second_axis)
    rotations = np.stack((first_axis, second_axis, normal), axis=2)
    sizes = np.column_stack([
        first_length[valid], second_length[valid], np.zeros(valid.sum())
    ])
    distances = np.linalg.norm(triangles.mean(axis=1)[:, :2], axis=1)
    gain = (length_m / np.hypot(distances, length_m)) ** 2
    opacity = 0.30 * (0.3 + 0.7 * np.sqrt(gain)) * _edge_fade(distances)

    levels = np.array([0.9, 0.75, 0.55, 0.35, 0.2, 0.1, 0.05])
    factors = np.sqrt(1 / levels - 1)
    # Filter before multiplying to keep even very large finite L bounded.
    factors = factors[factors < DISPLAY_RADIUS_M / length_m]
    contour_radii = factors * length_m
    rings = np.empty((len(contour_radii), len(angles), 3))
    rings[:, :, 0] = contour_radii[:, None] * np.cos(angles)
    rings[:, :, 1] = contour_radii[:, None] * np.sin(angles)
    # Contours must follow the actual piecewise-linear graph, rather than
    # the analytic height between radial knots, which can be below the mesh.
    # They share its azimuth sectors, so radial interpolation matches the
    # rendered trapezoid planes; the lift then reliably avoids z-fighting.
    contour_heights = np.interp(
        contour_radii, radii, surface_height(radii, length_m)
    )
    rings[:, :, 2] = contour_heights[:, None] + 0.012
    segments = np.stack(
        (rings[:, :-1], rings[:, 1:]), axis=2
    ).reshape(-1, 2, 3)
    contour_opacity = np.repeat(
        0.48 * _edge_fade(contour_radii), AZIMUTH_SECTORS
    )
    return SurfaceGeometry(
        triangles[:, 0], sizes, rotations.reshape(-1, 9), opacity,
        segments, contour_opacity,
    )


class DisturbanceVisualization:
    """Draw independent strength graphs for each aircraft without touching physics.

    The display consumes same-time world positions, source gates and applied L.
    It uses no model/data reference, random sampling or physical time stepping.
    The surfaces follow current source positions; source shutdown fades them.
    """

    def __init__(self) -> None:
        """Initialize hidden sources with a lazily generated local mesh."""
        self.enabled = True
        self._time_s: float | None = None
        self._positions = np.zeros((2, 3))
        self._last_active = np.full(2, -np.inf)
        self._opacity = np.zeros(2)
        self._length_m = 2.0
        self._mesh_length_m: float | None = None
        self._geometry: SurfaceGeometry | None = None
        self._draw_scene: mujoco.MjvScene | None = None
        self._draw_key: tuple[float, tuple[bool, ...], int] | None = None
        self._draw_count = 0
        self._source_draws: list[SourceDrawCache] = []

    def _clear(self) -> None:
        """Drop visible/fading source state after a toggle or time rewind."""
        self._last_active[:] = -np.inf
        self._opacity[:] = 0
        self._draw_key = None

    def set_enabled(self, enabled: bool) -> None:
        """Set visibility and clear old surfaces on any visibility transition.

        Args:
            enabled: Whether source updates may make strength graphs visible.
        """
        if bool(enabled) != self.enabled:
            self._clear()
        self.enabled = bool(enabled)

    def update(
        self,
        time_s: float,
        positions: FloatArray,
        source_active: Sequence[bool],
        length_m: float = 2.0,
    ) -> None:
        """Update current source positions and simulation-time shutdown fading.

        Args:
            time_s: Finite nonnegative simulation time in seconds.
            positions: Finite world root positions, shape (N, 3), metres; N=2 or 4.
            source_active: N source gates, independent of interaction mode.
            length_m: Applied L controlling the normalized strength graph.

        Raises:
            ValueError: If time, positions, source dimensions or L are invalid.
        """
        points = np.asarray(positions, dtype=np.float64)
        active = np.asarray(source_active, dtype=bool)
        if not math.isfinite(time_s) or time_s < 0:
            raise ValueError("visual time must be finite and nonnegative")
        if (points.ndim != 2 or points.shape[1] != 3
                or len(points) not in (2, 4) or not np.isfinite(points).all()):
            raise ValueError("visual positions must be finite with shape (N, 3), N=2 or 4")
        if active.shape != (len(points),):
            raise ValueError("visual source gates must have shape (N,)")
        if not math.isfinite(length_m) or length_m <= 0:
            raise ValueError("surface length must be finite and positive")
        if len(points) != len(self._last_active):
            self._clear()
            self._last_active = np.full(len(points), -np.inf)
            self._opacity = np.zeros(len(points))
        if self._time_s is not None and time_s < self._time_s:
            self._clear()
        self._time_s = float(time_s)
        self._positions = points.copy()
        self._length_m = float(length_m)
        if not self.enabled:
            return
        self._last_active[active] = time_s
        self._opacity = np.clip(
            1 - (time_s - self._last_active) / FADE_OUT_S, 0, 1
        )

    def draw(self, scene: mujoco.MjvScene, *, append: bool = False) -> int:
        """Reuse static geometry only in an exclusively owned native user scene.

        Args:
            scene: Native user scene, or offscreen physical scene when appending.
            append: Preserve physical prefix; append always uses a full build.

        Returns:
            Number of surface/contour geoms drawn, respecting scene capacity.
        """
        if append:
            self._draw_key = None
            return self._draw_full(scene, append=True)
        active = tuple(bool(value) for value in self._opacity > 0)
        key = (self._length_m, active, scene.maxgeom)
        if (self.enabled and self._draw_scene is scene and self._draw_key == key
                and scene.ngeom == self._draw_count):
            for cache in self._source_draws:
                position = self._positions[cache.index]
                opacity = float(self._opacity[cache.index])
                if not np.array_equal(position, cache.position):
                    origins = cache.local_positions + position
                    for geom, origin in zip(cache.geoms, origins):
                        geom.pos[:] = origin
                    cache.position[:] = position
                if opacity != cache.opacity:
                    for geom, alpha in zip(cache.geoms, cache.base_alpha * opacity):
                        geom.rgba[3] = alpha
                    cache.opacity = opacity
            return self._draw_count
        count = self._draw_full(scene)
        self._draw_scene = scene
        self._draw_key = key if self.enabled and count else None
        self._draw_count = count
        self._source_draws = []
        if count and self._geometry is not None:
            alpha = np.concatenate((self._geometry.opacity,
                                    self._geometry.contour_opacity))
            cursor = 0
            for index in np.flatnonzero(active):
                size = min(len(alpha), count - cursor)
                if size <= 0:
                    break
                geoms = tuple(scene.geoms[cursor:cursor + size])
                position = self._positions[index].copy()
                local_positions = np.array([g.pos.copy() for g in geoms]) - position
                self._source_draws.append(SourceDrawCache(
                    int(index), geoms, local_positions, alpha[:size],
                    position, float(self._opacity[index]),
                ))
                cursor += size
        return count

    def _draw_full(self, scene: mujoco.MjvScene, *, append: bool = False) -> int:
        """Write bounded translucent triangles and contour lines under viewer lock.

        Args:
            scene: Owned user scene, or an offscreen scene with physical geoms.
            append: Preserve existing physical geoms instead of replacing scene.

        Returns:
            Number of display geoms added, bounded by the available capacity.
        """
        if not append:
            scene.ngeom = 0
        start_count = scene.ngeom
        if not self.enabled or not np.any(self._opacity > 0):
            return 0
        if self._geometry is None or self._mesh_length_m != self._length_m:
            self._geometry = _make_geometry(self._length_m)
            self._mesh_length_m = self._length_m
        mesh = self._geometry
        for index, visibility in enumerate(self._opacity):
            if visibility <= 0:
                continue
            center = self._positions[index]
            color = SOURCE_COLORS[index]
            # Vectorize per-frame translations/colors instead of allocating
            # several tiny arrays for every rendered triangle.
            origins = mesh.origins + center
            colors = np.empty((len(mesh.origins), 4), dtype=np.float32)
            colors[:, :3] = color
            colors[:, 3] = mesh.opacity * visibility
            for origin, size, rotation, rgba in zip(
                origins, mesh.sizes, mesh.rotations, colors
            ):
                if scene.ngeom >= scene.maxgeom:
                    return scene.ngeom - start_count
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(
                    geom, mujoco.mjtGeom.mjGEOM_TRIANGLE, size, origin,
                    rotation, rgba,
                )
                geom.category = mujoco.mjtCatBit.mjCAT_DECOR
                geom.segid = -1
                geom.specular = 0.1
                geom.shininess = 0.2
                scene.ngeom += 1
            segments = mesh.contours + center
            colors = np.empty((len(segments), 4), dtype=np.float32)
            colors[:, :3] = color * 0.45
            colors[:, 3] = mesh.contour_opacity * visibility
            zero = np.zeros(3)
            identity = np.eye(3).ravel()
            for segment, rgba in zip(segments, colors):
                if scene.ngeom >= scene.maxgeom:
                    return scene.ngeom - start_count
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(
                    geom, mujoco.mjtGeom.mjGEOM_LINE, zero, zero,
                    identity, rgba,
                )
                mujoco.mjv_connector(
                    geom, mujoco.mjtGeom.mjGEOM_LINE, 1.4,
                    segment[0], segment[1],
                )
                geom.category = mujoco.mjtCatBit.mjCAT_DECOR
                geom.segid = -1
                scene.ngeom += 1
        return scene.ngeom - start_count
