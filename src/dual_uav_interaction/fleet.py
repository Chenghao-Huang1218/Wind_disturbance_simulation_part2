"""Shared fleet identities and initial geometry for two or four aircraft."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]
AIRCRAFT_LABELS = ("a", "b", "c", "d")
AIRCRAFT_COLORS = (
    (0.1, 0.5, 1., 1.), (1., .45, .1, 1.),
    (.15, .8, .4, 1.), (.65, .3, .9, 1.),
)


def validate_fleet(aircraft_count: int, instance_base: int = 0) -> None:
    """Validate supported fleet sizes and consecutive PX4 instance IDs.

    Args:
        aircraft_count: Either two or four independent aircraft.
        instance_base: First instance; every resulting index must be 0..9.

    Raises:
        ValueError: If dimensions or instance indices are unsupported.
    """
    if (isinstance(aircraft_count, bool)
            or not isinstance(aircraft_count, (int, np.integer))
            or aircraft_count not in (2, 4)):
        raise ValueError("aircraft_count must be 2 or 4")
    if (isinstance(instance_base, bool)
            or not isinstance(instance_base, (int, np.integer))
            or not 0 <= instance_base <= 10 - aircraft_count):
        raise ValueError(f"instance_base must be in 0..{10-aircraft_count}")


def initial_offsets(separation_m: float, aircraft_count: int) -> FloatArray:
    """Return XY translations for a pair or a same-heading square formation.

    Args:
        separation_m: Pair separation or square side length in metres.
        aircraft_count: Validated supported fleet size.

    Returns:
        Per-aircraft world XY frame translations, in A/B/C/D order.
    """
    validate_fleet(aircraft_count)
    if not np.isfinite(separation_m) or separation_m < 1.5:
        raise ValueError("initial separation must be finite and >=1.5 m")
    if aircraft_count == 2:
        return np.array([[0., -.5], [0., .5]]) * separation_m
    return np.array([[-.5, -.5], [-.5, .5], [.5, -.5], [.5, .5]]) * separation_m


def box_attachment_offsets(aircraft_count: int) -> FloatArray:
    """Return noncrossing local top attachment points for the shared box.

    Args:
        aircraft_count: Two or four aircraft, ordered as the formation.

    Returns:
        Local payload site coordinates in metres, shape (N, 3).
    """
    validate_fleet(aircraft_count)
    if aircraft_count == 2:
        return np.array([[0., -.12, .075], [0., .12, .075]])
    return np.array([[-.15, -.10, .075], [-.15, .10, .075],
                     [.15, -.10, .075], [.15, .10, .075]])


def horizontal_distances(positions: FloatArray) -> FloatArray:
    """Return finite all-pair XY distances without a diagonal self distance.

    Args:
        positions: Finite aircraft positions with XY in the first two columns.

    Returns:
        Symmetric (N, N) distances, with diagonal entries equal to zero.
    """
    points = np.asarray(positions, dtype=float)
    if (points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 2
            or not np.isfinite(points).all()):
        raise ValueError("positions must contain finite XY coordinates")
    with np.errstate(over="ignore"):
        delta = points[:, None, :2] - points[None, :, :2]
        distances = np.hypot(delta[..., 0], delta[..., 1])
    if not np.isfinite(distances).all():
        raise ValueError("horizontal separation must be finite")
    return distances
