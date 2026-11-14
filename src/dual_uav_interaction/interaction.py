"""Approved artificial distance-dependent interaction, not a calibrated wake."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np
import numpy.typing as npt

from .fleet import horizontal_distances

FloatArray = npt.NDArray[np.float64]
MODES = ("off", "horizontal", "vertical", "both")


@dataclass(frozen=True)
class InteractionConfig:
    """Immutable settings applied atomically by the simulation.

    Args:
        mode: off, horizontal, vertical, or both.
        horizontal_m_s2: Horizontal peak acceleration, nonnegative.
        vertical_m_s2: Downward peak acceleration, nonnegative.
        length_m: Positive half-amplitude horizontal separation.
    """

    mode: str = "off"
    horizontal_m_s2: float = 0.5
    vertical_m_s2: float = 0.5
    length_m: float = 2.0

    def __post_init__(self) -> None:
        """Reject invalid settings before they replace an active configuration."""
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        for name in ("horizontal_m_s2", "vertical_m_s2", "length_m"):
            value = float(getattr(self, name))
            if (
                not math.isfinite(value)
                or value < 0
                or (name == "length_m" and value == 0)
            ):
                raise ValueError(
                    f'{name} must be finite and {"positive" if name == "length_m" else "nonnegative"}'
                )
            object.__setattr__(self, name, value)


def compute_pair_accelerations(
    positions: Sequence[Sequence[float]],
    config: InteractionConfig,
    source_active: Sequence[bool] | None = None,
) -> FloatArray:
    """Sum the approved artificial law from every other active aircraft.

    The phenomenological g=1/(1+(d/L)^2) law is a simulation assumption,
    not a literature fit. Contributions add without averaging or saturation.

    Args:
        positions: Finite world positions, shape (N, 3), z upwards, N >= 2.
        config: Validated artificial-interaction settings.
        source_active: N source gates, or None to enable every source.

    Returns:
        World acceleration per receiver in m/s², shape (N, 3).

    Raises:
        ValueError: Invalid dimensions or undefined horizontal direction.
    """
    points = np.asarray(positions, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 2
            or not np.isfinite(points).all()):
        raise ValueError("positions must be finite with shape (N, 3), N >= 2")
    count = len(points)
    active = (np.ones(count, dtype=bool) if source_active is None
              else np.asarray(source_active, dtype=bool))
    if active.shape != (count,):
        raise ValueError("source_active must have shape (N,)")
    result = np.zeros((count, 3), dtype=np.float64)
    if config.mode == "off" or not active.any():
        return result
    # Axis 0 is the receiver and axis 1 the source; positive delta repels.
    distances = horizontal_distances(points)
    delta = points[:, None, :2] - points[None, :, :2]
    off_diagonal = ~np.eye(count, dtype=bool)
    gain = (config.length_m / np.hypot(distances, config.length_m)) ** 2
    gain *= off_diagonal & active[None, :]
    if config.mode in ("horizontal", "both") and config.horizontal_m_s2 > 0:
        if np.any((distances <= 1e-6) & off_diagonal & active[None, :]):
            raise ValueError("horizontal direction undefined at coincident XY positions")
        direction = np.divide(delta, distances[..., None], out=np.zeros_like(delta),
                              where=distances[..., None] > 1e-6)
        result[:, :2] = config.horizontal_m_s2 * np.sum(
            gain[..., None] * direction, axis=1
        )
    if config.mode in ("vertical", "both"):
        result[:, 2] = -config.vertical_m_s2 * gain.sum(axis=1)
    if not np.isfinite(result).all():
        raise FloatingPointError("non-finite accumulated interaction acceleration")
    return result
