"""Approved artificial distance-dependent interaction, not a calibrated wake."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np
import numpy.typing as npt

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
    source_active: Sequence[bool] = (True, True),
) -> FloatArray:
    """Evaluate both directions from a single world-frame state snapshot.

    The user-selected phenomenological law is g=1/(1+(d/L)^2).
    Horizontal components push the aircraft apart; both vertical components
    point down. This formula is a simulation assumption, not a literature fit.

    Args:
        positions: Two finite world positions, shape (2, 3), z upwards.
        config: Validated artificial-interaction settings.
        source_active: Whether each aircraft currently generates disturbance.

    Returns:
        World-frame acceleration on A and B in m/s², shape (2, 3).

    Raises:
        ValueError: For invalid states or an undefined horizontal direction.
    """
    points = np.asarray(positions, dtype=np.float64)
    if points.shape != (2, 3) or not np.isfinite(points).all():
        raise ValueError("positions must be finite with shape (2, 3)")
    active = np.asarray(source_active, dtype=bool)
    if active.shape != (2,):
        raise ValueError("source_active must have shape (2,)")
    result = np.zeros((2, 3), dtype=np.float64)
    if config.mode == "off" or not active.any():
        return result
    delta = points[1, :2] - points[0, :2]
    distance = math.hypot(*delta)
    if not math.isfinite(distance):
        raise ValueError("horizontal separation must be finite")
    gain = (config.length_m / math.hypot(distance, config.length_m)) ** 2
    if config.mode in ("horizontal", "both") and config.horizontal_m_s2 > 0:
        if distance <= 1e-6:
            raise ValueError(
                "horizontal direction undefined at coincident XY positions"
            )
        direction = delta / distance
        result[0, :2] = -config.horizontal_m_s2 * gain * direction
        result[1, :2] = -result[0, :2]
    if config.mode in ("vertical", "both"):
        result[:, 2] = -config.vertical_m_s2 * gain
    # The source for A is B, and the source for B is A.
    result[0] *= active[1]
    result[1] *= active[0]
    return result
