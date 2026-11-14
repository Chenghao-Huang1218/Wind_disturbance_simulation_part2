"""Behavioral checks for the approved artificial interaction model."""

import numpy as np
import pytest

from dual_uav_interaction.interaction import (
    InteractionConfig,
    compute_pair_accelerations,
)


@pytest.mark.parametrize(
    "mode,expected",
    [
        ("off", [[0, 0, 0], [0, 0, 0]]),
        ("horizontal", [[-0.5, 0, 0], [0.5, 0, 0]]),
        ("vertical", [[0, 0, -1], [0, 0, -1]]),
        ("both", [[-0.5, 0, -1], [0.5, 0, -1]]),
    ],
)
def test_four_modes(mode: str, expected: list[list[float]]) -> None:
    positions = np.array([[-1.0, 0, 2], [1.0, 0, 2]])
    result = compute_pair_accelerations(
        positions, InteractionConfig(mode, 1, 2, 2)
    )
    np.testing.assert_allclose(result, expected, atol=1e-15)


def test_rotation_exchange_source_and_decay() -> None:
    config = InteractionConfig("both", 1, 2, 2)
    positions = np.array([[0.0, 0, 2], [0, 2, 2]])
    result = compute_pair_accelerations(positions, config)
    np.testing.assert_allclose(result[0], [0, -0.5, -1])
    np.testing.assert_allclose(
        compute_pair_accelerations(positions[::-1], config), result[::-1]
    )
    inactive = compute_pair_accelerations(positions, config, (False, True))
    np.testing.assert_allclose(inactive[0], result[0])
    np.testing.assert_array_equal(inactive[1], 0)
    farther = compute_pair_accelerations(positions * [1, 2, 1], config)
    assert np.linalg.norm(farther) < np.linalg.norm(result)


@pytest.mark.parametrize(
    "args",
    [
        ("wrong", 1, 1, 2),
        ("off", -1, 1, 2),
        ("both", 1, np.nan, 2),
        ("vertical", 1, 1, 0),
    ],
)
def test_invalid_config(args: tuple) -> None:
    with pytest.raises(ValueError):
        InteractionConfig(*args)


def test_coincident_and_nonfinite() -> None:
    points = np.zeros((2, 3))
    np.testing.assert_array_equal(
        compute_pair_accelerations(points, InteractionConfig()), 0
    )
    assert np.isfinite(
        compute_pair_accelerations(points, InteractionConfig("vertical"))
    ).all()
    with pytest.raises(ValueError, match="horizontal"):
        compute_pair_accelerations(points, InteractionConfig("horizontal"))
    with pytest.raises(ValueError):
        compute_pair_accelerations(
            np.full((2, 3), np.nan), InteractionConfig()
        )
