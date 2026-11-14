"""Shared-world application tests use real physics and socket-free bridges."""

import time
from typing import Any

import mujoco
import numpy as np
import pytest

from dual_uav_interaction.interaction import InteractionConfig
from dual_uav_interaction.simulation import DualSimulation


class Bridge:
    def __init__(self) -> None:
        self.armed = True
        self.offboard = True
        self.last_control_received_wall_time = 1
        self.last_arm_ack_result = 0
        self.last_statustext = "test"
        self.commands = []

    def __getattr__(self, name: str) -> Any:
        if name == "receive_rotor_thrusts":
            return lambda: np.full(4, 20.0)
        if name == "heartbeat_is_recent":
            return lambda: True
        if name == "control_is_stale":
            return lambda _: False
        return lambda *args, **kwargs: None


def airborne_app() -> DualSimulation:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    for item in app.aircraft:
        app.data.qpos[item.bindings.qpos + 2] += 2
        item.phase = "hover"
    mujoco.mj_forward(app.model, app.data)
    return app


def test_shared_step_force_switch_and_atomic_edit() -> None:
    app = airborne_app()
    app.set_interaction("both", 1, 2, 2)
    previous = app.data.time
    sample = app.step()
    assert app.data.time == pytest.approx(previous + app.model.opt.timestep)
    np.testing.assert_allclose(
        sample["aircraft"][0]["interaction_acceleration"], [0, -0.5, -1]
    )
    np.testing.assert_allclose(
        sample["aircraft"][1]["interaction_acceleration"], [0, 0.5, -1]
    )
    old = app.interaction
    with pytest.raises(ValueError):
        app.set_interaction("both", -1, 2, 2)
    assert app.interaction is old
    app.set_interaction("off", 1, 2, 2)
    app.step()
    np.testing.assert_array_equal(app.data.xfrc_applied, 0)


def test_grounded_receiver_and_inactive_source() -> None:
    app = airborne_app()
    app.set_interaction("horizontal", 1, 1, 2)
    app.aircraft[0].bridge.armed = False
    app.aircraft[0].phase = "landed"
    app.data.qpos[app.bindings[0].qpos + 2] = app.aircraft[0].home[2]
    app.step()
    np.testing.assert_array_equal(app.last_accelerations[1], 0)
    np.testing.assert_array_equal(app.last_accelerations[0], 0)


def test_visual_source_gate_is_independent_of_force_mode() -> None:
    app = airborne_app()
    sample = app.step()  # Default force mode is off; both sources still emit.
    assert [row["disturbance_source_active"] for row in sample["aircraft"]] == [
        True, True
    ]
    np.testing.assert_array_equal(app.last_accelerations, 0)
    app.aircraft[0].bridge.armed = False
    app.aircraft[0].phase = "emergency"
    sample = app.step()
    assert [row["disturbance_source_active"] for row in sample["aircraft"]] == [
        False, True
    ]
    grounded = DualSimulation(bridges=[Bridge(), Bridge()])
    sample = grounded.step()
    assert not any(row["disturbance_source_active"] for row in sample["aircraft"])


def test_takeoff_requires_both_armed() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    app.aircraft[1].bridge.armed = False
    with pytest.raises(RuntimeError, match="armed"):
        app.command("takeoff")


def test_takeoff_preserves_spacing_and_rejects_repeated_airborne_command() -> (
    None
):
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    app.command("separation", 0.5)
    expected_xy = np.array([item.target[:2].copy() for item in app.aircraft])
    app.command("takeoff")
    np.testing.assert_allclose(
        [item.target[:2] for item in app.aircraft], expected_xy
    )
    before = np.array([item.target.copy() for item in app.aircraft])
    with pytest.raises(RuntimeError, match="ground"):
        app.command("takeoff")
    np.testing.assert_array_equal(
        [item.target for item in app.aircraft], before
    )


def test_disarm_above_ground_during_landing_is_failure() -> None:
    app = airborne_app()
    app.command("land")
    app.aircraft[0].bridge.armed = False
    with pytest.raises(RuntimeError, match="disarm"):
        app.step()
    assert app.aircraft[0].phase != "landed"


def test_landing_disarm_requires_ground_and_settling_confirmation() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    item = app.aircraft[0]
    item.phase = "landing"
    item.bridge.armed = False
    with pytest.raises(RuntimeError, match="disarm"):
        app.step()
    item.landing_disarm_requested = True
    app.step()
    assert item.phase == "landed"


def test_landing_disarm_ack_uses_latched_settling_confirmation() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    item = app.aircraft[0]
    item.phase = "landing"
    item.landed_since = app.data.time - 1
    app.step()  # Issue disarm only after the required ground settling interval.
    item.landed_since = None  # A contact transient before the acknowledgement.
    item.bridge.armed = False
    app.step()
    assert item.phase == "landed"


def test_repeated_land_preserves_pending_disarm_confirmation() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    for item in app.aircraft:
        item.phase = "landing"
        item.landed_since = app.data.time - 1
    app.step()
    app.command("land")
    for item in app.aircraft:
        item.bridge.armed = False
    app.step()
    assert all(item.phase == "landed" for item in app.aircraft)


def test_near_ground_without_contact_does_not_request_disarm() -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    item = app.aircraft[0]
    item.phase = "landing"
    app.data.qpos[item.bindings.qpos + 2] += 0.04
    item.landed_since = app.data.time - 1
    app.step()
    assert not item.landing_disarm_requested


def test_ground_settling_uses_physics_time_not_render_wall_time(
    monkeypatch,
) -> None:
    app = DualSimulation(bridges=[Bridge(), Bridge()])
    item = app.aircraft[0]
    item.phase = "landing"
    item.landed_since = 0
    monkeypatch.setattr(time, "perf_counter", lambda: 1000000.0)
    app.step()
    assert not item.landing_disarm_requested
    app.data.time = 0.8
    app.step()
    assert item.landing_disarm_requested
