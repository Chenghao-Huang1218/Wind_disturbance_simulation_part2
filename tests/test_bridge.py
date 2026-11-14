"""MAVLink target and sender isolation without real PX4 sockets."""

from types import SimpleNamespace
from typing import Any

import numpy as np

from dual_uav_interaction.px4_bridge import Px4Bridge


def message(kind: str, system: int, **fields: Any) -> Any:
    return SimpleNamespace(
        get_type=lambda: kind,
        get_srcSystem=lambda: system,
        get_srcComponent=lambda: 1,
        **fields
    )


class Link:
    def __init__(self, messages: list | None = None) -> None:
        self.messages = list(messages or [])
        self.mav = SimpleNamespace()

    def recv_match(self, **kwargs: Any) -> Any:
        return self.messages.pop(0) if self.messages else None


def test_targets_senders_and_foreign_traffic() -> None:
    calls = []

    def factory(endpoint: str, **kwargs: Any) -> Link:
        calls.append((endpoint, kwargs))
        return Link()

    bridge = Px4Bridge(
        [50] * 4,
        sim_tcp_port=4561,
        command_udp_port=14581,
        system_id=2,
        simulator_system_id=201,
        command_system_id=207,
        connection_factory=factory,
    )
    bridge.connect()
    assert (
        calls[0][0].endswith(":4561") and calls[0][1]["source_system"] == 201
    )
    assert calls[1][1]["source_system"] == 207
    commands = []
    bridge.command_link.mav = SimpleNamespace(
        command_long_send=lambda *args: commands.append(args)
    )
    bridge.request_arm()
    assert commands[0][:2] == (2, 1)
    bridge.command_link.messages = [
        message("HEARTBEAT", 1, base_mode=128, custom_mode=6 << 16),
        message("COMMAND_ACK", 1, command=400, result=0),
    ]
    bridge.poll_status()
    assert not bridge.armed and bridge.arm_ack_sequence == 0
    bridge.command_link.messages = [
        message("HEARTBEAT", 2, base_mode=128, custom_mode=6 << 16)
    ]
    bridge.poll_status()
    assert bridge.armed and bridge.offboard
    bridge.sim_link.messages = [
        message("HIL_ACTUATOR_CONTROLS", 1, controls=[1] * 16)
    ]
    np.testing.assert_array_equal(bridge.receive_rotor_thrusts(), 0)
    bridge.sim_link.messages = [
        message("HIL_ACTUATOR_CONTROLS", 2, controls=[0.5] * 16)
    ]
    np.testing.assert_array_equal(bridge.receive_rotor_thrusts(), 25)
