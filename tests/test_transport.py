"""Acceptance requires movement of the cargo, not only aircraft targets."""

from copy import deepcopy

import pytest

from dual_uav_interaction.transport import TransportSequence


def snapshot(time: float = 0.) -> dict:
    return {'time_s': time, 'payload': {'position': [0., 0., 1.],
            'velocity': [0., 0., 0.], 'angular_velocity': [0., 0., 0.],
            'bottom_clearance_m': .9, 'floor_contact': False},
            'ropes': [{'tension_n': 3.}, {'tension_n': 3.}],
            'aircraft': [{'position': [0., -1., 2.], 'target': [0., -1., 2.], 'phase': 'hover'},
                         {'position': [0., 1., 2.], 'target': [0., 1., 2.], 'phase': 'hover'}]}


def test_cargo_lift_transfer_and_ground_requirements() -> None:
    sequence = TransportSequence([0., 0.])
    sample = snapshot()
    assert sequence.update(sample) is None
    sample['time_s'] = 1.1
    assert sequence.update(sample) == 'move'
    assert sequence.completion['payload_lifted']
    for row in sample['aircraft']:
        row['position'][0] = row['target'][0] = .5
    sample['time_s'] = 2.3
    assert sequence.update(sample) is None
    assert not sequence.completion['payload_translated']
    sample['payload']['position'][0] = .5
    sample['time_s'] = 2.4
    sequence.update(sample)
    sample['time_s'] = 3.5
    assert sequence.update(sample) == 'hold'
    sequence.phase = 'landing'
    for row in sample['aircraft']:
        row['phase'] = 'landed'
    sample['time_s'] = 4.6
    assert sequence.update(sample) is None
    sample['payload']['bottom_clearance_m'] = 0.
    sample['payload']['floor_contact'] = True
    for row in sample['ropes']:
        row['tension_n'] = 0.
    sample['time_s'] = 4.7
    sequence.update(sample)
    sample['time_s'] = 5.5
    assert sequence.update(sample) == 'done'
    assert sequence.completion['payload_grounded']


def test_settling_resets_after_loss_of_clearance() -> None:
    sequence = TransportSequence([0., 0.])
    sample = snapshot()
    sequence.update(sample)
    bad = deepcopy(sample)
    bad['time_s'] = .9
    bad['payload']['floor_contact'] = True
    sequence.update(bad)
    sample['time_s'] = 1.1
    assert sequence.update(sample) is None


@pytest.mark.parametrize('phase', ['transferring', 'holding'])
def test_airborne_cargo_must_not_touch_floor_before_landing(phase: str) -> None:
    sequence = TransportSequence([0., 0.])
    sequence.phase = phase
    sample = snapshot()
    sample['payload']['floor_contact'] = True
    with pytest.raises(RuntimeError, match='cargo'):
        sequence.update(sample)


def test_airborne_box_rotation_is_not_an_attitude_control_requirement() -> None:
    sequence = TransportSequence([0., 0.])
    sample = snapshot()
    sample['payload']['angular_velocity'] = [0., .6, 0.]
    sequence.update(sample)
    sample['time_s'] = 1.1
    assert sequence.update(sample) == 'move'
