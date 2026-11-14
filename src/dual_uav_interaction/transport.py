"""Measured cargo acceptance for the bounded automatic carrying demonstration."""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np


class TransportSequence:
    """Require stable lift, actual +X transport and settled ground contact.

    Args:
        initial_payload_xy: Ground cargo centre before takeoff.
    """

    def __init__(self, initial_payload_xy: Sequence[float]) -> None:
        self.initial_xy = np.asarray(initial_payload_xy, dtype=float).copy()
        self.offset = np.array([.5, 0., 0.])
        self.phase = 'lifting'
        self.since: float | None = None
        self.completion = dict(payload_lifted=False, payload_translated=False,
                               payload_grounded=False)

    def update(self, sample: dict[str, Any]) -> str | None:
        """Advance only on sustained measured physics conditions.

        Args:
            sample: Same-time simulation snapshot, including cargo and ropes.

        Returns:
            move, hold, done on transitions, or None while evidence is incomplete.
        """
        cargo = sample['payload']
        if self.phase in ('transferring', 'holding') and (
            cargo['floor_contact'] or cargo['bottom_clearance_m'] <= .15
        ):
            raise RuntimeError('cargo lost airborne clearance before commanded landing')
        speed = np.linalg.norm(cargo['velocity'])
        stable_air = (cargo['bottom_clearance_m'] > .15
                      and not cargo['floor_contact'] and speed < .2
                      and all(r['tension_n'] > .1 for r in sample['ropes'])
                      and all(np.linalg.norm(np.array(r['position']) - r['target']) < .15
                              for r in sample['aircraft']))
        if self.phase == 'lifting':
            valid = stable_air
            interval = 1.
            result = 'move'
        elif self.phase == 'transferring':
            displacement = np.array(cargo['position'][:2]) - self.initial_xy
            valid = (stable_air and displacement[0] >= .35
                     and np.linalg.norm(displacement - self.offset[:2]) < .15)
            interval = 1.
            result = 'hold'
        elif self.phase == 'landing':
            valid = (cargo['floor_contact'] and abs(cargo['bottom_clearance_m']) < .03
                     and speed < .2 and np.linalg.norm(cargo['angular_velocity']) < .3
                     and all(r['tension_n'] < .1 for r in sample['ropes'])
                     and all(r['phase'] == 'landed' for r in sample['aircraft']))
            interval = .7
            result = 'done'
        else:
            return None
        if not valid:
            self.since = None
            return None
        now = float(sample['time_s'])
        if self.since is None:
            self.since = now
        if now - self.since < interval:
            return None
        self.since = None
        if result == 'move':
            self.completion['payload_lifted'] = True
            self.phase = 'transferring'
        elif result == 'hold':
            self.completion['payload_translated'] = True
            self.phase = 'holding'
        else:
            self.completion['payload_grounded'] = True
            self.phase = 'done'
        return result
