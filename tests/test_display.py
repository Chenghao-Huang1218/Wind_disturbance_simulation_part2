"""Wall-clock display rates never enqueue missed refreshes or touch physics."""

import pytest

from dual_uav_interaction.display import DisplaySchedule


def test_independent_refresh_rates_and_no_backlog() -> None:
    schedule = DisplaySchedule()
    counts = dict(panel=0, events=0, viewer=0, field=0)
    for tick in range(200):
        due = schedule.poll(tick * .005)
        for key in counts:
            counts[key] += getattr(due, key)
    assert counts == dict(panel=2, events=30, viewer=20, field=10)
    due = schedule.poll(100.)
    assert all(getattr(due, key) for key in counts)
    assert not any(getattr(schedule.poll(100.), key) for key in counts)


def test_invalid_clock_is_rejected_and_clock_reset_recovers() -> None:
    schedule = DisplaySchedule()
    schedule.poll(10.)
    assert schedule.poll(0.).panel
    for invalid in (float('nan'), float('inf')):
        with pytest.raises(ValueError):
            schedule.poll(invalid)
