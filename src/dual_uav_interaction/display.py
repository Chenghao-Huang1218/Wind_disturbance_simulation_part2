"""Independent wall-clock display cadence, without a physics clock of its own."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RefreshDue:
    """Work due on this iteration; missed frames are never queued."""

    panel: bool
    events: bool
    viewer: bool
    field: bool


class DisplaySchedule:
    """Separate slow numeric readouts from responsive event and scene updates."""

    rates_hz = dict(panel=2., events=30., viewer=20., field=10.)

    def __init__(self, panel_hz: float = 2.) -> None:
        """Set numeric refresh independently from input and rendering.

        Args:
            panel_hz: Finite positive readout frequency in wall-clock Hz.
        """
        if not math.isfinite(panel_hz) or panel_hz <= 0:
            raise ValueError("panel refresh frequency must be finite and positive")
        self.rates_hz = {**type(self).rates_hz, "panel": float(panel_hz)}
        self._next: dict[str, float] = {}
        self._last: float | None = None

    def poll(self, now: float) -> RefreshDue:
        """Return independent due flags, skipping stale frames after a stall.

        Args:
            now: Finite monotonic wall-clock seconds, not simulation time.

        Returns:
            At most one refresh per channel; a clock reset restarts the cadence.
        """
        if not math.isfinite(now):
            raise ValueError("display clock must be finite")
        if self._last is None or now < self._last:
            self._next = dict.fromkeys(self.rates_hz, now)
        self._last = now
        flags = {}
        for name, rate in self.rates_hz.items():
            deadline = self._next[name]
            due = now + 1e-12 >= deadline
            flags[name] = due
            if due:
                interval = 1 / rate
                skipped = math.floor(max(0., now - deadline) / interval + 1e-10)
                self._next[name] = deadline + (skipped + 1) * interval
        return RefreshDue(**flags)
