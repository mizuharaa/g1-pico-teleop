"""Stream-resume gate — LOCAL ADDITION (2026-08-10).

Problem this solves: after headset sleep/wake or a service reconnect, the
XRoboToolkit PC Service can hand a freshly-connected client ONE cached body
frame whose device timestamp then never advances (the app stopped sending).
A reader that accepts any new timestamp publishes that zombie frame once —
the robot/viewer "retargets for 1 frame" and freezes.

The gate demands a LIVE stream before letting frames through again: after
any silence gap (or at startup), the first advancing frame is swallowed as a
probe, and frames flow only when the timestamp advances AGAIN within the
probe window. A real 50–90 Hz stream loses ~11 ms; a cached lone frame is
blocked entirely.
"""
from __future__ import annotations

import time


class StreamResumeGate:
    def __init__(
        self,
        *,
        gap_s: float = 2.0,
        probe_window_s: float = 1.0,
        clock=None,
        log=print,
    ) -> None:
        self.gap_s = float(gap_s)
        self.probe_window_s = float(probe_window_s)
        self._clock = clock or time.monotonic
        self._log = log
        self._last_accept: float | None = None
        self._probe_at: float | None = None

    def accept(self) -> bool:
        """Call once per NEW (advancing-timestamp) frame.

        Returns True when the frame may be published, False when it must be
        swallowed as a resume probe.
        """
        now = self._clock()
        gap = (
            self._last_accept is None
            or now - self._last_accept > self.gap_s
        )
        if gap:
            if (
                self._probe_at is None
                or now - self._probe_at > self.probe_window_s
            ):
                self._probe_at = now
                self._log(
                    "[StreamGate] frame after silence — waiting for a second "
                    "advancing frame before resuming (zombie-frame filter)"
                )
                return False
        self._probe_at = None
        self._last_accept = now
        return True


__all__ = ["StreamResumeGate"]
