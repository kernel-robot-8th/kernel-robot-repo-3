"""The ceiling camera talking to a Pinky while it drives into the loading spot (2026-11-18).

The parking node follows one spot's marker live (/pinky_parking/state): where the Pinky is against its registered spot,
frame by frame. While the Pinky that carries that marker is on a mission and in view, this sends each new offset to
the fleet dashboard (POST /api/robots/{id}/dock-guide), which passes it on to the Pinky; the Pinky's drive into the
station steers on it (pinky_fleet_dashboard/scripts/mission_behaviors.DockEntry) and stops where the camera sees the
spot. So it comes in on the spot in one go instead of stopping where its map said and being sent back in.

The live state stays what the contract says for everything else: never the condition for an arm step. Arrival is
still decided by a confirmed check over new frames (dock_handshake.py). Here the same numbers only steer the last
25 cm of driving, and a Pinky that stops hearing them simply drives in on its odometry as before."""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

TRACKED_STATES = frozenset({"MOVING", "IN_POSITION", "OFF_POSITION"})
PARKED_PHASES = frozenset({"parked"})


def guide_from(live: dict[str, Any] | None) -> dict[str, float] | None:
    """forward_mm, left_mm, dyaw_deg from a live snapshot while the marker is tracked, else None."""
    if not isinstance(live, dict) or live.get("state") not in TRACKED_STATES:
        return None
    offset = live.get("offset") or {}
    try:
        guide = {key: float(offset[key]) for key in ("forward_mm", "left_mm", "dyaw_deg")}
    except (KeyError, TypeError, ValueError):
        return None
    return guide if all(math.isfinite(value) for value in guide.values()) else None


class DockGuideRelay:
    """snapshot() -> the live state; phase(pinky_id) -> the Pinky's mission phase; send(pinky_id, guide) -> True when
    the dashboard took it. One frame is sent once; after a refusal or an error it stays quiet for `retry_s`."""

    def __init__(self, snapshot: Callable[[], dict[str, Any]], pinky_id: str,
                 phase: Callable[[str], str | None], send: Callable[[str, dict[str, float]], bool], *,
                 period_s: float = 0.1, retry_s: float = 3.0, clock: Callable[[], float] = time.monotonic) -> None:
        self.snapshot, self.pinky_id, self.phase, self.send = snapshot, pinky_id, phase, send
        self.period_s, self.retry_s, self.clock = period_s, retry_s, clock
        self.sent = self.failed = 0
        self.last: dict[str, Any] | None = None  # the last guide sent, with when
        self._frame: Any = None
        self._quiet_until = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def tick(self) -> str:
        """One round; returns what happened (for the status line and the tests)."""
        now = self.clock()
        if now < self._quiet_until:
            return "QUIET"
        live = self.snapshot()
        guide = guide_from(live)
        if guide is None:
            return "NOT_TRACKED"
        frame = live.get("frame_sequence")
        if frame is not None and frame == self._frame:
            return "SAME_FRAME"  # a stalled camera must not look fresh to the Pinky
        try:
            if self.phase(self.pinky_id) in PARKED_PHASES:
                return "PARKED"
            self._frame = frame
            if self.send(self.pinky_id, guide):
                self.sent += 1
                self.last = {**guide, "at": time.time()}
                return "SENT"
        except Exception:  # the dashboard is away: try again later, the Pinky drives in without the guide
            pass
        self.failed += 1
        self._quiet_until = now + self.retry_s
        return "REFUSED"

    def start(self) -> None:
        def loop() -> None:
            while not self._stop.wait(self.period_s):
                self.tick()
        self._thread = threading.Thread(target=loop, name="dock-guide-relay", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def status(self) -> dict[str, Any]:
        return {"pinky_id": self.pinky_id, "sent": self.sent, "failed": self.failed, "last": self.last}
