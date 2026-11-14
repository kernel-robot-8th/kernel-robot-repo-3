"""Shelf inventory from the cameras: what is on the shelves, what open orders have claimed, what can still be sold.

The shelf counter (camera_counts.ShelfStockCounter, or the mock world) already takes the median of a few frames,
but YOLO counts still flicker between readings and an arm reaching into a shelf hides what is behind it. So:

- A changed count is accepted only after it reads the same `confirm_readings` times in a row.
- While an arm is working (the loading spot is held, or a policy runs) readings are not used: the book keeps the
  last confirmed counts, and the units being picked are still covered by their orders' reservations.
- With no usable reading for `stale_after_s` (camera gone, detector not ready) the stock is unknown, never zero.

Our own picks are booked as they happen: when the order flow confirms a unit moved from a shelf onto a Pinky,
that shelf's count drops by one at once (a ledger), and the camera later only confirms it. A change the camera
confirms that we did not make (someone took an object, or restocked) is logged as such.

Available = shelf counts - reserved by open orders (ordered and not yet moved onto a Pinky). It is what the order
window may sell: zero is sold out, `low_stock_at` or fewer is low.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

CONFIRM_READINGS = 3
STALE_AFTER_S = 10.0
LOW_STOCK_AT = 1
READ_PERIOD_S = 2.0
EVENT_LOG_SIZE = 40


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


@dataclass(frozen=True)
class StockReading:
    """One look at the shelves: counts per shelf, or why there are none; `busy` when an arm is at work."""

    at: float  # monotonic seconds
    shelves: Mapping[str, Mapping[str, int]] | None = None
    error: str | None = None
    busy: bool = False


class InventoryBook:
    """Confirmed shelf counts, built from readings; no I/O, so the rules are testable with plain readings."""

    def __init__(self, *, confirm_readings: int = CONFIRM_READINGS, stale_after_s: float = STALE_AFTER_S,
                 low_stock_at: int = LOW_STOCK_AT) -> None:
        self.confirm_readings = confirm_readings
        self.stale_after_s = stale_after_s
        self.low_stock_at = low_stock_at
        self._confirmed: dict[tuple[str, str], int] = {}
        self._pending: dict[tuple[str, str], tuple[int, int]] = {}  # key -> (value seen, readings in a row)
        self._has_confirmed = False
        self._last_usable_at: float | None = None
        self._last_error: str | None = None
        self._busy = False

    def observe(self, reading: StockReading) -> list[dict[str, Any]]:
        """Take one reading; returns the changes it confirmed (restocked or removed), oldest first."""
        if reading.error is not None:
            self._last_error = reading.error
            return []
        self._busy = reading.busy
        if reading.busy or reading.shelves is None:
            return []
        self._last_error = None
        self._last_usable_at = reading.at
        seen = {(shelf, name): int(count) for shelf, counts in reading.shelves.items() for name, count in counts.items()}
        changes = []
        for key in sorted(set(seen) | set(self._confirmed) | set(self._pending)):
            value = seen.get(key, 0)
            if self._has_confirmed and value == self._confirmed.get(key, 0):
                self._pending.pop(key, None)
                continue
            previous, streak = self._pending.get(key, (None, 0))
            streak = streak + 1 if previous == value else 1
            if streak < self.confirm_readings:
                self._pending[key] = (value, streak)
                continue
            self._pending.pop(key, None)
            before = self._confirmed.get(key, 0)
            self._confirmed[key] = value
            if self._has_confirmed and value != before:
                shelf, name = key
                # Our own picks are already booked, so a change confirmed here came from outside the order flow.
                changes.append({"shelf_id": shelf, "object_class": name, "from": before, "to": value,
                                "kind": "RESTOCKED" if value > before else "REMOVED_OUTSIDE"})
        if not self._has_confirmed and not self._pending:
            self._has_confirmed = True  # every key read the same `confirm_readings` times: the first full picture
        return changes

    def book_moves(self, moves: Iterable[tuple[str, str]]) -> None:
        """Our confirmed transfers, (shelf, object) each: take them off the shelf now, before the camera sees it."""
        for shelf, name in moves:
            key = (shelf, name)
            self._confirmed[key] = max(0, self._confirmed.get(key, 0) - 1)
            self._pending.pop(key, None)

    def state(self, now: float) -> tuple[str, str | None]:
        """(state, reason): live, held (an arm is at work), unknown (no confirmed picture or no reading lately)."""
        if not self._has_confirmed:
            return "unknown", self._last_error or "재고를 확인하는 중입니다"
        if self._busy:
            return "held", "팔이 작업 중이라 마지막 확인값을 씁니다"
        if self._last_usable_at is None or now - self._last_usable_at > self.stale_after_s:
            return "unknown", self._last_error or "선반 카메라의 새 값이 없습니다"
        return "live", None

    def on_shelves(self) -> dict[str, dict[str, int]]:
        shelves: dict[str, dict[str, int]] = {}
        for (shelf, name), count in sorted(self._confirmed.items()):
            if count > 0:
                shelves.setdefault(shelf, {})[name] = count
        return shelves

    def snapshot(self, products: Iterable[Mapping[str, str]], reserved: Mapping[str, int], now: float) -> dict[str, Any]:
        """Per product: on each shelf, reserved, available and a status the order window and the screen show."""
        state, reason = self.state(now)
        shelves = self.on_shelves()
        rows = []
        for product in products:
            name = product["object_class"]
            per_shelf = {shelf: counts[name] for shelf, counts in shelves.items() if counts.get(name)}
            total = sum(per_shelf.values())
            held = int(reserved.get(name, 0))
            available = max(0, total - held) if state != "unknown" else None
            if available is None:
                status = "unknown"
            elif available == 0:
                status = "sold_out"
            elif available <= self.low_stock_at:
                status = "low"
            else:
                status = "in_stock"
            rows.append({"object_class": name, "label": product.get("label", name), "on_shelves": total,
                         "per_shelf": per_shelf, "reserved": held, "available": available, "status": status})
        return {"state": state, "reason": reason, "products": rows}


class InventoryMonitor:
    """Reads the shelves every `period_s` on a background thread and keeps the book and a short change log."""

    def __init__(self, read_shelves: Callable[[], Mapping[str, Any]], busy: Callable[[], bool],
                 products: Callable[[], list[dict[str, str]]], reserved: Callable[[], Mapping[str, int]],
                 transfers: Callable[[], list[tuple[str, str, str]]], book: InventoryBook | None = None,
                 period_s: float = READ_PERIOD_S, clock: Callable[[], float] = time.monotonic) -> None:
        self.read_shelves = read_shelves
        self.busy = busy
        self.products = products
        self.reserved = reserved
        self.transfers = transfers
        # Transfers that happened before this start are already reflected in what the camera sees.
        self._booked = {step_id for step_id, _, _ in transfers()}
        self.book = book or InventoryBook()
        self.period_s = period_s
        self.clock = clock
        self._events: deque[dict[str, Any]] = deque(maxlen=EVENT_LOG_SIZE)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def read_once(self) -> None:
        moves = [(step_id, shelf, name) for step_id, shelf, name in self.transfers() if step_id not in self._booked]
        busy = bool(self.busy())
        try:
            reading = StockReading(self.clock(), None if busy else self.read_shelves()["shelves"], busy=busy)
        except Exception as exc:  # camera gone, detector not ready, no fresh frames: the stock becomes unknown
            reading = StockReading(self.clock(), error=f"선반 재고를 읽지 못했습니다: {exc}")
        with self._lock:
            self.book.book_moves((shelf, name) for _, shelf, name in moves)
            self._booked.update(step_id for step_id, _, _ in moves)
            for change in self.book.observe(reading):
                self._events.append({"at": _now_iso(), **change})

    def snapshot(self) -> dict[str, Any]:
        reserved = self.reserved()
        with self._lock:
            view = self.book.snapshot(self.products(), reserved, self.clock())
            view["events"] = list(self._events)[::-1]
        view["checked_at"] = _now_iso()
        return view

    def start(self) -> InventoryMonitor:
        def loop() -> None:
            while not self._stop.is_set():
                self.read_once()
                self._stop.wait(self.period_s)
        self._thread = threading.Thread(target=loop, name="inventory-monitor", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

