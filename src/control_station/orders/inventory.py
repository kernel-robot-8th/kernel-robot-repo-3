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
window may sell. An object whose stock is unknown is closed for ordering, and both windows say why.

The operator can override any object (InventoryOverrides, kept in a small JSON file): `closed` stops selling it,
`manual` sells from a count the operator typed instead of the camera's (e.g. while a camera is down), `auto` goes
back to the camera.
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

CONFIRM_READINGS = 3
STALE_AFTER_S = 10.0
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

    def __init__(self, *, confirm_readings: int = CONFIRM_READINGS, stale_after_s: float = STALE_AFTER_S) -> None:
        self.confirm_readings = confirm_readings
        self.stale_after_s = stale_after_s
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

    def snapshot(self, products: Iterable[Mapping[str, str]], reserved: Mapping[str, int], now: float,
                 overrides: Mapping[str, Mapping[str, Any]] | None = None) -> dict[str, Any]:
        """Per product: where the count comes from, on each shelf, reserved, available, and its status:
        available (some left), sold_out, unknown (camera count missing, closed for ordering), closed (by the
        operator)."""
        state, reason = self.state(now)
        shelves = self.on_shelves()
        overrides = overrides or {}
        rows = []
        for product in products:
            name = product["object_class"]
            override = overrides.get(name) or {}
            held = int(reserved.get(name, 0))
            per_shelf = {shelf: counts[name] for shelf, counts in shelves.items() if counts.get(name)}
            if override.get("mode") == "closed":
                source, total, available, status = "closed", sum(per_shelf.values()), 0, "closed"
            elif override.get("mode") == "manual":
                source, total = "manual", int(override.get("count", 0))
                available = max(0, total - held)
                status = "available" if available else "sold_out"
            elif state == "unknown":
                source, total, available, status = "camera", None, None, "unknown"
            else:
                source, total = "camera", sum(per_shelf.values())
                available = max(0, total - held)
                status = "available" if available else "sold_out"
            rows.append({"object_class": name, "label": product.get("label", name), "source": source,
                         "on_shelves": total, "per_shelf": per_shelf, "reserved": held, "available": available,
                         "status": status, "override": dict(override) or None})
        return {"state": state, "reason": reason, "products": rows}


class InventoryOverrides:
    """The operator's per-object settings, kept in a JSON file so they survive a restart."""

    MODES = frozenset({"auto", "closed", "manual"})

    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.Lock()
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            loaded = {}
        self._values: dict[str, dict[str, Any]] = {
            name: value for name, value in (loaded.items() if isinstance(loaded, dict) else [])
            if isinstance(value, dict) and value.get("mode") in {"closed", "manual"}
        }

    def all(self) -> dict[str, dict[str, Any]]:
        with self._lock:
            return {name: dict(value) for name, value in self._values.items()}

    def set(self, object_class: str, mode: str, count: object = None) -> dict[str, Any] | None:
        """Set one object's mode; returns the stored override (None for auto). Raises ValueError on bad input."""
        if mode not in self.MODES:
            raise ValueError("mode는 auto, closed, manual 중 하나여야 합니다")
        if mode == "manual" and (isinstance(count, bool) or not isinstance(count, int) or not 0 <= count <= 99):
            raise ValueError("수동 수량은 0에서 99 사이의 정수여야 합니다")
        value = None if mode == "auto" else ({"mode": "manual", "count": count} if mode == "manual" else {"mode": "closed"})
        with self._lock:
            if value is None:
                self._values.pop(object_class, None)
            else:
                self._values[object_class] = {**value, "set_at": _now_iso()}
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(".tmp")
            temporary.write_text(json.dumps(self._values, ensure_ascii=False, indent=2), encoding="utf-8")
            temporary.replace(self.path)
            return dict(self._values[object_class]) if value is not None else None


class InventoryMonitor:
    """Reads the shelves every `period_s` on a background thread and keeps the book and a short change log."""

    def __init__(self, read_shelves: Callable[[], Mapping[str, Any]], busy: Callable[[], bool],
                 products: Callable[[], list[dict[str, str]]], reserved: Callable[[], Mapping[str, int]],
                 transfers: Callable[[], list[tuple[str, str, str]]], book: InventoryBook | None = None,
                 overrides: InventoryOverrides | None = None, period_s: float = READ_PERIOD_S,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.read_shelves = read_shelves
        self.busy = busy
        self.products = products
        self.reserved = reserved
        self.transfers = transfers
        # Transfers that happened before this start are already reflected in what the camera sees.
        self._booked = {step_id for step_id, _, _ in transfers()}
        self.book = book or InventoryBook()
        self.overrides = overrides
        self.period_s = period_s
        self.clock = clock
        self._events: deque[dict[str, Any]] = deque(maxlen=EVENT_LOG_SIZE)
        self._simulated = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def read_once(self) -> None:
        moves = [(step_id, shelf, name) for step_id, shelf, name in self.transfers() if step_id not in self._booked]
        busy = bool(self.busy())
        try:
            answer = None if busy else self.read_shelves()
            if answer is not None:
                self._simulated = bool(answer.get("simulated"))
            reading = StockReading(self.clock(), None if answer is None else answer["shelves"], busy=busy)
        except Exception as exc:  # camera gone, detector not ready, no fresh frames: the stock becomes unknown
            reading = StockReading(self.clock(), error=f"선반 재고를 읽지 못했습니다: {exc}")
        with self._lock:
            self.book.book_moves((shelf, name) for _, shelf, name in moves)
            self._booked.update(step_id for step_id, _, _ in moves)
            for change in self.book.observe(reading):
                self._events.append({"at": _now_iso(), **change})

    def snapshot(self) -> dict[str, Any]:
        reserved = self.reserved()
        overrides = self.overrides.all() if self.overrides is not None else {}
        with self._lock:
            view = self.book.snapshot(self.products(), reserved, self.clock(), overrides)
            view["events"] = list(self._events)[::-1]
        # Counts from the mock world are labeled as such, never as the camera's.
        view["simulated"] = self._simulated
        if self._simulated:
            for row in view["products"]:
                if row["source"] == "camera":
                    row["source"] = "mock_world"
        view["checked_at"] = _now_iso()
        return view

    def set_override(self, object_class: str, mode: str, count: object = None) -> dict[str, Any] | None:
        """The operator closes, hand-counts or returns an object to the camera; the change is logged."""
        if self.overrides is None:
            raise ValueError("재고 수정 기능이 켜져 있지 않습니다")
        if object_class not in {product["object_class"] for product in self.products()}:
            raise ValueError(f"주문할 수 있는 물건이 아닙니다: {object_class}")
        stored = self.overrides.set(object_class, mode, count)
        with self._lock:
            self._events.append({"at": _now_iso(), "kind": "OVERRIDE", "object_class": object_class, "mode": mode,
                                 "count": count if mode == "manual" else None})
        return stored

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

