"""Automatic dispatch: start the waiting orders one after another, oldest first, without the operator's [시작].

The choice itself (which Pinky, which receiving zone, which arms) stays where it was: OrderDesk.start with the pure
rules in orders/assignment.py. This module only decides *when* the next order may start, and says why it is not starting:

- Order of departure. The next order starts as soon as the Pinky of the order before it has been sent off (its
  VALIDATE_ORDER step, which dispatches the Pinky, is done), so three orders mean three Pinkys leaving one after
  another in the order the orders came in. The Pinky fleet's own reservations (parking exit, passage, loading spot:
  first to ask goes first) then keep them in that order on the road. There is one loading spot and the Pinky that
  stops there first takes it (OrderFlow.station_owner); with wait_for_spot=True
  (CONTROL_STATION_DISPATCH_WAIT_FOR_SPOT=1) the next order instead starts only after the Pinky before it was
  confirmed at the spot, which guarantees the order there at the price of one Pinky driving at a time.
- A stopped order whose alert the operator has not acknowledged: nothing starts. Its Pinky may still stand at
  the loading spot, and after a failure the next robot command is the operator's to release (AGENTS.md).
- No free Pinky or receiving zone (PINKY_UNAVAILABLE, ZONE_BUSY), or the shelves cannot be read: the whole line
  waits, so a later order never overtakes the first one.
- A problem of that order alone (not enough on the shelves, no arm can pick it, it needs to be split): that order is
  passed over and the next one goes. It stays in the intake and is tried again on every pass, still first in line.

Off by default (CONTROL_STATION_AUTO_DISPATCH=1 or the switch on the order screen turns it on): starting an order
sends a real Pinky, which AGENTS.md leaves to the operator unless they asked for it.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Any

from control_station import utc_now_iso
from control_station.orders.desk import OrderDeskError
from control_station.orders.task_runner import TaskExecutionConflict
from control_station.storage.store import InvalidStateTransition, TaskNotFoundError, shipment_of

# The line waits on these: they clear by themselves when a Pinky comes home or a zone is picked up.
WAIT_CODES = {"PINKY_UNAVAILABLE", "ZONE_BUSY", "SHELF_STOCK_UNAVAILABLE"}
# Steps of a started order during which its Pinky has not been sent off yet (VALIDATE_ORDER dispatches it), and, for
# wait_for_spot, during which it has not been confirmed at the loading spot. START_SHIPMENT is in neither: there the
# order waits for a parked Pinky, nothing is driving.
BEFORE_DEPARTURE = frozenset({"VALIDATE_ORDER"})
BEFORE_SPOT = frozenset({"VALIDATE_ORDER", "CHECK_SHELF_STOCK", "PLAN_ARM_SEQUENCE", "WAIT_PINKY_ARRIVAL"})
_DONE = "SUCCEEDED"
# Stopped orders keep their Pinky until the operator acknowledges the alert (store._pinky_holding_orders).
_STOPPED = {"FAILED", "NEEDS_INTERVENTION"}


def step_in(order: dict[str, Any], step_types: frozenset[str]) -> dict[str, Any] | None:
    """The step a started order is in now if it is one of step_types, else None."""
    steps = sorted(order.get("steps") or [], key=lambda step: step["sequence"])
    current = next((step for step in steps if step["id"] == order.get("active_step_id")), None)
    if current is None:  # between two steps, or just claimed: the first step that is not done yet
        current = next((step for step in steps if step["status"] != _DONE), None)
    return current if current is not None and current["step_type"] in step_types else None


def step_before_spot(order: dict[str, Any]) -> dict[str, Any] | None:
    """The step a started order is in while its Pinky is still on its way to the loading spot, else None."""
    return step_in(order, BEFORE_SPOT)


def own_problem(preview: dict[str, Any]) -> dict[str, str] | None:
    """What stops this order whatever the others do, from OrderDesk.preview: {code, message}, or None."""
    fleet = preview.get("fleet") or {}
    if fleet.get("error"):  # the order cannot be split into shipments
        return dict(fleet["error"])
    for trip in fleet.get("trips") or []:
        error = (trip.get("arms") or {}).get("error")
        if error:
            return dict(error)
    return None


class OrderDispatcher:
    def __init__(self, desk: Any, store: Any, *, enabled: bool = False, interval_s: float = 1.0,
                 wait_for_spot: bool = False) -> None:
        self.desk, self.store, self.interval_s, self.wait_for_spot = desk, store, interval_s, wait_for_spot
        self._lock = threading.Lock()
        self._enabled = enabled
        self._state: dict[str, Any] = {"waiting": 0, "blocked": None, "skipped": [], "checked_at": None}
        self._started: deque[dict[str, Any]] = deque(maxlen=20)  # newest first, in memory only
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ---- switch and view -----------------------------------------------------------------------------------

    def set_enabled(self, enabled: bool) -> dict[str, Any]:
        with self._lock:
            self._enabled = bool(enabled)
            if not self._enabled:
                self._state.update(blocked=None, skipped=[])
        return self.snapshot()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {"auto": self._enabled, "wait_for_spot": self.wait_for_spot, **self._state, "skipped": list(self._state["skipped"]),
                    "started": list(self._started)}

    # ---- one pass ------------------------------------------------------------------------------------------

    def tick(self) -> dict[str, Any]:
        """Look at the line once and start what may start. Safe to call from the loop or a test."""
        with self._lock:
            enabled = self._enabled
        if not enabled:
            return self.snapshot()
        orders = self.desk.pending()
        blocked: dict[str, Any] | None = None
        skipped: list[dict[str, Any]] = []
        for order in orders:
            order_id = order["id"]
            gate = self._line_gate()
            if gate is not None:
                blocked = {"order_id": order_id, **gate}
                break
            try:
                problem = own_problem(self.desk.preview(order_id))
            except OrderDeskError:
                continue  # started or rejected by the operator meanwhile
            if problem is not None and problem.get("code") in WAIT_CODES:
                blocked = {"order_id": order_id, **problem}
                break
            if problem is not None:
                skipped.append({"order_id": order_id, **problem})
                continue
            try:
                started, changed = self.desk.start(order_id)
            except OrderDeskError as exc:
                entry = {"order_id": order_id, "code": exc.error_code, "message": str(exc)}
                if exc.error_code in WAIT_CODES:
                    blocked = entry
                    break
                skipped.append(entry)
                continue
            except (TaskExecutionConflict, InvalidStateTransition, TaskNotFoundError) as exc:
                skipped.append({"order_id": order_id, "code": getattr(exc, "error_code", "ORDER_CONTROL_ERROR"),
                                "message": str(exc)})
                continue
            if changed:
                with self._lock:
                    self._started.appendleft({"order_id": order_id, "pinky_id": started.get("pinky_id"),
                                              "receiving_zone": started.get("receiving_zone"), "at": utc_now_iso()})
        with self._lock:
            self._state = {"waiting": len(self.desk.pending()), "blocked": blocked, "skipped": skipped,
                           "checked_at": utc_now_iso()}
        return self.snapshot()

    def _line_gate(self) -> dict[str, Any] | None:
        """Why no order at all may start now: {code, message, waiting_for}, or None."""
        for holder in sorted(set(self.store.busy_pinkies().values())):
            task = self.store.get_task(holder)
            if task is not None and task["status"] in _STOPPED:
                return {"code": "STOPPED_ORDER", "waiting_for": holder,
                        "message": f"멈춘 주문 {holder}의 경보를 [확인]하면 이어서 출발해요"}
        for running in self.store.running_orders():
            step = step_in(running, BEFORE_SPOT if self.wait_for_spot else BEFORE_DEPARTURE)
            if step is None:
                continue
            shipment = shipment_of(step)
            trip = f" (운송 {shipment})" if shipment > 1 else ""
            if self.wait_for_spot:
                return {"code": "SPOT_ORDER", "waiting_for": running["id"],
                        "message": f"앞 주문 {running['id']}의 핑키가 적재 자리에 도착하면 출발해요{trip}"}
            return {"code": "DEPARTURE_ORDER", "waiting_for": running["id"],
                    "message": f"앞 주문 {running['id']}의 핑키가 출발하면 바로 이어서 출발해요{trip}"}
        return None

    # ---- background loop -----------------------------------------------------------------------------------

    def start(self) -> "OrderDispatcher":
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True, name="order-dispatcher")
            self._thread.start()
        return self

    def _run(self) -> None:
        while not self._stop.wait(self.interval_s):
            try:
                self.tick()
            except Exception as exc:  # noqa: BLE001 (a pass must never end the loop; the screen shows why)
                with self._lock:
                    self._state = {**self._state, "checked_at": utc_now_iso(),
                                   "blocked": {"order_id": None, "code": "DISPATCH_ERROR", "message": str(exc)}}

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
