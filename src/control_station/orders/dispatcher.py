"""자동 출발: 운영자의 [시작] 없이, 기다리는 주문을 오래된 것부터 차례로 출발시킨다.

무엇을 고를지(어느 핑키, 어느 수령 구역, 어느 팔)는 원래 자리 그대로다: OrderDesk.start와 orders/assignment.py의
순수 규칙. 이 모듈은 다음 주문이 *언제* 출발해도 되는지만 정하고, 출발하지 않는 이유를 알려 준다.

- 출발 순서. 앞 주문의 핑키가 출발하자마자(핑키를 보내는 VALIDATE_ORDER 단계가 끝나면) 다음 주문이 출발한다.
  그래서 주문 세 건이면 핑키 세 대가 주문이 들어온 순서대로 하나씩 떠난다. 길 위에서는 핑키 관제의 구역 예약
  (주차장 나가기, 통로, 적재 자리: 먼저 요청한 쪽이 먼저)이 그 순서를 지킨다. 적재 자리는 하나이고 먼저 선 핑키가
  쓴다(OrderFlow.station_owner). wait_for_spot=True(CONTROL_STATION_DISPATCH_WAIT_FOR_SPOT=1)면 대신 앞 핑키가
  적재 자리에 도착했다고 확인된 뒤에야 다음 주문이 출발한다. 자리 순서는 보장되지만 한 번에 핑키 한 대만 달린다.
- 운영자가 경보를 확인하지 않은 멈춘 주문이 있음: 아무것도 출발시키지 않는다. 그 핑키가 아직 적재 자리에 서 있을
  수 있고, 실패 뒤 다음 로봇 명령은 운영자가 풀어 줄 일이다(AGENTS.md).
- 빈 핑키나 수령 구역이 없음(PINKY_UNAVAILABLE, ZONE_BUSY), 또는 선반을 읽을 수 없음: 줄 전체가 기다린다.
  뒤 주문이 맨 앞 주문을 앞지르지 않게 하려는 것이다.
- 그 주문만의 문제(선반에 부족, 집을 팔이 없음, 나눠야 하는 주문): 그 주문은 건너뛰고 다음 주문을 본다. 건너뛴
  주문은 접수 상태로 남아 매번 줄 맨 앞에서 다시 확인한다.

기본은 꺼져 있다(CONTROL_STATION_AUTO_DISPATCH=1이나 주문 화면의 스위치로 켠다). 주문을 시작하면 실제 핑키가
나가는데, AGENTS.md는 사용자가 요청하지 않는 한 그것을 운영자에게 맡기기 때문이다.

스레드: start()가 띄운 백그라운드 스레드가 interval_s(기본 1초)마다 tick()을 부른다. 스위치와 화면용 상태는
`_lock`으로 지키고, 주문 시작 자체는 락 밖에서 OrderDesk에 맡긴다.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Any

from control_station import utc_now_iso
from control_station.orders.desk import OrderDeskError
from control_station.orders.task_runner import TaskExecutionConflict
from control_station.storage.store import InvalidStateTransition, TaskNotFoundError, shipment_of

# 줄 전체가 기다리는 이유들: 핑키가 돌아오거나 구역에서 수령이 끝나면 저절로 풀린다.
WAIT_CODES = {"PINKY_UNAVAILABLE", "ZONE_BUSY", "SHELF_STOCK_UNAVAILABLE"}
# 시작한 주문의 핑키가 아직 출발하지 않은 단계(VALIDATE_ORDER가 출발시킨다), 그리고 wait_for_spot일 때 아직 적재
# 자리 도착이 확인되지 않은 단계. START_SHIPMENT는 둘 다 아니다: 거기서는 주문이 주차된 핑키를 기다릴 뿐 달리는
# 핑키가 없다.
BEFORE_DEPARTURE = frozenset({"VALIDATE_ORDER"})
BEFORE_SPOT = frozenset({"VALIDATE_ORDER", "CHECK_SHELF_STOCK", "PLAN_ARM_SEQUENCE", "WAIT_PINKY_ARRIVAL"})
_DONE = "SUCCEEDED"
# 멈춘 주문은 운영자가 경보를 확인할 때까지 핑키를 쥔다(store._pinky_holding_orders).
_STOPPED = {"FAILED", "NEEDS_INTERVENTION"}


def step_in(order: dict[str, Any], step_types: frozenset[str]) -> dict[str, Any] | None:
    """시작한 주문이 지금 있는 단계가 step_types 중 하나면 그 단계, 아니면 None."""
    steps = sorted(order.get("steps") or [], key=lambda step: step["sequence"])
    current = next((step for step in steps if step["id"] == order.get("active_step_id")), None)
    if current is None:  # 두 단계 사이이거나 막 선점됨: 아직 끝나지 않은 첫 단계를 지금 단계로 본다
        current = next((step for step in steps if step["status"] != _DONE), None)
    return current if current is not None and current["step_type"] in step_types else None


def step_before_spot(order: dict[str, Any]) -> dict[str, Any] | None:
    """시작한 주문의 핑키가 아직 적재 자리로 가는 중이면 지금 단계, 아니면 None."""
    return step_in(order, BEFORE_SPOT)


def own_problem(preview: dict[str, Any]) -> dict[str, str] | None:
    """다른 주문과 상관없이 이 주문을 막는 문제({code, message}), 없으면 None. OrderDesk.preview 결과에서 읽는다."""
    fleet = preview.get("fleet") or {}
    if fleet.get("error"):  # 주문을 운송분으로 나눌 수 없다(예: 핑키 한 대에 안 실리는 물체)
        return dict(fleet["error"])
    for trip in fleet.get("trips") or []:
        error = (trip.get("arms") or {}).get("error")
        if error:
            return dict(error)
    return None


class OrderDispatcher:
    """기다리는 주문을 접수 순서대로 자동 출발시키는 줄 관리자. 켜고 끄기는 서버 메모리에만 있다."""

    def __init__(self, desk: Any, store: Any, *, enabled: bool = False, interval_s: float = 1.0,
                 wait_for_spot: bool = False) -> None:
        self.desk, self.store, self.interval_s, self.wait_for_spot = desk, store, interval_s, wait_for_spot
        self._lock = threading.Lock()
        self._enabled = enabled
        self._state: dict[str, Any] = {"waiting": 0, "blocked": None, "skipped": [], "checked_at": None}
        self._started: deque[dict[str, Any]] = deque(maxlen=20)  # 최근 자동 출발 20건, 최신부터, 메모리에만
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ---- 스위치와 화면 ---------------------------------------------------------------------------------

    def set_enabled(self, enabled: bool) -> dict[str, Any]:
        """자동 출발을 켜거나 끈다. 끄면 대기 이유와 건너뛴 목록을 지운다(이미 출발한 주문은 그대로 진행)."""
        with self._lock:
            self._enabled = bool(enabled)
            if not self._enabled:
                self._state.update(blocked=None, skipped=[])
        return self.snapshot()

    def snapshot(self) -> dict[str, Any]:
        """GET /api/dispatch가 돌려주는 상태: 켜짐 여부, 기다리는 수, 막힌 이유, 건너뛴 주문, 최근 출발."""
        with self._lock:
            return {"auto": self._enabled, "wait_for_spot": self.wait_for_spot, **self._state, "skipped": list(self._state["skipped"]),
                    "started": list(self._started)}

    # ---- 한 번 훑기 ---------------------------------------------------------------------------------------

    def tick(self) -> dict[str, Any]:
        """줄을 한 번 보고 출발해도 되는 주문을 출발시킨다. 백그라운드 루프나 테스트에서 불러도 안전하다.

        주문마다 먼저 줄 전체를 막는 이유(_line_gate)를 본다. 그래서 한 번 훑을 때 출발하는 주문은 보통 하나다:
        방금 출발한 주문이 아직 VALIDATE_ORDER(핑키 출발 전)에 있어 다음 주문을 막기 때문이다."""
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
                continue  # 그사이 운영자가 시작했거나 거절했다
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
        """지금 어떤 주문도 출발하면 안 되는 이유({code, message, waiting_for}), 없으면 None."""
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

    # ---- 백그라운드 루프 -------------------------------------------------------------------------------

    def start(self) -> "OrderDispatcher":
        """백그라운드 스레드를 한 번만 띄운다(이미 떠 있으면 그대로)."""
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True, name="order-dispatcher")
            self._thread.start()
        return self

    def _run(self) -> None:
        """interval_s마다 tick(). 한 번의 오류로 루프가 끝나지 않게 하고, 그 이유를 화면에 DISPATCH_ERROR로 보인다."""
        while not self._stop.wait(self.interval_s):
            try:
                self.tick()
            except Exception as exc:  # noqa: BLE001 (a pass must never end the loop; the screen shows why)
                with self._lock:
                    self._state = {**self._state, "checked_at": utc_now_iso(),
                                   "blocked": {"order_id": None, "code": "DISPATCH_ERROR", "message": str(exc)}}

    def close(self) -> None:
        """루프를 멈추고 스레드를 최대 2초 기다린다."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
