"""주문 창구: 주문 창에서 들어온 주문은 운영자(또는 자동 출발)가 시작할 때까지 여기서 기다린다.

주문을 받으면 핑키 없이, 그리고 운영자가 지정하지 않았다면 수령 구역도 없이 저장한다. 미리보기는 지금 시작하면
어느 핑키·팔·수령 구역이 맡을지와 그 이유를 보여 준다. 시작하면 그 순간의 상태로 같은 선택을 다시 하고, 주문에
기록한 뒤 작업 실행기(MockTaskOrchestrator)에 넘긴다. 팔 계획은 주문 흐름 안에서 선반 확인 뒤에 다시 세우므로,
미리보기는 조언일 뿐 예약이 아니다. 여기서는 장치를 움직이지 않는다.

손님 쪽(주문 창·추적 화면)과 운영자 쪽(접수 목록·미리보기·시작·거절) API가 모두 이 클래스를 부른다.
선택 규칙 자체는 orders/assignment.py의 순수 함수다.
"""

from __future__ import annotations

import hashlib
import json
import threading
from collections import Counter
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from typing import Any

from control_station.api.common import validate_order_items
from control_station.orders.assignment import (
    ArmPlanError,
    PickStats,
    PinkyCandidate,
    ShipmentError,
    ZoneAssignment,
    ZoneCandidate,
    assign_fleet,
    choose_pinky,
    choose_zone,
    plan_arms,
    schedule_loading,
    split_into_shipments,
)
from control_station.orders.flow import OrderDevices
from control_station.orders.planning import FleetPlanning, load_fleet_planning
from control_station.orders.receiving_zones import ReceivingZone
from control_station.pinky.fleet_client import PinkyFleetError
from control_station.storage.store import (
    PINKY_IDS,
    PINKY_NAMES,
    InvalidStateTransition,
    PinkyUnavailable,
    TaskStore,
    ZoneUnavailable,
    shipment_of,
)


class OrderDeskError(RuntimeError):
    """창구가 거절한 요청. `status`는 API가 답할 HTTP 상태, details는 응답 본문에 덧붙일 값이다."""

    def __init__(self, error_code: str, message: str, status: int = 409, **details: object) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.status = status
        self.details = details


# 손님에게 보이는 주문 단계(키, 글자)와, 주문 흐름의 단계가 각각 어느 손님 단계에 속하는지.
_CUSTOMER_STAGES = (("received", "접수"), ("preparing", "준비"), ("pinky_coming", "핑키 출발"),
                    ("loading", "싣는 중"), ("delivering", "가는 중"), ("arrived", "도착"), ("done", "완료"))
_STAGE_INDEX = {key: index for index, (key, _) in enumerate(_CUSTOMER_STAGES)}
_STEP_STAGE = {
    "VALIDATE_ORDER": "preparing", "CHECK_SHELF_STOCK": "preparing", "PLAN_ARM_SEQUENCE": "preparing",
    "WAIT_PINKY_ARRIVAL": "pinky_coming", "CONFIRM_DOCKING": "loading", "PICK_ITEM": "loading",
    "VERIFY_TRANSFER": "loading", "VERIFY_LOAD_TOTAL": "loading", "NOTIFY_PINKY_RECEIVED": "delivering",
    "WAIT_AT_RECEIVING_ZONE": "delivering", "CONFIRM_PICKUP": "arrived", "WAIT_PINKY_PARKED": "done",
    "FINALIZE_TASK": "done",
}
# 손님 단계 → (제목, 설명). {zone}, {moved}, {total}은 track()이 채운다.
_CUSTOMER_TEXT = {
    "received": ("주문을 받았어요", "곧 준비를 시작해요"),
    "preparing": ("주문을 준비하고 있어요", "선반 재고를 확인하고 로봇을 배정해요"),
    "pinky_coming": ("핑키가 물건을 실으러 가고 있어요", "적재 자리에 도착하면 로봇 팔이 실어요"),
    "loading": ("로봇 팔이 물건을 싣고 있어요", "{moved}/{total}개 실었어요"),
    "delivering": ("핑키가 {zone}로 가고 있어요", "도착하면 알려 드릴게요"),
    "arrived": ("{zone}에 도착했어요", "물건을 받은 뒤 아래 버튼을 눌러 주세요"),
    "done": ("주문이 끝났어요", "이용해 주셔서 감사해요"),
    "canceled": ("주문이 취소됐어요", "다시 주문할 수 있어요"),
    "problem": ("주문 처리에 문제가 생겼어요", "직원이 확인하고 있어요"),
}
# 주문이 시작되어 서버가 구역을 정하기 전까지 손님에게 구역 대신 보이는 말.
ZONE_PENDING_NOTE = "받을 곳은 로봇이 출발할 때 알려 드려요"
# 구역이 배정되기 전 주문의 목적지 글자(운영자 화면에 보인다).
ZONE_PENDING_LABEL = "수령 구역 (시작 때 배정)"


def _request_hash(order_items: list[dict[str, Any]], zone_id: object) -> str:
    """주문 내용의 지문(sha256). 같은 키로 온 요청이 재전송인지, 다른 주문인지 가르는 데 쓴다."""
    body = json.dumps({"items": order_items, "receiving_zone": zone_id}, sort_keys=True, ensure_ascii=False)
    return hashlib.sha256(body.encode("utf-8")).hexdigest()


def _customer_stage(status: str, step_type: str | None) -> str:
    """작업 상태와 지금 단계 → 손님 단계 키. 멈춘 주문은 손님에게 "problem"으로만 보인다."""
    if status == "PENDING":
        return "received"
    if status == "CANCELED":
        return "canceled"
    if status == "FAILED":
        return "problem"
    if status == "SUCCEEDED":
        return "done"
    return _STEP_STAGE.get(step_type or "", "preparing")


# 평소처럼 주문할 수 없는 상품 아래에 주문 창이 보이는 말(재고 상태 → 글자).
_PRODUCT_NOTICE = {"unknown": "재고 확인 중이라 주문을 닫았습니다", "closed": "판매 중지", "sold_out": "품절"}


def _topic(word: str) -> str:
    """낱말에 주제 조사를 붙인다: 받침이 있으면 은, 없으면 는(사과는, 당근은). 한글로 끝나지 않으면 "은(는)"."""
    last = word[-1:] if word else ""
    if "가" <= last <= "힣":
        return word + ("은" if (ord(last) - ord("가")) % 28 else "는")
    return word + "은(는)"


def _start_of_today() -> str:
    """오늘 현지 자정을 UTC ISO 시각으로. 저장소 시각과 문자열로 비교할 수 있다('오늘 운행 횟수'의 기준)."""
    midnight = datetime.now().astimezone().replace(hour=0, minute=0, second=0, microsecond=0)
    return midnight.astimezone(timezone.utc).isoformat(timespec="microseconds")


@dataclass
class OrderDesk:
    """주문 접수·미리보기·시작·추적을 맡는 창구. 상태는 모두 저장소에 있고, 이 객체는 접수 잠금만 가진다."""

    store: TaskStore
    orchestrator: Any  # .start(task_id) -> (task, changed)
    devices: OrderDevices
    arms: Any  # control_station.arms.config 레지스트리(.arms: 설정 순서의 팔 dict 목록)
    policies: Any  # control_station.arms.policies 레지스트리
    zones: Mapping[str, ReceivingZone]
    pick_stats: Callable[[], PickStats]
    planning: FleetPlanning = field(default_factory=load_fleet_planning)
    today: Callable[[], str] = _start_of_today  # '오늘'의 시작 시각(시험에서 바꿔 끼운다)
    # 선반 재고(inventory.InventoryMonitor.snapshot). 있으면 주문 창은 팔 수 있는 만큼(선반 − 열린 주문)만 팔고
    # 그 이상은 거절한다. 없으면 창구가 선반을 직접 읽는다.
    inventory: Callable[[], dict[str, Any]] | None = None
    # OrderFlow.confirm_pickup: 주문의 핑키가 수령 구역에서 기다리고 있었으면 True.
    pickup: Callable[[str], bool] | None = None
    # OrderFlow.confirm_arrival: 주문이 운영자의 핑키 도착 확인을 기다리고 있었으면 True.
    arrival: Callable[[str], bool] | None = None
    # 재고 확인과 주문 저장을 한 덩어리로 묶는 잠금(두 창이 마지막 하나를 동시에 팔지 못하게).
    _intake: threading.Lock = field(default_factory=threading.Lock, repr=False)

    # ---- 주문 창 ----------------------------------------------------------------------------------------------

    def catalog(self) -> dict[str, Any]:
        """주문할 수 있는 것(학습 정책이 있는 물체), 팔 수 있는 수량과 상태, 수령 구역(참고용) — GET /api/catalog."""
        if self.policies is None:
            return {"products": [], "stock_error": "정책 정의 파일이 없어 주문을 받을 수 없습니다", "zones": []}
        zones = [zone.as_dict() for zone in self.zones.values()]
        if self.inventory is not None:
            view = self.inventory()
            products = [{"object_class": row["object_class"], "label": row["label"], "available": row["available"],
                         "status": row["status"], "notice": _PRODUCT_NOTICE.get(row["status"])}
                        for row in view["products"]]
            notice = None
            if view["state"] == "unknown" and any(row["status"] == "unknown" for row in view["products"]):
                notice = f"재고를 확인하지 못해 해당 물건의 주문을 잠시 닫았습니다 ({view['reason']})"
            return {"products": products, "stock_error": notice, "notice": notice, "zones": zones}
        stock, stock_error = self._stock()
        on_shelves: Counter = Counter()
        for counts in stock.values():
            on_shelves.update(counts)
        products: dict[str, dict[str, Any]] = {}
        for policy in self.policies.policies.values():
            products.setdefault(policy["object_class"], {
                "object_class": policy["object_class"],
                "label": policy["label"],
                "available": on_shelves[policy["object_class"]] if stock_error is None else None,
            })
        return {
            "products": list(products.values()),
            "stock_error": stock_error,
            "zones": [zone.as_dict() for zone in self.zones.values()],
        }

    def receive(self, items: object, zone_id: object = None) -> dict[str, Any]:
        """새 주문을 접수 목록에 저장한다(상태 PENDING, 아직 핑키 없음).

        손님은 수령 구역을 고르지 않는다: `start`가 정한다(choose_zone). 여기서 구역을 주면 작업자의 선택으로 보고
        그대로 둔다."""
        return self.receive_once(items, zone_id)[0]

    def receive_once(self, items: object, zone_id: object = None, idempotency_key: str | None = None,
                     ) -> tuple[dict[str, Any], bool]:
        """다시 보낸 요청이 주문을 두 번 만들지 못하는 `receive`: (주문, 지금 만들었는지).

        Idempotency-Key가 있으면 첫 요청이 주문을 만들고, 같은 키·같은 본문의 뒤 요청은 모두 그 주문을 돌려받는다
        (지금 만들었는지 = False). 그 주문 때문에 재고가 바닥났더라도 마찬가지다. 같은 키에 다른 본문은 거절한다
        (IDEMPOTENCY_KEY_REUSED): 두 번째 주문이 첫 주문 뒤에 숨어 버리기 때문이다.
        거절: ORDER_INVALID, RECEIVING_ZONE_*, ITEM_CLOSED, OUT_OF_STOCK, ITEM_TOO_BIG_FOR_PINKY."""
        try:
            order_items = validate_order_items(items, self.policies)
        except ValueError as exc:
            raise OrderDeskError("ORDER_INVALID", str(exc), status=400) from exc
        request_hash = _request_hash(order_items, zone_id)
        # 확인과 예약(주문 저장)을 한 번에 한다: 두 창이 마지막 하나를 둘 다 팔 수 없게.
        with self._intake:
            if idempotency_key is not None:
                known = self.store.order_request(idempotency_key)
                if known is not None:
                    stored_hash, order_id = known
                    if stored_hash != request_hash:
                        raise OrderDeskError("IDEMPOTENCY_KEY_REUSED",
                                             "같은 주문 키로 다른 주문이 왔어요. 새 주문은 새 키로 보내 주세요", status=422)
                    order = self.store.get_task(order_id)
                    if order is not None:
                        return order, False
            if not self.zones:
                raise OrderDeskError("RECEIVING_ZONE_NOT_CONFIGURED", "수령 구역이 설정되지 않아 주문을 받을 수 없어요")
            zone = None if zone_id is None else self._zone(zone_id)
            summary = " · ".join(f"{self._label(item['object_class'])} {item['quantity']}" for item in order_items)
            self._check_available(order_items)
            # 쟁반 하나에 다 안 실리면 처음부터 운송분 단계로 만든다.
            shipments = self._shipments({"order_items": order_items})
            order = self.store.create_task(
                zone.label if zone else ZONE_PENDING_LABEL, f"{summary} 적재", order_items=order_items,
                receiving_zone=zone.zone_id if zone else None, defer_pinky=True, zone_steps=True,
                idempotency=None if idempotency_key is None else (idempotency_key, request_hash),
                shipments=[list(shipment.units) for shipment in shipments] if len(shipments) > 1 else None,
            )
            return order, True

    def _shipments(self, order: dict[str, Any]) -> tuple[Any, ...]:
        """주문의 핑키 적재분(쟁반 칸 수와 크기는 config/fleet-planning.json), 무거운 것부터."""
        try:
            return split_into_shipments(self._units(order), self.planning.pinky_capacity, self.planning.size_of,
                                        self._label)
        except ShipmentError as exc:
            raise OrderDeskError(exc.error_code, str(exc), status=400) from exc

    def _check_available(self, order_items: list[dict[str, Any]]) -> None:
        """지금 팔 수 없는 것은 거절한다: 닫힌 물체(재고 모름, 또는 운영자가 중지), 또는 열린 주문을 빼고 선반이
        채울 수 있는 것보다 많은 수량. 거절한 물체를 모두 한 메시지에 담는다. 재고 장부가 없으면 확인하지 않는다."""
        if self.inventory is None:
            return
        rows = {row["object_class"]: row for row in self.inventory()["products"]}
        closed, short = [], []
        for item in order_items:
            row = rows.get(item["object_class"]) or {"status": "unknown", "available": None}
            subject = _topic(self._label(item["object_class"]))
            if row["status"] == "unknown":
                closed.append(f"{subject} 재고를 확인하지 못해 주문을 닫았습니다")
            elif row["status"] == "closed":
                closed.append(f"{subject} 지금 판매하지 않습니다")
            elif item["quantity"] > (row["available"] or 0):
                short.append(f"{subject} 품절입니다" if not row["available"] else f"{subject} {row['available']}개 남았습니다")
        if closed or short:
            raise OrderDeskError("ITEM_CLOSED" if closed else "OUT_OF_STOCK", " · ".join(closed + short))

    # ---- 운영자의 접수 목록 -------------------------------------------------------------------------------

    def pending(self) -> list[dict[str, Any]]:
        """운영자를 기다리는 주문들, 오래된 것부터."""
        return self.store.pending_orders()

    def preview(self, order_id: str) -> dict[str, Any]:
        """이 주문을 지금 시작하면 누가 맡을지와 그 이유. 아무것도 바꾸지 않는다(GET /api/orders/{id}/assignment).

        `pinky`와 `arms`는 핑키 한 대 주문을 설명한다. `fleet`은 큰 주문을 차례로 도는 운송분으로 나누고(쟁반 칸 수는
        config/fleet-planning.json), 운송분마다 팔을 계획하고 적재 시간을 잰다. 첫 운송분의 핑키는 `start`가 고를
        그 핑키다. 뒤 운송분의 핑키는 그 운송분이 시작할 때 다시 고르므로, 미리보기는 앞 운행을 센 최선의 추측이다."""
        order = self._pending_order(order_id)
        units = self._units(order)
        candidates = self.pinky_candidates()
        pinky = choose_pinky(candidates)
        zone = self.zone_plan(order).as_dict()
        try:
            shipments = split_into_shipments(units, self.planning.pinky_capacity, self.planning.size_of, self._label)
        except ShipmentError as exc:
            error = {"code": exc.error_code, "message": str(exc)}
            return {"order_id": order_id, "pinky": pinky.as_dict(), "zone": zone, "arms": {"error": error},
                    "fleet": {"error": error}}
        fleet = assign_fleet(shipments, candidates)
        stock, stock_error = self._stock()
        guesses = self._later_pinkys(candidates, len(shipments))
        arm_ids = [arm["arm_id"] for arm in self.arms.arms]
        stats = self.pick_stats()
        trips = []
        for index, trip in enumerate(fleet.trips):  # 뒤 운송분은 앞 운송분이 선반에 남긴 것으로 계획한다
            entry = trip.as_dict()
            if len(fleet.trips) > 1:  # 운송분은 차례로 돈다: 각 핑키는 그 운송분이 시작할 때 고른다
                guess = guesses[index]
                entry.update(pinky_id=guess.pinky_id, pinky_name=PINKY_NAMES.get(guess.pinky_id or ""),
                             reason=guess.summary if index == 0 else f"앞 운송이 끝나면 다시 고름 · 지금 기준 {guess.summary}")
            for unit in entry["units"]:  # 쟁반 그림에 필요한 것: 이름과 차지하는 칸 수
                unit["label"] = self._label(unit["object_class"])
                unit["size"] = self.planning.size_of(unit["object_class"])
            if stock_error is not None:
                entry["arms"] = {"error": {"code": "SHELF_STOCK_UNAVAILABLE", "message": stock_error}}
            else:
                try:
                    plan = plan_arms(trip.shipment.units, stock, self.arms.arms, self.policies.resolve,
                                     self._label, stats)
                except ArmPlanError as exc:
                    entry["arms"] = {"error": {"code": exc.error_code, "message": str(exc)}}
                else:
                    for pick in plan.picks:
                        stock[pick.shelf_id][pick.object_class] -= 1
                    entry["arms"] = plan.as_dict()
                    entry["schedule"] = schedule_loading(plan.picks, self.planning.timing, arm_ids,
                                                         overlap=self.planning.arms_overlap).as_dict()
            trips.append(entry)
        sequential = len(trips) > 1
        summary = (f"운송 {len(trips)}번으로 나눠 차례로 · " + ", ".join(
            f"{index}. {trip['pinky_name'] or '핑키 대기'} {trip['load']}칸" for index, trip in enumerate(trips, start=1))
                   if sequential else fleet.summary)
        arms = trips[0]["arms"] if not sequential else {
            "error": {"code": "MULTI_SHIPMENT", "message": f"{summary} · 운송별 팔 계획은 fleet에 있습니다"}}
        return {"order_id": order_id, "pinky": pinky.as_dict(), "zone": zone, "arms": arms,
                "fleet": {"summary": summary, "complete": all(trip["pinky_id"] for trip in trips),
                          "capacity": self.planning.pinky_capacity, "trips": trips, "sequential": sequential}}

    def start(self, order_id: str) -> tuple[dict[str, Any], bool]:
        """수령 구역과 핑키를 지금 정하고(주문에 없을 때만) 주문 흐름을 시작한다(POST /api/orders/{id}/start).

        둘 다 쓰기 전에 둘 다 확인하므로, 확인 단계의 거절(ZONE_BUSY, PINKY_UNAVAILABLE, ORDER_NEEDS_SHIPMENTS)은
        주문을 그대로 둔다. 이미 시작한 주문이면 실행기에 그대로 넘긴다(시작 요청 반복은 멱등)."""
        order = self._pending_order(order_id, allow_started=True)
        if order["status"] == "PENDING":
            self._refuse_unsplit(order)
            zone = self.zone_plan(order)
            if zone.zone_id is None:
                raise OrderDeskError("ZONE_BUSY", zone.summary, zone=zone.as_dict())
            assignment = None
            if order["pinky_id"] is None:
                assignment = choose_pinky(self.pinky_candidates())
                if assignment.pinky_id is None:
                    raise OrderDeskError("PINKY_UNAVAILABLE", assignment.summary, assignment=assignment.as_dict())
            # 구역을 먼저 쓴다. 접수 대기 주문의 구역은 다른 주문을 막지 않지만 핑키는 막기 때문이다. 그래서 두 쓰기
            # 사이의 경쟁으로 거절되더라도 핑키가 이 주문에 묶이지 않는다(2026-11-25 수정: 예전에는 핑키를 먼저 써서,
            # 구역 쓰기가 실패하면 핑키만 배정된 접수 대기 주문이 남았다).
            if order["receiving_zone"] is None:
                try:
                    self.store.assign_zone(order_id, zone.zone_id, self.zones[zone.zone_id].label, zone.as_dict())
                except ZoneUnavailable as exc:  # 고른 뒤 쓰기 전에 다른 주문이 거기서 시작했다
                    raise OrderDeskError("ZONE_BUSY", f"수령 구역이 모두 사용 중이에요: {exc}") from exc
            if assignment is not None:
                try:
                    self.store.assign_pinky(order_id, assignment.pinky_id, assignment.as_dict())
                except PinkyUnavailable as exc:  # 고른 뒤 쓰기 전에 다른 주문이 가져갔다
                    raise OrderDeskError("PINKY_UNAVAILABLE", str(exc)) from exc
        return self.orchestrator.start(order_id)

    @staticmethod
    def _later_pinkys(candidates: list[PinkyCandidate], count: int) -> list[Any]:
        """운송분들을 지금 차례로 돌린다고 칠 때 운송분마다의 핑키 선택. 앞 운송분의 핑키는 오늘 운행이 하나 늘고
        방금 끝난 것으로 쳐서 다음 선택에 반영한다."""
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        pool = list(candidates)
        choices = []
        for _ in range(count):
            choice = choose_pinky(pool)
            choices.append(choice)
            pool = [replace(c, trips_today=c.trips_today + 1, last_finished_at=now) if c.pinky_id == choice.pinky_id
                    else c for c in pool]
        return choices

    def _refuse_unsplit(self, order: dict[str, Any]) -> None:
        """운송분이 생기기 전에 받은 주문(또는 이전 /api/tasks 경로로 받은 주문)은 적재분 하나의 단계만 갖는다.
        이제 쟁반 하나에 다 실리지 않으면 그대로는 돌 수 없으니 ORDER_NEEDS_SHIPMENTS로 거절한다."""
        needed = len(self._shipments(order))
        have = max((shipment_of(step) for step in order["steps"]), default=1)
        if needed > have:
            raise OrderDeskError(
                "ORDER_NEEDS_SHIPMENTS",
                f"이 주문은 핑키 쟁반 {self.planning.pinky_capacity}칸에 다 실리지 않아 운송 {needed}번이 필요한데, "
                "나누지 않은 채로 접수돼 있습니다. 거절하고 다시 주문해 주세요",
            )

    def assign_next_shipment(self, order_id: str, shipment: int) -> dict[str, Any]:
        """진행 중인 주문의 뒤 운송분 핑키를 지금 골라 주문에 적는다(OrderFlow의 START_SHIPMENT가 부른다).

        앞 운송분의 핑키는 이제 주차장에 돌아와 있으니 다시 가도 된다. 다만 순위(배터리, 오늘 운행, 마지막 끝난
        시각) 때문에 보통 다른 핑키가 간다. 빈 핑키가 없으면 {"pinky_id": None}을 돌려주고 흐름이 기다린다."""
        # 이 주문 자신이 쥔 핑키는 '진행 중 주문 있음'으로 빼지 않는다.
        candidates = [replace(candidate, holding_order=None) if candidate.holding_order == order_id else candidate
                      for candidate in self.pinky_candidates()]
        assignment = choose_pinky(candidates)
        if assignment.pinky_id is None:
            return {"pinky_id": None, "summary": assignment.summary, "assignment": assignment.as_dict()}
        try:
            self.store.assign_shipment_pinky(order_id, shipment, assignment.pinky_id,
                                             {**assignment.as_dict(), "shipment": shipment})
        except (PinkyUnavailable, InvalidStateTransition) as exc:
            return {"pinky_id": None, "summary": str(exc), "assignment": assignment.as_dict()}
        return {"pinky_id": assignment.pinky_id, "summary": assignment.summary, "assignment": assignment.as_dict()}

    def confirm_arrival(self, order_id: str) -> dict[str, Any]:
        """운영자가 주문의 핑키가 적재 자리에 제대로 섰다고 확인한다([도착 확인]). 마커가 없거나 카메라가 판정하지
        못해 흐름이 사람을 기다릴 때만 받는다(아니면 ARRIVAL_NOT_WAITING)."""
        order = self.store.get_task(order_id)
        if order is None or order.get("order_items") is None:
            raise OrderDeskError("ORDER_NOT_FOUND", f"주문 {order_id}이 없습니다", status=404)
        if self.arrival is None or not self.arrival(order_id):
            raise OrderDeskError("ARRIVAL_NOT_WAITING", "이 주문은 지금 도착 확인을 기다리고 있지 않습니다")
        return {"order_id": order_id, "accepted": True, "message": "도착 확인을 받았습니다. 적재를 시작합니다"}

    def confirm_pickup(self, order_id: str) -> dict[str, Any]:
        """손님이 수령 구역에서 주문을 받았다. 그러면 흐름이 핑키를 주차장으로 돌려보낸다. 핑키가 아직 구역에서
        기다리지 않으면 PICKUP_NOT_READY."""
        order = self.store.get_task(order_id)
        if order is None or order.get("order_items") is None:
            raise OrderDeskError("ORDER_NOT_FOUND", f"주문 {order_id}이 없습니다", status=404)
        if self.pickup is None or not self.pickup(order_id):
            raise OrderDeskError("PICKUP_NOT_READY", "핑키가 아직 수령 구역에서 기다리고 있지 않습니다")
        return {"order_id": order_id, "accepted": True, "message": "수령 확인을 받았습니다. 핑키가 주차장으로 돌아갑니다"}

    def track(self, order_id: str) -> dict[str, Any]:
        """손님이 보는 주문 하나: 어디까지 왔는지 쉬운 말로, 그리고 [받았어요]를 누를 수 있는지(/track 화면)."""
        order = self.store.get_task(order_id)
        if order is None or order.get("order_items") is None:
            raise OrderDeskError("ORDER_NOT_FOUND", f"주문 {order_id}이 없습니다", status=404)
        steps = order["steps"]
        active = next((step for step in steps if step["id"] == order.get("active_step_id")), None)
        stage = _customer_stage(order["status"], active["step_type"] if active else None)
        count = max((int((step.get("detail") or {}).get("shipments") or 1) for step in steps), default=1)
        current = shipment_of(active) if active else (count if order["status"] == "SUCCEEDED" else 1)
        between = False
        if count > 1 and active is not None and order["status"] in {"RUNNING", "VALIDATING"}:
            if active["step_type"] == "WAIT_PINKY_PARKED" and current < count:
                current, between = current + 1, True  # 이 적재분은 수령됐고 다음 적재분이 오는 중이다
            elif active["step_type"] == "START_SHIPMENT":
                between = True
            if between:
                stage = "pinky_coming"
        zone_id = order.get("receiving_zone")
        known = self.zones.get(zone_id or "")
        zone_label = (known.label if known else order.get("destination")) if zone_id else None
        zone = zone_label or "수령 구역"
        mine = [step for step in steps if shipment_of(step) == current]
        moved = sum(1 for step in mine if step["step_type"] == "VERIFY_TRANSFER" and step["status"] == "SUCCEEDED")
        total = sum(1 for step in mine if step["step_type"] == "VERIFY_TRANSFER")
        title, detail = _CUSTOMER_TEXT[stage]
        title, detail = title.format(zone=zone), detail.format(zone=zone, moved=moved, total=total)
        if between:
            done_count = {1: "첫", 2: "두", 3: "세", 4: "네"}.get(current - 1, str(current - 1))
            title, detail = "남은 물건을 가지러 가요", f"{done_count} 번째 운송을 받았어요 · 남은 물건을 준비해요"
        load = Counter(step["detail"]["object_class"] for step in mine if step["step_type"] == "PICK_ITEM")
        return {
            "order_id": order_id, "status": order["status"], "stage": stage,
            "stages": [{"key": key, "label": label, "state": "done" if index < _STAGE_INDEX.get(stage, -1)
                        else "current" if key == stage else "todo"}
                       for index, (key, label) in enumerate(_CUSTOMER_STAGES)],
            "title": title, "detail": detail,
            "items": [{"label": self._label(item["object_class"]), "quantity": item["quantity"]} for item in order["order_items"]],
            "zone": zone_label, "zone_note": None if zone_label else ZONE_PENDING_NOTE,
            "pinky": PINKY_NAMES.get(order.get("pinky_id") or ""),
            "loaded": moved, "total": total, "can_confirm_pickup": stage == "arrived",
            # 큰 주문은 여러 적재분으로 차례로 온다. 적재분마다 따로 수령한다.
            "shipment": current, "shipments": count,
            "shipment_items": ([{"label": self._label(name), "quantity": quantity} for name, quantity in load.items()]
                               if count > 1 else None),
            "created_at": order.get("created_at"), "finished_at": order.get("finished_at"),
        }

    def reject(self, order_id: str) -> dict[str, Any]:
        """시작하지 않은 주문을 거절(CANCELED)한다. 아직 어떤 장치도 움직이지 않았다."""
        self._pending_order(order_id)
        return self.store.cancel_pending_task(order_id)

    # ---- 구역·핑키 선택의 재료 -------------------------------------------------------------------------

    def zone_candidates(self) -> list[ZoneCandidate]:
        """설정의 구역마다: 경로 길이, 지금 쥔 주문, 오늘 사용 횟수."""
        holding = self.store.busy_zones()
        usage = self.store.zone_usage(self.today())
        return [ZoneCandidate(zone.zone_id, zone.label, len(zone.route_labels), holding.get(zone.zone_id),
                              usage.get(zone.zone_id, 0)) for zone in self.zones.values()]

    def zone_plan(self, order: dict[str, Any]) -> ZoneAssignment:
        """이 주문이 지금 갈 구역: 자기 구역(작업자 지정, 확인만 함) 또는 choose_zone이 고른 구역.
        기다리는 주문은 구역을 쥐지 않으므로 자기 자신에게 막히는 일은 없다."""
        return choose_zone(self.zone_candidates(), requested=order.get("receiving_zone"))

    def pinky_candidates(self) -> list[PinkyCandidate]:
        """핑키마다 지금 상태: 쥔 주문, 오늘 운행, 핑키 관제가 알려 준 단계·준비·배터리. 답이 없으면 reachable=False."""
        holding = self.store.busy_pinkies()
        usage = self.store.pinky_usage(self.today())
        candidates = []
        for pinky_id in PINKY_IDS:
            used = usage.get(pinky_id, {})
            common = {
                "pinky_id": pinky_id, "name": PINKY_NAMES[pinky_id], "holding_order": holding.get(pinky_id),
                "trips_today": int(used.get("trips", 0)), "last_finished_at": used.get("last_finished_at"),
            }
            try:
                status = self.devices.fleet_status(pinky_id)
            except (PinkyFleetError, OSError):
                candidates.append(PinkyCandidate(**common, reachable=False))
                continue
            candidates.append(PinkyCandidate(
                **common, phase=status.get("phase"), ready_for_motion=status.get("ready_for_motion"),
                readiness_label=status.get("readiness_label"), battery_percent=status.get("battery_percent"),
            ))
        return candidates

    # ---- 도우미 --------------------------------------------------------------------------------------------

    def _stock(self) -> tuple[dict[str, Counter], str | None]:
        """(선반 → 물체 → 개수, 오류 문장). 못 읽으면 ({}, 이유)."""
        try:
            shelves = self.devices.shelf_stock()["shelves"]
        except Exception as exc:  # 카메라 준비 안 됨, 프레임 없음, 노드 꺼짐: 창이 이유를 보인다
            return {}, f"선반 재고를 읽지 못했습니다: {exc}"
        return {shelf: Counter(counts) for shelf, counts in shelves.items()}, None

    @staticmethod
    def _units(order: dict[str, Any]) -> list[dict[str, Any]]:
        """주문 물체를 한 개씩 편 단위 목록: {object_class, unit(1부터), of(수량)}."""
        return [{"object_class": item["object_class"], "unit": index, "of": item["quantity"]}
                for item in order["order_items"] for index in range(1, int(item["quantity"]) + 1)]

    def _zone(self, zone_id: object) -> ReceivingZone:
        """주문과 함께 이름이 온 구역(작업자 지정). 설정에 없으면 RECEIVING_ZONE_UNKNOWN."""
        if not isinstance(zone_id, str) or zone_id not in self.zones:
            known = ", ".join(self.zones) or "없음"
            raise OrderDeskError("RECEIVING_ZONE_UNKNOWN", f"알 수 없는 수령 구역이에요 (가능: {known})", status=400)
        return self.zones[zone_id]

    def _label(self, object_class: str) -> str:
        """물체 클래스 → 사람이 읽는 이름(정책 파일의 label). 없으면 클래스 그대로."""
        found = self.policies.matches(object_class)
        return found[0]["label"] if found else object_class

    def _pending_order(self, order_id: str, *, allow_started: bool = False) -> dict[str, Any]:
        """주문 하나(접수 상태여야 함, allow_started면 시작한 것도). 없으면 ORDER_NOT_FOUND, 시작했으면 ORDER_NOT_PENDING."""
        order = self.store.get_task(order_id)
        if order is None or order.get("order_items") is None:
            raise OrderDeskError("ORDER_NOT_FOUND", f"주문 {order_id}이 없습니다", status=404)
        if order["status"] != "PENDING" and not allow_started:
            raise OrderDeskError("ORDER_NOT_PENDING", f"주문 {order_id}은 이미 {order['status']} 상태입니다")
        return order
