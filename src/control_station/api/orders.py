"""주문 창과 주문 접수 API: control_station.orders.desk.OrderDesk 위의 얇은 HTTP 층.

GET  /api/catalog                    주문할 수 있는 물건, 아직 팔 수 있는 수량, 수령 구역(참고용)
GET  /api/inventory                  선반 재고: 선반 위 수량, 예약, 주문 가능 수량, 최근 변화
POST /api/inventory/{object}         작업자의 재고 수정 {mode: auto|closed|manual, count}
POST /api/orders                     {items: [{object_class, quantity}], receiving_zone?} -> 대기 중인 주문
                                     (구역이 없으면 시작할 때 배정하고, 구역을 보내면 작업자가 고른 것으로 저장)
                                     Idempotency-Key 머리글: 같은 키로 다시 보내면 같은 주문을 돌려줌(200)
GET  /api/orders                     접수 창에서 시작을 기다리는 주문, 오래된 순
GET  /api/dispatch                   자동 출발: 켜짐 여부, 줄이 무엇을 기다리는지, 건너뛴 주문
POST /api/dispatch                   {auto: true|false} 자동 출발 켜기·끄기
GET  /api/orders/{id}/assignment     지금 시작하면 어느 핑키·팔·수령 구역이 맡는지와 그 이유
POST /api/orders/{id}/start          수령 구역과 핑키를 배정하고 주문 흐름을 시작
POST /api/orders/{id}/reject         대기 중인 주문을 거절
GET  /api/orders/{id}/track          고객 화면: 지금 단계를 쉬운 말로, 수령 확인을 누를 수 있는지
POST /api/orders/{id}/arrival-confirm 마커 없는 핑키가 적재 자리에 선 것을 작업자가 눈으로 확인함; 적재 시작
POST /api/orders/{id}/pickup-confirm 고객이 수령 구역에서 주문을 받아 감; 핑키는 주차장으로 돌아감

검증·배정·상태 변경은 모두 OrderDesk가 하고, 그 오류(OrderDeskError)의 상태 코드·오류 코드를 그대로 응답한다.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from control_station.orders.desk import OrderDeskError
from control_station.orders.task_runner import TaskExecutionConflict
from control_station.storage.store import InvalidStateTransition, TaskNotFoundError

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def _desk_error(h: ControlStationHandler, exc: OrderDeskError) -> None:
    """OrderDeskError를 그 상태 코드·오류 코드·추가 정보(details) 그대로 오류 응답으로 보낸다."""
    h._json_error(exc.status, exc.error_code, str(exc), **exc.details)


def get_catalog(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 창: 주문할 수 있는 물건·남은 수량·수령 구역"""
    h._json(200, h.app.order_desk.catalog())


def _inventory_view(app) -> dict[str, Any]:
    """재고 화면 본문. 카메라로 센 재고(app.camera_inventory: 탑 캠 선반 영역·정면 캠 영역)가 있으면
    camera_stock으로 함께 싣는다."""
    view = app.inventory.snapshot()
    camera_inventory = getattr(app, "camera_inventory", None)
    if camera_inventory is not None:
        view["camera_stock"] = camera_inventory()
    return view


def get_inventory(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """선반 재고: 물건별 선반 수량, 예약, 주문 가능 수량, 품절 여부와 최근 변화"""
    h._json(200, _inventory_view(h.app))


def post_inventory(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """재고 수정(작업자): {mode: auto|closed|manual, count?} — 카메라 기준, 판매 중지, 직접 센 수량"""
    if not set(body) <= {"mode", "count"} or "mode" not in body:
        h._json_error(400, "INVENTORY_OVERRIDE_INVALID", "mode(auto, closed, manual)와 manual일 때 count를 보내 주세요")
        return
    try:
        h.app.inventory.set_override(parts[2], body["mode"], body.get("count"))
    except ValueError as exc:
        h._json_error(400, "INVENTORY_OVERRIDE_INVALID", str(exc))
        return
    h._json(200, _inventory_view(h.app))


def get_orders(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 접수 창: 시작을 기다리는 주문"""
    dispatcher = getattr(h.app, "order_dispatcher", None)
    h._json(200, {"orders": h.app.order_desk.pending(),
                  "dispatch": dispatcher.snapshot() if dispatcher is not None else None})


def get_dispatch(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """자동 출발 상태: 켜짐 여부, 줄이 기다리는 이유, 건너뛴 주문, 최근 자동 출발"""
    dispatcher = getattr(h.app, "order_dispatcher", None)
    if dispatcher is None:
        h._json_error(404, "DISPATCH_NOT_CONFIGURED", "자동 출발이 설정되지 않았습니다")
        return
    h._json(200, {"dispatch": dispatcher.snapshot()})


def post_dispatch(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """자동 출발 켜기·끄기: {auto: true|false}. 끄면 이미 출발한 주문은 그대로 진행"""
    dispatcher = getattr(h.app, "order_dispatcher", None)
    if dispatcher is None:
        h._json_error(404, "DISPATCH_NOT_CONFIGURED", "자동 출발이 설정되지 않았습니다")
        return
    if set(body) != {"auto"} or not isinstance(body["auto"], bool):
        h._json_error(400, "DISPATCH_INVALID", "auto(true 또는 false)만 보내 주세요")
        return
    h._json(200, {"dispatch": dispatcher.set_enabled(body["auto"])})


def get_assignment(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """배정 미리보기: 지금 시작하면 어느 핑키·팔·수령 구역이 맡는지와 그 이유"""
    try:
        h._json(200, h.app.order_desk.preview(parts[2]))
    except OrderDeskError as exc:
        _desk_error(h, exc)


def post_order(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """주문 접수: 핑키·수령 구역 없이 접수 상태로 저장(수령 구역은 시작 때 배정)"""
    unsupported = set(body) - {"items", "receiving_zone"}
    if unsupported:
        h._json_error(400, "ORDER_INVALID", "지원하지 않는 필드: " + ", ".join(sorted(unsupported)))
        return
    key = h.headers.get("Idempotency-Key")
    if key is not None and not _valid_idempotency_key(key):
        h._json_error(400, "IDEMPOTENCY_KEY_INVALID", "Idempotency-Key는 1~200자의 영문·숫자·기호(공백 없이)여야 해요")
        return
    try:
        order, made = h.app.order_desk.receive_once(body.get("items"), body.get("receiving_zone"), key)
    except OrderDeskError as exc:
        _desk_error(h, exc)
        return
    # 첫 응답을 받지 못해 다시 보낸 요청은 이미 만든 주문을 받는다. 그때는 201 대신 200이다.
    h._json(201 if made else 200, {"order": order, "replayed": not made})


def _valid_idempotency_key(key: str) -> bool:
    """1–200자의 출력 가능한 ASCII(33–126: 공백·제어 문자·한글 제외)만 키로 받는다."""
    return 0 < len(key) <= 200 and all(33 <= ord(char) <= 126 for char in key)


def get_track(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """고객 앱: 내 주문이 어디쯤인지와 수령 확인 가능 여부"""
    try:
        h._json(200, h.app.order_desk.track(parts[2]))
    except OrderDeskError as exc:
        _desk_error(h, exc)


def post_order_action(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 시작·거절·도착 확인·수령 확인.

    수령·도착 확인은 202(핑키 쪽 동작은 이어서 일어남)다. 시작은 실제로 시작했으면 202, 상태가 바뀌지 않았으면
    (이미 실행 중 등) 200이다.
    작업 실행기(TaskExecutionConflict)나 저장소(상태 전이 불가, 작업 없음)의 거부는 409로 돌려준다."""
    order_id, action = parts[2], parts[3]
    try:
        if action == "pickup-confirm":
            h._json(202, h.app.order_desk.confirm_pickup(order_id))
        elif action == "arrival-confirm":
            h._json(202, h.app.order_desk.confirm_arrival(order_id))
        elif action == "start":
            order, changed = h.app.order_desk.start(order_id)
            h._json(202 if changed else 200, {"order": order})
        else:
            h._json(200, {"order": h.app.order_desk.reject(order_id)})
    except OrderDeskError as exc:
        _desk_error(h, exc)
    except TaskExecutionConflict as exc:
        h._json_error(409, exc.error_code, str(exc))
    except (InvalidStateTransition, TaskNotFoundError) as exc:
        h._json_error(409, "ORDER_CONTROL_ERROR", str(exc))
