"""Order window and order intake API: a thin HTTP layer over control_station.order_desk.OrderDesk.

GET  /api/catalog                    what can be ordered, how many can still be sold, receiving zones
GET  /api/inventory                  shelf inventory: on the shelves, reserved, available, recent changes
POST /api/orders                     {items: [{object_class, quantity}], receiving_zone} -> a waiting order
GET  /api/orders                     orders waiting in the intake, oldest first
GET  /api/orders/{id}/assignment     which Pinky and arms would serve it now, and why (changes nothing)
POST /api/orders/{id}/start          assign the Pinky and start the order flow
POST /api/orders/{id}/reject         turn down a waiting order
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from control_station.order_desk import OrderDeskError
from control_station.store import InvalidStateTransition, TaskNotFoundError
from control_station.task_orchestrator import TaskExecutionConflict

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def _desk_error(h: ControlStationHandler, exc: OrderDeskError) -> None:
    h._json_error(exc.status, exc.error_code, str(exc), **exc.details)


def get_catalog(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 창: 주문할 수 있는 물건·남은 수량·수령 구역"""
    h._json(200, h.app.order_desk.catalog())


def get_inventory(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """선반 재고: 물건별 선반 수량, 예약, 주문 가능 수량, 품절 여부와 최근 변화"""
    h._json(200, h.app.inventory.snapshot())


def get_orders(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 접수 창: 시작을 기다리는 주문"""
    h._json(200, {"orders": h.app.order_desk.pending()})


def get_assignment(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """배정 미리보기: 지금 시작하면 어느 핑키·팔이 맡는지와 그 이유"""
    try:
        h._json(200, h.app.order_desk.preview(parts[2]))
    except OrderDeskError as exc:
        _desk_error(h, exc)


def post_order(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """주문 접수: 핑키 없이 접수 상태로 저장"""
    unsupported = set(body) - {"items", "receiving_zone"}
    if unsupported:
        h._json_error(400, "ORDER_INVALID", "지원하지 않는 필드: " + ", ".join(sorted(unsupported)))
        return
    try:
        order = h.app.order_desk.receive(body.get("items"), body.get("receiving_zone"))
    except OrderDeskError as exc:
        _desk_error(h, exc)
        return
    h._json(201, {"order": order})


def post_order_action(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 시작·거절"""
    order_id, action = parts[2], parts[3]
    try:
        if action == "start":
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
