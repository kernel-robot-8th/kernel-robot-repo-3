"""Order window and order intake API: a thin HTTP layer over control_station.order_desk.OrderDesk.

GET  /api/catalog                    what can be ordered, how many can still be sold, receiving zones (info)
GET  /api/inventory                  shelf inventory: on the shelves, reserved, available, recent changes
POST /api/inventory/{object}         operator override {mode: auto|closed|manual, count}
POST /api/orders                     {items: [{object_class, quantity}], receiving_zone?} -> a waiting order
                                     (no zone: start assigns one; a zone given is an operator's choice)
GET  /api/orders                     orders waiting in the intake, oldest first
GET  /api/orders/{id}/assignment     which Pinky, arms and receiving zone would serve it now, and why
POST /api/orders/{id}/start          assign the receiving zone and the Pinky, start the order flow
POST /api/orders/{id}/reject         turn down a waiting order
GET  /api/orders/{id}/track          the customer's view: stage in plain words, whether pickup can be confirmed
POST /api/orders/{id}/pickup-confirm the customer took the order at the receiving zone; the Pinky goes home
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
    h._json(200, h.app.inventory.snapshot())


def get_orders(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 접수 창: 시작을 기다리는 주문"""
    h._json(200, {"orders": h.app.order_desk.pending()})


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
    try:
        order = h.app.order_desk.receive(body.get("items"), body.get("receiving_zone"))
    except OrderDeskError as exc:
        _desk_error(h, exc)
        return
    h._json(201, {"order": order})


def get_track(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """고객 앱: 내 주문이 어디쯤인지와 수령 확인 가능 여부"""
    try:
        h._json(200, h.app.order_desk.track(parts[2]))
    except OrderDeskError as exc:
        _desk_error(h, exc)


def post_order_action(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 시작·거절·수령 확인"""
    order_id, action = parts[2], parts[3]
    try:
        if action == "pickup-confirm":
            h._json(202, h.app.order_desk.confirm_pickup(order_id))
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
