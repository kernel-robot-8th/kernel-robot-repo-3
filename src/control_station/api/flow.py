"""주문 진행 화면: 주문·핑키·팔 요약, 경보 확인, 모의 세계 조작"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from control_station.flow_view import PINKY_NAMES, build_flow_view
from control_station.mock_world import FAULT_KINDS
from control_station.pinky_fleet import PinkyFleetError
from control_station.store import TaskNotFoundError

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_flow(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """주문 진행 요약 (변경 없음)"""
    devices = getattr(h.app, "order_devices", None)
    statuses: dict[str, dict[str, Any]] = {}
    for pinky_id in PINKY_NAMES:
        if devices is None:
            statuses[pinky_id] = {"error": "핑키 관제가 설정되지 않았습니다"}
            continue
        try:
            statuses[pinky_id] = devices.fleet_status(pinky_id)
        except (PinkyFleetError, OSError) as exc:
            statuses[pinky_id] = {"error": str(exc)}
    flow = getattr(h.app, "order_flow", None)
    arm_commands = getattr(h.app, "arm_commands", None)
    view = build_flow_view(
        h.app.task_store.recent_orders(),
        statuses,
        getattr(h.app, "arms", None),
        flow.station_owner if flow is not None else None,
        arm_commands.workers() if arm_commands is not None else None,
    )
    world = getattr(h.app, "mock_world", None)
    view["mock_world"] = world.snapshot() if world is not None else None
    view["mock_fault_kinds"] = sorted(FAULT_KINDS) if world is not None else []
    view["arms_error"] = getattr(h.app, "arms_error", None)
    view["station_held_by_stopped"] = bool(flow is not None and flow.station_owner in flow.held_by_stopped)
    h._json(200, view)


def post_alert_ack(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """멈춘 주문의 경보를 운영자가 확인함 (작업 상태는 그대로)"""
    task_id = parts[3]
    note = body.get("note", "운영자 확인")
    if set(body) - {"note"} or not isinstance(note, str) or len(note) > 200:
        h._json_error(400, "INVALID_ALERT_ACK", "body may have only note (up to 200 characters)")
        return
    try:
        task = h.app.task_store.acknowledge_alert(task_id, note.strip() or "운영자 확인")
    except TaskNotFoundError:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    flow = getattr(h.app, "order_flow", None)
    freed = flow.free_spot(task_id) if flow is not None else False
    h._json(200, {"task_id": task["id"], "acknowledged": True, "loading_spot_freed": freed})


def post_mock_world(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """모의 세계 조작: 사고 넣기(inject) 또는 재고 채우기(restock). 모의 세계가 없으면 409."""
    world = getattr(h.app, "mock_world", None)
    if world is None:
        h._json_error(409, "MOCK_WORLD_DISABLED", "모의 세계를 쓰지 않는 실행입니다")
        return
    action = body.get("action")
    if action == "restock" and set(body) == {"action"}:
        world.restock()
    elif action == "inject" and set(body) <= {"action", "fault", "argument"} and body.get("fault") in FAULT_KINDS:
        argument = body.get("argument", "")
        if not isinstance(argument, str) or len(argument) > 40:
            h._json_error(400, "INVALID_MOCK_REQUEST", "argument must be text up to 40 characters")
            return
        world.inject(body["fault"], argument.strip())
    else:
        h._json_error(400, "INVALID_MOCK_REQUEST",
                      "body is {action: restock} or {action: inject, fault: " + "|".join(sorted(FAULT_KINDS)) + ", argument}")
        return
    h._json(200, {"mock_world": world.snapshot()})
