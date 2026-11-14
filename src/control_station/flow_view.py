"""What the operator screen shows for orders: where each order is, what each robot is doing, and what needs a person.

Built only from stored order steps/events plus the Pinky fleet status; it changes nothing.
"""

from __future__ import annotations

from typing import Any

from control_station.pinky_fleet import STATION_PHASES
from control_station.store import PINKY_NAMES, shipment_of  # noqa: E402  (re-exported for the API)
PHASE_LABELS = {
    "parked": "주차장 대기", "driving": "이동 중", "docking_manual": "5번 도착 · 정지",
    "docking_anchor": "도킹완료 · 위치 재설정 중", "docked_manual": "정지 유지 · 수령 대기",
    "dock_exit": "도킹 탈출 · 후진", "recovering": "장애물 우회 중",
    "course_complete": "코스 완주", "returning": "복귀 중",
}
# The order pipeline on the screen, each stage made of step types.
STAGES = (
    ("ORDER", "주문 접수", ("VALIDATE_ORDER",)),
    ("STOCK", "선반 확인", ("CHECK_SHELF_STOCK",)),
    ("PLAN", "팔 배정", ("PLAN_ARM_SEQUENCE",)),
    ("PINKY", "핑키 도착", ("START_SHIPMENT", "WAIT_PINKY_ARRIVAL", "CONFIRM_DOCKING")),
    ("LOAD", "적재", ("PICK_ITEM", "VERIFY_TRANSFER")),
    ("TOTAL", "총량 확인", ("VERIFY_LOAD_TOTAL",)),
    ("SIGNAL", "완료 신호", ("NOTIFY_PINKY_RECEIVED", "FINALIZE_TASK")),
    # Only orders with a receiving zone have these steps; for the others the stage is left out.
    ("DELIVER", "수령", ("WAIT_AT_RECEIVING_ZONE", "CONFIRM_PICKUP", "WAIT_PINKY_PARKED")),
)
ACTIVE_STEP = frozenset({"COMMAND_SENT", "ACCEPTED", "RUNNING"})
ALERT_EVENTS = frozenset({"PINKY_POSITION_ABNORMAL", "PINKY_FLEET_UNREACHABLE"})
# Events after which a waiting alert no longer holds.
ALERT_CLEARED_BY = frozenset({"PARKING_OBSERVED", "PINKY_STATUS", "ORDER_STEP_COMPLETED"})
ERROR_TITLES = {
    "SHELF_STOCK_SHORT": "선반 재고 부족", "ITEM_LOST": "물건 분실", "PICK_FAILED_STILL_ON_SHELF": "집기 실패",
    "TRANSFER_NOT_VERIFIED": "옮김 확인 불가", "LOAD_INCOMPLETE": "적재 부족", "LOAD_EXTRA_ITEM": "주문 외 물건",
    "LOAD_MISMATCH": "적재 불일치", "PINKY_ARRIVAL_TIMEOUT": "핑키 도착 시간 초과",
    "PINKY_SIGNAL_REJECTED": "핑키 신호 거부", "PINKY_FLEET_UNREACHABLE": "핑키 관제 연결 오류",
    "PINKY_SIGNAL_NOT_APPLIED": "핑키 신호 미적용", "POLICY_NOT_FOR_ARM": "팔에 정책 없음",
    "ORDER_POLICY_NOT_FOUND": "정책 없는 물체", "ARMS_NOT_CONFIGURED": "팔 설정 없음",
    "ORDER_FLOW_NOT_CONFIGURED": "주문 흐름 미설정",
}


def _unit_name(detail: dict[str, Any] | None) -> str:
    detail = detail or {}
    return f"{detail.get('label') or detail.get('object_class', '?')} #{detail.get('unit', '?')}"


def _stage_state(steps: list[dict[str, Any]]) -> str:
    if not steps:
        return "todo"
    statuses = [step["status"] for step in steps]
    if any(status == "FAILED" for status in statuses):
        return "failed"
    if all(status == "SUCCEEDED" for status in statuses):
        return "done"
    if any(status in ACTIVE_STEP for status in statuses) or any(status == "SUCCEEDED" for status in statuses):
        return "active"
    if all(status == "CANCELED" for status in statuses):
        return "skipped"
    return "todo"


def _current_text(order: dict[str, Any], step: dict[str, Any] | None, events: list[dict[str, Any]]) -> str:
    if order["status"] == "PENDING":
        return "시작 대기"
    if order["status"] == "SUCCEEDED":
        return "완료 · 핑키에 수령 완료 신호 전달"
    if order["status"] in {"FAILED", "CANCELED"}:
        return (order.get("error") or {}).get("message") or ("취소됨" if order["status"] == "CANCELED" else "멈춤")
    if step is None:
        return "진행 중"
    step_events = [event for event in events if event.get("step_id") == step["id"]]
    latest_message = next((event["payload"].get("message") for event in reversed(step_events)
                           if isinstance(event.get("payload"), dict) and event["payload"].get("message")), None)
    detail = step.get("detail") or {}
    kind = step["step_type"]
    if kind == "PICK_ITEM":
        return f"{detail.get('arm_label', '팔')}: {_unit_name(detail)} 집는 중" + (f" · {latest_message}" if latest_message else "")
    if kind == "VERIFY_TRANSFER":
        return f"{_unit_name(detail)} 옮김 확인 중 (선반 −1, 핑키 +1)"
    defaults = {
        "VALIDATE_ORDER": "주문 확인 중", "CHECK_SHELF_STOCK": "선반 카메라로 재고 확인 중",
        "PLAN_ARM_SEQUENCE": "물체별 팔 배정 중", "WAIT_PINKY_ARRIVAL": "핑키 도착 대기",
        "CONFIRM_DOCKING": "핑키에 도킹완료 전달 중", "VERIFY_LOAD_TOTAL": "핑키 위 총량 확인 중",
        "NOTIFY_PINKY_RECEIVED": "핑키에 수령 완료 전달 중", "FINALIZE_TASK": "마무리 중",
        "WAIT_AT_RECEIVING_ZONE": "핑키가 수령 구역으로 가는 중", "CONFIRM_PICKUP": "수령 구역 도착 · 손님 수령 확인 대기",
        "WAIT_PINKY_PARKED": "핑키 복귀 중", "START_SHIPMENT": "다음 운송 핑키 고르는 중",
    }
    return latest_message or defaults.get(kind, kind)


def _shipments(order: dict[str, Any], active: dict[str, Any] | None) -> tuple[int, int]:
    """(current shipment, shipments) of an order; (1, 1) when it is not split."""
    steps = order.get("steps", [])
    count = max((int((step.get("detail") or {}).get("shipments") or 1) for step in steps), default=1)
    if count == 1:
        return 1, 1
    if active is not None:
        return shipment_of(active), count
    if order["status"] == "SUCCEEDED":
        return count, count
    stopped = next((step for step in steps if step["status"] == "FAILED"), None)
    return (shipment_of(stopped) if stopped else 1), count


def summarize_order(order: dict[str, Any]) -> dict[str, Any]:
    steps, events = order.get("steps", []), order.get("events", [])
    active = next((step for step in steps if step["id"] == order.get("active_step_id") and step["status"] in ACTIVE_STEP), None)
    current, count = _shipments(order, active)
    # A split order shows the stages of the shipment under way; shared steps (stock, plan, finish) count for each.
    steps_now = [step for step in steps if (step.get("detail") or {}).get("shipment") in (None, current)]
    by_type: dict[str, list[dict[str, Any]]] = {}
    for step in steps_now:
        by_type.setdefault(step["step_type"], []).append(step)
    stages = []
    for key, label, types in STAGES:
        stage_steps = [step for kind in types for step in by_type.get(kind, [])]
        if key == "DELIVER" and not stage_steps:
            continue
        stages.append({"key": key, "label": label, "state": _stage_state(stage_steps)})
    verified = [step for step in by_type.get("VERIFY_TRANSFER", []) if step["status"] == "SUCCEEDED"]
    total = len(by_type.get("VERIFY_TRANSFER", []))
    load_stage = next(stage for stage in stages if stage["key"] == "LOAD")
    load_stage["label"] = f"적재 {len(verified)}/{total}"
    plan = next((step.get("result", {}).get("sequence") for step in by_type.get("PLAN_ARM_SEQUENCE", [])
                 if step.get("result")), None)
    alert = None
    if order["status"] in {"RUNNING", "VALIDATING"} and active is not None:
        for event in reversed([event for event in events if event.get("step_id") == active["id"]]):
            if event["event_type"] in ALERT_CLEARED_BY:
                break
            blocked = event["event_type"] == "STATION_BUSY" and event["payload"].get("alert")
            if event["event_type"] in ALERT_EVENTS or blocked:
                title = {"PINKY_POSITION_ABNORMAL": "핑키 위치 이상", "PINKY_FLEET_UNREACHABLE": "핑키 관제 연결 오류",
                         "STATION_BUSY": "적재 자리 막힘"}[event["event_type"]]
                alert = {"level": "warning", "code": event["event_type"], "message": event["payload"].get("message"),
                         "at": event["received_at"], "task_id": order["id"], "title": title}
                break
    if alert is None and active is not None and active["step_type"] == "CONFIRM_PICKUP":
        # Not a fault: a person has to confirm the pickup (the customer's app, or the operator for them).
        waiting = next((event for event in reversed(events) if event["event_type"] == "PICKUP_WAITING"), None)
        alert = {"level": "action", "code": "PICKUP_WAITING", "title": "수령 확인 대기",
                 "message": (waiting or {}).get("payload", {}).get("message") or "손님 수령 확인을 기다립니다",
                 "at": (waiting or {}).get("received_at"), "task_id": order["id"], "pickup": True}
    acknowledged = any(event["event_type"] == "ALERT_ACKNOWLEDGED" for event in events)
    if order["status"] == "FAILED" and not acknowledged:
        error = order.get("error") or {}
        alert = {"level": "error", "code": error.get("code"), "title": ERROR_TITLES.get(error.get("code"), "주문 멈춤"),
                 "message": error.get("message"), "at": order.get("finished_at"), "task_id": order["id"],
                 "acknowledgeable": True}
    if alert is not None:
        alert["pinky_name"] = PINKY_NAMES.get(order.get("pinky_id") or "", order.get("pinky_id"))
    simulated = any(isinstance(event.get("payload"), dict) and event["payload"].get("simulated") for event in events)
    return {
        "id": order["id"], "status": order["status"], "pinky_id": order.get("pinky_id"),
        "pinky_name": PINKY_NAMES.get(order.get("pinky_id") or "", order.get("pinky_id")),
        "order_items": order.get("order_items"), "created_at": order.get("created_at"),
        "started_at": order.get("started_at"), "finished_at": order.get("finished_at"),
        "stages": stages, "loaded": len(verified), "total": total,
        "shipment": {"index": current, "count": count} if count > 1 else None,
        "current": (f"운송 {current}/{count} · " if count > 1 and order["status"] in {"RUNNING", "VALIDATING"} else "")
                   + _current_text(order, active, events),
        "active_step": active["step_type"] if active else None,
        "plan": plan, "alert": alert, "error": order.get("error"), "simulated": simulated,
    }


def build_flow_view(
    orders: list[dict[str, Any]],
    pinky_statuses: dict[str, dict[str, Any]],
    arms: Any,
    station_owner: str | None,
    arm_workers: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    summaries = [summarize_order(order) for order in orders]
    running = [order for order in orders if order["status"] in {"VALIDATING", "RUNNING", "CANCELING"}]
    pinkies = []
    for pinky_id, name in PINKY_NAMES.items():
        status = pinky_statuses.get(pinky_id) or {}
        holder = next((order["id"] for order in running if order.get("pinky_id") == pinky_id), None)
        # A Pinky still standing at the station after its order stopped needs a person, not a new order.
        latest = next((order for order in orders if order.get("pinky_id") == pinky_id), None)
        stopped = (
            latest["id"] if holder is None and latest is not None and latest["status"] == "FAILED"
            and status.get("phase") in STATION_PHASES else None
        )
        pinkies.append({
            "pinky_id": pinky_id, "name": name, "phase": status.get("phase"),
            # Without a connection the fleet has no route for it, which would otherwise read as "parked".
            "phase_label": "연결 끊김" if status.get("connected") is False
            else PHASE_LABELS.get(status.get("phase"), status.get("phase") or "상태 모름"),
            "message": status.get("message") or status.get("error"), "location": status.get("location"),
            "order_id": holder, "stopped_order_id": stopped, "simulated": status.get("simulated", False),
            "reachable": bool(status) and "error" not in status,
        })
    arm_rows = []
    workers = {worker["robot_id"]: worker for worker in (arm_workers or [])}
    for arm in (arms.arms if arms is not None else ()):
        doing = None
        for order in running:
            step = next((item for item in order["steps"] if item["id"] == order.get("active_step_id")), None)
            if step and step["step_type"] == "PICK_ITEM" and step.get("robot_id") == arm["arm_id"]:
                doing = {"order_id": order["id"], "text": f"{_unit_name(step.get('detail'))} 집는 중"}
        arm_rows.append({**arm, "state": "busy" if doing else "idle", "doing": doing,
                         "worker": workers.get(arm["arm_id"])})
    alerts = [summary["alert"] for summary in summaries if summary["alert"]]
    return {
        "orders": summaries, "pinkies": pinkies, "arms": arm_rows, "station_owner": station_owner,
        "alerts": sorted(alerts, key=lambda alert: (alert["level"] != "error", alert.get("at") or "")),
    }
