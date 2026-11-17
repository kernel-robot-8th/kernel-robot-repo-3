"""작업 목록·생성·시작·취소, 명령 해석"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from control_station.api.common import validate_order_items, validate_pick_request
from control_station.command_parser import parse_order_text, parse_pick_command
from control_station.store import PINKY_IDS, PinkyUnavailable
from control_station.task_orchestrator import TaskExecutionConflict

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_tasks(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """작업 목록"""
    h._json(200, {"tasks": h.app.task_store.list_tasks()})


def get_task(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """작업 하나 조회"""
    task_id = route.removeprefix("/api/tasks/")
    if not task_id or "/" in task_id:
        h._json(404, {"error": "Task not found"})
        return
    task = h.app.task_store.get_task(task_id)
    if task is None:
        h._json(404, {"error": "Task not found"})
    else:
        task["local_execution_owner"] = h.app.task_orchestrator.local_worker_state(task_id)
        h._json(200, {"task": task})


_TEXT_LIMITS = (("destination", 40), ("arm_job", 120))


def _task_texts(h: ControlStationHandler, destination: object, arm_job: object) -> tuple[str, str] | None:
    """목적지·팔 작업 문구의 앞뒤 공백을 빼고 길이 확인, 틀리면 400 응답 후 None"""
    texts: list[str] = []
    for (name, maximum), value in zip(_TEXT_LIMITS, (destination, arm_job)):
        text = value.strip() if isinstance(value, str) else ""
        if not text or len(text) > maximum:
            h._json(400, {"error": f"{name} must contain 1 to {maximum} characters"})
            return None
        texts.append(text)
    return texts[0], texts[1]


def post_task(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """작업 생성: 주문(order_items, 선택 pinky_id) 또는 이전 형식(선택 pick_request)"""
    unsupported_fields = set(body) - {"destination", "arm_job", "pick_request", "order_items", "pinky_id"}
    if unsupported_fields:
        h._json(400, {"error": "Unsupported task fields: " + ", ".join(sorted(unsupported_fields))})
        return
    if "order_items" in body:
        _post_order(h, body)
        return
    if "pinky_id" in body:
        h._json(400, {"error": "pinky_id is only accepted with order_items"})
        return
    if not isinstance(body.get("destination"), str) or not isinstance(body.get("arm_job"), str):
        h._json(400, {"error": "destination and arm_job must be strings"})
        return
    texts = _task_texts(h, body["destination"], body["arm_job"])
    if texts is None:
        return
    destination, arm_job = texts
    try:
        pick_request = validate_pick_request(body.get("pick_request"))
    except ValueError as exc:
        h._json(400, {"error": str(exc)})
        return
    task = h.app.task_store.create_task(destination, arm_job, pick_request)
    h._json(201, {"task": task})


def _post_order(h: ControlStationHandler, body: dict[str, Any]) -> None:
    """주문: 물체·수량을 검증하고 비어 있는 핑키(또는 지정한 핑키)를 배정"""
    if body.get("pick_request") is not None:
        h._json(400, {"error": "order_items and pick_request cannot be sent together"})
        return
    policies = h.app.arm_commands.act_policies
    try:
        order_items = validate_order_items(body.get("order_items"), policies)
    except ValueError as exc:
        h._json(400, {"error": str(exc)})
        return
    pinky_id = body.get("pinky_id")
    if pinky_id is not None and pinky_id not in PINKY_IDS:
        h._json(400, {"error": "pinky_id must be one of: " + ", ".join(PINKY_IDS)})
        return
    labels = {policy["object_class"]: policy["label"] for policy in policies.policies.values()}
    summary = " · ".join(f"{labels.get(item['object_class'], item['object_class'])} {item['quantity']}" for item in order_items)
    texts = _task_texts(h, body.get("destination", "핑키 적재 위치"), body.get("arm_job", f"{summary} 적재"))
    if texts is None:
        return
    try:
        task = h.app.task_store.create_task(*texts, order_items=order_items, pinky_id=pinky_id)
    except PinkyUnavailable as exc:
        h._json_error(409, "PINKY_UNAVAILABLE", str(exc))
        return
    h._json(201, {"task": task})


def post_task_control(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """작업 시작·취소 (모의 실행)"""
    task_id = parts[2]
    if not task_id:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    try:
        task, changed = (
            h.app.task_orchestrator.start(task_id)
            if parts[3] == "start"
            else h.app.task_orchestrator.cancel(task_id)
        )
    except TaskExecutionConflict as exc:
        details: dict[str, Any] = {} if exc.task is None else {"task": exc.task}
        h._json_error(409, exc.error_code, str(exc), **details)
        return
    except LookupError:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    except RuntimeError as exc:
        h._json_error(409, "TASK_CONTROL_ERROR", str(exc))
        return
    accepted = changed and task["status"] not in {"SUCCEEDED", "FAILED", "CANCELED"}
    h._json(202 if accepted else 200, {"task": task, "execution_mode": "mock"})


def post_parse_command(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """명령 문장 해석 → 집기 조건 초안"""
    text = body.get("text")
    if set(body) != {"text"} or not isinstance(text, str) or not text.strip() or len(text) > 500:
        h._json(400, {"error": "text must contain 1 to 500 characters"})
        return
    search_camera_detector = next(
        (camera for camera in h.app.camera_manager.snapshots() if camera.position == "HEAD"),
        None,
    )
    detector_ready = search_camera_detector is not None and search_camera_detector.detector_state == "ready"
    known_classes = (
        list(dict.fromkeys(search_camera_detector.detector_classes or []))
        if detector_ready and search_camera_detector is not None
        else []
    )
    allow_open_vocabulary = (
        detector_ready
        and search_camera_detector is not None
        and search_camera_detector.detector_model_type == "YOLO_WORLD_OPEN_VOCAB"
    )
    h._json(
        200,
        {
            "draft": parse_pick_command(text, known_classes or None, allow_open_vocabulary=allow_open_vocabulary),
            # Order items from the trained-policy names (당근 1개랑 사과 2개); the form for a new order uses these.
            "order": parse_order_text(text, h.app.arm_commands.act_policies),
        },
    )
