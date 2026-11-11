"""작업 목록·생성·시작·취소, 명령 해석"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from control_station.api.common import validate_pick_request
from control_station.command_parser import parse_pick_command
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


def post_task(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """작업 생성 (선택: 집기 조건)"""
    unsupported_fields = set(body) - {"destination", "arm_job", "pick_request"}
    if unsupported_fields:
        h._json(400, {"error": "Unsupported task fields: " + ", ".join(sorted(unsupported_fields))})
        return
    destination = body.get("destination")
    arm_job = body.get("arm_job")
    if not isinstance(destination, str) or not isinstance(arm_job, str):
        h._json(400, {"error": "destination and arm_job must be strings"})
        return
    destination, arm_job = destination.strip(), arm_job.strip()
    if not destination or len(destination) > 40:
        h._json(400, {"error": "destination must contain 1 to 40 characters"})
        return
    if not arm_job or len(arm_job) > 120:
        h._json(400, {"error": "arm_job must contain 1 to 120 characters"})
        return
    try:
        pick_request = validate_pick_request(body.get("pick_request"))
    except ValueError as exc:
        h._json(400, {"error": str(exc)})
        return
    task = h.app.task_store.create_task(destination, arm_job, pick_request)
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
        {"draft": parse_pick_command(text, known_classes or None, allow_open_vocabulary=allow_open_vocabulary)},
    )
