"""팔 텔레오퍼레이션, OMX 명령 큐, 선반 칸, 좌표 초안"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlsplit

from control_station.arm_commands import ArmCommandConflict, ArmCommandError, ArmCommandNotFound
from control_station.coordinate_target_drafts import CoordinateDraftError
from control_station.shelf_slot_zones import SlotSelectionError, select_slot_for_class

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_status(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """팔 텔레오퍼레이션 상태"""
    h._json(200, {"arm": h.app.arm_supervisor.snapshot()})


def post_teleop(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """팔 텔레오퍼레이션 시작·정지"""
    try:
        state = h.app.arm_supervisor.start() if route == "/api/arm/start" else h.app.arm_supervisor.stop()
    except RuntimeError as exc:
        h._json(409, {"error": str(exc), "arm": h.app.arm_supervisor.snapshot()})
        return
    h._json(202, {"arm": state})


def get_commands(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """OMX 명령 목록"""
    h._json(200, h.app.arm_commands.list_recent())


def get_command(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """OMX 명령 하나 조회"""
    command_id = route.removeprefix("/api/arm/commands/")
    try:
        h._json(200, {"command": h.app.arm_commands.get(command_id)})
    except ArmCommandNotFound:
        h._json_error(404, "ARM_COMMAND_NOT_FOUND", "Arm command not found")


def get_shelf_slots(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """선반 칸 정의"""
    slot_map = h.app.arm_commands.slot_map
    if slot_map is None:
        h._json_error(
            404,
            "SHELF_SLOTS_NOT_CONFIGURED",
            h.app.arm_commands.slot_map_error or "칸 정의 파일이 없습니다",
        )
    else:
        h._json(200, {"slot_map_version": slot_map.version, "slot_map": slot_map.document})


def get_next_command(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """워커가 다음 명령 받기 (기다리다 없으면 204)"""
    robot_id = route.split("/")[3]
    query = parse_qs(urlsplit(h.path).query)
    try:
        wait_s = float(query.get("wait_s", ["20"])[0])
        command = h.app.arm_commands.claim_next(robot_id, query.get("worker_id", [None])[0], wait_s)
    except ArmCommandNotFound:
        h._json_error(404, "ARM_NOT_FOUND", "Unknown arm robot_id")
        return
    except (ArmCommandError, ValueError) as exc:
        h._json_error(400, "ARM_COMMAND_INVALID", str(exc))
        return
    if command is None:
        h.send_response(204)
        h.send_header("Cache-Control", "no-store")
        h.end_headers()
        return
    h._json(200, {"command": command})


def post_command(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """OMX 명령 등록·진행 이벤트·취소·불명 해제"""
    store = h.app.arm_commands
    try:
        if len(parts) == 3:
            h._json(201, {"command": store.create(body)})
        elif parts[4] == "events":
            command, applied = store.apply_worker_event(parts[3], body)
            h._json(200, {"command": command, "applied": applied})
        elif parts[4] == "cancel":
            if body:
                raise ArmCommandError("cancel takes an empty JSON object")
            h._json(200, {"command": store.cancel(parts[3])})
        else:
            if set(body) != {"note"}:
                raise ArmCommandError("clear-unknown takes exactly note")
            h._json(200, {"command": store.clear_unknown(parts[3], body["note"])})
    except ArmCommandNotFound:
        h._json_error(404, "ARM_COMMAND_NOT_FOUND", "Arm command not found")
    except ArmCommandError as exc:
        h._json_error(400, "ARM_COMMAND_INVALID", str(exc))
    except ArmCommandConflict as exc:
        h._json_error(409, exc.error_code, str(exc), command=exc.command)


def post_command_from_camera(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """정면 카메라가 물건을 찾은 칸으로 OMX 이동 명령 등록 (한 칸에만 보일 때만)"""
    allowed = {"object_class", "camera_id", "valid_for_s"}
    object_class = body.get("object_class")
    if set(body) - allowed or not isinstance(object_class, str) or not object_class.strip():
        h._json_error(400, "ARM_COMMAND_INVALID", "body needs object_class and may have camera_id, valid_for_s")
        return
    camera_id = body.get("camera_id", "shelf_camera")
    snapshot = next((camera for camera in h.app.camera_manager.snapshots() if camera.id == camera_id), None)
    if snapshot is None:
        h._json_error(404, "CAMERA_NOT_FOUND", f"camera {camera_id!r} is not configured")
        return
    try:
        selection = select_slot_for_class(h.app.shelf_slot_zones.annotate(snapshot.to_dict()), object_class)
        command = h.app.arm_commands.create(
            {
                "robot_id": "arm-01",
                "operation": "MOVE_TO_SLOT",
                "valid_for_s": body.get("valid_for_s", 60),
                "target": {"object_class": object_class.strip(), "pick_slot_id": selection["slot_id"]},
                "place": {"slot_id": "CART_1"},
                "source": {"kind": "OPERATOR_INPUT"},  # replaced by the camera evidence below
            },
            camera_source=selection,
        )
    except SlotSelectionError as exc:
        h._json_error(409, exc.error_code, str(exc), details=exc.details)
        return
    except ArmCommandError as exc:
        h._json_error(400, "ARM_COMMAND_INVALID", str(exc))
        return
    except ArmCommandConflict as exc:
        h._json_error(409, exc.error_code, str(exc), command=exc.command)
        return
    h._json(201, {"command": command, "selection": selection})


def post_coordinate_draft(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """좌표 목표 초안 저장"""
    try:
        draft = h.app.coordinate_target_drafts.create(body)
    except CoordinateDraftError as exc:
        h._json(400, {"error": str(exc)})
        return
    h._json(201, {"draft": draft})
