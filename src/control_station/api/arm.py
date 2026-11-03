"""Arm teleoperation process control, the OMX command queue and coordinate target drafts."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlsplit

from control_station.arm_commands import ArmCommandConflict, ArmCommandError, ArmCommandNotFound
from control_station.coordinate_target_drafts import CoordinateDraftError

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_status(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    h._json(200, {"arm": h.app.arm_supervisor.snapshot()})


def post_teleop(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """POST /api/arm/start|stop: the configured teleoperation process (no body read)."""
    try:
        state = h.app.arm_supervisor.start() if route == "/api/arm/start" else h.app.arm_supervisor.stop()
    except RuntimeError as exc:
        h._json(409, {"error": str(exc), "arm": h.app.arm_supervisor.snapshot()})
        return
    h._json(202, {"arm": state})


def get_commands(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    h._json(200, h.app.arm_commands.list_recent())


def get_command(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    command_id = route.removeprefix("/api/arm/commands/")
    try:
        h._json(200, {"command": h.app.arm_commands.get(command_id)})
    except ArmCommandNotFound:
        h._json_error(404, "ARM_COMMAND_NOT_FOUND", "Arm command not found")


def get_shelf_slots(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
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
    """Worker long-poll: 200 with one frozen command, or 204 when none arrived in time."""
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
    """POST /api/arm/commands and /api/arm/commands/{id}/events|cancel|clear-unknown."""
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


def post_coordinate_draft(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    try:
        draft = h.app.coordinate_target_drafts.create(body)
    except CoordinateDraftError as exc:
        h._json(400, {"error": str(exc)})
        return
    h._json(201, {"draft": draft})
