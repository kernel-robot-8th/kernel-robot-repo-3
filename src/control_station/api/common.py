"""경로 처리 공통: 다른 출처 확인, JSON 본문, 집기 조건·관측 프레임 검증"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import urlsplit

from control_station.camera_adapters import MAX_OBSERVATION_IMAGE_BYTES
from control_station.command_parser import MAX_SHELF_LEVEL

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
TASK_ID = re.compile(r"TASK-[A-F0-9]{8}")
EVENT_ID = re.compile(r"EVT-[A-F0-9]{12}")
SHA256 = re.compile(r"[0-9a-f]{64}")


def is_cross_origin(handler: ControlStationHandler) -> bool:
    """다른 출처(브라우저) 요청인지"""
    origin = handler.headers.get("Origin")
    return bool(origin) and urlsplit(origin).netloc != handler.headers.get("Host")


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """중복 키를 거부하는 JSON 객체 읽기"""
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON object key: {key}")
        value[key] = item
    return value


def read_json_object(handler: ControlStationHandler) -> dict[str, Any] | None:
    """요청 본문을 JSON 객체로 읽기, 실패하면 오류 응답 후 None"""
    if handler.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
        handler._json(415, {"error": "Content-Type must be application/json"})
        return None
    try:
        length = int(handler.headers.get("Content-Length", "0"))
    except ValueError:
        handler._json(400, {"error": "Invalid Content-Length"})
        return None
    if not 0 < length <= 4096:
        handler._json(400, {"error": "Request body must be between 1 and 4096 bytes"})
        return None
    try:
        body = json.loads(handler.rfile.read(length), object_pairs_hook=_unique_json_object)
    except (ValueError, UnicodeDecodeError, RecursionError):
        handler._json(400, {"error": "Request body must be valid JSON with unique object keys"})
        return None
    if not isinstance(body, dict):
        handler._json(400, {"error": "Request body must be a JSON object"})
        return None
    return body


def validate_pick_request(value: object) -> dict[str, object] | None:
    """집기 조건(종류, 색, 선반 층) 검증"""
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError("pick_request must be an object")
    allowed = {"category", "color", "shelf_level"}
    if set(value) - allowed:
        raise ValueError("pick_request supports only category, color, and shelf_level")
    category = value.get("category")
    if (
        not isinstance(category, str)
        or not category.strip()
        or len(category.strip()) > 80
        or any(ord(character) < 32 or ord(character) == 127 for character in category.strip())
    ):
        raise ValueError(
            "pick_request.category must contain 1 to 80 characters without control characters"
        )
    result: dict[str, object] = {"category": category.strip()}
    if "color" in value and value["color"] is not None:
        color = value["color"]
        if (
            not isinstance(color, str)
            or not color.strip()
            or len(color.strip()) > 40
            or any(ord(character) < 32 or ord(character) == 127 for character in color.strip())
        ):
            raise ValueError(
                "pick_request.color must contain 1 to 40 characters without control characters"
            )
        result["color"] = color.strip()
    if "shelf_level" in value and value["shelf_level"] is not None:
        shelf_level = value["shelf_level"]
        if (
            isinstance(shelf_level, bool)
            or not isinstance(shelf_level, int)
            or not 1 <= shelf_level <= MAX_SHELF_LEVEL
        ):
            raise ValueError(
                f"pick_request.shelf_level must be an integer from 1 to {MAX_SHELF_LEVEL}"
            )
        result["shelf_level"] = shelf_level
    return result


def validate_stored_pick_request(value: object) -> dict[str, object]:
    """저장된 집기 조건 검증"""
    try:
        validated = validate_pick_request(value)
    except ValueError as exc:
        raise ValueError("Stored pick_request does not match the supported contract") from exc
    if validated is None or validated != value:
        raise ValueError("Stored pick_request is missing or is not in canonical form")
    return validated


def pending_pick_step_id(task: dict[str, Any], step_type: str) -> str | None:
    """대기 중인 작업 단계 ID"""
    steps = task.get("steps")
    if not isinstance(steps, list):
        return None
    matches = [
        step
        for step in steps
        if isinstance(step, dict) and step.get("step_type") == step_type
    ]
    if len(matches) != 1 or matches[0].get("status") != "PENDING":
        return None
    step_id = matches[0].get("id")
    return step_id if isinstance(step_id, str) and step_id else None


def read_observation_png(task_id: str, file_name: str) -> bytes | None:
    """저장된 관측 PNG 안전하게 읽기"""
    try:
        control_directory = REPOSITORY_ROOT / ".control-station"
        artifact_root = control_directory / "observations"
        task_directory = artifact_root / task_id
        image_path = task_directory / file_name
        if any(
            path.is_symlink()
            for path in (control_directory, artifact_root, task_directory, image_path)
        ):
            return None
        resolved_artifact_root = artifact_root.resolve(strict=True)
        resolved_task_directory = task_directory.resolve(strict=True)
        resolved_image_path = image_path.resolve(strict=True)
        if (
            resolved_artifact_root != artifact_root
            or resolved_task_directory != task_directory
            or resolved_task_directory.parent != resolved_artifact_root
            or resolved_image_path != image_path
            or not image_path.is_file()
            or image_path.suffix != ".png"
            or image_path.stat().st_size > MAX_OBSERVATION_IMAGE_BYTES
        ):
            return None
        with image_path.open("rb") as image_file:
            image_bytes = image_file.read(MAX_OBSERVATION_IMAGE_BYTES + 1)
    except (OSError, RuntimeError):
        return None
    return image_bytes if len(image_bytes) <= MAX_OBSERVATION_IMAGE_BYTES else None


def verified_observation_frame(
    task_id: str,
    artifact: object,
    frame_sha256: object,
    frame_width: object,
    frame_height: object,
) -> bool:
    """저장된 관측 프레임 확인 (경로, 해시, 크기)"""
    if not TASK_ID.fullmatch(task_id) or not isinstance(artifact, dict):
        return False
    relative_path = artifact.get("path")
    file_sha256 = artifact.get("file_sha256")
    if (
        artifact.get("media_type") != "image/png"
        or artifact.get("pixel_format") != "BGR8"
        or artifact.get("source_frame_sha256") != frame_sha256
        or any(
            isinstance(artifact.get(dimension), bool)
            or not isinstance(artifact.get(dimension), int)
            or artifact.get(dimension) <= 0
            for dimension in ("width", "height")
        )
        or artifact.get("width") != frame_width
        or artifact.get("height") != frame_height
        or not isinstance(frame_sha256, str)
        or not SHA256.fullmatch(frame_sha256)
        or not isinstance(relative_path, str)
        or not isinstance(file_sha256, str)
        or not SHA256.fullmatch(file_sha256)
    ):
        return False
    path_parts = relative_path.split("/")
    if (
        relative_path.startswith("/")
        or "\\" in relative_path
        or ".." in path_parts
        or len(path_parts) != 4
        or path_parts[:3] != [".control-station", "observations", task_id]
        or not path_parts[3].endswith(".png")
        or Path(path_parts[3]).name != path_parts[3]
    ):
        return False
    image_bytes = read_observation_png(task_id, path_parts[3])
    return image_bytes is not None and hashlib.sha256(image_bytes).hexdigest() == file_sha256
