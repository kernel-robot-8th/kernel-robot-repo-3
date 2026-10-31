#!/usr/bin/env python3
"""Create a review-only shelf image-region draft from a saved task observation."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from control_station.command_parser import MAX_SHELF_LEVEL  # noqa: E402
from calibration_artifacts import (  # noqa: E402
    ensure_distinct_output_path,
    event_step_link_matches,
    install_file_exclusive,
    read_regular_file_bounded,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
_TASK_ID = re.compile(r"^TASK-[A-F0-9]{8}$")
_EVENT_ID = re.compile(r"^EVT-[A-F0-9]{12}$")
_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_MAX_ARTIFACT_BYTES = 1_000_000
_MAX_TASK_JSON_BYTES = 64 * 1024 * 1024
_MAX_SOURCE_IMAGE_BYTES = 64 * 1024 * 1024
_MAX_REGIONS = 64
_MAX_POLYGON_VERTICES = 512


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"JSON object에 중복 키가 있습니다: {key}")
        value[key] = item
    return value


def _load_object(path: Path, label: str) -> dict[str, Any]:
    raw = read_regular_file_bounded(path, _MAX_TASK_JSON_BYTES, label)
    value = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_json_object)
    if not isinstance(value, dict):
        raise ValueError(f"{label} 최상위 값은 객체여야 합니다.")
    return value


def _point(value: object, field: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field}는 [x, y] 숫자 배열이어야 합니다.")
    if any(isinstance(part, bool) or not isinstance(part, (int, float)) for part in value):
        raise ValueError(f"{field} 좌표는 숫자여야 합니다.")
    try:
        point = float(value[0]), float(value[1])
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{field} 좌표는 유한해야 합니다.") from exc
    if not all(math.isfinite(part) for part in point):
        raise ValueError(f"{field} 좌표는 유한해야 합니다.")
    return point


def _polygon_area(points: list[tuple[float, float]]) -> float:
    return 0.5 * abs(
        sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
        )
    )


def _orientation(
    first: tuple[float, float], second: tuple[float, float], third: tuple[float, float]
) -> float:
    return (second[0] - first[0]) * (third[1] - first[1]) - (
        second[1] - first[1]
    ) * (third[0] - first[0])


def _on_segment(
    first: tuple[float, float], second: tuple[float, float], point: tuple[float, float]
) -> bool:
    return (
        min(first[0], second[0]) <= point[0] <= max(first[0], second[0])
        and min(first[1], second[1]) <= point[1] <= max(first[1], second[1])
    )


def _segments_intersect(
    first_start: tuple[float, float], first_end: tuple[float, float],
    second_start: tuple[float, float], second_end: tuple[float, float],
) -> bool:
    orientations = (
        _orientation(first_start, first_end, second_start),
        _orientation(first_start, first_end, second_end),
        _orientation(second_start, second_end, first_start),
        _orientation(second_start, second_end, first_end),
    )
    if (
        (orientations[0] > 0 > orientations[1] or orientations[1] > 0 > orientations[0])
        and (orientations[2] > 0 > orientations[3] or orientations[3] > 0 > orientations[2])
    ):
        return True
    return any(
        orientation == 0 and _on_segment(start, end, point)
        for orientation, start, end, point in (
            (orientations[0], first_start, first_end, second_start),
            (orientations[1], first_start, first_end, second_end),
            (orientations[2], second_start, second_end, first_start),
            (orientations[3], second_start, second_end, first_end),
        )
    )


def _simple_polygon(points: list[tuple[float, float]]) -> bool:
    count = len(points)
    for index in range(count):
        previous = points[index - 1]
        current = points[index]
        following = points[(index + 1) % count]
        if _orientation(previous, current, following) == 0:
            incoming = current[0] - previous[0], current[1] - previous[1]
            outgoing = following[0] - current[0], following[1] - current[1]
            if incoming[0] * outgoing[0] + incoming[1] * outgoing[1] < 0:
                return False
    for first_index in range(count):
        first_next = (first_index + 1) % count
        for second_index in range(first_index + 1, count):
            second_next = (second_index + 1) % count
            if first_index == second_index or first_next == second_index or second_next == first_index:
                continue
            if _segments_intersect(
                points[first_index], points[first_next], points[second_index], points[second_next]
            ):
                return False
    return True


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    serialized = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if len(serialized.encode("utf-8")) > _MAX_ARTIFACT_BYTES:
        raise ValueError("선반 영역 초안이 1 MB 제한을 초과합니다.")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=f".{path.name}.", suffix=".tmp", delete=False,
        ) as output_file:
            temporary_path = Path(output_file.name)
            output_file.write(serialized)
        install_file_exclusive(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def _load_observation(
    task_json: Path, event_id: str
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any], Path]:
    response = _load_object(task_json, "task response")
    task = response.get("task", response)
    if not isinstance(task, dict) or not isinstance(task.get("events"), list):
        raise ValueError("task JSON에는 events 배열이 있는 task 객체가 필요합니다.")
    task_id = task.get("id")
    if not isinstance(task_id, str) or not _TASK_ID.fullmatch(task_id):
        raise ValueError("task ID 형식이 올바르지 않습니다.")
    if task.get("status") != "PENDING":
        raise ValueError("source observation의 task는 PENDING 상태여야 합니다.")
    matches = [event for event in task["events"] if isinstance(event, dict) and event.get("id") == event_id]
    if len(matches) != 1:
        raise ValueError("event ID가 task 이력에 정확히 한 번 존재해야 합니다.")
    event = matches[0]
    if (
        event.get("event_type") != "TARGET_OBSERVED"
        or event.get("task_id") != task_id
        or not event_step_link_matches(task, event, "DETECT_TARGETS")
        or event.get("previous_status") != "PENDING"
        or event.get("new_status") != "PENDING"
    ):
        raise ValueError("source event는 task 상태를 바꾸지 않은 TARGET_OBSERVED여야 합니다.")
    payload = event.get("payload")
    if (
        not isinstance(payload, dict)
        or payload.get("event") != "TARGET_OBSERVED"
        or payload.get("source") != "camera_observation"
        or payload.get("arm_command_sent") is not False
        or payload.get("task_state_changed") is not False
    ):
        raise ValueError("source event가 관측 전용 provenance를 만족하지 않습니다.")
    pick_request = payload.get("pick_request")
    task_pick_request = task.get("pick_request")
    observation = payload.get("observation")
    if (
        not isinstance(pick_request, dict)
        or pick_request != task_pick_request
        or not isinstance(observation, dict)
    ):
        raise ValueError("source event의 pick_request/observation이 task와 맞지 않습니다.")
    if (
        isinstance(observation.get("observation_schema_version"), bool)
        or not isinstance(observation.get("observation_schema_version"), int)
        or observation.get("observation_schema_version") not in {2, 3}
        or observation.get("decision_scope") != "OBSERVATION_ONLY"
        or observation.get("arm_command_sent") is not False
        or observation.get("frame_freshness") != "FRESH"
    ):
        raise ValueError("source observation은 fresh한 schema v2/v3 관측 전용이어야 합니다.")

    camera_id = observation.get("camera_id")
    transform_id = observation.get("pixel_transform_id")
    frame_sha256 = observation.get("frame_sha256")
    width, height = observation.get("frame_width"), observation.get("frame_height")
    frame_sequence = observation.get("frame_sequence")
    captured_at = observation.get("captured_at")
    if not isinstance(camera_id, str) or not _IDENTIFIER.fullmatch(camera_id):
        raise ValueError("observation camera_id 형식이 올바르지 않습니다.")
    if not isinstance(transform_id, str) or not _IDENTIFIER.fullmatch(transform_id):
        raise ValueError("observation pixel_transform_id 형식이 올바르지 않습니다.")
    if not isinstance(frame_sha256, str) or not _SHA256.fullmatch(frame_sha256):
        raise ValueError("observation frame_sha256가 올바르지 않습니다.")
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in (width, height)):
        raise ValueError("observation frame dimensions must be positive integers")
    if isinstance(frame_sequence, bool) or not isinstance(frame_sequence, int) or frame_sequence <= 0:
        raise ValueError("observation frame_sequence must be a positive integer")
    if not isinstance(captured_at, str) or not captured_at.strip():
        raise ValueError("observation captured_at is missing")

    artifact = payload.get("frame_artifact")
    if (
        payload.get("frame_saved") is not True
        or not isinstance(artifact, dict)
        or artifact.get("media_type") != "image/png"
        or artifact.get("pixel_format") != "BGR8"
        or artifact.get("source_frame_sha256") != frame_sha256
        or isinstance(artifact.get("width"), bool)
        or not isinstance(artifact.get("width"), int)
        or artifact.get("width") != width
        or isinstance(artifact.get("height"), bool)
        or not isinstance(artifact.get("height"), int)
        or artifact.get("height") != height
    ):
        raise ValueError("source event에 frame과 일치하는 저장 PNG artifact가 없습니다.")
    relative_path = artifact.get("path")
    file_sha256 = artifact.get("file_sha256")
    if (
        not isinstance(relative_path, str)
        or not isinstance(file_sha256, str)
        or not _SHA256.fullmatch(file_sha256)
        or relative_path.startswith("/")
        or "\\" in relative_path
        or ".." in relative_path.split("/")
    ):
        raise ValueError("frame artifact path/hash가 유효하지 않습니다.")
    try:
        control_directory = REPOSITORY_ROOT / ".control-station"
        artifact_root = control_directory / "observations"
        image_root = artifact_root / task_id
        image_path = image_root / relative_path.split("/")[-1]
        if any(
            path.is_symlink()
            for path in (control_directory, artifact_root, image_root, image_path)
        ):
            raise ValueError("source PNG path cannot use symbolic links")
        resolved_artifact_root = artifact_root.resolve(strict=True)
        resolved_image_root = image_root.resolve(strict=True)
        resolved_image_path = image_path.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise ValueError("source PNG path cannot be resolved safely") from exc
    if (
        resolved_artifact_root != artifact_root
        or resolved_image_root != image_root
        or resolved_image_root.parent != resolved_artifact_root
        or resolved_image_path != image_path
        or image_path.suffix != ".png"
        or not image_path.is_file()
        or image_path.stat().st_size > _MAX_SOURCE_IMAGE_BYTES
    ):
        raise ValueError("source PNG는 task의 관리 observation 디렉터리 바로 아래에 있어야 합니다.")
    try:
        with image_path.open("rb") as image_file:
            image_bytes = image_file.read(_MAX_SOURCE_IMAGE_BYTES + 1)
    except (OSError, RuntimeError) as exc:
        raise ValueError("source observation PNG를 읽을 수 없습니다.") from exc
    if len(image_bytes) > _MAX_SOURCE_IMAGE_BYTES:
        raise ValueError("source observation PNG exceeds the 64 MiB limit")
    if hashlib.sha256(image_bytes).hexdigest() != file_sha256:
        raise ValueError("source PNG SHA-256이 event artifact와 다릅니다.")
    image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (height, width):
        raise ValueError("source PNG를 해독할 수 없거나 해상도가 event와 다릅니다.")
    decoded_frame_hash = hashlib.sha256(image.tobytes(order="C")).hexdigest()
    if decoded_frame_hash != frame_sha256:
        raise ValueError("source PNG의 복원 픽셀 hash가 observation frame hash와 다릅니다.")

    source = {
        "task_id": task_id,
        "event_id": event_id,
        "step_id": event.get("step_id"),
        "camera_id": camera_id,
        "pixel_transform_id": transform_id,
        "frame_sequence": frame_sequence,
        "captured_at": captured_at,
        "frame_sha256": frame_sha256,
        "image_path": relative_path,
        "image_sha256": file_sha256,
        "image_size_px": {"width": width, "height": height},
    }
    return task, event, observation, source, image_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task_json", type=Path, help="GET /api/tasks/{task_id} 응답 JSON")
    parser.add_argument("regions_json", type=Path, help="측정한 polygon을 담은 입력 JSON")
    parser.add_argument("--event-id", required=True, help="저장된 원본 PNG가 연결된 TARGET_OBSERVED ID")
    parser.add_argument("--output", type=Path, help="결과 JSON 경로")
    args = parser.parse_args()
    if not _EVENT_ID.fullmatch(args.event_id):
        parser.error("--event-id 형식이 올바르지 않습니다.")

    try:
        task, event, observation, source, source_image_path = _load_observation(
            args.task_json, args.event_id
        )
        region_bytes = read_regular_file_bounded(
            args.regions_json, _MAX_ARTIFACT_BYTES, "regions 입력"
        )
        region_input = json.loads(
            region_bytes.decode("utf-8"), object_pairs_hook=_unique_json_object
        )
        if not isinstance(region_input, dict) or set(region_input) != {
            "schema_version", "source_image_sha256", "source_image_size_px", "regions"
        }:
            raise ValueError(
                "regions JSON에는 schema_version, source image provenance, regions만 포함해야 합니다."
            )
        schema_version = region_input.get("schema_version")
        if (
            isinstance(schema_version, bool)
            or not isinstance(schema_version, int)
            or schema_version != 2
        ):
            raise ValueError("지원하지 않는 regions schema_version입니다.")
        source_image_size = region_input.get("source_image_size_px")
        if (
            region_input.get("source_image_sha256") != source["image_sha256"]
            or not isinstance(source_image_size, dict)
            or set(source_image_size) != {"width", "height"}
            or any(
                isinstance(source_image_size.get(axis), bool)
                or not isinstance(source_image_size.get(axis), int)
                or source_image_size.get(axis) <= 0
                for axis in ("width", "height")
            )
            or source_image_size != source["image_size_px"]
        ):
            raise ValueError("regions polygon의 원본 PNG hash/해상도가 task 관측 프레임과 다릅니다.")
        raw_regions = region_input.get("regions")
        if not isinstance(raw_regions, list) or not raw_regions or len(raw_regions) > _MAX_REGIONS:
            raise ValueError(f"측정 regions는 1~{_MAX_REGIONS}개여야 합니다.")

        width = source["image_size_px"]["width"]
        height = source["image_size_px"]["height"]
        regions: list[dict[str, Any]] = []
        seen_levels: set[int] = set()
        for index, region in enumerate(raw_regions):
            if not isinstance(region, dict) or set(region) != {"shelf_level", "polygon_px"}:
                raise ValueError(f"regions[{index}]에는 shelf_level과 polygon_px만 허용됩니다.")
            level = region.get("shelf_level")
            if (
                isinstance(level, bool)
                or not isinstance(level, int)
                or not 1 <= level <= MAX_SHELF_LEVEL
            ):
                raise ValueError(
                    f"regions[{index}].shelf_level은 1부터 {MAX_SHELF_LEVEL}까지의 정수여야 합니다."
                )
            if level in seen_levels:
                raise ValueError(f"shelf_level {level}이 중복되었습니다.")
            seen_levels.add(level)
            raw_polygon = region.get("polygon_px")
            if (
                not isinstance(raw_polygon, list)
                or not 3 <= len(raw_polygon) <= _MAX_POLYGON_VERTICES
            ):
                raise ValueError(
                    f"regions[{index}].polygon_px는 3~{_MAX_POLYGON_VERTICES}개 정점이어야 합니다."
                )
            points = [
                _point(value, f"regions[{index}].polygon_px[{point_index}]")
                for point_index, value in enumerate(raw_polygon)
            ]
            if len(set(points)) != len(points):
                raise ValueError(f"regions[{index}].polygon_px에 중복 정점이 있습니다.")
            if not _simple_polygon(points):
                raise ValueError(f"regions[{index}].polygon_px의 변이 서로 교차합니다.")
            if any(not (0 <= x < width and 0 <= y < height) for x, y in points):
                raise ValueError(f"regions[{index}].polygon_px가 영상 범위를 벗어났습니다.")
            area = _polygon_area(points)
            if area <= 0:
                raise ValueError(f"regions[{index}].polygon_px가 퇴화했습니다.")
            regions.append({"shelf_level": level, "polygon_px": [list(point) for point in points]})

        calibration_id = str(uuid.uuid4())
        payload = {
            "schema_version": 1,
            "calibration_id": calibration_id,
            "created_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "status": "DRAFT_ONLY",
            "calibration_scope": "CAMERA_IMAGE_SHELF_REGIONS_2D",
            "camera_id": source["camera_id"],
            "pixel_transform_id": source["pixel_transform_id"],
            "image_size_px": source["image_size_px"],
            "source": source,
            "source_task_pick_request": task.get("pick_request"),
            "source_observation_status": observation.get("status"),
            "source_event_type": event.get("event_type"),
            "regions_input_file": args.regions_json.name,
            "regions_input_sha256": hashlib.sha256(region_bytes).hexdigest(),
            "regions": regions,
            "review_required": True,
            "selection_enabled": False,
            "usable_for_robot_control": False,
            "arm_command_sent": False,
        }
        output_path = args.output or (
            Path(".control-station/camera-calibrations")
            / f"{source['camera_id']}-shelf-regions-{calibration_id}.json"
        )
        ensure_distinct_output_path(
            output_path, [args.task_json, args.regions_json, source_image_path]
        )
        _write_json(output_path, payload)
    except (
        OSError, RuntimeError, UnicodeError, TypeError, ValueError, cv2.error
    ) as exc:
        print(f"선반 영상 영역 초안을 만들 수 없습니다: {exc}", file=sys.stderr)
        return 2

    print(f"Shelf image-region draft: {output_path}")
    print(
        f"calibration_id={calibration_id} · camera_id={source['camera_id']} · "
        f"regions={len(regions)} · source_event={args.event_id}"
    )
    print("DRAFT_ONLY · review required · selection disabled · no robot command sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
