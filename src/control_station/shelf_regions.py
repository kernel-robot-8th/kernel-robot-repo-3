"""Review-only shelf-level comparison using operator-provided image polygons."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from pathlib import Path
from typing import Any

from control_station.command_parser import MAX_SHELF_LEVEL


_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_TASK_ID = re.compile(r"^TASK-[A-F0-9]{8}$")
_EVENT_ID = re.compile(r"^EVT-[A-F0-9]{12}$")
_MAX_CALIBRATION_BYTES = 1_000_000
_MAX_REGIONS = 64
_MAX_POLYGON_VERTICES = 512


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON object key: {key}")
        value[key] = item
    return value


def _finite_point(value: object) -> tuple[float, float] | None:
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value)
    ):
        return None
    try:
        point = float(value[0]), float(value[1])
    except (OverflowError, ValueError):
        return None
    return point if all(math.isfinite(item) for item in point) else None


def _point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> int:
    """Return 1 inside, 0 on a boundary, and -1 outside."""
    x, y = point
    inside = False
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        x1, y1 = start
        x2, y2 = end
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        scale = max(1.0, abs(x), abs(y), abs(x1), abs(y1), abs(x2), abs(y2))
        if (
            abs(cross) <= 1e-9 * scale
            and min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9
            and min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9
        ):
            return 0
        if (y1 > y) != (y2 > y):
            crossing_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < crossing_x:
                inside = not inside
    return 1 if inside else -1


def _orientation(
    first: tuple[float, float], second: tuple[float, float], third: tuple[float, float]
) -> float:
    return (second[0] - first[0]) * (third[1] - first[1]) - (
        second[1] - first[1]
    ) * (third[0] - first[0])


def _segments_intersect(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, float],
    second_end: tuple[float, float],
) -> bool:
    values = (
        _orientation(first_start, first_end, second_start),
        _orientation(first_start, first_end, second_end),
        _orientation(second_start, second_end, first_start),
        _orientation(second_start, second_end, first_end),
    )
    if values[0] * values[1] < 0 and values[2] * values[3] < 0:
        return True
    for orient, start, end, point in (
        (values[0], first_start, first_end, second_start),
        (values[1], first_start, first_end, second_end),
        (values[2], second_start, second_end, first_start),
        (values[3], second_start, second_end, first_end),
    ):
        if orient == 0 and (
            min(start[0], end[0]) <= point[0] <= max(start[0], end[0])
            and min(start[1], end[1]) <= point[1] <= max(start[1], end[1])
        ):
            return True
    return False


def _simple_polygon(polygon: list[tuple[float, float]]) -> bool:
    count = len(polygon)
    for index in range(count):
        previous, current, following = polygon[index - 1], polygon[index], polygon[(index + 1) % count]
        if _orientation(previous, current, following) == 0:
            incoming = current[0] - previous[0], current[1] - previous[1]
            outgoing = following[0] - current[0], following[1] - current[1]
            if incoming[0] * outgoing[0] + incoming[1] * outgoing[1] < 0:
                return False
    for first_index in range(count):
        first_next = (first_index + 1) % count
        for second_index in range(first_index + 1, count):
            second_next = (second_index + 1) % count
            if first_next == second_index or second_next == first_index:
                continue
            if _segments_intersect(
                polygon[first_index], polygon[first_next], polygon[second_index], polygon[second_next]
            ):
                return False
    return True


def _result(
    requested_level: int,
    status: str,
    *,
    calibration_id: str | None = None,
    calibration_artifact_sha256: str | None = None,
    candidate_indices: list[int] | None = None,
    assignments: list[dict[str, Any]] | None = None,
    detail: str | None = None,
) -> dict[str, Any]:
    evaluation: dict[str, Any] = {
        "requested_shelf_level": requested_level,
        "status": status,
        "calibration_id": calibration_id,
        "candidate_indices": candidate_indices or [],
        "candidate_assignments": assignments or [],
        "calibration_status": "DRAFT_ONLY" if calibration_id else None,
        "review_required": True,
        "used_for_selection": False,
    }
    if calibration_artifact_sha256 is not None:
        evaluation["calibration_artifact_sha256"] = calibration_artifact_sha256
    if detail:
        evaluation["detail"] = detail
    return evaluation


def evaluate_shelf_level(
    *,
    camera_id: str,
    frame_width: int | None,
    frame_height: int | None,
    pixel_transform_id: str | None,
    candidates: list[dict[str, Any]],
    requested_level: int,
    calibration_dir: str | Path,
) -> dict[str, Any]:
    """Compare candidate centers with a specifically named, operator-selected ROI draft."""
    if not isinstance(camera_id, str) or not _IDENTIFIER.fullmatch(camera_id):
        return _result(requested_level, "CALIBRATION_INVALID", detail="camera_id is invalid")
    try:
        directory = Path(os.path.abspath(Path(calibration_dir).expanduser()))
        calibration_path = directory / f"{camera_id}.json"
    except (OSError, TypeError, ValueError) as exc:
        return _result(requested_level, "CALIBRATION_UNREADABLE", detail=str(exc))
    if directory.is_symlink() or calibration_path.is_symlink():
        return _result(
            requested_level,
            "CALIBRATION_INVALID",
            detail="calibration directory and artifact must not be symbolic links",
        )
    try:
        resolved_directory = directory.resolve(strict=False)
        resolved_artifact = calibration_path.resolve(strict=False)
    except (OSError, RuntimeError) as exc:
        return _result(requested_level, "CALIBRATION_UNREADABLE", detail=str(exc))
    if resolved_directory != directory or resolved_artifact != calibration_path:
        return _result(
            requested_level,
            "CALIBRATION_INVALID",
            detail="calibration path cannot contain symbolic-link components",
        )
    if not calibration_path.is_file():
        if calibration_path.exists():
            return _result(
                requested_level,
                "CALIBRATION_INVALID",
                detail="artifact is not a regular file",
            )
        return _result(requested_level, "CALIBRATION_NOT_CONFIGURED")
    try:
        with calibration_path.open("rb") as artifact_file:
            raw = artifact_file.read(_MAX_CALIBRATION_BYTES + 1)
    except OSError as exc:
        return _result(requested_level, "CALIBRATION_UNREADABLE", detail=str(exc))
    if len(raw) > _MAX_CALIBRATION_BYTES:
        return _result(requested_level, "CALIBRATION_INVALID", detail="artifact exceeds 1 MB")
    calibration_artifact_sha256 = hashlib.sha256(raw).hexdigest()

    def artifact_result(
        status: str,
        *,
        calibration_id: str | None = None,
        candidate_indices: list[int] | None = None,
        assignments: list[dict[str, Any]] | None = None,
        detail: str | None = None,
    ) -> dict[str, Any]:
        return _result(
            requested_level,
            status,
            calibration_id=calibration_id,
            calibration_artifact_sha256=calibration_artifact_sha256,
            candidate_indices=candidate_indices,
            assignments=assignments,
            detail=detail,
        )

    try:
        calibration = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_json_object)
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        return artifact_result("CALIBRATION_INVALID", detail=str(exc))
    if not isinstance(calibration, dict):
        return artifact_result("CALIBRATION_INVALID", detail="artifact is not an object")

    calibration_id = calibration.get("calibration_id")
    if (
        set(calibration)
        != {
            "schema_version",
            "calibration_id",
            "created_at",
            "status",
            "calibration_scope",
            "camera_id",
            "pixel_transform_id",
            "image_size_px",
            "source",
            "source_task_pick_request",
            "source_observation_status",
            "source_event_type",
            "regions_input_file",
            "regions_input_sha256",
            "regions",
            "review_required",
            "selection_enabled",
            "usable_for_robot_control",
            "arm_command_sent",
        }
        or isinstance(calibration.get("schema_version"), bool)
        or not isinstance(calibration.get("schema_version"), int)
        or calibration.get("schema_version") != 1
        or calibration.get("status") != "DRAFT_ONLY"
        or calibration.get("calibration_scope") != "CAMERA_IMAGE_SHELF_REGIONS_2D"
        or calibration.get("review_required") is not True
        or calibration.get("selection_enabled") is not False
        or calibration.get("usable_for_robot_control") is not False
        or calibration.get("arm_command_sent") is not False
        or not isinstance(calibration_id, str)
        or not _IDENTIFIER.fullmatch(calibration_id)
    ):
        return artifact_result("CALIBRATION_INVALID", detail="artifact contract is invalid")

    artifact_size = calibration.get("image_size_px")
    source = calibration.get("source")
    if not isinstance(source, dict) or set(source) != {
        "task_id",
        "event_id",
        "step_id",
        "camera_id",
        "pixel_transform_id",
        "frame_sequence",
        "captured_at",
        "frame_sha256",
        "image_path",
        "image_sha256",
        "image_size_px",
    }:
        return artifact_result(calibration_id=calibration_id, status="CALIBRATION_INVALID",
                               detail="source provenance is missing")
    source_task_id = source.get("task_id")
    source_event_id = source.get("event_id")
    source_step_id = source.get("step_id")
    source_frame_sequence = source.get("frame_sequence")
    source_width = source.get("image_size_px")
    source_image_path = source.get("image_path")
    source_image_path_parts = source_image_path.split("/") if isinstance(source_image_path, str) else []
    regions_hash = calibration.get("regions_input_sha256")
    regions_file = calibration.get("regions_input_file")
    if (
        not isinstance(source_task_id, str)
        or not _TASK_ID.fullmatch(source_task_id)
        or not isinstance(source_event_id, str)
        or not _EVENT_ID.fullmatch(source_event_id)
        or (
            source_step_id is not None
            and (
                not isinstance(source_step_id, str)
                or not _IDENTIFIER.fullmatch(source_step_id)
            )
        )
        or isinstance(source_frame_sequence, bool)
        or not isinstance(source_frame_sequence, int)
        or source_frame_sequence <= 0
        or not isinstance(source.get("captured_at"), str)
        or not source["captured_at"].strip()
        or not isinstance(source_image_path, str)
        or source_image_path.startswith("/")
        or "\\" in source_image_path
        or ".." in source_image_path_parts
        or len(source_image_path_parts) != 4
        or source_image_path_parts[:3] != [".control-station", "observations", source_task_id]
        or not source_image_path.endswith(".png")
        or not isinstance(source.get("image_sha256"), str)
        or not _SHA256.fullmatch(source["image_sha256"])
        or not isinstance(source.get("frame_sha256"), str)
        or not _SHA256.fullmatch(source["frame_sha256"])
        or not isinstance(source_width, dict)
        or set(source_width) != {"width", "height"}
        or not isinstance(artifact_size, dict)
        or set(artifact_size) != {"width", "height"}
        or any(
            isinstance(size.get(axis), bool) or not isinstance(size.get(axis), int) or size[axis] <= 0
            for size in (source_width, artifact_size)
            for axis in ("width", "height")
        )
        or source_width != artifact_size
        or calibration.get("source_event_type") != "TARGET_OBSERVED"
        or not isinstance(regions_file, str)
        or not regions_file.strip()
        or Path(regions_file).name != regions_file
        or not isinstance(regions_hash, str)
        or not _SHA256.fullmatch(regions_hash)
    ):
        return artifact_result(calibration_id=calibration_id, status="CALIBRATION_INVALID",
                               detail="source provenance fields are invalid")

    artifact_camera_id = calibration.get("camera_id")
    artifact_transform_id = calibration.get("pixel_transform_id")
    image_size = artifact_size
    if artifact_camera_id != camera_id or source.get("camera_id") != camera_id:
        return artifact_result("CALIBRATION_CAMERA_MISMATCH", calibration_id=calibration_id)
    if (
        not isinstance(pixel_transform_id, str)
        or artifact_transform_id != pixel_transform_id
        or source.get("pixel_transform_id") != pixel_transform_id
    ):
        return artifact_result("CALIBRATION_PIXEL_TRANSFORM_MISMATCH", calibration_id=calibration_id)
    if (
        not isinstance(image_size, dict)
        or isinstance(frame_width, bool)
        or isinstance(frame_height, bool)
        or not isinstance(frame_width, int)
        or not isinstance(frame_height, int)
        or image_size.get("width") != frame_width
        or image_size.get("height") != frame_height
    ):
        return artifact_result("CALIBRATION_RESOLUTION_MISMATCH", calibration_id=calibration_id)

    regions_data = calibration.get("regions")
    if not isinstance(regions_data, list) or not regions_data or len(regions_data) > _MAX_REGIONS:
        return artifact_result("CALIBRATION_INVALID", calibration_id=calibration_id,
                               detail="regions are missing")
    regions: list[tuple[int, list[tuple[float, float]]]] = []
    levels: set[int] = set()
    for region in regions_data:
        if not isinstance(region, dict) or set(region) != {"shelf_level", "polygon_px"}:
            return artifact_result("CALIBRATION_INVALID", calibration_id=calibration_id,
                                   detail="region is not an object")
        level = region.get("shelf_level")
        polygon_data = region.get("polygon_px")
        if (
            isinstance(level, bool)
            or not isinstance(level, int)
            or not 1 <= level <= MAX_SHELF_LEVEL
            or level in levels
            or not isinstance(polygon_data, list)
            or len(polygon_data) < 3
            or len(polygon_data) > _MAX_POLYGON_VERTICES
        ):
            return artifact_result("CALIBRATION_INVALID", calibration_id=calibration_id,
                                   detail="region level or polygon shape is invalid")
        polygon: list[tuple[float, float]] = []
        for raw_point in polygon_data:
            point = _finite_point(raw_point)
            if point is None or not (0 <= point[0] < frame_width and 0 <= point[1] < frame_height):
                return artifact_result("CALIBRATION_INVALID", calibration_id=calibration_id,
                                       detail="polygon point is invalid or outside the calibrated image")
            polygon.append(point)
        area = sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1])
        )
        if area == 0 or len(set(polygon)) != len(polygon) or not _simple_polygon(polygon):
            return artifact_result("CALIBRATION_INVALID", calibration_id=calibration_id,
                                   detail="polygon is degenerate or repeats vertices")
        levels.add(level)
        regions.append((level, polygon))

    if requested_level not in levels:
        return artifact_result("REQUESTED_LEVEL_NOT_CONFIGURED", calibration_id=calibration_id)
    if not candidates:
        return artifact_result("NO_CANDIDATES", calibration_id=calibration_id)

    assignments: list[dict[str, Any]] = []
    matching_indices: list[int] = []
    ambiguous_assignment = False
    invalid_assignment = False
    any_region_match = False
    for index, candidate in enumerate(candidates):
        if not isinstance(candidate, dict):
            invalid_assignment = True
            assignments.append({"candidate_index": index, "status": "INVALID_CANDIDATE"})
            continue
        point = _finite_point(candidate.get("center_px"))
        if point is None or not (0 <= point[0] < frame_width and 0 <= point[1] < frame_height):
            invalid_assignment = True
            assignments.append({"candidate_index": index, "status": "INVALID_CENTER_PX"})
            continue
        containing: list[int] = []
        on_boundary: list[int] = []
        for level, polygon in regions:
            relation = _point_in_polygon(point, polygon)
            if relation == 1:
                containing.append(level)
            elif relation == 0:
                on_boundary.append(level)
        if on_boundary or len(containing) > 1:
            ambiguous_assignment = True
            assignments.append({
                "candidate_index": index,
                "status": "AMBIGUOUS_REGION_ASSIGNMENT",
                "shelf_levels": sorted(set(containing + on_boundary)),
            })
        elif containing:
            any_region_match = True
            assigned_level = containing[0]
            assignments.append({
                "candidate_index": index,
                "status": "SINGLE_REGION_MATCH",
                "shelf_level": assigned_level,
            })
            if assigned_level == requested_level:
                matching_indices.append(index)
        else:
            assignments.append({"candidate_index": index, "status": "OUTSIDE_REGIONS"})

    if ambiguous_assignment:
        status = "AMBIGUOUS_REGION_ASSIGNMENT"
    elif invalid_assignment:
        status = "INVALID_CANDIDATE_POSITION"
    elif matching_indices:
        status = "SINGLE_SHELF_LEVEL_MATCH" if len(matching_indices) == 1 else "MULTIPLE_SHELF_LEVEL_MATCHES"
    elif any_region_match:
        status = "NO_SHELF_LEVEL_MATCH"
    else:
        status = "NO_REGION_MATCH"
    return artifact_result(
        status,
        calibration_id=calibration_id,
        candidate_indices=matching_indices,
        assignments=assignments,
    )
