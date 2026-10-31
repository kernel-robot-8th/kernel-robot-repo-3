#!/usr/bin/env python3
"""Project a fresh task observation candidate onto a calibrated fixed shelf plane."""

from __future__ import annotations

import argparse
import hashlib
from http.client import HTTPException
import json
import math
import re
import sys
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from calibration_artifacts import (
    ensure_distinct_output_path,
    event_step_link_matches,
    install_file_exclusive,
    read_regular_file_bounded,
    validate_intrinsic_draft,
    validate_plane_sample_sources,
    validate_plane_source_provenance,
)

_MAX_REVIEW_ARTIFACT_BYTES = 1_000_000
_MAX_TASK_JSON_BYTES = 64 * 1024 * 1024
_MAX_SOURCE_IMAGE_BYTES = 64 * 1024 * 1024


def _unique_json_object(pairs: list[tuple[str, Any]], label: str) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"{label} contains a duplicate JSON key: {key}")
        value[key] = item
    return value


def _load_object(path: Path, label: str, maximum_bytes: int) -> dict[str, Any]:
    raw = read_regular_file_bounded(path, maximum_bytes, label)
    payload = json.loads(
        raw.decode("utf-8"),
        object_pairs_hook=lambda pairs: _unique_json_object(pairs, label),
    )
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return payload


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except (OverflowError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _managed_calibration_reference(path: Path, repository_root: Path, label: str) -> dict[str, str]:
    managed_directory = (
        repository_root / ".control-station" / "camera-calibrations"
    ).resolve()
    if path.is_symlink():
        raise ValueError(f"{label} calibration artifact cannot be a symlink")
    resolved = path.resolve()
    if (
        repository_root.resolve() not in managed_directory.parents
        or resolved.parent != managed_directory
        or resolved.suffix != ".json"
        or not resolved.is_file()
        or resolved.stat().st_size > _MAX_REVIEW_ARTIFACT_BYTES
    ):
        raise ValueError(
            f"--record-review-event requires {label} calibration JSON directly under "
            "the repository .control-station/camera-calibrations directory"
        )
    raw = resolved.read_bytes()
    if len(raw) > _MAX_REVIEW_ARTIFACT_BYTES:
        raise ValueError(f"{label} calibration artifact exceeds 1 MB")
    return {
        "path": resolved.relative_to(repository_root).as_posix(),
        "sha256": hashlib.sha256(raw).hexdigest(),
    }


def _finite_matrix(value: object, shape: tuple[int, int], label: str, np):
    try:
        matrix = np.asarray(value, dtype=np.float64)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a numeric matrix") from exc
    if matrix.shape != shape or not np.isfinite(matrix).all():
        raise ValueError(f"{label} must be a finite {shape[0]}x{shape[1]} matrix")
    return matrix


def _validated_detector_provenance(value: object) -> dict[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError("detector_provenance must be an object or null")
    weights_sha256 = value.get("weights_sha256")
    confidence_threshold = value.get("confidence_threshold")
    confidence_number = (
        _finite_number(confidence_threshold)
        if confidence_threshold is not None
        else None
    )
    inference_image_size = value.get("inference_image_size")
    device = value.get("device")
    ultralytics_version = value.get("ultralytics_version")
    if weights_sha256 is not None and (
        not isinstance(weights_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", weights_sha256)
    ):
        raise ValueError("detector weights_sha256 must be a lowercase SHA-256 or null")
    if confidence_threshold is not None and (
        confidence_number is None or not 0.0 <= confidence_number <= 1.0
    ):
        raise ValueError("detector confidence_threshold must be between 0 and 1 or null")
    if inference_image_size is not None and (
        isinstance(inference_image_size, bool)
        or not isinstance(inference_image_size, int)
        or inference_image_size <= 0
    ):
        raise ValueError("detector inference_image_size must be a positive integer or null")
    if device is not None and (not isinstance(device, str) or not device or len(device) > 80):
        raise ValueError("detector device must be a non-empty string up to 80 characters or null")
    if ultralytics_version is not None and (
        not isinstance(ultralytics_version, str)
        or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.+!_-]{0,79}", ultralytics_version)
    ):
        raise ValueError("detector ultralytics_version has an invalid format or is null")
    return {
        "weights_sha256": weights_sha256,
        "confidence_threshold": (
            confidence_number if confidence_threshold is not None else None
        ),
        "inference_image_size": inference_image_size,
        "device": device,
        "ultralytics_version": ultralytics_version,
    }


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            json.dump(payload, temporary_file, ensure_ascii=False, indent=2, allow_nan=False)
            temporary_file.write("\n")
        install_file_exclusive(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def _write_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            temporary_file.write(payload)
        install_file_exclusive(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def _verify_saved_observation_frame(
    task_id: str,
    event_payload: dict[str, Any],
    observation: dict[str, Any],
    cv2,
    np,
):
    artifact = event_payload.get("frame_artifact")
    if not isinstance(artifact, dict):
        raise ValueError("operator selection source event has no saved frame artifact")
    relative_path = artifact.get("path")
    file_sha256 = artifact.get("file_sha256")
    frame_sha256 = observation.get("frame_sha256")
    width, height = observation.get("frame_width"), observation.get("frame_height")
    if (
        artifact.get("media_type") != "image/png"
        or artifact.get("pixel_format") != "BGR8"
        or artifact.get("source_frame_sha256") != frame_sha256
        or isinstance(width, bool)
        or not isinstance(width, int)
        or width <= 0
        or isinstance(height, bool)
        or not isinstance(height, int)
        or height <= 0
        or artifact.get("width") != width
        or artifact.get("height") != height
        or not isinstance(frame_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", frame_sha256)
        or not isinstance(file_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", file_sha256)
        or not isinstance(relative_path, str)
    ):
        raise ValueError("operator selection source frame metadata is inconsistent")
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
        raise ValueError("operator selection source frame path is outside the task observation directory")
    repository_root = Path(__file__).resolve().parents[1]
    control_directory = repository_root / ".control-station"
    artifact_root = control_directory / "observations"
    task_root = artifact_root / task_id
    image_path = task_root / path_parts[3]
    if any(
        path.is_symlink()
        for path in (control_directory, artifact_root, task_root, image_path)
    ):
        raise ValueError("operator selection source frame path cannot use symbolic links")
    resolved_artifact_root = artifact_root.resolve(strict=True)
    resolved_task_root = task_root.resolve(strict=True)
    resolved_image_path = image_path.resolve(strict=True)
    if (
        resolved_artifact_root != artifact_root
        or resolved_task_root != task_root
        or resolved_task_root.parent != resolved_artifact_root
        or resolved_image_path != image_path
        or image_path.suffix != ".png"
        or not image_path.is_file()
        or image_path.stat().st_size > _MAX_SOURCE_IMAGE_BYTES
    ):
        raise ValueError("operator selection source frame resolves outside the task observation directory")
    try:
        with image_path.open("rb") as image_file:
            image_bytes = image_file.read(_MAX_SOURCE_IMAGE_BYTES + 1)
    except OSError as exc:
        raise ValueError("operator selection source PNG cannot be read") from exc
    if len(image_bytes) > _MAX_SOURCE_IMAGE_BYTES:
        raise ValueError("operator selection source PNG exceeds the 64 MiB limit")
    if hashlib.sha256(image_bytes).hexdigest() != file_sha256:
        raise ValueError("operator selection source PNG hash does not match the event")
    image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (height, width):
        raise ValueError("operator selection source PNG is invalid or has a different resolution")
    if hashlib.sha256(image.tobytes(order="C")).hexdigest() != frame_sha256:
        raise ValueError("operator selection source PNG pixels do not match the observation frame hash")
    return image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task_json", type=Path, help="GET /api/tasks/{task_id} JSON response")
    parser.add_argument("plane_calibration", type=Path, help="DRAFT_ONLY shelf plane calibration JSON")
    parser.add_argument("intrinsics", type=Path, help="referenced CAMERA_INTRINSICS_ONLY JSON")
    parser.add_argument("--event-id", required=True, help="TARGET_OBSERVED event ID")
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--candidate-index", type=int, help="explicit 0-based index within event candidates")
    selection.add_argument("--selection-event-id", help="TARGET_SELECTED operator-review event ID")
    parser.add_argument("--max-age-seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path, help="draft JSON output path")
    parser.add_argument(
        "--record-review-event",
        action="store_true",
        help="after saving, append the review-only draft to the pending task history",
    )
    parser.add_argument(
        "--control-station-url",
        default="http://127.0.0.1:8000",
        help="local control-station base URL used only with --record-review-event",
    )
    parser.add_argument(
        "--preview-output",
        type=Path,
        help="optional review-only PNG overlay; requires --selection-event-id",
    )
    args = parser.parse_args()
    if args.candidate_index is not None and args.candidate_index < 0:
        parser.error("--candidate-index must be zero or greater")
    if args.selection_event_id is not None and not re.fullmatch(
        r"EVT-[A-F0-9]{12}", args.selection_event_id
    ):
        parser.error("--selection-event-id format is invalid")
    if args.preview_output is not None and args.selection_event_id is None:
        parser.error("--preview-output requires --selection-event-id")
    if args.preview_output is not None and args.preview_output.suffix.lower() != ".png":
        parser.error("--preview-output must have a .png extension")
    if args.record_review_event:
        server_url = urlsplit(args.control_station_url)
        if (
            server_url.scheme != "http"
            or server_url.hostname not in {"127.0.0.1", "localhost"}
            or server_url.username is not None
            or server_url.password is not None
            or server_url.path not in {"", "/"}
            or server_url.query
            or server_url.fragment
        ):
            parser.error("--record-review-event requires an HTTP localhost control-station URL")
        try:
            if server_url.port is not None and not 1 <= server_url.port <= 65535:
                parser.error("--control-station-url port is invalid")
        except ValueError:
            parser.error("--control-station-url port is invalid")
    if not math.isfinite(args.max_age_seconds) or not 0.1 <= args.max_age_seconds <= 60:
        parser.error("--max-age-seconds must be between 0.1 and 60")

    try:
        import cv2
        import numpy as np
    except ImportError:
        print("Install OpenCV and NumPy in the control-station environment first.", file=sys.stderr)
        return 2

    try:
        task_response = _load_object(
            args.task_json, "task response", _MAX_TASK_JSON_BYTES
        )
        task = task_response.get("task", task_response)
        if not isinstance(task, dict) or not isinstance(task.get("events"), list):
            raise ValueError("task JSON must contain a task object with an events array")
        if task.get("status") != "PENDING":
            raise ValueError("task must still be PENDING when projecting a camera observation")
        task_id = task.get("id")
        if not isinstance(task_id, str) or not re.fullmatch(r"TASK-[A-F0-9]{8}", task_id):
            raise ValueError("task id is missing or invalid")
        event_id_matches = [
            item for item in task["events"]
            if isinstance(item, dict) and item.get("id") == args.event_id
        ]
        if not event_id_matches:
            raise ValueError("TARGET_OBSERVED event_id was not found in task history")
        if len(event_id_matches) != 1:
            raise ValueError("event_id is duplicated in task history")
        event = event_id_matches[0]
        if event.get("event_type") != "TARGET_OBSERVED":
            raise ValueError("event_id does not identify a TARGET_OBSERVED event")
        if event.get("task_id") != task_id:
            raise ValueError("event task_id does not match the task response")
        if not event_step_link_matches(task, event, "DETECT_TARGETS"):
            raise ValueError("observation event step_id does not match the pending DETECT_TARGETS step")
        if event.get("previous_status") != "PENDING" or event.get("new_status") != "PENDING":
            raise ValueError("observation event must not represent a task state transition")
        event_payload = event.get("payload")
        if not isinstance(event_payload, dict):
            raise ValueError("event payload must be an object")
        if event_payload.get("event") != "TARGET_OBSERVED":
            raise ValueError("event payload type does not match TARGET_OBSERVED history entry")
        if event_payload.get("source") != "camera_observation":
            raise ValueError("event payload source is not the camera observation adapter")
        if event_payload.get("task_state_changed") is not False:
            raise ValueError("observation event does not confirm that task state was unchanged")
        if event_payload.get("arm_command_sent") is not False:
            raise ValueError("observation event does not confirm that no arm command was sent")
        observation = event_payload.get("observation")
        if not isinstance(observation, dict):
            raise ValueError("event does not contain an observation object")
        observation_schema_version = observation.get("observation_schema_version")
        if (
            isinstance(observation_schema_version, bool)
            or not isinstance(observation_schema_version, int)
            or observation_schema_version not in {2, 3}
        ):
            raise ValueError("unsupported target observation schema version")
        if observation.get("decision_scope") != "OBSERVATION_ONLY" or observation.get("arm_command_sent") is not False:
            raise ValueError("event observation is not explicitly observation-only")
        pick_request = event_payload.get("pick_request")
        if not isinstance(pick_request, dict) or pick_request.get("category") != observation.get("target_class"):
            raise ValueError("event request category does not match the observed target class")
        task_pick_request = task.get("pick_request")
        if not isinstance(task_pick_request, dict) or task_pick_request != pick_request:
            raise ValueError("observation event request does not match the task's stored pick_request")
        detector_provenance = _validated_detector_provenance(
            observation.get("detector_provenance")
        )
        selection_event: dict[str, Any] | None = None
        selection_payload: dict[str, Any] | None = None
        review_snapshot_artifact: dict[str, Any] | None = None
        review_snapshot_image = None
        if args.selection_event_id:
            selection_matches = [
                item for item in task["events"]
                if isinstance(item, dict) and item.get("id") == args.selection_event_id
            ]
            if len(selection_matches) != 1:
                raise ValueError("TARGET_SELECTED event ID must occur exactly once in task history")
            selection_event = selection_matches[0]
            selection_payload = selection_event.get("payload")
            if (
                selection_event.get("event_type") != "TARGET_SELECTED"
                or selection_event.get("task_id") != task_id
                or not event_step_link_matches(task, selection_event, "SELECT_TARGET")
                or selection_event.get("previous_status") != "PENDING"
                or selection_event.get("new_status") != "PENDING"
                or not isinstance(selection_payload, dict)
                or selection_payload.get("event") != "TARGET_SELECTED"
                or selection_payload.get("source") != "operator_review"
                or selection_payload.get("selection_scope") != "HUMAN_REVIEW_ONLY"
                or selection_payload.get("observation_event_id") != args.event_id
                or selection_payload.get("pick_request") != pick_request
                or selection_payload.get("detector_provenance")
                != observation.get("detector_provenance")
                or selection_payload.get("task_state_changed") is not False
                or selection_payload.get("arm_command_sent") is not False
                or event_payload.get("frame_saved") is not True
            ):
                raise ValueError("TARGET_SELECTED event does not refer to this pending observation")
            args.candidate_index = selection_payload.get("candidate_index")
            if (
                isinstance(args.candidate_index, bool)
                or not isinstance(args.candidate_index, int)
                or args.candidate_index < 0
            ):
                raise ValueError("TARGET_SELECTED event candidate_index is invalid")
            if any(
                selection_payload.get(key) != observation.get(key)
                for key in ("camera_id", "frame_sequence", "frame_sha256", "pixel_transform_id")
            ):
                raise ValueError("TARGET_SELECTED event frame provenance differs from its observation")
            review_snapshot_image = _verify_saved_observation_frame(
                task_id, event_payload, observation, cv2, np
            )
            frame_artifact = event_payload.get("frame_artifact")
            if not isinstance(frame_artifact, dict):
                raise ValueError("operator selection source event has no saved frame artifact")
            review_snapshot_artifact = {
                "path": frame_artifact["path"],
                "file_sha256": frame_artifact["file_sha256"],
                "media_type": frame_artifact["media_type"],
            }
        if observation.get("frame_freshness") != "FRESH":
            raise ValueError("observation frame was not fresh when it was recorded")
        if observation.get("status") not in {"SINGLE_CANDIDATE", "AMBIGUOUS"}:
            raise ValueError("observation status has no selectable candidate")
        candidates = observation.get("candidates")
        if not isinstance(candidates, list) or not 0 <= args.candidate_index < len(candidates):
            raise ValueError("candidate index is outside the recorded candidate list")
        candidate = candidates[args.candidate_index]
        if not isinstance(candidate, dict):
            raise ValueError("selected candidate must be an object")
        if selection_payload is not None and selection_payload.get("candidate") != candidate:
            raise ValueError("TARGET_SELECTED candidate snapshot differs from the source observation")
        candidate_class = candidate.get("class_name")
        target_class = observation.get("target_class")
        if (
            not isinstance(candidate_class, str)
            or not isinstance(target_class, str)
            or candidate_class.casefold() != target_class.casefold()
        ):
            raise ValueError("candidate class_name does not match the observed target class")
        confidence = candidate.get("confidence")
        confidence_number = _finite_number(confidence)
        if confidence_number is None or not 0 <= confidence_number <= 1:
            raise ValueError("candidate confidence must be between 0 and 1")
        camera_id = observation.get("camera_id")
        pixel_transform_id = observation.get("pixel_transform_id")
        frame_sha256 = observation.get("frame_sha256")
        captured_at = observation.get("captured_at")
        frame_sequence = observation.get("frame_sequence")
        if not isinstance(camera_id, str) or not camera_id:
            raise ValueError("observation has no camera_id")
        if not re.fullmatch(r"[A-Za-z0-9_.-]{1,80}", camera_id):
            raise ValueError("camera_id has an unsupported format")
        if not isinstance(pixel_transform_id, str) or not re.fullmatch(
            r"[A-Za-z0-9_.-]{1,80}", pixel_transform_id
        ):
            raise ValueError("observation pixel_transform_id is missing or invalid")
        if not isinstance(frame_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", frame_sha256):
            raise ValueError("observation frame_sha256 is missing or invalid")
        if (
            isinstance(frame_sequence, bool)
            or not isinstance(frame_sequence, int)
            or frame_sequence <= 0
        ):
            raise ValueError("observation frame_sequence must be a positive integer")
        if candidate.get("camera_id") != camera_id:
            raise ValueError("candidate camera_id does not match its observation")
        if candidate.get("pixel_transform_id") != pixel_transform_id:
            raise ValueError("candidate pixel_transform_id does not match its observation")
        if candidate.get("frame_sha256") != frame_sha256:
            raise ValueError("candidate frame_sha256 does not match its observation")
        if candidate.get("frame_sequence") != frame_sequence:
            raise ValueError("candidate frame_sequence does not match its observation")
        if candidate.get("captured_at") != captured_at:
            raise ValueError("candidate captured_at does not match its observation")
        if not isinstance(captured_at, str):
            raise ValueError("observation has no captured_at timestamp")
        normalized_time = captured_at[:-1] + "+00:00" if captured_at.endswith("Z") else captured_at
        captured_time = datetime.fromisoformat(normalized_time)
        if captured_time.tzinfo is None:
            raise ValueError("captured_at timestamp must include a timezone")
        age_seconds = (datetime.now(timezone.utc) - captured_time.astimezone(timezone.utc)).total_seconds()
        if age_seconds < -1.0:
            raise ValueError(
                f"observation age {age_seconds:.3f}s is outside the allowed window "
                "[-1.0, infinity]s"
            )
        if selection_event is None and age_seconds > args.max_age_seconds:
            raise ValueError(
                f"observation age {age_seconds:.3f}s exceeds the direct-index limit "
                f"of {args.max_age_seconds:.3f}s"
            )

        plane = _load_object(
            args.plane_calibration, "plane calibration", _MAX_REVIEW_ARTIFACT_BYTES
        )
        intrinsics = _load_object(
            args.intrinsics, "intrinsic calibration", _MAX_REVIEW_ARTIFACT_BYTES
        )
        intrinsic_review = validate_intrinsic_draft(intrinsics)
        plane_source_provenance = validate_plane_source_provenance(plane)
        plane_source_images = validate_plane_sample_sources(plane)
        if plane.get("status") != "DRAFT_ONLY" or plane.get("calibration_scope") != "CAMERA_TO_FIXED_PLANE_ONLY":
            raise ValueError("plane calibration must be CAMERA_TO_FIXED_PLANE_ONLY and DRAFT_ONLY")
        if plane.get("input_pixel_domain") != "DISTORTED_IMAGE_PIXELS_UNDISTORTED_WITH_REFERENCED_INTRINSICS":
            raise ValueError("plane calibration has an unsupported input pixel domain")
        if intrinsics.get("status") != "DRAFT_ONLY" or intrinsics.get("calibration_scope") != "CAMERA_INTRINSICS_ONLY":
            raise ValueError("intrinsics must be CAMERA_INTRINSICS_ONLY and DRAFT_ONLY")
        if (
            plane.get("usable_for_robot_control") is not False
            or plane.get("arm_command_sent") is not False
            or intrinsics.get("usable_for_robot_control") is not False
        ):
            raise ValueError("calibration artifacts must explicitly be unusable for robot control")
        if plane.get("camera_id") != camera_id or intrinsics.get("camera_id") != camera_id:
            raise ValueError("observation and calibration camera_id values must match")
        if (
            plane.get("pixel_transform_id") != pixel_transform_id
            or intrinsics.get("pixel_transform_id") != pixel_transform_id
        ):
            raise ValueError("observation and calibration pixel_transform_id values must match")
        if plane.get("intrinsic_calibration_id") != intrinsics.get("calibration_id"):
            raise ValueError("plane calibration references a different intrinsic calibration")
        if not isinstance(plane.get("calibration_id"), str) or not isinstance(
            intrinsics.get("calibration_id"), str
        ):
            raise ValueError("both calibration_id values are required")
        calibration_artifacts = None
        if args.record_review_event:
            repository_root = Path(__file__).resolve().parents[1]
            calibration_artifacts = {
                "plane": _managed_calibration_reference(
                    args.plane_calibration, repository_root, "plane"
                ),
                "intrinsics": _managed_calibration_reference(
                    args.intrinsics, repository_root, "intrinsic"
                ),
            }

        image_size = plane.get("image_size_px")
        if image_size != intrinsics.get("image_size_px"):
            raise ValueError("plane and intrinsic calibration image sizes differ")
        if not isinstance(image_size, dict):
            raise ValueError("calibration image_size_px is missing")
        width, height = image_size.get("width"), image_size.get("height")
        if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in (width, height)):
            raise ValueError("calibration image_size_px dimensions must be positive integers")
        if candidate.get("frame_width") != width or candidate.get("frame_height") != height:
            raise ValueError("candidate image resolution does not match the calibration")

        center = candidate.get("center_px")
        if (
            not isinstance(center, list)
            or len(center) != 2
            or any(_finite_number(value) is None for value in center)
        ):
            raise ValueError("candidate center_px must contain two numbers")
        pixel = np.asarray([_finite_number(value) for value in center], dtype=np.float64)
        if not np.isfinite(pixel).all() or not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
            raise ValueError("candidate center_px is outside the calibrated image")
        bbox = candidate.get("bbox_xyxy_px")
        if (
            not isinstance(bbox, list)
            or len(bbox) != 4
            or any(_finite_number(value) is None for value in bbox)
        ):
            raise ValueError("candidate bbox_xyxy_px must contain four numbers")
        x1, y1, x2, y2 = (_finite_number(value) for value in bbox)
        if (
            x1 < 0
            or y1 < 0
            or x2 > width
            or y2 > height
            or x2 <= x1
            or y2 <= y1
        ):
            raise ValueError("candidate bbox_xyxy_px is invalid or outside the calibrated image")
        if x1 <= 0 or y1 <= 0 or x2 >= width or y2 >= height:
            raise ValueError("candidate bbox touches the image boundary and may be truncated")
        if not (x1 <= pixel[0] <= x2 and y1 <= pixel[1] <= y2):
            raise ValueError("candidate center_px does not lie within bbox_xyxy_px")
        projection_pixel = np.asarray(((x1 + x2) / 2.0, y2), dtype=np.float64)
        camera_matrix = _finite_matrix(intrinsics.get("camera_matrix"), (3, 3), "camera_matrix", np)
        if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
            raise ValueError("camera_matrix focal lengths must be positive")
        try:
            distortion = np.asarray(
                intrinsics.get("distortion_coefficients"), dtype=np.float64
            ).reshape(-1, 1)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError(
                "distortion_coefficients must be finite OpenCV coefficients"
            ) from exc
        if distortion.size not in {4, 5, 8, 12, 14} or not np.isfinite(distortion).all():
            raise ValueError("distortion_coefficients must be finite OpenCV coefficients")
        pixel_undistorted = cv2.undistortPoints(
            projection_pixel.reshape(1, 1, 2), camera_matrix, distortion, P=camera_matrix
        )
        samples = plane.get("samples")
        if not isinstance(samples, list):
            raise ValueError("plane calibration samples are missing")
        raw_support_pixels: list[list[float]] = []
        support_roles: list[str] = []
        support_plane_points: list[list[float]] = []
        stored_projected_points: list[list[float]] = []
        stored_residuals: list[float] = []
        for index, sample in enumerate(samples):
            if not isinstance(sample, dict):
                raise ValueError(f"plane calibration samples[{index}] must be an object")
            role = sample.get("role")
            if role not in {"fit", "validation"}:
                raise ValueError(f"plane calibration samples[{index}].role is invalid")
            sample_pixel = sample.get("image_px")
            if (
                not isinstance(sample_pixel, list)
                or len(sample_pixel) != 2
                or any(_finite_number(value) is None for value in sample_pixel)
            ):
                raise ValueError(f"plane calibration samples[{index}].image_px is invalid")
            point = np.asarray([_finite_number(value) for value in sample_pixel], dtype=np.float64)
            if (
                not np.isfinite(point).all()
                or not (0 <= point[0] < width and 0 <= point[1] < height)
            ):
                raise ValueError(f"plane calibration samples[{index}].image_px is out of bounds")
            raw_support_pixels.append([float(point[0]), float(point[1])])
            sample_plane = sample.get("plane_xy")
            if (
                not isinstance(sample_plane, list)
                or len(sample_plane) != 2
                or any(_finite_number(value) is None for value in sample_plane)
            ):
                raise ValueError(f"plane calibration samples[{index}].plane_xy is invalid")
            plane_point = [_finite_number(value) for value in sample_plane]
            support_roles.append(role)
            support_plane_points.append(plane_point)

            projected_sample = sample.get("projected_plane_xy")
            if (
                not isinstance(projected_sample, list)
                or len(projected_sample) != 2
                or any(_finite_number(value) is None for value in projected_sample)
            ):
                raise ValueError(f"plane calibration samples[{index}].projected_plane_xy is invalid")
            projected_point = [_finite_number(value) for value in projected_sample]
            residual = sample.get("residual")
            residual_number = _finite_number(residual)
            if residual_number is None or residual_number < 0:
                raise ValueError(f"plane calibration samples[{index}] projection metrics are invalid")
            stored_projected_points.append(projected_point)
            stored_residuals.append(residual_number)
        if len(raw_support_pixels) < 4:
            raise ValueError("plane calibration requires at least four projection support samples")
        support_undistorted = cv2.undistortPoints(
            np.asarray(raw_support_pixels, dtype=np.float64).reshape(-1, 1, 2),
            camera_matrix,
            distortion,
            P=camera_matrix,
        ).reshape(-1, 2)
        support_hull = cv2.convexHull(support_undistorted.astype(np.float32))
        undistorted_anchor = pixel_undistorted.reshape(2).astype(np.float32)
        if cv2.pointPolygonTest(support_hull, tuple(map(float, undistorted_anchor)), False) < 0:
            raise ValueError("candidate bbox-bottom-center proxy is outside the calibration sample convex hull")
        homography = _finite_matrix(
            plane.get("homography_undistorted_image_px_to_plane_xy"),
            (3, 3),
            "homography_undistorted_image_px_to_plane_xy",
            np,
        )
        if np.linalg.matrix_rank(homography) < 3:
            raise ValueError("homography must be invertible")
        plane_xy = cv2.perspectiveTransform(pixel_undistorted, homography).reshape(2)
        if not np.isfinite(plane_xy).all():
            raise ValueError("pixel projection produced a non-finite plane coordinate")
        plane_frame = plane.get("plane_frame")
        if not isinstance(plane_frame, dict):
            raise ValueError("plane_frame metadata is missing")
        frame_id, unit, axis_definition = (
            plane_frame.get("frame_id"), plane_frame.get("unit"), plane_frame.get("axis_definition")
        )
        if not isinstance(frame_id, str) or unit not in {"m", "mm"}:
            raise ValueError("plane_frame requires a frame_id and unit m or mm")
        if not re.fullmatch(r"[A-Za-z0-9_./-]{1,80}", frame_id):
            raise ValueError("plane_frame.frame_id has an unsupported format")
        if not isinstance(axis_definition, str) or not axis_definition.strip() or len(axis_definition) > 200:
            raise ValueError("plane_frame axis_definition is missing")
        if plane.get("review_required") is not True or intrinsics.get("review_required") is not True:
            raise ValueError("both calibration drafts must require operator review")
        fit_point_count = plane.get("fit_point_count")
        validation_point_count = plane.get("validation_point_count")
        if (
            isinstance(fit_point_count, bool)
            or not isinstance(fit_point_count, int)
            or fit_point_count < 4
            or isinstance(validation_point_count, bool)
            or not isinstance(validation_point_count, int)
            or validation_point_count < 2
        ):
            raise ValueError("plane calibration requires at least four fit and two validation points")
        fit_rms_residual = plane.get("fit_rms_residual")
        validation_rms_residual = plane.get("validation_rms_residual")
        fit_rms_number = _finite_number(fit_rms_residual)
        validation_rms_number = _finite_number(validation_rms_residual)
        if (
            fit_rms_number is None
            or fit_rms_number < 0
            or validation_rms_number is None
            or validation_rms_number < 0
        ):
            raise ValueError("plane calibration residuals must be finite non-negative numbers")
        if plane.get("residual_unit") != unit:
            raise ValueError("plane calibration residual unit does not match its frame unit")
        if any(sample.get("residual_unit") != unit for sample in samples):
            raise ValueError("plane calibration sample residual units do not match its frame unit")
        actual_fit_count = sum(role == "fit" for role in support_roles)
        actual_validation_count = sum(role == "validation" for role in support_roles)
        if (fit_point_count, validation_point_count) != (
            actual_fit_count,
            actual_validation_count,
        ):
            raise ValueError("plane calibration point counts do not match its labeled samples")

        recomputed_projected = cv2.perspectiveTransform(
            support_undistorted.reshape(-1, 1, 2), homography
        ).reshape(-1, 2)
        sample_plane_array = np.asarray(support_plane_points, dtype=np.float64)
        recomputed_residuals = np.linalg.norm(recomputed_projected - sample_plane_array, axis=1)
        if not np.allclose(
            recomputed_projected,
            np.asarray(stored_projected_points, dtype=np.float64),
            rtol=1e-7,
            atol=1e-9,
        ) or not np.allclose(
            recomputed_residuals,
            np.asarray(stored_residuals, dtype=np.float64),
            rtol=1e-7,
            atol=1e-9,
        ):
            raise ValueError("plane calibration stored point projections/residuals do not match its homography")
        role_array = np.asarray(support_roles)
        recomputed_fit_rms = float(
            math.sqrt(float(np.mean(recomputed_residuals[role_array == "fit"] ** 2)))
        )
        recomputed_validation_rms = float(
            math.sqrt(float(np.mean(recomputed_residuals[role_array == "validation"] ** 2)))
        )
        if not math.isclose(fit_rms_number, recomputed_fit_rms, rel_tol=1e-7, abs_tol=1e-9):
            raise ValueError("plane calibration fit RMS does not match its samples")
        if not math.isclose(
            validation_rms_number, recomputed_validation_rms, rel_tol=1e-7, abs_tol=1e-9
        ):
            raise ValueError("plane calibration validation RMS does not match its samples")
        intrinsic_view_count = intrinsic_review["view_count"]
        intrinsic_rms_px = intrinsic_review["rms_reprojection_error_px"]
        source_images = intrinsic_review["source_images"]
        per_image_errors = intrinsic_review["per_image_reprojection_errors"]

        draft_id = str(uuid.uuid4())
        unapplied_request_constraints = {
            key: pick_request[key]
            for key in ("color", "shelf_level")
            if key in pick_request
        }
        draft = {
            "schema_version": 2,
            "id": draft_id,
            "created_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "status": "DRAFT_ONLY",
            "draft_scope": "CAMERA_PIXEL_TO_FIXED_SHELF_PLANE_2D",
            "usable_for_robot_control": False,
            "arm_command_sent": False,
            "task_id": task_id,
            "event_id": event.get("id"),
            "observation_step_id": event.get("step_id"),
            "candidate_index": args.candidate_index,
            "candidate_selection_source": (
                "TARGET_SELECTED_EVENT" if selection_event is not None else "CLI_ARGUMENT"
            ),
            "selection_is_review_only": selection_event is not None,
            "operator_selection_event_id": (
                selection_event.get("id") if selection_event is not None else None
            ),
            "selection_step_id": (
                selection_event.get("step_id") if selection_event is not None else None
            ),
            "review_snapshot_artifact": review_snapshot_artifact,
            "camera_id": camera_id,
            "pixel_transform_id": pixel_transform_id,
            "frame_sha256": frame_sha256,
            "observation_schema_version": observation_schema_version,
            "detector_provenance": detector_provenance,
            "pick_request": pick_request,
            "unapplied_request_constraints": unapplied_request_constraints,
            "target_selection_complete": (
                selection_event is not None and not unapplied_request_constraints
            ),
            "captured_at": captured_at,
            "frame_sequence": frame_sequence,
            "candidate": candidate,
            "projection_anchor": "DETECTION_BBOX_BOTTOM_CENTER_PROXY",
            "projection_pixel_px": {
                "x": float(projection_pixel[0]),
                "y": float(projection_pixel[1]),
            },
            "projection_anchor_is_proxy": True,
            "projection_assumption": "bbox bottom center approximates the object's support point; this is unverified",
            "physical_contact_point_verified": False,
            "usable_for_pick_target": False,
            "source_pixel_domain": "DISTORTED_IMAGE_PIXELS",
            "calibration_support_scope": "WITHIN_CONVEX_HULL_OF_CALIBRATION_SAMPLES",
            "plane_calibration_id": plane.get("calibration_id"),
            "intrinsic_calibration_id": intrinsics.get("calibration_id"),
            **(
                {"calibration_artifacts": calibration_artifacts}
                if calibration_artifacts is not None
                else {}
            ),
            "calibration_review_summary": {
                "review_required": True,
                "plane_fit_point_count": fit_point_count,
                "plane_validation_point_count": validation_point_count,
                "plane_fit_rms_residual": float(fit_rms_residual),
                "plane_validation_rms_residual": float(validation_rms_residual),
                "plane_residual_unit": unit,
                "plane_correspondence_source": plane_source_provenance,
                "plane_source_images": plane_source_images,
                "intrinsic_calibrated_view_count": intrinsic_view_count,
                "intrinsic_rms_reprojection_error_px": float(intrinsic_rms_px),
                "intrinsic_checkerboard": dict(intrinsic_review["checkerboard"]),
                "intrinsic_reprojection_record_count": len(per_image_errors),
                "intrinsic_source_images": list(source_images),
                "intrinsic_per_image_reprojection_errors": [
                    {
                        "file": image_error["file"],
                        "sha256": image_error["sha256"],
                        "reprojection_error_px": float(image_error["reprojection_error_px"]),
                    }
                    for image_error in per_image_errors
                ],
            },
            "plane_frame": {
                "frame_id": frame_id,
                "unit": unit,
                "axis_definition": axis_definition,
            },
            "plane_xy": {"x": float(plane_xy[0]), "y": float(plane_xy[1])},
            "age_at_projection_seconds": age_seconds,
            "projection_freshness_policy": (
                "SAVED_REVIEW_SNAPSHOT_NOT_FOR_CURRENT_CONTROL"
                if selection_event is not None
                else "MAX_AGE_ENFORCED"
            ),
            "review_required": True,
        }
        output_path = args.output or (
            Path(".control-station/camera-calibrations") / f"{camera_id}-projection-{draft_id}.json"
        )
        if args.record_review_event:
            repository_root = Path(__file__).resolve().parents[1]
            managed_directory = (
                repository_root / ".control-station" / "camera-calibrations"
            ).resolve()
            resolved_output = output_path.resolve()
            if (
                repository_root not in managed_directory.parents
                or resolved_output.parent != managed_directory
                or resolved_output.suffix != ".json"
                or output_path.is_symlink()
                or resolved_output.exists()
            ):
                raise ValueError(
                    "--record-review-event requires the output JSON directly under "
                    "the repository .control-station/camera-calibrations directory, "
                    "with a new non-symlink filename"
                )
        repository_root = Path(__file__).resolve().parents[1]
        input_paths = [args.task_json, args.plane_calibration, args.intrinsics]
        source_frame_path: Path | None = None
        if review_snapshot_artifact is not None:
            source_frame_path = (repository_root / review_snapshot_artifact["path"]).resolve()
            input_paths.append(source_frame_path)
        ensure_distinct_output_path(output_path, input_paths)
        if args.preview_output is not None:
            if review_snapshot_image is None or review_snapshot_artifact is None:
                raise ValueError("a saved review snapshot is required for the preview")
            preview_path = args.preview_output
            ensure_distinct_output_path(preview_path, [*input_paths, output_path])

            preview = review_snapshot_image.copy()
            cv2.rectangle(
                preview,
                (int(round(x1)), int(round(y1))),
                (int(round(x2)), int(round(y2))),
                (0, 220, 255),
                2,
            )
            cv2.drawMarker(
                preview,
                (int(round(projection_pixel[0])), int(round(projection_pixel[1]))),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=20,
                thickness=2,
            )
            preview_text = (
                "DRAFT ONLY - NOT A ROBOT TARGET",
                "Anchor: bbox bottom-center proxy; contact point unverified",
                f"Shelf plane XY: {plane_xy[0]:.3f}, {plane_xy[1]:.3f} {unit}",
            )
            for line_index, line in enumerate(preview_text):
                origin = (12, 24 + line_index * 24)
                cv2.putText(
                    preview,
                    line,
                    origin,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (0, 0, 0),
                    3,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    preview,
                    line,
                    origin,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            encoded_ok, encoded_preview = cv2.imencode(".png", preview)
            if not encoded_ok:
                raise ValueError("could not encode the review preview PNG")
            preview_bytes = encoded_preview.tobytes()
            _write_bytes(preview_path, preview_bytes)
            draft["review_preview_artifact"] = {
                "path": str(preview_path),
                "media_type": "image/png",
                "file_sha256": hashlib.sha256(preview_bytes).hexdigest(),
                "source_frame_sha256": frame_sha256,
                "review_only": True,
                "usable_for_robot_control": False,
            }
        _write_json(output_path, draft)
    except (
        OSError,
        RuntimeError,
        UnicodeError,
        TypeError,
        ValueError,
        OverflowError,
        KeyError,
        np.linalg.LinAlgError,
        cv2.error,
    ) as exc:
        print(f"Could not create a fixed-plane coordinate draft: {exc}", file=sys.stderr)
        return 2

    print(f"2D fixed-plane coordinate draft: {output_path}")
    print(f"plane_frame={frame_id} · unit={unit} · xy={draft['plane_xy']}")
    if args.record_review_event:
        repository_root = Path(__file__).resolve().parents[1]
        resolved_output = output_path.resolve()
        relative_path = resolved_output.relative_to(repository_root).as_posix()
        if resolved_output.stat().st_size > _MAX_REVIEW_ARTIFACT_BYTES:
            print(
                "Projection draft saved, but it exceeds the 1 MB task-history artifact limit.",
                file=sys.stderr,
            )
            return 2
        artifact_bytes = resolved_output.read_bytes()
        if len(artifact_bytes) > _MAX_REVIEW_ARTIFACT_BYTES:
            print(
                "Projection draft saved, but it exceeds the 1 MB task-history artifact limit.",
                file=sys.stderr,
            )
            return 2
        artifact_sha256 = hashlib.sha256(artifact_bytes).hexdigest()
        request_body = json.dumps(
            {"artifact_path": relative_path, "artifact_sha256": artifact_sha256}
        ).encode("utf-8")
        request = Request(
            f"{args.control_station_url.rstrip('/')}/api/tasks/{task_id}/projection-draft",
            data=request_body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=5.0) as response:
                response_bytes = response.read(4097)
                if len(response_bytes) > 4096:
                    raise ValueError("server response exceeded 4096 bytes")
                result = json.loads(
                    response_bytes.decode("utf-8"),
                    object_pairs_hook=lambda pairs: _unique_json_object(
                        pairs, "API response"
                    ),
                )
        except (
            HTTPError,
            HTTPException,
            URLError,
            OSError,
            TimeoutError,
            UnicodeError,
            ValueError,
            RecursionError,
        ) as exc:
            print(
                "Projection draft saved, but task-history recording could not be confirmed. "
                f"Inspect the task event history before retrying: {exc}",
                file=sys.stderr,
            )
            return 2
        review_event_id = result.get("event_id") if isinstance(result, dict) else None
        if (
            not isinstance(review_event_id, str)
            or result.get("execution_started") is not False
            or not isinstance(result.get("already_recorded"), bool)
        ):
            print(
                "Projection draft saved, but task-history recording could not be confirmed. "
                "Inspect the task event history before retrying.",
                file=sys.stderr,
            )
            return 2
        print(f"task_review_event={review_event_id} · ESTIMATE_TARGET_POSE remains pending")
    if draft["operator_selection_event_id"]:
        print(f"operator_selection_event={draft['operator_selection_event_id']} · review-only saved snapshot")
    if draft.get("review_preview_artifact"):
        print(f"review_preview={draft['review_preview_artifact']['path']} · review-only overlay")
    if draft["unapplied_request_constraints"]:
        print(f"Unapplied target constraints: {draft['unapplied_request_constraints']}")
    print("DRAFT_ONLY · z/OMX transform/IK/workspace/grasp unavailable · no robot command sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
