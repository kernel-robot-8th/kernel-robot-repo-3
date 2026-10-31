"""Validation for review-only fixed-shelf-plane projection artifacts."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any


_TASK_ID = re.compile(r"^TASK-[A-F0-9]{8}$")
_EVENT_ID = re.compile(r"^EVT-[A-F0-9]{12}$")
_STEP_ID = re.compile(r"^STEP-[A-F0-9]{12}$")
_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
_FRAME_ID = re.compile(r"^[A-Za-z0-9_./-]{1,80}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_MAX_ARTIFACT_BYTES = 1_000_000


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON object key: {key}")
        value[key] = item
    return value


class ProjectionDraftError(ValueError):
    """Raised when a projection artifact is not linked to this pending task."""


def _unique_pending_step_id(task: dict[str, Any], step_type: str) -> str | None:
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
    return step_id if isinstance(step_id, str) and _STEP_ID.fullmatch(step_id) else None


def _unique_event(task: dict[str, Any], event_id: object, event_type: str) -> dict[str, Any]:
    if not isinstance(event_id, str) or not _EVENT_ID.fullmatch(event_id):
        raise ProjectionDraftError(f"{event_type} event ID is invalid")
    events = task.get("events")
    if not isinstance(events, list):
        raise ProjectionDraftError("task event history is unavailable")
    matches = [event for event in events if isinstance(event, dict) and event.get("id") == event_id]
    if len(matches) != 1:
        raise ProjectionDraftError(f"{event_type} event ID must occur exactly once")
    event = matches[0]
    if (
        event.get("event_type") != event_type
        or event.get("task_id") != task.get("id")
        or event.get("previous_status") != "PENDING"
        or event.get("new_status") != "PENDING"
    ):
        raise ProjectionDraftError(f"{event_type} event does not belong to this pending task")
    return event


def _same_finite_number(left: object, right: object) -> bool:
    left_number = _finite_number(left)
    right_number = _finite_number(right)
    return (
        left_number is not None
        and right_number is not None
        and math.isclose(left_number, right_number, rel_tol=1e-7, abs_tol=1e-9)
    )


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except (OverflowError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _same_integer(left: object, right: object) -> bool:
    return (
        not isinstance(left, bool)
        and not isinstance(right, bool)
        and isinstance(left, int)
        and isinstance(right, int)
        and left == right
    )


def _matches_image_size(value: object, width: int, height: int) -> bool:
    return (
        isinstance(value, dict)
        and set(value) == {"width", "height"}
        and _same_integer(value.get("width"), width)
        and _same_integer(value.get("height"), height)
    )


def _read_artifact(
    repository_root: Path, relative_path: object, expected_sha256: object
) -> tuple[dict[str, Any], str]:
    if (
        not isinstance(relative_path, str)
        or not isinstance(expected_sha256, str)
        or not _SHA256.fullmatch(expected_sha256)
    ):
        raise ProjectionDraftError("artifact path and SHA-256 are required")
    parts = relative_path.split("/")
    if (
        relative_path.startswith("/")
        or "\\" in relative_path
        or ".." in parts
        or len(parts) != 3
        or parts[:2] != [".control-station", "camera-calibrations"]
        or not parts[2].endswith(".json")
        or Path(parts[2]).name != parts[2]
    ):
        raise ProjectionDraftError("artifact must be a JSON file in camera-calibrations")
    try:
        root = repository_root.resolve()
        unresolved_artifact_path = root / relative_path
        if unresolved_artifact_path.is_symlink():
            raise ProjectionDraftError("artifact symlinks are not accepted")
        artifact_root = (root / ".control-station" / "camera-calibrations").resolve()
        artifact_path = unresolved_artifact_path.resolve()
        if (
            root not in artifact_root.parents
            or artifact_path.parent != artifact_root
            or artifact_path.suffix != ".json"
            or not artifact_path.is_file()
            or artifact_path.stat().st_size > _MAX_ARTIFACT_BYTES
        ):
            raise ProjectionDraftError(
                "artifact is not a small regular file in the managed calibration directory"
            )
        with artifact_path.open("rb") as artifact_file:
            raw = artifact_file.read(_MAX_ARTIFACT_BYTES + 1)
    except (OSError, RuntimeError) as exc:
        raise ProjectionDraftError("artifact cannot be read safely") from exc
    if len(raw) > _MAX_ARTIFACT_BYTES:
        raise ProjectionDraftError("artifact exceeds 1 MB")
    actual_sha256 = hashlib.sha256(raw).hexdigest()
    if actual_sha256 != expected_sha256:
        raise ProjectionDraftError("artifact SHA-256 does not match the submitted value")
    try:
        draft = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_json_object)
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise ProjectionDraftError("artifact is not valid UTF-8 JSON") from exc
    if not isinstance(draft, dict):
        raise ProjectionDraftError("artifact must contain a JSON object")
    return draft, actual_sha256


def read_projection_draft_artifact(
    repository_root: Path, relative_path: object, expected_sha256: object
) -> dict[str, Any]:
    """Read a hash-verified projection JSON from the managed calibration directory."""
    draft, _ = _read_artifact(repository_root, relative_path, expected_sha256)
    return draft


def validate_projection_draft_artifact(
    task: dict[str, Any],
    repository_root: Path,
    relative_path: object,
    expected_sha256: object,
) -> dict[str, Any]:
    """Validate an offline 2D projection draft and return a compact review event payload."""
    task_id = task.get("id")
    if not isinstance(task_id, str) or not _TASK_ID.fullmatch(task_id):
        raise ProjectionDraftError("task ID is invalid")
    if task.get("status") != "PENDING":
        raise ProjectionDraftError("task must remain PENDING")
    estimate_step_id = _unique_pending_step_id(task, "ESTIMATE_TARGET_POSE")
    detect_step_id = _unique_pending_step_id(task, "DETECT_TARGETS")
    if estimate_step_id is None or detect_step_id is None:
        raise ProjectionDraftError("task must have unique pending detection and pose-estimation steps")

    draft, artifact_sha256 = _read_artifact(repository_root, relative_path, expected_sha256)
    if (
        isinstance(draft.get("schema_version"), bool)
        or not isinstance(draft.get("schema_version"), int)
        or draft.get("schema_version") != 2
        or draft.get("status") != "DRAFT_ONLY"
        or draft.get("draft_scope") != "CAMERA_PIXEL_TO_FIXED_SHELF_PLANE_2D"
        or draft.get("usable_for_robot_control") is not False
        or draft.get("usable_for_pick_target") is not False
        or draft.get("physical_contact_point_verified") is not False
        or draft.get("arm_command_sent") is not False
        or draft.get("projection_anchor") != "DETECTION_BBOX_BOTTOM_CENTER_PROXY"
        or draft.get("projection_anchor_is_proxy") is not True
        or draft.get("review_required") is not True
        or draft.get("task_id") != task_id
    ):
        raise ProjectionDraftError("artifact is not a review-only plane-projection draft for this task")
    draft_id = draft.get("id")
    if not isinstance(draft_id, str) or not re.fullmatch(
        r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}", draft_id
    ):
        raise ProjectionDraftError("draft ID is invalid")

    observation_event = _unique_event(task, draft.get("event_id"), "TARGET_OBSERVED")
    observation_payload = observation_event.get("payload")
    observation = (
        observation_payload.get("observation")
        if isinstance(observation_payload, dict)
        else None
    )
    if (
        not isinstance(task.get("pick_request"), dict)
        or not isinstance(task["pick_request"].get("category"), str)
        or not isinstance(observation_payload, dict)
        or observation_payload.get("event") != "TARGET_OBSERVED"
        or observation_payload.get("source") != "camera_observation"
        or observation_payload.get("task_state_changed") is not False
        or observation_payload.get("arm_command_sent") is not False
        or not isinstance(observation, dict)
        or observation.get("observation_schema_version") not in {2, 3}
        or observation.get("decision_scope") != "OBSERVATION_ONLY"
        or observation.get("arm_command_sent") is not False
        or observation.get("frame_freshness") != "FRESH"
        or observation.get("status") not in {"SINGLE_CANDIDATE", "AMBIGUOUS"}
        or not isinstance(observation.get("target_class"), str)
        or observation["target_class"].casefold()
        != task["pick_request"].get("category", "").casefold()
        or not isinstance(observation.get("camera_id"), str)
        or not _IDENTIFIER.fullmatch(observation["camera_id"])
        or isinstance(observation.get("frame_sequence"), bool)
        or not isinstance(observation.get("frame_sequence"), int)
        or observation.get("frame_sequence", 0) <= 0
        or not isinstance(observation.get("captured_at"), str)
        or not observation.get("captured_at", "").strip()
        or not isinstance(observation.get("pixel_transform_id"), str)
        or not _IDENTIFIER.fullmatch(observation["pixel_transform_id"])
        or any(
            isinstance(observation.get(dimension), bool)
            or not isinstance(observation.get(dimension), int)
            or observation.get(dimension) <= 0
            for dimension in ("frame_width", "frame_height")
        )
        or observation_payload.get("pick_request") != task.get("pick_request")
        or draft.get("pick_request") != task.get("pick_request")
        or observation_event.get("step_id") not in (None, detect_step_id)
        or draft.get("observation_step_id") != observation_event.get("step_id")
    ):
        raise ProjectionDraftError("artifact source does not match the task observation event")
    if any(
        draft.get(draft_field) != observation.get(observation_field)
        for draft_field, observation_field in (
            ("camera_id", "camera_id"),
            ("pixel_transform_id", "pixel_transform_id"),
            ("frame_sha256", "frame_sha256"),
            ("frame_sequence", "frame_sequence"),
            ("captured_at", "captured_at"),
            ("observation_schema_version", "observation_schema_version"),
        )
    ):
        raise ProjectionDraftError("artifact camera/frame provenance differs from the observation")
    frame_sha256 = draft.get("frame_sha256")
    if not isinstance(frame_sha256, str) or not _SHA256.fullmatch(frame_sha256):
        raise ProjectionDraftError("artifact frame SHA-256 is invalid")

    candidate_index = draft.get("candidate_index")
    candidates = observation.get("candidates")
    candidate_count = observation.get("candidate_count")
    if (
        isinstance(candidate_index, bool)
        or not isinstance(candidate_index, int)
        or candidate_index < 0
        or not isinstance(candidates, list)
        or isinstance(candidate_count, bool)
        or not isinstance(candidate_count, int)
        or candidate_count != len(candidates)
        or candidate_count < 1
        or (observation.get("status") == "SINGLE_CANDIDATE" and candidate_count != 1)
        or (observation.get("status") == "AMBIGUOUS" and candidate_count < 2)
        or candidate_index >= len(candidates)
        or not isinstance(candidates[candidate_index], dict)
        or draft.get("candidate") != candidates[candidate_index]
    ):
        raise ProjectionDraftError("artifact candidate snapshot does not match the observation")
    candidate = candidates[candidate_index]
    if (
        not isinstance(candidate.get("class_name"), str)
        or candidate.get("class_name", "").casefold()
        != task["pick_request"].get("category", "").casefold()
        or any(
            candidate.get(field) != observation.get(field)
            for field in (
                "camera_id",
                "frame_sequence",
                "captured_at",
                "frame_sha256",
                "pixel_transform_id",
                "frame_width",
                "frame_height",
            )
        )
    ):
        raise ProjectionDraftError("artifact candidate camera/frame provenance is inconsistent")

    selection_event_id = draft.get("operator_selection_event_id")
    selection_event: dict[str, Any] | None = None
    if selection_event_id is not None:
        selection_event = _unique_event(task, selection_event_id, "TARGET_SELECTED")
        selection_payload = selection_event.get("payload")
        select_step_id = _unique_pending_step_id(task, "SELECT_TARGET")
        if (
            select_step_id is None
            or selection_event.get("step_id") not in (None, select_step_id)
            or draft.get("selection_step_id") != selection_event.get("step_id")
            or not isinstance(selection_payload, dict)
            or selection_payload.get("event") != "TARGET_SELECTED"
            or selection_payload.get("source") != "operator_review"
            or selection_payload.get("selection_scope") != "HUMAN_REVIEW_ONLY"
            or selection_payload.get("observation_event_id") != observation_event.get("id")
            or selection_payload.get("pick_request") != task.get("pick_request")
            or selection_payload.get("task_state_changed") is not False
            or selection_payload.get("arm_command_sent") is not False
            or any(
                selection_payload.get(field) != observation.get(field)
                for field in (
                    "camera_id",
                    "frame_sequence",
                    "frame_sha256",
                    "pixel_transform_id",
                )
            )
            or isinstance(selection_payload.get("candidate_index"), bool)
            or not isinstance(selection_payload.get("candidate_index"), int)
            or selection_payload.get("candidate_index") != candidate_index
            or selection_payload.get("candidate") != candidates[candidate_index]
            or draft.get("candidate_selection_source") != "TARGET_SELECTED_EVENT"
            or draft.get("selection_is_review_only") is not True
        ):
            raise ProjectionDraftError("artifact does not match its operator selection event")
    elif (
        draft.get("candidate_selection_source") != "CLI_ARGUMENT"
        or draft.get("selection_is_review_only") is not False
        or draft.get("selection_step_id") is not None
        or draft.get("target_selection_complete") is not False
    ):
        raise ProjectionDraftError("unselected artifact has inconsistent selection metadata")

    plane_frame = draft.get("plane_frame")
    plane_xy = draft.get("plane_xy")
    if (
        not isinstance(plane_frame, dict)
        or not isinstance(plane_frame.get("frame_id"), str)
        or not _FRAME_ID.fullmatch(plane_frame["frame_id"])
        or not isinstance(plane_frame.get("unit"), str)
        or plane_frame.get("unit") not in {"m", "mm"}
        or not isinstance(plane_frame.get("axis_definition"), str)
        or not 1 <= len(plane_frame.get("axis_definition", "").strip()) <= 200
        or not isinstance(plane_xy, dict)
        or set(plane_xy) != {"x", "y"}
        or any(_finite_number(plane_xy.get(axis)) is None for axis in ("x", "y"))
    ):
        raise ProjectionDraftError("plane frame or 2D coordinates are invalid")
    if not all(
        isinstance(draft.get(key), str) and _IDENTIFIER.fullmatch(draft[key])
        for key in ("plane_calibration_id", "intrinsic_calibration_id")
    ):
        raise ProjectionDraftError("calibration references are missing")
    calibration_references = draft.get("calibration_artifacts")
    if not isinstance(calibration_references, dict) or set(calibration_references) != {
        "plane",
        "intrinsics",
    }:
        raise ProjectionDraftError("source calibration artifacts are not linked")
    plane_reference = calibration_references.get("plane")
    intrinsic_reference = calibration_references.get("intrinsics")
    if not isinstance(plane_reference, dict) or not isinstance(intrinsic_reference, dict):
        raise ProjectionDraftError("source calibration artifact references are invalid")
    plane_artifact, plane_artifact_sha256 = _read_artifact(
        repository_root,
        plane_reference.get("path"),
        plane_reference.get("sha256"),
    )
    intrinsic_artifact, intrinsic_artifact_sha256 = _read_artifact(
        repository_root,
        intrinsic_reference.get("path"),
        intrinsic_reference.get("sha256"),
    )
    expected_image_size = {
        "width": observation["frame_width"],
        "height": observation["frame_height"],
    }
    if (
        isinstance(plane_artifact.get("schema_version"), bool)
        or not isinstance(plane_artifact.get("schema_version"), int)
        or plane_artifact.get("schema_version") != 4
        or plane_artifact.get("status") != "DRAFT_ONLY"
        or plane_artifact.get("calibration_scope") != "CAMERA_TO_FIXED_PLANE_ONLY"
        or plane_artifact.get("usable_for_robot_control") is not False
        or plane_artifact.get("arm_command_sent") is not False
        or plane_artifact.get("review_required") is not True
        or plane_artifact.get("input_pixel_domain")
        != "DISTORTED_IMAGE_PIXELS_UNDISTORTED_WITH_REFERENCED_INTRINSICS"
        or plane_artifact.get("camera_id") != observation.get("camera_id")
        or plane_artifact.get("pixel_transform_id") != observation.get("pixel_transform_id")
        or plane_artifact.get("calibration_id") != draft.get("plane_calibration_id")
        or plane_artifact.get("intrinsic_calibration_id")
        != draft.get("intrinsic_calibration_id")
        or plane_artifact.get("plane_frame") != plane_frame
        or not _matches_image_size(
            plane_artifact.get("image_size_px"),
            expected_image_size["width"],
            expected_image_size["height"],
        )
        or isinstance(intrinsic_artifact.get("schema_version"), bool)
        or not isinstance(intrinsic_artifact.get("schema_version"), int)
        or intrinsic_artifact.get("schema_version") != 2
        or intrinsic_artifact.get("status") != "DRAFT_ONLY"
        or intrinsic_artifact.get("calibration_scope") != "CAMERA_INTRINSICS_ONLY"
        or intrinsic_artifact.get("usable_for_robot_control") is not False
        or intrinsic_artifact.get("arm_command_sent") is not False
        or intrinsic_artifact.get("review_required") is not True
        or intrinsic_artifact.get("camera_id") != observation.get("camera_id")
        or intrinsic_artifact.get("pixel_transform_id")
        != observation.get("pixel_transform_id")
        or intrinsic_artifact.get("calibration_id")
        != draft.get("intrinsic_calibration_id")
        or not _matches_image_size(
            intrinsic_artifact.get("image_size_px"),
            expected_image_size["width"],
            expected_image_size["height"],
        )
    ):
        raise ProjectionDraftError("source calibration artifacts do not match this projection")
    calibration_summary = draft.get("calibration_review_summary")
    plane_samples = plane_artifact.get("samples")
    intrinsic_sources = intrinsic_artifact.get("source_images")
    intrinsic_errors = intrinsic_artifact.get("per_image_reprojection_errors")
    if (
        not isinstance(calibration_summary, dict)
        or calibration_summary.get("review_required") is not True
        or not isinstance(plane_samples, list)
        or not all(isinstance(sample, dict) for sample in plane_samples)
        or not isinstance(intrinsic_sources, list)
        or not isinstance(intrinsic_errors, list)
        or len(intrinsic_sources) != len(intrinsic_errors)
        or not all(isinstance(source, str) and source.strip() for source in intrinsic_sources)
        or not all(isinstance(record, dict) for record in intrinsic_errors)
    ):
        raise ProjectionDraftError("calibration review summary or source records are missing")
    plane_source_images: dict[str, str] = {}
    for sample in plane_samples:
        source_image = sample.get("source_image")
        source_sha256 = sample.get("source_image_sha256")
        if not isinstance(source_image, str) or not isinstance(source_sha256, str):
            raise ProjectionDraftError("plane calibration source image records are invalid")
        previous_hash = plane_source_images.setdefault(source_image, source_sha256)
        if previous_hash != source_sha256:
            raise ProjectionDraftError("plane calibration source image hashes conflict")
    expected_intrinsic_errors = [
        {
            "file": record.get("file"),
            "sha256": record.get("sha256"),
            "reprojection_error_px": record.get("reprojection_error_px"),
        }
        for record in intrinsic_errors
    ]
    if (
        not _same_integer(
            calibration_summary.get("plane_fit_point_count"),
            plane_artifact.get("fit_point_count"),
        )
        or not _same_integer(
            calibration_summary.get("plane_validation_point_count"),
            plane_artifact.get("validation_point_count"),
        )
        or calibration_summary.get("plane_residual_unit") != plane_artifact.get("residual_unit")
        or not _same_finite_number(
            calibration_summary.get("plane_fit_rms_residual"),
            plane_artifact.get("fit_rms_residual"),
        )
        or not _same_finite_number(
            calibration_summary.get("plane_validation_rms_residual"),
            plane_artifact.get("validation_rms_residual"),
        )
        or calibration_summary.get("plane_source_images")
        != [
            {"source_image": name, "sha256": digest}
            for name, digest in plane_source_images.items()
        ]
        or not _same_integer(
            calibration_summary.get("intrinsic_calibrated_view_count"),
            intrinsic_artifact.get("calibrated_view_count"),
        )
        or not _same_finite_number(
            calibration_summary.get("intrinsic_rms_reprojection_error_px"),
            intrinsic_artifact.get("rms_reprojection_error_px"),
        )
        or calibration_summary.get("intrinsic_checkerboard")
        != intrinsic_artifact.get("checkerboard")
        or not _same_integer(
            calibration_summary.get("intrinsic_reprojection_record_count"),
            len(intrinsic_errors),
        )
        or calibration_summary.get("intrinsic_source_images") != intrinsic_sources
        or calibration_summary.get("intrinsic_per_image_reprojection_errors")
        != expected_intrinsic_errors
    ):
        raise ProjectionDraftError("calibration review metrics differ from source artifacts")
    expected_constraints = {
        key: task["pick_request"][key]
        for key in ("color", "shelf_level")
        if key in task["pick_request"]
    }
    if draft.get("unapplied_request_constraints") != expected_constraints:
        raise ProjectionDraftError("artifact unapplied constraints differ from the task request")
    if draft.get("target_selection_complete") is not (
        selection_event is not None and not expected_constraints
    ):
        raise ProjectionDraftError("target-selection completion flag is inconsistent")

    summary = {
        "source": "projection_draft_review",
        "event": "TARGET_PLANE_PROJECTION_RECORDED",
        "draft_id": draft_id,
        "artifact_path": str(relative_path),
        "artifact_sha256": artifact_sha256,
        "source_observation_event_id": observation_event.get("id"),
        "observation_step_id": observation_event.get("step_id"),
        "operator_selection_event_id": (
            selection_event.get("id") if selection_event is not None else None
        ),
        "selection_step_id": selection_event.get("step_id") if selection_event else None,
        "candidate_index": candidate_index,
        "camera_id": draft["camera_id"],
        "frame_sha256": draft["frame_sha256"],
        "pixel_transform_id": draft["pixel_transform_id"],
        "projection_anchor": draft.get("projection_anchor"),
        "plane_calibration_id": draft["plane_calibration_id"],
        "intrinsic_calibration_id": draft["intrinsic_calibration_id"],
        "calibration_artifacts": {
            "plane": {
                "path": plane_reference["path"],
                "sha256": plane_artifact_sha256,
            },
            "intrinsics": {
                "path": intrinsic_reference["path"],
                "sha256": intrinsic_artifact_sha256,
            },
        },
        "plane_frame": {
            key: plane_frame[key] for key in ("frame_id", "unit", "axis_definition")
        },
        "plane_xy": {key: float(plane_xy[key]) for key in ("x", "y")},
        "review_required": True,
        "target_selection_complete": draft["target_selection_complete"],
        "usable_for_robot_control": False,
        "arm_command_sent": False,
        "task_state_changed": False,
    }
    return summary
