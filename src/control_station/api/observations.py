"""대상 관측·선택, 투영 초안, 관측 이미지 (검토 기록만, 로봇 명령 없음)"""

from __future__ import annotations

import hashlib
import os
import re
from dataclasses import replace
from typing import TYPE_CHECKING, Any

from control_station.api.common import (
    EVENT_ID,
    REPOSITORY_ROOT,
    SHA256,
    TASK_ID,
    pending_pick_step_id,
    read_observation_png,
    validate_stored_pick_request,
    verified_observation_frame,
)
from control_station.camera_adapters import YOLO_WORLD_REQUEST_PROMPT_POLICY_VERSION
from control_station.command_parser import canonicalize_color
from control_station.models import CameraSnapshot, TaskStatus
from control_station.projection_drafts import (
    ProjectionDraftError,
    read_projection_draft_artifact,
    validate_projection_draft_artifact,
)
from control_station.shelf_regions import evaluate_shelf_level
from control_station.store import StateTransitionConflict, StepNotFoundError, TaskNotFoundError

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler

COLOR_EVALUATION_ALGORITHM_VERSION = "exact-estimated-hint-match-v1"
_NAME = re.compile(r"[A-Za-z0-9_.-]{1,80}")


def evaluate_color_hints(candidates: list[dict[str, object]], requested_color: str) -> dict[str, object]:
    """후보 색 힌트와 요청 색 비교"""
    canonical_color = canonicalize_color(requested_color)
    if canonical_color is None:
        status = "UNSUPPORTED_REQUEST_COLOR"
        candidate_indices: list[int] = []
    elif not candidates:
        status = "NO_CANDIDATES"
        candidate_indices = []
    else:
        estimated = [
            (index, candidate.get("color_hint"))
            for index, candidate in enumerate(candidates)
            if isinstance(candidate.get("color_hint"), dict)
            and candidate["color_hint"].get("status") == "ESTIMATED"
        ]
        if not estimated:
            status = "NO_ESTIMATED_COLOR_HINTS"
            candidate_indices = []
        else:
            candidate_indices = [
                index
                for index, hint in estimated
                if isinstance(hint, dict) and hint.get("color") == canonical_color
            ]
            status = {
                0: "NO_COLOR_HINT_MATCH",
                1: "SINGLE_COLOR_HINT_MATCH",
            }.get(len(candidate_indices), "MULTIPLE_COLOR_HINT_MATCHES")
    return {
        "algorithm_version": COLOR_EVALUATION_ALGORITHM_VERSION,
        "requested_color": requested_color,
        "canonical_color": canonical_color,
        "status": status,
        "candidate_indices": candidate_indices,
        "review_required": True,
        "used_for_selection": False,
    }


def target_observation(
    camera: CameraSnapshot,
    target_class: str,
    requested_color: str | None = None,
    requested_shelf_level: int | None = None,
) -> dict[str, Any]:
    """카메라 관측을 대상 후보로 분류"""
    detector_state = camera.detector_state
    candidates: list[dict[str, object]] = []
    candidate_count: int | None = None
    if detector_state is None:
        status = "DETECTOR_NOT_CONFIGURED"
    elif detector_state == "error":
        status = "DETECTOR_ERROR"
    elif detector_state == "loading":
        status = "DETECTOR_LOADING"
    elif detector_state != "ready":
        status = "DETECTOR_NOT_READY"
    elif camera.connection_state.value != "CONNECTED":
        status = "CAMERA_DISCONNECTED"
    elif camera.stream_state.value != "STREAMING":
        status = "CAMERA_STREAM_UNAVAILABLE"
    elif camera.frame_freshness.value == "STALE":
        status = "FRAME_STALE"
    elif camera.frame_freshness.value != "FRESH":
        status = "FRAME_UNAVAILABLE"
    elif not isinstance(camera.pixel_transform_id, str) or not _NAME.fullmatch(camera.pixel_transform_id):
        status = "PIXEL_TRANSFORM_UNKNOWN"
    elif not isinstance(camera.frame_sha256, str) or not SHA256.fullmatch(camera.frame_sha256):
        status = "FRAME_PROVENANCE_UNKNOWN"
    elif (
        camera.position == "FIXED"
        and camera.detector_inference_frame_sequence != camera.frame_sequence
    ):
        status = "DETECTOR_NOT_RUN"
    elif camera.detector_classes is not None and not any(
        name.casefold() == target_class.casefold() for name in camera.detector_classes
    ):
        status = "CLASS_UNKNOWN"
    else:
        candidates = [
            detection
            for detection in camera.detections
            if str(detection.get("class_name", "")).casefold() == target_class.casefold()
            and detection.get("camera_id") == camera.id
            and detection.get("frame_sequence") == camera.frame_sequence
            and detection.get("captured_at") == camera.last_frame_at
            and detection.get("frame_sha256") == camera.frame_sha256
            and detection.get("pixel_transform_id") == camera.pixel_transform_id
        ]
        candidate_count = len(candidates)
        if candidate_count == 0:
            status = "TARGET_NOT_FOUND"
        elif candidate_count == 1:
            status = "SINGLE_CANDIDATE"
        else:
            status = "AMBIGUOUS"
    observation = {
        "observation_schema_version": 3,
        "camera_id": camera.id,
        "target_class": target_class,
        "status": status,
        "detector_state": detector_state,
        "detector_inference_frame_sequence": camera.detector_inference_frame_sequence,
        "camera_connection_state": camera.connection_state.value,
        "camera_stream_state": camera.stream_state.value,
        "available_classes": camera.detector_classes,
        "detector_provenance": {
            "model_type": camera.detector_model_type,
            "prompts": camera.detector_prompts,
            "weights_sha256": camera.detector_weights_sha256,
            "confidence_threshold": camera.detector_confidence_threshold,
            "inference_image_size": camera.detector_inference_image_size,
            "device": camera.detector_device,
            "ultralytics_version": camera.detector_ultralytics_version,
        },
        "frame_freshness": camera.frame_freshness.value,
        "frame_sequence": camera.frame_sequence,
        "captured_at": camera.last_frame_at,
        "source_stamp": camera.source_stamp,
        "source_frame_id": camera.source_frame_id,
        "source_stamp_clock_status": (
            "ROS_SOURCE_HEADER_NOT_ALIGNED_WITH_HOST_CLOCK"
            if camera.source_stamp is not None
            else "NOT_PROVIDED"
        ),
        "frame_width": camera.frame_width,
        "frame_height": camera.frame_height,
        "frame_sha256": camera.frame_sha256,
        "pixel_transform_id": camera.pixel_transform_id,
        "candidate_count": candidate_count,
        "candidates": candidates,
        "depth_region_observation": {
            "state": "NOT_REQUESTED",
            "review_required": True,
            "used_for_selection": False,
            "target_pose_use": False,
        },
        "decision_scope": "OBSERVATION_ONLY",
        "arm_command_sent": False,
    }
    if requested_color is not None:
        observation["color_evaluation"] = evaluate_color_hints(candidates, requested_color)
    if requested_shelf_level is not None:
        calibration_dir = os.environ.get(
            "CONTROL_STATION_SHELF_REGION_DIR",
            ".control-station/shelf-region-calibrations",
        ).strip() or ".control-station/shelf-region-calibrations"
        observation["shelf_level_evaluation"] = evaluate_shelf_level(
            camera_id=camera.id,
            frame_width=camera.frame_width,
            frame_height=camera.frame_height,
            pixel_transform_id=camera.pixel_transform_id,
            candidates=candidates,
            requested_level=requested_shelf_level,
            calibration_dir=calibration_dir,
        )
    return observation


def get_observation_image(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """저장된 관측 이미지"""
    if len(parts) != 6 or parts[:2] != ["api", "tasks"] or parts[3] != "observations" or parts[5] != "image":
        h._json(404, {"error": "Observation image not found"})
        return
    task_id, event_id = parts[2], parts[4]
    if not TASK_ID.fullmatch(task_id) or not EVENT_ID.fullmatch(event_id):
        h._json(404, {"error": "Observation image not found"})
        return
    task = h.app.task_store.get_task(task_id)
    if task is None:
        h._json(404, {"error": "Observation image not found"})
        return
    matching_events = [event for event in task["events"] if event.get("id") == event_id]
    if len(matching_events) != 1:
        h._json(404, {"error": "Observation image not found"})
        return
    event = matching_events[0]
    payload = event.get("payload")
    artifact = payload.get("frame_artifact") if isinstance(payload, dict) else None
    observation = payload.get("observation") if isinstance(payload, dict) else None
    if (
        event.get("event_type") != "TARGET_OBSERVED"
        or not isinstance(artifact, dict)
        or artifact.get("media_type") != "image/png"
        or not isinstance(observation, dict)
        or artifact.get("source_frame_sha256") != observation.get("frame_sha256")
    ):
        h._json(404, {"error": "Observation image not found"})
        return
    relative_path = artifact.get("path")
    expected_hash = artifact.get("file_sha256")
    if (
        not isinstance(relative_path, str)
        or not isinstance(expected_hash, str)
        or not SHA256.fullmatch(expected_hash)
        or not verified_observation_frame(
            task_id,
            artifact,
            observation.get("frame_sha256"),
            observation.get("frame_width"),
            observation.get("frame_height"),
        )
    ):
        h._json(404, {"error": "Observation image not found"})
        return
    image_bytes = read_observation_png(task_id, relative_path.split("/")[-1])
    if image_bytes is None:
        h._json(404, {"error": "Observation image not found"})
        return
    if hashlib.sha256(image_bytes).hexdigest() != expected_hash:  # changed since it was verified above
        h._json(409, {"error": "Observation image integrity check failed"})
        return
    h._image(image_bytes)


def get_projection_artifact(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """투영 초안 파일"""
    if (
        len(parts) != 6
        or parts[:2] != ["api", "tasks"]
        or parts[3] != "projection-drafts"
        or parts[5] != "artifact"
        or not TASK_ID.fullmatch(parts[2])
        or not EVENT_ID.fullmatch(parts[4])
    ):
        h._json(404, {"error": "Projection draft artifact not found"})
        return
    task = h.app.task_store.get_task(parts[2])
    if task is None:
        h._json(404, {"error": "Projection draft artifact not found"})
        return
    matches = [
        event
        for event in task.get("events", [])
        if isinstance(event, dict) and event.get("id") == parts[4]
    ]
    if len(matches) != 1 or matches[0].get("event_type") != "TARGET_PLANE_PROJECTION_RECORDED":
        h._json(404, {"error": "Projection draft artifact not found"})
        return
    event_payload = matches[0].get("payload")
    if (
        not isinstance(event_payload, dict)
        or event_payload.get("source") != "projection_draft_review"
        or event_payload.get("event") != "TARGET_PLANE_PROJECTION_RECORDED"
        or event_payload.get("task_state_changed") is not False
        or event_payload.get("arm_command_sent") is not False
        or event_payload.get("usable_for_robot_control") is not False
    ):
        h._json(409, {"error": "Projection review event payload is invalid"})
        return
    try:
        artifact = read_projection_draft_artifact(
            REPOSITORY_ROOT,
            event_payload.get("artifact_path"),
            event_payload.get("artifact_sha256"),
        )
    except ProjectionDraftError as exc:
        h._json(409, {"error": str(exc)})
        return
    if (
        artifact.get("id") != event_payload.get("draft_id")
        or artifact.get("task_id") != parts[2]
        or artifact.get("status") != "DRAFT_ONLY"
    ):
        h._json(409, {"error": "Projection artifact no longer matches its review event"})
        return
    h._json(200, {"artifact": artifact, "review_event_id": parts[4]})


def post_projection_draft(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """투영 초안 검토 기록"""
    task_id = parts[2]
    if set(body) != {"artifact_path", "artifact_sha256"}:
        h._json_error(
            400,
            "INVALID_PROJECTION_DRAFT_REQUEST",
            "artifact_path and artifact_sha256 are required",
        )
        return
    task = h.app.task_store.get_task(task_id)
    if task is None:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    try:
        validate_stored_pick_request(task.get("pick_request"))
    except ValueError as exc:
        h._json_error(409, "PICK_REQUEST_INVALID", str(exc))
        return
    try:
        projection_step_id = pending_pick_step_id(task, "ESTIMATE_TARGET_POSE")
        if projection_step_id is None:
            h._json_error(
                409,
                "POSE_ESTIMATION_STEP_UNAVAILABLE",
                "Task has no unique pending ESTIMATE_TARGET_POSE step",
            )
            return
        projection_payload = validate_projection_draft_artifact(
            task,
            REPOSITORY_ROOT,
            body.get("artifact_path"),
            body.get("artifact_sha256"),
        )
        _, event_id, already_recorded = h.app.task_store.record_projection_draft_event(
            task_id,
            projection_payload,
            expected_status=TaskStatus.PENDING,
            step_id=projection_step_id,
        )
    except ProjectionDraftError as exc:
        h._json_error(409, "PROJECTION_DRAFT_REJECTED", str(exc))
        return
    except TaskNotFoundError:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    except StepNotFoundError:
        h._json_error(
            409,
            "POSE_ESTIMATION_STEP_UNAVAILABLE",
            "Pose-estimation workflow step is unavailable",
        )
        return
    except StateTransitionConflict as exc:
        h._json_error(409, "TASK_STATE_CONFLICT", str(exc))
        return
    h._json(
        200 if already_recorded else 201,
        {
            "event_id": event_id,
            "projection_draft": projection_payload,
            "execution_started": False,
            "already_recorded": already_recorded,
        },
    )


def _selectable_observation(
    source_event: dict[str, Any],
    task_id: str,
    detection_step_id: str,
    pick_request: dict[str, object],
) -> bool:
    """선택할 수 있는 관측 이벤트인지"""
    source_payload = source_event.get("payload")
    observation = source_payload.get("observation") if isinstance(source_payload, dict) else None
    return not (
        source_event.get("event_type") != "TARGET_OBSERVED"
        or source_event.get("task_id") != task_id
        or source_event.get("step_id") not in (None, detection_step_id)
        or source_event.get("previous_status") != TaskStatus.PENDING.value
        or source_event.get("new_status") != TaskStatus.PENDING.value
        or not isinstance(source_payload, dict)
        or source_payload.get("event") != "TARGET_OBSERVED"
        or source_payload.get("source") != "camera_observation"
        or source_payload.get("pick_request") != pick_request
        or source_payload.get("task_state_changed") is not False
        or source_payload.get("arm_command_sent") is not False
        or not isinstance(observation, dict)
        or isinstance(observation.get("observation_schema_version"), bool)
        or not isinstance(observation.get("observation_schema_version"), int)
        or observation.get("observation_schema_version") not in {2, 3}
        or observation.get("decision_scope") != "OBSERVATION_ONLY"
        or observation.get("arm_command_sent") is not False
        or observation.get("frame_freshness") != "FRESH"
        or observation.get("status") not in {"SINGLE_CANDIDATE", "AMBIGUOUS"}
        or not isinstance(pick_request, dict)
        or not isinstance(pick_request.get("category"), str)
        or not isinstance(observation.get("target_class"), str)
        or observation.get("target_class", "").casefold()
        != pick_request.get("category", "").casefold()
        or not isinstance(observation.get("camera_id"), str)
        or not _NAME.fullmatch(observation.get("camera_id", ""))
        or isinstance(observation.get("frame_sequence"), bool)
        or not isinstance(observation.get("frame_sequence"), int)
        or observation.get("frame_sequence") <= 0
        or not isinstance(observation.get("captured_at"), str)
        or not observation.get("captured_at", "").strip()
        or not isinstance(observation.get("frame_sha256"), str)
        or not SHA256.fullmatch(observation.get("frame_sha256", ""))
        or not isinstance(observation.get("pixel_transform_id"), str)
        or not _NAME.fullmatch(observation.get("pixel_transform_id", ""))
        or any(
            isinstance(observation.get(field), bool)
            or not isinstance(observation.get(field), int)
            or observation.get(field) <= 0
            for field in ("frame_width", "frame_height")
        )
    )


def post_target_selection(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """운영자 대상 선택 기록"""
    task_id = parts[2]
    observation_event_id = body.get("observation_event_id")
    candidate_index = body.get("candidate_index")
    if (
        set(body) != {"observation_event_id", "candidate_index"}
        or not isinstance(observation_event_id, str)
        or not EVENT_ID.fullmatch(observation_event_id)
        or isinstance(candidate_index, bool)
        or not isinstance(candidate_index, int)
        or candidate_index < 0
    ):
        h._json_error(
            400,
            "INVALID_TARGET_SELECTION_REQUEST",
            "observation_event_id and a non-negative candidate_index are required",
        )
        return
    task = h.app.task_store.get_task(task_id)
    if task is None:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    if task.get("pick_request") is None:
        h._json_error(409, "PICK_REQUEST_MISSING", "Task has no structured pick request")
        return
    try:
        pick_request = validate_stored_pick_request(task.get("pick_request"))
    except ValueError as exc:
        h._json_error(409, "PICK_REQUEST_INVALID", str(exc))
        return
    if task["status"] != TaskStatus.PENDING.value:
        h._json_error(409, "TASK_NOT_PENDING", "A candidate can be recorded only while the task is pending")
        return
    selection_step_id = pending_pick_step_id(task, "SELECT_TARGET")
    if selection_step_id is None:
        h._json_error(409, "SELECTION_STEP_UNAVAILABLE", "Task has no unique pending SELECT_TARGET step")
        return
    detection_step_id = pending_pick_step_id(task, "DETECT_TARGETS")
    if detection_step_id is None:
        h._json_error(409, "DETECTION_STEP_UNAVAILABLE", "Task has no unique pending DETECT_TARGETS step")
        return
    matches = [event for event in task["events"] if event.get("id") == observation_event_id]
    if len(matches) != 1:
        h._json_error(404, "OBSERVATION_EVENT_NOT_FOUND", "Target observation event not found")
        return
    source_event = matches[0]
    if not _selectable_observation(source_event, task_id, detection_step_id, pick_request):
        h._json_error(
            409,
            "OBSERVATION_NOT_SELECTABLE",
            "Source event is not a selectable fresh observation for this task",
        )
        return
    source_payload = source_event["payload"]
    observation = source_payload["observation"]
    candidates = observation.get("candidates")
    candidate_count = observation.get("candidate_count")
    if (
        not isinstance(candidates, list)
        or isinstance(candidate_count, bool)
        or not isinstance(candidate_count, int)
        or candidate_count != len(candidates)
        or candidate_count < 1
        or (observation.get("status") == "SINGLE_CANDIDATE" and candidate_count != 1)
        or (observation.get("status") == "AMBIGUOUS" and candidate_count < 2)
        or candidate_index >= len(candidates)
    ):
        h._json_error(409, "CANDIDATE_INDEX_OUT_OF_RANGE", "Candidate index is outside the recorded observation")
        return
    candidate = candidates[candidate_index]
    if (
        not isinstance(candidate, dict)
        or not isinstance(candidate.get("class_name"), str)
        or candidate.get("class_name", "").casefold() != pick_request.get("category", "").casefold()
        or candidate.get("camera_id") != observation.get("camera_id")
        or candidate.get("frame_sequence") != observation.get("frame_sequence")
        or candidate.get("captured_at") != observation.get("captured_at")
        or candidate.get("frame_sha256") != observation.get("frame_sha256")
        or candidate.get("pixel_transform_id") != observation.get("pixel_transform_id")
        or candidate.get("frame_width") != observation.get("frame_width")
        or candidate.get("frame_height") != observation.get("frame_height")
    ):
        h._json_error(409, "CANDIDATE_PROVENANCE_MISMATCH", "Candidate provenance does not match its observation")
        return
    selection_payload: dict[str, object] = {
        "event": "TARGET_SELECTED",
        "source": "operator_review",
        "selection_scope": "HUMAN_REVIEW_ONLY",
        "observation_event_id": observation_event_id,
        "pick_request": pick_request,
        "detector_provenance": observation.get("detector_provenance"),
        "camera_id": observation.get("camera_id"),
        "frame_sequence": observation.get("frame_sequence"),
        "frame_sha256": observation.get("frame_sha256"),
        "pixel_transform_id": observation.get("pixel_transform_id"),
        "candidate_index": candidate_index,
        "candidate": candidate,
        "color_evaluation": observation.get("color_evaluation"),
        "shelf_level_evaluation": observation.get("shelf_level_evaluation"),
        "task_state_changed": False,
        "arm_command_sent": False,
    }
    latest_task = h.app.task_store.get_task(task_id)
    if latest_task is None:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    if latest_task["status"] != TaskStatus.PENDING.value:
        h._json_error(409, "TASK_NOT_PENDING", "A candidate can be recorded only while the task is pending")
        return
    existing_selection = next(
        (
            event
            for event in latest_task.get("events", [])
            if isinstance(event, dict)
            and event.get("event_type") == "TARGET_SELECTED"
            and event.get("step_id") == selection_step_id
            and event.get("payload") == selection_payload
        ),
        None,
    )
    if existing_selection is not None:
        existing_event_id = existing_selection.get("id")
        if not isinstance(existing_event_id, str):
            h._json_error(500, "SELECTION_EVENT_NOT_RETRIEVABLE", "The existing selection event has no readable ID")
            return
        h._json(
            200,
            {
                "task": latest_task,
                "selection": selection_payload,
                "selection_event_id": existing_event_id,
                "already_recorded": True,
            },
        )
        return
    if not verified_observation_frame(
        task_id,
        source_payload.get("frame_artifact"),
        observation.get("frame_sha256"),
        observation.get("frame_width"),
        observation.get("frame_height"),
    ):
        h._json_error(
            409,
            "SOURCE_FRAME_REQUIRED",
            "A saved, hash-verified source frame is required for operator selection",
        )
        return
    try:
        recorded, selection_event_id, already_recorded = h.app.task_store.record_target_selection_event(
            task_id,
            selection_payload,
            expected_status=TaskStatus.PENDING,
            step_id=selection_step_id,
        )
    except TaskNotFoundError:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    except StepNotFoundError:
        h._json_error(409, "SELECTION_STEP_UNAVAILABLE", "Target-selection workflow step is unavailable")
        return
    except StateTransitionConflict as exc:
        h._json_error(409, "TASK_STATE_CONFLICT", str(exc))
        return
    if not isinstance(selection_event_id, str):
        h._json_error(
            500,
            "SELECTION_EVENT_NOT_RETRIEVABLE",
            "Selection was recorded but its event ID could not be read back",
        )
        return
    h._json(
        200 if already_recorded else 201,
        {
            "task": recorded,
            "selection": selection_payload,
            "selection_event_id": selection_event_id,
            "already_recorded": already_recorded,
        },
    )


def _observe_with_detectors(
    h: ControlStationHandler, camera: CameraSnapshot, target_class: str
) -> tuple[CameraSnapshot, bool, str | None, bool, str | None]:
    """고정 카메라 추론, YOLO-World 요청 프롬프트 추론"""
    request_prompt_attempted = False
    request_prompt_error: str | None = None
    fixed_detector_attempted = False
    fixed_detector_error: str | None = None
    if (
        camera.position == "FIXED"
        and camera.detector_state == "ready"
        and camera.connection_state.value == "CONNECTED"
        and camera.stream_state.value == "STREAMING"
        and camera.frame_freshness.value == "FRESH"
        and isinstance(camera.frame_sequence, int)
        and not isinstance(camera.frame_sequence, bool)
        and camera.frame_sequence > 0
        and isinstance(camera.frame_sha256, str)
    ):
        detect_observation = getattr(h.app.camera_manager, "detect_for_observation", None)
        if callable(detect_observation):
            fixed_detector_attempted = True
            try:
                camera = detect_observation(camera)
            except Exception as exc:
                fixed_detector_error = f"{type(exc).__name__}: {exc}"[:512]
    if (
        camera.detector_state == "ready"
        and camera.detector_model_type == "YOLO_WORLD_OPEN_VOCAB"
        and camera.connection_state.value == "CONNECTED"
        and camera.stream_state.value == "STREAMING"
        and camera.frame_freshness.value == "FRESH"
        and camera.detector_classes is not None
        and not any(name.casefold() == target_class.casefold() for name in camera.detector_classes)
    ):
        request_prompt_attempted = True
        observe_prompt = getattr(h.app.camera_manager, "detect_for_prompt", None)
        if callable(observe_prompt):
            try:
                camera = observe_prompt(camera, target_class)
            except Exception as exc:
                request_prompt_error = f"{type(exc).__name__}: {exc}"[:512]
                try:
                    latest_camera = next(
                        (item for item in h.app.camera_manager.snapshots() if item.id == camera.id),
                        None,
                    )
                except Exception:
                    latest_camera = None
                if latest_camera is not None:
                    camera = replace(
                        camera,
                        detector_state=latest_camera.detector_state,
                        error_message=latest_camera.error_message,
                    )
        else:
            request_prompt_error = "camera manager does not support request-prompt inference"
    return camera, fixed_detector_attempted, fixed_detector_error, request_prompt_attempted, request_prompt_error


def post_target_observation(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """대상 관측 기록 (선택: 프레임 저장)"""
    task_id = parts[2]
    camera_id = body.get("camera_id")
    save_frame = body.get("save_frame", False)
    if set(body) - {"camera_id", "save_frame"}:
        h._json_error(400, "INVALID_TARGET_OBSERVATION_REQUEST", "Only camera_id and optional save_frame are accepted")
        return
    if not task_id or not isinstance(camera_id, str) or not camera_id.strip():
        h._json_error(400, "INVALID_TARGET_OBSERVATION_REQUEST", "task_id and camera_id are required")
        return
    if not isinstance(save_frame, bool):
        h._json_error(400, "INVALID_TARGET_OBSERVATION_REQUEST", "save_frame must be a boolean")
        return
    task = h.app.task_store.get_task(task_id)
    if task is None:
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    if task.get("pick_request") is None:
        h._json_error(409, "PICK_REQUEST_MISSING", "Task has no structured pick request")
        return
    try:
        pick_request = validate_stored_pick_request(task.get("pick_request"))
    except ValueError as exc:
        h._json_error(409, "PICK_REQUEST_INVALID", str(exc))
        return
    if task["status"] != TaskStatus.PENDING.value:
        h._json_error(409, "TASK_NOT_PENDING", "Target observation can be recorded only while the task is pending")
        return
    cameras = h.app.camera_manager.snapshots()
    camera = next((item for item in cameras if item.id == camera_id.strip()), None)
    if camera is None:
        h._json_error(404, "CAMERA_NOT_FOUND", "Camera not found")
        return
    observation_step_type = {"HEAD": "DETECT_TARGETS", "FIXED": "DETECT_TARGETS", "JOINT": "REFINE_TARGET"}.get(
        camera.position
    )
    if observation_step_type is None:
        h._json_error(409, "CAMERA_ROLE_UNSUPPORTED", "Camera position has no Pick & Place observation step")
        return
    observation_step_id = pending_pick_step_id(task, observation_step_type)
    if observation_step_id is None:
        h._json_error(409, "OBSERVATION_STEP_UNAVAILABLE", f"Task has no unique pending {observation_step_type} step")
        return
    frame_artifact: dict[str, object] | None = None
    if save_frame:
        save_observation = getattr(h.app.camera_manager, "save_observation_frame", None)
        if not callable(save_observation):
            h._json_error(409, "FRAME_SAVE_UNAVAILABLE", "Camera manager cannot save observation frames")
            return
        frame_sha256 = camera.frame_sha256
        frame_sequence = camera.frame_sequence
        if not isinstance(frame_sha256, str) or not isinstance(frame_sequence, int):
            h._json_error(409, "FRAME_PROVENANCE_UNAVAILABLE", "The current camera frame has no saveable provenance")
            return
        try:
            frame_artifact = save_observation(camera.id, frame_sequence, frame_sha256, task_id)
        except (RuntimeError, ValueError, OSError) as exc:
            h._json_error(409, "FRAME_SAVE_FAILED", f"Could not save the requested camera frame: {exc}")
            return
    target_class = str(pick_request["category"])
    camera, fixed_attempted, fixed_error, prompt_attempted, prompt_error = _observe_with_detectors(
        h, camera, target_class
    )
    requested_color = pick_request.get("color")
    requested_shelf_level = pick_request.get("shelf_level")
    observation = target_observation(
        camera,
        target_class,
        requested_color if isinstance(requested_color, str) else None,
        requested_shelf_level
        if isinstance(requested_shelf_level, int)
        and not isinstance(requested_shelf_level, bool)
        and requested_shelf_level > 0
        else None,
    )
    if prompt_attempted:
        detector_provenance = observation.get("detector_provenance")
        if isinstance(detector_provenance, dict):
            detector_provenance["prompt_scope"] = "REQUEST_CLASS_ONE_SHOT"
            detector_provenance["requested_prompt"] = target_class
            detector_provenance["prompt_policy_version"] = YOLO_WORLD_REQUEST_PROMPT_POLICY_VERSION
            detector_provenance["configured_prompts"] = camera.detector_prompts
            if prompt_error is not None:
                detector_provenance["prompt_inference_error"] = prompt_error
        if prompt_error is not None:
            observation["status"] = "REQUEST_PROMPT_INFERENCE_ERROR"
    if fixed_attempted:
        detector_provenance = observation.get("detector_provenance")
        if isinstance(detector_provenance, dict):
            detector_provenance["inference_scope"] = "FIXED_CAMERA_TASK_OBSERVATION"
            if fixed_error is not None:
                detector_provenance["inference_error"] = fixed_error
        if fixed_error is not None:
            observation["status"] = "DETECTOR_INFERENCE_ERROR"
    if camera.position == "FIXED":
        summarize_depth = getattr(h.app.camera_manager, "summarize_detection_depth", None)
        if callable(summarize_depth):
            try:
                observation["depth_region_observation"] = summarize_depth(camera, observation["candidates"])
            except Exception as exc:
                observation["depth_region_observation"] = {
                    "state": "SUMMARY_ERROR",
                    "error": f"{type(exc).__name__}: {exc}"[:256],
                    "review_required": True,
                    "used_for_selection": False,
                    "target_pose_use": False,
                }
        else:
            observation["depth_region_observation"] = {
                "state": "UNAVAILABLE",
                "review_required": True,
                "used_for_selection": False,
                "target_pose_use": False,
            }
    event_payload: dict[str, object] = {
        "event": "TARGET_OBSERVED",
        "pick_request": pick_request,
        "observation": observation,
        "frame_saved": frame_artifact is not None,
        "frame_artifact": frame_artifact,
        "task_state_changed": False,
        "arm_command_sent": False,
    }

    def discard_saved_frame() -> None:
        if frame_artifact is None:
            return
        delete_observation = getattr(h.app.camera_manager, "delete_observation_frame", None)
        if callable(delete_observation):
            try:
                delete_observation(str(frame_artifact["path"]))
            except (OSError, ValueError):
                pass

    try:
        recorded = h.app.task_store.record_task_event(
            task_id,
            "TARGET_OBSERVED",
            event_payload,
            expected_status=TaskStatus.PENDING,
            step_id=observation_step_id,
        )
    except TaskNotFoundError:
        discard_saved_frame()
        h._json_error(404, "TASK_NOT_FOUND", "Task not found")
        return
    except StepNotFoundError:
        discard_saved_frame()
        h._json_error(409, "OBSERVATION_STEP_UNAVAILABLE", "Camera observation workflow step is unavailable")
        return
    except StateTransitionConflict as exc:
        discard_saved_frame()
        h._json_error(409, "TASK_STATE_CONFLICT", str(exc))
        return
    except ValueError as exc:
        discard_saved_frame()
        h._json_error(409, "OBSERVATION_RECORD_REJECTED", str(exc))
        return
    except Exception:
        discard_saved_frame()
        raise
    h._json(201, {"task": recorded, "observation": observation, "frame_artifact": frame_artifact})
