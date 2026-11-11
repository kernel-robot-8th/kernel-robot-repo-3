"""카메라 대상 관측 판정 (관측 전용, 로봇 명령 없음)"""

from __future__ import annotations

import os
import re
from typing import Any

from control_station.api.common import SHA256
from control_station.command_parser import canonicalize_color
from control_station.models import CameraSnapshot
from control_station.shelf_regions import evaluate_shelf_level

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
        "frame_width": camera.frame_width,
        "frame_height": camera.frame_height,
        "frame_sha256": camera.frame_sha256,
        "pixel_transform_id": camera.pixel_transform_id,
        "candidate_count": candidate_count,
        "candidates": candidates,
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
