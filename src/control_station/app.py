"""HTTP API and small status page for the robot control station."""

from __future__ import annotations

import hashlib
import json
import os
import re
import time
from dataclasses import replace
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlsplit

from control_station.arm_supervisor import ArmTeleopSupervisor
from control_station.camera_adapters import (
    MAX_OBSERVATION_IMAGE_BYTES,
    OpenCVArmCameraStreamManager,
    Ros2SimulationCameraStreamManager,
    UnconfiguredArmCameraStreamManager,
    YOLO_WORLD_REQUEST_PROMPT_POLICY_VERSION,
)
from control_station.command_parser import (
    MAX_SHELF_LEVEL,
    canonicalize_color,
    parse_pick_command,
)
from control_station.coordinate_target_drafts import CoordinateDraftError, CoordinateTargetDraftStore
from control_station.models import CameraSnapshot, TaskStatus
from control_station.projection_drafts import (
    ProjectionDraftError,
    read_projection_draft_artifact,
    validate_projection_draft_artifact,
)
from control_station.robot_adapters import make_status_provider
from control_station.shelf_regions import evaluate_shelf_level
from control_station.store import (
    StateTransitionConflict,
    StepNotFoundError,
    TaskNotFoundError,
    TaskStore,
)
from control_station.task_orchestrator import MockTaskOrchestrator, TaskExecutionConflict


WEB_DIR = Path(__file__).with_name("web")
COLOR_EVALUATION_ALGORITHM_VERSION = "exact-estimated-hint-match-v1"


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON object key: {key}")
        value[key] = item
    return value


def validate_pick_request(value: object) -> dict[str, object] | None:
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
    """Reject persisted task conditions that do not match the current API contract."""
    try:
        validated = validate_pick_request(value)
    except ValueError as exc:
        raise ValueError("Stored pick_request does not match the supported contract") from exc
    if validated is None or validated != value:
        raise ValueError("Stored pick_request is missing or is not in canonical form")
    return validated


def pending_pick_step_id(task: dict[str, Any], step_type: str) -> str | None:
    """Return the unique pending workflow step used to scope review evidence."""
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


def _verified_observation_frame(
    task_id: str,
    artifact: object,
    frame_sha256: object,
    frame_width: object,
    frame_height: object,
) -> bool:
    if not re.fullmatch(r"TASK-[A-F0-9]{8}", task_id) or not isinstance(artifact, dict):
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
        or not re.fullmatch(r"[0-9a-f]{64}", frame_sha256)
        or not isinstance(relative_path, str)
        or not isinstance(file_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", file_sha256)
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
    try:
        repository_root = Path(__file__).resolve().parents[2]
        control_directory = repository_root / ".control-station"
        artifact_root = control_directory / "observations"
        task_directory = artifact_root / task_id
        image_path = task_directory / path_parts[3]
        if any(
            path.is_symlink()
            for path in (control_directory, artifact_root, task_directory, image_path)
        ):
            return False
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
            return False
        with image_path.open("rb") as image_file:
            image_bytes = image_file.read(MAX_OBSERVATION_IMAGE_BYTES + 1)
    except (OSError, RuntimeError):
        return False
    return (
        len(image_bytes) <= MAX_OBSERVATION_IMAGE_BYTES
        and hashlib.sha256(image_bytes).hexdigest() == file_sha256
    )


class ControlStationServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class ControlStationHandler(SimpleHTTPRequestHandler):
    server_version = "ControlStation/0.2"

    @property
    def app(self) -> ControlStationServer:
        return self.server  # type: ignore[return-value]

    def log_message(self, format: str, *args: object) -> None:
        super().log_message(format, *args)

    def _json(self, status: int, body: dict[str, Any]) -> None:
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _json_error(
        self,
        status: int,
        error_code: str,
        message: str,
        **details: object,
    ) -> None:
        body: dict[str, Any] = {
            "error_code": error_code,
            "message": message,
            "error": message,
        }
        body.update(details)
        self._json(status, body)

    def _image(self, payload: bytes) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        route = urlsplit(self.path).path
        if route.startswith("/streams/") and route.endswith(".mjpg"):
            camera_id = route.removeprefix("/streams/").removesuffix(".mjpg")
            self._stream_camera(camera_id)
            return
        if route == "/api/health":
            self._json(200, {"status": "ok", "mode": self.app.mode})
        elif route == "/api/robots":
            self._json(200, {"robots": self.app.robot_provider.snapshots()})
        elif route == "/api/cameras":
            self._json(
                200,
                {"cameras": [camera.to_dict() for camera in self.app.camera_manager.snapshots()]},
            )
        elif route.startswith("/api/cameras/") and route.endswith("/target-observation"):
            camera_id = route.removeprefix("/api/cameras/").removesuffix("/target-observation")
            query = parse_qs(urlsplit(self.path).query, keep_blank_values=True)
            class_names = query.get("class_name", [])
            colors = query.get("color", [])
            shelf_levels = query.get("shelf_level", [])
            if (
                set(query) - {"class_name", "color", "shelf_level"}
                or len(class_names) != 1
                or len(colors) > 1
                or len(shelf_levels) > 1
            ):
                self._json(
                    400,
                    {"error": "class_name is required once; color and shelf_level may be provided once"},
                )
                return
            target_class = class_names[0].strip()
            if not target_class or len(target_class) > 80:
                self._json(400, {"error": "class_name must contain 1 to 80 characters"})
                return
            requested_color = colors[0].strip() if colors else None
            if requested_color is not None and (not requested_color or len(requested_color) > 40):
                self._json(400, {"error": "color must contain 1 to 40 characters"})
                return
            requested_shelf_level: int | None = None
            if shelf_levels:
                raw_level = shelf_levels[0].strip()
                if (
                    not re.fullmatch(r"[1-9][0-9]*", raw_level)
                    or len(raw_level) > len(str(MAX_SHELF_LEVEL))
                    or int(raw_level) > MAX_SHELF_LEVEL
                ):
                    self._json(
                        400,
                        {"error": f"shelf_level must be an integer from 1 to {MAX_SHELF_LEVEL}"},
                    )
                    return
                requested_shelf_level = int(raw_level)
            cameras = self.app.camera_manager.snapshots()
            camera = next((item for item in cameras if item.id == camera_id), None)
            if camera is None:
                self._json(404, {"error": "Camera not found"})
                return
            self._json(
                200,
                {
                    "observation": self._target_observation(
                        camera, target_class, requested_color, requested_shelf_level
                    )
                },
            )
        elif route.startswith("/api/tasks/") and "/observations/" in route and route.endswith("/image"):
            parts = route.strip("/").split("/")
            if (
                len(parts) != 6
                or parts[:2] != ["api", "tasks"]
                or parts[3] != "observations"
                or parts[5] != "image"
            ):
                self._json(404, {"error": "Observation image not found"})
                return
            self._serve_observation_image(parts[2], parts[4])
        elif (
            route.startswith("/api/tasks/")
            and "/projection-drafts/" in route
            and route.endswith("/artifact")
        ):
            parts = route.strip("/").split("/")
            if (
                len(parts) != 6
                or parts[:2] != ["api", "tasks"]
                or parts[3] != "projection-drafts"
                or parts[5] != "artifact"
                or not re.fullmatch(r"TASK-[A-F0-9]{8}", parts[2])
                or not re.fullmatch(r"EVT-[A-F0-9]{12}", parts[4])
            ):
                self._json(404, {"error": "Projection draft artifact not found"})
                return
            task = self.app.task_store.get_task(parts[2])
            if task is None:
                self._json(404, {"error": "Projection draft artifact not found"})
                return
            matches = [
                event
                for event in task.get("events", [])
                if isinstance(event, dict) and event.get("id") == parts[4]
            ]
            if len(matches) != 1 or matches[0].get("event_type") != "TARGET_PLANE_PROJECTION_RECORDED":
                self._json(404, {"error": "Projection draft artifact not found"})
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
                self._json(409, {"error": "Projection review event payload is invalid"})
                return
            try:
                artifact = read_projection_draft_artifact(
                    Path(__file__).resolve().parents[2],
                    event_payload.get("artifact_path"),
                    event_payload.get("artifact_sha256"),
                )
            except ProjectionDraftError as exc:
                self._json(409, {"error": str(exc)})
                return
            if (
                artifact.get("id") != event_payload.get("draft_id")
                or artifact.get("task_id") != parts[2]
                or artifact.get("status") != "DRAFT_ONLY"
            ):
                self._json(409, {"error": "Projection artifact no longer matches its review event"})
                return
            self._json(200, {"artifact": artifact, "review_event_id": parts[4]})
        elif route == "/api/arm/status":
            self._json(200, {"arm": self.app.arm_supervisor.snapshot()})
        elif route == "/api/tasks":
            self._json(200, {"tasks": self.app.task_store.list_tasks()})
        elif route.startswith("/api/tasks/"):
            task_id = route.removeprefix("/api/tasks/")
            if not task_id or "/" in task_id:
                self._json(404, {"error": "Task not found"})
                return
            task = self.app.task_store.get_task(task_id)
            if task is None:
                self._json(404, {"error": "Task not found"})
            else:
                task["local_execution_owner"] = self.app.task_orchestrator.local_worker_state(task_id)
                self._json(200, {"task": task})
        else:
            if route == "/":
                self.path = "/index.html"
            elif route.startswith("/api/"):
                self._json(404, {"error": "API route not found"})
                return
            super().do_GET()

    def _serve_observation_image(self, task_id: str, event_id: str) -> None:
        if not re.fullmatch(r"TASK-[A-F0-9]{8}", task_id) or not re.fullmatch(
            r"EVT-[A-F0-9]{12}", event_id
        ):
            self._json(404, {"error": "Observation image not found"})
            return
        task = self.app.task_store.get_task(task_id)
        if task is None:
            self._json(404, {"error": "Observation image not found"})
            return
        matching_events = [event for event in task["events"] if event.get("id") == event_id]
        if len(matching_events) != 1:
            self._json(404, {"error": "Observation image not found"})
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
            self._json(404, {"error": "Observation image not found"})
            return

        relative_path = artifact.get("path")
        expected_hash = artifact.get("file_sha256")
        if (
            not isinstance(relative_path, str)
            or not isinstance(expected_hash, str)
            or not re.fullmatch(r"[0-9a-f]{64}", expected_hash)
            or not _verified_observation_frame(
                task_id,
                artifact,
                observation.get("frame_sha256"),
                observation.get("frame_width"),
                observation.get("frame_height"),
            )
        ):
            self._json(404, {"error": "Observation image not found"})
            return
        try:
            repository_root = Path(__file__).resolve().parents[2]
            artifact_root = repository_root / ".control-station" / "observations"
            task_artifact_root = artifact_root / task_id
            image_path = task_artifact_root / relative_path.split("/")[-1]
            if any(
                path.is_symlink()
                for path in (
                    repository_root / ".control-station",
                    artifact_root,
                    task_artifact_root,
                    image_path,
                )
            ):
                self._json(404, {"error": "Observation image not found"})
                return
            resolved_artifact_root = artifact_root.resolve(strict=True)
            resolved_task_root = task_artifact_root.resolve(strict=True)
            resolved_image = image_path.resolve(strict=True)
            if (
                resolved_artifact_root != artifact_root
                or resolved_task_root != task_artifact_root
                or resolved_task_root.parent != resolved_artifact_root
                or resolved_image != image_path
                or image_path.suffix != ".png"
                or not image_path.is_file()
                or image_path.stat().st_size > MAX_OBSERVATION_IMAGE_BYTES
            ):
                self._json(404, {"error": "Observation image not found"})
                return
            with image_path.open("rb") as image_file:
                image_bytes = image_file.read(MAX_OBSERVATION_IMAGE_BYTES + 1)
        except (OSError, RuntimeError):
            self._json(404, {"error": "Observation image not found"})
            return
        if len(image_bytes) > MAX_OBSERVATION_IMAGE_BYTES:
            self._json(404, {"error": "Observation image not found"})
            return
        if hashlib.sha256(image_bytes).hexdigest() != expected_hash:
            self._json(409, {"error": "Observation image integrity check failed"})
            return
        self._image(image_bytes)

    @staticmethod
    def _evaluate_color_hints(
        candidates: list[dict[str, object]], requested_color: str
    ) -> dict[str, object]:
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

    @staticmethod
    def _target_observation(
        camera: CameraSnapshot,
        target_class: str,
        requested_color: str | None = None,
        requested_shelf_level: int | None = None,
    ) -> dict[str, Any]:
        """Classify a camera observation without creating a robot-control signal."""
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
        elif not isinstance(camera.pixel_transform_id, str) or not re.fullmatch(
            r"[A-Za-z0-9_.-]{1,80}", camera.pixel_transform_id
        ):
            status = "PIXEL_TRANSFORM_UNKNOWN"
        elif not isinstance(camera.frame_sha256, str) or not re.fullmatch(
            r"[0-9a-f]{64}", camera.frame_sha256
        ):
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
            observation["color_evaluation"] = ControlStationHandler._evaluate_color_hints(
                candidates, requested_color
            )
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

    def _stream_camera(self, camera_id: str) -> None:
        manager = self.app.camera_manager
        known_ids = {camera.id for camera in manager.snapshots()}
        if camera_id not in known_ids or not hasattr(manager, "sequence_for"):
            self._json(404, {"error": "Camera stream not found"})
            return

        frame = manager.frame_for(camera_id)
        for _ in range(50):
            if frame is not None:
                break
            time.sleep(0.1)
            frame = manager.frame_for(camera_id)
        if frame is None:
            self._json(503, {"error": "Camera is not producing frames"})
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Connection", "close")
        self.end_headers()
        last_sequence = -1
        try:
            while True:
                sequence = manager.sequence_for(camera_id)
                if sequence == last_sequence:
                    time.sleep(0.02)
                    continue
                frame = manager.frame_for(camera_id)
                if frame is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                    + frame
                    + b"\r\n"
                )
                self.wfile.flush()
                last_sequence = sequence
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def do_POST(self) -> None:
        route = urlsplit(self.path).path
        parts = route.strip("/").split("/")
        if len(parts) == 4 and parts[:2] == ["api", "tasks"] and parts[3] in {"start", "cancel"}:
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                message = "Cross-origin task-control request rejected"
                self._json(
                    403,
                    {
                        "error_code": "CROSS_ORIGIN_REQUEST_REJECTED",
                        "message": message,
                        "error": message,
                    },
                )
                return
            task_id = parts[2]
            if not task_id:
                message = "Task not found"
                self._json(
                    404,
                    {
                        "error_code": "TASK_NOT_FOUND",
                        "message": message,
                        "error": message,
                    },
                )
                return
            try:
                task, changed = (
                    self.app.task_orchestrator.start(task_id)
                    if parts[3] == "start"
                    else self.app.task_orchestrator.cancel(task_id)
                )
            except TaskExecutionConflict as exc:
                body: dict[str, Any] = {
                    "error_code": exc.error_code,
                    "message": str(exc),
                    "error": str(exc),
                }
                if exc.task is not None:
                    body["task"] = exc.task
                self._json(409, body)
                return
            except LookupError:
                message = "Task not found"
                self._json(
                    404,
                    {
                        "error_code": "TASK_NOT_FOUND",
                        "message": message,
                        "error": message,
                    },
                )
                return
            except RuntimeError as exc:
                self._json(
                    409,
                    {
                        "error_code": "TASK_CONTROL_ERROR",
                        "message": str(exc),
                        "error": str(exc),
                    },
                )
                return
            accepted = changed and task["status"] not in {
                "SUCCEEDED",
                "FAILED",
                "CANCELED",
            }
            self._json(202 if accepted else 200, {"task": task, "execution_mode": "mock"})
            return
        if route in {"/api/arm/start", "/api/arm/stop"}:
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self._json(403, {"error": "Cross-origin arm-control request rejected"})
                return
            try:
                state = (
                    self.app.arm_supervisor.start()
                    if route == "/api/arm/start"
                    else self.app.arm_supervisor.stop()
                )
            except RuntimeError as exc:
                self._json(409, {"error": str(exc), "arm": self.app.arm_supervisor.snapshot()})
                return
            self._json(202, {"arm": state})
            return
        target_observation_route = (
            len(parts) == 4
            and parts[:2] == ["api", "tasks"]
            and parts[3] == "target-observation"
        )
        target_selection_route = (
            len(parts) == 4
            and parts[:2] == ["api", "tasks"]
            and parts[3] == "target-selection"
        )
        projection_draft_route = (
            len(parts) == 4
            and parts[:2] == ["api", "tasks"]
            and parts[3] == "projection-draft"
        )
        parse_command_route = route == "/api/tasks/parse-command"
        if parse_command_route:
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self._json(403, {"error": "Cross-origin command-preview request rejected"})
                return
        if target_observation_route:
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self._json_error(
                    403,
                    "CROSS_ORIGIN_REQUEST_REJECTED",
                    "Cross-origin task-observation request rejected",
                )
                return
        if target_selection_route:
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self._json_error(
                    403,
                    "CROSS_ORIGIN_REQUEST_REJECTED",
                    "Cross-origin target-selection request rejected",
                )
                return
        if projection_draft_route:
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self._json_error(
                    403,
                    "CROSS_ORIGIN_REQUEST_REJECTED",
                    "Cross-origin projection-draft request rejected",
                )
                return
        if route == "/api/arm/coordinate-target-drafts":
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self._json(403, {"error": "Cross-origin coordinate draft request rejected"})
                return
        elif (
            route != "/api/tasks"
            and not target_observation_route
            and not target_selection_route
            and not projection_draft_route
            and not parse_command_route
        ):
            self._json(404, {"error": "API route not found"})
            return
        if self.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
            self._json(415, {"error": "Content-Type must be application/json"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._json(400, {"error": "Invalid Content-Length"})
            return
        if not 0 < length <= 4096:
            self._json(400, {"error": "Request body must be between 1 and 4096 bytes"})
            return
        try:
            body = json.loads(
                self.rfile.read(length), object_pairs_hook=_unique_json_object
            )
        except (ValueError, UnicodeDecodeError, RecursionError):
            self._json(
                400,
                {"error": "Request body must be valid JSON with unique object keys"},
            )
            return
        if not isinstance(body, dict):
            self._json(400, {"error": "Request body must be a JSON object"})
            return
        if parse_command_route:
            text = body.get("text")
            if set(body) != {"text"} or not isinstance(text, str) or not text.strip() or len(text) > 500:
                self._json(400, {"error": "text must contain 1 to 500 characters"})
                return
            search_camera_detector = next(
                (
                    camera
                    for camera in self.app.camera_manager.snapshots()
                    if camera.position in {"HEAD", "FIXED"}
                ),
                None,
            )
            known_classes = (
                list(dict.fromkeys(search_camera_detector.detector_classes or []))
                if search_camera_detector is not None
                and search_camera_detector.detector_state == "ready"
                else []
            )
            allow_open_vocabulary = (
                search_camera_detector is not None
                and search_camera_detector.detector_state == "ready"
                and search_camera_detector.detector_model_type == "YOLO_WORLD_OPEN_VOCAB"
            )
            self._json(
                200,
                {
                    "draft": parse_pick_command(
                        text,
                        known_classes or None,
                        allow_open_vocabulary=allow_open_vocabulary,
                    )
                },
            )
            return
        if projection_draft_route:
            task_id = parts[2]
            if set(body) != {"artifact_path", "artifact_sha256"}:
                self._json_error(
                    400,
                    "INVALID_PROJECTION_DRAFT_REQUEST",
                    "artifact_path and artifact_sha256 are required",
                )
                return
            task = self.app.task_store.get_task(task_id)
            if task is None:
                self._json_error(404, "TASK_NOT_FOUND", "Task not found")
                return
            try:
                validate_stored_pick_request(task.get("pick_request"))
            except ValueError as exc:
                self._json_error(409, "PICK_REQUEST_INVALID", str(exc))
                return
            try:
                projection_step_id = pending_pick_step_id(task, "ESTIMATE_TARGET_POSE")
                if projection_step_id is None:
                    self._json_error(
                        409,
                        "POSE_ESTIMATION_STEP_UNAVAILABLE",
                        "Task has no unique pending ESTIMATE_TARGET_POSE step",
                    )
                    return
                projection_payload = validate_projection_draft_artifact(
                    task,
                    Path(__file__).resolve().parents[2],
                    body.get("artifact_path"),
                    body.get("artifact_sha256"),
                )
                _, event_id, already_recorded = (
                    self.app.task_store.record_projection_draft_event(
                        task_id,
                        projection_payload,
                        expected_status=TaskStatus.PENDING,
                        step_id=projection_step_id,
                    )
                )
            except ProjectionDraftError as exc:
                self._json_error(409, "PROJECTION_DRAFT_REJECTED", str(exc))
                return
            except TaskNotFoundError:
                self._json_error(404, "TASK_NOT_FOUND", "Task not found")
                return
            except StepNotFoundError:
                self._json_error(
                    409,
                    "POSE_ESTIMATION_STEP_UNAVAILABLE",
                    "Pose-estimation workflow step is unavailable",
                )
                return
            except StateTransitionConflict as exc:
                self._json_error(409, "TASK_STATE_CONFLICT", str(exc))
                return
            self._json(
                200 if already_recorded else 201,
                {
                    "event_id": event_id,
                    "projection_draft": projection_payload,
                    "execution_started": False,
                    "already_recorded": already_recorded,
                },
            )
            return
        if target_selection_route:
            task_id = parts[2]
            observation_event_id = body.get("observation_event_id")
            candidate_index = body.get("candidate_index")
            if (
                set(body) != {"observation_event_id", "candidate_index"}
                or not isinstance(observation_event_id, str)
                or not re.fullmatch(r"EVT-[A-F0-9]{12}", observation_event_id)
                or isinstance(candidate_index, bool)
                or not isinstance(candidate_index, int)
                or candidate_index < 0
            ):
                self._json_error(
                    400,
                    "INVALID_TARGET_SELECTION_REQUEST",
                    "observation_event_id and a non-negative candidate_index are required",
                )
                return
            task = self.app.task_store.get_task(task_id)
            if task is None:
                self._json_error(404, "TASK_NOT_FOUND", "Task not found")
                return
            if task.get("pick_request") is None:
                self._json_error(
                    409,
                    "PICK_REQUEST_MISSING",
                    "Task has no structured pick request",
                )
                return
            try:
                pick_request = validate_stored_pick_request(task.get("pick_request"))
            except ValueError as exc:
                self._json_error(409, "PICK_REQUEST_INVALID", str(exc))
                return
            if task["status"] != TaskStatus.PENDING.value:
                self._json_error(
                    409,
                    "TASK_NOT_PENDING",
                    "A candidate can be recorded only while the task is pending",
                )
                return
            selection_step_id = pending_pick_step_id(task, "SELECT_TARGET")
            if selection_step_id is None:
                self._json_error(
                    409,
                    "SELECTION_STEP_UNAVAILABLE",
                    "Task has no unique pending SELECT_TARGET step",
                )
                return
            detection_step_id = pending_pick_step_id(task, "DETECT_TARGETS")
            if detection_step_id is None:
                self._json_error(
                    409,
                    "DETECTION_STEP_UNAVAILABLE",
                    "Task has no unique pending DETECT_TARGETS step",
                )
                return
            matches = [event for event in task["events"] if event.get("id") == observation_event_id]
            if len(matches) != 1:
                self._json_error(
                    404,
                    "OBSERVATION_EVENT_NOT_FOUND",
                    "Target observation event not found",
                )
                return
            source_event = matches[0]
            source_payload = source_event.get("payload")
            observation = source_payload.get("observation") if isinstance(source_payload, dict) else None
            if (
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
                or not re.fullmatch(r"[A-Za-z0-9_.-]{1,80}", observation.get("camera_id", ""))
                or isinstance(observation.get("frame_sequence"), bool)
                or not isinstance(observation.get("frame_sequence"), int)
                or observation.get("frame_sequence") <= 0
                or not isinstance(observation.get("captured_at"), str)
                or not observation.get("captured_at", "").strip()
                or not isinstance(observation.get("frame_sha256"), str)
                or not re.fullmatch(r"[0-9a-f]{64}", observation.get("frame_sha256", ""))
                or not isinstance(observation.get("pixel_transform_id"), str)
                or not re.fullmatch(
                    r"[A-Za-z0-9_.-]{1,80}", observation.get("pixel_transform_id", "")
                )
                or any(
                    isinstance(observation.get(field), bool)
                    or not isinstance(observation.get(field), int)
                    or observation.get(field) <= 0
                    for field in ("frame_width", "frame_height")
                )
            ):
                self._json_error(
                    409,
                    "OBSERVATION_NOT_SELECTABLE",
                    "Source event is not a selectable fresh observation for this task",
                )
                return
            candidates = observation.get("candidates")
            candidate_count = observation.get("candidate_count")
            if (
                not isinstance(candidates, list)
                or isinstance(candidate_count, bool)
                or not isinstance(candidate_count, int)
                or candidate_count != len(candidates)
                or candidate_count < 1
                or (
                    observation.get("status") == "SINGLE_CANDIDATE"
                    and candidate_count != 1
                )
                or (
                    observation.get("status") == "AMBIGUOUS"
                    and candidate_count < 2
                )
                or candidate_index >= len(candidates)
            ):
                self._json_error(
                    409,
                    "CANDIDATE_INDEX_OUT_OF_RANGE",
                    "Candidate index is outside the recorded observation",
                )
                return
            candidate = candidates[candidate_index]
            if (
                not isinstance(candidate, dict)
                or not isinstance(candidate.get("class_name"), str)
                or candidate.get("class_name", "").casefold()
                != pick_request.get("category", "").casefold()
                or candidate.get("camera_id") != observation.get("camera_id")
                or candidate.get("frame_sequence") != observation.get("frame_sequence")
                or candidate.get("captured_at") != observation.get("captured_at")
                or candidate.get("frame_sha256") != observation.get("frame_sha256")
                or candidate.get("pixel_transform_id") != observation.get("pixel_transform_id")
                or candidate.get("frame_width") != observation.get("frame_width")
                or candidate.get("frame_height") != observation.get("frame_height")
            ):
                self._json_error(
                    409,
                    "CANDIDATE_PROVENANCE_MISMATCH",
                    "Candidate provenance does not match its observation",
                )
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
            latest_task = self.app.task_store.get_task(task_id)
            if latest_task is None:
                self._json_error(404, "TASK_NOT_FOUND", "Task not found")
                return
            if latest_task["status"] != TaskStatus.PENDING.value:
                self._json_error(
                    409,
                    "TASK_NOT_PENDING",
                    "A candidate can be recorded only while the task is pending",
                )
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
                    self._json_error(
                        500,
                        "SELECTION_EVENT_NOT_RETRIEVABLE",
                        "The existing selection event has no readable ID",
                    )
                    return
                self._json(
                    200,
                    {
                        "task": latest_task,
                        "selection": selection_payload,
                        "selection_event_id": existing_event_id,
                        "already_recorded": True,
                    },
                )
                return
            if not _verified_observation_frame(
                task_id,
                source_payload.get("frame_artifact"),
                observation.get("frame_sha256"),
                observation.get("frame_width"),
                observation.get("frame_height"),
            ):
                self._json_error(
                    409,
                    "SOURCE_FRAME_REQUIRED",
                    "A saved, hash-verified source frame is required for operator selection",
                )
                return
            try:
                recorded, selection_event_id, already_recorded = (
                    self.app.task_store.record_target_selection_event(
                        task_id,
                        selection_payload,
                        expected_status=TaskStatus.PENDING,
                        step_id=selection_step_id,
                    )
                )
            except TaskNotFoundError:
                self._json_error(404, "TASK_NOT_FOUND", "Task not found")
                return
            except StepNotFoundError:
                self._json_error(
                    409,
                    "SELECTION_STEP_UNAVAILABLE",
                    "Target-selection workflow step is unavailable",
                )
                return
            except StateTransitionConflict as exc:
                self._json_error(409, "TASK_STATE_CONFLICT", str(exc))
                return
            if not isinstance(selection_event_id, str):
                self._json_error(
                    500,
                    "SELECTION_EVENT_NOT_RETRIEVABLE",
                    "Selection was recorded but its event ID could not be read back",
                )
                return
            self._json(
                200 if already_recorded else 201,
                {
                    "task": recorded,
                    "selection": selection_payload,
                    "selection_event_id": selection_event_id,
                    "already_recorded": already_recorded,
                },
            )
            return
        if target_observation_route:
            task_id = parts[2]
            camera_id = body.get("camera_id")
            save_frame = body.get("save_frame", False)
            if set(body) - {"camera_id", "save_frame"}:
                self._json_error(
                    400,
                    "INVALID_TARGET_OBSERVATION_REQUEST",
                    "Only camera_id and optional save_frame are accepted",
                )
                return
            if not task_id or not isinstance(camera_id, str) or not camera_id.strip():
                self._json_error(
                    400,
                    "INVALID_TARGET_OBSERVATION_REQUEST",
                    "task_id and camera_id are required",
                )
                return
            if not isinstance(save_frame, bool):
                self._json_error(
                    400,
                    "INVALID_TARGET_OBSERVATION_REQUEST",
                    "save_frame must be a boolean",
                )
                return
            task = self.app.task_store.get_task(task_id)
            if task is None:
                self._json_error(404, "TASK_NOT_FOUND", "Task not found")
                return
            if task.get("pick_request") is None:
                self._json_error(
                    409,
                    "PICK_REQUEST_MISSING",
                    "Task has no structured pick request",
                )
                return
            try:
                pick_request = validate_stored_pick_request(task.get("pick_request"))
            except ValueError as exc:
                self._json_error(409, "PICK_REQUEST_INVALID", str(exc))
                return
            if task["status"] != TaskStatus.PENDING.value:
                self._json_error(
                    409,
                    "TASK_NOT_PENDING",
                    "Target observation can be recorded only while the task is pending",
                )
                return
            cameras = self.app.camera_manager.snapshots()
            camera = next((item for item in cameras if item.id == camera_id.strip()), None)
            if camera is None:
                self._json_error(404, "CAMERA_NOT_FOUND", "Camera not found")
                return
            observation_step_type = {
                "HEAD": "DETECT_TARGETS",
                "FIXED": "DETECT_TARGETS",
                "JOINT": "REFINE_TARGET",
            }.get(camera.position)
            if observation_step_type is None:
                self._json_error(
                    409,
                    "CAMERA_ROLE_UNSUPPORTED",
                    "Camera position has no Pick & Place observation step",
                )
                return
            observation_step_id = pending_pick_step_id(task, observation_step_type)
            if observation_step_id is None:
                self._json_error(
                    409,
                    "OBSERVATION_STEP_UNAVAILABLE",
                    f"Task has no unique pending {observation_step_type} step",
                )
                return
            frame_artifact: dict[str, object] | None = None
            if save_frame:
                save_observation = getattr(self.app.camera_manager, "save_observation_frame", None)
                if not callable(save_observation):
                    self._json_error(
                        409,
                        "FRAME_SAVE_UNAVAILABLE",
                        "Camera manager cannot save observation frames",
                    )
                    return
                frame_sha256 = camera.frame_sha256
                frame_sequence = camera.frame_sequence
                if not isinstance(frame_sha256, str) or not isinstance(frame_sequence, int):
                    self._json_error(
                        409,
                        "FRAME_PROVENANCE_UNAVAILABLE",
                        "The current camera frame has no saveable provenance",
                    )
                    return
                try:
                    frame_artifact = save_observation(
                        camera.id, frame_sequence, frame_sha256, task_id
                    )
                except (RuntimeError, ValueError, OSError) as exc:
                    self._json_error(
                        409,
                        "FRAME_SAVE_FAILED",
                        f"Could not save the requested camera frame: {exc}",
                    )
                    return
            target_class = str(pick_request["category"])
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
                detect_observation = getattr(
                    self.app.camera_manager, "detect_for_observation", None
                )
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
                and not any(
                    name.casefold() == target_class.casefold()
                    for name in camera.detector_classes
                )
            ):
                request_prompt_attempted = True
                observe_prompt = getattr(
                    self.app.camera_manager, "detect_for_prompt", None
                )
                if callable(observe_prompt):
                    try:
                        camera = observe_prompt(camera, target_class)
                    except Exception as exc:
                        request_prompt_error = f"{type(exc).__name__}: {exc}"[:512]
                        try:
                            latest_camera = next(
                                (
                                    item
                                    for item in self.app.camera_manager.snapshots()
                                    if item.id == camera.id
                                ),
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
            requested_color = pick_request.get("color")
            requested_shelf_level = pick_request.get("shelf_level")
            observation = self._target_observation(
                camera,
                target_class,
                requested_color if isinstance(requested_color, str) else None,
                requested_shelf_level
                if isinstance(requested_shelf_level, int)
                and not isinstance(requested_shelf_level, bool)
                and requested_shelf_level > 0
                else None,
            )
            if request_prompt_attempted:
                detector_provenance = observation.get("detector_provenance")
                if isinstance(detector_provenance, dict):
                    detector_provenance["prompt_scope"] = "REQUEST_CLASS_ONE_SHOT"
                    detector_provenance["requested_prompt"] = target_class
                    detector_provenance["prompt_policy_version"] = (
                        YOLO_WORLD_REQUEST_PROMPT_POLICY_VERSION
                    )
                    detector_provenance["configured_prompts"] = camera.detector_prompts
                    if request_prompt_error is not None:
                        detector_provenance["prompt_inference_error"] = request_prompt_error
                if request_prompt_error is not None:
                    observation["status"] = "REQUEST_PROMPT_INFERENCE_ERROR"
            if fixed_detector_attempted:
                detector_provenance = observation.get("detector_provenance")
                if isinstance(detector_provenance, dict):
                    detector_provenance["inference_scope"] = "FIXED_CAMERA_TASK_OBSERVATION"
                    if fixed_detector_error is not None:
                        detector_provenance["inference_error"] = fixed_detector_error
                if fixed_detector_error is not None:
                    observation["status"] = "DETECTOR_INFERENCE_ERROR"
            if camera.position == "FIXED":
                summarize_depth = getattr(
                    self.app.camera_manager, "summarize_detection_depth", None
                )
                if callable(summarize_depth):
                    try:
                        observation["depth_region_observation"] = summarize_depth(
                            camera, observation["candidates"]
                        )
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
                delete_observation = getattr(
                    self.app.camera_manager, "delete_observation_frame", None
                )
                if callable(delete_observation):
                    try:
                        delete_observation(str(frame_artifact["path"]))
                    except (OSError, ValueError):
                        pass

            try:
                recorded = self.app.task_store.record_task_event(
                    task_id,
                    "TARGET_OBSERVED",
                    event_payload,
                    expected_status=TaskStatus.PENDING,
                    step_id=observation_step_id,
                )
            except TaskNotFoundError:
                discard_saved_frame()
                self._json_error(404, "TASK_NOT_FOUND", "Task not found")
                return
            except StepNotFoundError:
                discard_saved_frame()
                self._json_error(
                    409,
                    "OBSERVATION_STEP_UNAVAILABLE",
                    "Camera observation workflow step is unavailable",
                )
                return
            except StateTransitionConflict as exc:
                discard_saved_frame()
                self._json_error(409, "TASK_STATE_CONFLICT", str(exc))
                return
            except ValueError as exc:
                discard_saved_frame()
                self._json_error(409, "OBSERVATION_RECORD_REJECTED", str(exc))
                return
            except Exception:
                discard_saved_frame()
                raise
            self._json(
                201,
                {
                    "task": recorded,
                    "observation": observation,
                    "frame_artifact": frame_artifact,
                },
            )
            return
        if route == "/api/arm/coordinate-target-drafts":
            try:
                draft = self.app.coordinate_target_drafts.create(body)
            except CoordinateDraftError as exc:
                self._json(400, {"error": str(exc)})
                return
            self._json(201, {"draft": draft})
            return
        unsupported_fields = set(body) - {"destination", "arm_job", "pick_request"}
        if unsupported_fields:
            self._json(
                400,
                {
                    "error": "Unsupported task fields: "
                    + ", ".join(sorted(unsupported_fields))
                },
            )
            return
        destination = body.get("destination")
        arm_job = body.get("arm_job")
        if not isinstance(destination, str) or not isinstance(arm_job, str):
            self._json(400, {"error": "destination and arm_job must be strings"})
            return
        destination, arm_job = destination.strip(), arm_job.strip()
        if not destination or len(destination) > 40:
            self._json(400, {"error": "destination must contain 1 to 40 characters"})
            return
        if not arm_job or len(arm_job) > 120:
            self._json(400, {"error": "arm_job must contain 1 to 120 characters"})
            return
        try:
            pick_request = validate_pick_request(body.get("pick_request"))
        except ValueError as exc:
            self._json(400, {"error": str(exc)})
            return
        task = self.app.task_store.create_task(destination, arm_job, pick_request)
        self._json(201, {"task": task})


def main() -> None:
    mode = os.environ.get("CONTROL_STATION_MODE", "mock").strip().lower()
    provider = make_status_provider(mode)
    task_store = TaskStore()
    host = "127.0.0.1"
    port = int(os.environ.get("CONTROL_STATION_PORT", "8000"))
    handler = partial(ControlStationHandler, directory=str(WEB_DIR))
    server = ControlStationServer((host, port), handler)
    server.mode = mode
    server.robot_provider = provider
    server.arm_supervisor = ArmTeleopSupervisor(
        disabled_reason=(
            "MuJoCo 관측 모드에서는 실물 LeRobot 텔레오퍼레이션을 사용할 수 없습니다."
            if mode == "mujoco"
            else None
        )
    )
    head_detector_configured = any(
        os.environ.get(name, "").strip()
        for name in (
            "CONTROL_STATION_HEAD_YOLO_WEIGHTS",
            "CONTROL_STATION_HEAD_YOLO_WORLD_PROMPTS",
        )
    )
    if mode == "mujoco":
        raw_domain = os.environ.get("CONTROL_STATION_MOBILE_DOMAIN", os.environ.get("ROS_DOMAIN_ID"))
        if raw_domain is None:
            raise RuntimeError("MuJoCo 카메라 구독에 ROS domain ID를 지정해야 합니다.")
        server.camera_manager = Ros2SimulationCameraStreamManager(int(raw_domain))
    elif not (server.arm_supervisor.enabled or head_detector_configured):
        server.camera_manager = UnconfiguredArmCameraStreamManager()
    else:
        server.camera_manager = OpenCVArmCameraStreamManager(
            joint_enabled=server.arm_supervisor.enabled
        )
    server.task_store = task_store
    server.task_orchestrator = MockTaskOrchestrator(task_store, mode=mode)
    draft_dir = os.environ.get(
        "CONTROL_STATION_COORDINATE_DRAFT_DIR", ".control-station/coordinate-target-drafts"
    )
    server.coordinate_target_drafts = CoordinateTargetDraftStore(draft_dir)
    mode_label = {
        "mock": "모의",
        "ros2": "ROS 2",
        "mujoco": "MuJoCo 시뮬레이션 관측",
    }[mode]
    print(f"관제 서버 ({mode_label} 모드): http://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nControl station stopped.", flush=True)
    finally:
        server.server_close()
        server.camera_manager.close()
        server.arm_supervisor.close()
        server.task_orchestrator.close()
        task_store.close()
        provider.close()


if __name__ == "__main__":
    main()
