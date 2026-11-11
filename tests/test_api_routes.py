"""HTTP behaviour of every control-station route: status codes, error codes and response fields.

Written before splitting app.py so the refactor keeps the API as it is. Uses an in-process server on an ephemeral
port with fake robot, camera and parking sources; nothing here opens a device or sends a robot command.
"""

from __future__ import annotations

import hashlib
import http.client
import json
import os
import shutil
import tempfile
import threading
import time
import unittest
from functools import partial
from pathlib import Path
from unittest import mock

from control_station.app import WEB_DIR, ControlStationHandler, ControlStationServer
from control_station.arm_commands import ArmCommandStore
from control_station.arm_supervisor import ArmTeleopSupervisor
from control_station.policy_run_supervisor import PolicyRunSupervisor
from control_station.coordinate_target_drafts import CoordinateTargetDraftStore
from control_station.models import CameraConnectionState, CameraSnapshot, CameraStreamState, FrameFreshness
from control_station.parking_ros import ParkingObservationError
from control_station.shelf_slot_map import load_shelf_slot_map
from control_station.shelf_slot_zones import ShelfSlotZones
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FRAME_SHA = "a" * 64
CAPTURED_AT = "2026-11-03T12:00:00.000+00:00"
PIXEL_TRANSFORM = "opencv-v4l2-capture-no-app-geometry-v1"


def detection(class_name: str, x: int, color: str) -> dict[str, object]:
    return {
        "class_name": class_name,
        "confidence": 0.9,
        "bbox_xyxy": [x, 100, x + 40, 140],
        "camera_id": "head_camera",
        "frame_sequence": 7,
        "captured_at": CAPTURED_AT,
        "frame_sha256": FRAME_SHA,
        "pixel_transform_id": PIXEL_TRANSFORM,
        "frame_width": 640,
        "frame_height": 480,
        "color_hint": {"status": "ESTIMATED", "color": color},
    }


def head_camera(**changes: object) -> CameraSnapshot:
    values: dict[str, object] = dict(
        id="head_camera",
        robot_id="arm-01",
        position="HEAD",
        connection_state=CameraConnectionState.CONNECTED,
        stream_state=CameraStreamState.STREAMING,
        frame_freshness=FrameFreshness.FRESH,
        last_frame_at=CAPTURED_AT,
        frame_sequence=7,
        frame_width=640,
        frame_height=480,
        frame_sha256=FRAME_SHA,
        pixel_transform_id=PIXEL_TRANSFORM,
        detector_state="ready",
        detector_model_type="YOLO_CLOSED_SET",
        detector_classes=["strawberry", "ball"],
        detections=[detection("strawberry", 100, "red"), detection("strawberry", 300, "green")],
    )
    values.update(changes)
    return CameraSnapshot(**values)  # type: ignore[arg-type]


class FakeProvider:
    def snapshots(self) -> list[dict[str, object]]:
        return [{"id": "pinky-01", "kind": "MOBILE"}]

    def close(self) -> None:
        pass


class FakeCameraManager:
    """Snapshots, a JPEG stream and observation frames saved where app.py verifies them."""

    def __init__(self, cameras: list[CameraSnapshot]) -> None:
        self.cameras = cameras
        self.saved: list[Path] = []

    def snapshots(self) -> list[CameraSnapshot]:
        return list(self.cameras)

    def frame_for(self, camera_id: str) -> bytes | None:
        return b"\xff\xd8jpeg\xff\xd9" if any(c.id == camera_id for c in self.cameras) else None

    def sequence_for(self, camera_id: str) -> int:
        return 1

    def save_observation_frame(self, camera_id: str, frame_sequence: int, frame_sha256: str, task_id: str):  # type: ignore[no-untyped-def]
        directory = REPOSITORY_ROOT / ".control-station" / "observations" / task_id
        directory.mkdir(parents=True, exist_ok=True)
        image = b"\x89PNG\r\n\x1a\n" + task_id.encode() + str(frame_sequence).encode()
        path = directory / f"{camera_id}-{frame_sequence}.png"
        path.write_bytes(image)
        self.saved.append(directory)
        return {
            "path": str(path.relative_to(REPOSITORY_ROOT)),
            "media_type": "image/png",
            "file_sha256": hashlib.sha256(image).hexdigest(),
            "source_frame_sha256": frame_sha256,
            "pixel_format": "BGR8",
            "width": 640,
            "height": 480,
        }

    def delete_observation_frame(self, relative_path: str) -> None:
        (REPOSITORY_ROOT / relative_path).unlink(missing_ok=True)

    def close(self) -> None:
        pass


class FakeParking:
    spot_id = "pinky_dock_1"

    def __init__(self) -> None:
        self.error: ParkingObservationError | None = None

    def snapshot(self) -> dict[str, object]:
        return {"state": "IN_POSITION", "spot_id": self.spot_id, "display_only": True}

    def check(self, spot_id: str) -> dict[str, object]:
        if self.error is not None:
            raise self.error
        return {"spot_id": spot_id, "status": "ARRIVED", "reason": "within tolerance"}

    def close(self) -> None:
        pass


class ApiTestCase(unittest.TestCase):
    parking = False

    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)
        self.task_store = TaskStore(str(self.directory / "tasks.sqlite3"))
        self.addCleanup(self.task_store.close)
        server = ControlStationServer(("127.0.0.1", 0), partial(ControlStationHandler, directory=str(WEB_DIR)))
        self.cameras = FakeCameraManager([head_camera()])
        self.addCleanup(self._remove_saved_frames)
        self.fake_parking = FakeParking()
        slot_map = load_shelf_slot_map()
        server.mode = "mock"
        server.robot_provider = FakeProvider()
        server.arm_supervisor = ArmTeleopSupervisor(disabled_reason="테스트에서는 팔을 쓰지 않습니다")
        server.policy_run_supervisor = PolicyRunSupervisor(disabled_reason="테스트에서는 정책을 실행하지 않습니다")
        server.arm_control_lock = threading.RLock()
        server.camera_manager = self.cameras
        server.task_store = self.task_store
        server.arm_commands = ArmCommandStore(self.task_store.session_factory, slot_map=slot_map)
        server.shelf_slot_zones = ShelfSlotZones(slot_map, directory=self.directory / "poses")
        server.parking_source = "ros" if self.parking else None
        server.parking_spot_id = "pinky_dock_1" if self.parking else None
        server.parking_camera_id = "head_camera" if self.parking else None
        server.parking_check = self.fake_parking.check if self.parking else None
        server.parking_tracker = self.fake_parking if self.parking else None
        server.task_orchestrator = MockTaskOrchestrator(self.task_store)
        self.addCleanup(server.task_orchestrator.close)
        server.coordinate_target_drafts = CoordinateTargetDraftStore(self.directory / "drafts")
        self.server = server
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)

    def _remove_saved_frames(self) -> None:
        for directory in set(self.cameras.saved):
            shutil.rmtree(directory, ignore_errors=True)

    def request(
        self,
        method: str,
        path: str,
        body: object = None,
        *,
        raw: bytes | None = None,
        content_type: str | None = "application/json",
        origin: str | None = None,
    ) -> tuple[int, object, dict[str, str]]:
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_address[1], timeout=10)
        headers: dict[str, str] = {}
        payload = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        if payload is not None and content_type is not None:
            headers["Content-Type"] = content_type
        if origin is not None:
            headers["Origin"] = origin
        connection.request(method, path, body=payload, headers=headers)
        response = connection.getresponse()
        data = response.read()
        connection.close()
        response_headers = {key.lower(): value for key, value in response.getheaders()}
        if response_headers.get("content-type", "").startswith("application/json"):
            return response.status, json.loads(data), response_headers
        return response.status, data, response_headers

    def get(self, path: str) -> tuple[int, object]:
        status, body, _ = self.request("GET", path)
        return status, body

    def post(self, path: str, body: object = None, **options: object) -> tuple[int, object]:
        status, data, _ = self.request("POST", path, body, **options)  # type: ignore[arg-type]
        return status, data

    def create_task(self, pick_request: dict[str, object] | None = None) -> str:
        body: dict[str, object] = {"destination": "작업대 A", "arm_job": "상자 이동"}
        if pick_request is not None:
            body["pick_request"] = pick_request
        status, data = self.post("/api/tasks", body)
        self.assertEqual(status, 201, data)
        return data["task"]["id"]  # type: ignore[index]


def error(message: str, code: str | None = None, **details: object) -> dict[str, object]:
    return {"error_code": code, "message": message, "error": message, **details} if code else {"error": message}


class ReadRouteTests(ApiTestCase):
    def test_health_without_parking(self) -> None:
        self.assertEqual(self.get("/api/health"), (200, {
            "status": "ok", "mode": "mock",
            "parking": {"source": None, "camera_id": None, "spot_id": None, "live_tracking": False},
        }))

    def test_robots(self) -> None:
        self.assertEqual(self.get("/api/robots"), (200, {"robots": [{"id": "pinky-01", "kind": "MOBILE"}]}))

    def test_cameras(self) -> None:
        # No shelf pose is activated for this camera, so the snapshot passes through without slot_zones.
        self.assertEqual(self.get("/api/cameras"), (200, {"cameras": [json.loads(json.dumps(head_camera().to_dict()))]}))

    def test_camera_target_observation(self) -> None:
        status, body = self.get("/api/cameras/head_camera/target-observation?class_name=strawberry&color=red")
        self.assertEqual(status, 200)
        observation = body["observation"]  # type: ignore[index]
        self.assertEqual(observation["status"], "AMBIGUOUS")
        self.assertEqual(observation["candidate_count"], 2)
        self.assertEqual(observation["color_evaluation"]["status"], "SINGLE_COLOR_HINT_MATCH")
        self.assertEqual(observation["color_evaluation"]["candidate_indices"], [0])
        self.assertEqual(observation["decision_scope"], "OBSERVATION_ONLY")
        self.assertIs(observation["arm_command_sent"], False)

    def test_camera_target_observation_errors(self) -> None:
        base = "/api/cameras/head_camera/target-observation"
        once = "class_name is required once; color and shelf_level may be provided once"
        for query in ("", "?class_name=a&class_name=b", "?class_name=a&other=1", "?class_name=a&color=r&color=g"):
            with self.subTest(query=query):
                self.assertEqual(self.get(base + query), (400, error(once)))
        self.assertEqual(self.get(base + "?class_name=%20"), (400, error("class_name must contain 1 to 80 characters")))
        self.assertEqual(self.get(base + "?class_name=a&color=%20"), (400, error("color must contain 1 to 40 characters")))
        status, body = self.get(base + "?class_name=a&shelf_level=0")
        self.assertEqual(status, 400)
        self.assertIn("shelf_level must be an integer from 1 to", body["error"])  # type: ignore[index]
        self.assertEqual(self.get("/api/cameras/nope/target-observation?class_name=a"), (404, error("Camera not found")))

    def test_tasks(self) -> None:
        task_id = self.create_task()
        status, body = self.get("/api/tasks")
        self.assertEqual(status, 200)
        self.assertEqual([task["id"] for task in body["tasks"]], [task_id])  # type: ignore[index]
        status, body = self.get(f"/api/tasks/{task_id}")
        self.assertEqual(status, 200)
        self.assertEqual(body["task"]["id"], task_id)  # type: ignore[index]
        self.assertIn("local_execution_owner", body["task"])  # type: ignore[index]
        for path in ("/api/tasks/TASK-00000000", "/api/tasks/", f"/api/tasks/{task_id}/extra"):
            with self.subTest(path=path):
                self.assertEqual(self.get(path), (404, error("Task not found")))

    def test_observation_image_not_found(self) -> None:
        task_id = self.create_task()
        for path in (
            f"/api/tasks/{task_id}/observations/EVT-000000000000/image",
            f"/api/tasks/{task_id}/observations/bad/image",
            "/api/tasks/TASK-00000000/observations/EVT-000000000000/image",
            f"/api/tasks/{task_id}/x/observations/EVT-000000000000/image",
        ):
            with self.subTest(path=path):
                self.assertEqual(self.get(path), (404, error("Observation image not found")))

    def test_projection_artifact_not_found(self) -> None:
        task_id = self.create_task()
        for path in (
            f"/api/tasks/{task_id}/projection-drafts/EVT-000000000000/artifact",
            f"/api/tasks/{task_id}/projection-drafts/bad/artifact",
            "/api/tasks/TASK-00000000/projection-drafts/EVT-000000000000/artifact",
        ):
            with self.subTest(path=path):
                self.assertEqual(self.get(path), (404, error("Projection draft artifact not found")))

    def test_arm_read_routes(self) -> None:
        status, body = self.get("/api/arm/status")
        self.assertEqual(status, 200)
        self.assertIn("arm", body)
        self.assertEqual(self.get("/api/arm/commands")[0], 200)
        status, body = self.get("/api/arm/shelf-slots")
        self.assertEqual(status, 200)
        self.assertEqual(sorted(body), ["slot_map", "slot_map_version"])  # type: ignore[arg-type]
        self.assertEqual(self.get("/api/arm/commands/CMD-404"), (404, error("Arm command not found", "ARM_COMMAND_NOT_FOUND")))
        status, body, _ = self.request("GET", "/api/arm/arm-01/commands/next?wait_s=0&worker_id=w1")
        self.assertEqual(status, 204)
        self.assertEqual(self.get("/api/arm/nobody/commands/next?wait_s=0"), (404, error("Unknown arm robot_id", "ARM_NOT_FOUND")))
        status, body = self.get("/api/arm/arm-01/commands/next?wait_s=abc")
        self.assertEqual((status, body["error_code"]), (400, "ARM_COMMAND_INVALID"))  # type: ignore[index]

    def test_shelf_slots_not_configured(self) -> None:
        self.server.arm_commands = ArmCommandStore(self.task_store.session_factory, slot_map=None, slot_map_error="없음")
        self.assertEqual(self.get("/api/arm/shelf-slots"), (404, error("없음", "SHELF_SLOTS_NOT_CONFIGURED")))

    def test_parking_routes_without_parking(self) -> None:
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/live"), (404, error(
            "이 주차 위치는 실시간 추적 중이 아닙니다 (CONTROL_STATION_PARKING_SPOT)", "PARKING_TRACKING_NOT_CONFIGURED")))
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/check"), (503, error(
            "주차 판정 노드가 설정되지 않았습니다 (CONTROL_STATION_PARKING_SPOT)", "PARKING_NODE_UNAVAILABLE")))

    def test_streams_and_static_files(self) -> None:
        self.assertEqual(self.get("/streams/nope.mjpg"), (404, error("Camera stream not found")))
        self.assertEqual(self.get("/api/nope"), (404, error("API route not found")))
        status, body, headers = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn("text/html", headers["content-type"])
        status, body, headers = self.request("GET", "/app.js")
        self.assertEqual(status, 200)
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_address[1], timeout=10)
        connection.request("GET", "/streams/head_camera.mjpg")
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Content-Type"), "multipart/x-mixed-replace; boundary=frame")
        self.assertTrue(response.read(64).startswith(b"--frame\r\nContent-Type: image/jpeg"))
        connection.close()


class ParkingRouteTests(ApiTestCase):
    parking = True

    def test_health_with_parking(self) -> None:
        self.assertEqual(self.get("/api/health")[1]["parking"], {  # type: ignore[index]
            "source": "ros", "camera_id": "head_camera", "spot_id": "pinky_dock_1", "live_tracking": True})

    def test_live_and_check(self) -> None:
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/live"), (200, {"live": self.fake_parking.snapshot()}))
        self.assertEqual(self.get("/api/parking-spots/other/live")[0], 404)
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/check"), (200, {"observation": {
            "spot_id": "pinky_dock_1", "status": "ARRIVED", "reason": "within tolerance"}}))
        self.fake_parking.error = ParkingObservationError(503, "PARKING_NODE_TIMEOUT", "늦음")
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/check"), (503, error("늦음", "PARKING_NODE_TIMEOUT")))


class RequestBodyTests(ApiTestCase):
    def test_body_rules(self) -> None:
        self.assertEqual(self.post("/api/tasks", {"destination": "A"}, content_type="text/plain"),
                         (415, error("Content-Type must be application/json")))
        self.assertEqual(self.post("/api/tasks", raw=b"x" * 4097), (400, error("Request body must be between 1 and 4096 bytes")))
        self.assertEqual(self.post("/api/tasks", raw=b"{bad"), (400, error("Request body must be valid JSON with unique object keys")))
        self.assertEqual(self.post("/api/tasks", raw=b'{"a": 1, "a": 2}'),
                         (400, error("Request body must be valid JSON with unique object keys")))
        self.assertEqual(self.post("/api/tasks", raw=b"[1]"), (400, error("Request body must be a JSON object")))
        self.assertEqual(self.post("/api/nope", {"a": 1}), (404, error("API route not found")))
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_address[1], timeout=10)
        connection.request("POST", "/api/tasks", headers={"Content-Type": "application/json"})
        response = connection.getresponse()
        self.assertEqual((response.status, json.loads(response.read())), (400, error("Request body must be between 1 and 4096 bytes")))
        connection.close()


class TaskRouteTests(ApiTestCase):
    def test_create_task_validation(self) -> None:
        self.assertEqual(self.post("/api/tasks", {"destination": "A", "arm_job": "B", "x": 1, "a": 2}),
                         (400, error("Unsupported task fields: a, x")))
        self.assertEqual(self.post("/api/tasks", {"destination": 1, "arm_job": "B"}),
                         (400, error("destination and arm_job must be strings")))
        self.assertEqual(self.post("/api/tasks", {"destination": " ", "arm_job": "B"}),
                         (400, error("destination must contain 1 to 40 characters")))
        self.assertEqual(self.post("/api/tasks", {"destination": "A", "arm_job": "x" * 121}),
                         (400, error("arm_job must contain 1 to 120 characters")))
        self.assertEqual(self.post("/api/tasks", {"destination": "A", "arm_job": "B", "pick_request": {"color": "red"}}),
                         (400, error("pick_request.category must contain 1 to 80 characters without control characters")))
        status, body = self.post("/api/tasks", {"destination": " A ", "arm_job": " B ",
                                                "pick_request": {"category": " strawberry ", "shelf_level": 2}})
        self.assertEqual(status, 201)
        self.assertEqual((body["task"]["destination"], body["task"]["pick_request"]),  # type: ignore[index]
                         ("A", {"category": "strawberry", "shelf_level": 2}))

    def test_start_and_cancel(self) -> None:
        task_id = self.create_task()
        self.assertEqual(self.post(f"/api/tasks/{task_id}/start", {}, origin="http://evil.example"),
                         (403, error("Cross-origin task-control request rejected", "CROSS_ORIGIN_REQUEST_REJECTED")))
        self.assertEqual(self.post("/api/tasks/TASK-00000000/start", {}), (404, error("Task not found", "TASK_NOT_FOUND")))
        self.assertEqual(self.post("/api/tasks//start", {}), (404, error("Task not found", "TASK_NOT_FOUND")))
        status, body = self.post(f"/api/tasks/{task_id}/start", {})
        self.assertEqual(status, 202)
        self.assertEqual(body["execution_mode"], "mock")  # type: ignore[index]
        status, body = self.post(f"/api/tasks/{task_id}/cancel", {})
        self.assertIn(status, (200, 202))
        self.assertIn("task", body)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and self.get(f"/api/tasks/{task_id}")[1]["task"]["status"] not in {  # type: ignore[index]
            "SUCCEEDED", "FAILED", "CANCELED"}:
            time.sleep(0.05)
        status, body = self.post(f"/api/tasks/{task_id}/start", {})
        self.assertEqual(status, 409)
        self.assertIn("error_code", body)

    def test_parse_command(self) -> None:
        status, body = self.post("/api/tasks/parse-command", {"text": "빨간 딸기 2층"})
        self.assertEqual(status, 200)
        self.assertIn("draft", body)
        self.assertEqual(self.post("/api/tasks/parse-command", {"text": ""}), (400, error("text must contain 1 to 500 characters")))
        self.assertEqual(self.post("/api/tasks/parse-command", {"text": "a", "b": 1}), (400, error("text must contain 1 to 500 characters")))
        self.assertEqual(self.post("/api/tasks/parse-command", {"text": "a"}, origin="http://evil.example"),
                         (403, error("Cross-origin command-preview request rejected")))


class ArmRouteTests(ApiTestCase):
    def test_teleop_start_stop(self) -> None:
        status, body = self.post("/api/arm/start", {})
        self.assertEqual(status, 409)
        self.assertEqual(body["error"], "테스트에서는 팔을 쓰지 않습니다")  # type: ignore[index]
        self.assertIn("arm", body)
        self.assertEqual(self.post("/api/arm/stop", {}, origin="http://evil.example"),
                         (403, error("Cross-origin arm-control request rejected")))

    def test_arm_command_routes(self) -> None:
        self.assertEqual(self.post("/api/arm/commands", {"x": 1}, origin="http://evil.example"),
                         (403, error("Cross-origin arm-command request rejected", "CROSS_ORIGIN_REQUEST_REJECTED")))
        status, body = self.post("/api/arm/commands", {"x": 1})
        self.assertEqual((status, body["error_code"]), (400, "ARM_COMMAND_INVALID"))  # type: ignore[index]
        self.assertEqual(self.post("/api/arm/commands/CMD-404/cancel", {"a": 1}),
                         (400, error("cancel takes an empty JSON object", "ARM_COMMAND_INVALID")))
        self.assertEqual(self.post("/api/arm/commands/CMD-404/cancel", {}),
                         (404, error("Arm command not found", "ARM_COMMAND_NOT_FOUND")))
        self.assertEqual(self.post("/api/arm/commands/CMD-404/clear-unknown", {"a": 1}),
                         (400, error("clear-unknown takes exactly note", "ARM_COMMAND_INVALID")))

    def test_coordinate_target_drafts(self) -> None:
        self.assertEqual(self.post("/api/arm/coordinate-target-drafts", {"frame_id": "base"}),
                         (400, error("request must contain exactly frame_id, unit, and position")))
        status, body = self.post("/api/arm/coordinate-target-drafts",
                                 {"frame_id": "omx_link0", "unit": "mm", "position": {"x": 1, "y": 2, "z": 3}})
        self.assertEqual(status, 201)
        self.assertEqual(body["draft"]["position"], {"x": 1.0, "y": 2.0, "z": 3.0})  # type: ignore[index]
        self.assertEqual(self.post("/api/arm/coordinate-target-drafts", {"a": 1}, origin="http://evil.example"),
                         (403, error("Cross-origin coordinate draft request rejected")))


class ObservationFlowTests(ApiTestCase):
    """Target observation -> saved frame image -> operator selection, and their rejections."""

    def test_observation_request_errors(self) -> None:
        plain = self.create_task()
        pick = self.create_task({"category": "strawberry", "color": "red"})
        cases = [
            (pick, {"camera_id": "head_camera", "x": 1}, 400,
             error("Only camera_id and optional save_frame are accepted", "INVALID_TARGET_OBSERVATION_REQUEST")),
            (pick, {"camera_id": " "}, 400, error("task_id and camera_id are required", "INVALID_TARGET_OBSERVATION_REQUEST")),
            (pick, {"camera_id": "head_camera", "save_frame": 1}, 400,
             error("save_frame must be a boolean", "INVALID_TARGET_OBSERVATION_REQUEST")),
            ("TASK-00000000", {"camera_id": "head_camera"}, 404, error("Task not found", "TASK_NOT_FOUND")),
            (plain, {"camera_id": "head_camera"}, 409, error("Task has no structured pick request", "PICK_REQUEST_MISSING")),
            (pick, {"camera_id": "nope"}, 404, error("Camera not found", "CAMERA_NOT_FOUND")),
        ]
        for task_id, body, status, expected in cases:
            with self.subTest(body=body):
                self.assertEqual(self.post(f"/api/tasks/{task_id}/target-observation", body), (status, expected))
        self.assertEqual(self.post(f"/api/tasks/{pick}/target-observation", {"camera_id": "head_camera"},
                                   origin="http://evil.example"),
                         (403, error("Cross-origin task-observation request rejected", "CROSS_ORIGIN_REQUEST_REJECTED")))

    def test_observe_save_view_and_select(self) -> None:
        task_id = self.create_task({"category": "strawberry", "color": "red"})
        status, body = self.post(f"/api/tasks/{task_id}/target-observation", {"camera_id": "head_camera", "save_frame": True})
        self.assertEqual(status, 201, body)
        observation = body["observation"]  # type: ignore[index]
        self.assertEqual((observation["status"], observation["candidate_count"]), ("AMBIGUOUS", 2))
        self.assertEqual(observation["color_evaluation"]["candidate_indices"], [0])
        artifact = body["frame_artifact"]  # type: ignore[index]
        event = next(e for e in body["task"]["events"] if e["event_type"] == "TARGET_OBSERVED")  # type: ignore[index]

        status, image, headers = self.request("GET", f"/api/tasks/{task_id}/observations/{event['id']}/image")
        self.assertEqual(status, 200)
        self.assertEqual(headers["content-type"], "image/png")
        self.assertEqual(hashlib.sha256(image).hexdigest(), artifact["file_sha256"])  # type: ignore[arg-type]

        selection_path = f"/api/tasks/{task_id}/target-selection"
        self.assertEqual(self.post(selection_path, {"observation_event_id": event["id"], "candidate_index": -1}),
                         (400, error("observation_event_id and a non-negative candidate_index are required",
                                     "INVALID_TARGET_SELECTION_REQUEST")))
        self.assertEqual(self.post(selection_path, {"observation_event_id": "EVT-000000000000", "candidate_index": 0}),
                         (404, error("Target observation event not found", "OBSERVATION_EVENT_NOT_FOUND")))
        self.assertEqual(self.post(selection_path, {"observation_event_id": event["id"], "candidate_index": 5}),
                         (409, error("Candidate index is outside the recorded observation", "CANDIDATE_INDEX_OUT_OF_RANGE")))
        status, body = self.post(selection_path, {"observation_event_id": event["id"], "candidate_index": 0})
        self.assertEqual(status, 201, body)
        self.assertIs(body["already_recorded"], False)  # type: ignore[index]
        self.assertEqual(body["selection"]["candidate_index"], 0)  # type: ignore[index]
        self.assertEqual(body["selection"]["selection_scope"], "HUMAN_REVIEW_ONLY")  # type: ignore[index]
        status, again = self.post(selection_path, {"observation_event_id": event["id"], "candidate_index": 0})
        self.assertEqual((status, again["already_recorded"], again["selection_event_id"]),  # type: ignore[index]
                         (200, True, body["selection_event_id"]))  # type: ignore[index]

    def test_selection_needs_a_saved_frame(self) -> None:
        task_id = self.create_task({"category": "strawberry"})
        status, body = self.post(f"/api/tasks/{task_id}/target-observation", {"camera_id": "head_camera"})
        self.assertEqual(status, 201)
        self.assertIsNone(body["frame_artifact"])  # type: ignore[index]
        event = next(e for e in body["task"]["events"] if e["event_type"] == "TARGET_OBSERVED")  # type: ignore[index]
        self.assertEqual(self.post(f"/api/tasks/{task_id}/target-selection", {"observation_event_id": event["id"], "candidate_index": 0}),
                         (409, error("A saved, hash-verified source frame is required for operator selection",
                                     "SOURCE_FRAME_REQUIRED")))
        self.assertEqual(self.get(f"/api/tasks/{task_id}/observations/{event['id']}/image"),
                         (404, error("Observation image not found")))

    def test_selection_and_projection_request_errors(self) -> None:
        plain = self.create_task()
        pick = self.create_task({"category": "strawberry"})
        self.assertEqual(self.post(f"/api/tasks/{plain}/target-selection", {"observation_event_id": "EVT-000000000000", "candidate_index": 0}),
                         (409, error("Task has no structured pick request", "PICK_REQUEST_MISSING")))
        self.assertEqual(self.post("/api/tasks/TASK-00000000/target-selection", {"observation_event_id": "EVT-000000000000", "candidate_index": 0}),
                         (404, error("Task not found", "TASK_NOT_FOUND")))
        self.assertEqual(self.post(f"/api/tasks/{pick}/target-selection", {"a": 1}, origin="http://evil.example"),
                         (403, error("Cross-origin target-selection request rejected", "CROSS_ORIGIN_REQUEST_REJECTED")))
        projection = f"/api/tasks/{pick}/projection-draft"
        self.assertEqual(self.post(projection, {"artifact_path": "x"}),
                         (400, error("artifact_path and artifact_sha256 are required", "INVALID_PROJECTION_DRAFT_REQUEST")))
        self.assertEqual(self.post("/api/tasks/TASK-00000000/projection-draft", {"artifact_path": "x", "artifact_sha256": "y"}),
                         (404, error("Task not found", "TASK_NOT_FOUND")))
        status, body = self.post(projection, {"artifact_path": "x", "artifact_sha256": "y"})
        self.assertEqual((status, body["error_code"]), (409, "PROJECTION_DRAFT_REJECTED"))  # type: ignore[index]
        self.assertEqual(self.post(projection, {"a": 1}, origin="http://evil.example"),
                         (403, error("Cross-origin projection-draft request rejected", "CROSS_ORIGIN_REQUEST_REJECTED")))


if __name__ == "__main__":
    unittest.main()
