"""HTTP behaviour of every control-station route: status codes, error codes and response fields.

Written before splitting app.py so the refactor keeps the API as it is. Uses an in-process server on an ephemeral
port with fake robot, camera and parking sources; nothing here opens a device or sends a robot command.
"""

from __future__ import annotations

import http.client
import json
import time
import unittest

from control_station.arms.commands import ArmCommandStore
from control_station.pinky.parking_ros import ParkingObservationError

from support import ApiTestCase, error, head_camera


class ReadRouteTests(ApiTestCase):
    def test_health_without_parking(self) -> None:
        self.assertEqual(self.get("/api/health"), (200, {
            "status": "ok", "mode": "mock",
            "parking": {"camera_id": None, "spot_id": None, "live_tracking": False, "live_spots": []},
        }))

    def test_robots(self) -> None:
        self.assertEqual(self.get("/api/robots"), (200, {"robots": [{"id": "pinky-01", "kind": "MOBILE"}]}))

    def test_cameras(self) -> None:
        # No shelf pose is activated for this camera, so the snapshot passes through without slot_zones.
        self.assertEqual(self.get("/api/cameras"), (200, {"cameras": [json.loads(json.dumps(head_camera().to_dict()))]}))

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

    def test_arm_read_routes(self) -> None:
        self.assertEqual(self.get("/api/arm/status")[0], 404)
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
            "이 주차 위치는 실시간 추적 중이 아닙니다 (CONTROL_STATION_PARKING_SPOT, config/pinky-markers.json)",
            "PARKING_TRACKING_NOT_CONFIGURED")))
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/check"), (503, error(
            "주차 판정 노드가 설정되지 않았습니다 (CONTROL_STATION_PARKING_SPOT)", "PARKING_NODE_UNAVAILABLE", retryable=True)))

    def test_streams_and_static_files(self) -> None:
        self.assertEqual(self.get("/streams/nope.mjpg"), (404, error("Camera stream not found")))
        self.assertEqual(self.get("/api/nope"), (404, error("API route not found")))
        status, body, headers = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn("text/html", headers["content-type"])
        status, body, headers = self.request("GET", "/js/app.js")
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
            "camera_id": "head_camera", "spot_id": "pinky_dock_1", "live_tracking": True,
            "live_spots": ["pinky_dock_1"]})

    def test_live_and_check(self) -> None:
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/live"), (200, {"live": self.fake_parking.snapshot()}))
        self.assertEqual(self.get("/api/parking-spots/other/live")[0], 404)
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/check"), (200, {"observation": {
            "spot_id": "pinky_dock_1", "status": "ARRIVED", "reason": "within tolerance"}}))
        self.fake_parking.error = ParkingObservationError(503, "PARKING_NODE_TIMEOUT", "늦음")
        self.assertEqual(self.get("/api/parking-spots/pinky_dock_1/check"), (503, error("늦음", "PARKING_NODE_TIMEOUT", retryable=True)))


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
    def test_teleop_routes_removed(self) -> None:
        self.assertEqual(self.post("/api/arm/start", {})[0], 404)
        self.assertEqual(self.post("/api/arm/stop", {})[0], 404)

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


if __name__ == "__main__":
    unittest.main()
