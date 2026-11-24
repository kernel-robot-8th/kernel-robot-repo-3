"""여러 테스트가 같이 쓰는 도우미.

- ApiTestCase: 가짜 카메라·주차·로봇 상태로 관제 서버를 이 프로세스 안에 띄우고 HTTP로 요청하는 테스트의 바탕.
  장치를 열거나 로봇에 명령을 보내지 않는다.
- devices, make_desk, finish: 모의 세계로 주문 흐름과 주문 창구를 만들고 주문 하나를 끝까지 돌리는 도우미.
"""

from __future__ import annotations

import http.client
import json
import os
import tempfile
import threading
import time
import unittest
from functools import partial
from pathlib import Path
from unittest import mock

from control_station.app import WEB_DIR, ControlStationHandler, ControlStationServer
from control_station.arms.commands import ArmCommandStore
from control_station.arms.config import load_arms
from control_station.arms.policies import load_act_policies
from control_station.arms.policy_runner import PolicyRunSupervisor
from control_station.cameras.shelf_slots import ShelfSlotZones, load_shelf_slot_map
from control_station.mock_world import MockWorld
from control_station.orders.desk import OrderDesk
from control_station.orders.flow import OrderDevices, OrderFlow
from control_station.orders.receiving_zones import ReceivingZone
from control_station.orders.task_runner import MockTaskOrchestrator
from control_station.pinky.parking_ros import ParkingObservationError
from control_station.storage.models import CameraConnectionState, CameraSnapshot, CameraStreamState, FrameFreshness
from control_station.storage.store import TaskStore

# ── 관제 서버를 띄우는 HTTP 테스트 ──

FRAME_SHA = "a" * 64
CAPTURED_AT = "2026-11-03T12:00:00.000+00:00"
PIXEL_TRANSFORM = "opencv-v4l2-capture-no-app-geometry-v1"


def detection(class_name: str, x: int) -> dict[str, object]:
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
        detections=[detection("strawberry", 100), detection("strawberry", 300)],
    )
    values.update(changes)
    return CameraSnapshot(**values)  # type: ignore[arg-type]


class FakeProvider:
    def snapshots(self) -> list[dict[str, object]]:
        return [{"id": "pinky-01", "kind": "MOBILE"}]

    def close(self) -> None:
        pass


class FakeCameraManager:
    """Snapshots and a JPEG stream."""

    def __init__(self, cameras: list[CameraSnapshot]) -> None:
        self.cameras = cameras

    def snapshots(self) -> list[CameraSnapshot]:
        return list(self.cameras)

    def frame_for(self, camera_id: str) -> bytes | None:
        return b"\xff\xd8jpeg\xff\xd9" if any(c.id == camera_id for c in self.cameras) else None

    def sequence_for(self, camera_id: str) -> int:
        return 1

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
        self.fake_parking = FakeParking()
        slot_map = load_shelf_slot_map()
        server.mode = "mock"
        server.robot_provider = FakeProvider()
        server.policy_run_supervisor = PolicyRunSupervisor(disabled_reason="테스트에서는 정책을 실행하지 않습니다")
        server.arm_control_lock = threading.RLock()
        server.camera_manager = self.cameras
        server.task_store = self.task_store
        server.arm_commands = ArmCommandStore(self.task_store.session_factory, slot_map=slot_map)
        server.shelf_slot_zones = ShelfSlotZones(slot_map, directory=self.directory / "poses")
        server.parking_spot_id = "pinky_dock_1" if self.parking else None
        server.parking_camera_id = "head_camera" if self.parking else None
        server.parking_check = self.fake_parking.check if self.parking else None
        server.parking_tracker = self.fake_parking if self.parking else None
        server.task_orchestrator = MockTaskOrchestrator(self.task_store)
        self.addCleanup(server.task_orchestrator.close)
        self.server = server
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)

    def request(
        self,
        method: str,
        path: str,
        body: object = None,
        *,
        raw: bytes | None = None,
        content_type: str | None = "application/json",
        origin: str | None = None,
        headers: dict[str, str] | None = None,
    ) -> tuple[int, object, dict[str, str]]:
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_address[1], timeout=10)
        headers = dict(headers or {})
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
        status, body, headers = self.request("GET", path)
        return status, self._without_request_id(body, headers)

    def post(self, path: str, body: object = None, **options: object) -> tuple[int, object]:
        status, data, headers = self.request("POST", path, body, **options)  # type: ignore[arg-type]
        return status, self._without_request_id(data, headers)

    def _without_request_id(self, body: object, headers: dict[str, str]) -> object:
        """Error bodies carry the request's random ID; check it matches X-Request-Id, then compare the rest."""
        if isinstance(body, dict) and "request_id" in body:
            body = dict(body)
            self.assertEqual(body.pop("request_id"), headers.get("x-request-id"))
        return body

    def create_task(self, pick_request: dict[str, object] | None = None) -> str:
        body: dict[str, object] = {"destination": "작업대 A", "arm_job": "상자 이동"}
        if pick_request is not None:
            body["pick_request"] = pick_request
        status, data = self.post("/api/tasks", body)
        self.assertEqual(status, 201, data)
        return data["task"]["id"]  # type: ignore[index]


def error(message: str, code: str | None = None, *, retryable: bool = False, **details: object) -> dict[str, object]:
    if not code:
        return {"error": message}
    return {"error_code": code, "message": message, "error": message, "retryable": retryable, **details}


# ── 주문 흐름과 주문 창구 ──

TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED"}
CARROT_APPLE = [{"object_class": "carrot", "quantity": 1}, {"object_class": "apple", "quantity": 2}]


def devices(world: MockWorld) -> OrderDevices:
    return OrderDevices(world.shelf_stock, world.pinky_load, world.fleet_status, world.fleet_confirm, world.parking,
                        world.pick, world.dispatch, world.mission_complete)

ZONES = {"zone_a": ReceivingZone("zone_a", "수령 구역 A", ("1", "5", "9"))}


def finish(desk: OrderDesk, store: TaskStore, order_id: str, *, pickup: bool = True) -> dict:
    """Run an order to its end, confirming pickup when its Pinky waits at the receiving zone."""
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        order = store.get_task(order_id)
        if order["status"] in TERMINAL:
            return order
        active = next((step for step in order["steps"] if step["id"] == order.get("active_step_id")), None)
        if pickup and active is not None and active["step_type"] == "CONFIRM_PICKUP" and desk.flow_waiting(order_id):
            desk.confirm_pickup(order_id)
        time.sleep(0.02)
    raise AssertionError(f"order did not finish: {store.get_task(order_id)['status']}")


def make_desk(store: TaskStore, *, zones=ZONES) -> tuple[OrderDesk, MockWorld, MockTaskOrchestrator]:
    world = MockWorld(drive_s=0.2, pick_s=0.05)
    flow = OrderFlow(load_arms(), load_act_policies(), devices(world), dwell_s=0.0, poll_s=0.02,
                     arrival_timeout_s=10.0, signal_timeout_s=2.0,
                     route_for=lambda zone_id: list(zones[zone_id].route_labels) if zone_id in zones else None)
    orchestrator = MockTaskOrchestrator(store, order_flow=flow)
    desk = OrderDesk(store, orchestrator, devices(world), load_arms(), load_act_policies(), zones,
                     pick_stats=lambda: store.pick_success_stats(simulated=True), pickup=flow.confirm_pickup)
    flow.choose_shipment_pinky = desk.assign_next_shipment  # as app.py wires it
    desk.flow_waiting = lambda order_id: order_id in flow._pickups  # test helper: is the step waiting yet
    return desk, world, orchestrator
