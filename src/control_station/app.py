"""관제 HTTP 서버: 서버 조립(main)과 요청 분배"""

from __future__ import annotations

import json
import os
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from control_station.api import routes
from control_station.api.common import is_cross_origin, read_json_object
from control_station.api.observations import evaluate_color_hints, target_observation
from control_station.arm_commands import ArmCommandStore
from control_station.arm_supervisor import ArmTeleopSupervisor
from control_station.camera_adapters import (
    OpenCVArmCameraStreamManager,
    Ros2SimulationCameraStreamManager,
    UnconfiguredArmCameraStreamManager,
)
from control_station.coordinate_target_drafts import CoordinateTargetDraftStore
from control_station.load_check import mock_load_observer_from_env
from control_station.parking_ros import RosParkingSource
from control_station.robot_adapters import make_status_provider
from control_station.shelf_slot_map import ShelfSlotMapError, load_shelf_slot_map
from control_station.shelf_slot_zones import ShelfSlotZones
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator


WEB_DIR = Path(__file__).with_name("web")


class ControlStationServer(ThreadingHTTPServer):
    """관제 HTTP 서버 (요청마다 스레드)"""
    daemon_threads = True
    allow_reuse_address = True


class ControlStationHandler(SimpleHTTPRequestHandler):
    """요청을 api/routes.py 경로 표대로 처리 함수에 넘김"""
    server_version = "ControlStation/0.2"

    # Kept on the handler for callers that classify observations directly.
    _target_observation = staticmethod(target_observation)
    _evaluate_color_hints = staticmethod(evaluate_color_hints)

    @property
    def app(self) -> ControlStationServer:
        return self.server  # type: ignore[return-value]

    def log_message(self, format: str, *args: object) -> None:
        super().log_message(format, *args)

    def _json(self, status: int, body: dict[str, Any]) -> None:
        """JSON 응답"""
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
        """오류 응답 (error_code, message, error)"""
        body: dict[str, Any] = {
            "error_code": error_code,
            "message": message,
            "error": message,
        }
        body.update(details)
        self._json(status, body)

    def _image(self, payload: bytes) -> None:
        """PNG 응답"""
        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        """GET 분배, 맞는 경로가 없으면 웹 화면 파일"""
        route = urlsplit(self.path).path
        parts = route.strip("/").split("/")
        entry = routes.find(routes.GET_ROUTES, route, parts)
        if entry is not None:
            entry.handle(self, route, parts, None)
            return
        if route == "/":
            self.path = "/index.html"
        elif route.startswith("/api/"):
            self._json(404, {"error": "API route not found"})
            return
        super().do_GET()

    def do_POST(self) -> None:
        """POST 분배: 다른 출처 거부, JSON 본문 읽기"""
        route = urlsplit(self.path).path
        parts = route.strip("/").split("/")
        entry = routes.find(routes.POST_ROUTES, route, parts)
        if entry is None:
            self._json(404, {"error": "API route not found"})
            return
        if entry.cross_origin_message is not None and is_cross_origin(self):
            if entry.cross_origin_error_code:
                self._json_error(403, "CROSS_ORIGIN_REQUEST_REJECTED", entry.cross_origin_message)
            else:
                self._json(403, {"error": entry.cross_origin_message})
            return
        body = None
        if entry.reads_body:
            body = read_json_object(self)
            if body is None:
                return
        entry.handle(self, route, parts, body)


def main() -> None:
    """관제 서버 실행"""
    mode = os.environ.get("CONTROL_STATION_MODE", "mock").strip().lower()
    provider = make_status_provider(mode)
    task_store = TaskStore()
    # Default stays local. Set e.g. 0.0.0.0 only on a trusted LAN so an arm worker on another
    # machine can reach the API; there is no authentication.
    host = os.environ.get("CONTROL_STATION_HOST", "127.0.0.1").strip() or "127.0.0.1"
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
    # The C++ parking node (cpp/parking_vision_ros) judges Pinky's parking; the control station has no judgment of
    # its own. Setting a spot gates VERIFY_ARRIVAL on a fresh check from the node and follows its live state.
    parking_spot_id = os.environ.get("CONTROL_STATION_PARKING_SPOT", "").strip() or None
    server.parking_spot_id = parking_spot_id
    if os.environ.get("CONTROL_STATION_PARKING_SOURCE", "ros").strip().lower() not in {"", "ros"}:
        raise RuntimeError("CONTROL_STATION_PARKING_SOURCE는 ros만 지원합니다 (관제 안의 Python 주차 판정은 제거됨).")
    server.parking_source = "ros" if parking_spot_id else None
    # The feed the parking panel streams: the Head Camera shows the ceiling view, taken from the driver topic.
    server.parking_camera_id = os.environ.get("CONTROL_STATION_PARKING_CAMERA", "").strip() or (
        "head_camera" if parking_spot_id else None
    )
    if mode == "mujoco":
        raw_domain = os.environ.get("CONTROL_STATION_MOBILE_DOMAIN", os.environ.get("ROS_DOMAIN_ID"))
        if raw_domain is None:
            raise RuntimeError("MuJoCo 카메라 구독에 ROS domain ID를 지정해야 합니다.")
        server.camera_manager = Ros2SimulationCameraStreamManager(int(raw_domain))
    elif not (
        server.arm_supervisor.enabled
        or head_detector_configured
        or server.parking_camera_id
        or os.environ.get("CONTROL_STATION_SHELF_CAMERA", "").strip()
        or os.environ.get("CONTROL_STATION_HEAD_CAMERA", "").strip().startswith("ros:")
    ):
        server.camera_manager = UnconfiguredArmCameraStreamManager()
    else:
        server.camera_manager = OpenCVArmCameraStreamManager(
            joint_enabled=server.arm_supervisor.enabled
        )
    server.task_store = task_store
    try:
        slot_map, slot_map_error = load_shelf_slot_map(), None
    except ShelfSlotMapError as exc:
        # Position commands still work; only pick_slot_id commands are refused.
        slot_map, slot_map_error = None, str(exc)
        print(f"칸 정의를 불러오지 못했습니다: {exc}", flush=True)
    server.arm_commands = ArmCommandStore(
        task_store.session_factory, slot_map=slot_map, slot_map_error=slot_map_error
    )
    server.shelf_slot_zones = ShelfSlotZones(slot_map)
    server.parking_check = None
    server.parking_tracker = None
    if parking_spot_id:
        raw_domain = os.environ.get("CONTROL_STATION_MOBILE_DOMAIN", os.environ.get("ROS_DOMAIN_ID"))
        if raw_domain is None:
            raise RuntimeError(
                "CONTROL_STATION_PARKING_SPOT에는 주차 판정 노드의 ROS domain ID"
                "(CONTROL_STATION_MOBILE_DOMAIN 또는 ROS_DOMAIN_ID)를 지정해야 합니다."
            )
        parking_node = RosParkingSource(int(raw_domain), parking_spot_id, server.parking_camera_id)
        server.parking_check = parking_node.check
        # Display-only live state of the same spot; VERIFY_ARRIVAL still decides only from a fresh check.
        server.parking_tracker = parking_node
    arrival_check = partial(server.parking_check, parking_spot_id) if parking_spot_id else None
    # Mock mode only: VERIFY_LOAD then records a simulated count instead of a plain mock success.
    load_check = mock_load_observer_from_env() if mode == "mock" else None
    server.task_orchestrator = MockTaskOrchestrator(
        task_store, mode=mode, arrival_check=arrival_check, load_check=load_check
    )
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
        if server.parking_tracker is not None:
            server.parking_tracker.close()
        server.camera_manager.close()
        server.arm_supervisor.close()
        server.task_orchestrator.close()
        task_store.close()
        provider.close()


if __name__ == "__main__":
    main()
