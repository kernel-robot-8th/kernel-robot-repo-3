"""관제 HTTP 서버: 서버 조립(main)과 요청 분배"""

from __future__ import annotations

import json
import os
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from control_station.api import routes
from control_station.api.common import is_cross_origin, read_json_object
from control_station.api.observations import evaluate_color_hints, target_observation
from control_station.act_policies import ActPolicyError, load_act_policies
from control_station.arm_commands import ARM_ROBOT_IDS, ArmCommandStore
from control_station.arms import ArmConfigError, load_arms
from control_station.arm_supervisor import ArmTeleopSupervisor
from control_station.policy_run_supervisor import PolicyRunSupervisor
from control_station.camera_counts import ShelfStockCounter, load_counter_from_env
from control_station.camera_adapters import (
    OpenCVArmCameraStreamManager,
    UnconfiguredArmCameraStreamManager,
)
from control_station.load_check import mock_load_observer_from_env
from control_station.mock_world import MockWorld
from control_station.order_flow import OrderDevices, OrderFlow
from control_station.pinky_fleet import PinkyFleetClient
from control_station.parking_ros import RosParkingSource
from control_station.robot_adapters import make_status_provider
from control_station.shelf_slot_map import ShelfSlotMapError, load_shelf_slot_map
from control_station.shelf_slot_zones import ShelfSlotZones, SlotSelectionError, select_slot_for_class
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


def front_camera_evidence(server: ControlStationServer, object_class: str) -> dict[str, Any]:
    """정면 카메라가 물체를 칸 하나에서만 볼 때 그 관측 기록 (작업 흐름 팔 단계의 SR-13 근거).

    수동 "정면 캠으로 칸 찾기"(POST /api/arm/commands/from-camera)와 같은 판정을 쓴다."""
    camera_id = os.environ.get("CONTROL_STATION_PICK_EVIDENCE_CAMERA", "shelf_camera").strip() or "shelf_camera"
    snapshot = next((camera for camera in server.camera_manager.snapshots() if camera.id == camera_id), None)
    if snapshot is None:
        raise SlotSelectionError("CAMERA_NOT_FOUND", f"정면 카메라 {camera_id!r}가 설정되지 않았습니다")
    return select_slot_for_class(server.shelf_slot_zones.annotate(snapshot.to_dict()), object_class)


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
    server.arm_supervisor = ArmTeleopSupervisor(disabled_reason=None)
    # One-shot LeRobot policy client started from the control station (POST /api/arm/policy/start).
    server.policy_run_supervisor = PolicyRunSupervisor()
    server.arm_control_lock = threading.RLock()
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
    front_camera_configured = any(
        os.environ.get(name, "").strip()
        for name in (
            "CONTROL_STATION_FRONT_CAMERA",
            "CONTROL_STATION_POLICY_FRONT_CAMERA",
            "CONTROL_STATION_FRONT_YOLO_WEIGHTS",
            "CONTROL_STATION_FRONT_YOLO_WORLD_PROMPTS",
        )
    )
    if not (
        server.arm_supervisor.enabled
        or head_detector_configured
        or front_camera_configured
        or server.parking_camera_id
        or os.environ.get("CONTROL_STATION_SHELF_CAMERA", "").strip()
        or os.environ.get("CONTROL_STATION_HEAD_CAMERA", "").strip().startswith("ros:")
    ):
        server.camera_manager = UnconfiguredArmCameraStreamManager()
    else:
        server.camera_manager = OpenCVArmCameraStreamManager(
            joint_enabled=server.arm_supervisor.enabled,
            policy_active=lambda: bool(server.policy_run_supervisor.snapshot()["process_alive"]),
        )
    server.task_store = task_store
    try:
        slot_map, slot_map_error = load_shelf_slot_map(), None
    except ShelfSlotMapError as exc:
        # Other commands still work; only pick_slot_id commands are refused.
        slot_map, slot_map_error = None, str(exc)
        print(f"칸 정의를 불러오지 못했습니다: {exc}", flush=True)
    try:
        act_policies, act_policies_error = load_act_policies(), None
    except ActPolicyError as exc:
        # Every other command still works; only PICK_WITH_POLICY is refused.
        act_policies, act_policies_error = None, str(exc)
        print(f"정책 정의를 불러오지 못했습니다: {exc}", flush=True)
    server.arm_commands = ArmCommandStore(
        task_store.session_factory, slot_map=slot_map, slot_map_error=slot_map_error, act_policies=act_policies
    )
    server.act_policies_error = act_policies_error
    # Which arm serves which shelf (left shelf -> left arm); orders need it to plan the arm for each object.
    try:
        server.arms, server.arms_error = load_arms(), None
        unknown = server.arms.arm_ids - ARM_ROBOT_IDS
        if unknown:
            raise ArmConfigError(f"알 수 없는 팔 ID: {sorted(unknown)}")
    except ArmConfigError as exc:
        server.arms, server.arms_error = None, str(exc)
        print(f"팔 설정을 불러오지 못했습니다: {exc}", flush=True)
    # Cameras the LeRobot policy opens itself; closed by the control station while a policy command runs.
    server.policy_camera_devices = [
        device
        for device in (
            os.environ.get("CONTROL_STATION_ACT_OVERHEAD_CAMERA", "").strip(),
            os.environ.get("CONTROL_STATION_JOINT_CAMERA", "").strip(),
        )
        if device
    ]
    server.policy_camera_lock = threading.Lock()
    server.policy_cameras_lent = False
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
    # Opt-in: EXECUTE_ARM_TASK queues a real PICK_WITH_POLICY arm command (CONTROL_STATION_TASK_REAL_ARM=1).
    real_arm = os.environ.get("CONTROL_STATION_TASK_REAL_ARM", "").strip() == "1"
    # Orders run against the simulated station until the camera counters and the Pinky fleet client are connected;
    # every observation it makes says simulated.
    server.mock_world = MockWorld.from_env()
    world = server.mock_world
    # Pinky arrival for orders: the real ceiling-camera judgment when a parking spot is configured (one registered
    # spot, the loading spot between the two arms), else the mock. CONTROL_STATION_ORDER_PARKING=mock forces the mock.
    order_parking = world.parking
    if server.parking_check is not None and os.environ.get("CONTROL_STATION_ORDER_PARKING", "").strip() != "mock":
        def order_parking(pinky_id: str) -> dict[str, Any]:
            return server.parking_check(parking_spot_id)
        print(f"주문 흐름의 핑키 도착 판정: 실제 주차 판정 ({parking_spot_id})", flush=True)
    # Pinky mission phases and the dock_complete / received signals: the Pinky team's fleet dashboard when its URL
    # is set (e.g. http://<pinky PC>:8765), else the mock world, which also sends the mock Pinky off for each order.
    fleet_url = os.environ.get("CONTROL_STATION_PINKY_FLEET_URL", "").strip()
    fleet_status, fleet_confirm, dispatch = world.fleet_status, world.fleet_confirm, world.dispatch
    if fleet_url:
        fleet = PinkyFleetClient(fleet_url)
        fleet_status, fleet_confirm, dispatch = fleet.fleet_status, fleet.fleet_confirm, None
        print(f"주문 흐름의 핑키 신호: 핑키 관제 {fleet_url}", flush=True)
    # Counting for the stock check and the per-object transfer check: the cameras' YOLO detections when
    # CONTROL_STATION_ORDER_COUNTS=camera (shelf cams with slot zones, the top cam inside the Pinky tray), else the
    # mock world. With camera counts but mock picks, a person has to move each object during the pick step.
    shelf_stock, pinky_load = world.shelf_stock, world.pinky_load
    if os.environ.get("CONTROL_STATION_ORDER_COUNTS", "").strip() == "camera":
        shelf_stock = ShelfStockCounter(server.camera_manager.snapshots, server.shelf_slot_zones, slot_map, server.arms)
        load_counter = load_counter_from_env(server.camera_manager.snapshots, Path(__file__).resolve().parents[2],
                                             parking_spot_id)
        pinky_load = load_counter
        print(f"주문 흐름의 개수 세기: 카메라 YOLO (핑키 쟁반 {load_counter.camera_id} {load_counter.region_or_none()})",
              flush=True)
    server.order_devices = OrderDevices(
        shelf_stock, pinky_load, fleet_status, fleet_confirm, order_parking, world.pick, dispatch,
    )
    try:
        arrival_timeout_s = float(os.environ.get("CONTROL_STATION_ARRIVAL_TIMEOUT_S", "600"))
    except ValueError:
        arrival_timeout_s = 600.0
    server.order_flow = OrderFlow(
        server.arms, act_policies, server.order_devices,
        dwell_s=float(os.environ.get("CONTROL_STATION_ORDER_STEP_DWELL_S", "0.8") or 0.8),
        arrival_timeout_s=arrival_timeout_s,
    )
    server.task_orchestrator = MockTaskOrchestrator(
        task_store, mode=mode, arrival_check=arrival_check, load_check=load_check,
        arm_commands=server.arm_commands if real_arm else None,
        target_evidence=partial(front_camera_evidence, server) if real_arm else None,
        order_flow=server.order_flow,
    )
    if real_arm:
        print("작업 흐름의 팔 단계가 실제 팔 명령(PICK_WITH_POLICY)을 보냅니다", flush=True)
    mode_label = {"mock": "모의", "ros2": "ROS 2"}[mode]
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
        server.policy_run_supervisor.close()
        server.task_orchestrator.close()
        task_store.close()
        provider.close()


if __name__ == "__main__":
    main()
