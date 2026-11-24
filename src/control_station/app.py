"""관제 HTTP 서버: 서버 조립(main)과 요청 분배. 실행: python -m control_station.app (보통 ./run-control-station.sh).

요청 분배: ControlStationHandler가 api/routes.py의 경로 표에서 맞는 처리 함수를 찾아 부른다. 맞는 GET 경로가 없으면
web/의 정적 파일(관제 화면, 주문 창)을 내준다. 서버는 요청마다 스레드를 쓰고(ThreadingHTTPServer), 모든 구성 요소는
서버 객체의 속성으로 붙어 있어 처리 함수가 h.app.<이름>으로 꺼내 쓴다. 인증은 없으므로 기본 주소는 127.0.0.1이다.

서버 조립(main)은 환경 변수(.control-station/control-station.env)를 읽어 다음 순서로 구성 요소를 만든다.
  1) 기본: 실행 모드(mock/ros2), 로봇 상태 제공자, 작업 저장소(SQLite), 주소, 정책 실행기
  2) 카메라: 설정된 카메라가 있으면 OpenCV/ROS 카메라 관리자, 없으면 빈 관리자
  3) 팔: 칸 정의·학습 정책 파일, 팔 명령 큐, 연속 실행, 팔 설정(config/arms.json), 정책이 쓰는 카메라 목록
  4) 핑키 마커와 주차 판정 노드(C++ 노드의 ROS 결과 구독)
  5) 주문 흐름의 핑키 쪽 장치: 모의 세계, 핑키 도착 판정, 핑키 관제 대시보드 클라이언트, 핑키 화면 게이트웨이
  6) 적재 자리 진입 안내(천장 캠 실시간 offset 전달)와 도착 자동 처리(도킹완료·다시 맞추기)
  7) 개수 세기: 선반 재고·핑키 적재를 카메라 YOLO로 셀지 모의 세계로 셀지
  8) 주문 집기: 모의 집기, 실제 팔 HTTP 큐, 실제 팔 ROS 2 액션 중 하나
  9) 주문 흐름(OrderFlow), 작업 실행기, 재고 감시, 주문 데스크, 자동 출발
  10) 서버 실행과 종료 정리

설정 파일을 읽지 못한 부분은 대부분 서버를 멈추지 않고 그 기능만 끈 채로 시작한다(오류를 출력하고 화면에 표시).
실제 장치가 설정되지 않은 부분은 모의 세계(mock_world.py)가 대신하고, 그 관측은 simulated로 표시된다.
"""

from __future__ import annotations

import json
import os
import threading
import uuid
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from control_station import REPO_ROOT, env_flag
from control_station.api import routes
from control_station.api.common import is_cross_origin, is_retryable, read_json_object
from control_station.arms.commands import ARM_ROBOT_IDS, ArmCommandStore
from control_station.arms.config import ArmConfigError, load_arms
from control_station.arms.pick import ArmActionPick, ArmQueuePick
from control_station.arms.policies import ActPolicyError, load_act_policies
from control_station.arms.policy_runner import PolicyRunSupervisor
from control_station.arms.ros_action import RosPickTransport
from control_station.arms.sequences import ArmSequenceSupervisor
from control_station.cameras.adapters import OpenCVArmCameraStreamManager, UnconfiguredArmCameraStreamManager
from control_station.cameras.counts import (
    ShelfStockCounter,
    camera_inventory,
    head_middle_view_from_env,
    head_shelf_view_from_env,
    load_counter_from_env,
    parse_region,
)
from control_station.cameras.shelf_slots import (
    ShelfSlotMapError,
    ShelfSlotZones,
    SlotSelectionError,
    load_shelf_slot_map,
    select_slot_for_class,
)
from control_station.mock_world import MockWorld
from control_station.orders.desk import OrderDesk
from control_station.orders.dispatcher import OrderDispatcher
from control_station.orders.flow import OrderDevices, OrderFlow
from control_station.orders.inventory import InventoryMonitor, InventoryOverrides
from control_station.orders.receiving_zones import ReceivingZoneError, load_receiving_zones
from control_station.orders.task_runner import MockTaskOrchestrator, mock_load_observer_from_env
from control_station.pinky.docking import DockAutoLoop, DockGuideRelay, DockSupervisor
from control_station.pinky.fleet_client import PinkyFleetClient
from control_station.pinky.gateway import PinkyGateway
from control_station.pinky.markers import PinkyMarkerError, arrival_for, load_pinky_markers
from control_station.pinky.parking_ros import RosParkingSource
from control_station.pinky.robot_status import make_status_provider
from control_station.storage.store import PINKY_NAMES, TaskStore

# 관제 화면·주문 창 정적 파일 폴더.
WEB_DIR = Path(__file__).with_name("web")


class ControlStationServer(ThreadingHTTPServer):
    """관제 HTTP 서버 (요청마다 스레드). main()이 구성 요소를 이 객체의 속성으로 붙인다.

    daemon_threads: 서버를 끌 때 MJPEG 스트림·long-poll처럼 오래 걸린 요청 스레드를 기다리지 않는다.
    allow_reuse_address: 다시 켤 때 이전 소켓의 TIME_WAIT 때문에 포트를 못 여는 일을 막는다."""
    daemon_threads = True
    allow_reuse_address = True


class ControlStationHandler(SimpleHTTPRequestHandler):
    """요청을 api/routes.py 경로 표대로 처리 함수에 넘김"""
    server_version = "ControlStation/0.2"

    @property
    def app(self) -> ControlStationServer:
        """구성 요소가 붙은 서버 객체 (처리 함수는 h.app.<이름>으로 쓴다)."""
        return self.server  # type: ignore[return-value]

    @property
    def request_id(self) -> str:
        """이 요청의 ID: 응답 머리글 X-Request-Id와 오류 본문 request_id로 돌려줘 서버 기록과 맞춰 볼 수 있게"""
        if getattr(self, "_request_id", None) is None:
            self._request_id = uuid.uuid4().hex[:12]
        return self._request_id

    def end_headers(self) -> None:
        # 관제 화면은 그냥 정적 파일로 내준다. 로컬 화면을 고쳐 가며 쓰는 동안, 운영자가 새로고침해도 브라우저가 예전
        # app.js/index.html을 계속 쓰는 일을 막는다(예전 index.html에 ROS 모니터 메뉴가 없던 일, 2026-11-16).
        # 예전의 "no-cache" 버전 메서드는 이 메서드에 가려져 쓰이지 않아 지웠다.
        # API와 스트림 응답은 각 처리 함수가 자기 Cache-Control을 붙인다.
        route = urlsplit(self.path).path
        if not route.startswith(("/api/", "/streams/")):
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        super().end_headers()

    def _json(self, status: int, body: dict[str, Any]) -> None:
        """JSON 응답 (한글을 그대로 두는 UTF-8, 캐시 금지, X-Request-Id 포함)"""
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Request-Id", self.request_id)
        self.end_headers()
        self.wfile.write(payload)

    def _json_error(
        self,
        status: int,
        error_code: str,
        message: str,
        *,
        retryable: bool | None = None,
        **details: object,
    ) -> None:
        """오류 응답 (error_code, message, error, retryable, request_id)

        retryable: 같은 요청을 나중에 다시 보내면 될 수 있는지(핑키·수령 구역이 비면 등). 기본은 상태 코드와 오류 코드로 판단."""
        body: dict[str, Any] = {
            "error_code": error_code,
            "message": message,
            "error": message,
            "retryable": is_retryable(status, error_code) if retryable is None else retryable,
            "request_id": self.request_id,
        }
        body.update(details)
        self._json(status, body)

    def do_GET(self) -> None:
        """GET 분배, 맞는 경로가 없으면 웹 화면 파일"""
        # 핸들러 객체 하나가 한 연결에서 요청을 여러 번 처리할 수 있으므로(keep-alive), 요청마다 ID를 새로 만든다.
        self._request_id = None
        route = urlsplit(self.path).path
        parts = route.strip("/").split("/")
        entry = routes.find(routes.GET_ROUTES, route, parts)
        if entry is not None:
            entry.handle(self, route, parts, None)
            return
        if route == "/":
            self.path = "/index.html"
        elif route == "/order":  # 손님 쪽 주문 창 (모의)
            self.path = "/order.html"
        elif route.startswith("/api/"):
            self._json(404, {"error": "API route not found"})
            return
        super().do_GET()

    def do_POST(self) -> None:
        """POST 분배: 경로 찾기 -> 다른 출처 거부(403) -> JSON 본문 읽기(실패 시 400/415) -> 처리 함수"""
        self._request_id = None
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
    """관제 서버 실행: 구성 요소를 조립하고 serve_forever로 요청을 받는다 (단계 목록은 모듈 설명)."""
    # ──── 1) 기본: 실행 모드, 로봇 상태, 작업 저장소, 주소, 정책 실행기 ────
    mode = os.environ.get("CONTROL_STATION_MODE", "mock").strip().lower()
    provider = make_status_provider(mode)
    task_store = TaskStore()
    # 기본은 이 PC에서만 접속한다. 다른 PC의 팔 워커가 API에 닿아야 할 때만, 믿을 수 있는 LAN에서 0.0.0.0 등으로
    # 바꾼다. 인증이 없다.
    host = os.environ.get("CONTROL_STATION_HOST", "127.0.0.1").strip() or "127.0.0.1"
    port = int(os.environ.get("CONTROL_STATION_PORT", "8000"))
    handler = partial(ControlStationHandler, directory=str(WEB_DIR))
    server = ControlStationServer((host, port), handler)
    server.mode = mode
    server.robot_provider = provider
    # 관제에서 직접 띄우는 1회성 LeRobot 정책 클라이언트 (POST /api/arm/policy/start).
    server.policy_run_supervisor = PolicyRunSupervisor()
    # 정책 시작·중단 요청을 하나씩 처리하는 잠금 (api/arm.py의 post_policy_run).
    server.arm_control_lock = threading.RLock()

    # ──── 2) 카메라 ────
    # 어떤 카메라 설정이 있는지 환경 변수로 확인한다. 하나도 없으면 빈 카메라 관리자를 쓴다(카메라 없이 모의로 실행).
    head_detector_configured = any(
        os.environ.get(name, "").strip()
        for name in (
            "CONTROL_STATION_HEAD_YOLO_WEIGHTS",
            "CONTROL_STATION_LOAD_YOLO_WEIGHTS",
            "CONTROL_STATION_HEAD_YOLO_WORLD_PROMPTS",
        )
    )
    # 핑키 주차 판정은 C++ 주차 노드(cpp/parking_vision_ros)가 한다. 관제에는 자체 판정이 없다. 자리를 설정하면
    # VERIFY_ARRIVAL이 노드의 새 판정(check)을 기다리고, 화면은 노드의 실시간 상태를 따라간다.
    parking_spot_id = os.environ.get("CONTROL_STATION_PARKING_SPOT", "").strip() or None
    server.parking_spot_id = parking_spot_id
    # 주차 패널이 보여 줄 영상: Head Camera가 천장 뷰를 보여 준다(카메라 드라이버의 ROS 토픽에서 받음).
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
    camera_input_configured = any(
        os.environ.get(name, "").strip()
        for name in (
            "CONTROL_STATION_LEFT_CEILING_CAMERA",
            "CONTROL_STATION_RIGHT_CEILING_CAMERA",
            "CONTROL_STATION_LEFT_JOINT_CAMERA",
            "CONTROL_STATION_RIGHT_JOINT_CAMERA",
            "CONTROL_STATION_JOINT_CAMERA",
            "CONTROL_STATION_ARM_CAMERA",
        )
    )
    ceiling_camera_configured = any(
        os.environ.get(name, "").strip()
        for name in (
            "CONTROL_STATION_LEFT_CEILING_CAMERA",
            "CONTROL_STATION_RIGHT_CEILING_CAMERA",
        )
    )
    paired_joint_cameras_configured = any(
        os.environ.get(name, "").strip()
        for name in (
            "CONTROL_STATION_LEFT_JOINT_CAMERA",
            "CONTROL_STATION_RIGHT_JOINT_CAMERA",
        )
    )
    if not (
        head_detector_configured
        or front_camera_configured
        or camera_input_configured
        or server.parking_camera_id
        or os.environ.get("CONTROL_STATION_SHELF_CAMERA", "").strip()
        or os.environ.get("CONTROL_STATION_HEAD_CAMERA", "").strip().startswith("ros:")
        or any(devices.devices for devices in server.policy_run_supervisor.arm_devices.values())
    ):
        server.camera_manager = UnconfiguredArmCameraStreamManager()
    else:
        # 팔 두 대일 때 카메라는 여섯 대: 천장(주차), 선반, 팔마다 그리퍼 캠과 탑 캠.
        # 예전 단일 관절 카메라는 팔별(LEFT/RIGHT) 관절 카메라 설정이 없을 때만 연다.
        # policy_active/policy_devices: 정책 프로세스가 쓰는 동안 그 카메라를 관제가 닫아 두기 위한 콜백.
        server.camera_manager = OpenCVArmCameraStreamManager(
            joint_enabled=(
                (
                    not paired_joint_cameras_configured
                    and (
                        bool(
                            os.environ.get("CONTROL_STATION_JOINT_CAMERA", "").strip()
                            or os.environ.get("CONTROL_STATION_ARM_CAMERA", "").strip()
                        )
                    )
                )
            ),
            head_enabled=(
                not ceiling_camera_configured
                or bool(os.environ.get("CONTROL_STATION_HEAD_CAMERA", "").strip())
                or head_detector_configured
                or bool(server.parking_camera_id)
            ),
            policy_active=lambda: bool(server.policy_run_supervisor.snapshot()["process_alive"]),
            policy_devices=server.policy_run_supervisor.active_devices,
            arm_cameras=server.policy_run_supervisor.arm_devices,
        )
    server.task_store = task_store

    # ──── 3) 팔: 칸 정의, 학습 정책, 명령 큐, 연속 실행, 팔 설정 ────
    try:
        slot_map, slot_map_error = load_shelf_slot_map(), None
    except ShelfSlotMapError as exc:
        # 다른 명령은 그대로 되고, pick_slot_id를 쓰는 명령만 거부한다.
        slot_map, slot_map_error = None, str(exc)
        print(f"칸 정의를 불러오지 못했습니다: {exc}", flush=True)
    try:
        act_policies, act_policies_error = load_act_policies(), None
    except ActPolicyError as exc:
        # 다른 명령은 모두 그대로 되고, PICK_WITH_POLICY만 거부한다.
        act_policies, act_policies_error = None, str(exc)
        print(f"정책 정의를 불러오지 못했습니다: {exc}", flush=True)
    server.arm_commands = ArmCommandStore(
        task_store.session_factory, slot_map=slot_map, slot_map_error=slot_map_error, act_policies=act_policies
    )
    server.arm_sequences = ArmSequenceSupervisor(task_store.session_factory, server.arm_commands)
    server.act_policies_error = act_policies_error
    # 어느 팔이 어느 선반을 맡는지(왼쪽 선반 -> 왼쪽 팔). 주문이 물체마다 팔을 정하는 데 필요하다.
    # 명령 큐가 아는 팔 ID(ARM_ROBOT_IDS)에 없는 팔이 있으면 설정 전체를 쓰지 않는다.
    try:
        server.arms, server.arms_error = load_arms(), None
        unknown = server.arms.arm_ids - ARM_ROBOT_IDS
        if unknown:
            raise ArmConfigError(f"알 수 없는 팔 ID: {sorted(unknown)}")
    except ArmConfigError as exc:
        server.arms, server.arms_error = None, str(exc)
        print(f"팔 설정을 불러오지 못했습니다: {exc}", flush=True)
    # LeRobot 정책이 직접 여는 카메라, 팔마다. 그 팔의 정책 명령이 실행되는 동안 관제가 닫아 둔다(api/arm.py의
    # sync_policy_cameras). CONTROL_STATION_ARMxx_*로 설정한 팔은 자기 한 쌍을 쓰고, "*"는 예전 단일 팔 한 쌍이다.
    legacy_policy_cameras = [
        device
        for device in (
            os.environ.get("CONTROL_STATION_ACT_OVERHEAD_CAMERA", "").strip(),
            os.environ.get("CONTROL_STATION_JOINT_CAMERA", "").strip(),
        )
        if device
    ]
    server.policy_camera_devices = {
        **({"*": legacy_policy_cameras} if legacy_policy_cameras else {}),
        **{arm_id: devices.devices for arm_id, devices in server.policy_run_supervisor.arm_devices.items()
           if devices.source == "arm" and devices.devices},
    }
    server.policy_camera_lock = threading.Lock()
    server.policy_cameras_lent = False
    # 카메라 검출 결과가 어느 선반 칸 안에 있는지 판정하는 칸 영역.
    server.shelf_slot_zones = ShelfSlotZones(slot_map)

    # ──── 4) 핑키 마커와 주차 판정 노드 ────
    # 핑키마다 자기 마커를 달고 있고, 적재 자리도 핑키마다 따로 등록해 둔다(config/pinky-markers.json).
    try:
        pinky_markers = load_pinky_markers()
    except PinkyMarkerError as exc:
        pinky_markers = {}
        print(f"핑키 마커 파일을 읽지 못했습니다: {exc}", flush=True)
    server.pinky_markers = pinky_markers
    server.parking_check = None
    server.parking_tracker = None
    if parking_spot_id:
        raw_domain = os.environ.get("CONTROL_STATION_MOBILE_DOMAIN", os.environ.get("ROS_DOMAIN_ID"))
        if raw_domain is None:
            raise RuntimeError(
                "CONTROL_STATION_PARKING_SPOT에는 주차 판정 노드의 ROS domain ID"
                "(CONTROL_STATION_MOBILE_DOMAIN 또는 ROS_DOMAIN_ID)를 지정해야 합니다."
            )
        # 실시간으로 따라가는 자리: 기본 자리와 핑키마다 등록한 자리. 자리마다 자기 주차 노드가 있다
        # (ceiling_parking.launch.py가 같은 목록으로 노드를 띄운다). 그래서 핑키마다 자기 마커로 안내받는다.
        parking_node = RosParkingSource(
            int(raw_domain), parking_spot_id, server.parking_camera_id,
            extra_spot_ids=[marker.spot_id for marker in pinky_markers.values() if marker.registered])
        server.parking_node = parking_node
        server.parking_check = parking_node.check
        # 같은 자리의 실시간 상태는 화면 표시용일 뿐이다. VERIFY_ARRIVAL은 여전히 새 판정(check)으로만 정한다.
        server.parking_tracker = parking_node
    arrival_check = partial(server.parking_check, parking_spot_id) if parking_spot_id else None
    # 모의 모드에서만: VERIFY_LOAD가 그냥 모의 성공 대신 모의 개수를 기록한다.
    load_check = mock_load_observer_from_env() if mode == "mock" else None
    # 일부러 켜야 함: 이전 형식 작업의 EXECUTE_ARM_TASK가 실제 PICK_WITH_POLICY 팔 명령을 큐에 넣는다
    # (CONTROL_STATION_TASK_REAL_ARM=1).
    real_arm = env_flag("CONTROL_STATION_TASK_REAL_ARM")
    # ──── 5) 주문 흐름의 핑키 쪽 장치: 모의 세계, 도착 판정, 핑키 관제 ────
    # 카메라 개수 세기와 핑키 관제 클라이언트가 연결되기 전까지 주문은 모의 정류장을 상대로 돈다. 모의 세계의
    # 관측은 모두 simulated로 표시된다. 아래에서 실제 장치가 설정된 부분만 실제 것으로 바꿔 끼운다.
    server.mock_world = MockWorld.from_env()
    world = server.mock_world
    # 주문의 핑키 도착 판정: 주차 노드가 설정되어 있으면 실제 천장 캠 판정, 아니면 모의다
    # (CONTROL_STATION_ORDER_PARKING=mock이면 강제로 모의). 판정은 배정된 핑키 자신의 등록 자리로 한다
    # (위에서 읽은 pinky_markers).
    order_parking = world.parking
    if server.parking_check is not None and os.environ.get("CONTROL_STATION_ORDER_PARKING", "").strip() != "mock":
        order_parking = partial(arrival_for, pinky_markers, server.parking_check)
        print("주문 흐름의 핑키 도착 판정: 실제 주차 판정, 핑키별 마커 " + ", ".join(
            f"{PINKY_NAMES[pid]}={m.marker_id}({'등록' if m.registered else '미등록'})" for pid, m in pinky_markers.items()),
            flush=True)
    # 핑키 임무 단계와 dock_complete / received 신호: URL이 설정되어 있으면 핑키 팀의 핑키 관제 대시보드
    # (예: http://<핑키 PC>:8765), 아니면 모의 세계다. 모의 세계는 주문마다 모의 핑키를 출발시키기도 한다.
    fleet_url = os.environ.get("CONTROL_STATION_PINKY_FLEET_URL", "").strip()
    # 주문을 시작하면 핑키의 경로(주차장 -> 5번 정류장 -> 주문의 수령 구역)가 시작된다. 수령 구역에서 수령을
    # 확인하면 주행완료(mission_complete)를 보내 핑키가 주차장으로 돌아간다.
    fleet_status, fleet_confirm = world.fleet_status, world.fleet_confirm
    dispatch, mission_complete = world.dispatch, world.mission_complete
    fleet = None
    if fleet_url:
        fleet = PinkyFleetClient(fleet_url)
        fleet_status, fleet_confirm = fleet.fleet_status, fleet.fleet_confirm
        dispatch, mission_complete = fleet.dispatch, fleet.mission_complete
        print(f"주문 흐름의 핑키 신호: 핑키 관제 {fleet_url} (주문을 시작하면 실제 핑키가 출발합니다)", flush=True)
        if order_parking == world.parking:
            # 여기에 주차 노드가 없다: 가상 핑키 관제는 자기 가상 천장 캠으로 자기 핑키를 판정한다(그래서 거기서
            # 자리를 벗어나 선 핑키는 다시 들어가게 된다). 실제 대시보드에는 그것이 없으므로(None) 모의 판정이 남는다.
            def order_parking(pinky_id: str) -> dict[str, Any]:
                return fleet.virtual_parking(pinky_id) or world.parking(pinky_id)
    # 핑키 화면도 같은 클라이언트로 읽고 명령하므로, 그 클라이언트의 상태 캐시를 함께 쓴다.
    server.pinky_gateway = PinkyGateway(fleet, world.fleet_status, markers=pinky_markers)

    # ──── 6) 적재 자리 진입 안내와 도착 자동 처리 (pinky/docking.py) ────
    # 시험 화면에서 주문 없이 움직인 핑키에도 같은 카메라 <-> 핑키 주고받기를 한다: 자기 등록 자리에서 판정하고,
    # 카메라의 offset으로 다시 맞추고, 제자리에 있는 것을 두 번 보면 도킹완료한다.
    # dock_signal: 현재 임무 토큰을 붙여 신호를 보낸다(offset이 있을 때만 offset 인자를 넘긴다).
    def dock_signal(pinky_id: str, action: str, offset: dict[str, float] | None) -> dict[str, Any]:
        token = fleet_status(pinky_id).get("mission_token")
        return (fleet_confirm(pinky_id, action, token, offset) if offset is not None
                else fleet_confirm(pinky_id, action, token))

    # 핑키가 들어오는 동안, 자기 등록 자리에서 벗어난 정도(카메라의 실시간 offset)를 핑키 관제를 거쳐 핑키에 보내고,
    # 핑키는 그 값으로 방향을 잡으며 정류장에 들어온다(pinky/docking.py). 자리가 등록되고 실시간으로 추적되는
    # 핑키마다 중계기를 하나씩 두고, 각자 자기 마커의 노드를 읽는다. 그래서 핑키가 다른 핑키의 등록 자리로 안내받는
    # 일이 없다. 실제 대시보드와 주차 노드가 모두 있을 때만 켜지고, CONTROL_STATION_DOCK_GUIDE=0이면 끈다.
    server.dock_guides = {}
    parking_live = getattr(server, "parking_node", None)
    if fleet is not None and parking_live is not None and os.environ.get("CONTROL_STATION_DOCK_GUIDE", "1").strip() != "0":
        def send_guide(pinky_id: str, guide: dict[str, float]) -> bool:
            # 실시간 값이라 0.5초 안에 못 보내면 버린다. 다음 값이 곧 다시 온다.
            status, _answer = fleet._request("POST", f"/api/robots/{pinky_id}/dock-guide", guide, timeout_s=0.5)
            return status == 200

        for pinky_id, marker in pinky_markers.items():
            if marker.registered and marker.spot_id in parking_live.live_spots:
                relay = DockGuideRelay(partial(parking_live.snapshot, marker.spot_id), pinky_id,
                                       lambda pinky_id: fleet.fleet_status(pinky_id).get("phase"), send_guide)
                relay.start()
                server.dock_guides[pinky_id] = relay
        if server.dock_guides:
            print("적재 자리 진입 안내: 천장 캠 실시간 값 → " + ", ".join(
                f"{PINKY_NAMES[pid]}({pinky_markers[pid].spot_id})" for pid in server.dock_guides)
                + " (핑키마다 자기 등록 자리, 핑키 관제 경유)", flush=True)
        unguided = [PINKY_NAMES[pid] for pid in pinky_markers if pid not in server.dock_guides]
        if unguided:
            print("적재 자리 진입 안내 없음(등록 안 됨): " + ", ".join(unguided), flush=True)

    dock_trials = REPO_ROOT / ".control-station" / "logs" / "dock-repeat.jsonl"
    dock_trials_lock = threading.Lock()

    def record_dock_trial(trial: dict[str, Any]) -> None:
        # 진입 반복 시험 기록: 진입 한 번마다 천장 캠 판정을 한 줄씩 남긴다(JSON Lines, 서버를 다시 켜도 남음).
        # 여러 요청 스레드가 동시에 쓸 수 있어 잠금 안에서 쓴다.
        with dock_trials_lock:
            dock_trials.parent.mkdir(parents=True, exist_ok=True)
            with dock_trials.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(trial, ensure_ascii=False) + "\n")

    server.dock_supervisor = DockSupervisor(
        fleet_status, lambda pinky_id: order_parking(pinky_id), dock_signal, record=record_dock_trial,
        in_order=lambda pinky_id: getattr(server, "order_flow", None) is not None and server.order_flow.holds_pinky(pinky_id))
    # 관제가 그 주고받기를 스스로 진행한다. 그래서 시험 화면을 열어 두지 않아도 적재 자리에 도착한 핑키가
    # dock_complete(또는 다시 맞추기)를 받는다. CONTROL_STATION_DOCK_AUTO=0이 아니면 켜지고, 시험 화면의 체크 상자로
    # 바꿀 수 있다(POST /api/pinky/dock-auto). 실제 핑키 관제가 있을 때만 돈다: 모의 세계는 스스로 확인한다.
    server.dock_auto = os.environ.get("CONTROL_STATION_DOCK_AUTO", "1").strip() != "0"
    server.dock_auto_loop = None
    if fleet is not None:
        server.dock_auto_loop = DockAutoLoop(
            PINKY_NAMES, lambda pinky_id: fleet_status(pinky_id).get("phase"),
            lambda pinky_id: server.dock_supervisor.step(pinky_id, True, None),
            enabled=lambda: server.dock_auto).start()
        print("적재 자리 도착 자동 처리: " + ("켜짐 (카메라가 자리에 있다고 두 번 보면 도킹완료, 어긋나면 다시 맞추기)"
                                    if server.dock_auto else "꺼짐 (CONTROL_STATION_DOCK_AUTO=0)"), flush=True)
    # ──── 7) 개수 세기: 재고 확인과 물체별 옮김 확인 ────
    # CONTROL_STATION_ORDER_COUNTS=camera면 카메라 YOLO 검출(선반 캠은 칸 영역 안, 탑 캠은 핑키 쟁반 안)로 세고,
    # 아니면 모의 세계로 센다. 카메라로 세면서 집기는 모의라면, 집기 단계 동안 사람이 물체를 하나씩 옮겨야 한다.
    shelf_stock, pinky_load = world.shelf_stock, world.pinky_load
    # 적재 카메라 카드의 "탑재 목록": 지금 카메라가 핑키 쟁반 위에서 보는 것. 모든 모드에서 화면 표시 전용이고,
    # 주문 흐름에는 절대 쓰이지 않는다(주문 흐름은 아래 카운터를 쓴다).
    load_snapshots = getattr(server.camera_manager, "load_snapshots", server.camera_manager.snapshots)
    server.load_view = load_counter_from_env(load_snapshots, REPO_ROOT,
                                             parking_spot_id)
    server.head_shelf_view = head_shelf_view_from_env(server.camera_manager.snapshots)
    server.head_middle_view = head_middle_view_from_env(server.camera_manager.snapshots)
    server.camera_inventory = partial(
        camera_inventory, server.camera_manager.snapshots, server.head_shelf_view.region,
        parse_region(os.environ.get("CONTROL_STATION_FRONT_INVENTORY_REGION", "0,0,640,480")),
    )
    if server.load_view.camera_id == "head_camera" and server.load_view.region is None:
        # 지금 640×480 탑 캠 화면 기준의 기본 쟁반 영역(픽셀 x1, y1, x2, y2). 주문 검증에는 여전히 등록된 영역이
        # 있어야 한다(이 기본값은 표시용 load_view에만 들어간다).
        server.load_view.region = [366.6, 107.8, 522.3, 263.5]
    if os.environ.get("CONTROL_STATION_ORDER_COUNTS", "").strip() == "camera":
        shelf_stock = ShelfStockCounter(server.camera_manager.snapshots, server.shelf_slot_zones, slot_map, server.arms)
        load_counter = load_counter_from_env(load_snapshots, REPO_ROOT,
                                             parking_spot_id)
        pinky_load = load_counter
        print(f"주문 흐름의 개수 세기: 카메라 YOLO (핑키 쟁반 {load_counter.camera_id} {load_counter.region_or_none()})",
              flush=True)
    if fleet is not None and pinky_load == world.pinky_load:
        # 실제(또는 가상) 핑키에 모의 개수: 고객이 주문을 받아 가면 모의 쟁반을 비운다. 모의 핑키가 수령 구역에서
        # 하는 것과 같다. 이것이 없으면 그 핑키의 다음 주문이 예전 짐을 보고 LOAD_EXTRA_ITEM으로 멈췄다
        # (가상 실행에서 확인, 2026-11-17).
        fleet_complete = mission_complete

        def mission_complete(pinky_id: str, token: str | None) -> dict[str, Any]:
            answer = fleet_complete(pinky_id, token)
            world.unload(pinky_id)
            return answer
    # ──── 8) 주문 집기 ────
    # 주문의 실제 집기(CONTROL_STATION_ORDER_REAL_ARM=1): 계획된 팔에 PICK_WITH_POLICY를 보낸다. 팔 명령 큐를 거쳐
    # 팔마다 HTTP 워커 하나에게, 또는 CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2면 팔마다 PickObject 액션 서버 하나에
    # 목표로 보낸다(/arm_01/pick_object; domain은 CONTROL_STATION_ARM_ACTION_DOMAIN, 없으면 ROS_DOMAIN_ID). 그러면
    # 옮김마다 카메라 개수로 판정한다. 모의 개수로는 팔이 움직이기 전에 주문이 멈춘다(TRANSFER_EVIDENCE_REQUIRED).
    order_real_arm = env_flag("CONTROL_STATION_ORDER_REAL_ARM")
    arm_transport = os.environ.get("CONTROL_STATION_ORDER_ARM_TRANSPORT", "http").strip().lower() or "http"
    if arm_transport not in {"http", "ros2"}:
        raise RuntimeError("CONTROL_STATION_ORDER_ARM_TRANSPORT는 http 또는 ros2여야 합니다.")
    order_pick: Any = world.pick
    pick_hooks: dict[str, Any] = {}
    if order_real_arm and arm_transport == "ros2":
        raw_arm_domain = os.environ.get("CONTROL_STATION_ARM_ACTION_DOMAIN") or os.environ.get("ROS_DOMAIN_ID") or "0"
        server.arm_action_transport = RosPickTransport(int(raw_arm_domain), sorted(ARM_ROBOT_IDS))
        order_pick = ArmActionPick(server.arm_commands, server.arm_action_transport)
        print(f"주문 흐름의 집기: 실제 팔, ROS 2 액션 /arm_0N/pick_object (domain {raw_arm_domain})", flush=True)
    elif order_real_arm:
        order_pick = ArmQueuePick(server.arm_commands)
        print("주문 흐름의 집기: 실제 팔 (PICK_WITH_POLICY, 팔마다 HTTP 워커 필요)", flush=True)
    elif arm_transport == "ros2":
        print("CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2는 CONTROL_STATION_ORDER_REAL_ARM=1일 때만 씁니다 (지금은 모의 집기)",
              flush=True)
    # 실제 집기면 주문 흐름이 옮김 확인 뒤 settle로 팔 명령(UNKNOWN)을 닫는다(arms/pick.py).
    if order_real_arm:
        pick_hooks = {"pick_is_real": True, "pick_settle": order_pick.settle}
    server.order_devices = OrderDevices(
        shelf_stock, pinky_load, fleet_status, fleet_confirm, order_parking, order_pick, dispatch, mission_complete,
        **pick_hooks,
    )

    # ──── 9) 주문 흐름, 작업 실행기, 재고 감시, 주문 데스크, 자동 출발 ────
    try:
        receiving_zones = load_receiving_zones()
    except ReceivingZoneError as exc:
        receiving_zones = {}
        print(f"수령 구역 파일을 읽지 못해 주문을 받을 수 없습니다: {exc}", flush=True)

    def route_for(zone_id: str | None) -> list[str] | None:
        # 수령 구역 -> 핑키가 지날 경로 지점 이름들. 모르는 구역이면 None(경로 없음).
        zone = receiving_zones.get(zone_id or "")
        return list(zone.route_labels) if zone else None
    try:
        arrival_timeout_s = float(os.environ.get("CONTROL_STATION_ARRIVAL_TIMEOUT_S", "600"))
    except ValueError:
        arrival_timeout_s = 600.0  # 숫자가 아니면 기본 600초(핑키 도착 대기 상한)
    # 팔 선택은 이 서버가 하는 집기 종류의 성공률을 참고한다: 실제 팔이면 실제 집기 기록, 아니면 모의 기록만.
    pick_stats = partial(task_store.pick_success_stats, simulated=not order_real_arm)
    server.order_flow = OrderFlow(
        server.arms, act_policies, server.order_devices,
        dwell_s=float(os.environ.get("CONTROL_STATION_ORDER_STEP_DWELL_S", "0.8") or 0.8),
        arrival_timeout_s=arrival_timeout_s,
        pick_stats=pick_stats,
        route_for=route_for,
    )
    server.task_orchestrator = MockTaskOrchestrator(
        task_store, mode=mode, arrival_check=arrival_check, load_check=load_check,
        arm_commands=server.arm_commands if real_arm else None,
        target_evidence=partial(front_camera_evidence, server) if real_arm else None,
        order_flow=server.order_flow,
    )

    def orderable_products() -> list[dict[str, str]]:
        # 주문할 수 있는 물건 = 학습 정책이 있는 물체 (같은 물체에 팔별 정책이 여럿이어도 한 번만).
        if act_policies is None:
            return []
        labels = {policy["object_class"]: policy["label"] for policy in act_policies.policies.values()}
        return [{"object_class": name, "label": label} for name, label in labels.items()]

    def arms_at_work() -> bool:
        # 팔이 집는 동안에는 팔이 선반을 가린다: 주문이 적재 자리를 잡고 있거나 정책 실행이 진행 중일 때다.
        # 재고 감시는 이때 선반을 읽지 않고 장부를 그대로 둔다(팔에 가려 적게 세는 것을 막음).
        return server.order_flow.station_owner is not None or bool(server.policy_run_supervisor.snapshot()["process_alive"])

    overrides_path = Path(os.environ.get("CONTROL_STATION_INVENTORY_OVERRIDES")
                          or REPO_ROOT / "data" / "inventory-overrides.json")
    server.inventory = InventoryMonitor(
        shelf_stock, arms_at_work, orderable_products, task_store.reserved_units, task_store.confirmed_transfers,
        overrides=InventoryOverrides(overrides_path),
    ).start()
    server.order_desk = OrderDesk(
        task_store, server.task_orchestrator, server.order_devices, server.arms, act_policies,
        receiving_zones, pick_stats, inventory=server.inventory.snapshot, pickup=server.order_flow.confirm_pickup,
        arrival=server.order_flow.confirm_arrival,
    )
    # 큰 주문은 여러 운송분으로 나눠 돈다. 두 번째 운송부터는 시작할 때 데스크에서 핑키를 받는다.
    server.order_flow.choose_shipment_pinky = server.order_desk.assign_next_shipment
    # 자동 출발(orders/dispatcher.py): 기다리는 주문을 오래된 순서로, 앞 핑키가 출발하는 대로 하나씩 시작한다.
    # CONTROL_STATION_AUTO_DISPATCH=1이거나 주문 화면의 스위치를 켜야 돈다.
    # CONTROL_STATION_DISPATCH_WAIT_FOR_SPOT=1: 다음 주문은 앞 핑키가 적재 자리에 설 때까지 기다린다.
    auto_dispatch = env_flag("CONTROL_STATION_AUTO_DISPATCH")
    server.order_dispatcher = OrderDispatcher(
        server.order_desk, task_store, enabled=auto_dispatch,
        wait_for_spot=env_flag("CONTROL_STATION_DISPATCH_WAIT_FOR_SPOT"),
    ).start()
    if auto_dispatch:
        print("자동 출발이 켜져 있습니다: 기다리는 주문을 접수 순서대로 출발시킵니다", flush=True)
    if real_arm:
        print("작업 흐름의 팔 단계가 실제 팔 명령(PICK_WITH_POLICY)을 보냅니다", flush=True)

    # ──── 10) 서버 실행과 종료 정리 ────
    # mode는 make_status_provider가 이미 mock/ros2만 받았으므로 KeyError가 나지 않는다.
    mode_label = {"mock": "모의", "ros2": "ROS 2"}[mode]
    print(f"관제 서버 ({mode_label} 모드): http://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nControl station stopped.", flush=True)
    finally:
        # 새 요청과 자동 출발을 먼저 멈춘 뒤, 장치·스레드·DB를 닫는다.
        server.order_dispatcher.close()
        server.server_close()
        if server.parking_tracker is not None:
            server.parking_tracker.close()
        server.camera_manager.close()
        server.policy_run_supervisor.close()
        server.task_orchestrator.close()
        server.inventory.close()
        task_store.close()
        provider.close()


if __name__ == "__main__":
    main()
