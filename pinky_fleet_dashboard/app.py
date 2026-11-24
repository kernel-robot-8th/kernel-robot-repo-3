#!/usr/bin/env python3
"""PC 안에서만 여는 핑키 관제 서버이자, 핑키를 한 대씩 차례로 띄우는 주행준비(bringup) 조정기다.

하는 일
  - 핑키 세 대(10·11·12번, ROS 도메인 10·11·12)에 SSH로 들어가 bringup, ADC 센서, IMU, 안전 노드, Nav2를 띄운다.
  - 로봇 도메인마다 ROS 2 토픽을 직접 구독해 센서·위치·행동 상태를 모으고 GET /api/status로 내보낸다.
  - 임무(주차장 → 적재 자리 5번 → 수령 구역 → 주차장)를 로봇 쪽 navigate_route.py에 맡기고, 그 출력 줄로
    진행 상황과 임무 단계(phase)를 따라간다.
  - 통로·적재 자리·주차장 출입 같은 공유 구역을 예약해 한 대씩만 들여보내고(/fleet/lease), 정면으로 마주친
    두 대 중 한 대를 물러나게 한다(/fleet/yield).
  - 자기 웹 화면(web/)도 내보낸다. 실제 운영 화면은 관제 서버(src/control_station)가 이 API에 HTTP로 붙어 만든다.

목차 (파일 안에서 "# ── N)"으로 찾는다)
   1) 설정값과 로봇 목록
   2) 관제 상태(FleetState)와 전역 객체
   3) 공유 구역 예약·마주침 양보 설정
   4) 숫자 도우미와 지도 파일(PGM/YAML) 읽기
   5) ROS 2 수신(Telemetry)
   6) SSH 실행과 원격 프로세스 시작
   7) Nav2 준비 확인과 프로세스 정리
   8) 로봇 초기화·스크립트 설치·안전 노드와 Nav2 시작
   9) 위치 잡기(라이다·주차 칸·운영자 지정)
  10) 임무 시작 검사와 경로 주행
  11) 정지·자동 복귀·경로 지우기
  12) 주행준비 전체 순서
  13) 백그라운드 루프(구역 허가·양보·램프와 화면·SSH 상태)
  14) HTTP API(Handler)
  15) 시작과 종료(main)

스레드와 잠금
  - HTTP 요청마다 스레드 하나(ThreadingHTTPServer), 오래 걸리는 SSH 작업은 WORKERS(스레드 8개), 백그라운드
    루프 4개(_traffic_loop 0.25초, _indicator_loop 0.25초, _auto_return_loop 1초, _health_loop 3초), ROS 수신
    executor 스레드가 같은 전역 상태를 함께 본다.
  - STATE.lock(RLock)은 STATE.robots·routes·waypoints·busy_robot·password를 지킨다. SSH처럼 몇 초 걸리는 일은
    이 잠금을 쥔 채 하지 않는다. 잠금을 쥔 채 TELEMETRY.snapshot()을 부르는 곳이 있지만, Telemetry 쪽은
    STATE.lock을 잡지 않으므로 교착이 생기지 않는다(잠금 순서: STATE.lock → TELEMETRY.lock).
  - STATE.busy_robot은 "오래 걸리는 준비·위치 작업은 한 번에 하나"라는 표시다. 로봇 id, 여러 대를 함께 다루면
    "all", 비었으면 None이다. 보통 HTTP 처리기가 STATE.lock 안에서 채우고, 그 일을 맡은 worker가 finally에서 비운다.
  - 테스트(tests/)가 이 모듈의 전역(STATE, TELEMETRY, _ssh_run 등)을 patch하므로 파일을 나누지 않는다.

로봇에 무엇을 언제 보내나
  - SSH(비밀번호는 SSH_ASKPASS로 넘기고 메모리에만 둔다): 프로세스 시작·정지(~/.cache/pinky-fleet/<stage>.pid),
    실행 묶음(runtime bundle)·지도·IR 보정값 설치, 경로 실행기 navigate_route.py(SSH를 열어 둔 채 출력을 읽는다),
    정지(stop_robot.py), 운영자 신호 파일(operator_confirm.json, mission_complete.json, dock_command.json).
  - DDS(로봇 도메인으로 직접 발행): /fleet/lease(구역 허가, 0.25초마다), /fleet/yield(양보 지시),
    /fleet/indicator(램프·화면 상태, 0.25초마다), /fleet/dock_guide(천장 카메라 안내 중계), /initialpose(AMCL 위치).
"""

from __future__ import annotations

import json
import math
import os
import re
import shlex
import socket
import signal
import subprocess
import threading
import tempfile
import time
import csv
import io
import uuid
import base64
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote

from scripts.route_geometry import DOCK_APPROACH_M, build_dense_route, build_mission_route, clean_dock_guide, corrected_dock
from scripts.line_following import default_profile, capture_profile, normalize, validate_profile
from scripts.ir_profiles import IRProfiles
from scripts.fleet_ops import operational_status, indicator, activity, BatteryView
from scripts.yielding import YieldArbiter, PROTECTED_PHASES
from scripts.traffic import TrafficCoordinator
from scripts.runtime_bundle import install_command
from scripts.motion_gate import geometry
from scripts.control_journal import ControlJournal
from scripts.scan_localizer import locate as locate_in_map, refine as refine_in_map
from scripts.fusion_grid import validated as validate_fusion_settings
from scripts.ir_calibration import IRCalibrationStore

# ── 1) 설정값과 로봇 목록 ───────────────────────────────────────────────────────────────
# config/ 아래 JSON은 관제를 띄울 때 한 번 읽는다. navigation.json만 주행준비 때마다 다시 읽는다(_reload_navigation).
ROOT = Path(__file__).resolve().parent
# 핑키별 주차 칸: map 좌표 x_m, y_m, yaw_rad와 이름(label). 위치 잡기와 귀환 목적지로 쓴다.
PARKING = json.loads((ROOT / "config" / "parking.json").read_text())
# 주행은 Nav2가 맡는다(2026-11-14 결정): IR은 보고만 하고, 초음파는 라이다와 함께 Nav2의 높이 셀 costmap에
# 들어가며, 초음파 안전 노드는 자기 우회 동작을 하지 않고 장애물 가까이에서 전진만 붙잡는다.
NAVIGATION_PATH = ROOT / "config" / "navigation.json"
NAVIGATION = json.loads(NAVIGATION_PATH.read_text())


def _reload_navigation() -> None:
    """navigation.json을 다시 읽는다. 바뀐 설정이 관제 재시작 없이 다음 주행준비부터 적용된다."""
    global NAVIGATION
    try:
        NAVIGATION = json.loads(NAVIGATION_PATH.read_text())
    except (OSError, ValueError) as exc:
        JOURNAL.record('navigation_config_error', state=str(exc))  # 마지막으로 제대로 읽은 설정을 계속 쓴다
WEB = ROOT / "web"  # 관제가 직접 내보내는 웹 화면 파일
# 이 관제 프로세스 자신의 ROS 도메인. 로봇 토픽은 Telemetry가 로봇 도메인(10·11·12)마다 연 context로 직접 읽는다.
ROS_DOMAIN = 0
# 이 PC 안에서만 연다. 인증 없는 API(비밀번호 입력, 주행 명령)가 있어 바깥 네트워크에 열지 않는다.
HOST = "127.0.0.1"
PORT = 8765
FRESH_SECONDS = 4.0  # 토픽 값이 이보다(초) 오래되면 끊긴 것(stale)으로 본다. map은 한 번만 오므로 예외다
TRAIL_POINTS = 1500   # TRAIL_STEP_M 간격이면 약 30 m 길이의 주행선
TRAIL_STEP_M = 0.02  # 주행선 점 간격(m)
START_TIMEOUT = 20.0  # bringup 뒤 필요한 토픽이 PC에 들어오기를 기다리는 시간(초)
# 주행준비 뒤 자동 위치 잡기: AMCL에 로봇 자기 주차 칸 위치를 넣고, AMCL이 알려 오는 map 위치가 그 근처(아래
# 거리·각도 안)로 돌아와야 한다. 넣는 위치가 정확하므로 이 값들은 AMCL의 첫 보정만 흡수한다. 라이다로 찾은
# 위치나 운영자가 지정한 위치를 넣을 때도 같은 기준을 쓴다(Telemetry.wait_for_pose_near).
LOCALIZE_RADIUS_M = 0.15
LOCALIZE_YAW_RAD = 0.35  # 약 20°
LOCALIZE_TIMEOUT_S = 12.0  # AMCL의 답을 기다리는 시간(초)
# 오도메트리 속도가 이보다 작으면(m/s, rad/s) 멈춰 있다고 본다. 주차 칸 위치를 넣기 전에 확인한다.
STILL_LINEAR_MPS = 0.02
STILL_ANGULAR_RPS = 0.05
# 연속한 라이다 스캔 두 장이 가리키는 위치가 서로 이만큼 안이어야 그 위치를 AMCL에 넣는다.
SCAN_AGREE_M = 0.05
SCAN_AGREE_YAW_RAD = math.radians(5.0)
WAYPOINTS_PATH = ROOT / "data" / "waypoints.json"
# 지도 원본은 저장소가 아니라 이 PC 홈 폴더에서 읽고, 주행준비 때 로봇의 REMOTE_MAP_DIR로 복사한다(_remote_map_path).
LOCAL_MAP_DIR = Path.home() / "Documents" / "Pinky_backup"
LOCAL_MAP_YAML = LOCAL_MAP_DIR / "map-proto.yaml"
LOCAL_MAP_IMAGE = LOCAL_MAP_DIR / "map-proto.pgm"
REMOTE_MAP_DIR = "/home/pinky/.cache/pinky-fleet/maps"
REMOTE_MAP_YAML = f"{REMOTE_MAP_DIR}/map-proto.yaml"
REMOTE_MAP_IMAGE = f"{REMOTE_MAP_DIR}/map-proto.pgm"

# 핑키 세 대. id는 화면·API의 번호, domain은 그 로봇의 ROS_DOMAIN_ID, ip는 SSH 주소다(네트워크가 바뀌면 고친다).
# ns는 도메인 0 브리지(run.sh --bridges, config/bridge/)가 쓰는 namespace로, 이 파일에서는 읽지 않는다.
ROBOTS: dict[str, dict[str, Any]] = {
    "10": {"id": "10", "name": "Pinky C76C", "ip": "192.168.0.9", "domain": 10, "ns": "/pinky_10"},
    "11": {"id": "11", "name": "Pinky D631", "ip": "192.168.0.8", "domain": 11, "ns": "/pinky_11"},
    "12": {"id": "12", "name": "Pinky 1E3E", "ip": "192.168.0.11", "domain": 12, "ns": "/pinky_12"},
}


# ── 2) 관제 상태(FleetState)와 전역 객체 ─────────────────────────────────────────────────


def _blank_robot(robot: dict[str, Any]) -> dict[str, Any]:
    """관제를 막 띄웠을 때의 로봇 한 대 상태(STATE.robots의 항목)."""
    return {
        "id": robot["id"],
        "name": robot["name"],
        "ip": robot["ip"],
        "domain": robot["domain"],
        "ssh_reachable": False,
        "stage": "idle",
        "message": "아직 주행준비를 시작하지 않았습니다.",
        "error": None,
        "map": "map-proto.yaml",
        "last_ssh_ok": None,
    }


class FleetState:
    """관제가 들고 있는 로봇별 상태. 필드는 self.lock(= STATE.lock, RLock)을 쥐고 다룬다.

    robots[id]     화면에 보이는 로봇 상태. stage는 idle → base_starting → sensors_starting → nav_starting →
                   prepared 순으로 가고, 실패하면 fault, 전체 초기화 중에는 stopping이다. message·error·map·
                   ssh_reachable·localized_at(위치를 잡은 시각)·nav_started_at(Nav2를 띄운 시각) 등이 함께 있다.
    waypoints[id]  로봇마다 기록한 웨이포인트 목록(data/waypoints.json에 저장).
    routes[id]     지금 또는 마지막 경로·임무 상태, 없으면 None. status는 starting → running → complete/failed이고,
                   정지하면 cancel_requested → cancelled다. phase는 로봇이 알려 온 임무 단계다(10절 머리말 참고).
    busy_robot     오래 걸리는 작업(주행준비, 위치 잡기, IR 설정 등)을 하는 로봇 id, 여러 대면 "all", 없으면 None.
                   차 있는 동안 다른 준비·위치 작업을 거절한다.
    password       SSH 비밀번호. 실행 중 메모리에만 두고 파일에 쓰지 않는다.
    state_path     마지막 주행준비 기록(data/last_preparation.json). 관제를 다시 띄워도 준비 상태를 이어 쓴다.

    백그라운드 루프 몇 곳은 dict 값 하나를 잠금 없이 읽는다. GIL 아래 값 하나 읽기는 안전하고, 여러 값을 함께
    보고 판단하는 곳은 잠금을 쥔다.
    """
    def __init__(self, state_path=None) -> None:
        self.state_path=Path(state_path) if state_path else None
        self.lock = threading.RLock()
        self.robots = {key: _blank_robot(value) for key, value in ROBOTS.items()}
        if self.state_path:
            try:
                saved=json.loads(self.state_path.read_text())
                for rid,info in saved.items():
                    if rid in self.robots:
                        self.robots[rid]['last_prepared_at']=info.get('last_prepared_at')
                        self.robots[rid]['previously_prepared']=bool(info.get('previously_prepared'))
                        # 관제를 다시 띄워도 로봇의 Nav2와 AMCL은 계속 돈다(2026-11-16): 준비돼 있던 핑키는 지도와
                        # 위치 잡은 시각을 그대로 갖고 다시 준비됨으로 본다. 그 사이 Nav2가 꺼졌다면 실시간 검사
                        # (map 위치, 라이다, 감시기)가 여전히 임무를 거절한다.
                        if info.get('stage')=='prepared' and info.get('map'):
                            self.robots[rid].update(stage='prepared',map=info['map'],localized_at=info.get('localized_at'),
                                                    nav_started_at=info.get('nav_started_at'),
                                                    message='관제를 다시 띄웠습니다 · 로봇의 Nav2와 위치를 그대로 씁니다.')
            except (OSError,ValueError):pass
        self.waypoints = {key: [] for key in ROBOTS}
        try:
            saved = json.loads(WAYPOINTS_PATH.read_text(encoding="utf-8"))
            if isinstance(saved, dict):
                for key in ROBOTS:
                    entries = saved.get(key, [])
                    if isinstance(entries, list):
                        self.waypoints[key] = [item for item in entries if isinstance(item, dict)]
        except (OSError, json.JSONDecodeError):
            pass
        self.busy_robot: str | None = None
        self.password: str | None = None
        self.monitor_error: str | None = None
        self.routes: dict[str, dict[str, Any] | None] = {key: None for key in ROBOTS}

    def save_waypoints(self) -> None:
        """웨이포인트를 임시 파일에 쓴 뒤 os.replace로 바꿔 넣는다(쓰다가 꺼져도 반쯤 쓴 파일이 남지 않는다).
        STATE.lock을 쥔 채 부른다."""
        WAYPOINTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=WAYPOINTS_PATH.parent, delete=False) as stream:
            json.dump(self.waypoints, stream, ensure_ascii=False, indent=2)
            temp_path = Path(stream.name)
        os.replace(temp_path, WAYPOINTS_PATH)

    def update(self, robot_id: str, **values: Any) -> None:
        """robots[robot_id]에 값을 넣는다(잠금은 여기서 잡는다). stage가 prepared가 되면 준비 시각을 남기고,
        stage·map·localized_at·nav_started_at이 바뀌면 state_path에 저장한다(저장 실패는 무시한다)."""
        with self.lock:
            self.robots[robot_id].update(values)
            if values.get('stage')=='prepared':
                self.robots[robot_id]['last_prepared_at']=time.time()
                self.robots[robot_id]['previously_prepared']=True
            if self.state_path and {'stage','map','localized_at','nav_started_at'} & set(values):
                try:
                    self.state_path.parent.mkdir(parents=True,exist_ok=True)
                    temp=self.state_path.with_suffix('.tmp')
                    temp.write_text(json.dumps({rid:{key:r.get(key) for key in ('last_prepared_at','previously_prepared','stage','map','localized_at','nav_started_at')}
                                                for rid,r in self.robots.items()}))
                    temp.replace(self.state_path)
                except OSError:
                    pass


STATE = FleetState(ROOT/'data/last_preparation.json')
IR_PROFILES = IRProfiles(ROOT / "data" / "ir_profiles.json", list(ROBOTS))  # 기체별 IR 보정값
IR_CALIBRATION = IRCalibrationStore(ROOT / 'data' / 'ir_calibration')  # IR 보정 페이지에서 모은 원시 샘플 기록
# 오래 걸리는 SSH 작업(주행준비, 경로 실행, 정지, 위치 잡기)을 HTTP 요청 스레드 밖에서 돌린다. 경로 실행은 주행이
# 끝날 때까지 스레드 하나를 쥐므로, 세 대가 동시에 달려도 다른 작업이 들어갈 자리가 남는 크기다.
WORKERS = ThreadPoolExecutor(max_workers=8, thread_name_prefix="pinky-fleet")
# 도킹 명령(POST .../docking)은 세 대를 통틀어 한 번에 하나만 처리한다. 적용 확인을 기다리는 동안 겹쳐 누르는 것을 막는다.
DOCK_COMMAND_LOCK = threading.Lock()

# ── 3) 공유 구역 예약·마주침 양보 설정 ───────────────────────────────────────────────────
# 구역은 관제를 띄울 때 한 번 정한다(좌표를 바꾸면 관제를 다시 띄운다). 좌표는 웨이포인트와 navigation.json에서
# 오고, 같은 구역 정보가 주행준비 때 로봇에도 설치된다(traffic_zones). 로봇이 알려 오는 traffic_config가 이것과
# 다르면 출발을 거절한다. parking 구역은 세 주차 칸을 감싸는 사각형 [x 최소, x 최대, y 최소, y 최대](칸 ± 0.2 m)이다.
_zones={'parking':{'bounds':[min(p['x_m'] for p in PARKING.values())-.2,max(p['x_m'] for p in PARKING.values())+.2,min(p['y_m'] for p in PARKING.values())-.2,max(p['y_m'] for p in PARKING.values())+.2]}}
def _any_waypoint(label):
    """어느 핑키가 기록했든 이름이 `label`인 첫 웨이포인트(지도는 가리지 않는다), 없으면 None."""
    return next((p for rid in ROBOTS for p in STATE.waypoints[rid] if p.get('label')==label),None)
# 규칙은 단순하게 둔다(2026-11-17, "좁은 곳은 들어가기 몇 초 전에만 보고, 나머지는 그냥 달린다"):
#   passage9   가운데 벽 옆 50 cm 틈: 그 중심에서 request_radius_m 안에 들면 요청한다(주행 약 3~4초 거리).
#              다른 핑키가 그 안에 있을 때만 앞에서 기다린다
#   dock5      5번 적재 자리: 대기 자리에서('5 정면'까지 가는 구간) 요청하고, 요청한 순서대로 들어간다
#   dock_queue 대기 자리 자체: 한 대만 서 있고, 다음 핑키는 통로 밖, 대기 자리 조금 앞에서 멈춘다
#   parking    주차장 출입은 한 번에 한 대
# 주차 칸에서 미리 요청하는 구역은 없다. 7번 교차로(통로 구역 안에 있는, 라인 코스 시절의 흔적)는 통로 구역이
# 설정돼 있으면 따로 구역을 두지 않는다.
# navigation.json을 여기서 한 번 더 읽는다(NAVIGATION과 같은 파일). 구역은 관제를 다시 띄워야 바뀐다.
_traffic_settings=json.loads((ROOT/'config'/'navigation.json').read_text())
_passage=_traffic_settings.get('passage_zone')
_has_passage=isinstance(_passage,dict) and all(isinstance(_passage.get(k),(int,float)) for k in ('x_m','y_m'))
# (웨이포인트 이름, 구역 이름, 요청 반경 m). 구역 중심은 P0~P2 누구든 먼저 기록한 그 웨이포인트 좌표다.
for _label,_name,_radius in [('7','junction7',.25),('5','dock5',.40)]:
    if _name=='junction7' and _has_passage:continue
    _anchor=_any_waypoint(_label)
    if _anchor:_zones[_name]=dict(x_m=_anchor['x_m'],y_m=_anchor['y_m'],request_radius_m=_radius)
# 2026-11-17: 가운데 벽 옆 통로는 한 대씩, 5번 앞 대기 자리에도 한 대, 5번은 요청한 순서대로 들어간다
# (navigation.json의 passage_zone, dock_queue). dock5는 5번으로 가는 구간('5 정면', '5')에서 요청하고,
# 5번에서 release_radius_m(기본 1.0 m)보다 멀어져야 풀린다. 그래서 대기 자리의 다음 핑키가 나가는 핑키와 겹치지 않는다.
if 'dock5' in _zones:
    _zones['dock5'].update(request_labels=['5 정면','5'],release_radius_m=float(_traffic_settings.get('dock_release_radius_m',1.0)))
if _has_passage:
    _zones['passage9']=dict(x_m=float(_passage['x_m']),y_m=float(_passage['y_m']),request_radius_m=float(_passage.get('request_radius_m',.45)))
_queue_label=(_traffic_settings.get('dock_queue') or {}).get('waypoint')
_queue=_any_waypoint(_queue_label) if _queue_label else None
if _queue:
    _zones['dock_queue']=dict(x_m=_queue['x_m'],y_m=_queue['y_m'],request_radius_m=.22)
# 통로 아래 대기 자리(navigation.json wait_low): 다음 핑키는 곧바로 여기까지 와 있다가 대기 자리와 통로가 비면
# 간다. 그래서 세 대가 주차장에 서 있지 않고 모두 길 위에 있게 된다.
_low_label=(_traffic_settings.get('wait_low') or {}).get('waypoint')
_low=_any_waypoint(_low_label) if _low_label else None
if _low:
    _zones['wait_low']=dict(x_m=_low['x_m'],y_m=_low['y_m'],request_radius_m=.30,request_labels=[_low_label])
# 예약은 data/traffic_reservations.json에 남아 관제를 다시 띄워도 이어진다. 연락이 끊긴 핑키의 점유는 저절로
# 풀지 않는다(그 자리에 아직 서 있을 수 있다).
TRAFFIC=TrafficCoordinator(ROOT/'data/traffic_reservations.json',_zones)
# 구역 밖에서 두 핑키가 정면으로 마주치면 한 대에 길을 비키라고 지시한다(scripts/yielding.py, 2026-11-24).
# navigation.json: head_on_yield(false면 끈다), yield_back_m(물러나는 거리, 0~0.40 m), yield_meet_m(이 거리 안에서
# 서로 앞에 두면 마주침으로 본다, 0.30~0.80 m).
def _yield_number(key,default,low,high):
    """navigation.json의 숫자 설정 `key`를 [low, high]로 자른 값. 숫자가 아니면(true/false 포함) default."""
    value=_traffic_settings.get(key,default)
    return min(high,max(low,float(value))) if isinstance(value,(int,float)) and not isinstance(value,bool) else default
YIELD_ENABLED=_traffic_settings.get('head_on_yield',True) is not False
YIELD=YieldArbiter(meet_m=_yield_number('yield_meet_m',.50,.30,.80),back_m=_yield_number('yield_back_m',.25,0.,.40))
# 구역 허가(/fleet/lease)에 붙는 관제 실행 id와 일련번호. 로봇은 중복·역순 허가로는 허가 수명을 늘리지 않고,
# 관제를 다시 띄우면 boot가 바뀌므로 일련번호가 0부터 다시 시작해도 구별된다.
TRAFFIC_BOOT=uuid.uuid4().hex
TRAFFIC_SEQUENCE=0
SHUTTING_DOWN=threading.Event()  # 종료 신호. 구역 허가·자동 복귀 루프가 이것을 보고 멈춘다
# PC 수신 기록(logs/telemetry.jsonl, 5 MB씩 회전). 기체마다 시계가 달라 이 PC 수신 시각을 공통 시간축으로 쓴다.
JOURNAL=ControlJournal(ROOT/'logs/telemetry.jsonl')


# ── 4) 숫자 도우미와 지도 파일(PGM/YAML) 읽기 ────────────────────────────────────────────


def _finite(value: Any) -> float | None:
    """유한한 float로 바꾼 값. 숫자가 아니거나 NaN·무한대면 None(_json이 allow_nan=False로 내보내므로)."""
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def _read_pgm(path: Path) -> tuple[int, int, list[int]] | None:
    """로컬 Nav2 지도가 쓰는 작은 PGM(이진 P5 또는 ASCII P2)을 읽는다. (폭, 높이, 픽셀 값) 또는 못 읽으면 None."""
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    cursor = 0

    def token() -> bytes | None:
        """헤더나 P2 본문의 다음 토큰. 공백과 '#' 주석 줄은 건너뛴다."""
        nonlocal cursor
        while cursor < len(raw):
            if raw[cursor] in b" \t\r\n\f":
                cursor += 1
            elif raw[cursor] == ord("#"):
                newline = raw.find(b"\n", cursor)
                cursor = len(raw) if newline < 0 else newline + 1
            else:
                break
        if cursor >= len(raw):
            return None
        start = cursor
        while cursor < len(raw) and raw[cursor] not in b" \t\r\n\f#":
            cursor += 1
        return raw[start:cursor]

    magic, width_token, height_token, maximum_token = (token(), token(), token(), token())
    try:
        width, height, maximum = int(width_token or b""), int(height_token or b""), int(maximum_token or b"")
    except ValueError:
        return None
    if magic not in (b"P2", b"P5") or width <= 0 or height <= 0 or not 1 <= maximum <= 65535:
        return None
    if magic == b"P2":
        values: list[int] = []
        while len(values) < width * height:
            item = token()
            if item is None:
                return None
            try:
                values.append(int(item))
            except ValueError:
                return None
    else:
        # maxval 뒤에는 공백 문자 하나가 온다(CRLF는 하나로 친다).
        if cursor >= len(raw) or raw[cursor] not in b" \t\r\n\f":
            return None
        if raw[cursor:cursor + 2] == b"\r\n":
            cursor += 2
        else:
            cursor += 1
        bytes_per_sample = 1 if maximum < 256 else 2
        size = width * height * bytes_per_sample
        pixels = raw[cursor:cursor + size]
        if len(pixels) != size:
            return None
        values = list(pixels) if bytes_per_sample == 1 else [int.from_bytes(pixels[i:i + 2], "big") for i in range(0, size, 2)]
    return width, height, values


def _load_local_map_grid(yaml_path: Path = LOCAL_MAP_YAML) -> dict[str, Any] | None:
    """map-proto의 YAML과 PGM으로 OccupancyGrid 모양의 대체 지도를 만든다(Nav2 /map을 아직 못 받았을 때 화면용).
    keepout 마스크처럼 같은 형식의 다른 YAML도 읽는다."""
    try:
        yaml_text = yaml_path.read_text(encoding="utf-8")
        image_name = re.search(r"(?m)^image:\s*(\S+)\s*$", yaml_text).group(1)  # type: ignore[union-attr]
        resolution = float(re.search(r"(?m)^resolution:\s*([0-9.eE+-]+)", yaml_text).group(1))  # type: ignore[union-attr]
        origin_match = re.search(r"(?m)^origin:\s*\[([^]]+)\]", yaml_text)
        origin = [float(item.strip()) for item in origin_match.group(1).split(",")] if origin_match else []
        occupied = float(re.search(r"(?m)^occupied_thresh:\s*([0-9.eE+-]+)", yaml_text).group(1))  # type: ignore[union-attr]
        free = float(re.search(r"(?m)^free_thresh:\s*([0-9.eE+-]+)", yaml_text).group(1))  # type: ignore[union-attr]
        negate = int(re.search(r"(?m)^negate:\s*(\d+)", yaml_text).group(1))  # type: ignore[union-attr]
        pgm = _read_pgm(yaml_path.parent / image_name)
    except (OSError, ValueError, AttributeError):
        return None
    if pgm is None or len(origin) < 3:
        return None
    width, height, pixels = pgm
    cells: list[int] = []
    # PGM 행은 위에서 아래로, OccupancyGrid 행은 아래에서 위로 간다.
    # map_server의 trinary 규칙: 점유 확률이 occupied_thresh보다 크면 100, free_thresh보다 작으면 0, 그 사이는
    # -1(모름). 밝기는 255로 나누므로 8비트 PGM(maxval 255)을 가정한다.
    for row in range(height - 1, -1, -1):
        for col in range(width):
            gray = pixels[row * width + col] / 255.0
            occupancy = gray if negate else 1.0 - gray
            cells.append(100 if occupancy > occupied else 0 if occupancy < free else -1)
    return {
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin_x_m": origin[0],
        "origin_y_m": origin[1],
        "origin_yaw_rad": origin[2],
        "cells": cells,
        "source": "local_file",
    }


LOCAL_MAP_GRID = _load_local_map_grid()  # 관제를 띄울 때 한 번 읽는다. 파일이 없으면 None


def keepout_view():
    """디스크의 Nav2 keepout 마스크를 그 마스크 자신의 원점·해상도(m) 그대로 화면용 격자로 내보낸다."""
    path = ROOT / 'config/keepout/keepout_mask.yaml'
    grid = _load_local_map_grid(path)
    if grid is None:
        return dict(available=False, error='keepout YAML/PGM 읽기 실패')
    return dict(grid, available=True, map='map-proto.yaml', frame_id='map',
                source='keepout_mask.yaml', display_only=True)



# ── 5) ROS 2 수신(Telemetry) ─────────────────────────────────────────────────────────────


class Telemetry:
    """PC 도메인 0의 ROS 2 구독. 브리지가 로봇마다 namespace로 다시 이름을 붙여 보낸다.

    (위는 처음 설계의 설명이다. 지금은 아래 __init__처럼 로봇 도메인(10·11·12)마다 rclpy context와 노드를 따로 열어
    브리지 없이 직접 구독하고, 같은 노드로 /fleet/* 와 /initialpose를 발행한다.)

    스레드: ROS 콜백은 executor 스레드에서 돌고, data·transforms·scans·ir_history·trails는 self.lock(RLock)으로
    지킨다. self.changed는 같은 잠금 위의 Condition이라 wait_for 같은 대기 함수가 새 메시지가 올 때 깨어난다.
    이 클래스는 STATE.lock을 잡지 않는다(snapshot은 STATE.robots에서 지도 이름 하나만 잠금 없이 읽는다).
    """

    # 로봇마다 받는 토픽 이름(화면 API의 topics 키). tf_pose는 토픽이 아니라 /tf에서 계산한 map 위치다.
    KEYS = ("scan", "ir", "ultrasonic", "batt_state", "imu", "battery_percent", "battery_voltage", "odom", "joints", "map", "amcl_pose", "tf_pose", "line_control", "safety_status", "indicator", "display", "plan")

    def __init__(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
        from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan, Range
        from std_msgs.msg import Float32, UInt16MultiArray, String
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        self.rclpy = rclpy
        self.data: dict[str, dict[str, dict[str, Any]]] = {
            rid: {key: {"value": None, "received_at": None} for key in self.KEYS}
            for rid in ROBOTS
        }
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.transforms: dict[str, dict[str, dict[str, Any]]] = {rid: {} for rid in ROBOTS}
        self.ir_history = {rid: deque(maxlen=100) for rid in ROBOTS}
        # 핑키마다 지나온 map 위치(map 프레임 위치에서), 2 cm마다 한 점. 주행선을 그리는 데 쓴다.
        self.trails = {rid: deque(maxlen=TRAIL_POINTS) for rid in ROBOTS}
        self.base_frames = {rid: "base_footprint" for rid in ROBOTS}
        # 로봇별 마지막 LaserScan 전체. 지도에서 로봇 위치를 찾는 데 쓴다(scan_localizer).
        self.scans: dict[str, dict[str, Any] | None] = {rid: None for rid in ROBOTS}
        self.pose_message = PoseWithCovarianceStamped
        self.node = Node("pinky_fleet_dashboard")
        types = {
            "scan": LaserScan,
            "ir": UInt16MultiArray,
            "ultrasonic": Range,
            "batt_state": BatteryState,
            "imu": Imu,
            "battery_percent": Float32,
            "battery_voltage": Float32,
            "odom": Odometry,
            "joints": JointState,
            "map": OccupancyGrid,
            "amcl_pose": PoseWithCovarianceStamped,
            "line_control": String,
            "safety_status": String,
            "indicator": String,
            "display": String,
            "plan": NavPath,
        }
        paths = {
            "scan": "scan",
            "ir": "ir_sensor/range",
            "ultrasonic": "us_sensor/range",
            "batt_state": "batt_state",
            "imu": "imu_raw",
            "battery_percent": "battery/percent",
            "battery_voltage": "battery/voltage",
            "odom": "odom",
            "joints": "joint_states",
            "map": "map",
            "amcl_pose": "amcl_pose",
            "line_control": "line/status",
            "safety_status": "safety/status",
            "indicator": "fleet/indicator_status",
            "display": "fleet/display_status",
            "plan": "plan",  # Nav2 전역 경로. 핑키가 어디로 가려는지 그리는 데 쓴다
        }
        self.traffic_publishers = {}
        self.initial_pose_publishers = {}
        self.indicator_publishers = {}
        self.yield_publishers = {}
        self.dock_guide_publishers = {}
        self.domain_nodes = {}
        self.domain_readers = []
        # 로봇 도메인마다 context·노드·executor(스레드 2개)를 따로 연다. 발행자는 모두 깊이 1의 최신 값 하나만 보낸다.
        for rid, robot in ROBOTS.items():
            context = rclpy.context.Context()
            rclpy.init(args=[], context=context, domain_id=robot['domain'])
            node = Node('pinky_fleet_reader_'+rid,context=context)
            executor = MultiThreadedExecutor(num_threads=2,context=context)
            executor.add_node(node)
            self.domain_readers.append((context,node,executor))
            self.traffic_publishers[rid]=node.create_publisher(String,'/fleet/lease',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.yield_publishers[rid]=node.create_publisher(String,'/fleet/yield',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            # reliable 발행자는 AMCL의 /initialpose 구독이 reliable이든 best effort든 맞물린다.
            self.initial_pose_publishers[rid]=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
            self.domain_nodes[rid]=node
            self.indicator_publishers[rid]=node.create_publisher(String,'/fleet/indicator',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            # 천장 카메라가 본 이 핑키의 적재 자리 대비 어긋남. 들어가는 동안 중계한다(POST dock-guide).
            self.dock_guide_publishers[rid]=node.create_publisher(String,'/fleet/dock_guide',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            for key, msg_type in types.items():
                topic = f"/{paths[key]}"
                subscription_qos = QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
                if key == "scan":
                    # 라이다 드라이버는 보통 LaserScan을 sensor data 프로필(best effort)로 발행한다.
                    # reliable만 받는 구독자는 QoS가 맞지 않아 아무 말 없이 스캔을 하나도 받지 못한다.
                    # (지금은 위 기본값과 같은 설정이라 이 분기는 뜻을 남기는 역할만 한다.)
                    subscription_qos = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                    )
                elif key == "plan":
                    subscription_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                  durability=DurabilityPolicy.VOLATILE)
                elif key == "map":
                    # Nav2는 OccupancyGrid를 transient-local durability로 발행한다.
                    # 같은 durability로 구독해야 관제보다 먼저 발행된 지도도 늦게 붙은 구독자에게 다시 온다.
                    subscription_qos = QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    )
                node.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, r=rid, k=key: self._receive(r, k, msg),
                    subscription_qos,
                )
            node.create_subscription(
                TFMessage,
                "/tf",
                lambda msg, r=rid: self._receive_tf(r, msg, is_static=False),
                100,
            )
            static_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            node.create_subscription(
                TFMessage,
                "/tf_static",
                lambda msg, r=rid: self._receive_tf(r, msg, is_static=True),
                static_qos,
            )

        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        for _,_,executor in self.domain_readers:
            threading.Thread(target=executor.spin,daemon=True).start()

    def _receive(self, robot_id: str, key: str, msg: Any) -> None:
        """ROS 콜백(executor 스레드). 메시지를 JSON으로 보낼 수 있는 작은 dict로 바꿔 PC 수신 시각과 함께 저장한다.
        line_control·safety_status 등은 로봇이 JSON 문자열로 보내는 상태이고, NaN·Infinity가 든 JSON은
        invalid_status로 바꾼다."""
        now = time.time()
        value: Any
        if key == "ir":
            value = {"channels": [int(v) for v in msg.data]}
        elif key in {"line_control", "safety_status", "indicator", "display"}:
            try:
                value = json.loads(msg.data,parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                if not isinstance(value, dict):
                    value = {"state": "invalid_status"}
            except (ValueError, TypeError):
                value = {"state": "invalid_status"}
        elif key == "ultrasonic":
            value = {"range_m": _finite(msg.range), "min_m": _finite(msg.min_range), "max_m": _finite(msg.max_range)}
        elif key == "scan":
            # 화면용 요약: 정면 ±30°(전방 60°) 안의 가장 가까운 거리. 원본 스캔은 위치 찾기용으로 따로 둔다.
            front: list[float] = []
            for index, raw in enumerate(msg.ranges):
                angle = msg.angle_min + index * msg.angle_increment
                if abs(angle) <= math.radians(30):
                    distance = _finite(raw)
                    if distance is not None and msg.range_min <= distance <= msg.range_max:
                        front.append(distance)
            value = {
                "samples": len(msg.ranges),
                "front_min_m": round(min(front), 3) if front else None,
                "angle_min": _finite(msg.angle_min),
                "angle_max": _finite(msg.angle_max),
            }
            raw_scan = {
                "frame_id": str(msg.header.frame_id).lstrip("/"), "angle_min": float(msg.angle_min),
                "angle_increment": float(msg.angle_increment), "range_min": float(msg.range_min),
                "range_max": float(msg.range_max), "received_at": now,
                "ranges": [float(r) if math.isfinite(r) else None for r in msg.ranges],
            }
        elif key == "batt_state":
            value = {"voltage_v": _finite(msg.voltage), "percentage": _finite(msg.percentage)}
        elif key == "imu":
            value = {
                "orientation": {"x": _finite(msg.orientation.x), "y": _finite(msg.orientation.y), "z": _finite(msg.orientation.z), "w": _finite(msg.orientation.w)},
                "angular_velocity": {"x": _finite(msg.angular_velocity.x), "y": _finite(msg.angular_velocity.y), "z": _finite(msg.angular_velocity.z)},
                "linear_acceleration": {"x": _finite(msg.linear_acceleration.x), "y": _finite(msg.linear_acceleration.y), "z": _finite(msg.linear_acceleration.z)},
            }
        elif key == "battery_percent":
            value = {"percent": _finite(msg.data)}
        elif key == "battery_voltage":
            value = {"voltage_v": _finite(msg.data)}
        elif key == "odom":
            pose = msg.pose.pose
            twist = msg.twist.twist
            self.base_frames[robot_id] = str(msg.child_frame_id).lstrip("/") or "base_footprint"
            value = {
                "x_m": _finite(pose.position.x), "y_m": _finite(pose.position.y),
                "linear_mps": _finite(twist.linear.x), "angular_rps": _finite(twist.angular.z),
            }
        elif key == "joints":
            value = {"names": list(msg.name), "positions": [_finite(v) for v in msg.position]}
        elif key == "map":
            origin = msg.info.origin
            q = origin.orientation
            origin_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            value = {
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution_m": _finite(msg.info.resolution),
                "origin_x_m": _finite(origin.position.x),
                "origin_y_m": _finite(origin.position.y),
                "origin_yaw_rad": _finite(origin_yaw),
                "source": "nav2",
                # OccupancyGrid.data는 지도 왼쪽 아래 원점부터 행 우선으로 담긴다. 그 값을 그대로 두어야
                # 브라우저가 같은 Nav2 map 프레임에서 지도를 그리고 로봇 위치를 놓을 수 있다.
                "cells": [int(cell) for cell in msg.data],
            }
        elif key == "plan":
            poses = msg.poses
            step = max(1, len(poses) // 80)  # 그리기에 충분한 약 80점만. 플래너의 몇 mm 간격 점을 다 보내지 않는다
            points = [[round(float(item.pose.position.x), 3), round(float(item.pose.position.y), 3)] for item in poses[::step]]
            if poses and (not points or points[-1] != [round(float(poses[-1].pose.position.x), 3), round(float(poses[-1].pose.position.y), 3)]):
                points.append([round(float(poses[-1].pose.position.x), 3), round(float(poses[-1].pose.position.y), 3)])
            value = {"frame_id": str(msg.header.frame_id), "points": points, "length": len(poses)}
        elif key == "amcl_pose":
            pose = msg.pose.pose
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            covariance = msg.pose.covariance
            value = {
                "x_m": _finite(pose.position.x),
                "y_m": _finite(pose.position.y),
                "yaw_rad": _finite(yaw),
                "frame_id": str(msg.header.frame_id),
                "covariance_xy": {
                    "xx": _finite(covariance[0]),
                    "xy": _finite((covariance[1] + covariance[6]) / 2.0),  # 6×6 공분산의 xy·yx 평균
                    "yy": _finite(covariance[7]),
                },
            }
        else:
            value = None
        # 주행 판단에 쓰인 값은 기록으로 남긴다(ControlJournal이 반복 기록을 5 Hz로 줄인다).
        if key in {'line_control','safety_status','odom','ir','ultrasonic'}:
            JOURNAL.record(key,robot_id=robot_id,state=(value or {}).get('state'),data=value)
        with self.changed:
            self.data[robot_id][key] = {"value": value, "received_at": now}
            if key == "scan":
                self.scans[robot_id] = raw_scan
            if key == "ir":
                self.ir_history[robot_id].append((now, value['channels']))
            if key == "odom":
                self._update_tf_pose_unlocked(robot_id, now)
            self.changed.notify_all()

    def _receive_tf(self, robot_id: str, msg: Any, is_static: bool) -> None:
        """/tf, /tf_static 콜백. 자식 프레임마다 부모 기준 (x, y, yaw)를 저장하고 map 위치(tf_pose)를 다시 계산한다."""
        now = time.time()
        with self.changed:
            for transform in msg.transforms:
                parent = str(transform.header.frame_id).lstrip("/")
                child = str(transform.child_frame_id).lstrip("/")
                if not parent or not child:
                    continue
                t = transform.transform.translation
                q = transform.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.transforms[robot_id][child] = {
                    "parent": parent,
                    "x": float(t.x),
                    "y": float(t.y),
                    "yaw": yaw,
                    "received_at": now,
                    "static": is_static,
                }
            self._update_tf_pose_unlocked(robot_id, now)
            self.changed.notify_all()

    def _update_tf_pose_unlocked(self, robot_id: str, now: float) -> None:
        """base 프레임에서 map까지 TF를 2D로 이어 붙여 tf_pose를 만든다. self.lock을 쥔 채 부른다.

        수신 시각은 체인 안 동적(static이 아닌) 변환 중 가장 오래된 것이다. 그래서 AMCL의 map→odom이 끊기면
        odom이 계속 와도 tf_pose가 오래된 것으로 보인다. 주행선은 TRAIL_STEP_M 이상 움직였을 때만 한 점을 더한다."""
        frame = self.base_frames[robot_id]
        x = y = yaw = 0.0
        seen: set[str] = set()
        base_received_at: float | None = None
        while frame != "map" and frame not in seen:
            seen.add(frame)
            edge = self.transforms[robot_id].get(frame)
            if edge is None:
                return
            edge_yaw = edge["yaw"]
            x, y = (
                edge["x"] + math.cos(edge_yaw) * x - math.sin(edge_yaw) * y,
                edge["y"] + math.sin(edge_yaw) * x + math.cos(edge_yaw) * y,
            )
            yaw += edge_yaw
            if not edge["static"]:
                base_received_at = edge["received_at"] if base_received_at is None else min(base_received_at,edge["received_at"])
            frame = edge["parent"]
        if frame != "map" or base_received_at is None:
            return
        pose = {
            "x_m": x,
            "y_m": y,
            "yaw_rad": math.atan2(math.sin(yaw), math.cos(yaw)),
            "frame_id": "map",
        }
        self.data[robot_id]["tf_pose"] = {"value": pose, "received_at": base_received_at}
        trail = getattr(self, "trails", {}).get(robot_id)
        if trail is not None and (not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) >= TRAIL_STEP_M):
            trail.append((round(x, 3), round(y, 3)))

    def wait_for(self, robot_id: str, keys: tuple[str, ...], timeout: float) -> tuple[bool, list[str]]:
        """`keys`가 모두 fresh가 될 때까지 최대 `timeout`초 기다린다. (모두 들어옴, 아직 안 들어온 키 목록)."""
        deadline = time.monotonic() + timeout
        with self.changed:
            while True:
                missing = [key for key in keys if not self._fresh_unlocked(robot_id, key)]
                if not missing:
                    return True, []
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, missing
                self.changed.wait(min(remaining, 0.5))

    def wait_for_pose_near(self, robot_id: str, since: float, x: float, y: float, yaw: float,
                           timeout: float) -> str | None:
        """`since` 뒤에 받은 map 위치 중 (x, y, yaw) 근처를 보인 쪽('amcl_pose' 또는 'tf_pose'), 없으면 None.

        AMCL은 새 /initialpose를 받으면 다음 스캔 때 /amcl_pose로 답하고 map->odom을 다시 보낸다. 위치를 보낸 뒤
        둘 중 하나라도 들어오면 AMCL이 받아들인 것이다. 로봇이 정말 거기 서 있다는 뜻은 아니다."""
        deadline = time.monotonic() + timeout
        with self.changed:
            while True:
                for key in ("amcl_pose", "tf_pose"):
                    entry = self.data[robot_id][key]
                    pose = entry["value"] or {}
                    px, py, pyaw = pose.get("x_m"), pose.get("y_m"), pose.get("yaw_rad")
                    if (entry["received_at"] is not None and entry["received_at"] >= since
                            and pose.get("frame_id") == "map" and None not in (px, py, pyaw)
                            and math.hypot(px - x, py - y) <= LOCALIZE_RADIUS_M
                            and abs(math.atan2(math.sin(pyaw - yaw), math.cos(pyaw - yaw))) <= LOCALIZE_YAW_RAD):
                        return key
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.changed.wait(min(remaining, 0.5))

    def scan_after(self, robot_id: str, after: float, timeout: float) -> dict[str, Any] | None:
        """`after`(PC 시계) 뒤에 받은 마지막 전체 스캔. 최대 `timeout`초 기다리고, 없으면 None."""
        deadline = time.monotonic() + timeout
        with self.changed:
            while True:
                scan = self.scans[robot_id]
                if scan is not None and scan["received_at"] > after:
                    return scan
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.changed.wait(min(remaining, 0.5))

    def frame_in_base(self, robot_id: str, frame: str) -> tuple[float, float, float] | None:
        """로봇 TF 트리(고정 장착)에서 본 `frame`의 base 프레임 기준 (x, y, yaw), 모르면 None."""
        base = self.base_frames[robot_id]
        x = y = yaw = 0.0
        seen: set[str] = set()
        with self.lock:
            while frame != base:
                edge = self.transforms[robot_id].get(frame)
                if edge is None or frame in seen:
                    return None
                seen.add(frame)
                c, s = math.cos(edge["yaw"]), math.sin(edge["yaw"])
                x, y = edge["x"] + c * x - s * y, edge["y"] + s * x + c * y
                yaw += edge["yaw"]
                frame = edge["parent"]
        return x, y, math.atan2(math.sin(yaw), math.cos(yaw))

    def publish_initial_pose(self, robot_id: str, x: float, y: float, yaw: float, wait_s: float = 5.0) -> bool:
        """이 PC에서 AMCL에 map 위치를 보낸다(SSH 없음). /initialpose 구독자를 최대 `wait_s`초 기다리고,
        AMCL이 구독하지 않으면 False. 놓칠 때를 대비해 0.1초 간격으로 세 번 보낸다."""
        publisher = self.initial_pose_publishers[robot_id]
        deadline = time.monotonic() + wait_s
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.1)
        if publisher.get_subscription_count() < 1:
            return False
        message = self.pose_message()
        message.header.frame_id = "map"
        message.pose.pose.position.x, message.pose.pose.position.y = float(x), float(y)
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        # 스캔 맞춤은 몇 cm·몇 도까지 맞으므로 그 둘레에 좁은 분포를 준다(위치 표준편차 5 cm, 방향 5°).
        message.pose.covariance[0] = message.pose.covariance[7] = 0.05 ** 2
        message.pose.covariance[35] = math.radians(5.0) ** 2
        for _ in range(3):
            message.header.stamp = self.domain_nodes[robot_id].get_clock().now().to_msg()
            publisher.publish(message)
            time.sleep(0.1)
        return True

    def _fresh_unlocked(self, robot_id: str, key: str) -> bool:
        received = self.data[robot_id][key]["received_at"]
        if key == "map":
            # 고정 지도는 transient-local로 한 번만 발행되므로 나이와 상관없이 한 번 받았으면 fresh다.
            return received is not None
        return received is not None and (time.time() - received) <= FRESH_SECONDS

    def forget(self, robot_id: str, key: str) -> None:
        """토픽 하나의 마지막 값을 지운다(멈춘 핑키의 Nav2 경로가 갈 곳처럼 지도에 남으면 안 된다). 'trail'이면
        지나온 주행선을 지운다."""
        with self.lock:
            if key == "trail":
                self.trails.get(robot_id, deque()).clear()
                return
            entry = self.data.get(robot_id, {}).get(key)
            if entry is not None:
                entry["value"], entry["received_at"] = None, None

    def snapshot(self) -> dict[str, Any]:
        """로봇별 {토픽: {value, age_s, fresh}} 복사본. 화면 API와 각 루프가 이것을 읽는다.
        map-proto를 쓰는데 Nav2 /map을 아직 못 받았으면 PC의 지도 파일(LOCAL_MAP_GRID)을 대신 넣는다."""
        now = time.time()
        with self.lock:
            result: dict[str, Any] = {}
            for robot_id, values in self.data.items():
                topics = {}
                for key, entry in values.items():
                    received = entry["received_at"]
                    age = None if received is None else max(0.0, now - received)
                    value = entry["value"]
                    if key == "map" and value is None and STATE.robots[robot_id].get("map") == "map-proto.yaml":
                        value = LOCAL_MAP_GRID
                    topics[key] = {
                        "value": value,
                        "age_s": round(age, 1) if age is not None else None,
                        "fresh": (age is not None) if key == "map" else (age is not None and age <= FRESH_SECONDS),
                    }
                trail = list(self.trails.get(robot_id, ()))
                topics["trail"] = {"value": {"points": [list(p) for p in trail]}, "age_s": None, "fresh": bool(trail)}
                result[robot_id] = topics
            return result


TELEMETRY: Telemetry | None = None  # main()에서 만든다. ROS를 못 띄우면 None으로 남고 화면에 오류를 보인다


# ── 6) SSH 실행과 원격 프로세스 시작 ─────────────────────────────────────────────────────


def _topic_ready(robot_id: str, keys: tuple[str, ...], timeout: float = START_TIMEOUT) -> tuple[bool, list[str]]:
    if TELEMETRY is None:
        return False, list(keys)
    return TELEMETRY.wait_for(robot_id, keys, timeout)


def _ssh_run(robot_id: str, remote_command: str, timeout_seconds: float = 35.0, keep_chars: int = 1200,
             send_password: bool = False, redact: bool = True) -> tuple[bool, str]:
    """로봇에서 `remote_command`를 실행한다. (성공, 출력 끝 `keep_chars`자). `send_password`면 SSH 비밀번호를 명령의
    표준 입력으로 써 넣는다(`sudo -S`용). 그래서 비밀번호가 어느 쪽 기계의 인자 목록에도 나타나지 않는다.

    비밀번호 로그인만 쓰고(공개키 없음), 처음 보는 호스트 키는 받아들인다. 접속 6초, 전체 `timeout_seconds`초
    제한이다. 출력의 비밀번호는 [redacted]로 가린다. 성공하면 last_ssh_ok와 ssh_reachable을 갱신한다.
    부르는 쪽은 STATE.lock을 쥐고 있으면 안 된다(몇 초 걸린다)."""
    robot = ROBOTS[robot_id]
    with STATE.lock:
        password = STATE.password
    if password is None:
        return False, "SSH 비밀번호를 먼저 입력하세요."

    args = [
        "-o", "PreferredAuthentications=password,keyboard-interactive",
        "-o", "PubkeyAuthentication=no",
        "-o", "ConnectTimeout=6",
        "-o", "StrictHostKeyChecking=accept-new",
        "-o", "RequestTTY=no",
        f"pinky@{robot['ip']}",
        remote_command,
    ]
    try:
        # OpenSSH askpass를 써서 관제가 외부 Python 패키지에 기대지 않게 한다. 비밀번호는 argv에도 스크립트
        # 파일에도 들어가지 않고 환경 변수 PINKY_SSH_PASSWORD로만 전달된다.
        with tempfile.TemporaryDirectory(prefix="pinky-askpass-") as temp_dir:
            askpass = Path(temp_dir) / "askpass"
            askpass.write_text("#!/bin/sh\nprintf '%s\\n' \"$PINKY_SSH_PASSWORD\"\n", encoding="utf-8")
            askpass.chmod(0o700)
            child_env = os.environ.copy()
            child_env.update({
                "SSH_ASKPASS": str(askpass),
                "SSH_ASKPASS_REQUIRE": "force",
                "DISPLAY": child_env.get("DISPLAY") or ":0",
                "PINKY_SSH_PASSWORD": password,
            })
            stdin_args = dict(input=password + "\n") if send_password else dict(stdin=subprocess.DEVNULL)
            result = subprocess.run(
                ["ssh", *args],
                **stdin_args,
                capture_output=True,
                text=True,
                timeout=timeout_seconds,
                env=child_env,
                check=False,
            )
        combined = result.stdout + result.stderr
        if redact:  # 비밀번호가 들어갈 수 없는 출력(시계 값)에서만 끈다: 한 자리 숫자 비밀번호라면
            combined = combined.replace(password, "[redacted]")  # 그 숫자가 출력에서 모두 지워질 것이다
        if result.returncode != 0:
            if "Permission denied" in combined:
                return False, "SSH 인증이 거부되었습니다."
            return False, combined[-keep_chars:] or f"SSH 종료 코드 {result.returncode}"
        STATE.update(robot_id, last_ssh_ok=time.time(), ssh_reachable=True)
        return True, combined[-keep_chars:]
    except subprocess.TimeoutExpired:
        return False, "SSH 명령이 시간 초과되었습니다."
    except OSError as exc:
        return False, str(exc)


def _remote_start_command(robot_id: str, stage: str, ros_command: str) -> str:
    """로봇에서 `ros_command`를 nohup으로 띄우는 셸 명령. ~/.cache/pinky-fleet/<stage>.pid에 PID를, <stage>.log에
    출력을 남긴다. 그 PID가 살아 있으면 다시 띄우지 않고 ALREADY_RUNNING, 0.3초 뒤 살아 있으면 STARTED,
    죽었으면 FAILED와 로그 끝 40줄을 출력한다."""
    robot = ROBOTS[robot_id]
    safe_stage = re.sub(r"[^a-z0-9_-]", "", stage.lower())
    work = f"/home/pinky/.cache/pinky-fleet/{safe_stage}"
    log_path = f"{work}.log"
    pid_path = f"{work}.pid"
    inner = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/pinky/pinky_pro/install/setup.bash && "
        f"export ROS_DOMAIN_ID={robot['domain']} && "
        f"exec {ros_command}"
    )
    quoted_inner = shlex.quote(inner)
    # 로그는 이어 쓴다. 표시 줄로 이번 실행의 줄과 그전 줄을 구별한다(scripts/nav_readiness.py가 읽는다).
    return (
        f"mkdir -p /home/pinky/.cache/pinky-fleet && "
        f"if [ -s {shlex.quote(pid_path)} ] && kill -0 \"$(cat {shlex.quote(pid_path)})\" 2>/dev/null; then "
        f"echo ALREADY_RUNNING; exit 0; fi; "
        f"echo \"=== PINKY_FLEET_START {safe_stage} $(date +%s) ===\" >> {shlex.quote(log_path)}; "
        f"nohup bash -lc {quoted_inner} >> {shlex.quote(log_path)} 2>&1 < /dev/null & "
        f"echo $! > {shlex.quote(pid_path)}; sleep 0.3; "
        f"if kill -0 \"$(cat {shlex.quote(pid_path)})\" 2>/dev/null; then echo STARTED; "
        f"else echo FAILED; tail -40 {shlex.quote(log_path)}; exit 1; fi"
    )


MAP_INSTALL_ERRORS: dict[str, str] = {}  # 로봇 id -> 그 로봇에 마지막으로 지도를 복사하지 못한 이유


def _remote_map_path(robot_id: str, map_name: str) -> str | None:
    """Nav2에 넘길 로봇 쪽 지도 YAML 절대 경로, 없으면 None.

    map-proto.yaml은 이 PC의 YAML·PGM을 base64로 SSH 명령에 실어 REMOTE_MAP_DIR에 복사한다(YAML의 image 경로는
    로봇 쪽 경로로 바꾼다). 다른 지도는 로봇의 /home/pinky 아래에서 찾는다."""
    if map_name == "map-proto.yaml":
        if not LOCAL_MAP_YAML.is_file() or not LOCAL_MAP_IMAGE.is_file():
            MAP_INSTALL_ERRORS[robot_id] = f"{LOCAL_MAP_YAML} 또는 {LOCAL_MAP_IMAGE.name}이 없습니다."
            return None
        yaml_text = LOCAL_MAP_YAML.read_text(encoding="utf-8")
        image_match = re.search(r"(?m)^image:\s*([^\s#]+)\s*$", yaml_text)
        if not image_match or Path(image_match.group(1)).name != LOCAL_MAP_IMAGE.name:
            return None
        yaml_text = yaml_text[:image_match.start(1)] + REMOTE_MAP_IMAGE + yaml_text[image_match.end(1):]
        yaml_data = base64.b64encode(yaml_text.encode("utf-8")).decode("ascii")
        image_data = base64.b64encode(LOCAL_MAP_IMAGE.read_bytes()).decode("ascii")
        install_command = (
            f"mkdir -p {shlex.quote(REMOTE_MAP_DIR)} && "
            f"printf '%s' {shlex.quote(image_data)} | base64 -d > {shlex.quote(REMOTE_MAP_IMAGE)} && "
            f"printf '%s' {shlex.quote(yaml_data)} | base64 -d > {shlex.quote(REMOTE_MAP_YAML)} && "
            f"test -s {shlex.quote(REMOTE_MAP_IMAGE)} && test -s {shlex.quote(REMOTE_MAP_YAML)} && "
            f"echo MAP_INSTALLED"
        )
        ok, output = _ssh_run(robot_id, install_command)
        if not ok or "MAP_INSTALLED" not in output:
            MAP_INSTALL_ERRORS[robot_id] = (output.strip() or "SSH 응답 없음")[-600:]
            return None
        return REMOTE_MAP_YAML

    # 저장된 지도는 SSH 로그인 작업 폴더 밖에 있을 수 있다. Nav2 map server는 YAML 경로 자체가 필요하므로
    # 고른 파일 이름을 로봇에서 찾아 launch 파일에 절대 경로로 넘긴다.
    find_command = (
        f"find /home/pinky -type f -name {shlex.quote(map_name)} -print -quit 2>/dev/null"
    )
    ok, output = _ssh_run(robot_id, find_command)
    if not ok:
        return None
    for line in output.splitlines():
        candidate = line.strip()
        if candidate.startswith("/home/pinky/") and not any(ch.isspace() for ch in candidate):
            return candidate
    return None


def _nav_diagnostics_command(robot_id: str) -> str:
    """Nav2가 안 떴을 때 오류에 붙일 진단: /map 토픽 정보, 액션 목록, 컴포넌트 목록, navigation.log 끝 80줄."""
    return (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/pinky/pinky_pro/install/setup.bash && "
        f"export ROS_DOMAIN_ID={ROBOTS[robot_id]['domain']} && {{ "
        "echo '--- /map topic ---'; "
        "timeout 6s ros2 topic info /map -v 2>&1 || true; "
        "echo '--- Nav2 actions ---'; "
        "timeout 8s ros2 action list -t 2>&1 || true; "
        "echo '--- Nav2 components ---'; "
        "timeout 6s ros2 component list /nav2_container 2>&1 || true; "
        "echo '--- navigation log ---'; "
        "tail -80 /home/pinky/.cache/pinky-fleet/navigation.log 2>&1 || true; "
        "}"
    )


# ── 7) Nav2 준비 확인과 프로세스 정리 ────────────────────────────────────────────────────
NAV_LOG_PATH = "/home/pinky/.cache/pinky-fleet/navigation.log"
NAV2_BRINGUP_RETRIES = 1  # lifecycle manager가 bringup을 포기했을 때 Nav2를 다시 띄우는 횟수


def _nav_action_ready(robot_id: str, deadline_s: float = 150.0) -> tuple[bool, str]:
    """로봇에서 scripts/nav_readiness.py를 돌려 Nav2 lifecycle 노드가 모두 active인지 최대 `deadline_s`초 확인한다.
    스크립트는 base64로 SSH 명령에 실어 보낸다. 출력에 NAV_ACTION_READY 또는 NAV_BRINGUP_ABORTED 등이 남는다.
    라즈베리파이에서 Nav2가 다 켜지는 데 오래 걸려 기본 150초를 준다."""
    # 액션은 lifecycle 노드가 inactive여도 목록에 보일 수 있다.
    # 그래서 lifecycle 서비스를 읽는다. 준비 확인을 위해 이동 목표를 보내는 일은 절대 없다.
    encoded = base64.b64encode((ROOT / 'scripts/nav_readiness.py').read_bytes()).decode('ascii')
    program = (f"import base64,sys;sys.argv=['nav_readiness',{str(float(deadline_s))!r},{NAV_LOG_PATH!r}];"
               "exec(compile(base64.b64decode(" + repr(encoded) + "),'nav_readiness.py','exec'))")
    command = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/pinky/pinky_pro/install/setup.bash && "
        f"export ROS_DOMAIN_ID={ROBOTS[robot_id]['domain']} && "
        "python3 -c " + shlex.quote(program)
    )
    return _ssh_run(robot_id, command, timeout_seconds=deadline_s + 20.0)


def _nav_ready_with_retry(robot_id: str, map_path: str, map_name: str) -> tuple[bool, str]:
    """(준비됨, 상세): Nav2 액션을 기다린다. lifecycle manager가 bringup을 포기했으면(NAV_BRINGUP_ABORTED) Nav2를
    멈추고 다시 띄우기를 NAV2_BRINGUP_RETRIES번까지 한다. 다른 실패는 그대로 돌려준다."""
    ready, detail = _nav_action_ready(robot_id)
    for attempt in range(NAV2_BRINGUP_RETRIES):
        if ready or "NAV_BRINGUP_ABORTED" not in detail:
            break
        STATE.update(robot_id, message=f"Nav2가 켜지다 멈췄습니다(라즈베리파이가 바빠 lifecycle 응답이 늦음). "
                                       f"다시 띄웁니다 ({attempt + 1}/{NAV2_BRINGUP_RETRIES}).")
        stopped, output = _stop_existing_navigation(robot_id)
        if not stopped or "NAV_PROCESSES_CLEARED" not in output:
            return False, detail + "\n다시 띄우려 했지만 Nav2를 정리하지 못했습니다: " + output.strip()
        ok, output = _start_navigation_with_safety(robot_id, map_path, map_name)
        if not ok:
            return False, detail + "\nNav2 재시작 실패: " + output.strip()
        map_ready, missing = _topic_ready(robot_id, ("map",), timeout=25.0)
        if not map_ready:
            return False, detail + "\n다시 띄운 Nav2가 /map을 발행하지 않습니다: " + ", ".join(missing)
        ready, detail = _nav_action_ready(robot_id)
    return ready, detail


def _stop_tracked_navigation(robot_id: str) -> tuple[bool, str]:
    """관제가 기록한 navigation.pid만 멈춘다(SIGINT, 12초 뒤에도 살아 있으면 SIGTERM).
    지금은 이 파일 안에서 부르는 곳이 없다(_stop_existing_navigation이 대신한다)."""
    # 앞서 실패한 시도가 살아 있는 launch 프로세스를 남겼을 수 있다. 이 관제가 기록한 PID에만 신호를 보내고,
    # 다시 시도하기 전에 ROS launch가 자식 프로세스를 정리하게 둔다.
    stop_command = (
        "pidfile=/home/pinky/.cache/pinky-fleet/navigation.pid; "
        "if [ -s \"$pidfile\" ]; then "
        "pid=$(cat \"$pidfile\"); "
        "if kill -0 \"$pid\" 2>/dev/null; then "
        "kill -INT \"$pid\" 2>/dev/null || true; "
        "for n in $(seq 1 12); do kill -0 \"$pid\" 2>/dev/null || break; sleep 1; done; "
        "if kill -0 \"$pid\" 2>/dev/null; then kill -TERM \"$pid\" 2>/dev/null || true; fi; "
        "fi; rm -f \"$pidfile\"; fi; echo NAV_STOPPED"
    )
    return _ssh_run(robot_id, stop_command)


NAV2_FILES = ("nav2_params.yaml", "bringup_launch.xml", "navigation_launch.xml", "localization_launch.xml",
              "keepout_mask.pgm", "keepout_mask.yaml")  # keepout 마스크가 바뀌어도 Nav2를 새로 띄워야 한다
NAV2_KEEP_PROBE_S = 20.0  # 떠 있는 Nav2가 정상인지 볼 때 기다리는 시간(초)


def _nav2_files_hash_command() -> str:
    """로봇에 설치된 Nav2 설정 파일(NAV2_FILES)을 이어 붙인 sha256 앞 16자를 출력하는 명령. 설정이 바뀌었는지 본다."""
    files = " ".join(shlex.quote(f"/home/pinky/.cache/pinky-fleet/nav2/{name}") for name in NAV2_FILES)
    return f"cat {files} 2>/dev/null | sha256sum | cut -c1-16"


def _running_nav2_keepable(robot_id: str, map_path: str) -> tuple[bool, str]:
    """이 관제가 로봇에 띄운 Nav2를 그대로 둬도 되는지: 이 지도로 돌고 있고 active로 답하는가.

    핑키에서 Nav2를 띄우는 일은 주행준비에서 느리고 잘 실패하는 부분이라(2026-11-16), ROSY가 Pinky Pro에서
    Nav2를 오래 도는 서비스로 두듯 건강한 Nav2는 살려 둔다. 설정이 바뀌었는지는 실행 묶음을 다시 설치한 뒤
    확인한다(_start_navigation_with_safety).
    지금은 부르는 곳이 없다: _prepare_robot이 Nav2를 매번 새로 띄우기로 바꿨다(2026-11-16 주석 참고)."""
    check = ("pidfile=/home/pinky/.cache/pinky-fleet/navigation.pid; "
             "if [ -s \"$pidfile\" ] && kill -0 \"$(cat \"$pidfile\")\" 2>/dev/null && "
             "tr '\\0' ' ' < \"/proc/$(cat \"$pidfile\")/cmdline\" | grep -qF -- " + shlex.quote(f"map:={map_path}") + "; "
             "then echo NAV2_RUNNING_SAME_MAP; else echo NAV2_NOT_RUNNING; fi")
    ok, output = _ssh_run(robot_id, check)
    if not ok or "NAV2_RUNNING_SAME_MAP" not in output:
        return False, "같은 지도로 떠 있는 Nav2가 없습니다."
    ready, detail = _nav_action_ready(robot_id, NAV2_KEEP_PROBE_S)
    return (True, "Nav2가 이미 정상입니다.") if ready and "NAV_ACTION_READY" in detail else (False, "떠 있는 Nav2가 정상으로 답하지 않습니다.")


def _stop_existing_navigation(robot_id: str) -> tuple[bool, str]:
    """깨끗한 Nav2 하나를 띄우기 전에, 기록된 Nav2와 기록 밖에 남은 Nav2 프로세스를 모두 멈춘다.

    순서: pid 파일의 launch가 Nav2가 맞으면 SIGINT(5초 기다린 뒤 SIGTERM) → 이름 패턴으로 남은 프로세스에 SIGINT →
    최대 5초 기다림 → 그래도 남으면 SIGKILL. 패턴의 '[r]os2'처럼 첫 글자를 괄호로 묶어 pgrep이 자기 자신을
    잡지 않게 한다. 끝나면 NAV_PROCESSES_CLEARED를 출력한다."""
    command = (
        "pidfile=/home/pinky/.cache/pinky-fleet/navigation.pid; "
        "if [ -s \"$pidfile\" ]; then "
        "pid=$(cat \"$pidfile\"); case \"$pid\" in ''|*[!0-9]*) ;; *) "
        "if kill -0 \"$pid\" 2>/dev/null; then "
        "cmdline=$(tr '\\0' ' ' < \"/proc/$pid/cmdline\" 2>/dev/null || true); "
        "case \"$cmdline\" in *'ros2 launch pinky_navigation bringup_launch.xml'*|*'ros2 launch /home/pinky/.cache/pinky-fleet/nav2/bringup_launch.xml'*) "
        "kill -INT \"$pid\" 2>/dev/null || true; "
        "for n in 1 2 3 4 5; do kill -0 \"$pid\" 2>/dev/null || break; sleep 1; done; "
        "if kill -0 \"$pid\" 2>/dev/null; then kill -TERM \"$pid\" 2>/dev/null || true; fi;; esac; "
        "fi;; esac; rm -f \"$pidfile\"; fi; "
        "for pattern in '[r]os2 launch pinky_navigation bringup_launch.xml' "
        "'[r]os2 launch /home/pinky/.cache/pinky-fleet/nav2/bringup_launch.xml' "
        "'[n]av2_' '[r]clcpp_components/component_container'; do "
        "for pid in $(pgrep -f \"$pattern\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; kill -INT \"$pid\" 2>/dev/null || true; done; done; "
        "for n in 1 2 3 4 5; do "
        "pgrep -f '[r]os2 launch (pinky_navigation|/home/pinky/.cache/pinky-fleet/nav2/bringup_launch.xml)|[n]av2_|[r]clcpp_components/component_container' >/dev/null || break; "
        "sleep 1; done; "
        "for pattern in '[r]os2 launch pinky_navigation bringup_launch.xml' "
        "'[r]os2 launch /home/pinky/.cache/pinky-fleet/nav2/bringup_launch.xml' "
        "'[n]av2_' '[r]clcpp_components/component_container'; do "
        "for pid in $(pgrep -f \"$pattern\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; kill -KILL \"$pid\" 2>/dev/null || true; done; done; "
        "echo NAV_PROCESSES_CLEARED"
    )
    return _ssh_run(robot_id, command)


def _stop_existing_base(robot_id: str) -> tuple[bool, str]:
    """새 bringup을 띄우기 전에 남아 있는 핑키 기본 구동·라이다 launch를 정리한다(순서는 _stop_existing_navigation과
    같다). 라이다 드라이버가 두 개 뜨면 시리얼 포트를 다툰다. 끝나면 BASE_PROCESSES_CLEARED를 출력한다."""
    command = (
        "pidfile=/home/pinky/.cache/pinky-fleet/base.pid; "
        "if [ -s \"$pidfile\" ]; then "
        "pid=$(cat \"$pidfile\"); "
        "case \"$pid\" in ''|*[!0-9]*) ;; *) "
        "if kill -0 \"$pid\" 2>/dev/null; then "
        "cmdline=$(tr '\\0' ' ' < \"/proc/$pid/cmdline\" 2>/dev/null || true); "
        "case \"$cmdline\" in *'ros2 launch pinky_bringup bringup_robot.launch.xml'*) "
        "kill -INT \"$pid\" 2>/dev/null || true; "
        "for n in 1 2 3 4 5; do kill -0 \"$pid\" 2>/dev/null || break; sleep 1; done; "
        "if kill -0 \"$pid\" 2>/dev/null; then kill -TERM \"$pid\" 2>/dev/null || true; fi;; esac; "
        "fi;; esac; rm -f \"$pidfile\"; fi; "
        "for pattern in '[s]llidar_ros2/lib/sllidar_ros2/sllidar_node' "
        "'[p]inky_bringup/lib/pinky_bringup/bringup' "
        "'[p]inky_bringup/lib/pinky_bringup/battery_publisher' "
        "'[j]oint_state_publisher' '[r]obot_state_publisher' "
        "'[r]os2 launch pinky_bringup bringup_robot.launch.xml'; do "
        "for pid in $(pgrep -f \"$pattern\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; kill -INT \"$pid\" 2>/dev/null || true; done; done; "
        "for n in 1 2 3 4 5; do "
        "pgrep -f '[s]llidar_ros2/lib/sllidar_ros2/sllidar_node|[p]inky_bringup/lib/pinky_bringup/bringup|[p]inky_bringup/lib/pinky_bringup/battery_publisher|[j]oint_state_publisher|[r]obot_state_publisher|[r]os2 launch pinky_bringup bringup_robot.launch.xml' >/dev/null || break; "
        "sleep 1; done; "
        "for pattern in '[s]llidar_ros2/lib/sllidar_ros2/sllidar_node' "
        "'[p]inky_bringup/lib/pinky_bringup/bringup' "
        "'[p]inky_bringup/lib/pinky_bringup/battery_publisher' "
        "'[j]oint_state_publisher' '[r]obot_state_publisher' "
        "'[r]os2 launch pinky_bringup bringup_robot.launch.xml'; do "
        "for pid in $(pgrep -f \"$pattern\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; kill -KILL \"$pid\" 2>/dev/null || true; done; done; "
        "echo BASE_PROCESSES_CLEARED"
    )
    return _ssh_run(robot_id, command)


def _base_diagnostics_command(robot_id: str) -> str:
    """/scan·/odom이 PC에 안 들어올 때의 진단: 로봇 안에서 직접 받은 샘플(base_topic_probe.py), 라이다·구동 오류
    로그, 관련 프로세스. 로봇 안에서는 받는데 PC에서 못 받으면 DDS 연결 문제로 본다(_prepare_robot)."""
    probe = base64.b64encode((ROOT / "scripts" / "base_topic_probe.py").read_bytes()).decode("ascii")
    probe_command = "python3 -c " + shlex.quote("import base64; exec(base64.b64decode(" + repr(probe) + "))")
    return (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/pinky/pinky_pro/install/setup.bash && "
        f"export ROS_DOMAIN_ID={ROBOTS[robot_id]['domain']} && {{ "
        "echo '--- live /scan and /odom samples (direct DDS) ---'; "
        f"{probe_command}; "
        "echo '--- LiDAR/base errors ---'; "
        "grep -E 'sllidar|SL_RESULT|operation time out|process has died|SerialException|Failed to read motor' "
        "/home/pinky/.cache/pinky-fleet/base.log 2>/dev/null | tail -16 || true; "
        "echo '--- matching processes ---'; "
        "pgrep -af '[s]llidar_ros2/lib/sllidar_ros2/sllidar_node|[p]inky_bringup/lib/pinky_bringup/bringup|[p]inky_bringup/lib/pinky_bringup/battery_publisher|[j]oint_state_publisher|[r]obot_state_publisher|[r]os2 launch pinky_bringup bringup_robot.launch.xml' || true; "
        "}"
    )


# ── 8) 로봇 초기화·스크립트 설치·안전 노드와 Nav2 시작 ─────────────────────────────────


def _robot_reset_command(robot_id: str) -> str:
    """이 관제가 띄운 프로세스와 남아 있는 키보드 조작(teleop) 노드를 멈추는 명령.

    먼저 /cmd_vel에 0 속도를 한 번 보내고, 단계마다 pid 파일의 PID가 그 단계의 명령줄과 맞을 때만
    SIGINT(4초) → SIGTERM(3초) → SIGKILL 순으로 멈춘다. 순서는 Nav2 → 감시기 → 센서 → 기본 구동이다."""
    stage_commands = {
        "navigation": "pinky_navigation bringup_launch.xml",
        "velocity_guard": "python3 /home/pinky/.cache/pinky-fleet/nav2/velocity_guard.py",
        "ultrasonic_safety": "python3 /home/pinky/.cache/pinky-fleet/nav2/ultrasonic_safety.py",
        "imu": "pinky_imu_bno055 main_node",
        "adc": "pinky_sensor_adc main_node",
        "base": "pinky_bringup bringup_robot.launch.xml",
    }
    steps = [
        "source /opt/ros/jazzy/setup.bash",
        "source /home/pinky/pinky_pro/install/setup.bash",
        f"export ROS_DOMAIN_ID={ROBOTS[robot_id]['domain']}",
        "timeout 3s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true",
        "pkill -INT -f '[t]eleop_twist_keyboard' 2>/dev/null || true",
        "stop_stage() { stage=\"$1\"; expected=\"$2\"; file=\"/home/pinky/.cache/pinky-fleet/${stage}.pid\"; "
        "[ -s \"$file\" ] || { echo \"$stage: not tracked\"; return 0; }; "
        "pid=$(cat \"$file\"); case \"$pid\" in ''|*[!0-9]*) rm -f \"$file\"; echo \"$stage: invalid pid file cleared\"; return 0;; esac; "
        "if ! kill -0 \"$pid\" 2>/dev/null; then rm -f \"$file\"; echo \"$stage: already stopped\"; return 0; fi; "
        "cmdline=$(tr '\\0' ' ' < \"/proc/$pid/cmdline\" 2>/dev/null || true); "
        "case \"$cmdline\" in *\"$expected\"*) ;; *) rm -f \"$file\"; echo \"$stage: stale pid cleared\"; return 0;; esac; "
        "kill -INT \"$pid\" 2>/dev/null || true; "
        "for n in 1 2 3 4; do kill -0 \"$pid\" 2>/dev/null || break; sleep 1; done; "
        "if kill -0 \"$pid\" 2>/dev/null; then kill -TERM \"$pid\" 2>/dev/null || true; fi; "
        "for n in 1 2 3; do kill -0 \"$pid\" 2>/dev/null || break; sleep 1; done; "
        "if kill -0 \"$pid\" 2>/dev/null; then kill -KILL \"$pid\" 2>/dev/null || true; fi; "
        "rm -f \"$file\"; echo \"$stage: stopped\"; }",
    ]
    steps.extend(f"stop_stage {shlex.quote(stage)} {shlex.quote(expected)}" for stage, expected in stage_commands.items())
    steps.append("echo RESET_COMPLETE")
    return "bash -lc " + shlex.quote("; ".join(steps))


def _reset_all_robots() -> None:
    """[전체 초기화] worker: 세 대를 차례로 초기화하고 busy_robot("all")을 비운다."""
    for robot_id in ROBOTS:
        ok, output = _ssh_run(robot_id, _robot_reset_command(robot_id))
        if ok and "RESET_COMPLETE" in output:
            STATE.update(robot_id, stage="idle", message="대시보드가 시작한 로봇 프로세스를 종료했습니다.", error=None)
        else:
            detail = output.strip() or "SSH 오류"
            STATE.update(robot_id, stage="fault", message="초기화 중 오류가 발생했습니다.", error=detail[-1200:])
    with STATE.lock:
        STATE.busy_robot = None


def _ssh_askpass_env() -> tuple[tempfile.TemporaryDirectory[str], dict[str, str]]:
    """오래 여는 SSH(경로 실행)용 askpass 임시 폴더와 환경 변수. 부르는 쪽이 끝나면 temp_dir.cleanup()을 부른다."""
    temp_dir = tempfile.TemporaryDirectory(prefix="pinky-askpass-")
    askpass = Path(temp_dir.name) / "askpass"
    askpass.write_text("#!/bin/sh\nprintf '%s\\n' \"$PINKY_SSH_PASSWORD\"\n", encoding="utf-8")
    askpass.chmod(0o700)
    with STATE.lock:
        password = STATE.password or ""
    env = os.environ.copy()
    env.update({
        "SSH_ASKPASS": str(askpass),
        "SSH_ASKPASS_REQUIRE": "force",
        "DISPLAY": env.get("DISPLAY") or ":0",
        "PINKY_SSH_PASSWORD": password,
    })
    return temp_dir, env


def _route_remote_command(robot_id: str, goals: list[dict[str, Any]]) -> str:
    """대상 핑키에 Nav2 액션 클라이언트(scripts/navigate_route.py)를 설치하고 실행하는 명령.
    목표 목록은 base64 JSON 인자로 넘기고, 셸 PID를 route.pid에 남긴다(정지할 때 쓴다)."""
    robot = ROBOTS[robot_id]
    script = (ROOT / "scripts" / "navigate_route.py").read_bytes()
    script_b64 = base64.b64encode(script).decode("ascii")
    goals_b64 = base64.b64encode(json.dumps(goals, separators=(",", ":")).encode("utf-8")).decode("ascii")
    route_script = "/home/pinky/.cache/pinky-fleet/navigate_route.py"
    pidfile = "/home/pinky/.cache/pinky-fleet/route.pid"
    inner = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/pinky/pinky_pro/install/setup.bash && "
        f"export ROS_DOMAIN_ID={robot['domain']} && "
        f"echo $$ > {shlex.quote(pidfile)} && "
        f"exec python3 -u {shlex.quote(route_script)} {shlex.quote(goals_b64)}"
    )
    return (
        "mkdir -p /home/pinky/.cache/pinky-fleet && "
        f"printf '%s' {shlex.quote(script_b64)} | base64 -d > {shlex.quote(route_script)} && "
        f"chmod 700 {shlex.quote(route_script)} && "
        f"bash -lc {shlex.quote(inner)}"
    )


def _ir_profile_install_command(profile: dict[str, Any]) -> str:
    """IR 보정값을 로봇의 nav2/ir_profile.json에 임시 파일 → mv로 써 넣는 명령. 로봇 감시기가 이 파일을 다시 읽는다."""
    payload = base64.b64encode(json.dumps(profile, allow_nan=False).encode('utf-8')).decode('ascii')
    target = '/home/pinky/.cache/pinky-fleet/nav2/ir_profile.json'
    return ("mkdir -p /home/pinky/.cache/pinky-fleet/nav2 && "
            f"printf '%s' {shlex.quote(payload)} | base64 -d > {target}.tmp && "
            f"mv {target}.tmp {target} && echo IR_PROFILE_SAVED")


def _reload_display(robot_id: str) -> tuple[int, dict[str, Any]]:
    """이 PC의 scripts/status_display.py를 로봇에 올리고 다시 시작한다. 화면만 건드리고 bringup·Nav2·경로는
    손대지 않으므로 핑키가 달리는 중에도 해도 된다."""
    target = '/home/pinky/.cache/pinky-fleet/nav2/status_display.py'
    payload = base64.b64encode((ROOT / 'scripts/status_display.py').read_bytes()).decode('ascii')
    ok, output = _ssh_run(robot_id, "mkdir -p /home/pinky/.cache/pinky-fleet/nav2 && "
                          f"printf '%s' {shlex.quote(payload)} | base64 -d > {target}.tmp && "
                          f"mv {target}.tmp {target} && echo DISPLAY_SCRIPT_SAVED")
    if not ok or 'DISPLAY_SCRIPT_SAVED' not in output:
        return 502, {"error": "화면 스크립트를 로봇에 올리지 못했습니다: " + (output.strip() or 'SSH 오류')}
    _ssh_run(robot_id, _stop_script_command("status_display"))
    ok, output = _ssh_run(robot_id, _remote_start_command(robot_id, "status_display", f"python3 {target}"))
    if not ok or not any(token in output for token in ("STARTED", "ALREADY_RUNNING")):
        return 502, {"error": "화면 스크립트를 다시 시작하지 못했습니다: " + (output.strip() or 'SSH 오류')}
    JOURNAL.record('display_reload', robot_id=robot_id)
    return 200, {"ok": True, "robot": robot_id}


def _nav2_runtime_install_command(robot_id: str, map_name: str) -> str:
    """로봇에 올릴 실행 묶음(runtime bundle: Nav2 설정, 감시기·행동 스크립트, 구역·차체·IR 설정) 설치 명령.
    scripts/runtime_bundle.py가 만들고, 성공하면 로봇이 NAV2_RUNTIME_READY를 출력한다. keepout은 map-proto에서만 켠다."""
    settings=json.loads((ROOT/'config/robot_geometry.json').read_text())
    dimensions=geometry({**settings['default'],**settings.get('robots',{}).get(robot_id,{})})
    return install_command(
        ROOT,
        dict(
            robot_id=robot_id,
            geometry=dimensions,
            traffic_zones=TRAFFIC.zones,
            keepout_enabled=map_name == "map-proto.yaml",
            navigation=NAVIGATION,
        ),
        IR_PROFILES.get(robot_id),
    )


def _stop_ultrasonic_safety(robot_id: str) -> tuple[bool, str]:
    """초음파 안전 노드와 최종 감시기(velocity_guard)를 멈춘다. 같은 셸 명령의 이름만 바꿔 두 번 보낸다.
    pid 파일의 PID가 다른 프로그램이면(SAFETY_PID_MISMATCH) 아무것도 죽이지 않고 실패로 돌려준다."""
    command = (
        "pidfile=/home/pinky/.cache/pinky-fleet/ultrasonic_safety.pid; "
        "if [ -s \"$pidfile\" ]; then pid=$(cat \"$pidfile\"); "
        "case \"$pid\" in "
        "''|*[!0-9]*) rm -f \"$pidfile\"; echo SAFETY_PID_INVALID ;; "
        "*) "
        "if kill -0 \"$pid\" 2>/dev/null; then "
        "cmdline=$(tr '\\0' ' ' < \"/proc/$pid/cmdline\" 2>/dev/null || true); "
        "case \"$cmdline\" in "
        "*'/home/pinky/.cache/pinky-fleet/nav2/ultrasonic_safety.py'*) "
        "kill -INT \"$pid\" 2>/dev/null || true; "
        "for n in 1 2 3 4 5; do kill -0 \"$pid\" 2>/dev/null || break; sleep 1; done; "
        "if kill -0 \"$pid\" 2>/dev/null; then kill -TERM \"$pid\" 2>/dev/null || true; fi; "
        "for n in 1 2 3; do kill -0 \"$pid\" 2>/dev/null || break; sleep 1; done; "
        "if kill -0 \"$pid\" 2>/dev/null; then kill -KILL \"$pid\" 2>/dev/null || true; fi; "
        "echo SAFETY_STOPPED ;; "
        "*) echo SAFETY_PID_MISMATCH ;; "
        "esac; "
        "else echo SAFETY_NOT_RUNNING; fi; rm -f \"$pidfile\" ;; "
        "esac; "
        "else echo SAFETY_NOT_RUNNING; fi"
    )
    outputs=[]
    for stage in ('ultrasonic_safety','velocity_guard'):
        current=command.replace('ultrasonic_safety',stage)
        ok,output=_ssh_run(robot_id,current)
        outputs.append(output)
        if not ok or 'SAFETY_PID_MISMATCH' in output:return False,'\n'.join(outputs)
    return True,'\n'.join(outputs)


def _stop_script_command(name: str) -> str:
    """pid 파일로 띄운 로봇 쪽 실행 스크립트 `name`.py를 멈추는 명령. 그 PID가 아직 그 스크립트일 때만 죽인다."""
    pidfile = f"/home/pinky/.cache/pinky-fleet/{name}.pid"
    return (f"pidfile={pidfile}; if [ -s \"$pidfile\" ]; then pid=$(cat \"$pidfile\"); "
            f"case \"$(tr '\\0' ' ' < /proc/$pid/cmdline 2>/dev/null)\" in *{name}.py*) kill \"$pid\";; esac; "
            f"rm -f \"$pidfile\"; fi; echo SCRIPT_STOPPED")


def _start_navigation_with_safety(robot_id: str, map_path: str, map_name: str,
                                  keep_nav2: bool = False) -> tuple[bool, str]:
    """실행 묶음을 설치하고 안전·램프 노드를 띄운 뒤 Nav2를 띄운다. `keep_nav2`면 다시 설치해도 설정이 그대로일 때
    떠 있는 Nav2를 둔다. (ok, 출력)을 돌려주고, 그대로 뒀으면 출력이 'NAV2_KEPT'다.

    순서: 이전 안전 노드 정지 → 묶음 설치 → 최종 감시기(velocity_guard) → 초음파 안전 노드 → (퓨전 노드) →
    램프·LED·화면 → Nav2. 감시기를 Nav2보다 먼저 띄워야 Nav2의 속도 명령이 감시기를 거치지 않고 나갈 틈이 없다."""
    _reload_navigation()
    before = _ssh_run(robot_id, _nav2_files_hash_command())[1].strip() if keep_nav2 else ""
    stopped, stop_output = _stop_ultrasonic_safety(robot_id)
    if not stopped or "SAFETY_PID_MISMATCH" in stop_output:
        return False, stop_output or "이전 초음파 안전 감시기를 정리하지 못했습니다."
    ok, output = _ssh_run(robot_id, _nav2_runtime_install_command(robot_id, map_name))
    if not ok or "NAV2_RUNTIME_READY" not in output:
        return False, output or "Nav2 안전 설정을 만들지 못했습니다."

    ok, output = _ssh_run(robot_id, _remote_start_command(robot_id,'velocity_guard',
                             'python3 /home/pinky/.cache/pinky-fleet/nav2/velocity_guard.py'))
    if not ok or not any(token in output for token in ('STARTED','ALREADY_RUNNING')):
        return False, output or '최종 충돌 감시기 시작 실패'
    safety_command = (
        "python3 /home/pinky/.cache/pinky-fleet/nav2/ultrasonic_safety.py"
    )
    ok, output = _ssh_run(robot_id, _remote_start_command(robot_id, "ultrasonic_safety", safety_command))
    if not ok or not any(token in output for token in ("STARTED", "ALREADY_RUNNING")):
        return False, output or "초음파 안전 감시기를 시작하지 못했습니다."

    # 상태 램프는 편의 기능이다: 핑키에 램프나 LED 패키지가 없어도 주행준비는 그냥 계속한다.
    # 램프 스크립트는 방금 실행 묶음과 함께 다시 설치됐다: 새 코드가 돌도록 이전 것을 멈춘다.
    _ssh_run(robot_id, _stop_script_command("status_lamp"))
    _ssh_run(robot_id, _stop_script_command("status_display"))
    # 퓨전을 계속 켜든 말든 이전 것은 멈춘다: ultrasonic_cloud.py는 예전 퓨전이고, 돌고 있는 fusion_grid는
    # 방금 설치한 코드와 설정으로 돌도록 다시 띄워야 한다.
    _ssh_run(robot_id, _stop_script_command("ultrasonic_cloud"))
    _ssh_run(robot_id, _stop_script_command("fusion_grid"))
    if NAVIGATION.get("ultrasonic_fusion"):
        # Nav2 costmap이 퓨전된 낮은 층을 읽도록 설정돼 있으므로, 주행준비에는 이 노드가 돌고 있어야 한다.
        try:
            validate_fusion_settings(NAVIGATION.get("fusion_grid"))
        except ValueError as exc:
            return False, f"config/navigation.json의 fusion_grid 설정이 잘못됐습니다: {exc}"
        ok, output = _ssh_run(robot_id, _remote_start_command(robot_id, "fusion_grid",
                                  "python3 /home/pinky/.cache/pinky-fleet/nav2/fusion_grid.py"))
        if not ok or not any(token in output for token in ("STARTED", "ALREADY_RUNNING")):
            return False, output or "라이다·초음파 퓨전 노드를 시작하지 못했습니다."
    for stage, lamp_command in (("lamp", "ros2 run pinky_lamp_control main_node"), ("led", "ros2 run pinky_led led_server"),
                                ("status_lamp", "python3 /home/pinky/.cache/pinky-fleet/nav2/status_lamp.py"),
                                ("status_display", "python3 /home/pinky/.cache/pinky-fleet/nav2/status_display.py")):
        # 화면 스크립트는 2026-11-16부터 꺼져 있었다(여기 pinkylib에는 LCD가 없어 CPU만 썼다). 지금은 먼저
        # pinky_emotion의 LCD 드라이버를 찾고, 그릴 수 있는 화면이 없으면 스스로 끝난다. 결과는 확인하지 않는다.
        _ssh_run(robot_id, _remote_start_command(robot_id, stage, lamp_command))

    if keep_nav2:
        after = _ssh_run(robot_id, _nav2_files_hash_command())[1].strip()
        if before and before == after:
            # 설정이 같다: 그대로 둔다. 위치는 주행준비 끝의 라이다 위치 잡기가 여전히 다시 확인한다.
            STATE.update(robot_id, nav_started_at=time.time())
            return True, "NAV2_KEPT"
        stopped, stop_output = _stop_existing_navigation(robot_id)  # 설정이 바뀌었다: 깨끗하게 다시 띄운다
        if not stopped or "NAV_PROCESSES_CLEARED" not in stop_output:
            return False, "Nav2 설정이 바뀌었지만 이전 Nav2를 정리하지 못했습니다: " + stop_output.strip()
    # 여기서부터 AMCL이 다시 시작한다: 위치 잡기가 새로 넣기 전까지 AMCL이 알려 오는 map 위치는 믿지 않는다
    # (_last_confirmed_pose가 nav_started_at > localized_at을 보고 판단한다).
    STATE.update(robot_id, nav_started_at=time.time())
    params_path = "/home/pinky/.cache/pinky-fleet/nav2/nav2_params.yaml"
    launch_path = "/home/pinky/.cache/pinky-fleet/nav2/bringup_launch.xml"
    command = (
        f"ros2 launch {shlex.quote(launch_path)} map:={shlex.quote(map_path)} "
        # Nav2 전체를 한 프로세스로 띄운다(핑키 기본값): 열 개를 따로 한꺼번에 띄우면 4코어 핑키의 load가 15까지
        # 올라 lifecycle bring-up이 시간 초과됐다(2026-11-16). 설치기는 합쳐진 노드의 cmd_vel도 따로 띄울 때와
        # 똑같이 감시기 사슬로 돌린다.
        f"use_composition:=True params_file:={shlex.quote(params_path)}"
    )
    return _ssh_run(robot_id, _remote_start_command(robot_id, "navigation", command))


# ── 9) 위치 잡기(라이다·주차 칸·운영자 지정) ─────────────────────────────────────────────
# 새로 띄운 AMCL은 위치를 모른다. 세 가지 방법으로 위치를 넣는다:
#   라이다(_localize_with_lidar)  스캔을 지도와 맞춰 찾는다. 주행준비·Nav2 복구 끝에 자동으로 한다.
#   주차 칸(_localize_at_parking) 핑키가 자기 칸에 서 있다고 보고 그 좌표를 넣는다([주차장 좌표로 맞추기]).
#   운영자(_set_operator_pose)     화면에서 지정한 위치를 라이다로 조금 다듬어 넣는다.
# 어느 쪽이든 멈춰 있을 때만 넣고, AMCL이 그 근처로 답해야 성공이며, 성공하면 localized_at을 남긴다.


def _initial_pose_command(robot_id: str) -> str:
    """로봇에서 scripts/set_initial_pose.py로 자기 주차 칸 좌표를 /initialpose에 발행하는 명령(SSH 경유)."""
    script = (ROOT / "scripts" / "set_initial_pose.py").read_bytes()
    script_b64 = base64.b64encode(script).decode("ascii")
    remote_script = "/home/pinky/.cache/pinky-fleet/set_initial_pose.py"
    return (
        "mkdir -p /home/pinky/.cache/pinky-fleet && "
        f"printf '%s' {shlex.quote(script_b64)} | base64 -d > {shlex.quote(remote_script)} && "
        f"source /opt/ros/jazzy/setup.bash && source /home/pinky/pinky_pro/install/setup.bash && "
        f"export ROS_DOMAIN_ID={ROBOTS[robot_id]['domain']} && python3 {shlex.quote(remote_script)} {PARKING[robot_id]['x_m']} {PARKING[robot_id]['y_m']} {PARKING[robot_id]['yaw_rad']}"
    )


def _standing_still(robot_id: str) -> str | None:
    """지금 위치를 넣을 수 없는 이유(오도메트리 없음 또는 움직이는 중), 멈춰 있으면 None."""
    odom = TELEMETRY.snapshot().get(robot_id, {}).get("odom", {}) if TELEMETRY else {}
    speed = odom.get("value") or {}
    if not odom.get("fresh"):
        return "오도메트리가 들어오지 않아 핑키가 멈춰 있는지 모릅니다."
    if abs(speed.get("linear_mps") or 0.0) > STILL_LINEAR_MPS or abs(speed.get("angular_rps") or 0.0) > STILL_ANGULAR_RPS:
        return "핑키가 움직이는 중이라 위치를 넣지 않았습니다."
    return None


def _lidar_match(robot_id: str, after: float):
    """(맞춤 결과, 안 되는 이유): `after`(PC 시계) 뒤에 받은 다음 스캔에 대한 scan_localizer의 답.
    스캔은 최대 3초 기다린다."""
    grid = TELEMETRY.snapshot().get(robot_id, {}).get("map", {}).get("value") if TELEMETRY else None
    if not grid or not grid.get("cells"):
        return None, "지도를 아직 받지 못했습니다."
    scan = TELEMETRY.scan_after(robot_id, after, 3.0)
    if scan is None:
        return None, "라이다 스캔이 들어오지 않습니다."
    laser = TELEMETRY.frame_in_base(robot_id, scan["frame_id"])
    if laser is None:
        return None, f"라이다 장착 위치(TF {scan['frame_id']} → {TELEMETRY.base_frames[robot_id]})를 모릅니다."
    # 서 있는 핑키가 있을 법한 곳: 주차 칸 아무거나, 5번(적재), 수령 구역. 대칭 후보 중 이런 곳에 가까운 쪽을 고른다.
    # 핑키는 어느 쪽을 보고도 주차하므로 칸은 위치로만 따진다. 그 거울상 자리는 알려진 곳 근처에 있지 않다.
    places = [(spot["x_m"], spot["y_m"], None) for spot in PARKING.values()]
    dock = TRAFFIC.zones.get("dock5") or {}
    if not dock:  # 이 PC에 웨이포인트가 없다: 로봇이 traffic_config로 알려 주는 5번 좌표를 쓴다
        for topics in TELEMETRY.snapshot().values():
            dock = (((topics.get("line_control") or {}).get("value") or {}).get("traffic_config") or {}).get("dock5") or {}
            if dock:
                break
    if isinstance(dock.get("x_m"), (int, float)) and isinstance(dock.get("y_m"), (int, float)):
        places.append((float(dock["x_m"]), float(dock["y_m"]), None))
    # 이 지도에서 어느 핑키든 웨이포인트로 기록한 수령 구역('수령 …')도 핑키가 서서 기다리는 곳이다.
    map_name = STATE.robots[robot_id]["map"]
    places += [(float(item["x_m"]), float(item["y_m"]), None) for rid in ROBOTS for item in STATE.waypoints[rid]
               if "수령" in str(item.get("label")) and item.get("map") == map_name]
    try:
        match = locate_in_map(grid, scan, laser, known_places=places)
    except ValueError as exc:
        return None, f"지도로 위치를 찾을 수 없습니다: {exc}"
    match.scan_received_at = scan["received_at"]
    return match, ""


def _same_pose(a: tuple[float, float, float], b: tuple[float, float, float]) -> bool:
    """두 (x, y, yaw)가 SCAN_AGREE_M·SCAN_AGREE_YAW_RAD 안에서 같은 위치인가."""
    return (math.hypot(a[0] - b[0], a[1] - b[1]) <= SCAN_AGREE_M
            and abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))) <= SCAN_AGREE_YAW_RAD)


CHOICE_MATCH_M = 0.10            # 고른 후보는 새 스캔에서 이 거리·각도 안으로 다시 나와야 한다
CHOICE_MATCH_YAW_RAD = math.radians(10.0)


def _closest_candidate(match, pose: tuple[float, float, float]):
    """`match.candidates`(점수, x, y, yaw) 중 `pose`에서 CHOICE_MATCH_M·CHOICE_MATCH_YAW_RAD 안의 첫 후보, 없으면 None."""
    for candidate in match.candidates:
        _, x, y, yaw = candidate
        if (math.hypot(x - pose[0], y - pose[1]) <= CHOICE_MATCH_M
                and abs(math.atan2(math.sin(yaw - pose[2]), math.cos(yaw - pose[2]))) <= CHOICE_MATCH_YAW_RAD):
            return candidate
    return None


def _last_confirmed_pose(robot_id: str) -> tuple[float, float, float] | None:
    """핑키가 위치를 잡고 있던 동안의 map 위치(Nav2나 기본 구동을 다시 띄우기 전에 읽어 둔다), 없으면 None.
    위치를 잡은 뒤 Nav2를 다시 띄웠으면(nav_started_at > localized_at) 믿을 수 없으므로 None이다."""
    if TELEMETRY is None:
        return None
    robot = STATE.robots[robot_id]
    if not robot.get("localized_at") or (robot.get("nav_started_at") or 0) > robot["localized_at"]:
        return None
    pose = TELEMETRY.snapshot().get(robot_id, {}).get("tf_pose", {})
    value = pose.get("value") or {}
    if not pose.get("fresh") or not all(isinstance(value.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_rad")):
        return None
    return value["x_m"], value["y_m"], value["yaw_rad"]


def _localize_with_lidar(robot_id: str, choice: tuple[float, float, float] | None = None,
                         prior: tuple[float, float, float] | None = None) -> tuple[bool, str]:
    """로봇 자신의 라이다로 지도에서 위치를 찾고(scripts/scan_localizer.py) 그 위치를 AMCL에 넣는다.

    셀이 직사각형이라 어떤 자리와 그 자리를 반 바퀴 돌린 자리가 스캔에 똑같이 잘 맞는 일이 많다. `choice`가
    없으면 한 자리가 분명히 가장 좋을 때, 또는 똑같이 좋은 자리 중 정확히 하나가 로봇 자기 주차 칸일 때만 그
    위치를 쓴다. 아니면 후보를 운영자용으로 저장해 둔다. `choice`는 운영자가 고른 후보의 위치 (x, y, yaw)이고,
    새 스캔 두 장 모두에서 여전히 후보여야 한다. 어느 경우든 연속한 스캔 두 장이 서로 맞아야 한다.
    위치는 이 PC에서 DDS로 AMCL에 보내고(SSH 비밀번호 불필요), AMCL이 그 자리의 map 위치를 알려 와야 한다.
    `prior`는 다시 띄우기 전 위치(_last_confirmed_pose)다. (성공, 일어난 일)을 돌려준다."""
    if TELEMETRY is None:
        return False, "ROS 수신이 없어 위치를 찾을 수 없습니다."
    why_not = _standing_still(robot_id)
    if why_not:
        return False, why_not
    first, why_not = _lidar_match(robot_id, time.time() - 1.0)
    if first is None:
        return False, why_not
    second, why_not = _lidar_match(robot_id, first.scan_received_at)
    if second is None:
        return False, why_not
    result = dict(first.as_dict(), at=time.time(), method="lidar")
    STATE.update(robot_id, localization=result)
    # 다시 띄우기 전에 서 있던 곳(2026-11-16): 라이다에게는 가운데 벽 위와 아래가 비슷해 P0 주차 칸과 5번이
    # 비긴다. 아무도 옮기지 않은 핑키는 그대로 그 자리에 있으므로 그 후보를 택한다.
    near_prior = (_closest_candidate(first, prior), _closest_candidate(second, prior)) if prior and choice is None else (None, None)
    if choice is not None:
        picked, again = _closest_candidate(first, choice), _closest_candidate(second, choice)
        if picked is None or again is None:
            return False, "고른 자리가 지금 라이다 후보에 없습니다. 핑키가 움직였는지 확인하고 후보를 다시 보세요."
        score, x, y, yaw = picked
        reason = "운영자가 고른 라이다 후보로 위치를 넣었습니다"
    elif near_prior[0] is not None and near_prior[1] is not None:
        score, x, y, yaw = near_prior[0]
        reason = "다시 띄우기 전 위치 근처의 라이다 후보로 위치를 넣었습니다"
    else:
        if not first.ok:
            return False, first.reason
        x, y, yaw, score, reason = first.x_m, first.y_m, first.yaw_rad, first.score, first.reason
        if not second.ok or not _same_pose((x, y, yaw), (second.x_m, second.y_m, second.yaw_rad)):
            return False, "연속한 라이다 스캔 두 장이 서로 다른 위치를 가리켜 정하지 않았습니다."
    since = time.time()
    if not TELEMETRY.publish_initial_pose(robot_id, x, y, yaw):
        return False, "AMCL이 /initialpose를 구독하지 않습니다. 주행준비(Nav2)를 먼저 하세요."
    source = TELEMETRY.wait_for_pose_near(robot_id, since, x, y, yaw, LOCALIZE_TIMEOUT_S)
    if source is None:
        return False, f"AMCL에 위치를 보냈지만 {LOCALIZE_TIMEOUT_S:.0f}초 안에 그 근처의 지도 위치가 돌아오지 않았습니다."
    STATE.update(robot_id, localization=dict(result, applied=True, applied_pose={"x_m": x, "y_m": y, "yaw_rad": yaw}),
                 localized_at=time.time())
    where = f"x {x:.2f} · y {y:.2f} m · {math.degrees(yaw):.0f}°"
    return True, f"{reason}: {where}, 일치도 {score:.2f}"


OPERATOR_REFINE_MIN_SCORE = 0.5  # 라이다 다듬기의 일치도가 이보다 낮으면 운영자가 지정한 그대로 쓴다


def _set_operator_pose(robot_id: str, x: float, y: float, yaw: float, refine: bool = True) -> None:
    """운영자가 지도에 핑키 위치를 찍었다(RViz의 2D Pose Estimate처럼). 라이다는 30 cm·30도 안에서만 다듬으므로
    대칭 셀의 거울 자리로 튀지 않는다. 그다음 AMCL에 넣는다. WORKERS에서 돌고, 끝나면 busy_robot을 비운다."""
    try:
        why_not = _standing_still(robot_id)
        if why_not:
            STATE.update(robot_id, message="지정한 위치를 넣지 않았습니다.", error=why_not)
            return
        pose, note = (x, y, yaw), "지정한 그대로"
        grid = TELEMETRY.snapshot().get(robot_id, {}).get("map", {}).get("value") if TELEMETRY else None
        scan = TELEMETRY.scan_after(robot_id, time.time() - 1.0, 3.0) if TELEMETRY else None
        laser = TELEMETRY.frame_in_base(robot_id, scan["frame_id"]) if scan else None
        if not refine:
            # 이 위치는 스캔보다 확실한 곳에서 왔다(5번의 천장 카메라. 그곳은 지도에 없는 팔과 선반 때문에
            # 라이다 맞춤이 17 cm나 어긋났다, 2026-11-17).
            note = "다듬지 않고 그대로"
        elif grid and grid.get("cells") and scan and laser:
            score, rx, ry, ryaw = refine_in_map(grid, scan, laser, x, y, yaw)
            if score >= OPERATOR_REFINE_MIN_SCORE:
                pose = (rx, ry, ryaw)
                note = f"라이다로 {math.hypot(rx - x, ry - y) * 100:.0f} cm·{abs(math.degrees(math.atan2(math.sin(ryaw - yaw), math.cos(ryaw - yaw)))):.0f}° 다듬음, 일치도 {score:.2f}"
            else:
                note = f"라이다 일치도 {score:.2f}로 낮아 지정한 그대로"
        since = time.time()
        if not TELEMETRY or not TELEMETRY.publish_initial_pose(robot_id, *pose):
            STATE.update(robot_id, message="지정한 위치를 넣지 못했습니다.",
                         error="AMCL이 /initialpose를 구독하지 않습니다. 주행준비(Nav2)를 먼저 하세요.")
            return
        if TELEMETRY.wait_for_pose_near(robot_id, since, *pose, LOCALIZE_TIMEOUT_S) is None:
            STATE.update(robot_id, message="지정한 위치를 보냈지만 AMCL이 그 근처로 답하지 않았습니다.", error="AMCL 응답 없음")
            return
        STATE.update(robot_id, localized_at=time.time(), error=None,
                     localization={"ok": True, "applied": True, "method": "operator", "at": time.time(),
                                   "reason": f"운영자가 지도에서 지정 ({note})", "x_m": pose[0], "y_m": pose[1],
                                   "yaw_rad": pose[2], "clicked": {"x_m": x, "y_m": y, "yaw_rad": yaw}, "candidates": []},
                     message=f"운영자가 지정한 위치를 넣었습니다: x {pose[0]:.2f} · y {pose[1]:.2f} m · {math.degrees(pose[2]):.0f}° ({note})")
    finally:
        with STATE.lock:
            if STATE.busy_robot == robot_id:
                STATE.busy_robot = None


def _relocalize(robot_id: str, choice: tuple[float, float, float] | None = None) -> None:
    """[라이다로 위치 찾기] worker: 결과를 화면 메시지로 남기고 busy_robot을 비운다."""
    try:
        ok, detail = _localize_with_lidar(robot_id, choice)
        if ok:
            STATE.update(robot_id, message=detail, error=None)
        else:
            STATE.update(robot_id, message="라이다로 위치를 찾지 못했습니다.", error=detail)
    finally:
        with STATE.lock:
            if STATE.busy_robot == robot_id:
                STATE.busy_robot = None


def _ir_steers(robot_id: str) -> bool:
    """IR 차선 추종은 navigation.json이 허락하고(ir_steering) 그 로봇의 IR 보정이 켜져 있을 때만 조향한다."""
    return bool(NAVIGATION.get("ir_steering")) and bool(IR_PROFILES.get(robot_id).get("enabled"))


def _localize_at_parking(robot_id: str) -> tuple[bool, str]:
    """AMCL에 로봇 자기 주차 칸 위치를 넣고, AMCL이 그 자리의 map 위치를 알려 올 때까지 기다린다.

    로봇은 자기 주차 칸에 서 있어야 한다: 여기서는 실제로 어디 있는지 알 길이 없으므로, 다른 곳에 둔 로봇은
    틀린 위치를 받는다. 움직이는 로봇은 거절한다. (성공, 안 되는 이유 또는 확인해 준 피드)를 돌려준다.
    이쪽은 SSH로 로봇에서 발행한다(set_initial_pose.py)."""
    home = PARKING[robot_id]
    if TELEMETRY is None:
        return False, "ROS 수신이 없어 위치를 잡을 수 없습니다."
    why_not = _standing_still(robot_id)
    if why_not:
        return False, why_not
    since = time.time()
    ok, output = _ssh_run(robot_id, _initial_pose_command(robot_id))
    if not ok or "Published AMCL initial pose" not in output:
        return False, output.strip() or "AMCL 응답 없음"
    source = TELEMETRY.wait_for_pose_near(robot_id, since, home["x_m"], home["y_m"], home["yaw_rad"], LOCALIZE_TIMEOUT_S)
    if source is None:
        return False, (f"AMCL에 {home['label']} 좌표를 보냈지만 {LOCALIZE_TIMEOUT_S:.0f}초 안에 "
                       "그 근처의 지도 위치가 돌아오지 않았습니다.")
    return True, source


def _set_initial_pose(robot_id: str) -> None:
    """[주차장 좌표로 맞추기] worker. 구동·Nav2가 살아 있지 않거나 AMCL이 /initialpose를 구독하지 않으면 주행준비를
    처음부터 다시 한다(그 끝에서 위치도 잡는다). 끝나면 busy_robot을 비운다."""
    try:
        topics=TELEMETRY.snapshot().get(robot_id,{}) if TELEMETRY else {}
        if any(not topics.get(key,{}).get('fresh') for key in ('scan','odom','map','line_control')):
            # 주행준비는 스스로 끝에서 위치를 잡는다.
            STATE.update(robot_id,message='실제 구동 상태가 없어 주행준비를 복구합니다.',error=None)
            _prepare_robot(robot_id,STATE.robots[robot_id]['map'],release_busy=False)
            return
        ok,detail=_localize_at_parking(robot_id)
        if not ok and 'AMCL is not subscribed' in detail:
            STATE.update(robot_id,message='AMCL 미실행 · 주행준비를 한 번 복구합니다.',error=None)
            _prepare_robot(robot_id,STATE.robots[robot_id]['map'],release_busy=False)
            return
        if ok:
            STATE.update(robot_id,stage='prepared',message=f"AMCL 위치를 {PARKING[robot_id]['label']} 좌표로 설정했습니다.",error=None,
                         localized_at=time.time())
        else:
            STATE.update(robot_id,stage='fault',message='AMCL 위치 설정 실패 · 주행준비가 필요합니다.',error=detail)
    finally:
        with STATE.lock:
            if STATE.busy_robot==robot_id:STATE.busy_robot=None


def _runtime_outdated(robot_topics: dict[str, Any]) -> bool:
    """핑키가 아직 예전 주행준비가 설치한 것을 돌리고 있는가: 공유 구역이 관제의 것과 다르거나(그 뒤 웨이포인트가
    옮겨졌다) 행동 층이 이 관제보다 오래됐다(dock_entry에 squaring이 없다). 이런 핑키는 "준비됨"인데도 출발마다
    "주행준비를 다시 실행하세요"로 거절됐고, [주행준비하고 위치 잡기]는 이미 준비됨으로 건너뛰었다(2026-11-19)."""
    guard = (robot_topics.get('line_control') or {}).get('value') or {}
    return bool(guard) and (guard.get('traffic_config') != TRAFFIC.zones or 'squaring' not in (guard.get('dock_entry') or {}))


# ── 10) 임무 시작 검사와 경로 주행 ───────────────────────────────────────────────────────
# 경로·임무는 목표 목록(goals)으로 만들어 로봇의 navigate_route.py에 넘긴다. 실행기는 SSH로 연 채 표준 출력에
# 진행 줄을 쓰고(WAYPOINT_START/DONE, CHECKPOINT_POSE, MISSION_STATUS, ROUTE_DONE/CANCELLED/FAILED),
# _run_waypoint_route가 그것을 STATE.routes에 옮긴다. MISSION_STATUS의 phase(임무 단계):
#   driving          웨이포인트 사이를 Nav2로 달린다          recovering   초음파 우회·차선 복귀 중
#   dock_entry       5번으로 들어간다(천장 카메라 안내 가능)  docking      도킹 동작 중
#   docking_manual   5번 도착·정지, dock_complete 신호 대기   docking_anchor  5번 좌표로 지도 위치 재설정
#   docked_manual    적재 완료(received) 신호 대기             dock_exit    20 cm 후진해 5번을 빠져나온다
#   redocking        카메라 판정으로 물러났다가 다시 들어간다
#   course_complete  수령 구역 도착·정지, 주행완료(mission-complete) 신호 대기
#   returning        자기 주차장으로 돌아간다
# 운영자 신호가 필요한 단계는 그때마다 새 mission_token을 내고, 관제는 token이 맞는 신호만 로봇에 쓴다.


def _route_start_problem(robot_id: str) -> tuple[int, dict[str, str]] | None:
    """지금 경로나 임무를 시작하면 안 되는 이유 (HTTP 상태, 답), 괜찮으면 None. STATE.lock을 쥔 채 부른다.

    비밀번호, 다른 작업 중, 이미 주행 중, 로봇 감시기 버전(recovery_policy·line_algorithm·dock_protocol·
    final_gate_v1), 공유 구역 설정 일치, IR 보정, 주행준비 완료, 필수 토픽 수신, 위치 확정, 같은 지도를 차례로 본다."""
    if STATE.password is None:
        return 401, {"error": "SSH 비밀번호를 먼저 입력하세요."}
    if STATE.busy_robot is not None:
        return 409, {"error": f"Pinky {STATE.busy_robot} 작업이 끝날 때까지 기다려 주세요."}
    if (STATE.routes[robot_id] or {}).get('status') in {'starting','running','cancel_requested'}:
        return 409, {'error': '이 로봇은 이미 경로 주행 중입니다.'}
    telemetry = TELEMETRY.snapshot() if TELEMETRY is not None else {}
    robot_topics = telemetry.get(robot_id, {})
    required_live = ("map", "tf_pose", "scan", "odom", "ultrasonic", "ir", "line_control", "safety_status")
    live_ready = all(robot_topics.get(key, {}).get("fresh") for key in required_live)
    guard = robot_topics.get('line_control', {})
    guard_value = guard.get('value') or {}
    if (not guard.get('fresh') or guard_value.get('recovery_policy') != 'parking_mission_v2'
            or guard_value.get('line_algorithm') != 'fleet_control_v4'
            or guard_value.get('dock_protocol') != 'sponge_v1'):
        return 409, {'error': '새 주행 감시기를 적용하려면 Nav2 복구 또는 주행준비를 실행하세요.'}
    if (robot_topics.get('safety_status', {}).get('value') or {}).get('version') != 'final_gate_v1':
        return 409, {'error':'독립 최종 충돌 감시기를 적용하려면 주행준비를 실행하세요.'}
    if TRAFFIC.storage_error or guard_value.get('traffic_config') != TRAFFIC.zones:
        return 409, {'error':'공유 구역 설정·예약 저장 상태를 확인하고 주행준비를 다시 실행하세요.'}
    calibration_error = validate_profile(IR_PROFILES.get(robot_id))
    if _ir_steers(robot_id) and calibration_error:
        return 409, {'error': '차선 재획득·도킹 IR 판정을 위한 보정이 필요합니다: '+calibration_error}
    if STATE.robots[robot_id]["stage"] != "prepared":
        return 409, {"error": "업데이트된 Nav2 안전 설정과 초음파 감시기를 적용하려면 먼저 주행준비를 완료하세요."}
    if not live_ready:
        return 409, {"error": "지도·현재 map 위치·LiDAR·오도메트리·초음파가 모두 수신될 때까지 기다려 주세요."}
    if operational_status(STATE.robots[robot_id], robot_topics)['state'] == 'unlocalized':
        return 409, {"error": "지도 위치가 확정되지 않았습니다. 라이다로 위치 찾기(또는 후보 고르기)를 먼저 하세요."}
    active_peers = [rid for rid,r in STATE.routes.items() if rid != robot_id and
                    (r or {}).get('status') in {'starting','running','cancel_requested'}]
    if any(STATE.robots[rid]['map'] != STATE.robots[robot_id]['map'] for rid in active_peers):
        return 409, {'error': '동시 주행 로봇은 같은 공용 지도를 선택해야 합니다.'}
    return None


def _mark_pass_through(goals: list[dict[str, Any]]) -> None:
    """멈추지 않고 지나가기만 하는 기준점에 통과 반경(pass_radius_m)을 붙인다: 통로 점(10 cm), 마지막 목적지가
    아니라면 대기 자리(6 cm), 5번 앞 정렬 점(5 cm, 그곳에서는 핑키가 스스로 돈다)."""
    queue_label = (NAVIGATION.get("dock_queue") or {}).get("waypoint")
    for index, point in enumerate(goals):
        if not point.get("anchor") or index == len(goals) - 1:
            continue
        if point["label"] in {"통로 아래", "통로 위"}:
            point["pass_radius_m"] = 0.10
        elif queue_label and point["label"] == queue_label:
            point["pass_radius_m"] = 0.06
        elif point["label"] == "5 정면":
            # 이만하면 충분하다(2026-11-19): 스테이션 옆에서 Nav2가 3 cm와 방향을 맞추려고 몇 분씩 헤맸다.
            # 핑키가 도킹 방향으로는 스스로 돈다(DockEntry). navigation.json에서 0이면 예전처럼 정확히 선다.
            radius = float(NAVIGATION.get("dock_square_radius_m", 0.05))
            if radius > 0:
                point["pass_radius_m"] = min(radius, 0.08)


def _dock_correction(robot_id: str, offset: object) -> dict[str, Any] | None:
    """다시 넣기(redock)용: 핑키가 지금 서 있다고 믿는 곳(5번에 멈춰 있다)에서 천장 카메라 어긋남만큼 고친 정렬 점과
    적재 자리. 어긋남·위치·웨이포인트 중 쓸 수 없는 것이 있으면 None이고, 그러면 저장된 웨이포인트로 다시 들어간다."""
    pose = TELEMETRY.snapshot().get(robot_id, {}).get("tf_pose", {}) if TELEMETRY else {}
    value = pose.get("value") or {}
    dock = _shared_waypoint(robot_id, "5")
    if (not pose.get("fresh") or dock is None
            or not all(isinstance(value.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_rad"))):
        return None
    corrected = corrected_dock((value["x_m"], value["y_m"], value["yaw_rad"]), offset, dock, DOCK_APPROACH_M)
    if corrected is None:
        return None
    square, spot = corrected
    return {"square": {k: round(square[k], 4) for k in ("x_m", "y_m", "yaw_rad")},
            "dock": {k: round(spot[k], 4) for k in ("x_m", "y_m", "yaw_rad")}}


def _shared_waypoint(robot_id: str, label: str) -> dict[str, Any] | None:
    """이 로봇 지도의 웨이포인트 `label`: 자기 것을 먼저, 없으면 같은 지도에서 다른 핑키가 기록한 것
    (수령 구역은 거기 서 본 핑키 하나가 한 번만 기록한다)."""
    map_name = STATE.robots[robot_id]["map"]
    for rid in [robot_id] + [other for other in ROBOTS if other != robot_id]:
        for item in STATE.waypoints[rid]:
            if item.get("label") == label and item.get("map") == map_name and item.get("frame_id") == "map":
                return item
    return None


def _run_waypoint_route(robot_id: str, goals: list[dict[str, Any]]) -> None:
    """로봇의 Nav2 액션 클라이언트가 내는 웨이포인트별 진행을 읽어 STATE.routes에 옮긴다(WORKERS에서 돈다).

    SSH 연결을 주행이 끝날 때까지 열어 둔다(ServerAliveInterval 2초 × 3번이면 끊긴 것으로 본다). 줄마다 STATE.lock을
    잠깐 쥐고, 그사이 경로가 바뀌었으면(route is not route_ref: 새 주행이나 지우기) 그 줄은 버린다."""
    temp_dir = None
    # 새 주행은 자기 선을 그린다: 앞선 주행(멈췄든 끝났든)의 주행선과 경로를 먼저 지운다.
    if TELEMETRY is not None:
        TELEMETRY.forget(robot_id, "trail")
        TELEMETRY.forget(robot_id, "plan")
    with STATE.lock:
        route_ref=STATE.routes[robot_id]
        # 대기열에 있던 시작은 worker가 돌기 전에 취소될 수 있다.
        if not route_ref or route_ref.get('status') != 'starting':
            return
    goals[0]["run_id"]=route_ref.get("run_id") if route_ref else None
    terminal_status = "failed"
    try:
        temp_dir, env = _ssh_askpass_env()
        robot = ROBOTS[robot_id]
        args = [
            "ssh", "-o", "PreferredAuthentications=password,keyboard-interactive",
            "-o", "PubkeyAuthentication=no", "-o", "ConnectTimeout=6",
            "-o", "StrictHostKeyChecking=accept-new", "-o", "RequestTTY=no",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3",
            f"pinky@{robot['ip']}", _route_remote_command(robot_id, goals),
        ]
        process = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)
        with STATE.lock:
            route = STATE.routes[robot_id]
            if route is route_ref and route and route.get("status")=="starting":
                route.update(status="running", message="Nav2 경로 주행 중")
        assert process.stdout is not None
        for raw_line in process.stdout:
            line = raw_line.strip()
            with STATE.lock:
                route = STATE.routes[robot_id]
                if route is not route_ref or route is None or route.get("status")=="cancelled":
                    continue
                if line.startswith("WAYPOINT_START "):
                    fields = line.split(" ", 2)
                    if len(fields) == 3:
                        route.update(current_index=int(fields[1]), current_label=fields[2], message=f"웨이포인트 {fields[2]}로 이동 중")
                elif line.startswith("WAYPOINT_DONE "):
                    fields = line.split(" ", 2)
                    if len(fields) == 3:
                        at = int(fields[1])-1
                        verb = '도착' if goals[at].get('anchor') else '통과·좌표 기록'
                        route.update(completed=int(fields[1]), current_label=fields[2], message=f"웨이포인트 {fields[2]} {verb}")
                elif line.startswith("CHECKPOINT_POSE "):
                    route["last_check"] = json.loads(line.removeprefix("CHECKPOINT_POSE "))
                elif line.startswith("MISSION_STATUS "):
                    mission = json.loads(line.removeprefix("MISSION_STATUS "))
                    route.update(phase=mission.get('phase'), mission_token=mission.get('token'), message=mission.get('message', ''))
                elif line.startswith("ROUTE_DONE"):
                    terminal_status = "complete"
                elif line.startswith("ROUTE_CANCELLED"):
                    terminal_status = "cancelled"
                elif line.startswith("ROUTE_FAILED "):
                    terminal_status = "failed"
                    route["message"] = line.removeprefix("ROUTE_FAILED ")
                    route["detail"] = (route.get("detail", "") + line + "\n")[-1200:]
                elif line and not line.startswith("[INFO]"):
                    route["detail"] = (route.get("detail", "") + line + "\n")[-1200:]
        return_code = process.wait()
        if return_code != 0 and terminal_status == "failed":
            with STATE.lock:
                route = STATE.routes[robot_id]
                if route is route_ref and route:
                    route["detail"] = (route.get("detail", "") + f"\nSSH 종료 코드 {return_code}")[-1200:]
    except (OSError, subprocess.SubprocessError, TypeError, ValueError) as exc:
        with STATE.lock:
            route = STATE.routes[robot_id]
            if route is route_ref and route:
                route["detail"] = str(exc)
    finally:
        if temp_dir is not None:
            temp_dir.cleanup()
        with STATE.lock:
            route = STATE.routes[robot_id]
            if route is route_ref and route and route.get('status') not in {'cancelled','cancel_requested'}:
                route.update(status=terminal_status,
                             message={"complete": "임무 종료 · 자기 주차장 복귀 완료",
                                      "cancelled": "주행 취소를 요청했습니다.",
                                      "failed": route.get("message") if route.get("message", "").startswith("Nav2") else "경로 주행이 중단됐습니다."}[terminal_status])


def _cancel_waypoint_route(robot_id: str) -> None:
    """정지 worker: 로봇에서 stop_robot.py <run_id>를 돌려 Nav2 목표를 취소하고 정지를 걸어 둔다(STOP_LATCHED).
    확인되면 경로를 cancelled로 두고 자동 복귀 시각을 정한다. 확인되지 않으면 다시 누르도록 안내한다."""
    with STATE.lock:
        route_ref=STATE.routes[robot_id]
        run_id=(route_ref or {}).get("run_id") or ""
    payload=base64.b64encode((ROOT/'scripts/stop_robot.py').read_bytes()).decode()
    remote='/home/pinky/.cache/pinky-fleet/stop_robot.py'
    command=("mkdir -p /home/pinky/.cache/pinky-fleet && printf '%s' "+shlex.quote(payload)+
             " | base64 -d > "+remote+" && source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID="+
             str(ROBOTS[robot_id]['domain'])+" && python3 "+remote+" "+shlex.quote(run_id))
    ok,output=_ssh_run(robot_id,command,timeout_seconds=15.)
    with STATE.lock:
        route=STATE.routes[robot_id]
        if route is route_ref and route and route.get('status')=='cancel_requested':
            route['stop_pending']=False
            if ok and 'STOP_LATCHED' in output:
                # 정지는 깔끔하다(2026-11-16): 경로와 Nav2 목표가 사라지고 지도에서 경로선도 지워지지만, Nav2·AMCL
                # 위치·감시기는 그대로라 다음 임무는 주행준비 없이 여기서 시작한다(navigate_route.py가 시작할 때
                # 정지 걸림을 푼다). 공유 구역 예약은 허가가 끊기고 핑키가 구역 밖이면 저절로 풀리고, 구역 안에서
                # 멈췄으면 계속 잡고 있다. 새 명령 없이 auto_return_after_stop_s가 지나면 스스로 자기 주차 칸으로
                # 돌아간다. 한 번 더 정지(hold)하거나 기존 경로 날리기(clear)를 하면 취소되고, 지운 경로는 통째로 버린다.
                if route.get('clear_after_stop'):
                    STATE.routes[robot_id] = None
                    if TELEMETRY is not None:
                        TELEMETRY.forget(robot_id, 'plan')
                        TELEMETRY.forget(robot_id, 'trail')
                    return
                wait_s = _auto_return_seconds()
                hold = bool(route.get('hold')) or not wait_s or route.get('return_home')
                stopped_pose = (TELEMETRY.snapshot().get(robot_id, {}).get('tf_pose', {}).get('value') or {}) if TELEMETRY else {}
                route.update(status='cancelled', ready_again=True, message='정지했습니다 · 경로를 비웠습니다',
                             auto_return_at=None if hold else time.time() + wait_s, stopped_at=time.time(),
                             stopped_pose={k: stopped_pose.get(k) for k in ('x_m', 'y_m', 'yaw_rad')},
                             detail='Nav2 목표를 취소하고 정지 유지를 걸었습니다. 위치와 Nav2는 그대로라 주행준비 없이 바로 '
                                    '다시 출발할 수 있습니다.' + ('' if hold else f' {wait_s:.0f}초 동안 명령이 없으면 주차장으로 돌아갑니다.'))
                if TELEMETRY is not None:
                    TELEMETRY.forget(robot_id, 'plan')
            else:
                route.update(message='정지 적용 미확인 · 정지 버튼으로 재시도하세요.',detail=output[-1200:])


# ── 11) 정지·자동 복귀·경로 지우기 ──────────────────────────────────────────────────────


def _auto_return_seconds() -> float:
    """멈춘 핑키가 집으로 가기 전에 명령을 기다리는 시간(초). 0이면 가지 않는다(navigation.json, 기본 60초)."""
    value = NAVIGATION.get("auto_return_after_stop_s", 60)
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0 else 0.0


def _return_home_goals(robot_id: str, via: list[str]) -> tuple[list[dict[str, Any]] | None, tuple[int, dict] | None]:
    """핑키가 선 곳에서 `via`를 거쳐 자기 주차 칸까지 가는 (목표 목록, None), 가면 안 되면 (None, (상태, 답)).
    STATE.lock을 쥔 채 부른다. 칸에서 8 cm 안이면 이미 칸에 있는 것으로 본다."""
    problem = _route_start_problem(robot_id)
    if problem:
        return None, problem
    passes = [(label, _shared_waypoint(robot_id, label)) for label in via]
    missing = [label for label, item in passes if item is None]
    if missing:
        return None, (400, {"error": "저장되지 않은 웨이포인트: " + ", ".join(missing)})
    pose = (TELEMETRY.snapshot().get(robot_id, {}).get("tf_pose", {}).get("value") or {}) if TELEMETRY else {}
    if not all(isinstance(pose.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_rad")):
        return None, (409, {"error": "지금 지도 위치를 읽지 못했습니다."})
    map_name = STATE.robots[robot_id]["map"]
    parking = dict(PARKING[robot_id], map=map_name, frame_id="map", label="주차장")
    if math.hypot(pose["x_m"] - parking["x_m"], pose["y_m"] - parking["y_m"]) < 0.08:
        return None, (409, {"error": "이미 자기 주차 칸에 있습니다."})
    start = dict(label="출발", map=map_name, frame_id="map", x_m=pose["x_m"], y_m=pose["y_m"], yaw_rad=pose["yaw_rad"])
    dock = _shared_waypoint(robot_id, "5")
    if (NAVIGATION.get("dock_exit_reverse", True) is not False and dock
            and math.hypot(pose["x_m"] - dock["x_m"], pose["y_m"] - dock["y_m"]) < 0.12):
        # 5번 팔 사이에 서 있다(12 cm 안): 임무 때처럼 먼저 곧게 후진해 나온다(2026-11-17). 그 자리에서 제자리
        # 회전으로 나오지 않는다.
        start = dict(start, label="5", yaw_rad=dock["yaw_rad"])
    try:
        goals = build_mission_route([start] + [dict(item) for _, item in passes] + [parking], passages=NAVIGATION.get('wall_passages'), reverse_exit=NAVIGATION.get('dock_exit_reverse', True) is not False)
    except ValueError as exc:
        return None, (400, {"error": str(exc)})
    goals[0]["parking_yaw_rad"] = pose["yaw_rad"]
    goals[-1]["yaw_rad"] = parking["yaw_rad"]
    goals[-1]["parking_return"] = True
    for point in goals:
        point["use_ir"] = _ir_steers(robot_id)
    _mark_pass_through(goals)
    return goals, None


def _start_return_home(robot_id: str, via: list[str], why: str) -> tuple[int, dict]:
    """집으로 가는 주행을 시작한다. (HTTP 상태, 답). STATE.lock은 여기서 직접 잡는다."""
    with STATE.lock:
        goals, problem = _return_home_goals(robot_id, via)
        if problem:
            return problem
        labels = [point["label"] for point in goals]
        STATE.routes[robot_id] = {
            "status": "starting", "run_id": uuid.uuid4().hex, "labels": labels, "current_index": 0,
            "execution": "free_mission", "nav_goal_count": max(0, sum(bool(p.get("anchor")) for p in goals) - 1),
            "current_label": None, "completed": 0, "return_home": True, "via": via,
            "message": f"주차장 복귀 ({why})", "detail": "",
        }
    WORKERS.submit(_run_waypoint_route, robot_id, goals)
    return 202, {"ok": True, "robot": robot_id, "labels": labels}


def _auto_return_loop() -> None:
    """1초마다: 기다림이 끝난 멈춘 핑키를 집으로 보낸다. 갈 수 없으면(위치 없음, Nav2 꺼짐 등) 이유를 보이고
    그 자리에 둔다. 정지 뒤 5 cm 넘게 옮겨졌거나 15° 넘게 돌았거나 위치를 다시 잡았으면 보내지 않는다."""
    while not SHUTTING_DOWN.wait(1.0):
        now = time.time()
        due = []
        with STATE.lock:
            for rid, route in STATE.routes.items():
                if route and route.get("status") == "cancelled" and route.get("auto_return_at") and route["auto_return_at"] <= now:
                    route["auto_return_at"] = None
                    due.append(rid)
        for rid in due:
            # 정지 뒤 누가 손을 댔다(들어 옮겼거나 위치를 다시 넣었다): 스스로 어디로도 가지 않는다.
            with STATE.lock:
                route = STATE.routes[rid] or {}
                before = route.get("stopped_pose") or {}
                moved_by_hand = (STATE.robots[rid].get("localized_at") or 0) > (route.get("stopped_at") or 0)
            now_pose = (TELEMETRY.snapshot().get(rid, {}).get("tf_pose", {}).get("value") or {}) if TELEMETRY else {}
            if all(isinstance(before.get(k), (int, float)) and isinstance(now_pose.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_rad")):
                turned = abs(math.atan2(math.sin(now_pose["yaw_rad"] - before["yaw_rad"]), math.cos(now_pose["yaw_rad"] - before["yaw_rad"])))
                moved_by_hand = moved_by_hand or math.hypot(now_pose["x_m"] - before["x_m"], now_pose["y_m"] - before["y_m"]) > 0.05 or turned > math.radians(15)
            if moved_by_hand:
                with STATE.lock:
                    if STATE.routes[rid]:
                        STATE.routes[rid]["message"] = "정지 뒤 옮겨졌거나 위치를 다시 잡아서 자동 복귀하지 않았습니다."
                continue
            status, answer = _start_return_home(rid, [], "정지 후 1분 동안 명령 없음")
            if status != 202:
                with STATE.lock:
                    route = STATE.routes[rid]
                    if route:
                        route["message"] = "자동 주차장 복귀를 하지 못했습니다: " + str(answer.get("error"))
            JOURNAL.record("auto_return", robot_id=rid, state=str(status))


def _clear_route(robot_id: str) -> tuple[int, dict]:
    """기존 경로 날리기: 달리는 경로는 먼저 멈추고, 정지가 걸리면 버린다. 그 밖(멈춤, 실패, 끝남, 또는 꺼진 핑키에
    닿지 못한 정지)은 바로 버린다. STATE.lock은 여기서 직접 잡는다."""
    with STATE.lock:
        route = STATE.routes[robot_id]
        running = bool(route) and route.get("status") in {"starting", "running", "cancel_requested"}
        reachable = STATE.robots[robot_id].get("ssh_reachable", True)
        if running and (route.get("stop_pending") or route.get("status") != "cancel_requested" or reachable):
            if route.get("stop_pending"):
                route["clear_after_stop"] = True
                return 202, {"ok": True, "message": "정지가 확인되면 경로를 지웁니다."}
            route.update(status="cancel_requested", message="정지 후 경로를 지웁니다.", stop_pending=True,
                         clear_after_stop=True, auto_return_at=None)
            WORKERS.submit(_cancel_waypoint_route, robot_id)
            return 202, {"ok": True, "message": "정지하고 경로를 지웁니다."}
        STATE.routes[robot_id] = None
    if TELEMETRY is not None:
        TELEMETRY.forget(robot_id, "plan")
        TELEMETRY.forget(robot_id, "trail")
    # 공유 구역 안에서 꺼진 핑키는 그 구역을 영원히 잡고 있다(2026-11-17: P2가 통로에서 꺼졌고 P1이 뒤에서
    # 기다렸다). 꺼진 핑키가 아직 거기 서 있을 수 있어 저절로 풀지는 않는다. 경로를 지우는 것이 운영자가 "치웠다"고
    # 말하는 것이다. 살아 있는 핑키가 구역 안에 있으면 곧바로 다시 잡는다(물리적 점유).
    released = []
    live = (TELEMETRY.snapshot().get(robot_id, {}).get("tf_pose") or {}).get("fresh") if TELEMETRY is not None else False
    if not live:
        try:
            released = TRAFFIC.drop(robot_id)
        except OSError as exc:
            return 500, {"error": f"구역 예약을 풀지 못했습니다: {exc}"}
    note = f" 잡고 있던 구역({', '.join(released)})도 풀었습니다." if released else ""
    return 200, {"ok": True, "released": released, "message": "기존 경로·주행선·자동 복귀를 지웠습니다." + note}


def _reset_robot(robot_id: str, map_name: str) -> None:
    """초기 세팅: 핑키를 멈춘 채 붙잡고(stop_robot.py reset), 경로와 선을 지운 뒤, 준비돼 있어도 주행준비를
    처음부터 한다. 끝나면 busy_robot을 비운다."""
    try:
        payload = base64.b64encode((ROOT / 'scripts/stop_robot.py').read_bytes()).decode()
        remote = '/home/pinky/.cache/pinky-fleet/stop_robot.py'
        _ssh_run(robot_id, "mkdir -p /home/pinky/.cache/pinky-fleet && printf '%s' " + shlex.quote(payload) +
                 " | base64 -d > " + remote + " && source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=" +
                 str(ROBOTS[robot_id]['domain']) + " && python3 " + remote + " reset", timeout_seconds=15.)
        with STATE.lock:
            STATE.routes[robot_id] = None
        if TELEMETRY is not None:
            TELEMETRY.forget(robot_id, "plan")
            TELEMETRY.forget(robot_id, "trail")
        _prepare_robot(robot_id, map_name, release_busy=False)
    finally:
        with STATE.lock:
            if STATE.busy_robot == robot_id:
                STATE.busy_robot = None


# ── 12) 주행준비 전체 순서 ───────────────────────────────────────────────────────────────
# _prepare_robot: 기본 구동 정리 → 시계 맞춤 → bringup(라이다·오도메트리) → ADC 센서 → IMU → 센서 수신 확인 →
# 지도 복사 → 이전 Nav2 정리 → 감시기·안전 노드·Nav2 시작 → /map 확인 → Nav2 lifecycle 확인 → 라이다 위치 잡기.
# 여러 대는 _prepare_robots_in_turn이 한 대씩 차례로 한다(라즈베리파이·네트워크 부하 때문에 동시에 하지 않는다).
CLOCK_TOLERANCE_S = 1.0  # 로봇 시계가 PC와 이보다(초) 많이 다르면 맞춘다


def _sync_robot_clock(robot_id: str) -> tuple[bool, str]:
    """로봇 시계가 이 PC와 CLOCK_TOLERANCE_S보다 많이 다르면 PC 시각으로 맞춘다. (바꿨는가, 일어난 일)

    핑키에는 시계 배터리가 없고 현장에는 인터넷이 없어 기체마다 틀린 시각을 갖고 있다(2026-11-16에 10번은 석 달,
    12번은 열흘 늦었다). 그러면 이 PC에서 보낸 위치에 AMCL이 둘 곳을 모르는 시각이 찍히고, 기계마다의 로그를 맞춰
    볼 수 없다. 돌고 있는 스택이 시각이 튀는 것을 보지 않도록 Nav2를 먼저 멈추므로, 부르는 쪽은 Nav2를 새로 띄워야
    한다. 시각 설정은 sudo -S로 하고 비밀번호는 표준 입력으로 보낸다(0.3초는 SSH 왕복 지연 보정)."""
    ok, output = _ssh_run(robot_id, "date +%s.%N", redact=False)
    try:
        offset = float(output.strip().splitlines()[-1]) - time.time()
    except (ValueError, IndexError):
        return False, "로봇 시계를 읽지 못했습니다: " + output.strip()[-200:]
    if not ok:
        return False, "로봇 시계를 읽지 못했습니다."
    if abs(offset) <= CLOCK_TOLERANCE_S:
        return False, f"로봇 시계 차이 {offset:+.1f}초 · 그대로 둡니다."
    _stop_existing_navigation(robot_id)
    ok, output = _ssh_run(robot_id, f"sudo -S -p '' date -s @{time.time() + 0.3:.3f} >/dev/null 2>&1 && date +%s.%N",
                          send_password=True, redact=False)
    try:
        after = float(output.strip().splitlines()[-1]) - time.time()
    except (ValueError, IndexError):
        after = None
    if not ok or after is None:
        return False, f"로봇 시계가 {offset:+.0f}초 어긋났지만 맞추지 못했습니다: " + output.strip()[-200:]
    return True, f"로봇 시계를 PC에 맞췄습니다 ({offset:+.0f}초 → {after:+.1f}초)."


def _prepare_robot(robot_id: str, map_name: str, release_busy: bool = True) -> None:
    """주행준비 worker(12절 머리말의 순서). 실패하면 stage를 fault로 두고 이유를 error에 남긴다.
    `release_busy`가 False면 busy_robot은 부르는 쪽이 비운다(여러 대 차례 준비, 초기 세팅)."""
    prior = _last_confirmed_pose(robot_id)  # 기본 구동을 다시 띄우기 전에 지금 위치를 읽어 둔다
    with STATE.lock:
        STATE.robots[robot_id].update(stage="base_starting", map=map_name, message="기본 bringup과 LiDAR를 시작합니다.", error=None)
    try:
        cleared, clear_output = _stop_existing_base(robot_id)
        if not cleared or "BASE_PROCESSES_CLEARED" not in clear_output:
            raise RuntimeError("이전 기본 구동·LiDAR 프로세스를 정리하지 못했습니다: " + clear_output.strip())
        # ROS 노드를 띄우기 전에: 로봇 시계를 이 PC에 맞춘다(바꿔야 하면 돌고 있는 Nav2를 멈춘다).
        clock_changed, clock_note = _sync_robot_clock(robot_id)
        STATE.update(robot_id, clock=clock_note)
        # (단계 이름, 로봇에서 띄울 명령, PC에 들어와야 할 토픽, 오류에 쓸 이름)
        stages = [
            ("base", "ros2 launch pinky_bringup bringup_robot.launch.xml", ("scan", "odom"), "기본 구동·LiDAR·오도메트리"),
            ("adc", "ros2 run pinky_sensor_adc main_node", (), "IR·초음파·ADC 배터리"),
            ("imu", "ros2 run pinky_imu_bno055 main_node", (), "BNO055 IMU"),
        ]
        for stage, command, required, label in stages:
            if stage == "adc":
                STATE.update(robot_id, stage="sensors_starting", message="IR·초음파·배터리·IMU를 시작합니다.")
            ok, output = _ssh_run(robot_id, _remote_start_command(robot_id, stage, command))
            if not ok:
                raise RuntimeError(f"{label} 시작 실패: {output.strip() or 'SSH 오류'}")
            if required:
                ready, missing = _topic_ready(robot_id, required)
                if not ready:
                    names = ", ".join(missing)
                    _, diagnostics = _ssh_run(robot_id, _base_diagnostics_command(robot_id))
                    detail = diagnostics.strip()
                    if 'SCAN_SAMPLE_OK' in detail and 'ODOM_SAMPLE_OK' in detail:
                        raise RuntimeError(
                            f"로봇 내부 /scan·/odom 발행은 정상이나 PC 관제의 DDS 수신이 끊겼습니다 ({names}). "
                            "유선/Wi-Fi 재연결 후 기존 DDS 연결이 남아 있을 수 있습니다. "
                            "다른 기체의 주행이 모두 정지한 상태에서 PC 관제 서비스를 재시작한 뒤 연결 설정·주행준비를 다시 실행하세요. "
                            "명령: systemctl --user restart pinky-fleet.service"
                            + ("\n" + detail[-1400:] if detail else ""))
                    raise RuntimeError(f"{label} 데이터가 PC 관제에 수신되지 않습니다 ({names})." + ("\n" + detail[-1400:] if detail else ""))

        required_sensors = ("ir", "ultrasonic", "imu", "battery_percent", "battery_voltage")
        ready, missing = _topic_ready(robot_id, required_sensors)
        if not ready:
            raise RuntimeError("센서 데이터 확인 실패: " + ", ".join(missing))

        map_path = _remote_map_path(robot_id, map_name)
        if map_path is None:
            if map_name == "map-proto.yaml":
                raise RuntimeError("PC의 map-proto.yaml/map-proto.pgm을 로봇에 복사하지 못했습니다: "
                                   + MAP_INSTALL_ERRORS.get(robot_id, "이유 모름"))
            raise RuntimeError(f"로봇에서 {map_name} 파일을 /home/pinky 아래에서 찾지 못했습니다.")

        STATE.update(robot_id, stage="nav_starting", message="떠 있는 Nav2가 쓸 만한지 확인합니다.")
        # Nav2를 더는 살려 두지 않는다(2026-11-16): 위에서 기본 구동을 방금 다시 띄웠고, 그러면 돌던 AMCL은 모든
        # 스캔을 버리며(그 밑의 오도메트리와 TF가 처음부터 다시 시작한다) 위치도 받지 않는다. 합친 프로세스 하나는
        # 약 30초면 뜨므로 매번 깨끗한 Nav2가 더 안전한 선택이다. (그래서 keep은 늘 False이고 아래 keep 분기는 쓰이지 않는다.)
        keep, keep_note = False, clock_note
        if not keep:
            stopped, stop_output = _stop_existing_navigation(robot_id)
            if not stopped or "NAV_PROCESSES_CLEARED" not in stop_output:
                raise RuntimeError(f"이전 Nav2 실행을 정리하지 못했습니다: {stop_output.strip()}")

        STATE.update(robot_id, stage="nav_starting",
                     message=(f"{keep_note} 다시 켜지 않고 씁니다 ({map_name})." if keep else f"Nav2와 {map_name} 지도를 시작합니다."))
        ok, output = _start_navigation_with_safety(robot_id, map_path, map_name, keep_nav2=keep)
        if not ok:
            raise RuntimeError(f"Nav2 시작 실패: {output.strip() or 'SSH 오류'}")
        map_ready, missing = _topic_ready(robot_id, ("map",), timeout=25.0)
        if not map_ready:
            _, diagnostics = _ssh_run(robot_id, _nav_diagnostics_command(robot_id))
            detail = diagnostics.strip() or "Nav2 로그를 가져오지 못했습니다."
            raise RuntimeError("Nav2가 /map을 발행하지 않습니다. map_server 상태와 로그: " + detail[-1600:])
        action_ready, action_detail = _nav_ready_with_retry(robot_id, map_path, map_name)
        if not action_ready:
            _, diagnostics = _ssh_run(robot_id, _nav_diagnostics_command(robot_id))
            detail = (action_detail + "\n" + diagnostics).strip() or "Nav2 액션 진단을 가져오지 못했습니다."
            raise RuntimeError("Nav2가 /navigate_to_pose 액션 서버를 올리지 못했습니다: " + detail[-1800:])
        # 막 띄운 AMCL은 위치가 없으므로 로봇에 map 위치(tf_pose)가 없고 내보낼 수 없다.
        # 로봇이 자기 라이다로 지도에서 스스로를 찾는다.
        STATE.update(robot_id, message="라이다로 지도에서 자기 위치를 찾습니다.")
        located, detail = _localize_with_lidar(robot_id, prior=prior)
        if located:
            STATE.update(robot_id, stage="prepared", error=None,
                         message=f"센서·지도·Nav2를 확인했습니다 ({map_name}). {detail}")
        else:
            # Nav2는 떴으므로 stage는 prepared다. map 위치가 없으니 화면에는 여전히 준비 안 됨으로 보인다.
            STATE.update(robot_id, stage="prepared", error=detail[-1200:],
                         message="Nav2는 준비됐지만 라이다로 위치를 찾지 못했습니다. [라이다로 위치 찾기]를 다시 누르거나, "
                                 f"{PARKING[robot_id]['label']} 칸에 있으면 [주차장 좌표로 맞추기]를 쓰세요.")
    except Exception as exc:  # 운영자가 조치하도록 관제 화면에 보인다
        STATE.update(robot_id, stage="fault", message="준비 과정에서 오류가 발생했습니다.", error=str(exc))
    finally:
        with STATE.lock:
            if release_busy and STATE.busy_robot == robot_id:
                STATE.busy_robot = None


def _prepare_robots_in_turn(robot_ids: list[str], map_name: str) -> None:
    """여러 로봇을 busy 표시("all") 하나 아래에서 한 대씩 차례로 주행준비한다. 한 대가 실패해도 나머지는 계속한다."""
    try:
        for robot_id in robot_ids:
            _prepare_robot(robot_id, map_name, release_busy=False)
    finally:
        with STATE.lock:
            if STATE.busy_robot == "all":
                STATE.busy_robot = None


def _recover_navigation(robot_id: str) -> None:
    """[Nav2 복구] worker: 기본 구동과 센서는 두고 Nav2만 다시 띄운 뒤 라이다로 위치를 다시 찾는다."""
    map_name = STATE.robots[robot_id]["map"]
    prior = _last_confirmed_pose(robot_id)
    try:
        STATE.update(robot_id, stage="nav_starting", message="Nav2를 재시작하고 주행 액션을 확인합니다.", error=None)
        stopped, output = _stop_existing_navigation(robot_id)
        if not stopped or "NAV_PROCESSES_CLEARED" not in output:
            raise RuntimeError("기존 Nav2 실행을 종료하지 못했습니다: " + output.strip())
        map_path = _remote_map_path(robot_id, map_name)
        if map_path is None:
            raise RuntimeError(f"로봇에서 {map_name} 지도 경로를 확인하지 못했습니다.")
        ok, output = _start_navigation_with_safety(robot_id, map_path, map_name)
        if not ok:
            raise RuntimeError("Nav2 시작 실패: " + output.strip())
        map_ready, missing = _topic_ready(robot_id, ("map",), timeout=25.0)
        if not map_ready:
            raise RuntimeError("Nav2가 /map을 발행하지 않습니다: " + ", ".join(missing))
        action_ready, action_detail = _nav_ready_with_retry(robot_id, map_path, map_name)
        if not action_ready:
            _, diagnostics = _ssh_run(robot_id, _nav_diagnostics_command(robot_id))
            raise RuntimeError("Nav2 주행 액션이 준비되지 않았습니다: " + (action_detail + "\n" + diagnostics)[-1800:])
        # 다시 띄운 AMCL은 위치가 없다. 경로 주행 중에는 복구를 거절하므로 로봇은 멈춰 있고, 라이다가 멈춘 그
        # 자리에서 다시 찾을 수 있다.
        STATE.update(robot_id, message="Nav2가 다시 떴습니다. 라이다로 위치를 다시 찾습니다.")
        located, detail = _localize_with_lidar(robot_id, prior=prior)
        if located:
            STATE.update(robot_id, stage="prepared", error=None,
                         message=f"Nav2 주행 액션이 준비됐습니다. {detail}")
        else:
            STATE.update(robot_id, stage="prepared", error=detail[-1200:],
                         message="Nav2는 다시 떴지만 라이다로 위치를 찾지 못했습니다. 상세에서 후보를 골라 주세요.")
    except Exception as exc:
        STATE.update(robot_id, stage="fault", message="Nav2 복구에 실패했습니다.", error=str(exc))
    finally:
        with STATE.lock:
            if STATE.busy_robot == robot_id:
                STATE.busy_robot = None


# ── 13) 백그라운드 루프(구역 허가·양보·램프와 화면·SSH 상태) ─────────────────────────────


def _traffic_observations(telemetry):
    """TrafficCoordinator에 넘길 로봇별 관측: 위치, 요청한 구역과 token, 주차 칸에 멈춰 있는지.

    위치·행동 상태가 1초 안에 들어왔고 최종 감시기의 센서가 모두 살아 있을 때만 fresh다. fresh가 아니면 그
    로봇에는 새 허가를 주지 않고 잡은 구역도 풀지 않는다. 주차 칸 5 cm 안에서 거의 멈춰 있으면 at_home_and_stopped."""
    result={}
    for rid in ROBOTS:
        topics=telemetry.get(rid,{})
        position=topics.get('tf_pose',{});control=topics.get('line_control',{})
        p=position.get('value') or {};request=(control.get('value') or {}).get('traffic_request') or {}
        fresh=bool(position.get('fresh') and control.get('fresh') and position.get('age_s',99) is not None
                   and position.get('age_s',99)<1. and control.get('age_s',99) is not None and control.get('age_s',99)<1.)
        final=topics.get('safety_status',{})
        fresh=bool(fresh and final.get('fresh') and (final.get('value') or {}).get('sensors_fresh'))
        pose=[p.get('x_m'),p.get('y_m')]
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose):fresh=False;pose=[0.,0.]
        if not isinstance(request,dict):request={};fresh=False
        if not isinstance(request.get('zones',[]),list) or any(not isinstance(z,str) or z not in TRAFFIC.zones for z in request.get('zones',[])):request={};fresh=False
        route=STATE.routes[rid] or {};home=PARKING[rid]
        odom=topics.get('odom',{});v=odom.get('value') or {}
        at_home_and_stopped=bool(fresh and odom.get('fresh')
                    and all(isinstance(v.get(k),(int,float)) and math.isfinite(v[k]) for k in ('linear_mps','angular_rps'))
                    and math.hypot(pose[0]-home['x_m'],pose[1]-home['y_m'])<.05
                    and abs(v.get('linear_mps') or 0)<.005 and abs(v.get('angular_rps') or 0)<.02)
        result[rid]=dict(fresh=fresh,pose=pose,token=request.get('token'),zones=request.get('zones',[]),
                         parked=at_home_and_stopped and route.get('status')=='complete',
                         at_home_and_stopped=at_home_and_stopped and route.get('status') not in {'starting','running','cancel_requested'})
    return result


def _indicator_loop() -> None:
    """1초에 네 번, 핑키마다 램프 색(scripts/fleet_ops.indicator)과 지금 하는 일(scripts/fleet_ops.activity, 화면용)을
    그 로봇 도메인의 /fleet/indicator로 보낸다. 데몬 스레드라 종료 신호를 따로 보지 않는다."""
    from std_msgs.msg import String
    batteries = {rid: BatteryView() for rid in ROBOTS}
    while True:
        time.sleep(0.25)  # 2026-11-24부터 1초에 네 번. 화면이 바뀐 상태를 더 빨리 따라간다
        if TELEMETRY is None:
            continue
        try:
            telemetry = TELEMETRY.snapshot()
            with STATE.lock:
                wanted = {}
                for rid, info in STATE.robots.items():
                    topics = telemetry.get(rid, {})
                    operational = operational_status(info, topics)
                    reading = topics.get('battery_percent', {})
                    battery, low = batteries[rid].update((reading.get('value') or {}).get('percent')
                                                         if reading.get('fresh') else None)
                    behaviour = topics.get('line_control', {})
                    waiting = bool(behaviour.get('fresh')) and (behaviour.get('value') or {}).get('state') == 'traffic_wait'
                    # 램프는 색과 모드를 읽고, 화면(status_display.py)은 하는 일도 함께 보인다.
                    wanted[rid] = {**indicator(info, STATE.routes[rid], operational),
                                   **activity(info, STATE.routes[rid], operational, battery, low, waiting)}
            for rid, value in wanted.items():
                TELEMETRY.indicator_publishers[rid].publish(String(data=json.dumps(value)))
        except Exception as exc:  # 램프는 편의 기능이다. 이것 때문에 관제가 멈추는 일은 없어야 한다
            JOURNAL.record('indicator_error', state=str(exc))


def _yield_robots(telemetry):
    """scripts/yielding.YieldArbiter가 핑키마다 알아야 할 것을 관제 상태에서 모은다. STATE.lock을 쥔 채 부른다.
    protected(도킹 등 물러나면 안 되는 단계), passage(통로를 잡고 있음), loaded(5번을 지나 짐을 실음), queue(5번 대기 순서)."""
    robots={}
    passage=(TRAFFIC.owners.get('passage9') or {}).get('robot_id')
    for rid in ROBOTS:
        position=telemetry.get(rid,{}).get('tf_pose',{});p=position.get('value') or {}
        pose=[p.get('x_m'),p.get('y_m'),p.get('yaw_rad')]
        fresh=bool(position.get('fresh') and position.get('age_s') is not None and position.get('age_s')<1.
                   and all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose))
        route=STATE.routes[rid] or {}
        labels=route.get('labels') or []
        robots[rid]=dict(pose=tuple(pose) if fresh else None,fresh=fresh,
                         driving=route.get('status')=='running' and not route.get('stop_pending'),
                         protected=route.get('phase') in PROTECTED_PHASES,passage=passage==rid,
                         loaded='5' in labels and int(route.get('completed') or 0)>labels.index('5'),
                         queue=TRAFFIC.queue.get(rid))
    return robots


def _yield_tick(telemetry):
    """정면 마주침 판정 한 번: 지금 길을 비킬 핑키마다 지시를 /fleet/yield로 보내고, 누구에게 비키는지 그 경로에
    적어 둔다(화면이 보여 준다). 발행은 STATE.lock 밖에서 한다."""
    from std_msgs.msg import String
    with STATE.lock:
        orders=YIELD.update(_yield_robots(telemetry),time.monotonic()) if YIELD_ENABLED else {}
        for rid in ROBOTS:
            route=STATE.routes[rid]
            if not route:continue
            to=(orders.get(rid) or {}).get('to')
            if route.get('yield_to')!=to:
                route['yield_to']=to
                JOURNAL.record('yield',robot_id=rid,state=to or 'released',data=orders.get(rid))
    for rid,order in orders.items():
        try:TELEMETRY.yield_publishers[rid].publish(String(data=json.dumps(order)))
        except Exception as exc:JOURNAL.record('yield_publish_error',state=str(exc),robot_id=rid)


def _traffic_loop():
    """0.25초마다 구역 예약을 갱신하고, 주행 중인 핑키에 허가(/fleet/lease)를 보낸 뒤 양보 판정을 한 번 돈다.

    로봇은 허가가 1.5초 넘게 갱신되지 않으면 멈춘다. 그래서 관제가 죽거나 이 루프가 막히면 핑키가 선다.
    비밀번호가 없으면(관제를 막 다시 띄움) 허가를 보내지 않는다. STATE 값 몇 개를 잠금 없이 읽는다."""
    global TRAFFIC_SEQUENCE
    from std_msgs.msg import String
    while not SHUTTING_DOWN.wait(.25):
        if TELEMETRY is None:continue
        observations=_traffic_observations(TELEMETRY.snapshot())
        try:grants=TRAFFIC.update(observations)
        except (OSError,ValueError) as exc:
            JOURNAL.record('traffic_storage_error',state=str(exc));continue
        if STATE.password is None:continue
        TRAFFIC_SEQUENCE+=1
        for rid,zones in grants.items():
            token=observations[rid].get('token')
            if token and (STATE.routes[rid] or {}).get('status') in {'starting','running'}:
                payload=dict(token=token,zones=zones,boot=TRAFFIC_BOOT,sequence=TRAFFIC_SEQUENCE)
                try:TELEMETRY.traffic_publishers[rid].publish(String(data=json.dumps(payload)))
                except Exception as exc:JOURNAL.record('traffic_publish_error',state=str(exc),robot_id=rid)
        try:_yield_tick(TELEMETRY.snapshot())
        except Exception as exc:JOURNAL.record('yield_error',state=str(exc))  # 양보는 돕는 기능일 뿐, 허가를 멈출 이유가 아니다


def _health_loop() -> None:
    """3초마다 각 로봇의 SSH 포트(22)에 0.35초 안에 TCP로 붙어 보고 ssh_reachable을 갱신한다(로그인은 하지 않는다)."""
    while True:
        for robot_id, robot in ROBOTS.items():
            reachable = False
            try:
                with socket.create_connection((robot["ip"], 22), timeout=0.35):
                    reachable = True
            except OSError:
                pass
            STATE.update(robot_id, ssh_reachable=reachable)
        time.sleep(3.0)


# ── 14) HTTP API(Handler) ────────────────────────────────────────────────────────────────


class Handler(BaseHTTPRequestHandler):
    """관제 HTTP API와 웹 화면. 요청마다 스레드 하나에서 돈다(ThreadingHTTPServer). <id>는 10·11·12.

    GET  /api/status                    로봇별 상태·토픽·경로·구역 예약 전체(화면과 관제 서버가 1초마다 읽는다)
    GET  /api/robots/<id>/processes     로봇 프로세스 목록(개발용, 읽기만)
    GET  /api/robots/<id>/nav-log       Nav2 로그 요약(개발용, 읽기만)
    GET  /api/robots/<id>/locate        라이다 위치 후보(AMCL은 건드리지 않음)
    GET  /api/robots/<id>/route-preview.csv, waypoints.csv   경로 미리보기·웨이포인트 내려받기
    GET  /api/ir-calibration/<id>       IR 보정 수집 기록
    POST /api/credentials               SSH 비밀번호(메모리에만)
    POST /api/robots/<id>/ready, /api/prepare-all, /reset, /nav/restart   주행준비·초기 세팅·Nav2 복구
    POST /api/robots/<id>/localize, /set-pose, /initial-pose              위치 잡기 세 가지
    POST /api/robots/<id>/mission, /route, /return-home                   임무·경로·귀환 시작
    POST /api/robots/<id>/route/cancel, /route/clear                      정지, 경로 지우기
    POST /api/robots/<id>/operator-confirm, /mission-complete, /docking   운영자 신호(token 확인 뒤 로봇에 파일로 씀)
    POST /api/robots/<id>/dock-guide    천장 카메라 안내 중계(/fleet/dock_guide)
    POST /api/robots/<id>/ir, /api/ir-calibration/<id>/capture            IR 보정
    POST /api/robots/<id>/waypoints, /display/reload, /api/traffic/release, /api/reset
    DELETE /api/robots/<id>/waypoints/<32자 id>

    오래 걸리는 일은 WORKERS에 넘기고 202로 바로 답한다. 결과는 /api/status의 message·error로 본다.
    """
    server_version = "PinkyFleet/0.1"

    def log_message(self, fmt: str, *args: Any) -> None:
        # API 본문은 기록하지 않는다. 실행 중 SSH 비밀번호가 들어 있을 수 있다(요청 줄만 찍는다).
        if '/dock-guide' in str(args[0] if args else ''):
            return  # 핑키가 들어가는 동안 1초에 여러 번 온다: 하나마다 한 줄씩 찍지 않는다
        print(f"{self.log_date_time_string()} {self.address_string()} {fmt % args}")

    def _json(self, status: int, body: dict[str, Any]) -> None:
        """JSON 답을 보낸다. NaN이 섞이면 allow_nan=False 때문에 예외가 나므로 값은 _finite로 거른다."""
        payload = json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _body(self) -> dict[str, Any]:
        """요청 본문 JSON(최대 8 KiB까지만 읽는다). dict가 아니면 빈 dict."""
        length = min(int(self.headers.get("Content-Length", "0")), 8192)
        raw = self.rfile.read(length) if length else b"{}"
        data = json.loads(raw.decode("utf-8"))
        return data if isinstance(data, dict) else {}

    def do_GET(self) -> None:  # noqa: N802
        path = unquote(self.path.split("?", 1)[0])
        calibration_match = re.fullmatch(r'/api/ir-calibration/(10|11|12)', path)
        if calibration_match:
            try:
                self._json(200, {'records': IR_CALIBRATION.records(calibration_match.group(1))})
            except (OSError, ValueError) as exc:
                self._json(500, {'error': f'수집 기록 읽기 실패: {exc}'})
            return
        processes_match = re.fullmatch(r"/api/robots/(10|11|12)/processes", path)
        if processes_match:
            # 읽기만, 개발용: 로봇에서 도는 것(ROS, Nav2, Python)을 CPU를 많이 쓰는 순으로 보인다.
            ok, output = _ssh_run(processes_match.group(1),
                                  "uptime; ps -eo pid,etime,pcpu,pmem,args --sort=-pcpu | grep -E 'ros|nav2|python3|PID' | grep -v grep | cut -c1-220 | head -60",
                                  keep_chars=12000)
            self._json(200 if ok else 502, {"ok": ok, "output": output})
            return
        nav_log_match = re.fullmatch(r"/api/robots/(10|11|12)/nav-log", path)
        if nav_log_match:
            # 읽기만, 개발용: Nav2가 왜 안 떴는지. 반복되는 keepout TF 경고는 뺀다.
            log = NAV_LOG_PATH
            ok, output = _ssh_run(nav_log_match.group(1),
                                  f"ls -la {log}; date; grep -anE 'ERROR|FATAL|died|exit code|Timed out|timed out|Failed|terminate|Aborting|Activating|Configuring|Waiting' {log} "
                                  "| grep -v KeepoutFilter | tail -120 | cut -c1-260; echo '--- 마지막 40줄 ---'; "
                                  f"tail -40 {log} | grep -av KeepoutFilter | cut -c1-260", keep_chars=40000)
            self._json(200 if ok else 502, {"ok": ok, "output": output})
            return
        locate_match = re.fullmatch(r"/api/robots/(10|11|12)/locate", path)
        if locate_match:
            # 읽기만: 라이다가 지금 로봇을 어디에 두는지와 그 밖에 맞는 자리들. AMCL은 건드리지 않는다.
            match, why_not = _lidar_match(locate_match.group(1), time.time() - 1.0)
            if match is None:
                self._json(409, {"error": why_not})
            else:
                self._json(200, {"robot": locate_match.group(1), "match": match.as_dict()})
            return
        if path == "/api/status":
            # 토픽 복사본은 STATE.lock 밖에서 먼저 뜨고, 로봇 상태는 잠금 안에서 한 번에 모은다.
            telemetry = TELEMETRY.snapshot() if TELEMETRY is not None else {}
            with STATE.lock:
                robots = []
                for robot_id, info in STATE.robots.items():
                    topics = telemetry.get(robot_id, {})
                    anchors = list(STATE.waypoints[robot_id])
                    try:
                        preview = build_dense_route(anchors)
                    except (KeyError, ValueError):
                        preview = []
                    profile = IR_PROFILES.get(robot_id)
                    raw = (topics.get('ir', {}).get('value') or {}).get('channels', [])
                    profile_view = {**profile, 'calibration_error': validate_profile(profile), 'normalized': normalize(raw, profile)}
                    robots.append({**info, "topics": topics, "waypoints": anchors, "route_preview": preview,
                                   "operational": operational_status(info,topics), "traffic": TRAFFIC.snapshot(), "ir_profile": profile_view, "parking": PARKING[robot_id], "route": STATE.routes[robot_id]})
                self._json(200, {"robots": robots, "busy_robot": STATE.busy_robot, "password_set": STATE.password is not None,
                                 "monitor_error": STATE.monitor_error, "keepout": keepout_view(), "traffic": TRAFFIC.snapshot()})
            return
        preview_match = re.fullmatch(r"/api/robots/(10|11|12)/route-preview\.csv", path)
        if preview_match:
            robot_id = preview_match.group(1)
            with STATE.lock:
                try:
                    points = build_dense_route(STATE.waypoints[robot_id])
                except (KeyError, ValueError) as exc:
                    self._json(409, {"error": str(exc)})
                    return
            output = io.StringIO()
            writer = csv.writer(output)
            writer.writerow(["seq", "name", "map", "frame", "x_m", "y_m", "yaw_rad", "anchor", "motion"])
            for seq, p in enumerate(points, 1):
                writer.writerow([seq, p["label"], p["map"], p["frame_id"], p["x_m"], p["y_m"], p["yaw_rad"], p["anchor"], "reverse" if p.get("reverse") else "forward"])
            payload = ("\ufeff" + output.getvalue()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", f'attachment; filename="pinky_{robot_id}_route_10cm.csv"')
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        csv_match = re.fullmatch(r"/api/robots/(10|11|12)/waypoints\.csv", path)
        if csv_match:
            robot_id = csv_match.group(1)
            with STATE.lock:
                entries = list(STATE.waypoints[robot_id])
            buffer = io.StringIO(newline="")
            writer = csv.writer(buffer)
            writer.writerow(["name", "map", "frame", "x_m", "y_m", "yaw_rad", "recorded_at"])
            for item in entries:
                writer.writerow([item.get("label"), item.get("map"), item.get("frame_id"), item.get("x_m"), item.get("y_m"), item.get("yaw_rad"), item.get("recorded_at")])
            payload = ("\ufeff" + buffer.getvalue()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", f'attachment; filename="pinky_{robot_id}_waypoints.csv"')
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
            return
        # 웹 화면 파일. 정해진 이름만 내보내므로 경로를 타고 다른 파일을 읽을 수 없다.
        filename = {"/": "index.html", "/app.js": "app.js", "/styles.css": "styles.css",
                    '/ir-calibration': 'ir-calibration.html', '/ir-calibration.js': 'ir-calibration.js',
                    '/ir-calibration.css': 'ir-calibration.css'}.get(path)
        if filename is None:
            self._json(404, {"error": "not found"})
            return
        content = (WEB / filename).read_bytes()
        content_type = "text/html; charset=utf-8" if filename.endswith(".html") else ("text/javascript; charset=utf-8" if filename.endswith(".js") else "text/css; charset=utf-8")
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def do_POST(self) -> None:  # noqa: N802
        """POST 경로를 위에서부터 차례로 맞춰 본다(클래스 docstring의 목록)."""
        path = unquote(self.path.split("?", 1)[0])
        try:
            data = self._body()
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
            self._json(400, {"error": "invalid JSON"})
            return
        # 운영자가 구역 예약을 손으로 푼다. 최신 위치로 구역 밖이거나 주차 칸에 멈춰 있을 때만 된다(traffic.release).
        if path=='/api/traffic/release':
            try:
                rid=str(data.get('robot_id'))
                if rid not in ROBOTS:raise ValueError('로봇을 확인하세요.')
                observation=_traffic_observations(TELEMETRY.snapshot() if TELEMETRY else {}).get(rid)
                TRAFFIC.release(data.get('zone'),rid,data.get('token'),observation)
                self._json(200,{'ok':True})
            except (ValueError,OSError) as exc:self._json(409,{'error':str(exc)})
            return
        # IR 도킹 화면의 [도킹완료 확인]/[수령 완료]. 로봇의 dock_command.json에 쓰고, 로봇이 line/status의
        # docking.command_id로 받았다고 알려 올 때까지 최대 4초 기다린다. 로봇은 3초 지난 명령을 버린다
        # (monotonic은 로봇 자신의 시계로 쓴 시각이다).
        docking_match = re.fullmatch(r'/api/robots/(10|11|12)/docking', path)
        if docking_match:
            robot_id = docking_match.group(1)
            action = data.get('action')
            if action not in ('complete', 'received'):
                self._json(400, {'error': '도킹 작업은 complete 또는 received여야 합니다.'})
                return
            if not DOCK_COMMAND_LOCK.acquire(blocking=False):
                self._json(409, {'error': '이전 도킹 명령의 적용 확인을 기다려 주세요.'})
                return
            code, result = 200, {}
            try:
                with STATE.lock:
                    route = STATE.routes[robot_id] or {}
                    if STATE.password is None:
                        raise ValueError('SSH 비밀번호를 먼저 입력하세요.')
                    if route.get('status') != 'running':
                        raise ValueError('진행 중인 해당 기체의 도킹 시나리오에서만 사용할 수 있습니다.')
                entry = TELEMETRY.snapshot().get(robot_id, {}).get('line_control', {}) if TELEMETRY else {}
                control = entry.get('value') or {}
                dock = control.get('docking') or {}
                if not data.get('dock_id') or data.get('dock_id') != dock.get('dock_id'):
                    raise ValueError('도킹 회차가 바뀌었습니다. 화면을 새로고침한 뒤 다시 눌러 주세요.')
                allowed = {'docking','await_confirmation','recovery_hold'} if action == 'complete' else ({'stopped'} if dock.get('anchored') else set())
                if (not entry.get('fresh') or not control.get('ir_fresh') or not control.get('lidar_fresh')
                        or not control.get('ultrasonic_fresh')
                        or dock.get('state') not in allowed or not dock.get('dock_id') or not dock.get('token')):
                    raise ValueError('현재 도킹 상태와 최신 센서 수신을 확인한 뒤 다시 눌러 주세요.')
                command_id = uuid.uuid4().hex
                command = dict(id=command_id, token=dock['token'], dock_id=dock['dock_id'], action=action)
                encoded = base64.b64encode(json.dumps(command).encode()).decode()
                program = ("import base64,json,time,os; from pathlib import Path; "
                           f"d=json.loads(base64.b64decode({encoded!r})); d['monotonic']=time.monotonic(); "
                           "p=Path('/home/pinky/.cache/pinky-fleet/dock_command.json'); t=p.with_suffix('.tmp'); "
                           "t.write_text(json.dumps(d)); os.replace(t,p); print('DOCK_COMMAND_SAVED')")
                ok, output = _ssh_run(robot_id, 'python3 -c '+shlex.quote(program), timeout_seconds=8.)
                if not ok or 'DOCK_COMMAND_SAVED' not in output:
                    raise OSError(output or '도킹 명령 전송 실패')
                deadline = time.monotonic()+4.
                acknowledged = None
                while time.monotonic() < deadline:
                    status = TELEMETRY.snapshot().get(robot_id, {}).get('line_control', {}) if TELEMETRY else {}
                    candidate = (status.get('value') or {}).get('docking') or {}
                    if status.get('fresh') and candidate.get('command_id') == command_id:
                        acknowledged = candidate
                        break
                    time.sleep(.08)
                if acknowledged is None:
                    raise OSError('명령 적용 확인을 받지 못했습니다. 현재 도킹 상태를 확인하세요. 지연 명령은 3초 후 만료됩니다.')
                if acknowledged.get('command_error'):
                    raise ValueError(acknowledged['command_error'])
                result = {'ok': True, 'docking': acknowledged, 'message':
                          '도킹완료 확인 신호를 전달했습니다. 흰색 유지와 5번 위치 재설정 확인 후 수령 대기로 전환합니다.' if action == 'complete'
                          else '물건 수령 완료를 확인했습니다. 시나리오를 재개합니다.'}
            except ValueError as exc:
                code, result = 409, {'error': str(exc)}
            except OSError as exc:
                code, result = 502, {'error': str(exc)}
            finally:
                DOCK_COMMAND_LOCK.release()
            self._json(code, result)
            return
        # IR 보정 수집: 바퀴가 멈춰 있는 동안 요청 뒤에 들어온 새 IR 샘플 20개를 5초 안에 모아 저장한다.
        # 로봇은 움직이지 않는다. 그동안 busy_robot을 잡아 주행·준비 작업과 겹치지 않게 한다.
        calibration_match = re.fullmatch(r'/api/ir-calibration/(10|11|12)/capture', path)
        if calibration_match:
            robot_id = calibration_match.group(1)
            offset = _finite(data.get('offset_cm'))
            session = data.get('session')
            if (offset is None or abs(offset) > 10 or isinstance(data.get('offset_cm'), bool)
                    or not isinstance(session, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', session)):
                self._json(400, {'error': '오프셋은 -10~10cm, 세션은 영문·숫자·밑줄·하이픈으로 지정하세요.'})
                return
            with STATE.lock:
                active = any((route or {}).get('status') in ('starting', 'running', 'cancel_requested')
                             for route in STATE.routes.values())
                if STATE.busy_robot is not None or active:
                    self._json(409, {'error': '주행·준비 작업을 정지한 뒤 수집하세요.'})
                    return
                STATE.busy_robot = robot_id
            status, result = 200, {}
            try:
                if TELEMETRY is None:
                    raise ValueError('ROS 센서 수신기가 준비되지 않았습니다.')
                started, deadline = time.time(), time.monotonic()+5.0
                samples = []
                while time.monotonic() < deadline:
                    topics = TELEMETRY.snapshot().get(robot_id, {})
                    odom = topics.get('odom', {})
                    velocity = odom.get('value') or {}
                    speed, rotation = _finite(velocity.get('linear_mps')), _finite(velocity.get('angular_rps'))
                    if not topics.get('ir', {}).get('fresh') or not odom.get('fresh'):
                        raise ValueError('최신 IR·오도메트리 수신이 필요합니다. 관제에서 센서를 준비하세요.')
                    if speed is None or rotation is None or abs(speed) > .002 or abs(rotation) > .02:
                        raise ValueError('바퀴 움직임이 감지됐습니다. 완전히 정지한 뒤 다시 수집하세요.')
                    with TELEMETRY.lock:
                        samples = [{'received_at': at, 'raw': list(row)}
                                   for at, row in TELEMETRY.ir_history[robot_id] if at > started][:20]
                    if len(samples) == 20:
                        break
                    time.sleep(.04)
                if len(samples) != 20:
                    raise ValueError(f'5초 안에 새 샘플 20개를 받지 못했습니다 ({len(samples)}개). 저장하지 않았습니다.')
                record = IR_CALIBRATION.append(robot_id, session, offset, samples)
                result = {'ok': True, 'record': record}
            except ValueError as exc:
                status, result = 409, {'error': str(exc)}
            except OSError as exc:
                status, result = 500, {'error': f'수집 기록 저장 실패: {exc}'}
            finally:
                with STATE.lock:
                    STATE.busy_robot = None
            self._json(status, result)
            return
        # IR 보정값 저장(floor/line: PC에만, 최근 0.8초 샘플로) 또는 IR 보조 켜기·끄기(enable/disable: 로봇에
        # 보정 파일을 쓰고, line/status의 revision이 바뀌어 적용을 알려 올 때까지 최대 6초 기다린다).
        ir_match = re.fullmatch(r"/api/robots/(10|11|12)/ir", path)
        if ir_match:
            robot_id = ir_match.group(1)
            action = data.get('action')
            if action not in ('floor', 'line', 'enable', 'disable'):
                self._json(400, {'error': 'IR 작업을 확인하세요.'})
                return
            with STATE.lock:
                if STATE.busy_robot is not None:
                    self._json(409, {'error': '현재 로봇 작업이 끝난 뒤 IR 설정을 변경하세요.'})
                    return
                route = STATE.routes[robot_id] or {}
                if route.get('status') in ('running', 'starting', 'cancel_requested'):
                    self._json(409, {'error': '경로 주행을 정지한 뒤 IR 설정을 변경하세요.'})
                    return
                profile = IR_PROFILES.get(robot_id)
                telemetry = TELEMETRY.snapshot().get(robot_id, {}) if TELEMETRY else {}
                control = telemetry.get('line_control', {})
                actual = control.get('value') or {}
                if action in ('floor', 'line'):
                    if profile.get('mode') == 'template':
                        self._json(409, {'error': '횡방향 CSV 모델 사용 중입니다. 새 CSV는 캘리브레이션 수집 페이지에서 수집하세요.'})
                        return
                    if profile.get('enabled') or (control.get('fresh') and actual.get('enabled')):
                        self._json(409, {'error': 'IR 보조를 끈 뒤 보정하세요.'})
                        return
                    channel = data.get('channel')
                    if action == 'line' and (type(channel) is not int or channel not in (0, 1, 2)):
                        self._json(400, {'error': 'CH1~CH3 중 보정 채널을 선택하세요.'})
                        return
                    samples = []
                    if TELEMETRY:
                        with TELEMETRY.lock:
                            samples = [row for at, row in TELEMETRY.ir_history[robot_id] if time.time()-at <= .8]
                    try:
                        candidate = capture_profile(profile, samples, action, channel)
                        candidate['revision'] = uuid.uuid4().hex
                        IR_PROFILES.save(robot_id, candidate)
                    except (OSError, ValueError) as exc:
                        self._json(400, {'error': str(exc)})
                        return
                    self._json(200, {'ok': True, 'message': '실측 IR 보정값을 저장했습니다.'})
                    return
                if STATE.password is None:
                    self._json(401, {'error': 'SSH 비밀번호를 먼저 입력하세요.'})
                    return
                if action == 'enable':
                    reason = validate_profile(profile)
                    if reason:
                        self._json(409, {'error': reason})
                        return
                    if STATE.robots[robot_id]['stage'] != 'prepared' or not control.get('fresh') or actual.get('version') != 1:
                        self._json(409, {'error': '주행준비를 완료해 IR 통합 감시기를 먼저 시작하세요.'})
                        return
                candidate = {**profile, 'enabled': action == 'enable', 'revision': uuid.uuid4().hex}
                STATE.busy_robot = robot_id
            try:
                ok, output = _ssh_run(robot_id, _ir_profile_install_command(candidate))
                if not ok or 'IR_PROFILE_SAVED' not in output:
                    self._json(502, {'error': output or 'IR 설정 전송에 실패했습니다.'})
                    return
                with STATE.lock:
                    IR_PROFILES.save(robot_id, candidate)
                acknowledged = False
                deadline = time.monotonic()+6
                while TELEMETRY and time.monotonic() < deadline:
                    entry = TELEMETRY.snapshot().get(robot_id, {}).get('line_control', {})
                    if entry.get('fresh') and (entry.get('value') or {}).get('revision') == candidate['revision']:
                        acknowledged = True
                        break
                    time.sleep(.1)
                if not acknowledged:
                    self._json(503, {'error': 'IR 설정은 저장됐으나 로봇의 적용 확인을 받지 못했습니다. 상태 표시를 확인하세요.'})
                    return
                self._json(200, {'ok': True, 'message': 'IR 차선 보조를 켰습니다.' if candidate['enabled'] else 'IR 차선 보조를 껐습니다.'})
            except OSError as exc:
                self._json(500, {'error': str(exc)})
            finally:
                with STATE.lock:
                    STATE.busy_robot = None
            return
        if path == "/api/credentials":
            password = data.get("password")
            if not isinstance(password, str) or not password:
                self._json(400, {"error": "비밀번호를 입력하세요."})
                return
            with STATE.lock:
                STATE.password = password
            self._json(200, {"ok": True, "message": "비밀번호를 실행 중인 프로세스 메모리에만 보관합니다."})
            return
        # 전체 초기화: 세 대에서 관제가 띄운 프로세스와 키보드 조작을 멈춘다(_reset_all_robots).
        if path == "/api/reset":
            with STATE.lock:
                if any((r or {}).get('status') in {'starting','running','cancel_requested'} for r in STATE.routes.values()):
                    self._json(409, {'error': '각 로봇의 경로를 정지한 뒤 전체 초기화하세요.'})
                    return
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                if STATE.busy_robot is not None:
                    self._json(409, {"error": "현재 로봇 작업이 끝난 뒤 초기화해 주세요."})
                    return
                STATE.busy_robot = "all"
                for robot_id in ROBOTS:
                    STATE.robots[robot_id].update(
                        stage="stopping",
                        message="대시보드 실행 노드와 키보드 조작을 종료합니다.",
                        error=None,
                    )
            WORKERS.submit(_reset_all_robots)
            self._json(202, {"ok": True, "message": "세 핑키 초기화를 시작했습니다."})
            return
        guide_match = re.fullmatch(r'/api/robots/(10|11|12)/dock-guide', path)
        if guide_match:
            # 천장 카메라가 실시간으로 본 이 핑키의 적재 자리 대비 어긋남(핑키가 화면에 보이는 동안 관제 서버가
            # 1초에 몇 번 보낸다). 그대로 /fleet/dock_guide로 핑키에 보내고, 핑키에서는 5번으로 들어가는 주행만
            # 이것으로 조향한다(mission_behaviors.DockEntry). navigation.json의 dock_camera_guide가 false면 중계를
            # 끄고, 핑키는 예전처럼 오도메트리로 들어간다.
            if not NAVIGATION.get('dock_camera_guide', True):
                self._json(409, {'error': '카메라 안내가 꺼져 있습니다 (navigation.json dock_camera_guide).'})
                return
            if TELEMETRY is None:
                self._json(503, {'error': 'ROS 연결이 아직 준비되지 않았습니다.'})
                return
            guide = clean_dock_guide(data)
            if guide is None:
                self._json(400, {'error': '안내 값은 앞 50 cm, 옆 25 cm 안의 숫자 세 개(앞, 왼쪽, 각도)여야 합니다.'})
                return
            from std_msgs.msg import String
            try:
                TELEMETRY.dock_guide_publishers[guide_match.group(1)].publish(String(data=json.dumps(guide)))
            except Exception as exc:
                self._json(500, {'error': f'안내를 보내지 못했습니다: {exc}'})
                return
            self._json(200, {'ok': True})
            return
        operator_match = re.fullmatch(r'/api/robots/(10|11|12)/operator-confirm', path)
        if operator_match:
            robot_id=operator_match.group(1)
            # 관제 서버가 보내는 운영자 신호. 임무 단계(phase)와 mission_token이 지금 것과 맞을 때만 로봇의
            # operator_confirm.json에 쓴다(SSH). 202는 "써 넣었다"는 뜻이고, 핑키가 움직였는지는 다음 phase로 본다.
            # redock: 천장 카메라가 핑키가 적재 자리에서 벗어났다고 판정했다. 물러났다가 다시 들어간다.
            # redock은 dock_complete 뒤(docked_manual)에도 적재가 확인될 때까지 받는다(2026-11-19).
            phases={'dock_complete':('docking_manual',),'redock':('docking_manual','docked_manual'),'received':('docked_manual',)}
            action=data.get('action')
            with STATE.lock:
                route=STATE.routes[robot_id] or {}
                token=route.get('mission_token')
                if action not in phases or route.get('status')!='running' or route.get('phase') not in phases[action] or not token or data.get('token')!=token:
                    self._json(409,{'error':'현재 임무의 확인 단계와 신호가 일치하지 않습니다.'})
                    return
            control=TELEMETRY.snapshot().get(robot_id,{}).get('line_control',{}) if TELEMETRY else {}
            if not control.get('fresh'):
                self._json(409,{'error':'안전 감시기 수신을 확인하세요.'});return
            command={'token':token,'action':action}
            note=''
            if action=='redock' and data.get('offset') is not None:
                correction=_dock_correction(robot_id,data.get('offset'))
                if correction:
                    command['dock_correction']=correction
                    note=' 카메라 오프셋만큼 고쳐서 다시 들어갑니다.'
                else:
                    note=' 카메라 오프셋은 쓰지 못해 같은 자리로 다시 들어갑니다.'
            payload=base64.b64encode(json.dumps(command).encode()).decode()
            program="import base64,json,time,os;from pathlib import Path;d=json.loads(base64.b64decode('"+payload+"'));d['monotonic']=time.monotonic();p=Path('/home/pinky/.cache/pinky-fleet/operator_confirm.json');t=p.with_suffix('.tmp');t.write_text(json.dumps(d));os.replace(t,p)"
            ok,output=_ssh_run(robot_id,'python3 -c '+shlex.quote(program))
            self._json(202 if ok else 502,{'ok':ok,'guided':'dock_correction' in command,'message':'현재 단계 확인 신호를 보냈습니다.'+note if ok else output});return
        # 수령 구역에서 주행완료(course_complete 단계): token이 맞으면 로봇의 mission_complete.json에 써서 귀환시킨다.
        completion_match = re.fullmatch(r'/api/robots/(10|11|12)/mission-complete', path)
        if completion_match:
            robot_id=completion_match.group(1)
            with STATE.lock:
                route=STATE.routes[robot_id] or {}
                token=route.get('mission_token')
                if route.get('status') != 'running' or route.get('phase') != 'course_complete' or not token or data.get('token') != token:
                    self._json(409, {'error':'코스 완주 후 현재 임무의 주행완료 신호만 처리할 수 있습니다.'})
                    return
            payload=base64.b64encode(json.dumps({'token':token}).encode()).decode()
            command="python3 -c "+shlex.quote("import base64,json,time,os; from pathlib import Path; p=Path('/home/pinky/.cache/pinky-fleet/mission_complete.json'); d=json.loads(base64.b64decode('"+payload+"')); d['monotonic']=time.monotonic(); t=p.with_suffix('.tmp'); t.write_text(json.dumps(d)); os.replace(t,p)")
            ok,output=_ssh_run(robot_id,command)
            self._json(202 if ok else 502,{'ok':ok,'message':'주행완료 신호를 전송했습니다. 자기 주차장으로 복귀합니다.' if ok else output})
            return
        mission_match = re.fullmatch(r"/api/robots/(10|11|12)/mission", path)
        if mission_match:
            # 주문 없는 시험 임무(관제 서버의 주문 흐름도 이것을 쓴다): 핑키가 선 곳에서 5번(정지, 도킹, 적재), 다음
            # 배달 웨이포인트(정지, 넘겨줌), 그다음 자기 주차 칸으로 간다. 정지점 사이 길은 Nav2가 짠다. 공유 구역
            # (주차장 출구, 7번 교차로, 5번)은 여전히 한 대씩 들어간다.
            robot_id = mission_match.group(1)
            dropoff = data.get("dropoff")
            if not isinstance(dropoff, str) or not dropoff.strip() or dropoff.strip() == "5":
                self._json(400, {"error": "배달 목적지 웨이포인트 이름을 고르세요 (5번은 적재 자리라 안 됩니다)."})
                return
            dropoff = dropoff.strip()
            # "원점": 5번에서 싣고 후진해 나온 뒤 출발한 자리로(서 있던 방향으로) 돌아간다. 넘겨주는 정지는 없다.
            return_to_start = dropoff == "원점"
            # "주차장": 5번에서 싣고 후진해 나온 뒤 자기 주차 칸으로 간다(수령 구역 없음).
            straight_home = dropoff == "주차장"
            via = data.get("via") or []
            if (not isinstance(via, list) or len(via) > 6
                    or any(not isinstance(label, str) or not label.strip() for label in via)):
                self._json(400, {"error": "경유지는 웨이포인트 이름 목록(최대 6개)이어야 합니다."})
                return
            via = [label.strip() for label in via]
            with STATE.lock:
                problem = _route_start_problem(robot_id)
                if problem:
                    self._json(*problem)
                    return
                pickup = _shared_waypoint(robot_id, "5")
                target = None if return_to_start or straight_home else _shared_waypoint(robot_id, dropoff)
                passes = [(label, _shared_waypoint(robot_id, label)) for label in via]
                needed = [("5", pickup)] + ([] if return_to_start or straight_home else [(dropoff, target)]) + passes
                missing = [label for label, item in needed if item is None]
                if missing:
                    self._json(400, {"error": "저장되지 않은 웨이포인트: " + ", ".join(missing)})
                    return
                pose = (TELEMETRY.snapshot().get(robot_id, {}).get("tf_pose", {}).get("value") or {}) if TELEMETRY else {}
                if not all(isinstance(pose.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_rad")):
                    self._json(409, {"error": "지금 지도 위치를 읽지 못했습니다."})
                    return
                map_name = STATE.robots[robot_id]["map"]
                bounds = TRAFFIC.zones.get("parking", {}).get("bounds") or [0, 0, 0, 0]
                in_parking = bounds[0] <= pose["x_m"] <= bounds[1] and bounds[2] <= pose["y_m"] <= bounds[3]
                start = dict(label="주차장" if in_parking else "출발", map=map_name, frame_id="map",
                             x_m=pose["x_m"], y_m=pose["y_m"], yaw_rad=pose["yaw_rad"])
                parking = dict(PARKING[robot_id], map=map_name, frame_id="map", label="주차장")
                try:
                    # `via`는 갈 때 고른 웨이포인트를 지나게 하고(예: 9, 7 = 가운데 벽 오른쪽 통로) 올 때 같은 길을
                    # 거꾸로 지나게 한다. 그래서 Nav2가 다른 틈을 고르지 않는다.
                    out = [dict(item) for _, item in passes]
                    back = [dict(item) for _, item in reversed(passes)]
                    # 5번은 대기 자리(navigation.json dock_queue)에서 들어간다: 한 대가 거기서 차례를 기다린다.
                    # 그 앞에 통로 아래 대기 자리(wait_low)를 두어 핑키들이 순서대로 줄 선다.
                    for setting in ("wait_low", "dock_queue"):
                        label = (NAVIGATION.get(setting) or {}).get("waypoint")
                        spot = _shared_waypoint(robot_id, label) if label and label not in via else None
                        if spot:
                            out.append(dict(spot))
                    if return_to_start:
                        home = dict(start, label="원점", arrival_yaw_rad=pose["yaw_rad"])
                        goals = build_mission_route([start] + out + [dict(pickup)] + back + [home], passages=NAVIGATION.get('wall_passages'), reverse_exit=NAVIGATION.get('dock_exit_reverse', True) is not False)
                    elif straight_home:
                        goals = build_mission_route([start] + out + [dict(pickup)] + back + [parking], passages=NAVIGATION.get('wall_passages'), reverse_exit=NAVIGATION.get('dock_exit_reverse', True) is not False)
                    else:
                        goals = build_mission_route([start] + out + [dict(pickup), dict(target)] + back + [parking], passages=NAVIGATION.get('wall_passages'), reverse_exit=NAVIGATION.get('dock_exit_reverse', True) is not False)
                except ValueError as exc:
                    self._json(400, {"error": str(exc)})
                    return
                goals[0]["parking_yaw_rad"] = pose["yaw_rad"]  # 선 자리에서, 서 있는 방향 그대로 출발한다
                anchors = [p for p in goals if p.get("anchor")]
                if not return_to_start:
                    goals[-1]["yaw_rad"] = parking["yaw_rad"]
                    goals[-1]["parking_return"] = True
                if not return_to_start and not straight_home:
                    drop_anchor = [p for p in anchors if p["label"] == dropoff][0]
                    drop_anchor["course_end"] = True  # 배달 지점: 넘겨줌(주행완료) 신호를 기다린다
                for point in goals:
                    point["use_ir"] = _ir_steers(robot_id)
                _mark_pass_through(goals)
                labels = [point["label"] for point in goals]
                STATE.routes[robot_id] = {
                    "status": "starting", "run_id": uuid.uuid4().hex, "labels": labels, "current_index": 0,
                    "execution": "free_mission", "nav_goal_count": max(0, len(anchors)-1), "dropoff": dropoff, "via": via,
                    "current_label": None, "completed": 0,
                    "message": (f"시험 임무: {' → '.join(via) + ' 경유 → ' if via else ''}5번 적재 → 후진 → 원점 복귀" if return_to_start else
                                f"시험 임무: {' → '.join(via) + ' 경유 → ' if via else ''}5번 적재 → 후진 → 주차장 복귀" if straight_home else
                                f"시험 임무: {' → '.join(via) + ' 경유 → ' if via else ''}5번 적재 → {dropoff} 배달 → 주차장 복귀"), "detail": "",
                }
            WORKERS.submit(_run_waypoint_route, robot_id, goals)
            self._json(202, {"ok": True, "robot": robot_id, "labels": labels})
            return
        # 웨이포인트 이름 목록대로 도는 경로(초기 라인 코스 방식). 앞뒤에 자기 주차 칸을 붙이고, 기준점 사이는
        # build_dense_route가 10 cm 간격 통과점으로 채운다.
        route_match = re.fullmatch(r"/api/robots/(10|11|12)/route", path)
        if route_match:
            robot_id = route_match.group(1)
            labels = data.get("labels")
            if not isinstance(labels, list) or not 1 <= len(labels) <= 50 or any(not isinstance(label, str) for label in labels):
                self._json(400, {"error": "웨이포인트 이름 목록을 확인하세요."})
                return
            with STATE.lock:
                problem = _route_start_problem(robot_id)
                if problem:
                    self._json(*problem)
                    return
                by_label = {item.get("label"): item for item in STATE.waypoints[robot_id]}
                missing = [label for label in labels if label not in by_label]
                if missing:
                    self._json(400, {"error": "저장되지 않은 웨이포인트: " + ", ".join(missing)})
                    return
                selected = [by_label[label] for label in labels]
                if any(item.get("frame_id") != "map" or item.get("map") != STATE.robots[robot_id]["map"] for item in selected):
                    self._json(409, {"error": "웨이포인트 지도 또는 좌표 프레임이 현재 설정과 다릅니다."})
                    return
                try:
                    parking=dict(PARKING[robot_id],map=STATE.robots[robot_id]['map'],frame_id='map')
                    parking['label']='주차장'
                    goals = build_dense_route([parking]+selected, ['주차장']+labels+['주차장'])
                    goals[0]['parking_yaw_rad']=parking['yaw_rad']
                    goals[-1]['yaw_rad']=parking['yaw_rad']
                    goals[-1]['parking_return']=True
                    # 마지막 코스 점에서 바깥 완료 신호(주행완료)를 기다린다.
                    anchors=[p for p in goals if p.get('anchor')]
                    anchors[-2]['course_end']=True
                    anchors[1]['arrival_yaw_rad']=0.
                    for point in goals:
                        point['use_ir']=_ir_steers(robot_id)
                except ValueError as exc:
                    self._json(400, {"error": str(exc)})
                    return
                labels = [point["label"] for point in goals]
                STATE.routes[robot_id] = {
                    "status": "starting", "run_id": uuid.uuid4().hex, "labels": labels, "current_index": 0,
                    "execution": "continuous_segments", "nav_goal_count": max(0, sum(bool(p.get('anchor')) for p in goals)-1),
                    "current_label": None, "completed": 0,
                    "message": "Nav2 경로 주행을 시작합니다.", "detail": "",
                }
            WORKERS.submit(_run_waypoint_route, robot_id, goals)
            self._json(202, {"ok": True, "robot": robot_id, "labels": labels})
            return
        # 운영자가 지도에서 지정한 위치. refine이 false면 라이다로 다듬지 않는다(천장 카메라 위치 등).
        set_pose_match = re.fullmatch(r"/api/robots/(10|11|12)/set-pose", path)
        if set_pose_match:
            robot_id = set_pose_match.group(1)
            pose = data.get("pose") if isinstance(data.get("pose"), dict) else {}
            values = [pose.get(k) for k in ("x_m", "y_m", "yaw_rad")]
            if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
                self._json(400, {"error": "pose는 x_m, y_m, yaw_rad 숫자여야 합니다."})
                return
            grid = (TELEMETRY.snapshot().get(robot_id, {}).get("map", {}).get("value") if TELEMETRY else None) or LOCAL_MAP_GRID or {}
            if grid.get("width"):
                x_lo, y_lo = grid["origin_x_m"], grid["origin_y_m"]
                x_hi, y_hi = x_lo + grid["width"] * grid["resolution_m"], y_lo + grid["height"] * grid["resolution_m"]
                if not (x_lo <= values[0] <= x_hi and y_lo <= values[1] <= y_hi):
                    self._json(400, {"error": "지도 밖입니다."})
                    return
            with STATE.lock:
                if STATE.busy_robot is not None:
                    self._json(409, {"error": f"Pinky {STATE.busy_robot} 작업이 끝난 뒤 위치를 지정하세요."})
                    return
                route = STATE.routes[robot_id]
                if route and route.get("status") in {"starting", "running", "cancel_requested"}:
                    self._json(409, {"error": "경로 주행을 정지한 뒤 위치를 지정하세요."})
                    return
                STATE.busy_robot = robot_id
                STATE.robots[robot_id].update(message="운영자가 지정한 위치를 넣습니다.", error=None)
            WORKERS.submit(_set_operator_pose, robot_id, *[float(v) for v in values], data.get("refine") is not False)
            self._json(202, {"ok": True, "message": "지정한 위치를 넣는 중입니다."})
            return
        localize_match = re.fullmatch(r"/api/robots/(10|11|12)/localize", path)
        if localize_match:
            # 라이다 위치 잡기는 이 PC에서 DDS로 AMCL에 보낸다: SSH 비밀번호는 필요 없고 Nav2만 돌고 있으면 된다.
            robot_id = localize_match.group(1)
            choice = None
            if data.get("pose") is not None:
                pose = data["pose"] if isinstance(data["pose"], dict) else {}
                values = [pose.get(k) for k in ("x_m", "y_m", "yaw_rad")]
                if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
                    self._json(400, {"error": "pose는 고른 후보의 x_m, y_m, yaw_rad여야 합니다."})
                    return
                choice = tuple(float(v) for v in values)
            with STATE.lock:
                if STATE.busy_robot is not None:
                    self._json(409, {"error": f"Pinky {STATE.busy_robot} 작업이 끝난 뒤 위치를 찾으세요."})
                    return
                route = STATE.routes[robot_id]
                if route and route.get("status") in {"starting", "running", "cancel_requested"}:
                    self._json(409, {"error": "경로 주행을 정지한 뒤 위치를 찾으세요."})
                    return
                STATE.busy_robot = robot_id
                STATE.robots[robot_id].update(message="라이다로 지도에서 자기 위치를 찾습니다.", error=None)
            WORKERS.submit(_relocalize, robot_id, choice)
            self._json(202, {"ok": True, "message": "라이다로 위치 찾기를 시작했습니다."})
            return
        initial_pose_match = re.fullmatch(r"/api/robots/(10|11|12)/initial-pose", path)
        if initial_pose_match:
            robot_id = initial_pose_match.group(1)
            with STATE.lock:
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                if STATE.busy_robot is not None:
                    self._json(409, {"error": f"Pinky {STATE.busy_robot} 작업이 끝난 뒤 위치를 설정하세요."})
                    return
                if STATE.robots[robot_id]["stage"] != "prepared":
                    self._json(409, {"error": "주행준비 완료 후 초기 위치를 설정하세요."})
                    return
                route = STATE.routes[robot_id]
                if route and route.get("status") in {"starting", "running", "cancel_requested"}:
                    self._json(409, {"error": "경로 주행을 정지한 뒤 초기 위치를 설정하세요."})
                    return
                STATE.busy_robot = robot_id
                STATE.robots[robot_id].update(message="지정 주차장 좌표로 AMCL 위치를 설정합니다.", error=None)
            WORKERS.submit(_set_initial_pose, robot_id)
            self._json(202, {"ok": True, "message": "AMCL 초기 위치 설정을 요청했습니다."})
            return
        # 정지: 경로가 없어도 정지 유지(hold)를 건다. 실제 정지는 _cancel_waypoint_route가 SSH로 한다.
        route_cancel_match = re.fullmatch(r"/api/robots/(10|11|12)/route/cancel", path)
        if route_cancel_match:
            robot_id = route_cancel_match.group(1)
            with STATE.lock:
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                route = STATE.routes[robot_id]
                if route and route.get('stop_pending'):
                    self._json(409, {'error':'정지 적용 확인 중입니다. 잠시 기다려 주세요.'})
                    return
                if route is None:
                    route={'status':'cancel_requested','labels':[],'message':'정지 유지 요청 중','detail':'','hold':True}
                    STATE.routes[robot_id]=route
                else:
                    # 이미 멈춘(또는 집에 가려고 기다리던) 핑키를 또 멈추면 그 자리에 붙잡아 둔다.
                    # 끝난 경로의 status는 'complete'다(2026-11-25 수정: 예전에는 'completed'와 비교해 끝난 경로를
                    # 알아보지 못했고, hold가 없어 60초 뒤 혼자 복귀하려 했다).
                    if route.get('status') in {'cancelled','failed','complete'}:
                        route['hold']=True
                    route.update(status='cancel_requested',message='정지 유지와 Nav2 취소를 요청합니다.',auto_return_at=None)
                route['stop_pending']=True
            WORKERS.submit(_cancel_waypoint_route, robot_id)
            self._json(202, {"ok": True, "message": "경로 취소를 요청했습니다."})
            return
        display_match = re.fullmatch(r"/api/robots/(10|11|12)/display/reload", path)
        if display_match:
            self._json(*_reload_display(display_match.group(1)))
            return
        clear_match = re.fullmatch(r"/api/robots/(10|11|12)/route/clear", path)
        if clear_match:
            self._json(*_clear_route(clear_match.group(1)))
            return
        home_match = re.fullmatch(r"/api/robots/(10|11|12)/return-home", path)
        if home_match:
            via = data.get("via") or []
            if not isinstance(via, list) or len(via) > 6 or any(not isinstance(v, str) or not v.strip() for v in via):
                self._json(400, {"error": "경유지는 웨이포인트 이름 목록(최대 6개)이어야 합니다."})
                return
            self._json(*_start_return_home(home_match.group(1), [v.strip() for v in via], "운영자 요청"))
            return
        reset_match = re.fullmatch(r"/api/robots/(10|11|12)/reset", path)
        if reset_match:
            robot_id = reset_match.group(1)
            map_name = data.get("map", "map-proto.yaml")
            if map_name not in {"map-proto.yaml", "my_map.yaml", "pinklab.yaml"}:
                self._json(400, {"error": "지원하는 지도는 map-proto.yaml, my_map.yaml, pinklab.yaml입니다."})
                return
            with STATE.lock:
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                if STATE.busy_robot is not None:
                    self._json(409, {"error": f"Pinky {STATE.busy_robot} 작업이 끝날 때까지 기다려 주세요."})
                    return
                STATE.busy_robot = robot_id
                STATE.robots[robot_id].update(stage="base_starting", message="초기 세팅: 정지하고 경로를 지운 뒤 주행준비를 처음부터 합니다.", error=None)
            WORKERS.submit(_reset_robot, robot_id, map_name)
            self._json(202, {"ok": True, "message": "초기 세팅을 시작했습니다."})
            return
        nav_recover_match = re.fullmatch(r"/api/robots/(10|11|12)/nav/restart", path)
        if nav_recover_match:
            robot_id = nav_recover_match.group(1)
            with STATE.lock:
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                if STATE.busy_robot is not None:
                    self._json(409, {"error": f"Pinky {STATE.busy_robot} 작업이 끝날 때까지 기다려 주세요."})
                    return
                if STATE.routes[robot_id] and STATE.routes[robot_id].get("status") in {"starting", "running", "cancel_requested"}:
                    self._json(409, {"error": "경로 주행을 먼저 정지하세요."})
                    return
                STATE.busy_robot = robot_id
                STATE.robots[robot_id].update(stage="nav_starting", message="Nav2 복구를 준비합니다.", error=None)
            WORKERS.submit(_recover_navigation, robot_id)
            self._json(202, {"ok": True, "message": "Nav2를 재시작하고 액션 서버를 확인합니다."})
            return
        if path == "/api/prepare-all":
            map_name = data.get("map", "map-proto.yaml")
            if map_name not in {"map-proto.yaml", "my_map.yaml", "pinklab.yaml"}:
                self._json(400, {"error": "지원하는 지도는 map-proto.yaml, my_map.yaml, pinklab.yaml입니다."})
                return
            with STATE.lock:
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                if STATE.busy_robot is not None:
                    self._json(409, {"error": f"Pinky {STATE.busy_robot} 작업이 끝날 때까지 기다려 주세요."})
                    return
                targets, skipped = [], []
                telemetry = TELEMETRY.snapshot() if TELEMETRY is not None else {}
                for robot_id, info in STATE.robots.items():
                    route = STATE.routes[robot_id] or {}
                    # "준비됨"은 그 Nav2가 아직 map 위치를 줄 때만 친다: 껐다 켠 핑키는 저장된 stage를 그대로
                    # 갖지만(관제 재시작에도 남는다) 다시 준비할 때까지 Nav2가 없다.
                    live = (telemetry.get(robot_id, {}).get("tf_pose") or {}).get("fresh")
                    if route.get("status") in {"starting", "running", "cancel_requested"}:
                        skipped.append(f"{robot_id} 주행 중")
                    elif info["stage"] == "prepared" and live and not _runtime_outdated(telemetry.get(robot_id, {})):
                        skipped.append(f"{robot_id} 이미 준비됨")
                    elif not info.get("ssh_reachable"):
                        skipped.append(f"{robot_id} 연결 안 됨")
                    else:
                        targets.append(robot_id)
                if not targets:
                    self._json(409, {"error": "준비할 핑키가 없습니다: " + ", ".join(skipped)})
                    return
                STATE.busy_robot = "all"
                for order, robot_id in enumerate(targets, 1):
                    STATE.robots[robot_id].update(map=map_name, error=None,
                                                  message=f"차례로 주행준비합니다 ({order}/{len(targets)}번째).")
            WORKERS.submit(_prepare_robots_in_turn, targets, map_name)
            note = f" (건너뜀: {', '.join(skipped)})" if skipped else ""
            self._json(202, {"ok": True, "robots": targets, "skipped": skipped,
                             "message": f"Pinky {', '.join(targets)} 차례로 주행준비하고 주차장 좌표로 위치를 잡습니다{note}."})
            return
        match = re.fullmatch(r"/api/robots/(10|11|12)/ready", path)
        if match:
            robot_id = match.group(1)
            map_name = data.get("map", "map-proto.yaml")  # 웨이포인트, keepout, 수령 구역이 모두 map-proto 기준이다
            if map_name not in {"map-proto.yaml", "my_map.yaml", "pinklab.yaml"}:
                self._json(400, {"error": "지원하는 지도는 map-proto.yaml, my_map.yaml, pinklab.yaml입니다."})
                return
            with STATE.lock:
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                if STATE.busy_robot is not None:
                    self._json(409, {"error": f"Pinky {STATE.busy_robot} 준비가 끝날 때까지 기다려 주세요."})
                    return
                current = STATE.robots[robot_id]["stage"]
                if current in {"base_starting", "sensors_starting", "nav_starting"}:
                    self._json(409, {"error": "이미 주행준비 중입니다."})
                    return
                # 준비된 핑키도 다시 준비할 수 있다(껐다 켰거나 깨끗하게 시작하려고). 주행 중에는 안 된다.
                route = STATE.routes[robot_id] or {}
                if route.get("status") in {"starting", "running", "cancel_requested"}:
                    self._json(409, {"error": "경로 주행을 정지한 뒤 주행준비하세요."})
                    return
                STATE.busy_robot = robot_id
                STATE.robots[robot_id].update(stage="base_starting", map=map_name, message="기본 bringup을 준비합니다.", error=None)
            # 시작 순서가 끝날 때까지 worker가 busy 표시를 쥐고 있다.
            WORKERS.submit(_prepare_robot, robot_id, map_name)
            self._json(202, {"ok": True, "robot": robot_id})
            return
        # 지금 TF map 위치를 웨이포인트로 기록한다(로봇별 최대 200개, data/waypoints.json).
        waypoint_match = re.fullmatch(r"/api/robots/(10|11|12)/waypoints", path)
        if waypoint_match:
            robot_id = waypoint_match.group(1)
            label = data.get("label", "")
            if not isinstance(label, str) or len(label.strip()) > 48:
                self._json(400, {"error": "지점 이름은 48자 이내로 입력하세요."})
                return
            telemetry = TELEMETRY.snapshot() if TELEMETRY is not None else {}
            topics = telemetry.get(robot_id, {})
            map_topic = topics.get("map", {})
            pose_topic = topics.get("tf_pose", {})
            pose = pose_topic.get("value") or {}
            if not map_topic.get("fresh") or not pose_topic.get("fresh"):
                self._json(409, {"error": "지도와 최신 TF 위치가 수신 중일 때 기록할 수 있습니다."})
                return
            if pose.get("frame_id") != "map":
                self._json(409, {"error": f"위치 프레임이 map이 아닙니다 ({pose.get('frame_id') or 'unknown'})."})
                return
            if any(pose.get(field) is None for field in ("x_m", "y_m", "yaw_rad")):
                self._json(409, {"error": "AMCL 위치값이 아직 유효하지 않습니다."})
                return
            with STATE.lock:
                entries = STATE.waypoints[robot_id]
                if len(entries) >= 200:
                    self._json(409, {"error": "로봇별 웨이포인트는 최대 200개까지 저장할 수 있습니다."})
                    return
                name = label.strip() or f"WP {len(entries) + 1:02d}"
                waypoint = {
                    "id": uuid.uuid4().hex,
                    "label": name,
                    "map": STATE.robots[robot_id]["map"],
                    "frame_id": pose["frame_id"],
                    "x_m": pose.get("x_m"),
                    "y_m": pose.get("y_m"),
                    "yaw_rad": pose.get("yaw_rad"),
                    "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                }
                entries.append(waypoint)
                try:
                    STATE.save_waypoints()
                except OSError as exc:
                    entries.pop()
                    self._json(500, {"error": f"웨이포인트 파일 저장 실패: {exc}"})
                    return
            self._json(201, {"ok": True, "waypoint": waypoint})
            return
        self._json(404, {"error": "not found"})

    def do_DELETE(self) -> None:  # noqa: N802
        """웨이포인트 하나를 지운다. 파일 저장에 실패하면 되돌린다."""
        path = unquote(self.path.split("?", 1)[0])
        match = re.fullmatch(r"/api/robots/(10|11|12)/waypoints/([a-f0-9]{32})", path)
        if not match:
            self._json(404, {"error": "not found"})
            return
        robot_id, waypoint_id = match.groups()
        with STATE.lock:
            entries = STATE.waypoints[robot_id]
            index = next((i for i, item in enumerate(entries) if item.get("id") == waypoint_id), None)
            if index is None:
                self._json(404, {"error": "웨이포인트를 찾을 수 없습니다."})
                return
            removed = entries.pop(index)
            try:
                STATE.save_waypoints()
            except OSError as exc:
                entries.insert(index, removed)
                self._json(500, {"error": f"웨이포인트 파일 저장 실패: {exc}"})
                return
        self._json(200, {"ok": True})


# ── 15) 시작과 종료(main) ────────────────────────────────────────────────────────────────


def main() -> None:
    """ROS 수신을 띄우고(실패해도 화면은 열고 오류를 보인다), 백그라운드 루프 네 개와 HTTP 서버를 시작한다.
    SIGTERM/SIGINT를 받으면 HTTP 서버와 ROS context를 모두 정리한다."""
    global TELEMETRY
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN)
    try:
        import rclpy
        rclpy.init(args=None)
        TELEMETRY = Telemetry()
    except Exception as exc:
        STATE.monitor_error = f"ROS 모니터를 시작하지 못했습니다: {exc}"
        print(STATE.monitor_error)
    threading.Thread(target=_traffic_loop,daemon=True,name="traffic-coordinator").start()
    threading.Thread(target=_health_loop, daemon=True, name="robot-health").start()
    threading.Thread(target=_indicator_loop, daemon=True, name="status-lamp").start()
    threading.Thread(target=_auto_return_loop, daemon=True, name="auto-return").start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    # rclpy는 ROS만 멈추고 HTTP는 멈추지 않는 신호 처리기를 단다.
    # 둘 다 깔끔하게 내려야 재시작할 때 DDS 참가자가 남지 않는다.
    # server.shutdown()은 serve_forever가 끝나기를 기다리므로 신호 처리기 안에서 바로 부르면 막힌다. 그래서 다른 스레드에서 부른다.
    def stop_server(*_):
        SHUTTING_DOWN.set()
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,stop_server)
    signal.signal(signal.SIGINT,stop_server)
    print(f"Pinky fleet dashboard: http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        SHUTTING_DOWN.set()
        WORKERS.shutdown(wait=False, cancel_futures=True)
        if TELEMETRY is not None:
            for context,node,executor in TELEMETRY.domain_readers:
                executor.shutdown()
                node.destroy_node()
                TELEMETRY.rclpy.shutdown(context=context)
            TELEMETRY.executor.shutdown()
            TELEMETRY.node.destroy_node()
            TELEMETRY.rclpy.shutdown()


if __name__ == "__main__":
    main()
