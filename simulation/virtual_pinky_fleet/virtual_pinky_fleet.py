#!/usr/bin/env python3
"""가상 핑키 관제: 실제 Nav2 지도 위의 가상 핑키 세 대로 핑키 팀 관제(dashboard)의 HTTP API에 답한다.

이것으로 관제 서버의 주문 흐름을 로봇, 핑키 관제 PC, ROS 없이 끝까지 돌릴 수 있다.

    python3 simulation/virtual_pinky_fleet/virtual_pinky_fleet.py                 # http://127.0.0.1:8766
    CONTROL_STATION_PINKY_FLEET_URL=http://127.0.0.1:8766 ./run-control-station.sh

이 파일 옆의 지도, 웨이포인트, 주차 자리는 팀 파일의 사본이다(2026-11-17, 설명은
docs/implementation/virtual-pinky-fleet.md). 모든 응답에 "simulated": true가 붙고, 관제 서버는 이 핑키들을
가상으로 표시한다. 메모리 안의 가상 핑키만 움직이며 실물 핑키·ROS·SSH와는 통신하지 않는다.

실제 핑키 관제와 같은 점(2026-11-17 기준 pinky_fleet_dashboard/app.py, scripts/navigate_route.py):
- GET /api/status: robots[](route {status, phase, mission_token, current_label, labels, completed}, operational,
  topics(map, tf_pose, battery, plan, trail...), waypoints, parking), 최상위에 keepout과 traffic
- POST /api/robots/<id>/route {labels}: 그 로봇 자신의 웨이포인트로 주차장 -> labels -> 주차장(하나라도 없으면
  400, 이미 경로를 주행 중이면 409)
- "5"에서: 정면에서 방향 맞추기, 진입, docking_manual(token) -> dock_complete -> docking_anchor -> docked_manual
  (token) -> received -> 앞으로 빠져나감(navigation.json dock_exit_reverse가 false일 때. true면 먼저 20 cm 후진)
- docking_manual에서 다시 맞추기(operator-confirm {action: redock, offset?}): 35 cm 후진, 방향 맞추기, 다시 진입,
  최대 세 번(지금 코드의 한도 REDOCK_LIMIT는 실제 핑키 관제와 같은 30이다). 천장 카메라의 offset
  {forward_mm, left_mm, dyaw_deg}이 오면 그만큼 고쳐서 들어온다(실제 핑키 관제가 쓰는 것과 같은
  route_geometry.corrected_dock)
- 가운데 벽을 사이에 둔 두 정지점 사이는 실제 임무 경로처럼 통로 점(navigation.json wall_passages)을 거쳐 간다
- 마지막 label에서: course_complete(token) -> mission-complete -> returning -> 자기 주차 자리로 복귀
- operator-confirm과 mission-complete: phase나 token이 맞지 않으면 409, 맞으면 202
여기에만 있는 것(실제 핑키 관제에는 이 경로가 없다. 관제 서버는 핑키가 가상일 때 첫 번째 경로를 쓴다):
- GET /api/robots/<id>/parking-check: 5번 자리의 가상 천장 카메라. ARRIVED / MISALIGNED / NOT_IN_VIEW(아직
  움직이는 중이면 MOVING)와, 핑키가 실제로 선 위치가 적재 자리에서 얼마나 떨어졌는지(offset)
- POST /api/virtual/dock-bias {robot?, forward_mm, left_mm, dyaw_deg}: 5번 자리에서 핑키가 실제로 선 위치가
  스스로 믿는 위치에서 얼마나 떨어져 있는지(위치 추정 편향). 다시 맞추기 반복을 시험할 때 쓴다. --dock-bias로도 준다
가상으로 하지 않는 것: Nav2 경로 계획(점 사이는 직선), 장애물과 충돌, 공유 구역 예약, IR 라인 추종, SSH,
배터리 소모. 모든 센서는 있는 것으로 친다.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[1]
DASHBOARD = REPOSITORY_ROOT / "pinky_fleet_dashboard"
sys.path.insert(0, str(DASHBOARD / "scripts"))
# 실제 핑키 관제의 도킹 기하 계산(정면 진입점 거리, 벽 통로 경유, 카메라 보정 진입)을 그대로 가져다 쓴다.
from route_geometry import DOCK_APPROACH_M, _through_passages, corrected_dock  # noqa: E402 - the real dashboard's math

# 다시 맞출 때 후진 거리(m)와 최대 횟수. 후진은 방향 맞추기가 진입선을 고칠 여유를 주려는 것이고, 30번은
# 관제 서버가 스스로 보내는 횟수(5)보다 훨씬 많다(진입 반복 시험이 쓴다).
REDOCK_BACK_M, REDOCK_LIMIT = 0.35, 30  # scripts/navigate_route.py와 같은 값
# 등록된 적재 자리의 허용 오차(.control-station/parking-spots/pinky_dock_1.json, 2026-11-17)와, 적재 자리에서
# 얼마나 떨어진 곳까지 가상 천장 카메라가 핑키를 볼 수 있는지(m).
CAMERA_TOLERANCE = {"position_mm": 40.0, "yaw_deg": 15.0}
CAMERA_VIEW_M = 0.45
PASS_LABELS = ("통로 아래", "통로 위")  # 이 파일에서는 쓰이지 않는다.
# 핑키 관제의 로봇 ID → 표시 이름, ROS domain, 배터리(%). 배터리는 줄지 않는다.
ROBOTS = {
    "10": {"name": "Pinky C76C", "domain": 10, "battery": 92.0},
    "11": {"name": "Pinky D631", "domain": 11, "battery": 78.0},
    "12": {"name": "Pinky 1E3E", "domain": 12, "battery": 64.0},
}
MAP_NAME = "map-proto.yaml"  # 웨이포인트의 map 필드가 이 이름이어야 쓴다.
# /api/status에서 늘 최신 값이 있는 것으로 보고할 센서 토픽들이다(모든 센서가 있는 것으로 친다).
SENSOR_TOPICS = ("scan", "ir", "ultrasonic", "batt_state", "imu", "odom", "joints", "line_control", "safety_status")
TRAIL_POINTS = 400  # 화면에 그릴 지나온 자취 점의 최대 개수다.


# ---- 지도 ------------------------------------------------------------------------------------------------------
def read_map(yaml_path: Path) -> dict[str, Any]:
    """trinary 지도 YAML과 그 PGM에서 Nav2 map_server가 만드는 것과 같은 점유 격자를 만든다(0행 = 지도 아래쪽).

    칸 값은 100(점유), 0(빈 곳), -1(모름)이다. 관제 화면에 그리는 용도이며 주행 계획에는 쓰지 않는다.
    """
    settings: dict[str, str] = {}
    # YAML 라이브러리 없이 `키: 값` 줄만 읽는다. `#` 뒤 주석은 버린다.
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        key, _, value = line.partition(":")
        if value:
            settings[key.strip()] = value.split("#")[0].strip()
    width, height, maxval, pixels = read_pgm(yaml_path.parent / settings["image"])
    origin = [float(v) for v in settings["origin"].strip("[]").split(",")]
    # 기본값 0.65/0.196은 map_server의 기본 문턱값이다.
    occupied, free = float(settings.get("occupied_thresh", 0.65)), float(settings.get("free_thresh", 0.196))
    negate = settings.get("negate", "0") == "1"
    cells = []
    # PGM은 위쪽 행부터 저장되고 점유 격자는 아래쪽 행부터이므로 행을 거꾸로 읽는다. 어두울수록 점유다.
    for row in range(height - 1, -1, -1):
        for col in range(width):
            value = pixels[row * width + col]
            occupancy = value / maxval if negate else (maxval - value) / maxval
            cells.append(100 if occupancy > occupied else 0 if occupancy < free else -1)
    return {"width": width, "height": height, "resolution_m": float(settings["resolution"]),
            "origin_x_m": origin[0], "origin_y_m": origin[1], "origin_yaw_rad": origin[2], "cells": cells,
            "source": "virtual_local_file"}


def read_pgm(path: Path) -> tuple[int, int, int, list[int]]:
    """PGM 이미지(P5 바이너리 또는 P2 텍스트)를 읽어 (너비, 높이, 최댓값, 픽셀 목록)을 돌려준다.

    헤더의 `#` 주석 줄은 건너뛴다. P5는 픽셀당 1바이트(최댓값 255 이하)로 읽는다.
    """
    data = path.read_bytes()
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b"P5":  # 헤더 끝의 공백 한 글자 뒤부터 픽셀 바이트다.
        return width, height, maxval, list(data[pos + 1:pos + 1 + width * height])
    if magic == b"P2":
        return width, height, maxval, [int(v) for v in data[pos:].split()][:width * height]
    raise ValueError(f"{path}: PGM P5 or P2 only, got {magic!r}")


# ---- 핑키 한 대 -------------------------------------------------------------------------------------------------
class Pinky:
    """가상 핑키 한 대. 자세·경로 상태·남은 단계 목록을 들고, tick마다 단계를 차례로 진행한다."""

    def __init__(self, pinky_id: str, parking: dict[str, Any], waypoints: list[dict[str, Any]], speed_mps: float) -> None:
        self.id = pinky_id
        self.parking = parking
        self.waypoints = waypoints
        self.speed = speed_mps
        self.battery = ROBOTS[pinky_id]["battery"]
        self.x, self.y, self.yaw = parking["x_m"], parking["y_m"], parking["yaw_rad"]
        self.route: dict[str, Any] | None = None
        self.steps: list[dict[str, Any]] = []
        self.trail: list[list[float]] = []
        self.moving = 0.0  # 이번 tick의 주행 속도(m/s). 0이면 멈춰 있다.
        # (forward_mm, left_mm, dyaw_deg): 5번 자리에서 핑키가 스스로 믿는 위치로부터 실제로 선 위치까지.
        # 보고하는 자세는 실제 핑키처럼 스스로 믿는 위치다.
        self.bias = (0.0, 0.0, 0.0)

    # 단계 종류: drive(한 점까지 주행, 후진일 수도 있음), wait(운영자 신호 대기), hold(시간이 정해진 phase),
    # phase(즉시 바뀜).
    def start(self, goals: list[dict[str, Any]], labels: list[str], course_end: int | None,
              passes: dict[int, list[dict[str, Any]]] | None = None, reverse_exit: bool = False, **route: Any) -> None:
        """목표 목록(첫 항목은 출발점)으로 단계 목록을 만들고 경로를 시작한다.

        `passes`: 목표 i 전에 지나가는 점(벽 통로)이며, 경로의 정지점은 아니다. `course_end`는 코스 완주 신호를
        기다릴 목표 번호이고(None이면 기다리지 않음), `route`의 나머지 키는 그대로 route 상태에 들어간다.
        """
        steps: list[dict[str, Any]] = [{"kind": "phase", "phase": "driving", "message": "Nav2 경로 주행 중 (가상)"}]
        for index, goal in enumerate(goals[1:], start=1):
            for point in (passes or {}).get(index, []):
                steps.append({"kind": "drive", "x": point["x_m"], "y": point["y_m"]})
            if goal["label"] == "5":
                # 5번(적재 자리)은 DOCK_APPROACH_M 앞의 정면 진입점에서 방향을 맞춘 뒤 곧게 들어간다.
                heading = goal["yaw_rad"]
                steps.append({"kind": "drive", "x": goal["x_m"] - DOCK_APPROACH_M * math.cos(heading),
                              "y": goal["y_m"] - DOCK_APPROACH_M * math.sin(heading), "yaw": heading,
                              "phase": "driving", "message": "5번 정면에서 방향 맞추는 중 (가상)"})
            # 도착 방향은 5번과 마지막 목표(주차장)에서만 맞춘다. 나머지 점에서는 진행 방향 그대로 둔다.
            steps.append({"kind": "drive", "x": goal["x_m"], "y": goal["y_m"], "label": goal["label"], "index": index,
                          "yaw": goal.get("yaw_rad") if goal["label"] == "5" or index == len(goals) - 1 else None})
            if goal["label"] == "5":
                steps += [
                    {"kind": "wait", "phase": "docking_manual", "signal": "dock_complete", "dock": dict(goal),
                     "message": "5번 도착 · 정지 · 수동 도킹완료 확인 대기"},
                    {"kind": "hold", "phase": "docking_anchor", "seconds": 2.0, "message": "5번 위치 다시 맞추는 중"},
                    {"kind": "wait", "phase": "docked_manual", "signal": "received",
                     "message": "도킹완료 · 정지 유지 · 물건 수령 완료 대기"},
                ]
                if reverse_exit:
                    steps.append({"kind": "drive", "phase": "dock_exit", "back_m": 0.20,
                                  "message": "수령 확인 · 20 cm 후진"})
                steps.append({"kind": "phase", "phase": "driving", "message": "Nav2 경로 주행 중 (가상)"})
            if index == course_end:  # 마지막 label: 주행완료 신호를 받은 뒤 주차장으로 돌아간다.
                steps += [
                    {"kind": "wait", "phase": "course_complete", "signal": "mission_complete",
                     "message": "코스 완주 · 정지 · 주행완료 신호 대기"},
                    {"kind": "phase", "phase": "returning", "message": "주행완료 수신 · 자기 주차장 복귀중"},
                ]
        self.steps = steps
        self.trail = [[self.x, self.y]]
        self.route = {"status": "running", "run_id": uuid.uuid4().hex, "labels": labels, "current_index": 0,
                      "current_label": None, "completed": 0, "phase": None, "mission_token": None,
                      "message": "Nav2 경로 주행을 시작합니다. (가상)", "detail": "", **route}

    def stop(self, message: str) -> None:
        """남은 단계를 버리고, 주행 중이던 경로를 cancelled로 표시한다(멈춘 자세를 함께 남긴다)."""
        self.steps = []
        if self.route and self.route["status"] == "running":
            self.route.update(status="cancelled", mission_token=None, message=message,
                              stopped_pose={"x_m": self.x, "y_m": self.y, "yaw_rad": self.yaw})

    def signal(self, name: str, token: object) -> bool:
        """대기 중인 단계에 대한 운영자 신호. phase와 token이 맞으면 True다(그러면 핑키가 다음 단계로 넘어간다)."""
        step = self.steps[0] if self.steps else None
        if (not self.route or self.route["status"] != "running" or not step or step["kind"] != "wait"
                or step["signal"] != name or not token or token != self.route.get("mission_token")):
            return False
        self.steps.pop(0)
        self.route["mission_token"] = None  # 토큰은 한 번만 쓴다. 늦게 온 같은 신호는 거절된다.
        return True

    def redock(self, token: object, offset: object) -> bool | None:
        """천장 카메라가 핑키가 적재 자리에서 벗어났다고 판정했다: 후진하고, 방향을 맞추고, 다시 들어간 뒤
        dock_complete를 한 번 더 기다린다. 카메라 offset이 있으면 그만큼 자리를 고친다. phase나 token이 맞지
        않으면 None, 아니면 offset을 썼는지 여부를 돌려준다."""
        step = self.steps[0] if self.steps else None
        if (not self.route or self.route["status"] != "running" or not step or step["kind"] != "wait"
                or step["signal"] != "dock_complete" or not token or token != self.route.get("mission_token")):
            return None
        attempt = step.get("redocks", 0) + 1
        self.route["mission_token"] = None
        if attempt > REDOCK_LIMIT:
            self.steps = []
            self.route.update(status="failed", phase="failed",
                              message=f"5번 다시 맞추기를 {REDOCK_LIMIT}번 했는데도 또 요청받았습니다 · 현장을 확인하세요. (가상)")
            return False
        dock = step["dock"]
        heading = dock["yaw_rad"]
        square = {"x_m": dock["x_m"] - DOCK_APPROACH_M * math.cos(heading),
                  "y_m": dock["y_m"] - DOCK_APPROACH_M * math.sin(heading), "yaw_rad": heading}
        # offset을 쓸 수 없으면(형식이 틀림 등) corrected_dock이 None을 돌려주고, 등록된 자리로 다시 들어간다.
        guided = corrected_dock((self.x, self.y, self.yaw), offset, dock, DOCK_APPROACH_M) if offset is not None else None
        if guided:
            square, dock = guided
        note = f"({attempt}/{REDOCK_LIMIT}){' · 카메라 보정' if guided else ''}"
        # entered를 지워, 다시 대기 단계에 들어갈 때 새 토큰을 발급하게 한다.
        step.update(redocks=attempt, entered=None)
        # phase는 한 tick 뒤가 아니라 신호와 함께 바뀐다: 그 사이에 상태를 읽어도 아직 기다리는 것처럼 보이면 안 된다.
        self.route.update(phase="redocking", message=f"5번 위치가 어긋나 다시 맞추는 중 {note}")
        # 대기 단계 앞에 후진·방향 맞추기·정면 직진을 끼워 넣는다. 다 가면 같은 대기 단계로 돌아온다.
        self.steps[:0] = [
            {"kind": "drive", "phase": "redocking", "back_m": REDOCK_BACK_M,
             "message": f"5번 위치가 어긋나 다시 맞추는 중 {note} · {REDOCK_BACK_M * 100:.0f}cm 후진"},
            {"kind": "drive", "phase": "redocking", "x": square["x_m"], "y": square["y_m"], "yaw": square["yaw_rad"],
             "message": f"5번 다시 맞추는 중 {note} · 정면 진입점에서 방향 맞추기"},
            {"kind": "drive", "phase": "redocking", "x": dock["x_m"], "y": dock["y_m"], "yaw": dock["yaw_rad"],
             "message": f"5번 다시 맞추는 중 {note} · 정면 직진"},
        ]
        return bool(guided)

    def seen_from(self, dock: dict[str, Any]) -> dict[str, float] | None:
        """5번 위 천장 카메라가 재는 값: 핑키가 실제로 선 위치(스스로 믿는 자세 + 편향)가 적재 자리 `dock`에서
        자리 자신의 축을 따라 얼마나 떨어졌는지. 카메라 시야 밖이면 None이다."""
        heading = dock["yaw_rad"]
        forward_m, left_m = self.bias[0] / 1000.0, self.bias[1] / 1000.0
        # 편향(자리 축 기준)을 지도 축으로 돌려 실제 위치를 구하고, 자리 중심과의 차이를 지도 축으로 잰다.
        x = self.x + forward_m * math.cos(heading) - left_m * math.sin(heading) - dock["x_m"]
        y = self.y + forward_m * math.sin(heading) + left_m * math.cos(heading) - dock["y_m"]
        if math.hypot(x, y) > CAMERA_VIEW_M:
            return None
        # 지도 축의 차이를 자리 축(앞·왼쪽)으로 돌려 mm로 바꾼다.
        forward = (x * math.cos(heading) + y * math.sin(heading)) * 1000.0
        left = (-x * math.sin(heading) + y * math.cos(heading)) * 1000.0
        dyaw = math.degrees((self.yaw - heading + math.pi) % (2 * math.pi) - math.pi) + self.bias[2]  # -180~180°로 감싼다.
        return {"forward_mm": round(forward, 1), "left_mm": round(left, 1), "dyaw_deg": round(dyaw, 2),
                "distance_mm": round(math.hypot(forward, left), 1)}

    def tick(self, dt: float) -> None:
        """가상 시간 dt초만큼 진행한다. 한 단계를 끝내고 시간이 남으면 같은 tick 안에서 다음 단계로 이어 간다."""
        self.moving = 0.0
        while self.route and self.route["status"] == "running":
            if not self.steps:
                self.route.update(status="complete", mission_token=None, message="경로 완료 · 주차장 도착 (가상)")
                return
            step = self.steps[0]
            if step.get("entered") is None:
                step["entered"] = True
                self._enter(step)
            if step["kind"] == "wait":  # 운영자 신호(signal/redock)가 단계를 빼 줄 때까지 머문다.
                return
            if step["kind"] == "hold":
                step["seconds"] -= dt
                if step["seconds"] > 0:
                    return
            elif step["kind"] == "drive":
                dt = self._drive(step, dt)
                if dt <= 0:
                    return
                self._arrive(step)
            self.steps.pop(0)

    def _enter(self, step: dict[str, Any]) -> None:
        """단계에 처음 들어갈 때 한 번: phase·메시지를 바꾸고 wait 단계면 새 mission_token을 발급한다.

        후진 단계는 이때의 위치·방향으로 목표점을 정하고, 웨이포인트 단계는 current_label을 바꾼다.
        """
        route = self.route
        if "phase" in step:
            token = uuid.uuid4().hex if step["kind"] == "wait" else None
            route.update(phase=step["phase"], mission_token=token, message=step["message"])
        if step["kind"] == "drive":
            if "back_m" in step:
                step["x"] = self.x - step["back_m"] * math.cos(self.yaw)
                step["y"] = self.y - step["back_m"] * math.sin(self.yaw)
            elif "index" in step:
                route.update(current_index=step["index"], current_label=step["label"],
                             message=f"웨이포인트 {step['label']}로 이동 중 (가상)")

    def _drive(self, step: dict[str, Any], dt: float) -> float:
        """단계의 점을 향해 움직인다. 도착하면 남은 시간을, 아직 주행 중이면 0을 돌려준다."""
        dx, dy = step["x"] - self.x, step["y"] - self.y
        distance = math.hypot(dx, dy)
        reach = self.speed * dt
        if distance > 1e-6 and "back_m" not in step:  # 후진은 방향을 바꾸지 않고 뒤로 간다.
            self.yaw = math.atan2(dy, dx)
        if distance > reach:
            self.x += dx / distance * reach
            self.y += dy / distance * reach
            self.moving = self.speed
            self._mark()
            return 0.0
        self.x, self.y = step["x"], step["y"]
        self._mark()
        return dt - distance / self.speed

    def _arrive(self, step: dict[str, Any]) -> None:
        """점에 도착: 정해진 도착 방향이 있으면 제자리에서 맞추고, 웨이포인트면 completed를 올린다."""
        if step.get("yaw") is not None:
            self.yaw = step["yaw"]
        if "index" in step:
            self.route.update(completed=step["index"], current_label=step["label"],
                              message=f"웨이포인트 {step['label']} 도착 (가상)")

    def _mark(self) -> None:
        """2 cm 이상 움직였을 때만 자취 점을 더하고, 최근 TRAIL_POINTS개만 남긴다."""
        if not self.trail or math.hypot(self.trail[-1][0] - self.x, self.trail[-1][1] - self.y) >= 0.02:
            self.trail = (self.trail + [[round(self.x, 3), round(self.y, 3)]])[-TRAIL_POINTS:]

    def plan_points(self) -> list[list[float]]:
        """화면에 그릴 남은 경로: 현재 위치와 앞으로 갈 주행 점들. 갈 점이 없으면 빈 목록이다.

        후진 단계는 들어가기 전에는 목표점이 없으므로 빠진다. 점 사이는 직선이다(Nav2 경로 계획 없음).
        """
        points = [[round(self.x, 3), round(self.y, 3)]]
        points += [[round(s["x"], 3), round(s["y"], 3)] for s in self.steps if s["kind"] == "drive" and "x" in s]
        return points if len(points) > 1 else []


# ---- 핑키 세 대 -------------------------------------------------------------------------------------------------
class VirtualFleet:
    """가상 핑키 세 대와 지도·설정을 들고, 백그라운드 스레드에서 0.05초마다 모두를 진행시킨다.

    HTTP 요청 스레드와 진행 스레드가 같은 상태를 만지므로, 상태를 읽고 바꾸는 일은 모두 `self.lock` 안에서 한다.
    """

    def __init__(self, data_dir: Path, speed_mps: float, time_scale: float, shared_waypoints: bool) -> None:
        self.grid = read_map(data_dir / "map-proto.yaml")
        parking = json.loads((data_dir / "parking.json").read_text(encoding="utf-8"))
        waypoints = json.loads((data_dir / "waypoints.json").read_text(encoding="utf-8"))
        self.shared_waypoints = shared_waypoints
        self.time_scale = time_scale
        self.lock = threading.Lock()
        self.pinkies = {pid: Pinky(pid, parking[pid], waypoints.get(pid, []), speed_mps) for pid in ROBOTS}
        # 실제 핑키 관제의 설정: 벽 통로가 어디인지, 핑키가 5번에서 후진으로 빠져나오는지.
        try:
            navigation = json.loads((DASHBOARD / "config" / "navigation.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            navigation = {}
        self.passages = navigation.get("wall_passages") or []
        # 값이 없으면 후진(True)으로 본다. false로 적었을 때만 앞으로 빠져나간다.
        self.reverse_exit = navigation.get("dock_exit_reverse", True) is not False
        # 진입 금지 구역 마스크는 화면에 그리라고만 보낸다(display_only). 가상 주행은 이를 피하지 않는다.
        keepout_yaml = DASHBOARD / "config" / "keepout" / "keepout_mask.yaml"
        self.keepout = dict(read_map(keepout_yaml), map=MAP_NAME, frame_id="map", available=True,
                            source="keepout_mask.yaml", display_only=True) if keepout_yaml.is_file() else None
        # 관제 화면이 그리는 교통 구역: 주차 자리들을 0.2 m 넓힌 주차장 사각형과, 10번 핑키의 7번(교차로)·5번(적재
        # 자리) 웨이포인트 둘레 반경 0.4 m 요청 구역. 화면 표시용이며 여기서는 예약을 하지 않는다.
        spots = [p for p in parking.values()]
        self.traffic_config: dict[str, Any] = {"parking": {"bounds": [
            min(p["x_m"] for p in spots) - .2, max(p["x_m"] for p in spots) + .2,
            min(p["y_m"] for p in spots) - .2, max(p["y_m"] for p in spots) + .2]}}
        for label, name in (("7", "junction7"), ("5", "dock5")):
            anchor = self.waypoint("10", label)
            if anchor:
                self.traffic_config[name] = {"x_m": anchor["x_m"], "y_m": anchor["y_m"], "request_radius_m": .40}
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def waypoint(self, pinky_id: str, label: str, shared: bool = False) -> dict[str, Any] | None:
        """로봇 자신의 웨이포인트 `label`. `shared`면 없을 때 다른 핑키가 기록한 것을 쓴다(핑키 관제의
        _shared_waypoint, return-home이 쓴다). 같은 지도(MAP_NAME)·`map` 프레임에서 기록한 것만 쓴다."""
        order = [pinky_id] + ([p for p in ROBOTS if p != pinky_id] if shared else [])
        for pid in order:
            for item in self.pinkies[pid].waypoints:
                if item.get("label") == label and item.get("map") == MAP_NAME and item.get("frame_id") == "map":
                    return item
        return None

    def _passes(self, goals: list[dict[str, Any]]) -> dict[int, list[dict[str, Any]]]:
        """각 목표 전에 지나갈 벽 통로 점들(실제 임무 경로처럼 route_geometry로 구한다). 목표 번호 → 점 목록."""
        passes: dict[int, list[dict[str, Any]]] = {}
        for index in range(1, len(goals)):
            pair = [dict(goal, map=MAP_NAME, frame_id="map") for goal in goals[index - 1:index + 1]]
            between = _through_passages(pair, self.passages)[1:-1]  # 앞뒤 정지점을 빼고 통로 점만 남긴다.
            if between:
                passes[index] = between
        return passes

    def dock(self, pinky_id: str) -> dict[str, Any] | None:
        """이 핑키가 쓸 5번(적재 자리) 웨이포인트. 자기 것이 없으면 다른 핑키가 기록한 것을 쓴다."""
        return self.waypoint(pinky_id, "5", shared=True)

    def parking_check(self, pinky_id: str) -> dict[str, Any]:
        """적재 자리에 있는 이 핑키에 대한 가상 천장 카메라의 판정(관제 서버의 주차 관측과 같은 모양).

        시야 밖이면 NOT_IN_VIEW, 움직이는 중이면 MOVING, 멈춰 있으면 위치·각도가 모두 허용 오차 안일 때 ARRIVED,
        아니면 MISALIGNED다.
        """
        with self.lock:
            pinky, dock = self.pinkies[pinky_id], self.dock(pinky_id)
            seen = pinky.seen_from(dock) if dock else None
            moving = pinky.moving > 0
        base = {"spot_id": "virtual_dock", "pinky_id": pinky_id, "tolerance": dict(CAMERA_TOLERANCE),
                "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        if seen is None:
            return {**base, "status": "NOT_IN_VIEW", "offset": None, "reason": "가상 천장 캠: 적재 자리에 핑키가 보이지 않음"}
        inside = (seen["distance_mm"] <= CAMERA_TOLERANCE["position_mm"]
                  and abs(seen["dyaw_deg"]) <= CAMERA_TOLERANCE["yaw_deg"])
        if moving:
            return {**base, "status": "MOVING", "offset": seen, "reason": "가상 천장 캠: 핑키가 아직 움직이는 중"}
        return {**base, "status": "ARRIVED" if inside else "MISALIGNED", "offset": seen,
                "reason": "가상 천장 캠: 정차 위치 안" if inside else "가상 천장 캠: 정차 위치에서 벗어남"}

    def set_bias(self, robot: object, values: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """POST /api/virtual/dock-bias: 한 핑키(robot 생략 시 전부)의 5번 위치 편향을 정한다. 빠진 값은 0이다.

        bool은 JSON에서 숫자로 섞여 들어오기 쉬우므로 숫자로 받지 않는다.
        """
        numbers = [values.get(key, 0) for key in ("forward_mm", "left_mm", "dyaw_deg")]
        if (robot not in (None, *ROBOTS)
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in numbers)):
            return 400, {"error": "robot(10, 11, 12 또는 생략)과 forward_mm, left_mm, dyaw_deg 숫자를 보내세요."}
        with self.lock:
            for pid in ([robot] if robot else list(ROBOTS)):
                self.pinkies[pid].bias = tuple(float(v) for v in numbers)
        return 200, {"ok": True, "bias": {pid: list(p.bias) for pid, p in self.pinkies.items()}}

    def redock(self, pinky_id: str, token: object, offset: object) -> tuple[int, dict[str, Any]]:
        """operator-confirm {action: redock}: 받으면 202와 카메라 offset을 썼는지, phase·token이 다르면 409."""
        with self.lock:
            guided = self.pinkies[pinky_id].redock(token, offset)
        if guided is None:
            return 409, {"error": "현재 임무의 확인 단계와 신호가 일치하지 않습니다."}
        note = (" 카메라 오프셋만큼 고쳐서 다시 들어갑니다." if guided
                else " 카메라 오프셋은 쓰지 못해 같은 자리로 다시 들어갑니다." if offset is not None else "")
        return 202, {"ok": True, "guided": guided, "message": "현재 단계 확인 신호를 받았습니다. (가상)" + note}

    def _run(self) -> None:
        """0.05초마다, 지난 실제 시간 × time_scale만큼 모든 핑키를 진행시킨다."""
        last = time.monotonic()
        while not self._stop.wait(0.05):
            now = time.monotonic()
            with self.lock:
                for pinky in self.pinkies.values():
                    pinky.tick((now - last) * self.time_scale)
            last = now

    def close(self) -> None:
        """백그라운드 진행 스레드를 멈춘다."""
        self._stop.set()

    # ---- 응답 ----
    def status(self) -> dict[str, Any]:
        """GET /api/status 응답. 실제 핑키 관제와 같은 모양이며 simulated: true가 붙는다."""
        with self.lock:
            return {"simulated": True, "robots": [self._robot(p) for p in self.pinkies.values()],
                    "busy_robot": None, "password_set": True, "monitor_error": None, "keepout": self.keepout,
                    "traffic": {"owners": {}, "waiting": {}, "error": ""}}

    def _robot(self, pinky: Pinky) -> dict[str, Any]:
        """로봇 한 대의 상태. 센서 토픽은 모두 최신(fresh)으로, 주행 준비는 늘 된 것으로 보고한다."""
        def topic(value: Any) -> dict[str, Any]:
            return {"value": value, "age_s": 0.1, "fresh": True}

        pose = {"x_m": round(pinky.x, 4), "y_m": round(pinky.y, 4), "yaw_rad": round(pinky.yaw, 4)}
        topics = {key: topic({"simulated": True}) for key in SENSOR_TOPICS}
        topics.update(
            map=topic(self.grid), tf_pose=topic(pose), amcl_pose=topic(pose),
            battery_percent=topic({"percent": pinky.battery}),
            # 배터리 %에서 만든 표시용 전압이다(0% 6.6 V ~ 100% 7.8 V). 실측 관계는 아니다.
            battery_voltage=topic({"voltage_v": round(6.6 + 0.012 * pinky.battery, 2)}),
            odom=topic({"linear_mps": pinky.moving, "angular_rps": 0.0}),
            line_control=topic({"line_algorithm": "가상 주행", "traffic_config": self.traffic_config}),
            safety_status=topic({"state": "allowed"}),
            plan=topic({"points": pinky.plan_points()}), trail=topic({"points": list(pinky.trail)}),
        )
        route = dict(pinky.route) if pinky.route else None
        return {
            "id": pinky.id, "name": ROBOTS[pinky.id]["name"] + " (가상)", "ip": "virtual", "domain": ROBOTS[pinky.id]["domain"],
            "ssh_reachable": True, "stage": "ready", "message": "가상 핑키 · 주행 준비됨", "error": None,
            "map": MAP_NAME, "simulated": True, "topics": topics, "waypoints": pinky.waypoints, "route_preview": [],
            "operational": {"state": "ready", "label": "주행 가능 (가상)", "ready_for_motion": True, "missing": []},
            "traffic": {"owners": {}, "waiting": {}, "error": ""}, "parking": pinky.parking, "route": route,
        }

    def start_mission(self, pinky_id: str, dropoff: object, via: object) -> tuple[int, dict[str, Any]]:
        """POST /mission {dropoff, via}: 주차장 -> via -> 5 -> 배달 목적지 -> 주차장. 실제 핑키 관제의 Nav2 임무와
        같다(관제 서버가 주문에 쓴다). 거기서처럼 다른 핑키가 기록한 웨이포인트도 쓴다."""
        # 목적지는 5번이 아닌 이름이어야 하고, 경유지는 문자열 최대 6개까지 받는다.
        via = [] if via is None else via
        if (not isinstance(dropoff, str) or not dropoff.strip() or dropoff.strip() == "5" or not isinstance(via, list)
                or len(via) > 6 or any(not isinstance(v, str) or not v.strip() for v in via)):
            return 400, {"error": "배달 목적지 웨이포인트 이름과 경유지 목록을 확인하세요."}
        return self.start_route(pinky_id, [v.strip() for v in via] + ["5", dropoff.strip()], shared=True,
                                execution="free_mission")

    def start_route(self, pinky_id: str, labels: object, shared: bool | None = None,
                    execution: str = "continuous_segments") -> tuple[int, dict[str, Any]]:
        """POST /route {labels}: 주차장 -> labels -> 주차장. 저장되지 않은 웨이포인트가 있으면 400, 주행 중이면 409.

        `shared`가 None이면 --shared-waypoints 설정을 따른다. 마지막 label에서 주행완료 신호를 기다린다.
        """
        if not isinstance(labels, list) or not 1 <= len(labels) <= 50 or any(not isinstance(v, str) for v in labels):
            return 400, {"error": "웨이포인트 이름 목록을 확인하세요."}
        with self.lock:
            pinky = self.pinkies[pinky_id]
            if pinky.route and pinky.route["status"] == "running":
                return 409, {"error": f"Pinky {pinky_id}가 이미 경로를 주행 중입니다."}
            selected = [self.waypoint(pinky_id, label, self.shared_waypoints if shared is None else shared)
                        for label in labels]
            missing = [label for label, item in zip(labels, selected) if item is None]
            if missing:
                return 400, {"error": "저장되지 않은 웨이포인트: " + ", ".join(missing)}
            parking = dict(pinky.parking, label="주차장")
            goals = [parking] + [dict(item) for item in selected] + [parking]
            names = ["주차장"] + list(labels) + ["주차장"]
            pinky.start(goals, names, course_end=len(goals) - 2, passes=self._passes(goals),
                        reverse_exit=self.reverse_exit, execution=execution)
        return 202, {"ok": True, "robot": pinky_id, "labels": names}

    def return_home(self, pinky_id: str, via: object) -> tuple[int, dict[str, Any]]:
        """POST /return-home {via}: 지금 위치에서 via를 거쳐 자기 주차 자리로 돌아간다. 완주 신호는 기다리지 않는다."""
        via = via if isinstance(via, list) else []
        with self.lock:
            pinky = self.pinkies[pinky_id]
            if pinky.route and pinky.route["status"] == "running":
                return 409, {"error": f"Pinky {pinky_id}가 이미 경로를 주행 중입니다."}
            passes = [self.waypoint(pinky_id, str(label), shared=True) for label in via]
            if any(item is None for item in passes):
                return 400, {"error": "저장되지 않은 웨이포인트: " + ", ".join(str(l) for l, i in zip(via, passes) if i is None)}
            # 주차 자리에서 8 cm 안이면 이미 도착한 것으로 본다.
            if math.hypot(pinky.x - pinky.parking["x_m"], pinky.y - pinky.parking["y_m"]) < 0.08:
                return 409, {"error": "이미 자기 주차 칸에 있습니다."}
            start = {"label": "출발", "x_m": pinky.x, "y_m": pinky.y, "yaw_rad": pinky.yaw}
            goals = [start] + [dict(item) for item in passes] + [dict(pinky.parking, label="주차장")]
            names = [g["label"] for g in goals]
            pinky.start(goals, names, course_end=None, passes=self._passes(goals), reverse_exit=self.reverse_exit,
                        execution="free_mission", return_home=True, via=via)
        return 202, {"ok": True, "robot": pinky_id, "labels": names}

    def confirm(self, pinky_id: str, signal: str, token: object) -> tuple[int, dict[str, Any]]:
        """operator-confirm·mission-complete 신호. phase와 token이 맞으면 202, 아니면 409."""
        with self.lock:
            if self.pinkies[pinky_id].signal(signal, token):
                return 202, {"ok": True, "message": "현재 단계 확인 신호를 받았습니다. (가상)"}
        return 409, {"error": "현재 임무의 확인 단계와 신호가 일치하지 않습니다."}

    def set_pose(self, pinky_id: str, pose: object) -> tuple[int, dict[str, Any]]:
        """POST /set-pose {pose}: 멈춰 있는 가상 핑키를 그 자세로 옮기고 자취를 지운다(시험 준비용)."""
        pose = pose if isinstance(pose, dict) else {}
        values = [pose.get(k) for k in ("x_m", "y_m", "yaw_rad")]
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
            return 400, {"error": "pose는 x_m, y_m, yaw_rad 숫자여야 합니다."}
        with self.lock:
            pinky = self.pinkies[pinky_id]
            if pinky.route and pinky.route["status"] == "running":
                return 409, {"error": "주행 중에는 위치를 지정할 수 없습니다."}
            pinky.x, pinky.y, pinky.yaw = (float(v) for v in values)
            pinky.trail = []
        return 202, {"ok": True, "message": "가상 핑키를 그 위치로 옮겼습니다."}


# ---- HTTP ------------------------------------------------------------------------------------------------------
# 실물 로봇에서만 의미 있는 핑키 관제 동작: 했다고 답하고, 여기서는 아무것도 바꾸지 않는다.
NO_OP = {"ready", "initial-pose", "localize", "reset", "nav/restart", "route/clear"}
# 첫 경로 조각이 이 이름이면 501로 답한다. "mission" 자체는 위에서 따로 처리하므로 "mission/..." 하위 경로만 걸린다.
UNSUPPORTED = {"mission", "waypoints"}


class Handler(BaseHTTPRequestHandler):
    """핑키 관제와 같은 경로에 답하는 HTTP 처리기. `fleet`은 serve()가 클래스 속성으로 묶는다."""

    fleet: VirtualFleet

    # 상태를 바꾸는 POST만 기록한다. 화면이 자주 부르는 GET /api/status까지 찍으면 로그가 넘친다.
    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002 - only POSTs, they change state
        if self.command == "POST":
            super().log_message(format, *args)

    def _json(self, status: int, body: dict[str, Any]) -> None:
        """JSON 응답을 보낸다. 모든 응답에 simulated: true를 붙인다."""
        raw = json.dumps({**body, "simulated": True}, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self) -> None:  # noqa: N802
        """상태, 가상 천장 카메라 판정, Nav2 로그(없음 안내)에 답한다."""
        path = self.path.split("?")[0]
        if path == "/api/status":
            self._json(200, self.fleet.status())
        elif re.fullmatch(r"/api/robots/(10|11|12)/parking-check", path):
            self._json(200, self.fleet.parking_check(path.split("/")[3]))
        elif re.fullmatch(r"/api/robots/(10|11|12)/nav-log", path):
            self._json(200, {"ok": True, "log": "가상 핑키 관제에는 Nav2 로그가 없습니다."})
        else:
            self._json(404, {"error": f"가상 핑키 관제에 없는 경로입니다: {path}"})

    def do_DELETE(self) -> None:  # noqa: N802
        """웨이포인트 삭제 등은 하지 않는다(501)."""
        self._json(501, {"error": "가상 핑키 관제는 웨이포인트를 바꾸지 않습니다."})

    def do_POST(self) -> None:  # noqa: N802
        """경로·임무 시작, 정지, 운영자 확인 신호, 위치 지정, 가상 편향 설정을 처리한다."""
        length = int(self.headers.get("Content-Length") or 0)
        try:
            data = json.loads(self.rfile.read(length) or b"{}") if length else {}
        except json.JSONDecodeError:
            self._json(400, {"error": "JSON 본문이 아닙니다."})
            return
        data = data if isinstance(data, dict) else {}
        path = self.path.split("?")[0]
        fleet = self.fleet
        if path in ("/api/prepare-all", "/api/credentials", "/api/reset"):  # 주행 준비·SSH 비밀번호 등은 할 일 없음
            self._json(202, {"ok": True, "message": "가상 핑키는 늘 준비돼 있습니다."})
            return
        if path == "/api/virtual/dock-bias":
            self._json(*fleet.set_bias(data.get("robot"), data))
            return
        match = re.fullmatch(r"/api/robots/(10|11|12)/(.+)", path)
        if not match:
            self._json(404, {"error": f"가상 핑키 관제에 없는 경로입니다: {path}"})
            return
        pinky_id, action = match.groups()
        if action == "route":
            self._json(*fleet.start_route(pinky_id, data.get("labels")))
        elif action == "mission":
            self._json(*fleet.start_mission(pinky_id, data.get("dropoff"), data.get("via")))
        elif action == "route/cancel":
            with fleet.lock:
                fleet.pinkies[pinky_id].stop("작업자 정지 (가상)")
            self._json(202, {"ok": True, "message": "정지했습니다. (가상)"})
        # 운영자 확인: redock은 다시 맞추기, dock_complete·received는 대기 단계를 넘긴다. 그 밖의 action은 409.
        elif action == "operator-confirm" and data.get("action") == "redock":
            self._json(*fleet.redock(pinky_id, data.get("token"), data.get("offset")))
        elif action == "operator-confirm":
            signal = {"dock_complete": "dock_complete", "received": "received"}.get(data.get("action"))
            self._json(*fleet.confirm(pinky_id, signal, data.get("token")) if signal else
                       (409, {"error": "현재 임무의 확인 단계와 신호가 일치하지 않습니다."}))
        elif action == "mission-complete":
            status, answer = fleet.confirm(pinky_id, "mission_complete", data.get("token"))
            if status == 409:
                answer = {"error": "코스 완주 후 현재 임무의 주행완료 신호만 처리할 수 있습니다."}
            self._json(status, answer)
        elif action == "return-home":
            self._json(*fleet.return_home(pinky_id, data.get("via")))
        elif action == "set-pose":
            self._json(*fleet.set_pose(pinky_id, data.get("pose")))
        elif action in NO_OP:
            self._json(202, {"ok": True, "message": "가상 핑키에서는 할 일이 없습니다."})
        elif action.split("/")[0] in UNSUPPORTED:
            self._json(501, {"error": "가상 핑키 관제에서는 지원하지 않습니다."})
        else:
            self._json(404, {"error": f"가상 핑키 관제에 없는 경로입니다: {path}"})


def serve(fleet: VirtualFleet, host: str, port: int) -> ThreadingHTTPServer:
    """`fleet`을 묶은 처리기 클래스로 HTTP 서버를 만든다(serve_forever는 부르는 쪽이 한다)."""
    handler = type("BoundHandler", (Handler,), {"fleet": fleet})
    return ThreadingHTTPServer((host, port), handler)


def main() -> int:
    """인자를 읽어 가상 핑키 세 대를 만들고, Ctrl+C까지 HTTP 서버를 돌린다."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766, help="8765 is the real dashboard's port")
    parser.add_argument("--data", type=Path, default=HERE, help="folder with map-proto.yaml/.pgm, waypoints.json, parking.json")
    parser.add_argument("--speed", type=float, default=0.15, help="driving speed in m/s")
    parser.add_argument("--time-scale", type=float, default=1.0, help="2 = everything twice as fast")
    parser.add_argument("--shared-waypoints", action="store_true",
                        help="let a Pinky use waypoints another Pinky recorded (the real /route does not)")
    parser.add_argument("--dock-bias", default="", metavar="FORWARD_MM,LEFT_MM,DYAW_DEG",
                        help="every Pinky really stands this far from where it believes it stands at station 5")
    args = parser.parse_args()
    fleet = VirtualFleet(args.data, args.speed, args.time_scale, args.shared_waypoints)
    if args.dock_bias:  # "앞mm,왼쪽mm,각도°" 세 숫자를 모든 핑키에 같은 편향으로 준다.
        forward, left, dyaw = (float(v) for v in args.dock_bias.split(","))
        fleet.set_bias(None, {"forward_mm": forward, "left_mm": left, "dyaw_deg": dyaw})
    for pid, pinky in fleet.pinkies.items():
        labels = [w["label"] for w in pinky.waypoints]
        print(f"가상 핑키 {pid} ({ROBOTS[pid]['name']}): 웨이포인트 {labels}", flush=True)
    server = serve(fleet, args.host, args.port)
    print(f"가상 핑키 관제: http://{args.host}:{args.port} (모든 응답 simulated: true)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        fleet.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
