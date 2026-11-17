#!/usr/bin/env python3
"""Virtual Pinky fleet: the Pinky team's dashboard HTTP API with three simulated Pinkys on the real Nav2 map.

The control station's order flow then runs end to end without robots, the dashboard PC or ROS:

    python3 simulation/virtual_pinky_fleet/virtual_pinky_fleet.py                 # http://127.0.0.1:8766
    CONTROL_STATION_PINKY_FLEET_URL=http://127.0.0.1:8766 ./run-control-station.sh

The map, waypoints and parking spots next to this file are copies of the team's files (2026-11-17, see README in
docs/implementation/virtual-pinky-fleet.md). Every answer carries "simulated": true, and the control station shows
the Pinkys as simulated.

Same as the real dashboard (pinky_fleet_dashboard/app.py, scripts/navigate_route.py as of 2026-11-17):
- GET /api/status: robots[] with route {status, phase, mission_token, current_label, labels, completed}, operational,
  topics (map, tf_pose, battery, plan, trail...), waypoints, parking; keepout and traffic at the top level
- POST /api/robots/<id>/route {labels}: parking -> labels -> parking with the robot's own waypoints (400 when one is
  missing, 409 while it is already on a route)
- at "5": docking_manual (token) -> dock_complete -> docking_anchor -> docked_manual (token) -> received -> dock_exit
- at the last label: course_complete (token) -> mission-complete -> returning -> back at its parking spot
- operator-confirm and mission-complete: 409 when the phase or token does not match, 202 otherwise
Not simulated: Nav2 planning (straight lines between waypoints), obstacles and collisions, shared-zone reservations,
IR line following, SSH, battery drain. Every sensor counts as present.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[1]
ROBOTS = {
    "10": {"name": "Pinky C76C", "domain": 10, "battery": 92.0},
    "11": {"name": "Pinky D631", "domain": 11, "battery": 78.0},
    "12": {"name": "Pinky 1E3E", "domain": 12, "battery": 64.0},
}
MAP_NAME = "map-proto.yaml"
SENSOR_TOPICS = ("scan", "ir", "ultrasonic", "batt_state", "imu", "odom", "joints", "line_control", "safety_status")
TRAIL_POINTS = 400


# ---- map -------------------------------------------------------------------------------------------------------
def read_map(yaml_path: Path) -> dict[str, Any]:
    """The occupancy grid Nav2's map_server makes from a trinary map YAML and its PGM (row 0 = bottom of the map)."""
    settings: dict[str, str] = {}
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        key, _, value = line.partition(":")
        if value:
            settings[key.strip()] = value.split("#")[0].strip()
    width, height, maxval, pixels = read_pgm(yaml_path.parent / settings["image"])
    origin = [float(v) for v in settings["origin"].strip("[]").split(",")]
    occupied, free = float(settings.get("occupied_thresh", 0.65)), float(settings.get("free_thresh", 0.196))
    negate = settings.get("negate", "0") == "1"
    cells = []
    for row in range(height - 1, -1, -1):
        for col in range(width):
            value = pixels[row * width + col]
            occupancy = value / maxval if negate else (maxval - value) / maxval
            cells.append(100 if occupancy > occupied else 0 if occupancy < free else -1)
    return {"width": width, "height": height, "resolution_m": float(settings["resolution"]),
            "origin_x_m": origin[0], "origin_y_m": origin[1], "origin_yaw_rad": origin[2], "cells": cells,
            "source": "virtual_local_file"}


def read_pgm(path: Path) -> tuple[int, int, int, list[int]]:
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
    if magic == b"P5":
        return width, height, maxval, list(data[pos + 1:pos + 1 + width * height])
    if magic == b"P2":
        return width, height, maxval, [int(v) for v in data[pos:].split()][:width * height]
    raise ValueError(f"{path}: PGM P5 or P2 only, got {magic!r}")


# ---- one Pinky -------------------------------------------------------------------------------------------------
class Pinky:
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
        self.moving = 0.0

    # Steps: drive (to a point, maybe backwards), wait (for an operator signal), hold (a timed phase), phase (instant).
    def start(self, goals: list[dict[str, Any]], labels: list[str], course_end: int | None, **route: Any) -> None:
        steps: list[dict[str, Any]] = [{"kind": "phase", "phase": "driving", "message": "Nav2 경로 주행 중 (가상)"}]
        for index, goal in enumerate(goals[1:], start=1):
            steps.append({"kind": "drive", "x": goal["x_m"], "y": goal["y_m"], "label": goal["label"], "index": index,
                          "yaw": goal.get("yaw_rad") if goal["label"] == "5" or index == len(goals) - 1 else None})
            if goal["label"] == "5":
                steps += [
                    {"kind": "wait", "phase": "docking_manual", "signal": "dock_complete",
                     "message": "5번 도착 · 정지 · 수동 도킹완료 확인 대기"},
                    {"kind": "hold", "phase": "docking_anchor", "seconds": 2.0, "message": "5번 위치 다시 맞추는 중"},
                    {"kind": "wait", "phase": "docked_manual", "signal": "received",
                     "message": "도킹완료 · 정지 유지 · 물건 수령 완료 대기"},
                    {"kind": "drive", "phase": "dock_exit", "back_m": 0.20, "label": "5", "index": index,
                     "message": "수령 확인 · 20 cm 후진"},
                    {"kind": "phase", "phase": "driving", "message": "Nav2 경로 주행 중 (가상)"},
                ]
            if index == course_end:
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
        self.steps = []
        if self.route and self.route["status"] == "running":
            self.route.update(status="cancelled", mission_token=None, message=message,
                              stopped_pose={"x_m": self.x, "y_m": self.y, "yaw_rad": self.yaw})

    def signal(self, name: str, token: object) -> bool:
        """An operator signal for the waiting step: True when phase and token match (then the Pinky moves on)."""
        step = self.steps[0] if self.steps else None
        if (not self.route or self.route["status"] != "running" or not step or step["kind"] != "wait"
                or step["signal"] != name or not token or token != self.route.get("mission_token")):
            return False
        self.steps.pop(0)
        self.route["mission_token"] = None
        return True

    def tick(self, dt: float) -> None:
        self.moving = 0.0
        while self.route and self.route["status"] == "running":
            if not self.steps:
                self.route.update(status="complete", mission_token=None, message="경로 완료 · 주차장 도착 (가상)")
                return
            step = self.steps[0]
            if step.get("entered") is None:
                step["entered"] = True
                self._enter(step)
            if step["kind"] == "wait":
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
        route = self.route
        if "phase" in step:
            token = uuid.uuid4().hex if step["kind"] == "wait" else None
            route.update(phase=step["phase"], mission_token=token, message=step["message"])
        if step["kind"] == "drive":
            if "back_m" in step:
                step["x"] = self.x - step["back_m"] * math.cos(self.yaw)
                step["y"] = self.y - step["back_m"] * math.sin(self.yaw)
            else:
                route.update(current_index=step["index"], current_label=step["label"],
                             message=f"웨이포인트 {step['label']}로 이동 중 (가상)")

    def _drive(self, step: dict[str, Any], dt: float) -> float:
        """Move toward the step's point; the time left over after arriving, or 0 while still driving."""
        dx, dy = step["x"] - self.x, step["y"] - self.y
        distance = math.hypot(dx, dy)
        reach = self.speed * dt
        if distance > 1e-6 and "back_m" not in step:
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
        if step.get("yaw") is not None:
            self.yaw = step["yaw"]
        if "back_m" not in step:
            self.route.update(completed=step["index"], current_label=step["label"],
                              message=f"웨이포인트 {step['label']} 도착 (가상)")

    def _mark(self) -> None:
        if not self.trail or math.hypot(self.trail[-1][0] - self.x, self.trail[-1][1] - self.y) >= 0.02:
            self.trail = (self.trail + [[round(self.x, 3), round(self.y, 3)]])[-TRAIL_POINTS:]

    def plan_points(self) -> list[list[float]]:
        points = [[round(self.x, 3), round(self.y, 3)]]
        points += [[round(s["x"], 3), round(s["y"], 3)] for s in self.steps if s["kind"] == "drive" and "x" in s]
        return points if len(points) > 1 else []


# ---- the fleet -------------------------------------------------------------------------------------------------
class VirtualFleet:
    def __init__(self, data_dir: Path, speed_mps: float, time_scale: float, shared_waypoints: bool) -> None:
        self.grid = read_map(data_dir / "map-proto.yaml")
        parking = json.loads((data_dir / "parking.json").read_text(encoding="utf-8"))
        waypoints = json.loads((data_dir / "waypoints.json").read_text(encoding="utf-8"))
        self.shared_waypoints = shared_waypoints
        self.time_scale = time_scale
        self.lock = threading.Lock()
        self.pinkies = {pid: Pinky(pid, parking[pid], waypoints.get(pid, []), speed_mps) for pid in ROBOTS}
        keepout_yaml = REPOSITORY_ROOT / "pinky_fleet_dashboard" / "config" / "keepout" / "keepout_mask.yaml"
        self.keepout = dict(read_map(keepout_yaml), map=MAP_NAME, frame_id="map", available=True,
                            source="keepout_mask.yaml", display_only=True) if keepout_yaml.is_file() else None
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
        """The robot's own waypoint `label`; with `shared`, else one another Pinky recorded (the dashboard's
        _shared_waypoint, used by return-home)."""
        order = [pinky_id] + ([p for p in ROBOTS if p != pinky_id] if shared else [])
        for pid in order:
            for item in self.pinkies[pid].waypoints:
                if item.get("label") == label and item.get("map") == MAP_NAME and item.get("frame_id") == "map":
                    return item
        return None

    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.wait(0.05):
            now = time.monotonic()
            with self.lock:
                for pinky in self.pinkies.values():
                    pinky.tick((now - last) * self.time_scale)
            last = now

    def close(self) -> None:
        self._stop.set()

    # ---- answers ----
    def status(self) -> dict[str, Any]:
        with self.lock:
            return {"simulated": True, "robots": [self._robot(p) for p in self.pinkies.values()],
                    "busy_robot": None, "password_set": True, "monitor_error": None, "keepout": self.keepout,
                    "traffic": {"owners": {}, "waiting": {}, "error": ""}}

    def _robot(self, pinky: Pinky) -> dict[str, Any]:
        def topic(value: Any) -> dict[str, Any]:
            return {"value": value, "age_s": 0.1, "fresh": True}

        pose = {"x_m": round(pinky.x, 4), "y_m": round(pinky.y, 4), "yaw_rad": round(pinky.yaw, 4)}
        topics = {key: topic({"simulated": True}) for key in SENSOR_TOPICS}
        topics.update(
            map=topic(self.grid), tf_pose=topic(pose), amcl_pose=topic(pose),
            battery_percent=topic({"percent": pinky.battery}),
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
        """POST /mission {dropoff, via}: parking -> via -> 5 -> the drop-off -> parking, as the real dashboard's Nav2
        mission (which the control station uses for orders). Waypoints another Pinky recorded count, as there."""
        via = [] if via is None else via
        if (not isinstance(dropoff, str) or not dropoff.strip() or dropoff.strip() == "5" or not isinstance(via, list)
                or len(via) > 6 or any(not isinstance(v, str) or not v.strip() for v in via)):
            return 400, {"error": "배달 목적지 웨이포인트 이름과 경유지 목록을 확인하세요."}
        return self.start_route(pinky_id, [v.strip() for v in via] + ["5", dropoff.strip()], shared=True,
                                execution="free_mission")

    def start_route(self, pinky_id: str, labels: object, shared: bool | None = None,
                    execution: str = "continuous_segments") -> tuple[int, dict[str, Any]]:
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
            pinky.start(goals, names, course_end=len(goals) - 2, execution=execution)
        return 202, {"ok": True, "robot": pinky_id, "labels": names}

    def return_home(self, pinky_id: str, via: object) -> tuple[int, dict[str, Any]]:
        via = via if isinstance(via, list) else []
        with self.lock:
            pinky = self.pinkies[pinky_id]
            if pinky.route and pinky.route["status"] == "running":
                return 409, {"error": f"Pinky {pinky_id}가 이미 경로를 주행 중입니다."}
            passes = [self.waypoint(pinky_id, str(label), shared=True) for label in via]
            if any(item is None for item in passes):
                return 400, {"error": "저장되지 않은 웨이포인트: " + ", ".join(str(l) for l, i in zip(via, passes) if i is None)}
            if math.hypot(pinky.x - pinky.parking["x_m"], pinky.y - pinky.parking["y_m"]) < 0.08:
                return 409, {"error": "이미 자기 주차 칸에 있습니다."}
            start = {"label": "출발", "x_m": pinky.x, "y_m": pinky.y, "yaw_rad": pinky.yaw}
            goals = [start] + [dict(item) for item in passes] + [dict(pinky.parking, label="주차장")]
            names = [g["label"] for g in goals]
            pinky.start(goals, names, course_end=None, execution="free_mission", return_home=True, via=via)
        return 202, {"ok": True, "robot": pinky_id, "labels": names}

    def confirm(self, pinky_id: str, signal: str, token: object) -> tuple[int, dict[str, Any]]:
        with self.lock:
            if self.pinkies[pinky_id].signal(signal, token):
                return 202, {"ok": True, "message": "현재 단계 확인 신호를 받았습니다. (가상)"}
        return 409, {"error": "현재 임무의 확인 단계와 신호가 일치하지 않습니다."}

    def set_pose(self, pinky_id: str, pose: object) -> tuple[int, dict[str, Any]]:
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
# Dashboard actions that only matter on a real robot: answered as done, nothing changes here.
NO_OP = {"ready", "initial-pose", "localize", "reset", "nav/restart", "route/clear"}
UNSUPPORTED = {"mission", "waypoints"}


class Handler(BaseHTTPRequestHandler):
    fleet: VirtualFleet

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002 - only POSTs, they change state
        if self.command == "POST":
            super().log_message(format, *args)

    def _json(self, status: int, body: dict[str, Any]) -> None:
        raw = json.dumps({**body, "simulated": True}, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?")[0]
        if path == "/api/status":
            self._json(200, self.fleet.status())
        elif re.fullmatch(r"/api/robots/(10|11|12)/nav-log", path):
            self._json(200, {"ok": True, "log": "가상 핑키 관제에는 Nav2 로그가 없습니다."})
        else:
            self._json(404, {"error": f"가상 핑키 관제에 없는 경로입니다: {path}"})

    def do_DELETE(self) -> None:  # noqa: N802
        self._json(501, {"error": "가상 핑키 관제는 웨이포인트를 바꾸지 않습니다."})

    def do_POST(self) -> None:  # noqa: N802
        length = int(self.headers.get("Content-Length") or 0)
        try:
            data = json.loads(self.rfile.read(length) or b"{}") if length else {}
        except json.JSONDecodeError:
            self._json(400, {"error": "JSON 본문이 아닙니다."})
            return
        data = data if isinstance(data, dict) else {}
        path = self.path.split("?")[0]
        fleet = self.fleet
        if path in ("/api/prepare-all", "/api/credentials", "/api/reset"):
            self._json(202, {"ok": True, "message": "가상 핑키는 늘 준비돼 있습니다."})
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
    handler = type("BoundHandler", (Handler,), {"fleet": fleet})
    return ThreadingHTTPServer((host, port), handler)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766, help="8765 is the real dashboard's port")
    parser.add_argument("--data", type=Path, default=HERE, help="folder with map-proto.yaml/.pgm, waypoints.json, parking.json")
    parser.add_argument("--speed", type=float, default=0.15, help="driving speed in m/s")
    parser.add_argument("--time-scale", type=float, default=1.0, help="2 = everything twice as fast")
    parser.add_argument("--shared-waypoints", action="store_true",
                        help="let a Pinky use waypoints another Pinky recorded (the real /route does not)")
    args = parser.parse_args()
    fleet = VirtualFleet(args.data, args.speed, args.time_scale, args.shared_waypoints)
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
