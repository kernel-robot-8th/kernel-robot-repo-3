#!/usr/bin/env python3
"""Local Pinky fleet dashboard and sequential bringup coordinator."""

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

from scripts.route_geometry import build_dense_route
from scripts.line_following import default_profile, capture_profile, normalize, validate_profile
from scripts.ir_profiles import IRProfiles
from scripts.ir_calibration import IRCalibrationStore

ROOT = Path(__file__).resolve().parent
PARKING = json.loads((ROOT / "config" / "parking.json").read_text())
WEB = ROOT / "web"
ROS_DOMAIN = 0
HOST = "127.0.0.1"
PORT = 8765
FRESH_SECONDS = 4.0
START_TIMEOUT = 20.0
WAYPOINTS_PATH = ROOT / "data" / "waypoints.json"
LOCAL_MAP_DIR = Path.home() / "Documents" / "Pinky_backup"
LOCAL_MAP_YAML = LOCAL_MAP_DIR / "map-proto.yaml"
LOCAL_MAP_IMAGE = LOCAL_MAP_DIR / "map-proto.pgm"
REMOTE_MAP_DIR = "/home/pinky/.cache/pinky-fleet/maps"
REMOTE_MAP_YAML = f"{REMOTE_MAP_DIR}/map-proto.yaml"
REMOTE_MAP_IMAGE = f"{REMOTE_MAP_DIR}/map-proto.pgm"

ROBOTS: dict[str, dict[str, Any]] = {
    "10": {"id": "10", "name": "Pinky C76C", "ip": "192.168.0.9", "domain": 10, "ns": "/pinky_10"},
    "11": {"id": "11", "name": "Pinky D631", "ip": "192.168.0.8", "domain": 11, "ns": "/pinky_11"},
    "12": {"id": "12", "name": "Pinky 1E3E", "ip": "192.168.0.11", "domain": 12, "ns": "/pinky_12"},
}


def _blank_robot(robot: dict[str, Any]) -> dict[str, Any]:
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
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.robots = {key: _blank_robot(value) for key, value in ROBOTS.items()}
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
        WAYPOINTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=WAYPOINTS_PATH.parent, delete=False) as stream:
            json.dump(self.waypoints, stream, ensure_ascii=False, indent=2)
            temp_path = Path(stream.name)
        os.replace(temp_path, WAYPOINTS_PATH)

    def update(self, robot_id: str, **values: Any) -> None:
        with self.lock:
            self.robots[robot_id].update(values)


STATE = FleetState()
IR_PROFILES = IRProfiles(ROOT / "data" / "ir_profiles.json", list(ROBOTS))
IR_CALIBRATION = IRCalibrationStore(ROOT / 'data' / 'ir_calibration')
WORKERS = ThreadPoolExecutor(max_workers=8, thread_name_prefix="pinky-fleet")
DOCK_COMMAND_LOCK = threading.Lock()


def _finite(value: Any) -> float | None:
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def _read_pgm(path: Path) -> tuple[int, int, list[int]] | None:
    """Read the small binary or ASCII PGM used by the local Nav2 map."""
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    cursor = 0

    def token() -> bytes | None:
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
        # maxval is followed by one whitespace separator (CRLF counts as one).
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


def _load_local_map_grid() -> dict[str, Any] | None:
    """Build an OccupancyGrid-shaped fallback from map-proto's YAML and PGM."""
    try:
        yaml_text = LOCAL_MAP_YAML.read_text(encoding="utf-8")
        image_name = re.search(r"(?m)^image:\s*(\S+)\s*$", yaml_text).group(1)  # type: ignore[union-attr]
        resolution = float(re.search(r"(?m)^resolution:\s*([0-9.eE+-]+)", yaml_text).group(1))  # type: ignore[union-attr]
        origin_match = re.search(r"(?m)^origin:\s*\[([^]]+)\]", yaml_text)
        origin = [float(item.strip()) for item in origin_match.group(1).split(",")] if origin_match else []
        occupied = float(re.search(r"(?m)^occupied_thresh:\s*([0-9.eE+-]+)", yaml_text).group(1))  # type: ignore[union-attr]
        free = float(re.search(r"(?m)^free_thresh:\s*([0-9.eE+-]+)", yaml_text).group(1))  # type: ignore[union-attr]
        negate = int(re.search(r"(?m)^negate:\s*(\d+)", yaml_text).group(1))  # type: ignore[union-attr]
        pgm = _read_pgm(LOCAL_MAP_DIR / image_name)
    except (OSError, ValueError, AttributeError):
        return None
    if pgm is None or len(origin) < 3:
        return None
    width, height, pixels = pgm
    cells: list[int] = []
    # PGM rows run top-to-bottom; OccupancyGrid rows run bottom-to-top.
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


LOCAL_MAP_GRID = _load_local_map_grid()


class Telemetry:
    """ROS 2 subscriptions in PC domain 0; bridges remap each robot into a namespace."""

    KEYS = ("scan", "ir", "ultrasonic", "batt_state", "imu", "battery_percent", "battery_voltage", "odom", "joints", "map", "amcl_pose", "tf_pose", "line_control")

    def __init__(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid, Odometry
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
        self.base_frames = {rid: "base_footprint" for rid in ROBOTS}
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
        }
        self.domain_readers = []
        for rid, robot in ROBOTS.items():
            context = rclpy.context.Context()
            rclpy.init(args=[], context=context, domain_id=robot['domain'])
            node = Node('pinky_fleet_reader_'+rid,context=context)
            executor = MultiThreadedExecutor(num_threads=2,context=context)
            executor.add_node(node)
            self.domain_readers.append((context,node,executor))
            for key, msg_type in types.items():
                topic = f"/{paths[key]}"
                subscription_qos = QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
                if key == "scan":
                    # LiDAR drivers commonly publish LaserScan with the sensor
                    # data profile (best effort). A reliable-only subscriber
                    # is QoS-incompatible and silently receives no scans.
                    subscription_qos = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                    )
                elif key == "map":
                    # Nav2 publishes OccupancyGrid with transient-local durability.
                    # Subscribe with matching durability so a map published before
                    # this dashboard starts is replayed to the late joiner.
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
        now = time.time()
        value: Any
        if key == "ir":
            value = {"channels": [int(v) for v in msg.data]}
        elif key == "line_control":
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    value = {"state": "invalid_status"}
            except (ValueError, TypeError):
                value = {"state": "invalid_status"}
        elif key == "ultrasonic":
            value = {"range_m": _finite(msg.range), "min_m": _finite(msg.min_range), "max_m": _finite(msg.max_range)}
        elif key == "scan":
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
                # OccupancyGrid.data is row-major from the map's lower-left
                # origin. Keeping that exact payload lets the browser render
                # and place the robot in the same Nav2 map frame.
                "cells": [int(cell) for cell in msg.data],
            }
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
                    "xy": _finite((covariance[1] + covariance[6]) / 2.0),
                    "yy": _finite(covariance[7]),
                },
            }
        else:
            value = None
        with self.changed:
            self.data[robot_id][key] = {"value": value, "received_at": now}
            if key == "ir":
                self.ir_history[robot_id].append((now, value['channels']))
            if key == "odom":
                self._update_tf_pose_unlocked(robot_id, now)
            self.changed.notify_all()

    def _receive_tf(self, robot_id: str, msg: Any, is_static: bool) -> None:
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
            if frame == self.base_frames[robot_id] and not edge["static"]:
                base_received_at = edge["received_at"]
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

    def wait_for(self, robot_id: str, keys: tuple[str, ...], timeout: float) -> tuple[bool, list[str]]:
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

    def _fresh_unlocked(self, robot_id: str, key: str) -> bool:
        received = self.data[robot_id][key]["received_at"]
        if key == "map":
            # Static map is published once with transient-local durability.
            return received is not None
        return received is not None and (time.time() - received) <= FRESH_SECONDS

    def snapshot(self) -> dict[str, Any]:
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
                result[robot_id] = topics
            return result


TELEMETRY: Telemetry | None = None


def _topic_ready(robot_id: str, keys: tuple[str, ...], timeout: float = START_TIMEOUT) -> tuple[bool, list[str]]:
    if TELEMETRY is None:
        return False, list(keys)
    return TELEMETRY.wait_for(robot_id, keys, timeout)


def _ssh_run(robot_id: str, remote_command: str, timeout_seconds: float = 35.0) -> tuple[bool, str]:
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
        # Use OpenSSH askpass so the dashboard has no third-party Python
        # dependency. The credential stays out of argv and the script file.
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
            result = subprocess.run(
                ["ssh", *args],
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                timeout=timeout_seconds,
                env=child_env,
                check=False,
            )
        combined = (result.stdout + result.stderr).replace(password, "[redacted]")
        if result.returncode != 0:
            if "Permission denied" in combined:
                return False, "SSH 인증이 거부되었습니다."
            return False, combined[-1200:] or f"SSH 종료 코드 {result.returncode}"
        STATE.update(robot_id, last_ssh_ok=time.time(), ssh_reachable=True)
        return True, combined[-1200:]
    except subprocess.TimeoutExpired:
        return False, "SSH 명령이 시간 초과되었습니다."
    except OSError as exc:
        return False, str(exc)


def _remote_start_command(robot_id: str, stage: str, ros_command: str) -> str:
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
    return (
        f"mkdir -p /home/pinky/.cache/pinky-fleet && "
        f"if [ -s {shlex.quote(pid_path)} ] && kill -0 \"$(cat {shlex.quote(pid_path)})\" 2>/dev/null; then "
        f"echo ALREADY_RUNNING; exit 0; fi; "
        f"nohup bash -lc {quoted_inner} >> {shlex.quote(log_path)} 2>&1 < /dev/null & "
        f"echo $! > {shlex.quote(pid_path)}; sleep 0.3; "
        f"if kill -0 \"$(cat {shlex.quote(pid_path)})\" 2>/dev/null; then echo STARTED; "
        f"else echo FAILED; tail -40 {shlex.quote(log_path)}; exit 1; fi"
    )


def _remote_map_path(robot_id: str, map_name: str) -> str | None:
    if map_name == "map-proto.yaml":
        if not LOCAL_MAP_YAML.is_file() or not LOCAL_MAP_IMAGE.is_file():
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
        ok, _ = _ssh_run(robot_id, install_command)
        return REMOTE_MAP_YAML if ok else None

    # Saved maps can live outside the SSH login working directory. Nav2's map
    # server needs the YAML path itself, so resolve the selected filename on
    # the robot and pass an absolute path to the launch file.
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


def _nav_action_ready(robot_id: str) -> tuple[bool, str]:
    command = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/pinky/pinky_pro/install/setup.bash && "
        f"export ROS_DOMAIN_ID={ROBOTS[robot_id]['domain']} && "
        "for attempt in $(seq 1 30); do "
        "actions=$(timeout 2s ros2 action list -t 2>&1 || true); "
        "if printf '%s\\n' \"$actions\" | grep -Fq '/navigate_to_pose [nav2_msgs/action/NavigateToPose]'; then "
        "echo NAV_ACTION_READY; exit 0; fi; "
        "sleep 1; done; echo NAV_ACTION_MISSING; printf '%s\\n' \"$actions\"; exit 1"
    )
    return _ssh_run(robot_id, command, timeout_seconds=100.0)


def _stop_tracked_navigation(robot_id: str) -> tuple[bool, str]:
    # A previous failed attempt may have left a live launch process. Signal
    # only the PID recorded by this dashboard, then let ROS launch clean up its
    # child processes before retrying.
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


def _stop_existing_navigation(robot_id: str) -> tuple[bool, str]:
    """Stop tracked and orphaned Nav2 processes before launching one clean stack."""
    command = (
        "pidfile=/home/pinky/.cache/pinky-fleet/navigation.pid; "
        "if [ -s \"$pidfile\" ]; then "
        "pid=$(cat \"$pidfile\"); case \"$pid\" in ''|*[!0-9]*) ;; *) "
        "if kill -0 \"$pid\" 2>/dev/null; then "
        "cmdline=$(tr '\\0' ' ' < \"/proc/$pid/cmdline\" 2>/dev/null || true); "
        "case \"$cmdline\" in *'ros2 launch pinky_navigation bringup_launch.xml'*) "
        "kill -INT \"$pid\" 2>/dev/null || true; "
        "for n in 1 2 3 4 5; do kill -0 \"$pid\" 2>/dev/null || break; sleep 1; done; "
        "if kill -0 \"$pid\" 2>/dev/null; then kill -TERM \"$pid\" 2>/dev/null || true; fi;; esac; "
        "fi;; esac; rm -f \"$pidfile\"; fi; "
        "for pattern in '[r]os2 launch pinky_navigation bringup_launch.xml' "
        "'[n]av2_' '[r]clcpp_components/component_container'; do "
        "for pid in $(pgrep -f \"$pattern\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; kill -INT \"$pid\" 2>/dev/null || true; done; done; "
        "for n in 1 2 3 4 5; do "
        "pgrep -f '[r]os2 launch pinky_navigation bringup_launch.xml|[n]av2_|[r]clcpp_components/component_container' >/dev/null || break; "
        "sleep 1; done; "
        "for pattern in '[r]os2 launch pinky_navigation bringup_launch.xml' "
        "'[n]av2_' '[r]clcpp_components/component_container'; do "
        "for pid in $(pgrep -f \"$pattern\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; kill -KILL \"$pid\" 2>/dev/null || true; done; done; "
        "echo NAV_PROCESSES_CLEARED"
    )
    return _ssh_run(robot_id, command)


def _stop_existing_base(robot_id: str) -> tuple[bool, str]:
    """Clear stale Pinky base/LiDAR launches before starting a fresh bringup."""
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


def _robot_reset_command(robot_id: str) -> str:
    """Stop processes started by this dashboard, plus a lingering teleop node."""
    stage_commands = {
        "navigation": "pinky_navigation bringup_launch.xml",
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
    """Install and run the Nav2 action client on the target Pinky."""
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
    payload = base64.b64encode(json.dumps(profile, allow_nan=False).encode('utf-8')).decode('ascii')
    target = '/home/pinky/.cache/pinky-fleet/nav2/ir_profile.json'
    return ("mkdir -p /home/pinky/.cache/pinky-fleet/nav2 && "
            f"printf '%s' {shlex.quote(payload)} | base64 -d > {target}.tmp && "
            f"mv {target}.tmp {target} && echo IR_PROFILE_SAVED")


def _nav2_runtime_install_command(robot_id: str) -> str:
    installer = (ROOT / "scripts" / "install_nav2_runtime.py").read_bytes()
    safety = (ROOT / "scripts" / "ultrasonic_safety.py").read_bytes()
    installer_b64 = base64.b64encode(installer).decode("ascii")
    safety_b64 = base64.b64encode(safety).decode("ascii")
    line_b64 = base64.b64encode((ROOT / 'scripts' / 'line_following.py').read_bytes()).decode('ascii')
    route_b64 = base64.b64encode((ROOT / 'scripts' / 'navigate_route.py').read_bytes()).decode('ascii')
    mission_b64 = base64.b64encode((ROOT / 'scripts' / 'mission_behaviors.py').read_bytes()).decode('ascii')
    runtime = "/home/pinky/.cache/pinky-fleet/nav2"
    return (
        f"mkdir -p /home/pinky/.cache/pinky-fleet {runtime} && "
        f"printf '%s' {shlex.quote(installer_b64)} | base64 -d > /home/pinky/.cache/pinky-fleet/install_nav2_runtime.py && "
        f"printf '%s' {shlex.quote(safety_b64)} | base64 -d > {runtime}/ultrasonic_safety.py && "
        "chmod 700 /home/pinky/.cache/pinky-fleet/install_nav2_runtime.py "
        f"{runtime}/ultrasonic_safety.py && "
        f"printf '%s' {shlex.quote(line_b64)} | base64 -d > {runtime}/line_following.py && "
        f"printf '%s' {shlex.quote(route_b64)} | base64 -d > /home/pinky/.cache/pinky-fleet/navigate_route.py && "
        f"printf '%s' {shlex.quote(mission_b64)} | base64 -d > {runtime}/mission_behaviors.py && "
        + _ir_profile_install_command(IR_PROFILES.get(robot_id)) + " && "
        "python3 /home/pinky/.cache/pinky-fleet/install_nav2_runtime.py"
    )


def _stop_ultrasonic_safety(robot_id: str) -> tuple[bool, str]:
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
        "echo SAFETY_STOPPED ;; "
        "*) echo SAFETY_PID_MISMATCH ;; "
        "esac; "
        "else echo SAFETY_NOT_RUNNING; fi; rm -f \"$pidfile\" ;; "
        "esac; "
        "else echo SAFETY_NOT_RUNNING; fi"
    )
    return _ssh_run(robot_id, command)


def _start_navigation_with_safety(robot_id: str, map_path: str) -> tuple[bool, str]:
    stopped, stop_output = _stop_ultrasonic_safety(robot_id)
    if not stopped or "SAFETY_PID_MISMATCH" in stop_output:
        return False, stop_output or "이전 초음파 안전 감시기를 정리하지 못했습니다."
    ok, output = _ssh_run(robot_id, _nav2_runtime_install_command(robot_id))
    if not ok or "NAV2_RUNTIME_READY" not in output:
        return False, output or "Nav2 안전 설정을 만들지 못했습니다."

    safety_command = (
        "python3 /home/pinky/.cache/pinky-fleet/nav2/ultrasonic_safety.py"
    )
    ok, output = _ssh_run(robot_id, _remote_start_command(robot_id, "ultrasonic_safety", safety_command))
    if not ok or not any(token in output for token in ("STARTED", "ALREADY_RUNNING")):
        return False, output or "초음파 안전 감시기를 시작하지 못했습니다."

    params_path = "/home/pinky/.cache/pinky-fleet/nav2/nav2_params.yaml"
    launch_path = "/home/pinky/.cache/pinky-fleet/nav2/bringup_launch.xml"
    command = (
        f"ros2 launch {shlex.quote(launch_path)} map:={shlex.quote(map_path)} "
        f"use_composition:=False params_file:={shlex.quote(params_path)}"
    )
    return _ssh_run(robot_id, _remote_start_command(robot_id, "navigation", command))


def _initial_pose_command(robot_id: str) -> str:
    script = (ROOT / "scripts" / "set_initial_pose.py").read_bytes()
    script_b64 = base64.b64encode(script).decode("ascii")
    remote_script = "/home/pinky/.cache/pinky-fleet/set_initial_pose.py"
    return (
        "mkdir -p /home/pinky/.cache/pinky-fleet && "
        f"printf '%s' {shlex.quote(script_b64)} | base64 -d > {shlex.quote(remote_script)} && "
        f"source /opt/ros/jazzy/setup.bash && source /home/pinky/pinky_pro/install/setup.bash && "
        f"export ROS_DOMAIN_ID={ROBOTS[robot_id]['domain']} && python3 {shlex.quote(remote_script)} {PARKING[robot_id]['x_m']} {PARKING[robot_id]['y_m']} {PARKING[robot_id]['yaw_rad']}"
    )


def _set_initial_pose(robot_id: str) -> None:
    try:
        topics=TELEMETRY.snapshot().get(robot_id,{}) if TELEMETRY else {}
        if any(not topics.get(key,{}).get('fresh') for key in ('scan','odom','map','line_control')):
            STATE.update(robot_id,message='실제 구동 상태가 없어 주행준비를 복구합니다.',error=None)
            _prepare_robot(robot_id,STATE.robots[robot_id]['map'],release_busy=False)
            if STATE.robots[robot_id]['stage']!='prepared':return
        ok,output=_ssh_run(robot_id,_initial_pose_command(robot_id))
        if not ok and 'AMCL is not subscribed' in output:
            STATE.update(robot_id,message='AMCL 미실행 · 주행준비를 한 번 복구합니다.',error=None)
            _prepare_robot(robot_id,STATE.robots[robot_id]['map'],release_busy=False)
            if STATE.robots[robot_id]['stage']!='prepared':return
            ok,output=_ssh_run(robot_id,_initial_pose_command(robot_id))
        if ok and 'Published AMCL initial pose' in output:
            STATE.update(robot_id,stage='prepared',message=f"AMCL 위치를 {PARKING[robot_id]['label']} 좌표로 설정했습니다.",error=None)
        else:
            STATE.update(robot_id,stage='fault',message='AMCL 위치 설정 실패 · 주행준비가 필요합니다.',error=output or 'AMCL 응답 없음')
    finally:
        with STATE.lock:
            if STATE.busy_robot==robot_id:STATE.busy_robot=None


def _run_waypoint_route(robot_id: str, goals: list[dict[str, Any]]) -> None:
    """Stream per-waypoint progress from the robot's Nav2 action client."""
    temp_dir = None
    with STATE.lock:
        route_ref=STATE.routes[robot_id]
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
            if route is route_ref and route and route.get('status')!='cancelled':
                route.update(status=terminal_status,
                             message={"complete": "임무 종료 · 자기 주차장 복귀 완료",
                                      "cancelled": "주행 취소를 요청했습니다.",
                                      "failed": route.get("message") if route.get("message", "").startswith("Nav2") else "경로 주행이 중단됐습니다."}[terminal_status])


def _cancel_waypoint_route(robot_id: str) -> None:
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
            if ok and 'STOP_LATCHED' in output:
                route.update(status='cancelled',message='정지 요청 적용 · 경로 취소',detail='정지 유지 상태를 설정했습니다.')
            else:
                route.update(status='failed',message='정지 적용 확인 실패',detail=output[-1200:])


def _prepare_robot(robot_id: str, map_name: str, release_busy: bool = True) -> None:
    with STATE.lock:
        STATE.robots[robot_id].update(stage="base_starting", map=map_name, message="기본 bringup과 LiDAR를 시작합니다.", error=None)
    try:
        cleared, clear_output = _stop_existing_base(robot_id)
        if not cleared or "BASE_PROCESSES_CLEARED" not in clear_output:
            raise RuntimeError("이전 기본 구동·LiDAR 프로세스를 정리하지 못했습니다: " + clear_output.strip())
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
                    raise RuntimeError(f"{label} 데이터가 브릿지로 들어오지 않습니다 ({names})." + ("\n" + detail[-1400:] if detail else ""))

        required_sensors = ("ir", "ultrasonic", "imu", "battery_percent", "battery_voltage")
        ready, missing = _topic_ready(robot_id, required_sensors)
        if not ready:
            raise RuntimeError("센서 데이터 확인 실패: " + ", ".join(missing))

        map_path = _remote_map_path(robot_id, map_name)
        if map_path is None:
            if map_name == "map-proto.yaml":
                raise RuntimeError("PC의 map-proto.yaml/map-proto.pgm 복사 실패 또는 파일이 없습니다.")
            raise RuntimeError(f"로봇에서 {map_name} 파일을 /home/pinky 아래에서 찾지 못했습니다.")

        stopped, stop_output = _stop_existing_navigation(robot_id)
        if not stopped or "NAV_PROCESSES_CLEARED" not in stop_output:
            raise RuntimeError(f"이전 Nav2 실행을 정리하지 못했습니다: {stop_output.strip()}")

        STATE.update(robot_id, stage="nav_starting", message=f"Nav2와 {map_name} 지도를 시작합니다.")
        ok, output = _start_navigation_with_safety(robot_id, map_path)
        if not ok:
            raise RuntimeError(f"Nav2 시작 실패: {output.strip() or 'SSH 오류'}")
        map_ready, missing = _topic_ready(robot_id, ("map",), timeout=25.0)
        if not map_ready:
            _, diagnostics = _ssh_run(robot_id, _nav_diagnostics_command(robot_id))
            detail = diagnostics.strip() or "Nav2 로그를 가져오지 못했습니다."
            raise RuntimeError("Nav2가 /map을 발행하지 않습니다. map_server 상태와 로그: " + detail[-1600:])
        action_ready, action_detail = _nav_action_ready(robot_id)
        if not action_ready:
            _, diagnostics = _ssh_run(robot_id, _nav_diagnostics_command(robot_id))
            detail = (action_detail + "\n" + diagnostics).strip() or "Nav2 액션 진단을 가져오지 못했습니다."
            raise RuntimeError("Nav2가 /navigate_to_pose 액션 서버를 올리지 못했습니다: " + detail[-1800:])
        STATE.update(robot_id, stage="prepared", message=f"센서·지도·Nav2 주행 액션을 확인했습니다 ({map_name}).")
    except Exception as exc:  # surfaced in the dashboard for operator action
        STATE.update(robot_id, stage="fault", message="준비 과정에서 오류가 발생했습니다.", error=str(exc))
    finally:
        with STATE.lock:
            if release_busy and STATE.busy_robot == robot_id:
                STATE.busy_robot = None


def _recover_navigation(robot_id: str) -> None:
    map_name = STATE.robots[robot_id]["map"]
    try:
        STATE.update(robot_id, stage="nav_starting", message="Nav2를 재시작하고 주행 액션을 확인합니다.", error=None)
        stopped, output = _stop_existing_navigation(robot_id)
        if not stopped or "NAV_PROCESSES_CLEARED" not in output:
            raise RuntimeError("기존 Nav2 실행을 종료하지 못했습니다: " + output.strip())
        map_path = _remote_map_path(robot_id, map_name)
        if map_path is None:
            raise RuntimeError(f"로봇에서 {map_name} 지도 경로를 확인하지 못했습니다.")
        ok, output = _start_navigation_with_safety(robot_id, map_path)
        if not ok:
            raise RuntimeError("Nav2 시작 실패: " + output.strip())
        map_ready, missing = _topic_ready(robot_id, ("map",), timeout=25.0)
        if not map_ready:
            raise RuntimeError("Nav2가 /map을 발행하지 않습니다: " + ", ".join(missing))
        action_ready, action_detail = _nav_action_ready(robot_id)
        if not action_ready:
            _, diagnostics = _ssh_run(robot_id, _nav_diagnostics_command(robot_id))
            raise RuntimeError("Nav2 주행 액션이 준비되지 않았습니다: " + (action_detail + "\n" + diagnostics)[-1800:])
        STATE.update(robot_id, stage="prepared", message="Nav2 주행 액션이 준비됐습니다. 경로를 다시 시작할 수 있습니다.", error=None)
    except Exception as exc:
        STATE.update(robot_id, stage="fault", message="Nav2 복구에 실패했습니다.", error=str(exc))
    finally:
        with STATE.lock:
            if STATE.busy_robot == robot_id:
                STATE.busy_robot = None


def _health_loop() -> None:
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


class Handler(BaseHTTPRequestHandler):
    server_version = "PinkyFleet/0.1"

    def log_message(self, fmt: str, *args: Any) -> None:
        # Avoid logging API payloads, which can include the runtime SSH password.
        print(f"{self.log_date_time_string()} {self.address_string()} {fmt % args}")

    def _json(self, status: int, body: dict[str, Any]) -> None:
        payload = json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _body(self) -> dict[str, Any]:
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
        if path == "/api/status":
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
                                   "ir_profile": profile_view, "parking": PARKING[robot_id], "route": STATE.routes[robot_id]})
                self._json(200, {"robots": robots, "busy_robot": STATE.busy_robot, "monitor_error": STATE.monitor_error})
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
            writer.writerow(["seq", "name", "map", "frame", "x_m", "y_m", "yaw_rad", "anchor"])
            for seq, p in enumerate(points, 1):
                writer.writerow([seq, p["label"], p["map"], p["frame_id"], p["x_m"], p["y_m"], p["yaw_rad"], p["anchor"]])
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
        path = unquote(self.path.split("?", 1)[0])
        try:
            data = self._body()
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
            self._json(400, {"error": "invalid JSON"})
            return
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
                allowed = {'docking','await_confirmation','recovery_hold'} if action == 'complete' else {'stopped'}
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
                          '도킹완료 확인 신호를 전달했습니다. 흰색 유지가 확인되면 정지합니다.' if action == 'complete'
                          else '물건 수령 완료를 확인했습니다. 시나리오를 재개합니다.'}
            except ValueError as exc:
                code, result = 409, {'error': str(exc)}
            except OSError as exc:
                code, result = 502, {'error': str(exc)}
            finally:
                DOCK_COMMAND_LOCK.release()
            self._json(code, result)
            return
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
        operator_match = re.fullmatch(r'/api/robots/(10|11|12)/operator-confirm', path)
        if operator_match:
            robot_id=operator_match.group(1)
            phases={'dock_complete':'docking_manual','received':'docked_manual'}
            action=data.get('action')
            with STATE.lock:
                route=STATE.routes[robot_id] or {}
                token=route.get('mission_token')
                if action not in phases or route.get('status')!='running' or route.get('phase')!=phases[action] or not token or data.get('token')!=token:
                    self._json(409,{'error':'현재 임무의 확인 단계와 신호가 일치하지 않습니다.'})
                    return
            control=TELEMETRY.snapshot().get(robot_id,{}).get('line_control',{}) if TELEMETRY else {}
            if not control.get('fresh'):
                self._json(409,{'error':'안전 감시기 수신을 확인하세요.'});return
            payload=base64.b64encode(json.dumps({'token':token,'action':action}).encode()).decode()
            program="import base64,json,time,os;from pathlib import Path;d=json.loads(base64.b64decode('"+payload+"'));d['monotonic']=time.monotonic();p=Path('/home/pinky/.cache/pinky-fleet/operator_confirm.json');t=p.with_suffix('.tmp');t.write_text(json.dumps(d));os.replace(t,p)"
            ok,output=_ssh_run(robot_id,'python3 -c '+shlex.quote(program))
            self._json(202 if ok else 502,{'ok':ok,'message':'현재 단계 확인 신호를 보냈습니다.' if ok else output});return
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
        route_match = re.fullmatch(r"/api/robots/(10|11|12)/route", path)
        if route_match:
            robot_id = route_match.group(1)
            labels = data.get("labels")
            if not isinstance(labels, list) or not 1 <= len(labels) <= 50 or any(not isinstance(label, str) for label in labels):
                self._json(400, {"error": "웨이포인트 이름 목록을 확인하세요."})
                return
            with STATE.lock:
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                if STATE.busy_robot is not None:
                    self._json(409, {"error": f"Pinky {STATE.busy_robot} 작업이 끝날 때까지 기다려 주세요."})
                    return
                if (STATE.routes[robot_id] or {}).get('status') in {'starting','running','cancel_requested'}:
                    self._json(409, {'error': '이 로봇은 이미 경로 주행 중입니다.'})
                    return
                telemetry = TELEMETRY.snapshot() if TELEMETRY is not None else {}
                robot_topics = telemetry.get(robot_id, {})
                required_live = ("map", "tf_pose", "scan", "odom", "ultrasonic", "ir", "line_control")
                live_ready = all(robot_topics.get(key, {}).get("fresh") for key in required_live)
                guard = robot_topics.get('line_control', {})
                guard_value = guard.get('value') or {}
                if (not guard.get('fresh') or guard_value.get('recovery_policy') != 'parking_mission_v2'
                        or guard_value.get('line_algorithm') != 'three_surface_pd_v2'):
                    self._json(409, {'error': '새 주행 감시기를 적용하려면 Nav2 복구 또는 주행준비를 실행하세요.'})
                    return
                calibration_error = validate_profile(IR_PROFILES.get(robot_id))
                if IR_PROFILES.get(robot_id).get('enabled') and calibration_error:
                    self._json(409, {'error': '차선 재획득·도킹 IR 판정을 위한 보정이 필요합니다: '+calibration_error})
                    return
                if STATE.robots[robot_id]["stage"] != "prepared":
                    self._json(409, {"error": "업데이트된 Nav2 안전 설정과 초음파 감시기를 적용하려면 먼저 주행준비를 완료하세요."})
                    return
                if not live_ready:
                    self._json(409, {"error": "지도·현재 map 위치·LiDAR·오도메트리·초음파가 모두 수신될 때까지 기다려 주세요."})
                    return
                active_peers = [rid for rid,r in STATE.routes.items() if rid != robot_id and
                                (r or {}).get('status') in {'starting','running','cancel_requested'}]
                if any(STATE.robots[rid]['map'] != STATE.robots[robot_id]['map'] for rid in active_peers):
                    self._json(409, {'error': '동시 주행 로봇은 같은 공용 지도를 선택해야 합니다.'})
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
                    # Last course point gates the external completion signal.
                    anchors=[p for p in goals if p.get('anchor')]
                    anchors[-2]['course_end']=True
                    anchors[1]['arrival_yaw_rad']=0.
                    for point in goals:
                        point['use_ir']=bool(IR_PROFILES.get(robot_id).get('enabled'))
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
        route_cancel_match = re.fullmatch(r"/api/robots/(10|11|12)/route/cancel", path)
        if route_cancel_match:
            robot_id = route_cancel_match.group(1)
            with STATE.lock:
                if STATE.password is None:
                    self._json(401, {"error": "SSH 비밀번호를 먼저 입력하세요."})
                    return
                route = STATE.routes[robot_id]
                if route is None:
                    route={'status':'cancel_requested','labels':[],'message':'정지 유지 요청 중','detail':''}
                    STATE.routes[robot_id]=route
                else:
                    route.update(status='cancel_requested',message='정지 유지와 Nav2 취소를 요청합니다.')
            WORKERS.submit(_cancel_waypoint_route, robot_id)
            self._json(202, {"ok": True, "message": "경로 취소를 요청했습니다."})
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
        match = re.fullmatch(r"/api/robots/(10|11|12)/ready", path)
        if match:
            robot_id = match.group(1)
            map_name = data.get("map", "my_map.yaml")
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
                if current in {"base_starting", "sensors_starting", "nav_starting", "prepared"}:
                    self._json(409, {"error": "이미 준비 중이거나 준비된 로봇입니다."})
                    return
                STATE.busy_robot = robot_id
                STATE.robots[robot_id].update(stage="base_starting", map=map_name, message="기본 bringup을 준비합니다.", error=None)
            # Worker owns the busy lock until its startup sequence finishes.
            WORKERS.submit(_prepare_robot, robot_id, map_name)
            self._json(202, {"ok": True, "robot": robot_id})
            return
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


def main() -> None:
    global TELEMETRY
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN)
    try:
        import rclpy
        rclpy.init(args=None)
        TELEMETRY = Telemetry()
    except Exception as exc:
        STATE.monitor_error = f"ROS 모니터를 시작하지 못했습니다: {exc}"
        print(STATE.monitor_error)
    threading.Thread(target=_health_loop, daemon=True, name="robot-health").start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    # rclpy installs signal handlers that stop ROS without stopping HTTP.
    # Shut down both cleanly so restarting does not strand DDS participants.
    def stop_server(*_):
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
