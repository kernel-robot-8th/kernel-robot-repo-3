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

from scripts.route_geometry import DOCK_APPROACH_M, build_dense_route, build_mission_route, clean_dock_guide, corrected_dock
from scripts.line_following import default_profile, capture_profile, normalize, validate_profile
from scripts.ir_profiles import IRProfiles
from scripts.fleet_ops import operational_status, indicator
from scripts.traffic import TrafficCoordinator
from scripts.runtime_bundle import install_command
from scripts.motion_gate import geometry
from scripts.control_journal import ControlJournal
from scripts.scan_localizer import locate as locate_in_map, refine as refine_in_map
from scripts.fusion_grid import validated as validate_fusion_settings
from scripts.ir_calibration import IRCalibrationStore

ROOT = Path(__file__).resolve().parent
PARKING = json.loads((ROOT / "config" / "parking.json").read_text())
# Driving is Nav2's (decided 2026-11-14): IR only reports, the ultrasonic joins the LiDAR in Nav2's height-cell
# costmap, and the ultrasonic safety node holds forward motion near an obstacle instead of running its own detour.
NAVIGATION_PATH = ROOT / "config" / "navigation.json"
NAVIGATION = json.loads(NAVIGATION_PATH.read_text())


def _reload_navigation() -> None:
    """Read navigation.json again, so a changed setting applies from the next 주행준비 without a restart."""
    global NAVIGATION
    try:
        NAVIGATION = json.loads(NAVIGATION_PATH.read_text())
    except (OSError, ValueError) as exc:
        JOURNAL.record('navigation_config_error', state=str(exc))  # keep the last good settings
WEB = ROOT / "web"
ROS_DOMAIN = 0
HOST = "127.0.0.1"
PORT = 8765
FRESH_SECONDS = 4.0
TRAIL_POINTS = 1500   # about 30 m of trail at TRAIL_STEP_M
TRAIL_STEP_M = 0.02
START_TIMEOUT = 20.0
# Automatic localization after 주행준비: AMCL is given the robot's own parking pose, then its reported map pose
# must come back this close to it. The pose is set exactly, so these only absorb AMCL's first correction.
LOCALIZE_RADIUS_M = 0.15
LOCALIZE_YAW_RAD = 0.35
LOCALIZE_TIMEOUT_S = 12.0
# Odometry speeds below these count as standing still before the parking pose is given.
STILL_LINEAR_MPS = 0.02
STILL_ANGULAR_RPS = 0.05
# Two scans in a row must place the robot within these of each other before AMCL is given the pose.
SCAN_AGREE_M = 0.05
SCAN_AGREE_YAW_RAD = math.radians(5.0)
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
                        # A dashboard restart leaves the robots' Nav2 and AMCL running (2026-11-16): a Pinky that was
                        # prepared is taken as prepared again, with its map and localization times. If its Nav2 is
                        # gone, the live checks (map pose, LiDAR, guard) still refuse a mission.
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
        WAYPOINTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=WAYPOINTS_PATH.parent, delete=False) as stream:
            json.dump(self.waypoints, stream, ensure_ascii=False, indent=2)
            temp_path = Path(stream.name)
        os.replace(temp_path, WAYPOINTS_PATH)

    def update(self, robot_id: str, **values: Any) -> None:
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
IR_PROFILES = IRProfiles(ROOT / "data" / "ir_profiles.json", list(ROBOTS))
IR_CALIBRATION = IRCalibrationStore(ROOT / 'data' / 'ir_calibration')
WORKERS = ThreadPoolExecutor(max_workers=8, thread_name_prefix="pinky-fleet")
DOCK_COMMAND_LOCK = threading.Lock()
_zones={'parking':{'bounds':[min(p['x_m'] for p in PARKING.values())-.2,max(p['x_m'] for p in PARKING.values())+.2,min(p['y_m'] for p in PARKING.values())-.2,max(p['y_m'] for p in PARKING.values())+.2]}}
def _any_waypoint(label):
    return next((p for rid in ROBOTS for p in STATE.waypoints[rid] if p.get('label')==label),None)
# The rules, kept simple (2026-11-17, "only look at the narrow place a few seconds before going in, else just drive"):
#   passage9   the 50 cm gap beside the middle wall: asked for within request_radius_m of it (about 3-4 s of driving);
#              a Pinky waits in front of it only while another is in it
#   dock5      station 5: asked for at the queue spot (the stretch to '5 정면'), in the order asked
#   dock_queue the queue spot itself: one Pinky stands there; the next stops just short of it, outside the passage
#   parking    one Pinky at a time in or out of the parking area
# Nothing is asked for ahead of time from the parking spot, and junction 7 (inside the passage zone, a leftover of
# the line course) is no longer a zone of its own when the passage zone is set.
_traffic_settings=json.loads((ROOT/'config'/'navigation.json').read_text())
_passage=_traffic_settings.get('passage_zone')
_has_passage=isinstance(_passage,dict) and all(isinstance(_passage.get(k),(int,float)) for k in ('x_m','y_m'))
for _label,_name,_radius in [('7','junction7',.25),('5','dock5',.40)]:
    if _name=='junction7' and _has_passage:continue
    _anchor=_any_waypoint(_label)
    if _anchor:_zones[_name]=dict(x_m=_anchor['x_m'],y_m=_anchor['y_m'],request_radius_m=_radius)
# 2026-11-17: one Pinky at a time through the passage beside the middle wall, one at the queue spot in front of
# station 5, and station 5 in the order asked for (navigation.json passage_zone, dock_queue).
if 'dock5' in _zones:
    _zones['dock5'].update(request_labels=['5 정면','5'],release_radius_m=float(_traffic_settings.get('dock_release_radius_m',1.0)))
if _has_passage:
    _zones['passage9']=dict(x_m=float(_passage['x_m']),y_m=float(_passage['y_m']),request_radius_m=float(_passage.get('request_radius_m',.45)))
_queue_label=(_traffic_settings.get('dock_queue') or {}).get('waypoint')
_queue=_any_waypoint(_queue_label) if _queue_label else None
if _queue:
    _zones['dock_queue']=dict(x_m=_queue['x_m'],y_m=_queue['y_m'],request_radius_m=.22)
# The waiting spot below the passage (navigation.json wait_low): the next Pinky drives up to it at once and goes on
# when the queue spot and the passage are free, so all three are on the way instead of standing in the parking area.
_low_label=(_traffic_settings.get('wait_low') or {}).get('waypoint')
_low=_any_waypoint(_low_label) if _low_label else None
if _low:
    _zones['wait_low']=dict(x_m=_low['x_m'],y_m=_low['y_m'],request_radius_m=.30,request_labels=[_low_label])
TRAFFIC=TrafficCoordinator(ROOT/'data/traffic_reservations.json',_zones)
TRAFFIC_BOOT=uuid.uuid4().hex
TRAFFIC_SEQUENCE=0
SHUTTING_DOWN=threading.Event()
JOURNAL=ControlJournal(ROOT/'logs/telemetry.jsonl')


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


def _load_local_map_grid(yaml_path: Path = LOCAL_MAP_YAML) -> dict[str, Any] | None:
    """Build an OccupancyGrid-shaped fallback from map-proto's YAML and PGM."""
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


def keepout_view():
    """Render the on-disk Nav2 mask in its own metric origin/resolution."""
    path = ROOT / 'config/keepout/keepout_mask.yaml'
    grid = _load_local_map_grid(path)
    if grid is None:
        return dict(available=False, error='keepout YAML/PGM 읽기 실패')
    return dict(grid, available=True, map='map-proto.yaml', frame_id='map',
                source='keepout_mask.yaml', display_only=True)



class Telemetry:
    """ROS 2 subscriptions in PC domain 0; bridges remap each robot into a namespace."""

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
        # Where each Pinky has been on the map (from the map-frame pose), one point per 2 cm, for drawing its trail.
        self.trails = {rid: deque(maxlen=TRAIL_POINTS) for rid in ROBOTS}
        self.base_frames = {rid: "base_footprint" for rid in ROBOTS}
        # The latest full LaserScan per robot, for finding the robot on the map (scan_localizer).
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
            "plan": "plan",  # Nav2's global plan, for drawing where the Pinky means to go
        }
        self.traffic_publishers = {}
        self.initial_pose_publishers = {}
        self.indicator_publishers = {}
        self.dock_guide_publishers = {}
        self.domain_nodes = {}
        self.domain_readers = []
        for rid, robot in ROBOTS.items():
            context = rclpy.context.Context()
            rclpy.init(args=[], context=context, domain_id=robot['domain'])
            node = Node('pinky_fleet_reader_'+rid,context=context)
            executor = MultiThreadedExecutor(num_threads=2,context=context)
            executor.add_node(node)
            self.domain_readers.append((context,node,executor))
            self.traffic_publishers[rid]=node.create_publisher(String,'/fleet/lease',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            # A reliable publisher matches AMCL's /initialpose subscription whether it is reliable or best effort.
            self.initial_pose_publishers[rid]=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
            self.domain_nodes[rid]=node
            self.indicator_publishers[rid]=node.create_publisher(String,'/fleet/indicator',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            # The ceiling camera's offset of this Pinky from its loading spot, relayed while it drives in (POST dock-guide).
            self.dock_guide_publishers[rid]=node.create_publisher(String,'/fleet/dock_guide',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
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
                elif key == "plan":
                    subscription_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                  durability=DurabilityPolicy.VOLATILE)
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
                # OccupancyGrid.data is row-major from the map's lower-left
                # origin. Keeping that exact payload lets the browser render
                # and place the robot in the same Nav2 map frame.
                "cells": [int(cell) for cell in msg.data],
            }
        elif key == "plan":
            poses = msg.poses
            step = max(1, len(poses) // 80)  # enough points to draw it, not the planner's every few millimetres
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
                    "xy": _finite((covariance[1] + covariance[6]) / 2.0),
                    "yy": _finite(covariance[7]),
                },
            }
        else:
            value = None
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
        """Which feed ('amcl_pose' or 'tf_pose') showed a map pose near (x, y, yaw) received after `since`, or None.

        AMCL answers a new /initialpose with /amcl_pose on its next scan and re-sends map->odom; either one,
        received after the pose was sent, shows AMCL took it. It does not show the robot really stands there."""
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
        """The latest full scan received after `after` (PC clock), waiting up to `timeout` seconds."""
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
        """`frame`'s (x, y, yaw) in the robot's base frame from its TF tree (static mounts), or None if unknown."""
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
        """Send AMCL a map pose from this PC (no SSH). False when no AMCL subscribes to /initialpose."""
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
        # The scan match is good to a few centimetres and degrees: a tight cloud around it.
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
            # Static map is published once with transient-local durability.
            return received is not None
        return received is not None and (time.time() - received) <= FRESH_SECONDS

    def forget(self, robot_id: str, key: str) -> None:
        """Drop one topic's last value (a stopped Pinky's Nav2 plan must not stay on the map as where it goes), or with
        'trail' the line of where it has driven."""
        with self.lock:
            if key == "trail":
                self.trails.get(robot_id, deque()).clear()
                return
            entry = self.data.get(robot_id, {}).get(key)
            if entry is not None:
                entry["value"], entry["received_at"] = None, None

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
                trail = list(self.trails.get(robot_id, ()))
                topics["trail"] = {"value": {"points": [list(p) for p in trail]}, "age_s": None, "fresh": bool(trail)}
                result[robot_id] = topics
            return result


TELEMETRY: Telemetry | None = None


def _topic_ready(robot_id: str, keys: tuple[str, ...], timeout: float = START_TIMEOUT) -> tuple[bool, list[str]]:
    if TELEMETRY is None:
        return False, list(keys)
    return TELEMETRY.wait_for(robot_id, keys, timeout)


def _ssh_run(robot_id: str, remote_command: str, timeout_seconds: float = 35.0, keep_chars: int = 1200,
             send_password: bool = False, redact: bool = True) -> tuple[bool, str]:
    """Run `remote_command` on the robot. With `send_password`, the SSH password is written to the command's standard
    input (for `sudo -S`), so it never appears in an argument list on either machine."""
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
        if redact:  # off only for output that cannot hold the password (a clock reading): a one-digit password would
            combined = combined.replace(password, "[redacted]")  # otherwise blank every such digit in it
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
    # The log is appended to; the mark tells this launch's lines from the ones before (scripts/nav_readiness.py).
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


MAP_INSTALL_ERRORS: dict[str, str] = {}  # robot id -> why the last map copy to it failed


def _remote_map_path(robot_id: str, map_name: str) -> str | None:
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


NAV_LOG_PATH = "/home/pinky/.cache/pinky-fleet/navigation.log"
NAV2_BRINGUP_RETRIES = 1


def _nav_action_ready(robot_id: str, deadline_s: float = 150.0) -> tuple[bool, str]:
    # An action can be discoverable while its lifecycle node is inactive.
    # Read lifecycle services; never submit a motion goal as a readiness probe.
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
    """(ready, detail): wait for Nav2's action; when the lifecycle manager gave the bringup up (NAV_BRINGUP_ABORTED),
    stop Nav2 and start it again, up to NAV2_BRINGUP_RETRIES times. Any other failure is returned as it is."""
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


NAV2_FILES = ("nav2_params.yaml", "bringup_launch.xml", "navigation_launch.xml", "localization_launch.xml",
              "keepout_mask.pgm", "keepout_mask.yaml")  # a changed keepout mask also needs a fresh Nav2
NAV2_KEEP_PROBE_S = 20.0


def _nav2_files_hash_command() -> str:
    files = " ".join(shlex.quote(f"/home/pinky/.cache/pinky-fleet/nav2/{name}") for name in NAV2_FILES)
    return f"cat {files} 2>/dev/null | sha256sum | cut -c1-16"


def _running_nav2_keepable(robot_id: str, map_path: str) -> tuple[bool, str]:
    """Whether the Nav2 this dashboard started on the robot can stay: it runs with this map and answers as active.

    Bringing Nav2 up on a Pinky is the slow, flaky part of 주행준비 (2026-11-16), so a healthy one is kept, the way
    ROSY keeps Nav2 as a long-running service on Pinky Pro. Whether its settings changed is checked after the
    runtime bundle is reinstalled (_start_navigation_with_safety)."""
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
    """Stop tracked and orphaned Nav2 processes before launching one clean stack."""
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


def _nav2_runtime_install_command(robot_id: str, map_name: str) -> str:
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
    """Stop the robot-side runtime script `name`.py started under its pid file, only if that pid is still it."""
    pidfile = f"/home/pinky/.cache/pinky-fleet/{name}.pid"
    return (f"pidfile={pidfile}; if [ -s \"$pidfile\" ]; then pid=$(cat \"$pidfile\"); "
            f"case \"$(tr '\\0' ' ' < /proc/$pid/cmdline 2>/dev/null)\" in *{name}.py*) kill \"$pid\";; esac; "
            f"rm -f \"$pidfile\"; fi; echo SCRIPT_STOPPED")


def _start_navigation_with_safety(robot_id: str, map_path: str, map_name: str,
                                  keep_nav2: bool = False) -> tuple[bool, str]:
    """Install the runtime bundle, start the safety and lamp nodes, then launch Nav2 — or, with `keep_nav2`, keep
    the running one when the reinstall left its settings unchanged. Returns (ok, output); 'NAV2_KEPT' when kept."""
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

    # The status lamp is a convenience: if the Pinky has no lamp or LED package, preparation goes on without it.
    # The lamp script was just reinstalled with the runtime bundle: stop the old one so the new code runs.
    _ssh_run(robot_id, _stop_script_command("status_lamp"))
    _ssh_run(robot_id, _stop_script_command("status_display"))
    # Old ones, whether fusion stays on or not: ultrasonic_cloud.py was the earlier fusion, and a running fusion_grid
    # must restart to run the code and settings just installed.
    _ssh_run(robot_id, _stop_script_command("ultrasonic_cloud"))
    _ssh_run(robot_id, _stop_script_command("fusion_grid"))
    if NAVIGATION.get("ultrasonic_fusion"):
        # Nav2's costmap is configured to read the fused low layer, so preparation needs the node running.
        try:
            validate_fusion_settings(NAVIGATION.get("fusion_grid"))
        except ValueError as exc:
            return False, f"config/navigation.json의 fusion_grid 설정이 잘못됐습니다: {exc}"
        ok, output = _ssh_run(robot_id, _remote_start_command(robot_id, "fusion_grid",
                                  "python3 /home/pinky/.cache/pinky-fleet/nav2/fusion_grid.py"))
        if not ok or not any(token in output for token in ("STARTED", "ALREADY_RUNNING")):
            return False, output or "라이다·초음파 퓨전 노드를 시작하지 못했습니다."
    for stage, lamp_command in (("lamp", "ros2 run pinky_lamp_control main_node"), ("led", "ros2 run pinky_led led_server"),
                                ("status_lamp", "python3 /home/pinky/.cache/pinky-fleet/nav2/status_lamp.py")):
        # status_display.py is not started: this Pinky's pinkylib has no LCD (it only cost CPU, 2026-11-16).
        _ssh_run(robot_id, _remote_start_command(robot_id, stage, lamp_command))

    if keep_nav2:
        after = _ssh_run(robot_id, _nav2_files_hash_command())[1].strip()
        if before and before == after:
            # Same settings: keep it. Its pose is still checked again by LiDAR localization at the end of 주행준비.
            STATE.update(robot_id, nav_started_at=time.time())
            return True, "NAV2_KEPT"
        stopped, stop_output = _stop_existing_navigation(robot_id)  # settings changed: a clean restart
        if not stopped or "NAV_PROCESSES_CLEARED" not in stop_output:
            return False, "Nav2 설정이 바뀌었지만 이전 Nav2를 정리하지 못했습니다: " + stop_output.strip()
    # From here AMCL restarts: whatever map pose it reports is not trusted until localization sets one.
    STATE.update(robot_id, nav_started_at=time.time())
    params_path = "/home/pinky/.cache/pinky-fleet/nav2/nav2_params.yaml"
    launch_path = "/home/pinky/.cache/pinky-fleet/nav2/bringup_launch.xml"
    command = (
        f"ros2 launch {shlex.quote(launch_path)} map:={shlex.quote(map_path)} "
        # One process for all of Nav2 (Pinky's own default): ten separate ones starting at once pushed a Pinky's
        # load to 15 on 4 cores and the lifecycle bring-up timed out (2026-11-16). The installer reroutes the composed
        # nodes' cmd_vel through the guard chain just as the separate ones.
        f"use_composition:=True params_file:={shlex.quote(params_path)}"
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


def _standing_still(robot_id: str) -> str | None:
    """Why the robot cannot be given a pose now (odometry missing or moving), or None when it stands still."""
    odom = TELEMETRY.snapshot().get(robot_id, {}).get("odom", {}) if TELEMETRY else {}
    speed = odom.get("value") or {}
    if not odom.get("fresh"):
        return "오도메트리가 들어오지 않아 핑키가 멈춰 있는지 모릅니다."
    if abs(speed.get("linear_mps") or 0.0) > STILL_LINEAR_MPS or abs(speed.get("angular_rps") or 0.0) > STILL_ANGULAR_RPS:
        return "핑키가 움직이는 중이라 위치를 넣지 않았습니다."
    return None


def _lidar_match(robot_id: str, after: float):
    """(match, why not): scan_localizer's answer for the next scan received after `after` (PC clock)."""
    grid = TELEMETRY.snapshot().get(robot_id, {}).get("map", {}).get("value") if TELEMETRY else None
    if not grid or not grid.get("cells"):
        return None, "지도를 아직 받지 못했습니다."
    scan = TELEMETRY.scan_after(robot_id, after, 3.0)
    if scan is None:
        return None, "라이다 스캔이 들어오지 않습니다."
    laser = TELEMETRY.frame_in_base(robot_id, scan["frame_id"])
    if laser is None:
        return None, f"라이다 장착 위치(TF {scan['frame_id']} → {TELEMETRY.base_frames[robot_id]})를 모릅니다."
    # Where a standing Pinky is expected: any parking slot, station 5 (loading) and the receiving zones.
    # Pinkys are parked facing either way, so a slot counts by position; its mirror image lies near no known place.
    places = [(spot["x_m"], spot["y_m"], None) for spot in PARKING.values()]
    dock = TRAFFIC.zones.get("dock5") or {}
    if not dock:  # no waypoints on this PC: the robots report station 5 in their traffic_config
        for topics in TELEMETRY.snapshot().values():
            dock = (((topics.get("line_control") or {}).get("value") or {}).get("traffic_config") or {}).get("dock5") or {}
            if dock:
                break
    if isinstance(dock.get("x_m"), (int, float)) and isinstance(dock.get("y_m"), (int, float)):
        places.append((float(dock["x_m"]), float(dock["y_m"]), None))
    # Receiving zones recorded as waypoints ('수령 …') by any Pinky on this map are places a Pinky waits too.
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
    return (math.hypot(a[0] - b[0], a[1] - b[1]) <= SCAN_AGREE_M
            and abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))) <= SCAN_AGREE_YAW_RAD)


CHOICE_MATCH_M = 0.10            # a picked candidate is found again in a fresh scan within these
CHOICE_MATCH_YAW_RAD = math.radians(10.0)


def _closest_candidate(match, pose: tuple[float, float, float]):
    for candidate in match.candidates:
        _, x, y, yaw = candidate
        if (math.hypot(x - pose[0], y - pose[1]) <= CHOICE_MATCH_M
                and abs(math.atan2(math.sin(yaw - pose[2]), math.cos(yaw - pose[2]))) <= CHOICE_MATCH_YAW_RAD):
            return candidate
    return None


def _last_confirmed_pose(robot_id: str) -> tuple[float, float, float] | None:
    """The map pose the Pinky had while it was localized (before Nav2 or the base is restarted), or None."""
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
    """Find the robot on its map from its own LiDAR (scripts/scan_localizer.py) and give that pose to AMCL.

    The cell is a rectangle, so a place and its half-turn often fit the scan equally well. Without `choice` the
    pose is used only when one place is clearly best, or when exactly one of the equally good places is the
    robot's own parking spot; otherwise the candidates are stored for the operator. `choice` is the pose
    (x, y, yaw) of the candidate the operator picked; it must still be a candidate of two fresh scans. Two scans in a row must agree either way. The pose goes to AMCL from this PC
    over DDS (no SSH password), and AMCL must then report a map pose there. Returns (ok, what happened)."""
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
    # Where it stood before the restart (2026-11-16): above and below the middle wall look alike to the LiDAR, so
    # parking P0 and station 5 tie. A Pinky nobody carried off is still where it was; take that candidate.
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


OPERATOR_REFINE_MIN_SCORE = 0.5


def _set_operator_pose(robot_id: str, x: float, y: float, yaw: float, refine: bool = True) -> None:
    """The operator put the Pinky's pose on the map (as RViz's 2D Pose Estimate). The LiDAR only polishes it within
    30 cm and 30 degrees, so it never jumps to the mirror place of the symmetric cell; then AMCL is given it."""
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
            # The pose comes from something surer than the scan here (the ceiling camera at station 5, where arms
            # and shelves that are not in the map pull the LiDAR fit off by 17 cm, 2026-11-17).
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
    """IR line following steers only when navigation.json allows it and the robot's IR profile is enabled."""
    return bool(NAVIGATION.get("ir_steering")) and bool(IR_PROFILES.get(robot_id).get("enabled"))


def _localize_at_parking(robot_id: str) -> tuple[bool, str]:
    """Give AMCL the robot's own parking pose and wait until AMCL reports a map pose there.

    The robot must stand in its own parking spot: nothing here can tell where it really is, so a robot left
    elsewhere gets a wrong pose. A moving robot is refused. Returns (ok, why not or what confirmed it)."""
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
    try:
        topics=TELEMETRY.snapshot().get(robot_id,{}) if TELEMETRY else {}
        if any(not topics.get(key,{}).get('fresh') for key in ('scan','odom','map','line_control')):
            # 주행준비 itself ends by localizing at the parking spot.
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
    """The Pinky still runs what an earlier preparation installed: its traffic zones are not the dashboard's (a
    waypoint moved since) or its behaviour layer is older than this dashboard's. Such a Pinky is "prepared" and yet
    refused every start with "주행준비를 다시 실행하세요"; [주행준비하고 위치 잡기] skipped it as 이미 준비됨 (2026-11-19)."""
    guard = (robot_topics.get('line_control') or {}).get('value') or {}
    return bool(guard) and (guard.get('traffic_config') != TRAFFIC.zones or 'squaring' not in (guard.get('dock_entry') or {}))


def _route_start_problem(robot_id: str) -> tuple[int, dict[str, str]] | None:
    """Why a route or mission may not start now, as (HTTP status, answer), or None. Call with STATE.lock held."""
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
    """Stops the Pinky only passes: the passage points (10 cm), unless it is the last stop the queue spot (6 cm), and
    the square-up point in front of station 5 (5 cm; it turns there itself)."""
    queue_label = (NAVIGATION.get("dock_queue") or {}).get("waypoint")
    for index, point in enumerate(goals):
        if not point.get("anchor") or index == len(goals) - 1:
            continue
        if point["label"] in {"통로 아래", "통로 위"}:
            point["pass_radius_m"] = 0.10
        elif queue_label and point["label"] == queue_label:
            point["pass_radius_m"] = 0.06
        elif point["label"] == "5 정면":
            # Close enough (2026-11-19): Nav2 hunted for minutes to meet its 3 cm and the heading there, next to
            # the station. The Pinky turns to the dock's heading itself (DockEntry). 0 in navigation.json: as before.
            radius = float(NAVIGATION.get("dock_square_radius_m", 0.05))
            if radius > 0:
                point["pass_radius_m"] = min(radius, 0.08)


def _dock_correction(robot_id: str, offset: object) -> dict[str, Any] | None:
    """For a redock: the square-up point and loading spot corrected by the ceiling camera's offset, from where the
    Pinky believes it stands right now (it is standing still at station 5). None when the offset, the pose or the
    waypoint is unusable; the Pinky then comes in again at the saved waypoint as before."""
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
    """The waypoint `label` on this robot's map: its own first, else one recorded by another Pinky on the same map
    (a receiving zone is recorded once, by whichever Pinky stood there)."""
    map_name = STATE.robots[robot_id]["map"]
    for rid in [robot_id] + [other for other in ROBOTS if other != robot_id]:
        for item in STATE.waypoints[rid]:
            if item.get("label") == label and item.get("map") == map_name and item.get("frame_id") == "map":
                return item
    return None


def _run_waypoint_route(robot_id: str, goals: list[dict[str, Any]]) -> None:
    """Stream per-waypoint progress from the robot's Nav2 action client."""
    temp_dir = None
    # A new run draws its own line: the trail and plan of the run before (stopped or finished) are wiped first.
    if TELEMETRY is not None:
        TELEMETRY.forget(robot_id, "trail")
        TELEMETRY.forget(robot_id, "plan")
    with STATE.lock:
        route_ref=STATE.routes[robot_id]
        # A queued start can be cancelled before its worker gets CPU time.
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
                # Stopped is clean (2026-11-16): the route and Nav2's goal are gone and the plan is wiped from the
                # map, while Nav2, AMCL's pose and the guard stay up, so the next mission starts from here without
                # another 주행준비 (navigate_route.py lifts the stop latch when it starts). Shared-zone reservations
                # free themselves once the lease stops and the Pinky is outside; one it stopped inside stays held.
                # With no new command for AUTO_RETURN_S it drives back to its own parking spot by itself; a second
                # stop (hold) or 기존 경로 날리기 (clear) cancels that, and a cleared route is dropped altogether.
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


def _auto_return_seconds() -> float:
    """Seconds a stopped Pinky waits for a command before it drives home; 0 = never (navigation.json)."""
    value = NAVIGATION.get("auto_return_after_stop_s", 60)
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0 else 0.0


def _return_home_goals(robot_id: str, via: list[str]) -> tuple[list[dict[str, Any]] | None, tuple[int, dict] | None]:
    """(goals, None) for a drive from where the Pinky stands, through `via`, to its own parking spot, or
    (None, (status, answer)) when it may not go. Call with STATE.lock held."""
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
        # Standing at station 5 between the arms: leave as a mission does, straight back first (2026-11-17), never by
        # turning on the spot there.
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
    """Start the drive home; (HTTP status, answer). Takes STATE.lock itself."""
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
    """Once a second: a stopped Pinky whose wait ran out drives home. If it may not (no pose, Nav2 down...),
    the reason is shown and it stays where it is."""
    while not SHUTTING_DOWN.wait(1.0):
        now = time.time()
        due = []
        with STATE.lock:
            for rid, route in STATE.routes.items():
                if route and route.get("status") == "cancelled" and route.get("auto_return_at") and route["auto_return_at"] <= now:
                    route["auto_return_at"] = None
                    due.append(rid)
        for rid in due:
            # Someone handled it after the stop (carried it, or set its pose): it is not going anywhere by itself.
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
    """기존 경로 날리기: a running route is stopped first and dropped once the stop holds; anything else (stopped,
    failed, finished, or a stop that never reached a Pinky that is off) is dropped now. Takes STATE.lock itself."""
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
    # A Pinky that went off inside a shared zone keeps holding it for ever (2026-11-17: P2 died in the passage and
    # P1 waited behind it). It is never released by itself, since the dead Pinky may still stand there; clearing its
    # route is the operator saying it is gone. A live Pinky inside a zone takes it again at once (physical occupancy).
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
    """초기 세팅: hold the Pinky stopped, drop its route and lines, then 주행준비 from the start even if prepared."""
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


CLOCK_TOLERANCE_S = 1.0


def _sync_robot_clock(robot_id: str) -> tuple[bool, str]:
    """Set the robot's clock to this PC's when they differ by more than CLOCK_TOLERANCE_S. (changed, what happened)

    A Pinky has no clock battery and the site has no internet, so each one keeps its own wrong time (10번 three months
    behind, 12번 ten days, 2026-11-16). Poses sent from this PC then carry a time AMCL cannot place, and the logs of
    different machines cannot be lined up. Nav2 is stopped first so a running stack never sees time jump; the caller
    must then start a fresh one."""
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
    prior = _last_confirmed_pose(robot_id)
    with STATE.lock:
        STATE.robots[robot_id].update(stage="base_starting", map=map_name, message="기본 bringup과 LiDAR를 시작합니다.", error=None)
    try:
        cleared, clear_output = _stop_existing_base(robot_id)
        if not cleared or "BASE_PROCESSES_CLEARED" not in clear_output:
            raise RuntimeError("이전 기본 구동·LiDAR 프로세스를 정리하지 못했습니다: " + clear_output.strip())
        # Before any ROS node starts: the robot's clock to this PC's (it stops a running Nav2 if it has to change it).
        clock_changed, clock_note = _sync_robot_clock(robot_id)
        STATE.update(robot_id, clock=clock_note)
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
        # Nav2 is not kept any more (2026-11-16): the base above was just restarted, and a running AMCL then drops
        # every scan (odometry and TF start over under it) and stops taking poses. One composed process comes up in
        # about 30 s, so a clean Nav2 each time is the safer trade.
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
        # A freshly started AMCL has no pose, so the robot has no map position (tf_pose) and cannot be dispatched.
        # The robot finds itself on the map with its own LiDAR.
        STATE.update(robot_id, message="라이다로 지도에서 자기 위치를 찾습니다.")
        located, detail = _localize_with_lidar(robot_id, prior=prior)
        if located:
            STATE.update(robot_id, stage="prepared", error=None,
                         message=f"센서·지도·Nav2를 확인했습니다 ({map_name}). {detail}")
        else:
            # Nav2 is up, so the stage is prepared; with no map position the robot still reads as not ready.
            STATE.update(robot_id, stage="prepared", error=detail[-1200:],
                         message="Nav2는 준비됐지만 라이다로 위치를 찾지 못했습니다. [라이다로 위치 찾기]를 다시 누르거나, "
                                 f"{PARKING[robot_id]['label']} 칸에 있으면 [주차장 좌표로 맞추기]를 쓰세요.")
    except Exception as exc:  # surfaced in the dashboard for operator action
        STATE.update(robot_id, stage="fault", message="준비 과정에서 오류가 발생했습니다.", error=str(exc))
    finally:
        with STATE.lock:
            if release_busy and STATE.busy_robot == robot_id:
                STATE.busy_robot = None


def _prepare_robots_in_turn(robot_ids: list[str], map_name: str) -> None:
    """주행준비 for several robots one after another under one busy lock; one robot's fault does not stop the rest."""
    try:
        for robot_id in robot_ids:
            _prepare_robot(robot_id, map_name, release_busy=False)
    finally:
        with STATE.lock:
            if STATE.busy_robot == "all":
                STATE.busy_robot = None


def _recover_navigation(robot_id: str) -> None:
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
        # A restarted AMCL has no pose. Recovery is refused while a route runs, so the robot stands still and the
        # LiDAR can find it again wherever it stopped.
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


def _traffic_observations(telemetry):
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
    """Twice a second, each Pinky's lamp colour (scripts/fleet_ops.indicator) to its own domain."""
    from std_msgs.msg import String
    while True:
        time.sleep(0.5)
        if TELEMETRY is None:
            continue
        try:
            telemetry = TELEMETRY.snapshot()
            with STATE.lock:
                wanted = {rid: indicator(info, STATE.routes[rid], operational_status(info, telemetry.get(rid, {})))
                          for rid, info in STATE.robots.items()}
            for rid, value in wanted.items():
                TELEMETRY.indicator_publishers[rid].publish(String(data=json.dumps(value)))
        except Exception as exc:  # the lamp is a convenience; never let it stop the dashboard
            JOURNAL.record('indicator_error', state=str(exc))


def _traffic_loop():
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
        if '/dock-guide' in str(args[0] if args else ''):
            return  # several a second while a Pinky drives in: not a line each
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
        processes_match = re.fullmatch(r"/api/robots/(10|11|12)/processes", path)
        if processes_match:
            # Read only, for developers: what runs on the robot (ROS, Nav2, Python), busiest first.
            ok, output = _ssh_run(processes_match.group(1),
                                  "uptime; ps -eo pid,etime,pcpu,pmem,args --sort=-pcpu | grep -E 'ros|nav2|python3|PID' | grep -v grep | cut -c1-220 | head -60",
                                  keep_chars=12000)
            self._json(200 if ok else 502, {"ok": ok, "output": output})
            return
        nav_log_match = re.fullmatch(r"/api/robots/(10|11|12)/nav-log", path)
        if nav_log_match:
            # Read only, for developers: why Nav2 did not come up, without the repeated keepout TF warnings.
            log = NAV_LOG_PATH
            ok, output = _ssh_run(nav_log_match.group(1),
                                  f"ls -la {log}; date; grep -anE 'ERROR|FATAL|died|exit code|Timed out|timed out|Failed|terminate|Aborting|Activating|Configuring|Waiting' {log} "
                                  "| grep -v KeepoutFilter | tail -120 | cut -c1-260; echo '--- 마지막 40줄 ---'; "
                                  f"tail -40 {log} | grep -av KeepoutFilter | cut -c1-260", keep_chars=40000)
            self._json(200 if ok else 502, {"ok": ok, "output": output})
            return
        locate_match = re.fullmatch(r"/api/robots/(10|11|12)/locate", path)
        if locate_match:
            # Read only: where the LiDAR puts the robot now and the other places that fit; AMCL is not touched.
            match, why_not = _lidar_match(locate_match.group(1), time.time() - 1.0)
            if match is None:
                self._json(409, {"error": why_not})
            else:
                self._json(200, {"robot": locate_match.group(1), "match": match.as_dict()})
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
        if path=='/api/traffic/release':
            try:
                rid=str(data.get('robot_id'))
                if rid not in ROBOTS:raise ValueError('로봇을 확인하세요.')
                observation=_traffic_observations(TELEMETRY.snapshot() if TELEMETRY else {}).get(rid)
                TRAFFIC.release(data.get('zone'),rid,data.get('token'),observation)
                self._json(200,{'ok':True})
            except (ValueError,OSError) as exc:self._json(409,{'error':str(exc)})
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
        guide_match = re.fullmatch(r'/api/robots/(10|11|12)/dock-guide', path)
        if guide_match:
            # The ceiling camera's live offset of this Pinky from its loading spot (the control station sends it a few
            # times a second while the Pinky is in view). It goes to the Pinky as it is, on /fleet/dock_guide; only the
            # drive into station 5 steers on it there (mission_behaviors.DockEntry). navigation.json dock_camera_guide
            # false turns the relay off, and the Pinky then drives in on odometry as before.
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
            # redock: the ceiling camera judged the Pinky off the loading spot; it backs out and comes in again.
            # redock is also taken after dock_complete (docked_manual), until the load is confirmed (2026-11-19).
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
            # A test mission without an order: from where the Pinky stands to station 5 (stop, dock, load), then
            # to the drop-off waypoint (stop, hand over), then back to its own parking spot. Nav2 plans the way
            # between stops; the shared zones (parking exit, junction 7, station 5) are still taken one at a time.
            robot_id = mission_match.group(1)
            dropoff = data.get("dropoff")
            if not isinstance(dropoff, str) or not dropoff.strip() or dropoff.strip() == "5":
                self._json(400, {"error": "배달 목적지 웨이포인트 이름을 고르세요 (5번은 적재 자리라 안 됩니다)."})
                return
            dropoff = dropoff.strip()
            # "원점": load at 5, back out, and drive back to where it started (facing as it stood); no hand-over stop.
            return_to_start = dropoff == "원점"
            # "주차장": load at 5, back out, and drive to its own parking spot (no receiving zone).
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
                    # `via` takes the Pinky through chosen waypoints on the way out (e.g. 9, 7: the right-hand passage
                    # past the middle wall) and back the same way in reverse, so Nav2 does not pick another gap.
                    out = [dict(item) for _, item in passes]
                    back = [dict(item) for _, item in reversed(passes)]
                    # Station 5 is entered from the queue spot (navigation.json dock_queue): one waits there in turn.
                    # Before it, the waiting spot below the passage (wait_low), so the Pinkys line up in order.
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
                goals[0]["parking_yaw_rad"] = pose["yaw_rad"]  # leave from where it stands, facing as it stands
                anchors = [p for p in goals if p.get("anchor")]
                if not return_to_start:
                    goals[-1]["yaw_rad"] = parking["yaw_rad"]
                    goals[-1]["parking_return"] = True
                if not return_to_start and not straight_home:
                    drop_anchor = [p for p in anchors if p["label"] == dropoff][0]
                    drop_anchor["course_end"] = True  # the drop-off: wait for the hand-over signal
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
                    # Last course point gates the external completion signal.
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
            # LiDAR localization goes to AMCL over DDS from this PC: no SSH password, only a running Nav2.
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
                    # Stopping a Pinky that already stopped (or was waiting to go home) holds it where it is.
                    if route.get('status') in {'cancelled','failed','completed'}:
                        route['hold']=True
                    route.update(status='cancel_requested',message='정지 유지와 Nav2 취소를 요청합니다.',auto_return_at=None)
                route['stop_pending']=True
            WORKERS.submit(_cancel_waypoint_route, robot_id)
            self._json(202, {"ok": True, "message": "경로 취소를 요청했습니다."})
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
                    # "prepared" counts only while its Nav2 still gives a map pose: a Pinky switched off and on again
                    # keeps the saved stage (it survives a dashboard restart) but has no Nav2 until prepared again.
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
            map_name = data.get("map", "map-proto.yaml")  # waypoints, keepout and receiving zones all use map-proto
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
                # A prepared Pinky may be prepared again (after it was switched off, or to start clean); not mid-route.
                route = STATE.routes[robot_id] or {}
                if route.get("status") in {"starting", "running", "cancel_requested"}:
                    self._json(409, {"error": "경로 주행을 정지한 뒤 주행준비하세요."})
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
    threading.Thread(target=_traffic_loop,daemon=True,name="traffic-coordinator").start()
    threading.Thread(target=_health_loop, daemon=True, name="robot-health").start()
    threading.Thread(target=_indicator_loop, daemon=True, name="status-lamp").start()
    threading.Thread(target=_auto_return_loop, daemon=True, name="auto-return").start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    # rclpy installs signal handlers that stop ROS without stopping HTTP.
    # Shut down both cleanly so restarting does not strand DDS participants.
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
