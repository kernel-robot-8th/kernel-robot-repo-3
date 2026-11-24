#!/usr/bin/env python3
"""라이다 + 초음파 센서 퓨전: 로봇 둘레의 확률 점유 격자, 칸(기둥)마다 높이 두 층.

실행 위치: PC·로봇 양쪽. 로봇에서는 주행준비 때 runtime_bundle로 ~/.cache/pinky-fleet/nav2/에 올라가고,
  navigation.ultrasonic_fusion이 켜져 있을 때만 app.py가 SSH로 이 노드(main)를 띄운다.
  PC에서는 app.py가 validated()만 import해 로봇에 올리기 전에 navigation.json의 fusion_grid 설정을 검사한다.
사용처: app.py, scripts/fusion_eval.py(오프라인 재생), tests/test_fusion_grid.py, tests/ros_fusion_smoke.py.
  Nav2 쪽 연결(costmap VoxelLayer 소스)은 scripts/install_nav2_runtime.py의 ULTRASONIC_SOURCES에 있다.

왜: 라이다는 바닥에서 약 12.5 cm 높이의 평면 하나만 보므로 그보다 낮은 상자는 보이지 않는다. 전면 초음파는
바닥에서 약 3.7 cm 높이를 보고 그 상자를 보지만, "약 15도 원뿔 안 어딘가, 이 거리에 무언가 있다"로만 본다.
어느 센서도 혼자서는 쓸 만한 낮은 장애물을 주지 못하고, 둘을 합치면 준다.

방법 (log-odds 형태의 역센서 모델로 하는 점유 격자 지도 작성, Probabilistic Robotics 9장):
  * 같은 칸들 위에 log-odds 층 두 개: `high`(라이다 평면)와 `low`(초음파 평면). 각 센서는 자기 높이 층만
    고치므로, 낮은 상자 위를 지나가는 라이다 빔이 그 상자를 지우는 일이 없다.
  * 라이다 역모델: 빔이 지나간 칸은 비어 있을 가능성이, 측정 거리 +-sigma 안의 칸은 점유일 가능성이 커진다.
    가는 빔이라 근거가 날카롭고 강하다.
  * 초음파 역모델: 원뿔 안에서 반사 거리보다 가까운 곳 -> 비어 있음, 반사 거리의 호 위 -> 점유(측정 거리
    노이즈의 가우시안으로 가중). 호가 넓어 칸 하나에 주는 근거는 약하다: 엉뚱한 반사 한 번으로는 장애물이
    생기지 않고, 일관된 반사가 쌓여야 생긴다.
  * 센서 간 연관: 원뿔의 각도 모호성을 라이다로 푼다. 호가 라이다가 이미 점유로 보는 칸과 만나면 반사는 그
    장애물이고 근거는 그 칸에만 간다. 라이다가 자기 높이에서 그 호를 비었다고 봤으면 반사는 라이다 평면 아래에서
    온 것이다: 낮은 장애물로 보고 호 전체에 퍼뜨린다. 라이다가 본 적 없는 곳이면 역시 퍼뜨리되 unresolved로 보고한다.
  * 기둥 규칙: 어느 높이든 점유면 그 칸은 로봇을 막는다(Nav2 VoxelLayer의 mark_threshold 0과 같다). 근거는
    층마다 다른 시간 상수로 "모름" 쪽으로 감쇠하므로, 옮긴 상자나 잘못 찍힌 표시는 사라진다.

FusionGrid에는 ROS가 없어 단위 테스트하고, 녹화한 주행에 오프라인으로 재생한다(scripts/fusion_eval.py).
로봇에서는 main()이 /scan, /us_sensor/range, TF를 읽어 다음을 발행한다:
  /fusion/low_marks  PointCloud2 (base_footprint): 점유된 low 층 칸, Nav2의 marking 소스
  /fusion/low_clear  PointCloud2 (base_footprint): 센서 높이에서 창 가장자리에 둔 링, clearing 소스.
                     이 광선은 라이다 평면 아래에 머무르므로 Nav2의 낮은 칸은 이 격자를 따르고 라이다 칸은 그대로다
  /fusion/grid       OccupancyGrid (odom): 합친 기둥 확률, RViz와 관제용
  /fusion/status     String JSON: 칸 수, 마지막 반사의 분류, 가장 가까운 낮은 장애물, 갱신 시간
설정: /home/pinky/.cache/pinky-fleet/nav2/runtime_config.json -> navigation.fusion_grid.
최종 정지(velocity_guard.py)는 이 노드를 쓰지 않고 센서를 직접 본다: 퓨전이 죽거나 틀려도 마지막 정지는 따로 돈다.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

# 기본 설정. navigation.json의 fusion_grid에 적은 값이 이를 덮는다(validated). 단위: m, rad, s, 확률 p는 0..1.
DEFAULTS = dict(
    size_m=2.0,                 # 로봇 중심의 정사각형 창 (한 변 m)
    resolution_m=0.02,          # 칸 한 변 2 cm -> 100 x 100 칸
    # 라이다(약 12.5 cm 높이): sigma는 노이즈 측정값, 나머지는 역모델 확률. beam_stride 2는 빔을 하나 걸러 쓴다
    lidar_sigma_m=0.01, lidar_hit_p=0.85, lidar_free_p=0.35, lidar_beam_stride=2,
    # 초음파(약 3.7 cm 높이). 시야 0.26 rad(약 15도), 3 cm보다 가까운 값은 믿지 않는다
    us_sigma_m=0.015, us_fov_rad=0.26, us_min_range_m=0.03,
    us_mark_max_range_m=0.6,    # 더 먼 반사는 비우기만 한다: 3.7 cm 높이에서는 원뿔이 바닥에 닿는다
    us_clear_max_range_m=0.8,   # 반사가 없을 때 비었다고 보는 거리
    us_hit_p=0.62,              # 호 위 칸마다: 약하게, 물체는 호 어딘가에 있다
    us_explained_p=0.80,        # 라이다도 점유로 보는 호 위 칸
    us_free_p=0.40,
    # 연관: 라이다 칸이 이만큼 확실하면 반사를 설명한다. 호의 이 비율 이상이 라이다로 비었으면 "라이다 아래"
    explain_p=0.7, lidar_free_known_p=0.35, known_free_share=0.5,
    # "모름" 쪽 감쇠(시간 상수 s). 라이다는 1초에 10번 둘레 전체를 다시 보지만 초음파는 좁은 원뿔 하나만 보므로,
    # 로봇이 낮은 장애물을 돌아 피하는 동안 그것을 기억하게 low 층은 오래 둔다.
    decay_high_s=4.0, decay_low_s=30.0,
    # OctoMap과 같은 clamping(0.12 / 0.97): 여러 번 비었다고 본 칸도 상자를 놓으면 반사 몇 번 만에 점유가 되고,
    # 점유 칸도 적당한 시간 안에 다시 비워진다. occupied_p 이상이면 점유로 판단한다.
    clamp_free_p=0.12, clamp_occupied_p=0.97, occupied_p=0.7,
    # 비교 실험용 스위치 (scripts/fusion_eval.py)
    use_ultrasonic=True, associate=True,
    us_accumulate=True,         # False: 읽을 때마다 칸을 덮어쓴다 (이진 표시/비우기, 기억 없음)
)

# 초음파 한 번의 분류(/fusion/status의 last_echo, echo_counts). lidar_obstacle: 라이다가 본 장애물의 반사,
# low_obstacle: 라이다는 비었다고 본 호(라이다 아래 물체), unresolved: 라이다가 본 적 없는 호,
# echo: associate=False일 때의 반사, far: mark_max..clear_max 사이 반사(비우기만), free: 반사 없음,
# unsure: NaN·최소 거리 미만·호가 창 밖, off: use_ultrasonic=False.
ECHO_CLASSES = ('lidar_obstacle', 'low_obstacle', 'unresolved', 'echo', 'far', 'free', 'unsure', 'off')


def logit(p: float) -> float:
    """확률 p를 log-odds log(p / (1 - p))로 바꾼다. 0.5 -> 0(모름), 점유 쪽은 양수, 비어 있음 쪽은 음수다.
    log-odds에서는 독립 측정을 합치는 베이즈 갱신이 덧셈이 된다."""
    return math.log(p / (1.0 - p))


def _wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def _shifted(a: np.ndarray, di: int, dj: int) -> np.ndarray:
    """new[i, j] = a[i + di, j + dj]. 그 자리가 배열 밖이면 0(모름)이다."""
    n_i, n_j = a.shape
    out = np.zeros_like(a)
    if abs(di) >= n_i or abs(dj) >= n_j:
        return out
    out[max(-di, 0):n_i + min(-di, 0), max(-dj, 0):n_j + min(-dj, 0)] = \
        a[max(di, 0):n_i + min(di, 0), max(dj, 0):n_j + min(dj, 0)]
    return out


def validated(settings: dict | None) -> dict:
    s = {**DEFAULTS, **(settings or {})}
    unknown = set(s) - set(DEFAULTS)
    if unknown:
        raise ValueError('unknown fusion_grid settings: ' + ', '.join(sorted(unknown)))
    for key, value in s.items():
        if isinstance(DEFAULTS[key], bool):
            if not isinstance(value, bool):
                raise ValueError(f'{key} must be true or false')
        elif not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be a positive number')
    for key in ('lidar_hit_p', 'lidar_free_p', 'us_hit_p', 'us_explained_p', 'us_free_p', 'explain_p',
                'lidar_free_known_p', 'known_free_share', 'occupied_p', 'clamp_free_p', 'clamp_occupied_p'):
        if not 0 < s[key] < 1:
            raise ValueError(f'{key} must be between 0 and 1')
    for key in ('lidar_hit_p', 'us_hit_p', 'us_explained_p', 'explain_p', 'occupied_p'):
        if s[key] <= 0.5:
            raise ValueError(f'{key} must say "occupied" (above 0.5)')
    for key in ('lidar_free_p', 'us_free_p', 'lidar_free_known_p', 'clamp_free_p'):
        if s[key] >= 0.5:
            raise ValueError(f'{key} must say "free" (below 0.5)')
    if not 20 <= round(s['size_m'] / s['resolution_m']) <= 400:
        raise ValueError('size_m / resolution_m must give 20..400 cells')
    if not s['us_min_range_m'] < s['us_mark_max_range_m'] <= s['us_clear_max_range_m']:
        raise ValueError('ultrasonic ranges must be min < mark_max <= clear_max')
    if s['clamp_occupied_p'] <= s['occupied_p']:
        raise ValueError('clamp_occupied_p must be above occupied_p, or nothing could ever be occupied')
    return s


class FusionGrid:
    """Two log-odds layers (high = LiDAR plane, low = ultrasonic plane) in a window around the robot, odom frame.

    Cell (i, j) covers x in origin_x + [j, j+1) * res and y in origin_y + [i, i+1) * res (row = y, as in
    nav_msgs/OccupancyGrid)."""

    def __init__(self, settings: dict | None = None):
        self.s = s = validated(settings)
        self.res = float(s['resolution_m'])
        self.n = int(round(s['size_m'] / self.res))
        self.high = np.zeros((self.n, self.n), np.float32)
        self.low = np.zeros((self.n, self.n), np.float32)
        self.origin = np.array([-self.n * self.res / 2, -self.n * self.res / 2])
        self.last_decay: float | None = None
        self.l = dict(lidar_hit=logit(s['lidar_hit_p']), lidar_free=logit(s['lidar_free_p']),
                      us_hit=logit(s['us_hit_p']), us_explained=logit(s['us_explained_p']), us_free=logit(s['us_free_p']),
                      explain=logit(s['explain_p']), known_free=logit(s['lidar_free_known_p']),
                      occupied=logit(s['occupied_p']), floor=logit(s['clamp_free_p']), ceiling=logit(s['clamp_occupied_p']))
        self.last_echo: dict = dict(kind='none')
        self.echo_counts = {kind: 0 for kind in ECHO_CLASSES}

    # -- frame keeping -------------------------------------------------------------------------------------------
    def center(self) -> np.ndarray:
        return self.origin + self.n * self.res / 2

    def recenter(self, x: float, y: float) -> None:
        """Keep the robot at the centre (within half a cell): shift the layers by whole cells, unknown comes in."""
        dj, di = (int(round(v)) for v in (np.array([x, y]) - self.center()) / self.res)
        if di or dj:
            self.high = _shifted(self.high, di, dj)
            self.low = _shifted(self.low, di, dj)
            self.origin = self.origin + np.array([dj, di]) * self.res

    def decay(self, now: float) -> None:
        if self.last_decay is None:
            self.last_decay = now
            return
        dt = now - self.last_decay
        if dt <= 0:
            return  # replayed messages can arrive slightly out of order
        self.high *= np.float32(math.exp(-dt / self.s['decay_high_s']))
        self.low *= np.float32(math.exp(-dt / self.s['decay_low_s']))
        self.last_decay = now

    def cell_index(self, xs, ys):
        """Flat indices of the cells holding the points, inside the window only."""
        j = np.floor((np.asarray(xs) - self.origin[0]) / self.res).astype(np.int64)
        i = np.floor((np.asarray(ys) - self.origin[1]) / self.res).astype(np.int64)
        inside = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        return i[inside] * self.n + j[inside]

    def cell_centers(self):
        axis = (np.arange(self.n) + 0.5) * self.res
        return np.meshgrid(self.origin[0] + axis, self.origin[1] + axis)  # X[i, j], Y[i, j]

    def _clip(self) -> None:
        np.clip(self.high, self.l['floor'], self.l['ceiling'], out=self.high)
        np.clip(self.low, self.l['floor'], self.l['ceiling'], out=self.low)

    def _low(self, mask: np.ndarray, delta) -> None:
        """Ultrasonic evidence for the low layer: added (Bayesian update), or written over (binary baseline)."""
        if self.s['us_accumulate']:
            self.low[mask] += delta
        else:
            self.low[mask] = np.where(np.asarray(delta) > 0, self.l['ceiling'], self.l['floor'])

    # -- LiDAR ---------------------------------------------------------------------------------------------------
    def update_scan(self, pose, angles, ranges, range_min: float, range_max: float, now: float) -> dict:
        """pose = (x, y, yaw) of the LiDAR in odom; angles in the LiDAR frame. Updates the high layer only."""
        self.decay(now)
        x, y, yaw = pose
        stride = max(1, int(self.s['lidar_beam_stride']))
        angles = np.asarray(angles, float)[::stride]
        r = np.asarray(ranges, float)[::stride]
        no_return = np.isinf(r) & (r > 0)
        valid = (np.isfinite(r) & (r >= range_min) & (r <= range_max)) | no_return
        r, a = r[valid], angles[valid] + yaw
        if not len(r):
            return dict(beams=0, hits=0)
        limit = self.n * self.res / 2
        sigma = max(float(self.s['lidar_sigma_m']), self.res / 2)
        hit = np.isfinite(r) & (r <= limit)
        free_end = np.where(hit, r - sigma, limit)
        step = self.res / 2
        t = np.arange(int(limit / step) + 1) * step
        c, s = np.cos(a)[:, None], np.sin(a)[:, None]
        along = t[None, :] < free_end[:, None]
        free = np.unique(self.cell_index((x + c * t)[along], (y + s * t)[along]))
        hit_cells = np.empty(0, np.int64)
        if hit.any():
            offsets = np.linspace(-sigma, sigma, int(math.ceil(2 * sigma / step)) + 1)
            band = r[hit][:, None] + offsets[None, :]
            hit_cells = np.unique(self.cell_index(x + c[hit] * band, y + s[hit] * band))
        free = np.setdiff1d(free, hit_cells, assume_unique=True)
        flat = self.high.reshape(-1)
        flat[free] += self.l['lidar_free']
        flat[hit_cells] += self.l['lidar_hit']
        self._clip()
        return dict(beams=int(len(r)), hits=int(hit.sum()))

    # -- ultrasonic ----------------------------------------------------------------------------------------------
    def update_range(self, pose, distance: float, fov: float | None, now: float) -> str:
        """pose = (x, y, yaw) of the ultrasonic in odom. Updates the low layer only; returns the echo's class."""
        self.decay(now)
        kind = self._update_range(pose, float(distance), fov)
        self.echo_counts[kind] += 1
        self.last_echo = dict(kind=kind, range_m=float(distance) if math.isfinite(distance) else None, at=now)
        return kind

    def _update_range(self, pose, r: float, fov) -> str:
        s = self.s
        if not s['use_ultrasonic']:
            return 'off'
        if math.isnan(r) or (math.isfinite(r) and r < s['us_min_range_m']):
            return 'unsure'
        width = fov if fov is not None and math.isfinite(fov) and 0 < fov < math.pi else s['us_fov_rad']
        sx, sy, syaw = pose
        X, Y = self.cell_centers()
        d = np.hypot(X - sx, Y - sy)
        cone = np.abs(_wrap(np.arctan2(Y - sy, X - sx) - syaw)) <= width / 2
        sigma = max(float(s['us_sigma_m']), self.res / 2)
        band = 2 * sigma
        if not math.isfinite(r) or r > s['us_mark_max_range_m']:
            # No echo, or one so far that it may be the floor: only "nothing low before it"
            reach = s['us_clear_max_range_m'] if not math.isfinite(r) else min(r, s['us_clear_max_range_m'])
            self._low(cone & (d < reach - band), self.l['us_free'])
            self._clip()
            return 'free' if not math.isfinite(r) or r > s['us_clear_max_range_m'] else 'far'
        self._low(cone & (d < r - band), self.l['us_free'])
        arc = cone & (np.abs(d - r) <= band)
        if not arc.any():
            self._clip()
            return 'unsure'
        weight = np.exp(-((d - r) ** 2) / (2 * sigma ** 2))
        if s['associate']:
            explained = arc & (self.high >= self.l['explain'])
            if explained.any():
                self._low(explained, self.l['us_explained'] * weight[explained])
                self._clip()
                return 'lidar_obstacle'
            known_free = float(np.mean(self.high[arc] <= self.l['known_free']))
            kind = 'low_obstacle' if known_free >= s['known_free_share'] else 'unresolved'
        else:
            kind = 'echo'
        self._low(arc, self.l['us_hit'] * weight[arc])
        self._clip()
        return kind

    # -- results -------------------------------------------------------------------------------------------------
    def low_occupied(self) -> np.ndarray:
        return self.low >= self.l['occupied']

    def low_obstacle_cells(self) -> np.ndarray:
        """Occupied low cells the LiDAR does not see as occupied: obstacles below the LiDAR plane."""
        return self.low_occupied() & (self.high < self.l['explain'])

    def column_occupied(self) -> np.ndarray:
        return (self.high >= self.l['occupied']) | self.low_occupied()

    def points(self, mask: np.ndarray) -> np.ndarray:
        """(k, 2) odom coordinates of the centres of the cells in mask."""
        i, j = np.nonzero(mask)
        return np.column_stack([self.origin[0] + (j + 0.5) * self.res, self.origin[1] + (i + 0.5) * self.res])

    def occupancy_data(self) -> np.ndarray:
        """int8 row-major for nav_msgs/OccupancyGrid, -1 = unknown. A height with evidence of occupancy decides the
        column (the larger one); with none, the clearer free one shows, so a LiDAR-free cell whose low height was
        never looked at reads free rather than 50."""
        top = np.maximum(self.high, self.low)
        p = 1.0 / (1.0 + np.exp(-np.where(top > 0, top, np.minimum(self.high, self.low))))
        data = np.round(100 * p).astype(np.int8)
        data[(np.abs(self.high) < 0.1) & (np.abs(self.low) < 0.1)] = -1
        return data.reshape(-1)

    def status(self, robot_xy=None) -> dict:
        low_cells = self.low_obstacle_cells()
        nearest = None
        if robot_xy is not None and low_cells.any():
            pts = self.points(low_cells)
            nearest = round(float(np.min(np.hypot(pts[:, 0] - robot_xy[0], pts[:, 1] - robot_xy[1]))), 3)
        return dict(low_obstacle_cells=int(low_cells.sum()), low_cells=int(self.low_occupied().sum()),
                    high_cells=int((self.high >= self.l['occupied']).sum()), nearest_low_obstacle_m=nearest,
                    last_echo=self.last_echo, echo_counts=dict(self.echo_counts))


def ring(radius: float, z: float, count: int = 180) -> list[tuple[float, float, float]]:
    return [(radius * math.cos(2 * math.pi * k / count), radius * math.sin(2 * math.pi * k / count), z)
            for k in range(count)]


def to_local(points: np.ndarray, pose) -> np.ndarray:
    """Odom points into the frame at pose = (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


LOW_Z_MAX = 0.045  # the low cloud must stay in Nav2's lowest 5 cm voxel row, under the LiDAR plane


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.duration import Duration
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan, PointCloud2, Range
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    runtime_path = Path('/home/pinky/.cache/pinky-fleet/nav2/runtime_config.json')
    try:
        runtime = json.loads(runtime_path.read_text())
    except (OSError, ValueError):
        runtime = {}
    settings = (runtime.get('navigation') or {}).get('fusion_grid') or {}
    geometry = runtime.get('geometry') or {}

    class FusionNode(Node):
        def __init__(self):
            super().__init__('pinky_fusion_grid')
            self.grid = FusionGrid(settings)
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.marks = self.create_publisher(PointCloud2, '/fusion/low_marks', qos_profile_sensor_data)
            self.clear = self.create_publisher(PointCloud2, '/fusion/low_clear', qos_profile_sensor_data)
            self.grid_pub = self.create_publisher(OccupancyGrid, '/fusion/grid', 1)
            self.status_pub = self.create_publisher(String, '/fusion/status', 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Range, '/us_sensor/range', self.on_range, qos_profile_sensor_data)
            self.create_timer(0.2, self.publish)
            self.create_timer(0.5, self.publish_grid)
            self.us_z = 0.037
            self.timing = dict(scan_ms=None, range_ms=None, publish_ms=None)
            self.last_scan_at = self.last_range_at = 0.0
            self.errors: dict[str, str] = {}
            self.get_logger().info('LiDAR + ultrasonic fusion grid ready')

        def lookup(self, frame: str, stamp):
            """(x, y, yaw, z) of frame in odom at the stamp, else the latest; None without TF."""
            for when in ((Time.from_msg(stamp),) if stamp.sec or stamp.nanosec else ()) + (Time(),):
                try:
                    tf = self.tf.lookup_transform('odom', frame.lstrip('/'), when, timeout=Duration(seconds=0.0))
                    t = tf.transform.translation
                    return t.x, t.y, yaw_of(tf.transform.rotation), t.z, tf.header.stamp
                except TransformException as exc:
                    self.errors['tf'] = str(exc)[:160]
            return None

        def on_scan(self, msg):
            pose = self.lookup(msg.header.frame_id, msg.header.stamp) if msg.header.frame_id else None
            if pose is None:
                return
            started = time.perf_counter()
            angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
            self.grid.update_scan(pose[:3], angles, msg.ranges, msg.range_min, msg.range_max, time.monotonic())
            self.timing['scan_ms'] = round((time.perf_counter() - started) * 1000, 2)
            self.last_scan_at = time.monotonic()

        def on_range(self, msg):
            if msg.header.frame_id:
                pose = self.lookup(msg.header.frame_id, msg.header.stamp)
                if pose is None:
                    return
                self.us_z = min(max(pose[3], 0.005), LOW_Z_MAX)
                pose = pose[:3]
            else:  # a driver without a frame: the measured mounting from robot_geometry.json
                base = self.lookup('base_footprint', msg.header.stamp)
                if base is None:
                    return
                bx, by, byaw = base[:3]
                ux, uy = geometry.get('ultrasound_x_m', 0.0), geometry.get('ultrasound_y_m', 0.0)
                pose = (bx + math.cos(byaw) * ux - math.sin(byaw) * uy, by + math.sin(byaw) * ux + math.cos(byaw) * uy,
                        byaw + geometry.get('ultrasound_yaw_rad', 0.0))
            started = time.perf_counter()
            self.grid.update_range(pose, float(msg.range), float(msg.field_of_view), time.monotonic())
            self.timing['range_ms'] = round((time.perf_counter() - started) * 1000, 2)
            self.last_range_at = time.monotonic()

        def publish(self):
            base = self.lookup('base_footprint', Time().to_msg())
            if base is None:
                return
            started = time.perf_counter()
            bx, by, byaw, _, stamp = base
            self.grid.recenter(bx, by)
            header = PointCloud2().header
            header.frame_id, header.stamp = 'base_footprint', stamp
            local = to_local(self.grid.points(self.grid.low_occupied()), (bx, by, byaw))
            self.marks.publish(point_cloud2.create_cloud_xyz32(header, [(px, py, self.us_z) for px, py in local]))
            radius = self.grid.n * self.grid.res / 2 - self.grid.res
            self.clear.publish(point_cloud2.create_cloud_xyz32(header, ring(radius, self.us_z)))
            self.timing['publish_ms'] = round((time.perf_counter() - started) * 1000, 2)
            now = time.monotonic()
            status = dict(version='fusion_grid_v1', **self.grid.status((bx, by)), timing=self.timing,
                          scan_fresh=now - self.last_scan_at < 0.6, ultrasonic_fresh=now - self.last_range_at < 0.6,
                          errors=self.errors)
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

        def publish_grid(self):
            msg = OccupancyGrid()
            msg.header.frame_id = 'odom'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.info.resolution = self.grid.res
            msg.info.width = msg.info.height = self.grid.n
            msg.info.origin.position.x, msg.info.origin.position.y = (float(v) for v in self.grid.origin)
            msg.info.origin.orientation.w = 1.0
            msg.data = self.grid.occupancy_data().tolist()
            self.grid_pub.publish(msg)

    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = FusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C, or the dashboard stopping it with SIGTERM
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
