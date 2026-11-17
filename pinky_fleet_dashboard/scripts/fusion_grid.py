#!/usr/bin/env python3
"""LiDAR + ultrasonic sensor fusion: a probabilistic occupancy grid around the robot, two heights per column.

Why: the LiDAR sees one plane about 12.5 cm above the floor, so a box lower than that is invisible to it. The front
ultrasonic looks about 3.7 cm above the floor and does see it, but only as "something in this ~15 degree cone at
this distance". Neither sensor alone gives a usable low obstacle; together they do.

How (occupancy grid mapping with inverse sensor models in log-odds form, Probabilistic Robotics ch. 9):
  * Two log-odds layers over the same cells: `high` (the LiDAR plane) and `low` (the ultrasonic plane). Each
    sensor updates only the layer of its own height, so a LiDAR ray passing over a low box never erases it.
  * LiDAR inverse model: cells the beam passes through become more likely free, cells within +-sigma of the
    measured range more likely occupied. A thin beam gives sharp, strong evidence.
  * Ultrasonic inverse model: inside the cone short of the echo -> free; on the arc at the echo distance ->
    occupied, weighted by a Gaussian of the measured range noise. The arc is wide, so the evidence per cell is weak:
    one stray echo never makes an obstacle, consistent echoes accumulate.
  * Cross-sensor association: the LiDAR resolves the cone's angular ambiguity. If the arc meets cells the LiDAR
    already holds as occupied, the echo is that obstacle and the evidence goes to those cells only. If the LiDAR has
    seen the arc as free at its height, the echo must come from below the LiDAR plane: a low obstacle, spread over
    the arc. If the LiDAR has not looked there, the echo is spread too but reported as unresolved.
  * Column rule: a cell blocks the robot when either height is occupied (as Nav2's VoxelLayer with
    mark_threshold 0). Evidence decays toward unknown with a time constant per layer, so a moved box or a stray
    mark fades out.

FusionGrid has no ROS in it, so it is unit tested and replayed offline on recorded runs (scripts/fusion_eval.py).
On the robot, main() reads /scan, /us_sensor/range and TF and publishes:
  /fusion/low_marks  PointCloud2 (base_footprint): occupied low-layer cells, a marking source for Nav2
  /fusion/low_clear  PointCloud2 (base_footprint): a ring at the window edge at sensor height, a clearing source;
                     its rays stay below the LiDAR plane, so Nav2's low cells follow this grid and the LiDAR's do not
  /fusion/grid       OccupancyGrid (odom): fused column probability, for RViz and the dashboard
  /fusion/status     String JSON: cell counts, the last echo's class, the nearest low obstacle, update times
Settings: /home/pinky/.cache/pinky-fleet/nav2/runtime_config.json -> navigation.fusion_grid.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

DEFAULTS = dict(
    size_m=2.0,                 # square window centred on the robot
    resolution_m=0.02,
    # LiDAR (about 12.5 cm high): sigma from the noise measurement, probabilities of the inverse model
    lidar_sigma_m=0.01, lidar_hit_p=0.85, lidar_free_p=0.35, lidar_beam_stride=2,
    # Ultrasonic (about 3.7 cm high)
    us_sigma_m=0.015, us_fov_rad=0.26, us_min_range_m=0.03,
    us_mark_max_range_m=0.6,    # farther echoes only clear: from 3.7 cm high the cone meets the floor
    us_clear_max_range_m=0.8,
    us_hit_p=0.62,              # per arc cell: weak, the object is somewhere on the arc
    us_explained_p=0.80,        # the arc cell the LiDAR also holds as occupied
    us_free_p=0.40,
    # Association: LiDAR cells this sure explain an echo; an arc this share LiDAR-free means "below the LiDAR"
    explain_p=0.7, lidar_free_known_p=0.35, known_free_share=0.5,
    # Fading toward unknown. The LiDAR re-observes all around ten times a second; the ultrasonic sees one narrow
    # cone, so a low obstacle is remembered while the robot turns to go around it.
    decay_high_s=4.0, decay_low_s=30.0,
    # Clamping as in OctoMap (0.12 / 0.97): a cell seen free many times still turns occupied after a few echoes
    # when a box is put there, and an occupied one clears again in reasonable time.
    clamp_free_p=0.12, clamp_occupied_p=0.97, occupied_p=0.7,
    # Switches for the comparison runs (scripts/fusion_eval.py)
    use_ultrasonic=True, associate=True,
    us_accumulate=True,         # False: each reading overwrites its cells (binary marking/clearing, no memory)
)

ECHO_CLASSES = ('lidar_obstacle', 'low_obstacle', 'unresolved', 'echo', 'far', 'free', 'unsure', 'off')


def logit(p: float) -> float:
    return math.log(p / (1.0 - p))


def _wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def _shifted(a: np.ndarray, di: int, dj: int) -> np.ndarray:
    """new[i, j] = a[i + di, j + dj], zero (unknown) where that falls outside."""
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
