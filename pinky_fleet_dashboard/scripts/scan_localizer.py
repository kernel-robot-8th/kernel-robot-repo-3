"""Find a Pinky's map pose from one LiDAR scan by matching it against the occupancy grid, without moving it.

Nav2 AMCL can also localize globally (/reinitialize_global_localization), but its particles only converge
while the robot drives; a Pinky at the start of the day stands still. So every free cell and heading of the
map is tried here: the scan's end points are placed on the map and scored on how close they land to walls
(a likelihood field, the same idea as AMCL's laser model). The best poses are refined on a finer grid and
the winner is handed to AMCL as its initial pose. When a pose far from the winner fits about as well (a
symmetric spot of the map), the answer is ambiguous and nothing is guessed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy import ndimage

SIGMA_M = 0.05            # how far from a wall an end point may land and still mostly count
MAX_RANGE_M = 4.0         # the cell is about 2.2 m x 2.6 m; farther returns see beyond the map
BEAMS = 180               # end points used per score (of 720)
COARSE_YAW_STEP = math.radians(3.0)
FINE_STEP_M = 0.01
FINE_YAW_STEP = math.radians(0.5)
FINE_SPAN_M = 0.05
FINE_SPAN_YAW = math.radians(4.0)
REFINE_TOP = 12
DISTINCT_M = 0.25         # a second candidate this far away (or turned this much) counts as another place
DISTINCT_YAW = math.radians(30.0)
MIN_SCORE = 0.45          # mean likelihood below this: the scan does not look like this map
AMBIGUOUS_RATIO = 0.92    # another place scoring this close to the best: refuse
PLACE_RADIUS_M = 0.15     # a candidate this close to a known place (a parking slot, station 5) stands there
PLACE_YAW_RAD = math.radians(35.0)  # and, where the place has a heading (parking: facing pi), turned at most this
CANDIDATE_RATIO = 0.8     # places scoring at least this share of the best are offered to the operator


@dataclass
class Match:
    ok: bool
    x_m: float | None
    y_m: float | None
    yaw_rad: float | None
    score: float
    runner_up: float
    reason: str
    # Distinct places that fit nearly as well as the best, best first: what an operator chooses from.
    candidates: tuple[tuple[float, float, float, float], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {"ok": self.ok, "x_m": self.x_m, "y_m": self.y_m, "yaw_rad": self.yaw_rad,
                "score": round(self.score, 3), "runner_up": round(self.runner_up, 3), "reason": self.reason,
                "candidates": [{"score": round(c[0], 3), "x_m": c[1], "y_m": c[2], "yaw_rad": c[3]} for c in self.candidates]}


class MapField:
    """The likelihood field of one occupancy grid: 1 on a wall, falling off with distance to the nearest wall."""

    def __init__(self, grid: dict[str, Any]):
        self.width, self.height = int(grid["width"]), int(grid["height"])
        self.res = float(grid["resolution_m"])
        self.origin = (float(grid["origin_x_m"]), float(grid["origin_y_m"]))
        if abs(float(grid.get("origin_yaw_rad") or 0.0)) > 1e-6:
            raise ValueError("회전된 지도 원점은 지원하지 않습니다")
        cells = np.asarray(grid["cells"], dtype=np.int16).reshape(self.height, self.width)  # row 0 = bottom
        occupied = cells >= 65
        if not occupied.any():
            raise ValueError("지도에 벽이 없습니다")
        distance = ndimage.distance_transform_edt(~occupied) * self.res
        self.field = np.exp(-(distance ** 2) / (2 * SIGMA_M ** 2)).astype(np.float32)
        self.free = cells == 0

    def free_cells(self) -> np.ndarray:
        """Centres of the free cells in map metres, shape (N, 2)."""
        rows, cols = np.nonzero(self.free)
        return np.column_stack([self.origin[0] + (cols + 0.5) * self.res, self.origin[1] + (rows + 0.5) * self.res])

    def score(self, points: np.ndarray, xs: np.ndarray, ys: np.ndarray, yaw: float) -> np.ndarray:
        """Mean likelihood of `points` (base frame, (B, 2)) for robots at (xs, ys) all facing `yaw`."""
        c, s = math.cos(yaw), math.sin(yaw)
        px = points[:, 0] * c - points[:, 1] * s
        py = points[:, 0] * s + points[:, 1] * c
        cols = np.floor((xs[:, None] + px[None, :] - self.origin[0]) / self.res).astype(np.int64)
        rows = np.floor((ys[:, None] + py[None, :] - self.origin[1]) / self.res).astype(np.int64)
        inside = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        values = np.zeros(cols.shape, dtype=np.float32)
        values[inside] = self.field[rows[inside], cols[inside]]
        return values.mean(axis=1)


def scan_points(scan: dict[str, Any], laser_pose: tuple[float, float, float], beams: int = BEAMS) -> np.ndarray:
    """End points of a LaserScan in the robot's base frame, evenly thinned to `beams`.

    `scan`: angle_min, angle_increment, range_min, range_max, ranges (None for no return).
    `laser_pose`: the laser frame's (x, y, yaw) in the base frame, from the robot's static TF."""
    ranges = np.array([np.nan if r is None else r for r in scan["ranges"]], dtype=np.float64)
    angles = float(scan["angle_min"]) + np.arange(len(ranges)) * float(scan["angle_increment"])
    upper = min(float(scan.get("range_max") or MAX_RANGE_M), MAX_RANGE_M)
    valid = np.isfinite(ranges) & (ranges > float(scan.get("range_min") or 0.0)) & (ranges < upper)
    ranges, angles = ranges[valid], angles[valid]
    if len(ranges) > beams:
        keep = np.linspace(0, len(ranges) - 1, beams).round().astype(int)
        ranges, angles = ranges[keep], angles[keep]
    lx, ly, lyaw = laser_pose
    return np.column_stack([lx + ranges * np.cos(angles + lyaw), ly + ranges * np.sin(angles + lyaw)])


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _at_place(candidate: tuple[float, float, float, float], places: list[tuple[float, float, float | None]]) -> bool:
    _, x, y, yaw = candidate
    return any(math.hypot(x - px, y - py) <= PLACE_RADIUS_M
               and (pyaw is None or abs(_wrap(yaw - pyaw)) <= PLACE_YAW_RAD) for px, py, pyaw in places)


def locate(grid: dict[str, Any], scan: dict[str, Any], laser_pose: tuple[float, float, float],
           known_places: list[tuple[float, float, float | None]] | None = None) -> Match:
    """The map pose that best explains `scan`, or why none can be trusted.

    The cell is a rectangle, so a pose and its half-turn about the centre often fit equally well. `known_places` are
    where a standing Pinky is expected: (x, y, heading or None). Any slot counts, not only the robot's own,
    since Pinkys get parked in each other's slots. Such a tie is settled when exactly one of the equally good
    candidates stands at one of them; otherwise the candidates go to the operator. The heading always comes
    from the scan."""
    points = scan_points(scan, laser_pose)
    if len(points) < 40:
        return Match(False, None, None, None, 0.0, 0.0, f"라이다 점이 {len(points)}개뿐이라 위치를 찾을 수 없습니다")
    field = MapField(grid)
    cells = field.free_cells()
    xs, ys = cells[:, 0], cells[:, 1]
    yaws = np.arange(-math.pi, math.pi, COARSE_YAW_STEP)
    coarse = np.stack([field.score(points, xs, ys, yaw) for yaw in yaws], axis=1)  # (cells, yaws)

    # Refine the best coarse candidates that are not next to one another.
    order = np.argsort(coarse, axis=None)[::-1]
    seeds: list[tuple[float, float, float]] = []
    for flat in order:
        i, j = np.unravel_index(flat, coarse.shape)
        x, y, yaw = float(xs[i]), float(ys[i]), float(yaws[j])
        if all(math.hypot(x - sx, y - sy) > 2 * field.res or abs(_wrap(yaw - syaw)) > 2 * COARSE_YAW_STEP
               for sx, sy, syaw in seeds):
            seeds.append((x, y, yaw))
        if len(seeds) >= REFINE_TOP:
            break
    offsets = np.arange(-FINE_SPAN_M, FINE_SPAN_M + 1e-9, FINE_STEP_M)
    dx, dy = (a.ravel() for a in np.meshgrid(offsets, offsets))
    refined: list[tuple[float, float, float, float]] = []
    for sx, sy, syaw in seeds:
        best = (-1.0, sx, sy, syaw)
        for yaw in syaw + np.arange(-FINE_SPAN_YAW, FINE_SPAN_YAW + 1e-9, FINE_YAW_STEP):
            scores = field.score(points, sx + dx, sy + dy, float(yaw))
            k = int(np.argmax(scores))
            if scores[k] > best[0]:
                best = (float(scores[k]), sx + float(dx[k]), sy + float(dy[k]), _wrap(float(yaw)))
        refined.append(best)
    refined.sort(reverse=True)
    places: list[tuple[float, float, float, float]] = []  # the best pose of each distinct place, best first
    for candidate in refined:
        _, cx, cy, cyaw = candidate
        if all(math.hypot(cx - px, cy - py) > DISTINCT_M or abs(_wrap(cyaw - pyaw)) > DISTINCT_YAW
               for _, px, py, pyaw in places):
            places.append(candidate)
    score, x, y, yaw = places[0]
    runner_up = places[1][0] if len(places) > 1 else 0.0
    offered = tuple(place for place in places[:3] if place[0] >= CANDIDATE_RATIO * score)
    if score < MIN_SCORE:
        return Match(False, x, y, yaw, score, runner_up,
                     f"라이다가 지도와 잘 맞지 않습니다 (일치도 {score:.2f} < {MIN_SCORE})", offered)
    contenders = [place for place in places if place[0] >= AMBIGUOUS_RATIO * score]
    if len(contenders) == 1:
        return Match(True, x, y, yaw, score, runner_up, "라이다로 위치를 찾았습니다", offered)
    near = [place for place in contenders if _at_place(place, known_places or [])]
    if len(near) == 1:
        chosen, x, y, yaw = near[0]
        rival = max(place[0] for place in contenders if place is not near[0])
        return Match(True, x, y, yaw, chosen, rival,
                     f"지도가 대칭이라 비슷한 자리 {len(contenders)}곳 중 주차 칸·5번처럼 서 있을 자리에 맞는 쪽을 골랐습니다", offered)
    return Match(False, x, y, yaw, score, runner_up,
                 f"지도가 대칭이라 비슷하게 맞는 자리가 {len(contenders)}곳입니다. 후보 중 실제 자리를 골라 주세요 "
                 f"({score:.2f} 대 {runner_up:.2f})", offered)
