"""라이다 스캔 한 번을 점유 격자 지도와 맞춰, 핑키를 움직이지 않고 지도 위 자세를 찾는다.

실행 위치: PC 관제(app.py가 import).
사용처: app.py가 locate(위치 찾기)와 refine(운영자가 지도에서 끌어 준 자세 다듬기)을 쓰고, 결과를 AMCL의
  초기 자세로 보낸다. 지도 격자와 스캔은 관제가 받은 텔레메트리에서 온다. tests/test_localization.py가 검증한다.

Nav2 AMCL도 전역 위치 추정(/reinitialize_global_localization)을 할 수 있지만, 파티클은 로봇이 달리는 동안에만
수렴한다. 하루를 시작하는 핑키는 서 있다. 그래서 여기서는 지도의 모든 빈 칸과 방향을 시도한다: 스캔 끝점을 지도에
놓고 벽에 얼마나 가깝게 떨어지는지로 점수를 매긴다(likelihood field, AMCL 레이저 모델과 같은 생각). 가장 좋은
자세들을 더 촘촘한 격자로 다듬고, 1등을 AMCL에 초기 자세로 넘긴다. 1등과 멀리 떨어진 자세가 거의 같게 맞으면
(지도의 대칭인 자리) 답이 모호한 것이고, 추측하지 않는다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy import ndimage

SIGMA_M = 0.05            # 끝점이 벽에서 이만큼(m) 떨어져 떨어져도 대체로 인정된다 (가우시안 sigma)
MAX_RANGE_M = 4.0         # 셀(주행장)이 약 2.2 m x 2.6 m라 더 먼 반사는 지도 밖을 본 것이다
BEAMS = 180               # 점수 한 번에 쓰는 끝점 수 (720개 중)
# 거친 탐색: 빈 칸마다 3도 간격. 정밀 탐색: 후보 둘레 +-5 cm를 1 cm, +-4도를 0.5도 간격으로.
COARSE_YAW_STEP = math.radians(3.0)
FINE_STEP_M = 0.01
FINE_YAW_STEP = math.radians(0.5)
FINE_SPAN_M = 0.05
FINE_SPAN_YAW = math.radians(4.0)
REFINE_TOP = 12           # 정밀 탐색에 올리는 서로 떨어진 거친 후보 수
DISTINCT_M = 0.25         # 두 번째 후보가 이만큼 떨어져 있거나(또는 이만큼 돌아 있으면) 다른 자리로 센다
DISTINCT_YAW = math.radians(30.0)
MIN_SCORE = 0.45          # 평균 likelihood가 이보다 낮으면 스캔이 이 지도처럼 보이지 않는다
AMBIGUOUS_RATIO = 0.92    # 다른 자리가 1등에 이만큼 가깝게 맞으면 답하지 않는다
PLACE_RADIUS_M = 0.15     # 알려진 자리(주차 칸, 5번)에 이만큼 가까운 후보는 그 자리에 서 있는 것이다
PLACE_YAW_RAD = math.radians(35.0)  # 자리에 방향이 있으면(주차: pi를 봄) 최대 이만큼만 돌아 있어야 한다
CANDIDATE_RATIO = 0.8     # 1등 점수의 이 비율 이상인 자리는 운영자에게 후보로 보여 준다


@dataclass
class Match:
    """locate 결과. ok가 False면 reason이 이유를 말한다(x/y/yaw는 참고용 1등일 수 있다). score는 평균 likelihood."""
    ok: bool
    x_m: float | None
    y_m: float | None
    yaw_rad: float | None
    score: float
    runner_up: float
    reason: str
    # 1등과 거의 같게 맞는 서로 다른 자리들(좋은 순): 운영자가 고르는 목록. 각 항목은 (score, x, y, yaw).
    candidates: tuple[tuple[float, float, float, float], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {"ok": self.ok, "x_m": self.x_m, "y_m": self.y_m, "yaw_rad": self.yaw_rad,
                "score": round(self.score, 3), "runner_up": round(self.runner_up, 3), "reason": self.reason,
                "candidates": [{"score": round(c[0], 3), "x_m": c[1], "y_m": c[2], "yaw_rad": c[3]} for c in self.candidates]}


class MapField:
    """점유 격자 하나의 likelihood field: 벽 위에서 1, 가장 가까운 벽과의 거리에 따라 가우시안으로 줄어든다."""

    def __init__(self, grid: dict[str, Any]):
        self.width, self.height = int(grid["width"]), int(grid["height"])
        self.res = float(grid["resolution_m"])
        self.origin = (float(grid["origin_x_m"]), float(grid["origin_y_m"]))
        if abs(float(grid.get("origin_yaw_rad") or 0.0)) > 1e-6:
            raise ValueError("회전된 지도 원점은 지원하지 않습니다")
        cells = np.asarray(grid["cells"], dtype=np.int16).reshape(self.height, self.width)  # 0번 행 = 아래쪽
        # 점유 확률 65 이상을 벽으로 본다(map_server의 occupied_thresh 0.65와 같은 값). 0만 빈 칸이다
        occupied = cells >= 65
        if not occupied.any():
            raise ValueError("지도에 벽이 없습니다")
        distance = ndimage.distance_transform_edt(~occupied) * self.res
        self.field = np.exp(-(distance ** 2) / (2 * SIGMA_M ** 2)).astype(np.float32)
        self.free = cells == 0

    def free_cells(self) -> np.ndarray:
        """빈 칸들의 중심, 지도 좌표(m), 모양 (N, 2)."""
        rows, cols = np.nonzero(self.free)
        return np.column_stack([self.origin[0] + (cols + 0.5) * self.res, self.origin[1] + (rows + 0.5) * self.res])

    def score(self, points: np.ndarray, xs: np.ndarray, ys: np.ndarray, yaw: float) -> np.ndarray:
        """로봇이 (xs, ys)에 모두 `yaw` 방향으로 있을 때 `points`(base 좌표계, (B, 2))의 평균 likelihood.
        위치 후보 전체를 numpy로 한 번에 계산한다. 지도 밖으로 나간 점은 0점이다."""
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
    """LaserScan 끝점들을 로봇 base 좌표계로, `beams`개로 고르게 솎아 낸다.

    `scan`: angle_min, angle_increment, range_min, range_max, ranges(반사 없음은 None).
    `laser_pose`: base 좌표계에서 레이저 좌표계의 (x, y, yaw), 로봇의 static TF에서 온다."""
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


# 후보 (score, x, y, yaw)가 알려진 자리 중 하나에 서 있는지 (PLACE_RADIUS_M, PLACE_YAW_RAD 기준).
def _at_place(candidate: tuple[float, float, float, float], places: list[tuple[float, float, float | None]]) -> bool:
    _, x, y, yaw = candidate
    return any(math.hypot(x - px, y - py) <= PLACE_RADIUS_M
               and (pyaw is None or abs(_wrap(yaw - pyaw)) <= PLACE_YAW_RAD) for px, py, pyaw in places)


def locate(grid: dict[str, Any], scan: dict[str, Any], laser_pose: tuple[float, float, float],
           known_places: list[tuple[float, float, float | None]] | None = None) -> Match:
    """`scan`을 가장 잘 설명하는 지도 자세, 또는 왜 어느 것도 믿을 수 없는지.

    셀이 직사각형이라 어떤 자세와 그것을 중심으로 반 바퀴 돌린 자세가 똑같이 맞는 일이 잦다. `known_places`는
    서 있는 핑키가 있을 법한 자리다: (x, y, 방향 또는 None). 자기 칸만이 아니라 어느 칸이든 센다, 핑키를 서로의
    칸에 세우기도 하므로. 똑같이 좋은 후보 중 정확히 하나만 그런 자리에 서 있으면 동점을 그것으로 정하고, 아니면
    후보를 운영자에게 넘긴다. 방향은 언제나 스캔에서 온다."""
    points = scan_points(scan, laser_pose)
    # 끝점이 40개 미만이면(가까이 막혔거나 스캔이 비었음) 맞춰 볼 근거가 모자란다
    if len(points) < 40:
        return Match(False, None, None, None, 0.0, 0.0, f"라이다 점이 {len(points)}개뿐이라 위치를 찾을 수 없습니다")
    field = MapField(grid)
    cells = field.free_cells()
    xs, ys = cells[:, 0], cells[:, 1]
    yaws = np.arange(-math.pi, math.pi, COARSE_YAW_STEP)
    coarse = np.stack([field.score(points, xs, ys, yaw) for yaw in yaws], axis=1)  # (cells, yaws)

    # 서로 붙어 있지 않은(2칸·6도 넘게 떨어진) 거친 후보 중 가장 좋은 것들을 다듬는다.
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
    places: list[tuple[float, float, float, float]] = []  # 서로 다른 자리마다 가장 좋은 자세, 좋은 순
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


def refine(grid: dict[str, Any], scan: dict[str, Any], laser_pose: tuple[float, float, float],
           x: float, y: float, yaw: float, span_m: float = 0.30, span_yaw: float = math.radians(30.0)
           ) -> tuple[float, float, float, float]:
    """운영자가 준 자세 근처에서 가장 잘 맞는 자세 (score, x, y, yaw). 그 자세의 span_m, span_yaw 안에서만 찾으므로
    답이 대칭인 셀에서 운영자가 고른 쪽에 머문다. 3 cm·3도 간격으로 한 번, 1등 둘레 +-4 cm를 1 cm·0.5도로 다시
    찾는다."""
    points = scan_points(scan, laser_pose)
    if len(points) < 40:
        return 0.0, x, y, yaw
    field = MapField(grid)
    best = (-1.0, x, y, yaw)
    for step_m, step_yaw, span, yspan in ((0.03, math.radians(3.0), span_m, span_yaw),
                                          (0.01, math.radians(0.5), 0.04, math.radians(4.0))):
        offsets = np.arange(-span, span + 1e-9, step_m)
        dx, dy = (a.ravel() for a in np.meshgrid(offsets, offsets))
        cx, cy, cyaw = best[1], best[2], best[3]
        for candidate_yaw in cyaw + np.arange(-yspan, yspan + 1e-9, step_yaw):
            scores = field.score(points, cx + dx, cy + dy, float(candidate_yaw))
            k = int(np.argmax(scores))
            if scores[k] > best[0]:
                best = (float(scores[k]), cx + float(dx[k]), cy + float(dy[k]), _wrap(float(candidate_yaw)))
    return best
