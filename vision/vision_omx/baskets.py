"""상단캠 화면의 칸(바구니) 영역. 검출 결과가 어느 칸에 있는지 정한다.

칸 영역은 카메라 위치에 따라 바뀌는 보정값이라 homography처럼 calib/에 카메라별로 저장한다.
"""
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class Baskets:
    regions: dict            # {칸 이름: [[u, v], ...]} 픽셀 다각형, 등록한 순서
    version: str             # <카메라>_baskets_<YYYYMMDD_HHMMSS>
    intrinsic_version: str = ''  # 왜곡을 편 프레임에서 그렸으면 그 intrinsic 버전


def locate(regions, point_uv):
    """점이 들어 있는 칸 이름. 어느 칸에도 없으면 None. 겹치면 먼저 등록한 칸."""
    for name, polygon in regions.items():
        contour = np.asarray(polygon, dtype=np.float32).reshape(-1, 1, 2)
        if cv2.pointPolygonTest(contour, (float(point_uv[0]), float(point_uv[1])), False) >= 0:
            return name
    return None


def lateral_position(polygon, point_uv):
    """칸 안 좌우 위치. 점 높이(v)에서 칸의 왼쪽 끝 0.0, 오른쪽 끝 1.0. 칸 밖이면 0~1로 자른다.

    상단캠은 선반을 정면에서 봐서 좌우는 잘 보이고 앞뒤 깊이는 거의 안 보인다. 좌우만 구한다.
    """
    u, v = float(point_uv[0]), float(point_uv[1])
    points = np.asarray(polygon, dtype=np.float64)
    crossings = []
    for (u1, v1), (u2, v2) in zip(points, np.roll(points, -1, axis=0)):
        if min(v1, v2) <= v <= max(v1, v2) and v1 != v2:
            crossings.append(u1 + (v - v1) * (u2 - u1) / (v2 - v1))
    if len(crossings) < 2:
        # 꼭짓점 높이보다 위나 아래면 다각형 전체 폭으로 잰다
        crossings = [points[:, 0].min(), points[:, 0].max()]
    left, right = min(crossings), max(crossings)
    if right - left < 1e-6:
        return 0.5
    return float(np.clip((u - left) / (right - left), 0.0, 1.0))


def position_label(fraction):
    """0~1 좌우 위치를 left / center / right 세 자리로 나눈다."""
    if fraction < 1 / 3:
        return 'left'
    if fraction > 2 / 3:
        return 'right'
    return 'center'


def save_baskets(regions, directory, camera_name, now=None, intrinsic_version=''):
    """calib/<카메라>_baskets_<YYYYMMDD_HHMMSS>.yaml 로 새 파일을 만든다."""
    now = now or datetime.now()
    version = f'{camera_name}_baskets_{now:%Y%m%d_%H%M%S}'
    path = Path(directory) / f'{version}.yaml'
    path.parent.mkdir(parents=True, exist_ok=True)
    regions = {name: [[int(u), int(v)] for u, v in polygon] for name, polygon in regions.items()}
    data = {'version': version, 'camera': camera_name, 'intrinsic_version': intrinsic_version,
            'regions': regions}
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return Baskets(regions, version, intrinsic_version), path


def load_latest_baskets(directory, camera_name):
    """그 카메라의 가장 최근 칸 영역 파일을 읽는다. 없으면 None."""
    pattern = re.compile(rf'{re.escape(camera_name)}_baskets_\d{{8}}_\d{{6}}\.yaml')
    files = sorted(p for p in Path(directory).glob('*.yaml') if pattern.fullmatch(p.name))
    if not files:
        return None
    data = yaml.safe_load(files[-1].read_text())
    return Baskets(data['regions'], data['version'], data.get('intrinsic_version', ''))
