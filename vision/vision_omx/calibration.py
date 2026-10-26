"""기준 마커 4개 이상으로 픽셀 → 바닥 평면(mm) homography를 만들고 카메라별로 저장한다."""
import math
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class Calibration:
    homography: np.ndarray   # 3x3, 픽셀 → 바닥 mm
    version: str             # 보정 파일 이름(확장자 제외). <카메라>_<YYYYMMDD_HHMMSS>
    intrinsic_version: str = ''  # 왜곡을 편 프레임으로 만들었으면 그 intrinsic 버전


def compute_homography(pixel_points, table_points_mm):
    """같은 ID의 픽셀 좌표와 바닥 좌표로 homography를 구한다. 4개 미만이면 None."""
    ids = sorted(set(pixel_points) & set(table_points_mm))
    if len(ids) < 4:
        return None
    src = np.array([pixel_points[i] for i in ids], dtype=np.float64)
    dst = np.array([table_points_mm[i] for i in ids], dtype=np.float64)
    homography, _ = cv2.findHomography(src, dst)
    return homography


def reprojection_errors(homography, pixel_points, table_points_mm):
    """{ID: 변환 결과와 실측 좌표의 거리(mm)}.

    보정에 쓴 기준 마커면 잔차(실제 오차보다 작게 나옴), 쓰지 않은 검증 마커면 실제 오차다.
    """
    ids = sorted(set(pixel_points) & set(table_points_mm))
    if not ids:
        return {}
    src = np.array([[pixel_points[i] for i in ids]], dtype=np.float64)
    projected = cv2.perspectiveTransform(src, homography)[0]
    return {i: math.dist(projected[k], table_points_mm[i]) for k, i in enumerate(ids)}


def save(homography, directory, camera_name, now=None, extra=None, intrinsic_version=''):
    """calib/<카메라>_<YYYYMMDD_HHMMSS>.yaml 로 새 파일을 만든다. 기존 파일은 덮어쓰지 않는다.

    extra는 기록용(사용한 마커, 잔차 등)이며 불러올 때는 쓰지 않는다.
    """
    now = now or datetime.now()
    version = f'{camera_name}_{now:%Y%m%d_%H%M%S}'
    path = Path(directory) / f'{version}.yaml'
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {'version': version, 'camera': camera_name, 'intrinsic_version': intrinsic_version,
            'homography': np.asarray(homography).tolist()}
    data.update(extra or {})
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return Calibration(np.asarray(homography), version, intrinsic_version), path


def load_latest(directory, camera_name):
    """그 카메라의 가장 최근 보정 파일을 읽는다. 없으면 None."""
    # top 과 top_a 처럼 접두어가 겹쳐도 섞이지 않게 이름 형식을 정확히 맞춘다
    pattern = re.compile(rf'{re.escape(camera_name)}_\d{{8}}_\d{{6}}\.yaml')
    files = sorted(p for p in Path(directory).glob('*.yaml') if pattern.fullmatch(p.name))
    if not files:
        return None
    data = yaml.safe_load(files[-1].read_text())
    return Calibration(np.array(data['homography'], dtype=np.float64), data['version'],
                       data.get('intrinsic_version', ''))


@dataclass(frozen=True)
class Intrinsics:
    camera_matrix: np.ndarray   # 3x3 K
    dist_coeffs: np.ndarray     # (k1, k2, p1, p2, k3)
    image_size: tuple           # (width, height). 다른 해상도에는 쓸 수 없다
    version: str                # <카메라>_intrinsic_<YYYYMMDD_HHMMSS>


def board_object_points(board_size, square_mm):
    """체크보드 안쪽 모서리의 보드 좌표(mm). board_size = (가로 안쪽 모서리 수, 세로 안쪽 모서리 수)."""
    cols, rows = board_size
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    return np.hstack([grid, np.zeros((len(grid), 1))]).astype(np.float32)


def compute_intrinsics(image_points_list, board_size, square_mm, image_size, fix_k3=True):
    """체크보드 사진 여러 장의 모서리 픽셀로 K와 왜곡 계수를 구한다. (K, dist, RMS 재투영 오차 px).

    화각이 좁은 렌즈에서 k3까지 풀면 찍지 않은 가장자리에서 값이 튀기 쉬워 기본은 k3 = 0으로 고정한다.
    """
    objp = board_object_points(board_size, square_mm)
    object_points = [objp] * len(image_points_list)
    image_points = [np.asarray(p, dtype=np.float32).reshape(-1, 1, 2) for p in image_points_list]
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points, image_points, tuple(image_size), None, None, flags=flags)
    return camera_matrix, dist_coeffs.ravel(), rms


def save_intrinsics(camera_matrix, dist_coeffs, image_size, directory, camera_name, now=None, extra=None):
    """calib/<카메라>_intrinsic_<YYYYMMDD_HHMMSS>.yaml 로 새 파일을 만든다."""
    now = now or datetime.now()
    version = f'{camera_name}_intrinsic_{now:%Y%m%d_%H%M%S}'
    path = Path(directory) / f'{version}.yaml'
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {'version': version, 'camera': camera_name,
            'image_size': [int(v) for v in image_size],
            'camera_matrix': np.asarray(camera_matrix).tolist(),
            'dist_coeffs': np.asarray(dist_coeffs).ravel().tolist()}
    data.update(extra or {})
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return Intrinsics(np.asarray(camera_matrix), np.asarray(dist_coeffs).ravel(),
                      tuple(int(v) for v in image_size), version), path


def load_latest_intrinsics(directory, camera_name):
    """그 카메라의 가장 최근 intrinsic 파일을 읽는다. 없으면 None."""
    pattern = re.compile(rf'{re.escape(camera_name)}_intrinsic_\d{{8}}_\d{{6}}\.yaml')
    files = sorted(p for p in Path(directory).glob('*.yaml') if pattern.fullmatch(p.name))
    if not files:
        return None
    data = yaml.safe_load(files[-1].read_text())
    return Intrinsics(np.array(data['camera_matrix'], dtype=np.float64),
                      np.array(data['dist_coeffs'], dtype=np.float64),
                      tuple(data['image_size']), data['version'])


class Undistorter:
    """프레임 왜곡 펴기. 맵을 한 번만 만들어 두고 매 프레임 remap 한다.

    펴진 프레임도 같은 K를 쓰므로 가장자리가 조금 잘린다.
    """

    def __init__(self, intrinsics):
        self.intrinsics = intrinsics
        self._maps = cv2.initUndistortRectifyMap(
            intrinsics.camera_matrix, intrinsics.dist_coeffs, None,
            intrinsics.camera_matrix, intrinsics.image_size, cv2.CV_16SC2)

    def __call__(self, frame):
        height, width = frame.shape[:2]
        if (width, height) != self.intrinsics.image_size:
            raise ValueError(f'intrinsic은 {self.intrinsics.image_size}용인데 프레임은 {(width, height)}')
        return cv2.remap(frame, *self._maps, cv2.INTER_LINEAR)
