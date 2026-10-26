import numpy as np
import pytest

from vision_omx.calibration import Calibration, compute_homography, reprojection_errors
from vision_omx.geometry import build_pick_target, pixel_to_table, table_to_omx
from vision_omx.types import Failure, FailureCode, PickTarget

CONFIG = {
    'omx': {'frame_id': 'omx_base_link', 'origin_on_table_mm': [150, -60],
            'yaw_deg': 90, 'reach_mm': 250},
    'objects': {10: {'name': 'box', 'grasp_z_mm': 30}},
}

# 픽셀 2px = 1mm 인 단순한 카메라
IDENTITY_X2 = Calibration(np.diag([0.5, 0.5, 1.0]), 'test')


def test_pixel_to_table_scale():
    assert pixel_to_table(IDENTITY_X2.homography, (200, 100)) == pytest.approx((100, 50))


def test_homography_from_four_corners():
    table = {0: [0, 0], 1: [300, 0], 2: [300, 200], 3: [0, 200]}
    # 원근이 들어간 임의의 픽셀 위치
    pixels = {0: (100, 80), 1: (700, 100), 2: (680, 500), 3: (120, 470)}
    homography = compute_homography(pixels, table)
    for marker_id, uv in pixels.items():
        assert pixel_to_table(homography, uv) == pytest.approx(table[marker_id], abs=1e-6)


def test_homography_needs_four_points():
    assert compute_homography({0: (0, 0), 1: (1, 0), 2: (1, 1)}, {0: [0, 0], 1: [1, 0], 2: [1, 1]}) is None


def test_reprojection_errors_zero_on_exact_points():
    table = {0: [0, 0], 1: [300, 0], 2: [300, 200], 3: [0, 200]}
    pixels = {0: (100, 80), 1: (700, 100), 2: (680, 500), 3: (120, 470), 20: (400, 300)}
    errors = reprojection_errors(compute_homography(pixels, table), pixels, table)
    assert set(errors) == {0, 1, 2, 3}  # 기준 좌표가 없는 마커는 빠진다
    assert max(errors.values()) == pytest.approx(0, abs=1e-6)


def test_reprojection_errors_on_check_marker():
    # 보정에 안 쓴 검증 마커의 실측을 20mm 잘못 적으면 그만큼 오차로 나온다
    table = {0: [0, 0], 1: [300, 0], 2: [300, 200], 3: [0, 200]}
    pixels = {i: (2 * x, 2 * y) for i, (x, y) in table.items()}
    pixels[4] = (300, 200)  # 실제 위치 (150, 100)
    homography = compute_homography(pixels, table)
    assert reprojection_errors(homography, pixels, {4: [150, 100]})[4] == pytest.approx(0, abs=1e-6)
    assert reprojection_errors(homography, pixels, {4: [170, 100]})[4] == pytest.approx(20)


def test_table_to_omx_rotation():
    # yaw 90도: 작업대 +y 방향이 OMX 정면(+X)
    assert table_to_omx((150, 40), (150, -60), 90) == pytest.approx((100, 0))
    # 작업대 +x 방향은 OMX 오른쪽(-Y)
    assert table_to_omx((250, -60), (150, -60), 90) == pytest.approx((0, -100))


def test_build_pick_target_success():
    # 픽셀 (300, 80) → 작업대 (150, 40) → OMX (100, 0) mm
    target = build_pick_target(10, (300, 80), IDENTITY_X2, CONFIG)
    assert isinstance(target, PickTarget)
    assert (target.x, target.y, target.z) == pytest.approx((0.1, 0.0, 0.03))
    assert target.calibration_version == 'test'


@pytest.mark.parametrize('marker_id, calibration, pixel, code', [
    (99, IDENTITY_X2, (300, 80), FailureCode.UNKNOWN_OBJECT),
    (10, None, (300, 80), FailureCode.NO_CALIBRATION),
    (10, IDENTITY_X2, (300, 600), FailureCode.OUT_OF_WORKSPACE),  # OMX에서 360mm
])
def test_build_pick_target_failures(marker_id, calibration, pixel, code):
    result = build_pick_target(marker_id, pixel, calibration, CONFIG)
    assert isinstance(result, Failure)
    assert result.code == code
