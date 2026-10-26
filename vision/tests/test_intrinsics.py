import numpy as np
import pytest

from vision_omx import calibration as calib

BOARD = (9, 6)
SQUARE_MM = 25
IMAGE_SIZE = (1280, 720)
TRUE_K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
TRUE_DIST = np.array([-0.25, 0.08, 0.0, 0.0, 0.0])  # 광각 웹캠 정도의 술통 왜곡


def synthetic_views():
    """알려진 K·왜곡으로 체크보드를 여러 자세에서 찍은 모서리 픽셀."""
    import cv2
    objp = calib.board_object_points(BOARD, SQUARE_MM)
    center = objp.mean(axis=0)
    views = []
    for rx, ry, tx, ty, tz in [(0, 0, 0, 0, 600), (0.3, 0, -150, -80, 650), (-0.3, 0.2, 150, 80, 650),
                               (0.2, -0.3, 200, -100, 700), (-0.2, 0.3, -200, 100, 700),
                               (0.4, 0.1, 0, 120, 600), (-0.1, -0.4, 0, -120, 600),
                               (0.1, 0.4, 250, 0, 750), (0.3, 0.3, -250, 0, 750),
                               (-0.35, -0.2, 100, 150, 680), (0.15, 0.35, -100, -150, 680)]:
        rvec = np.array([rx, ry, 0.05])
        tvec = np.array([tx, ty, tz]) - cv2.Rodrigues(rvec)[0] @ center
        pixels, _ = cv2.projectPoints(objp, rvec, tvec, TRUE_K, TRUE_DIST)
        views.append(pixels.reshape(-1, 2))
    return views


def test_compute_intrinsics_recovers_lens():
    camera_matrix, dist_coeffs, rms = calib.compute_intrinsics(synthetic_views(), BOARD, SQUARE_MM, IMAGE_SIZE)
    assert rms < 0.01
    assert camera_matrix == pytest.approx(TRUE_K, abs=1.0)
    assert dist_coeffs[:2] == pytest.approx(TRUE_DIST[:2], abs=0.01)
    assert dist_coeffs[4] == 0  # 기본은 k3 고정


def test_board_object_points_layout():
    objp = calib.board_object_points((3, 2), 25)
    assert objp.shape == (6, 3)
    assert objp[1].tolist() == [25, 0, 0]   # 가로 먼저
    assert objp[3].tolist() == [0, 25, 0]


def test_save_and_load_intrinsics_separate_from_homography(tmp_path):
    from datetime import datetime
    calib.save(np.eye(3), tmp_path, 'top_a', now=datetime(2026, 10, 24, 9, 0, 0))
    calib.save_intrinsics(TRUE_K, TRUE_DIST, IMAGE_SIZE, tmp_path, 'top_a', now=datetime(2026, 10, 24, 10, 0, 0))

    intrinsics = calib.load_latest_intrinsics(tmp_path, 'top_a')
    assert intrinsics.version == 'top_a_intrinsic_20261024_100000'
    assert intrinsics.image_size == IMAGE_SIZE
    assert np.allclose(intrinsics.camera_matrix, TRUE_K)
    # intrinsic 파일이 더 최신이어도 homography 쪽에 섞이지 않는다
    assert calib.load_latest(tmp_path, 'top_a').version == 'top_a_20261024_090000'
    assert calib.load_latest_intrinsics(tmp_path, 'top_b') is None


def test_homography_remembers_intrinsic_version(tmp_path):
    calib.save(np.eye(3), tmp_path, 'top_a', intrinsic_version='top_a_intrinsic_20261024_100000')
    assert calib.load_latest(tmp_path, 'top_a').intrinsic_version == 'top_a_intrinsic_20261024_100000'


def test_undistorter_straightens_and_checks_size():
    import cv2
    intrinsics = calib.Intrinsics(TRUE_K, TRUE_DIST, IMAGE_SIZE, 'test')
    undistort = calib.Undistorter(intrinsics)
    # 가장자리 점: 왜곡된 위치에 점을 찍으면 펴진 화면에서는 이상적인 위치에 나와야 한다
    ideal = np.array([[[1100.0, 600.0]]])
    normalized = cv2.undistortPoints(ideal, TRUE_K, None)
    distorted, _ = cv2.projectPoints(np.dstack([normalized, np.ones((1, 1))]).reshape(-1, 3),
                                     np.zeros(3), np.zeros(3), TRUE_K, TRUE_DIST)
    u, v = np.round(distorted.ravel()).astype(int)
    frame = np.zeros((IMAGE_SIZE[1], IMAGE_SIZE[0]), dtype=np.uint8)
    cv2.circle(frame, (int(u), int(v)), 4, 255, -1)

    out = undistort(frame)
    ys, xs = np.nonzero(out > 128)
    assert (xs.mean(), ys.mean()) == pytest.approx((1100, 600), abs=3)

    with pytest.raises(ValueError):
        undistort(np.zeros((480, 640), dtype=np.uint8))
