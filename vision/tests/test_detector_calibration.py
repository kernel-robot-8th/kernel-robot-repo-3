from datetime import datetime

import numpy as np
import pytest
import yaml

from vision_omx import calibration as calib
from vision_omx.detector import MarkerDetector


def test_detects_rendered_markers():
    detector = MarkerDetector('DICT_4X4_50')
    canvas = np.full((600, 800), 255, dtype=np.uint8)
    canvas[100:200, 100:200] = detector.render(0, 100)
    canvas[300:400, 500:600] = detector.render(10, 100)

    centers, _, _ = detector.detect(canvas)

    assert set(centers) == {0, 10}
    assert centers[0] == pytest.approx((149.5, 149.5), abs=1.0)
    assert centers[10] == pytest.approx((549.5, 349.5), abs=1.0)


def test_save_and_load_latest(tmp_path):
    old = np.eye(3)
    new = np.diag([0.5, 0.5, 1.0])
    calib.save(old, tmp_path, 'top_a', now=datetime(2026, 10, 19, 9, 0, 0))
    _, path = calib.save(new, tmp_path, 'top_a', now=datetime(2026, 10, 19, 10, 0, 0))

    loaded = calib.load_latest(tmp_path, 'top_a')

    assert path.name == 'top_a_20261019_100000.yaml'
    assert loaded.version == 'top_a_20261019_100000'
    assert np.allclose(loaded.homography, new)
    assert len(list(tmp_path.glob('*.yaml'))) == 2  # 덮어쓰지 않음


def test_load_latest_per_camera(tmp_path):
    calib.save(np.eye(3), tmp_path, 'top_a', now=datetime(2026, 10, 19, 9, 0, 0))
    calib.save(np.diag([2.0, 2.0, 1.0]), tmp_path, 'top_b', now=datetime(2026, 10, 19, 10, 0, 0))
    calib.save(np.diag([3.0, 3.0, 1.0]), tmp_path, 'top', now=datetime(2026, 10, 19, 11, 0, 0))

    assert calib.load_latest(tmp_path, 'top_a').version == 'top_a_20261019_090000'
    assert calib.load_latest(tmp_path, 'top_b').version == 'top_b_20261019_100000'
    assert calib.load_latest(tmp_path, 'top').version == 'top_20261019_110000'  # top_a·top_b와 안 섞임
    assert calib.load_latest(tmp_path, 'top_c') is None


def test_save_records_extra(tmp_path):
    _, path = calib.save(np.eye(3), tmp_path, 'top_a', extra={'reference_ids': [0, 1, 2, 3]})
    assert yaml.safe_load(path.read_text())['reference_ids'] == [0, 1, 2, 3]


def test_load_latest_empty(tmp_path):
    assert calib.load_latest(tmp_path, 'top_a') is None
