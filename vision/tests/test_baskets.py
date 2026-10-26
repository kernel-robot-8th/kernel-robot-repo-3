from datetime import datetime

import pytest

from vision_omx.baskets import lateral_position, load_latest_baskets, locate, position_label, save_baskets

REGIONS = {
    'left': [[0, 0], [100, 0], [100, 100], [0, 100]],
    'right': [[100, 0], [200, 0], [200, 100], [100, 100]],
}


def test_locate_inside():
    assert locate(REGIONS, (50, 50)) == 'left'
    assert locate(REGIONS, (150, 20)) == 'right'


def test_locate_outside():
    assert locate(REGIONS, (300, 50)) is None


def test_locate_shared_edge_goes_to_first_registered():
    assert locate(REGIONS, (100, 50)) == 'left'


def test_save_and_load_latest(tmp_path):
    save_baskets({'old': REGIONS['left']}, tmp_path, 'top_a', now=datetime(2026, 10, 26, 9, 0, 0))
    _, path = save_baskets(REGIONS, tmp_path, 'top_a', now=datetime(2026, 10, 26, 10, 0, 0),
                           intrinsic_version='top_a_intrinsic_x')

    loaded = load_latest_baskets(tmp_path, 'top_a')

    assert path.name == 'top_a_baskets_20261026_100000.yaml'
    assert list(loaded.regions) == ['left', 'right']
    assert loaded.regions['right'] == REGIONS['right']
    assert loaded.intrinsic_version == 'top_a_intrinsic_x'


def test_load_latest_ignores_other_cameras_and_homography(tmp_path):
    save_baskets(REGIONS, tmp_path, 'top_ab', now=datetime(2026, 10, 26, 10, 0, 0))
    (tmp_path / 'top_a_20261026_110000.yaml').write_text('homography: []\n')
    assert load_latest_baskets(tmp_path, 'top_a') is None


def test_lateral_position_rectangle():
    assert lateral_position(REGIONS['left'], (0, 50)) == pytest.approx(0.0)
    assert lateral_position(REGIONS['left'], (50, 50)) == pytest.approx(0.5)
    assert lateral_position(REGIONS['left'], (100, 50)) == pytest.approx(1.0)


def test_lateral_position_uses_width_at_point_height():
    # 위는 0~100, 아래는 0~200으로 넓어지는 사다리꼴. v=100에서 폭은 0~150
    trapezoid = [[0, 0], [100, 0], [200, 200], [0, 200]]
    assert lateral_position(trapezoid, (75, 100)) == pytest.approx(0.5)


def test_lateral_position_clips_outside():
    assert lateral_position(REGIONS['left'], (-20, 50)) == 0.0
    assert lateral_position(REGIONS['left'], (150, 50)) == 1.0


def test_position_label():
    assert [position_label(f) for f in (0.1, 0.5, 0.9)] == ['left', 'center', 'right']
