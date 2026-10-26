import pytest

from vision_omx.yolo_detector import to_detections

NAMES = {0: 'strawberry', 1: 'car', 2: 'ball'}


def test_to_detections_center_and_name():
    [det] = to_detections([[100, 200, 300, 400]], [0.9], [2.0], NAMES)
    assert det.name == 'ball'
    assert det.class_id == 2
    assert det.center_uv == pytest.approx((200, 300))
    assert det.bbox_xyxy == pytest.approx((100, 200, 300, 400))


def test_to_detections_sorted_by_confidence():
    dets = to_detections([[0, 0, 10, 10], [0, 0, 20, 20]], [0.6, 0.95], [0, 1], NAMES)
    assert [d.name for d in dets] == ['car', 'strawberry']


def test_to_detections_empty():
    assert to_detections([], [], [], NAMES) == []
