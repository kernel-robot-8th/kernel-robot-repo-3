"""픽셀 → 작업대(mm) → OMX base(m) 좌표 변환.

OMX base 좌표계: 원점은 OMX 바닥 중심, +X는 OMX 정면, +Z는 위.
작업대 좌표계에서 OMX 정면은 x축으로부터 yaw_deg만큼 회전한 방향이다.
"""
import math

import cv2
import numpy as np

from .types import Failure, FailureCode, PickTarget


def pixel_to_table(homography, pixel_uv):
    point = np.array([[pixel_uv]], dtype=np.float64)
    x, y = cv2.perspectiveTransform(point, homography)[0][0]
    return float(x), float(y)


def table_to_omx(table_xy_mm, origin_mm, yaw_deg):
    """작업대 좌표(mm)를 OMX 평면 좌표(mm)로 바꾼다."""
    dx = table_xy_mm[0] - origin_mm[0]
    dy = table_xy_mm[1] - origin_mm[1]
    yaw = math.radians(yaw_deg)
    # 작업대 벡터를 OMX 축에 투영 (회전 -yaw)
    x = dx * math.cos(yaw) + dy * math.sin(yaw)
    y = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return x, y


def build_pick_target(object_id, pixel_uv, calibration, config):
    """물체 마커 픽셀 위치로 집기 목표를 만든다. 실패하면 Failure를 돌려준다."""
    objects = config['objects']
    if object_id not in objects:
        return Failure(FailureCode.UNKNOWN_OBJECT, f'marker {object_id}')
    if calibration is None:
        return Failure(FailureCode.NO_CALIBRATION)

    omx = config['omx']
    table_xy = pixel_to_table(calibration.homography, pixel_uv)
    x_mm, y_mm = table_to_omx(table_xy, omx['origin_on_table_mm'], omx['yaw_deg'])
    distance = math.hypot(x_mm, y_mm)
    if distance > omx['reach_mm']:
        return Failure(FailureCode.OUT_OF_WORKSPACE,
                       f'{distance:.0f} mm > {omx["reach_mm"]} mm')

    obj = objects[object_id]
    return PickTarget(
        object_id=object_id,
        object_name=obj['name'],
        x=x_mm / 1000.0,
        y=y_mm / 1000.0,
        z=obj['grasp_z_mm'] / 1000.0,
        frame_id=omx['frame_id'],
        calibration_version=calibration.version,
        table_xy_mm=table_xy,
    )
