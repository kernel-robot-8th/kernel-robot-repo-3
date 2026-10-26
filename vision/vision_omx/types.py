"""PoC에서 주고받는 결과 타입."""
from dataclasses import dataclass
from enum import Enum


class FailureCode(str, Enum):
    NOT_FOUND = 'NOT_FOUND'                # 물체 마커가 화면에 없음
    UNKNOWN_OBJECT = 'UNKNOWN_OBJECT'      # 설정에 없는 마커 ID
    NO_CALIBRATION = 'NO_CALIBRATION'      # 보정(homography) 없음
    OUT_OF_WORKSPACE = 'OUT_OF_WORKSPACE'  # OMX 도달 범위 밖


@dataclass(frozen=True)
class PickTarget:
    """OMX 어댑터에 넘길 집기 목표. 좌표 단위는 m."""
    object_id: int
    object_name: str
    x: float
    y: float
    z: float
    frame_id: str
    calibration_version: str
    table_xy_mm: tuple


@dataclass(frozen=True)
class Detection:
    """검출기 공통 출력. 좌표 단위는 픽셀."""
    name: str
    class_id: int
    center_uv: tuple
    bbox_xyxy: tuple
    confidence: float


@dataclass(frozen=True)
class Failure:
    code: FailureCode
    detail: str = ''
