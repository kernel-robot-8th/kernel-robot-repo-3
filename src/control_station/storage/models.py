"""관제 API와 장치 어댑터가 함께 쓰는 상태 어휘(enum)와 카메라 스냅샷.

작업·단계 상태 값은 DB(tasks.status, task_steps.status)와 API 응답에 문자열 그대로 들어간다. 어떤 상태에서 어떤
상태로 갈 수 있는지는 store.py의 TASK_TRANSITIONS / STEP_TRANSITIONS가 정한다.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum


class StringEnum(str, Enum):
    """Python 3.10 이상에서 쓰는 문자열 값 enum(StrEnum은 3.11부터라 str과 Enum을 직접 섞는다)."""


class TaskStatus(StringEnum):
    """작업(주문) 상태. SUCCEEDED·FAILED·CANCELED는 끝 상태라 더 바뀌지 않는다."""

    PENDING = "PENDING"  # 접수됨, 아직 시작 전
    VALIDATING = "VALIDATING"  # 시작함, 첫 단계(확인) 실행 중
    RUNNING = "RUNNING"  # 나머지 단계 실행 중
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"  # 멈춤. 자동 재시도는 없고 운영자가 경보를 보고 판단한다
    CANCELING = "CANCELING"  # 취소 요청을 받아 진행 중인 단계를 멈추는 중
    CANCELED = "CANCELED"
    # 사람이 봐야 하는 상태. 운영자 확인(resume_confirmed) 없이는 RUNNING으로 돌아가지 않는다.
    NEEDS_INTERVENTION = "NEEDS_INTERVENTION"


class StepStatus(StringEnum):
    """단계 상태. 보통 PENDING → COMMAND_SENT → ACCEPTED → RUNNING → 끝 상태로 간다.

    TIMED_OUT·UNKNOWN은 결과를 확정하지 못한 상태다. 시간 초과나 통신 단절만으로 성공·실패를 추정하지 않고,
    나중에 확인한 결과로 SUCCEEDED/FAILED/CANCELED로 닫는다."""

    PENDING = "PENDING"
    COMMAND_SENT = "COMMAND_SENT"
    ACCEPTED = "ACCEPTED"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    TIMED_OUT = "TIMED_OUT"
    CANCELED = "CANCELED"
    UNKNOWN = "UNKNOWN"


class CameraConnectionState(StringEnum):
    """카메라 장치가 열려 있는지."""

    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    UNKNOWN = "UNKNOWN"


class CameraStreamState(StringEnum):
    """프레임이 들어오고 있는지."""

    STREAMING = "STREAMING"
    NO_FRAMES = "NO_FRAMES"
    STOPPED = "STOPPED"
    ERROR = "ERROR"
    UNKNOWN = "UNKNOWN"


class FrameFreshness(StringEnum):
    """마지막 프레임이 최근 것인지(오래된 프레임으로 판단하지 않으려고 따로 본다)."""

    FRESH = "FRESH"
    STALE = "STALE"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class CameraSnapshot:
    """카메라 하나의 지금 상태. 카메라 탭과 /api 응답에 to_dict()로 그대로 나간다."""

    id: str
    robot_id: str | None  # 어느 로봇에도 붙지 않은 고정 환경 카메라(천장·선반 캠)는 None
    position: str
    connection_state: CameraConnectionState
    stream_state: CameraStreamState
    frame_freshness: FrameFreshness = FrameFreshness.UNKNOWN
    last_frame_at: str | None = None
    frame_sequence: int = 0
    frame_width: int | None = None
    frame_height: int | None = None
    frame_sha256: str | None = None
    pixel_transform_id: str | None = None
    frame_rate: float | None = None
    error_message: str | None = None
    stream_url: str | None = None
    # detector_*: 이 카메라 프레임에 붙은 객체 검출기(YOLO)의 상태와 설정. 검출기가 없으면 None.
    detector_state: str | None = None
    detector_model_type: str | None = None
    detector_prompts: list[str] | None = None
    detector_classes: list[str] | None = None
    detector_confidence_threshold: float | None = None
    detector_weights_sha256: str | None = None
    detector_inference_image_size: int | None = None
    detector_device: str | None = None
    detector_ultralytics_version: str | None = None
    detections: list[dict[str, object]] = field(default_factory=list)
    # YOLO가 그림을 그리기 전 원본 프레임에서 본 ArUco 마커(DICT_4X4_50): {id, center_px, corners_px}.
    markers: list[dict[str, object]] = field(default_factory=list)
    # 카메라의 쓰임을 말로(예: "왼쪽 팔 탑 캠 (ACT 정책)"), 장치를 여는 프로그램(control_station / policy /
    # ros_node), 사람이 볼 두 단어 상태("연결됨" 또는 "연결 전").
    role: str | None = None
    opened_by: str | None = None
    link_label: str | None = None
    setting: str | None = None  # 장치 경로를 정하는 환경 변수 이름

    def to_dict(self) -> dict[str, object]:
        return asdict(self)
