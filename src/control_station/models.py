"""Shared state vocabulary for the control-station API and adapters."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum


class StringEnum(str, Enum):
    """String-valued enum compatible with Python 3.10 and newer."""


class TaskStatus(StringEnum):
    PENDING = "PENDING"
    VALIDATING = "VALIDATING"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELING = "CANCELING"
    CANCELED = "CANCELED"
    NEEDS_INTERVENTION = "NEEDS_INTERVENTION"


class StepStatus(StringEnum):
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
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    UNKNOWN = "UNKNOWN"


class CameraStreamState(StringEnum):
    STREAMING = "STREAMING"
    NO_FRAMES = "NO_FRAMES"
    STOPPED = "STOPPED"
    ERROR = "ERROR"
    UNKNOWN = "UNKNOWN"


class FrameFreshness(StringEnum):
    FRESH = "FRESH"
    STALE = "STALE"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class CameraSnapshot:
    id: str
    robot_id: str
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
    source_stamp: dict[str, int] | None = None
    source_frame_id: str | None = None
    depth_observation: dict[str, object] | None = None
    camera_info_observation: dict[str, object] | None = None
    point_cloud_observation: dict[str, object] | None = None
    tf_observation: dict[str, object] | None = None
    frame_rate: float | None = None
    error_message: str | None = None
    stream_url: str | None = None
    detector_state: str | None = None
    detector_model_type: str | None = None
    detector_prompts: list[str] | None = None
    detector_classes: list[str] | None = None
    detector_confidence_threshold: float | None = None
    detector_weights_sha256: str | None = None
    detector_inference_image_size: int | None = None
    detector_device: str | None = None
    detector_ultralytics_version: str | None = None
    detector_inference_frame_sequence: int | None = None
    detections: list[dict[str, object]] = field(default_factory=list)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)
