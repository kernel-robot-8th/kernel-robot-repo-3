"""Camera capture ownership and MJPEG frame access for the dashboard."""

from __future__ import annotations

import hashlib
from importlib.metadata import version as package_version
import math
import os
import re
import tempfile
import threading
import time
import uuid
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Protocol

from control_station.models import (
    CameraConnectionState,
    CameraSnapshot,
    CameraStreamState,
    FrameFreshness,
)


PIXEL_TRANSFORM_ID = "opencv-v4l2-capture-no-app-geometry-v1"
MUJOCO_ROS_PIXEL_TRANSFORM_ID = "mujoco-ros-rgb8-to-bgr8-no-app-geometry-v1"
MAX_OBSERVATION_IMAGE_BYTES = 64 * 1024 * 1024
MAX_RETAINED_DEPTH_BYTES = 16 * 1024 * 1024
MAX_DEPTH_SUMMARY_CANDIDATES = 32
MAX_DEPTH_SUMMARY_TOTAL_PIXELS = 8_000_000
MAX_SOURCE_STAMP_HISTORY = 64
DEPTH_REGION_SUMMARY_POLICY_VERSION = "exact-rgb-depth-stamp-bbox-raw-summary-v1"
COLOR_HINT_ALGORITHM_VERSION = "hsv-inner-box-thresholds-v1"
YOLO_WORLD_REQUEST_PROMPT_POLICY_VERSION = "yolo-world-request-class-one-shot-v1"


def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _estimate_color_hint(image, bbox_xyxy):  # type: ignore[no-untyped-def]
    """Return an uncalibrated HSV color hint for the inner part of one detection box."""
    try:
        import cv2
        import numpy as np

        x1, y1, x2, y2 = (float(value) for value in bbox_xyxy)
        height, width = image.shape[:2]
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
            raise ValueError("non-finite bbox")
        left = max(0, min(width, math.floor(x1)))
        top = max(0, min(height, math.floor(y1)))
        right = max(0, min(width, math.ceil(x2)))
        bottom = max(0, min(height, math.ceil(y2)))
        box_width, box_height = right - left, bottom - top
        inset_x, inset_y = int(box_width * 0.2), int(box_height * 0.2)
        crop = image[top + inset_y : bottom - inset_y, left + inset_x : right - inset_x]
        if crop.size == 0:
            raise ValueError("bbox crop is empty")
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hue, saturation, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        colored = saturation >= 65
        masks = {
            "black": value <= 45,
            "white": (saturation <= 45) & (value >= 200),
            "red": colored & ((hue <= 10) | (hue >= 170)) & (value >= 50),
            "yellow": colored & (hue >= 16) & (hue <= 37) & (value >= 50),
            "green": colored & (hue >= 38) & (hue <= 85) & (value >= 50),
            "blue": colored & (hue >= 90) & (hue <= 130) & (value >= 50),
        }
        total = int(hue.size)
        counts = {name: int(np.count_nonzero(mask)) for name, mask in masks.items()}
        ranked = sorted(counts.items(), key=lambda item: (-item[1], item[0]))
        color_name, color_pixels = ranked[0]
        fraction = color_pixels / total
        if fraction < 0.2:
            status, estimate = "LOW_COLOR_COVERAGE", None
        elif len(ranked) > 1 and (color_pixels - ranked[1][1]) / total < 0.05:
            status, estimate = "MIXED_COLOR", None
        else:
            status, estimate = "ESTIMATED", color_name
        return {
            "method": "UNCALIBRATED_HSV_PIXEL_FRACTION",
            "algorithm_version": COLOR_HINT_ALGORITHM_VERSION,
            "status": status,
            "color": estimate,
            "pixel_fraction": round(fraction, 3),
            "sampled_pixel_count": total,
            "review_required": True,
            "used_for_selection": False,
        }
    except Exception:
        return {
            "method": "UNCALIBRATED_HSV_PIXEL_FRACTION",
            "algorithm_version": COLOR_HINT_ALGORITHM_VERSION,
            "status": "UNAVAILABLE",
            "color": None,
            "pixel_fraction": None,
            "sampled_pixel_count": 0,
            "review_required": True,
            "used_for_selection": False,
        }


class CameraStreamManager(Protocol):
    def snapshots(self) -> list[CameraSnapshot]: ...

    def detect_for_prompt(self, camera: CameraSnapshot, prompt: str) -> CameraSnapshot: ...

    def frame_for(self, camera_id: str) -> bytes | None: ...

    def save_observation_frame(
        self, camera_id: str, frame_sequence: int, frame_sha256: str, task_id: str
    ) -> dict[str, object]: ...

    def delete_observation_frame(self, relative_path: str) -> None: ...

    def close(self) -> None: ...


@dataclass
class _CameraFeed:
    camera_id: str
    position: str
    device: str | None
    frame: bytes | None = None
    frame_width: int | None = None
    frame_height: int | None = None
    frame_sha256: str | None = None
    raw_frames: dict[int, tuple[str, bytes, int, int, str, float]] = field(default_factory=dict)
    connection_state: CameraConnectionState = CameraConnectionState.UNKNOWN
    stream_state: CameraStreamState = CameraStreamState.UNKNOWN
    last_frame_at: str | None = None
    last_frame_monotonic: float | None = None
    frame_times: list[float] = field(default_factory=list)
    error_message: str | None = None
    sequence: int = 0
    detector: "_YoloDetector | None" = None
    detections: list[dict[str, object]] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)
    thread: threading.Thread | None = None


class _YoloDetector:
    """Draw YOLO detections and compute advisory color hints for camera frames.

    Results are shown on the dashboard and are not used as a robot-control signal.
    """

    def __init__(self, weights: str, env_prefix: str) -> None:
        self.weights = weights
        self.detector_model_type = "YOLO_CLOSED_SET"
        self.prompts: list[str] = []
        raw_confidence = os.environ.get(f"{env_prefix}_CONF", "0.25").strip()
        raw_prompts = os.environ.get(f"{env_prefix}_WORLD_PROMPTS")
        self.confidence = 0.25
        self.device = os.environ.get(f"{env_prefix}_DEVICE", "").strip() or None
        self.state = "loading"
        self.error_message: str | None = None
        self._model = None
        self._inference_lock = threading.RLock()
        self.class_names: list[str] = []
        self.weights_sha256: str | None = None
        self.confidence_threshold: float | None = None
        self.inference_image_size = 640
        self.ultralytics_version: str | None = None
        if raw_prompts is not None and raw_prompts.strip():
            self.detector_model_type = "YOLO_WORLD_OPEN_VOCAB"
            prompts = [prompt.strip() for prompt in raw_prompts.split(",")]
            if (
                not 1 <= len(prompts) <= 100
                or any(not prompt or len(prompt) > 80 for prompt in prompts)
                or len({prompt.casefold() for prompt in prompts}) != len(prompts)
            ):
                self.state = "error"
                self.error_message = (
                    f"{env_prefix}_WORLD_PROMPTS는 중복 없이 "
                    "1–100개의 1–80자 프롬프트를 쉼표로 구분해야 합니다"
                )
            else:
                self.prompts = prompts
        try:
            confidence = float(raw_confidence)
        except ValueError:
            confidence = math.nan
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            self.state = "error"
            self.error_message = (
                f"{env_prefix}_CONF는 0에서 1 사이의 유한한 숫자여야 합니다"
            )
        else:
            self.confidence = confidence
            self.confidence_threshold = confidence

    def load(self) -> bool:
        if self.state == "error":
            return False
        if not os.path.isfile(self.weights):
            model_name = "YOLO-World" if self.prompts else "YOLO"
            self.state, self.error_message = "error", f"{model_name} 가중치 파일이 없습니다: {self.weights}"
            return False
        try:
            if self.prompts:
                from ultralytics import YOLOWorld
            else:
                from ultralytics import YOLO
            ultralytics_version = package_version("ultralytics")
        except ImportError:
            self.state = "error"
            self.error_message = "ultralytics 미설치: requirements-vision.txt를 설치하세요"
            return False
        except Exception as exc:
            self.state = "error"
            self.error_message = f"ultralytics 초기화 실패: {exc}"
            return False
        try:
            initial_hash = _sha256_file(self.weights)
            model = YOLOWorld(self.weights) if self.prompts else YOLO(self.weights)
            if self.prompts:
                model.set_classes(self.prompts)
            loaded_hash = _sha256_file(self.weights)
            if initial_hash != loaded_hash:
                raise ValueError("YOLO 가중치 파일이 로드 중 변경됐습니다")
            if self.prompts:
                class_names = list(self.prompts)
            else:
                names = model.names
                if isinstance(names, dict):
                    class_names = [str(names[index]) for index in sorted(names)]
                else:
                    class_names = [str(name) for name in names]
        except Exception as exc:
            self._model = None
            self.class_names = []
            self.state = "error"
            self.error_message = f"YOLO 모델 로드 실패: {exc}"
            return False
        self._model = model
        self.class_names = class_names
        self.weights_sha256 = loaded_hash
        self.ultralytics_version = ultralytics_version
        self.state = "ready"
        return True

    def _predict(self, image, *, render: bool = True):  # type: ignore[no-untyped-def]
        result = self._model.predict(
            source=image,
            conf=self.confidence,
            imgsz=self.inference_image_size,
            device=self.device,
            verbose=False,
        )[0]
        height, width = result.orig_shape
        detections = [
            {
                "class_name": result.names[int(box.cls.item())],
                "confidence": round(float(box.conf.item()), 3),
                "center_px": [round(value, 1) for value in box.xywh[0][:2].tolist()],
                "bbox_xyxy_px": [round(value, 1) for value in box.xyxy[0].tolist()],
                "color_hint": _estimate_color_hint(image, box.xyxy[0].tolist()),
                "frame_width": int(width),
                "frame_height": int(height),
            }
            for box in result.boxes
        ]
        return (result.plot() if render else image), detections

    def annotate(self, image):  # type: ignore[no-untyped-def]
        with self._inference_lock:
            if self.state != "ready" or self._model is None:
                raise RuntimeError("YOLO detector is no longer ready")
            return self._predict(image)

    def detect(self, image) -> list[dict[str, object]]:  # type: ignore[no-untyped-def]
        with self._inference_lock:
            if self.state != "ready" or self._model is None:
                raise RuntimeError("YOLO detector is no longer ready")
            _, detections = self._predict(image, render=False)
            return detections

    def detect_for_prompt(self, image, prompt: str) -> list[dict[str, object]]:  # type: ignore[no-untyped-def]
        if self.detector_model_type != "YOLO_WORLD_OPEN_VOCAB" or not self.prompts:
            raise RuntimeError("요청별 prompt는 YOLO-World detector에서만 지원합니다")
        if (
            not isinstance(prompt, str)
            or not prompt.strip()
            or len(prompt.strip()) > 80
            or any(ord(character) < 32 or ord(character) == 127 for character in prompt)
        ):
            raise ValueError("요청 prompt는 제어 문자가 없는 1–80자 문자열이어야 합니다")
        if self.state != "ready" or self._model is None:
            raise RuntimeError("YOLO-World detector가 준비되지 않았습니다")

        normalized_prompt = prompt.strip()
        with self._inference_lock:
            if self.state != "ready" or self._model is None:
                raise RuntimeError("YOLO-World detector is no longer ready")
            try:
                self._model.set_classes([normalized_prompt])
                _, detections = self._predict(image, render=False)
                return detections
            finally:
                try:
                    self._model.set_classes(self.prompts)
                except Exception as exc:
                    self.state = "error"
                    self.error_message = f"YOLO-World 기본 prompt 복원 실패: {exc}"
                    raise RuntimeError(self.error_message) from exc


class UnconfiguredArmCameraStreamManager:
    """Report camera slots without opening any physical device."""

    def snapshots(self) -> list[CameraSnapshot]:
        return [
            CameraSnapshot(
                id="head_camera",
                robot_id="arm-01",
                position="HEAD",
                connection_state=CameraConnectionState.UNKNOWN,
                stream_state=CameraStreamState.UNKNOWN,
                error_message="Head Camera 장치 경로 미설정",
            ),
            CameraSnapshot(
                id="joint_camera",
                robot_id="arm-01",
                position="JOINT",
                connection_state=CameraConnectionState.UNKNOWN,
                stream_state=CameraStreamState.UNKNOWN,
                error_message="팔 카메라 스트림을 사용하려면 --arm 모드로 실행하세요",
            ),
        ]

    def frame_for(self, camera_id: str) -> bytes | None:
        return None

    def detect_for_prompt(self, camera: CameraSnapshot, prompt: str) -> CameraSnapshot:
        raise RuntimeError("camera streams and detectors are not configured")

    def save_observation_frame(
        self, camera_id: str, frame_sequence: int, frame_sha256: str, task_id: str
    ) -> dict[str, object]:
        raise ValueError("camera stream manager has no captured frame to save")

    def delete_observation_frame(self, relative_path: str) -> None:
        return

    def close(self) -> None:
        return


class OpenCVArmCameraStreamManager:
    """Own configured V4L2 devices once and expose their latest JPEG frames."""

    def __init__(self, joint_enabled: bool = True) -> None:
        joint_device = os.environ.get(
            "CONTROL_STATION_JOINT_CAMERA",
            os.environ.get(
                "CONTROL_STATION_ARM_CAMERA",
                "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0",
            ),
        ).strip()
        head_device = os.environ.get(
            "CONTROL_STATION_HEAD_CAMERA",
            "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0",
        ).strip()
        head_weights = os.environ.get("CONTROL_STATION_HEAD_YOLO_WEIGHTS", "").strip()
        head_prompts = os.environ.get("CONTROL_STATION_HEAD_YOLO_WORLD_PROMPTS", "").strip()
        joint_weights = os.environ.get("CONTROL_STATION_JOINT_YOLO_WEIGHTS", "").strip()
        joint_prompts = os.environ.get("CONTROL_STATION_JOINT_YOLO_WORLD_PROMPTS", "").strip()
        joint_feed = _CameraFeed(
            "joint_camera", "JOINT", (joint_device or None) if joint_enabled else None
        )
        if not joint_enabled:
            joint_feed.error_message = "팔 카메라 스트림을 사용하려면 --arm 모드로 실행하세요"
        elif joint_weights or joint_prompts:
            joint_feed.detector = _YoloDetector(
                joint_weights, "CONTROL_STATION_JOINT_YOLO"
            )
        self._feeds = {
            "head_camera": _CameraFeed(
                "head_camera",
                "HEAD",
                head_device or None,
                detector=(
                    _YoloDetector(head_weights, "CONTROL_STATION_HEAD_YOLO")
                    if head_weights or head_prompts
                    else None
                ),
            ),
            "joint_camera": joint_feed,
        }
        head_feed = self._feeds["head_camera"]
        if joint_feed.device and head_feed.device:
            if os.path.realpath(joint_feed.device) == os.path.realpath(head_feed.device):
                message = (
                    "Head Camera와 Joint Camera 경로가 같은 V4L2 장치로 연결됩니다. "
                    "실제 장착을 확인한 뒤 CONTROL_STATION_HEAD_CAMERA와 "
                    "CONTROL_STATION_JOINT_CAMERA를 서로 다른 장치로 설정하세요."
                )
                for feed in (head_feed, joint_feed):
                    feed.device = None
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = message
        self._closed = threading.Event()
        for feed in self._feeds.values():
            if feed.device:
                feed.thread = threading.Thread(
                    target=self._capture_loop,
                    args=(feed,),
                    name=f"camera-{feed.camera_id}",
                    daemon=True,
                )
                try:
                    feed.thread.start()
                except Exception as exc:
                    with feed.lock:
                        feed.connection_state = CameraConnectionState.DISCONNECTED
                        feed.stream_state = CameraStreamState.ERROR
                        feed.error_message = f"카메라 worker 시작 실패: {exc}"
                    feed.thread = None

    @staticmethod
    def _release_capture(capture) -> str | None:  # type: ignore[no-untyped-def]
        try:
            capture.release()
        except Exception as exc:
            return f"카메라 자원 해제 실패: {exc}"
        return None

    def _capture_loop(self, feed: _CameraFeed) -> None:
        try:
            import cv2
        except ImportError:
            with feed.lock:
                feed.connection_state = CameraConnectionState.DISCONNECTED
                feed.stream_state = CameraStreamState.ERROR
                feed.error_message = "OpenCV 미설치: 관제 서버 가상환경에서 requirements.txt를 설치하세요"
            return

        if feed.detector is not None:
            feed.detector.load()

        while not self._closed.is_set():
            device = feed.device
            if not device:
                return
            resolved_device = os.path.realpath(device)
            video_match = re.fullmatch(r"/dev/video(\d+)", resolved_device)
            if not os.path.exists(resolved_device):
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = f"카메라 장치 경로가 없습니다: {device}"
                self._closed.wait(2)
                continue

            # OpenCV's V4L2 backend expects a camera index, not a device-name string.
            source: int | str = int(video_match.group(1)) if video_match else device
            capture = None
            try:
                capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
                opened = capture.isOpened()
            except Exception as exc:
                release_error = self._release_capture(capture) if capture is not None else None
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = f"카메라 열기 실패: {exc}"
                    if release_error:
                        feed.error_message += f" · {release_error}"
                self._closed.wait(2)
                continue
            if not opened:
                release_error = self._release_capture(capture)
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = f"카메라를 열 수 없습니다: {feed.device} (다른 프로그램 점유 여부 확인)"
                    if release_error:
                        feed.error_message += f" · {release_error}"
                self._closed.wait(2)
                continue

            try:
                # Both physical cameras share a USB 2.0 hub. Request compressed
                # MJPEG explicitly so simultaneous streams do not reserve the
                # bus bandwidth required by uncompressed YUYV.
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                capture.set(cv2.CAP_PROP_FPS, 30)
            except Exception as exc:
                release_error = self._release_capture(capture)
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = f"카메라 형식 설정 실패: {exc}"
                    if release_error:
                        feed.error_message += f" · {release_error}"
                self._closed.wait(2)
                continue
            with feed.lock:
                feed.connection_state = CameraConnectionState.CONNECTED
                feed.stream_state = CameraStreamState.NO_FRAMES
                feed.error_message = None

            failed_reads = 0
            capture_failure: str | None = None
            try:
                while not self._closed.is_set():
                    ok, image = capture.read()
                    if not ok:
                        failed_reads += 1
                        if failed_reads >= 10:
                            break
                        self._closed.wait(0.05)
                        continue
                    if image is None or image.ndim != 3 or image.shape[2] != 3:
                        with feed.lock:
                            feed.error_message = "카메라 프레임이 지원되는 BGR8 형식이 아닙니다"
                        self._closed.wait(0.05)
                        continue
                    failed_reads = 0
                    captured_at = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                    captured_monotonic = time.monotonic()
                    raw_height, raw_width = image.shape[:2]
                    raw_pixels = image.tobytes(order="C")
                    frame_sha256 = hashlib.sha256(raw_pixels).hexdigest()
                    detections: list[dict[str, object]] = []
                    detector = feed.detector
                    if detector is not None and detector.state == "ready":
                        try:
                            image, detections = detector.annotate(image)
                        except Exception as exc:  # keep streaming raw frames if inference fails
                            detector.state, detector.error_message = "error", f"YOLO 추론 실패: {exc}"
                    encoded, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not encoded:
                        continue
                    frame_height, frame_width = image.shape[:2]
                    with feed.lock:
                        feed.sequence += 1
                        frame_sequence = feed.sequence
                        feed.frame_width = int(frame_width)
                        feed.frame_height = int(frame_height)
                        feed.frame_sha256 = frame_sha256
                        feed.raw_frames[frame_sequence] = (
                            frame_sha256,
                            raw_pixels,
                            int(raw_width),
                            int(raw_height),
                            captured_at,
                            captured_monotonic,
                        )
                        while len(feed.raw_frames) > 5:
                            del feed.raw_frames[min(feed.raw_frames)]
                        for detection in detections:
                            detection.update(
                                {
                                    "camera_id": feed.camera_id,
                                    "captured_at": captured_at,
                                    "frame_sequence": frame_sequence,
                                    "frame_sha256": frame_sha256,
                                    "pixel_transform_id": PIXEL_TRANSFORM_ID,
                                }
                            )
                        feed.frame = jpeg.tobytes()
                        feed.detections = detections
                        feed.last_frame_at = captured_at
                        feed.last_frame_monotonic = captured_monotonic
                        feed.frame_times.append(captured_monotonic)
                        feed.frame_times = [
                            stamp for stamp in feed.frame_times if captured_monotonic - stamp <= 2
                        ]
                        feed.connection_state = CameraConnectionState.CONNECTED
                        feed.stream_state = CameraStreamState.STREAMING
                        feed.error_message = None
            except Exception as exc:
                capture_failure = f"카메라 프레임 처리 실패: {exc}"
            finally:
                release_error = self._release_capture(capture)
                capture_failure = capture_failure or release_error
            if not self._closed.is_set():
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = (
                        capture_failure or f"카메라 프레임 수신 중단: {feed.device}"
                    )
                self._closed.wait(2)

    def snapshots(self) -> list[CameraSnapshot]:
        result = []
        for feed in self._feeds.values():
            with feed.lock:
                configured = feed.device is not None
                freshness = FrameFreshness.UNKNOWN
                if feed.last_frame_monotonic is not None:
                    freshness = (
                        FrameFreshness.FRESH
                        if time.monotonic() - feed.last_frame_monotonic <= 2
                        else FrameFreshness.STALE
                    )
                frame_rate = None
                if len(feed.frame_times) >= 2:
                    elapsed = feed.frame_times[-1] - feed.frame_times[0]
                    if elapsed > 0:
                        frame_rate = round((len(feed.frame_times) - 1) / elapsed, 1)
                result.append(
                    CameraSnapshot(
                        id=feed.camera_id,
                        robot_id="arm-01",
                        position=feed.position,
                        connection_state=(
                            feed.connection_state if configured else CameraConnectionState.UNKNOWN
                        ),
                        stream_state=feed.stream_state if configured else CameraStreamState.UNKNOWN,
                        frame_freshness=freshness,
                        last_frame_at=feed.last_frame_at,
                        frame_sequence=feed.sequence,
                        frame_width=feed.frame_width,
                        frame_height=feed.frame_height,
                        frame_sha256=feed.frame_sha256,
                        pixel_transform_id=PIXEL_TRANSFORM_ID if configured else None,
                        frame_rate=frame_rate,
                        error_message=(
                            feed.error_message
                            or (feed.detector.error_message if feed.detector else None)
                            or (None if configured else "카메라 장치 경로 미설정")
                        ),
                        detector_state=feed.detector.state if feed.detector else None,
                        detector_model_type=(
                            feed.detector.detector_model_type if feed.detector else None
                        ),
                        detector_prompts=(
                            list(feed.detector.prompts) if feed.detector else None
                        ),
                        detector_classes=(
                            list(feed.detector.class_names)
                            if feed.detector and feed.detector.state == "ready"
                            else None
                        ),
                        detector_confidence_threshold=(
                            feed.detector.confidence_threshold if feed.detector else None
                        ),
                        detector_weights_sha256=(
                            feed.detector.weights_sha256 if feed.detector else None
                        ),
                        detector_inference_image_size=(
                            feed.detector.inference_image_size if feed.detector else None
                        ),
                        detector_device=(feed.detector.device if feed.detector else None),
                        detector_ultralytics_version=(
                            feed.detector.ultralytics_version if feed.detector else None
                        ),
                        detections=list(feed.detections),
                        stream_url=(
                            f"/streams/{feed.camera_id}.mjpg"
                            if configured and feed.frame is not None
                            else None
                        ),
                    )
                )
        return result

    def frame_for(self, camera_id: str) -> bytes | None:
        feed = self._feeds.get(camera_id)
        if feed is None:
            return None
        with feed.lock:
            return feed.frame

    def detect_for_prompt(self, camera: CameraSnapshot, prompt: str) -> CameraSnapshot:
        camera_id = camera.id
        feed = self._feeds.get(camera_id)
        if feed is None:
            raise ValueError("camera stream not found")
        detector = feed.detector
        if detector is None or detector.detector_model_type != "YOLO_WORLD_OPEN_VOCAB":
            raise RuntimeError("camera has no YOLO-World detector")
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("NumPy is required for request-prompt inference") from exc

        if (
            isinstance(camera.frame_sequence, bool)
            or not isinstance(camera.frame_sequence, int)
            or camera.frame_sequence <= 0
            or not isinstance(camera.frame_sha256, str)
        ):
            raise ValueError("requested observation has no frame sequence/hash")
        with feed.lock:
            captured = feed.raw_frames.get(camera.frame_sequence)
            if captured is None or captured[0] != camera.frame_sha256:
                raise RuntimeError("requested observation frame is no longer retained")
            (
                frame_sha256,
                raw_pixels,
                frame_width,
                frame_height,
                captured_at,
                captured_monotonic,
            ) = captured
        if (
            frame_sha256 != camera.frame_sha256
            or captured_at != camera.last_frame_at
            or frame_width != camera.frame_width
            or frame_height != camera.frame_height
        ):
            raise RuntimeError("requested observation frame provenance changed")
        image = np.frombuffer(raw_pixels, dtype=np.uint8).reshape(
            (frame_height, frame_width, 3)
        ).copy()
        detections = detector.detect_for_prompt(image, prompt)
        frame_freshness = (
            FrameFreshness.FRESH
            if time.monotonic() - captured_monotonic <= 2
            else FrameFreshness.STALE
        )
        for detection in detections:
            detection.update(
                {
                    "camera_id": camera_id,
                    "captured_at": captured_at,
                    "frame_sequence": camera.frame_sequence,
                    "frame_sha256": frame_sha256,
                    "pixel_transform_id": camera.pixel_transform_id or PIXEL_TRANSFORM_ID,
                }
            )
        return replace(
            camera,
            connection_state=camera.connection_state,
            stream_state=camera.stream_state,
            frame_freshness=frame_freshness,
            last_frame_at=captured_at,
            frame_sequence=camera.frame_sequence,
            frame_width=frame_width,
            frame_height=frame_height,
            frame_sha256=frame_sha256,
            pixel_transform_id=camera.pixel_transform_id or PIXEL_TRANSFORM_ID,
            detector_state=detector.state,
            detector_classes=[prompt.strip()],
            detections=detections,
        )

    def save_observation_frame(
        self, camera_id: str, frame_sequence: int, frame_sha256: str, task_id: str
    ) -> dict[str, object]:
        if not re.fullmatch(r"TASK-[A-F0-9]{8}", task_id):
            raise ValueError("task_id has an unsupported format")
        feed = self._feeds.get(camera_id)
        if feed is None:
            raise ValueError("camera stream not found")
        with feed.lock:
            captured = feed.raw_frames.get(frame_sequence)
            if captured is None or captured[0] != frame_sha256:
                raise ValueError("requested observation frame is no longer available or does not match")
            _, raw_pixels, width, height, _, _ = captured

        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("OpenCV and NumPy are required to save an observation image") from exc
        image = np.frombuffer(raw_pixels, dtype=np.uint8).reshape((height, width, 3))
        try:
            encoded, png = cv2.imencode(".png", image)
        except cv2.error as exc:
            raise RuntimeError(f"OpenCV could not encode the observation image: {exc}") from exc
        if not encoded:
            raise RuntimeError("OpenCV could not encode the observation image")
        image_bytes = png.tobytes()
        if len(image_bytes) > MAX_OBSERVATION_IMAGE_BYTES:
            raise ValueError("encoded observation image exceeds the 64 MiB limit")
        image_sha256 = hashlib.sha256(image_bytes).hexdigest()

        repository_root = Path(__file__).resolve().parents[2]
        control_directory = repository_root / ".control-station"
        artifact_root = control_directory / "observations"
        if control_directory.is_symlink() or artifact_root.is_symlink():
            raise ValueError("observation artifact directories cannot be symbolic links")
        artifact_root.mkdir(mode=0o700, parents=True, exist_ok=True)
        if artifact_root.resolve(strict=True) != artifact_root:
            raise ValueError("observation artifact directory cannot use symbolic-link path components")
        task_directory = artifact_root / task_id
        task_directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        if task_directory.is_symlink() or task_directory.resolve(strict=True) != task_directory:
            raise ValueError("task observation directory cannot use symbolic links")
        filename = f"frame-{frame_sequence}-{uuid.uuid4().hex}.png"
        destination = task_directory / filename
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb", dir=task_directory, prefix=".frame-", suffix=".tmp", delete=False
            ) as temporary_file:
                temporary_path = Path(temporary_file.name)
                temporary_file.write(image_bytes)
            os.replace(temporary_path, destination)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()

        return {
            "path": str(destination.relative_to(repository_root)),
            "media_type": "image/png",
            "file_sha256": image_sha256,
            "source_frame_sha256": frame_sha256,
            "pixel_format": "BGR8",
            "width": width,
            "height": height,
        }

    def delete_observation_frame(self, relative_path: str) -> None:
        path_parts = relative_path.split("/") if isinstance(relative_path, str) else []
        if (
            len(path_parts) != 4
            or path_parts[0] != ".control-station"
            or path_parts[1] != "observations"
            or not re.fullmatch(r"TASK-[A-F0-9]{8}", path_parts[2])
            or not re.fullmatch(r"frame-[0-9]+-[0-9a-f]{32}\.png", path_parts[3])
        ):
            raise ValueError("observation artifact path has an unsupported format")
        try:
            repository_root = Path(__file__).resolve().parents[2]
            control_directory = repository_root / ".control-station"
            artifact_root = control_directory / "observations"
            task_directory = artifact_root / path_parts[2]
            candidate = task_directory / path_parts[3]
            if any(
                path.is_symlink()
                for path in (control_directory, artifact_root, task_directory, candidate)
            ):
                raise ValueError("observation artifact path cannot use symbolic links")
            resolved_root = artifact_root.resolve(strict=True)
            resolved_task = task_directory.resolve(strict=True)
            resolved_candidate = candidate.resolve(strict=False)
        except (OSError, RuntimeError) as exc:
            raise ValueError("observation artifact path cannot be resolved safely") from exc
        if (
            resolved_root != artifact_root
            or resolved_task != task_directory
            or resolved_task.parent != resolved_root
            or resolved_candidate != candidate
            or candidate.suffix != ".png"
        ):
            raise ValueError("observation artifact path is outside the managed image directory")
        candidate.unlink(missing_ok=True)

    def sequence_for(self, camera_id: str) -> int:
        feed = self._feeds.get(camera_id)
        if feed is None:
            return -1
        with feed.lock:
            return feed.sequence

    def close(self) -> None:
        self._closed.set()
        for feed in self._feeds.values():
            worker = feed.thread
            if worker is None:
                continue
            worker.join(timeout=2)
            if worker.is_alive():
                continue
            with feed.lock:
                feed.connection_state = CameraConnectionState.DISCONNECTED
                if feed.stream_state != CameraStreamState.ERROR:
                    feed.stream_state = CameraStreamState.STOPPED


class Ros2SimulationCameraStreamManager(UnconfiguredArmCameraStreamManager):
    """Expose one read-only MuJoCo RGB Image topic through the existing MJPEG API."""

    CAMERA_ID = "mujoco_fixed_camera"
    IMAGE_TOPIC = "/mujoco_omx/fixed_camera/color/image_raw"
    DEPTH_TOPIC = "/mujoco_omx/fixed_camera/depth/image_raw"
    CAMERA_INFO_TOPIC = "/mujoco_omx/fixed_camera/camera_info"
    POINT_CLOUD_TOPIC = "/mujoco_omx/fixed_camera/points"
    FRAME_ID = "placeholder_fixed_camera_optical_frame"
    TARGET_FRAME_ID = "mujoco_omx/link0"

    def __init__(self, domain_id: int) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from tf2_ros import Buffer, TransformListener
            from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo 카메라 관측에는 ROS 2 Jazzy, sensor_msgs, tf2_ros가 필요합니다."
            ) from exc

        self._context = Context()
        self._context.init(domain_id=domain_id)
        self._node = Node("control_station_mujoco_camera", context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._tf_buffer = Buffer(node=self._node)
        self._tf_listener = TransformListener(
            self._tf_buffer, self._node, spin_thread=False
        )
        self._feed = _CameraFeed(
            self.CAMERA_ID,
            "FIXED",
            self.IMAGE_TOPIC,
            error_message="MuJoCo ROS RGB frame 수신 대기",
        )
        self._feeds = {self.CAMERA_ID: self._feed}
        self._source_stamp: dict[str, int] | None = None
        self._source_frame_id: str | None = None
        self._rgb_frame_source_headers: dict[int, dict[str, object]] = {}
        self._rgb_stamp_occurrences: dict[tuple[int, int], int] = {}
        self._depth_observation: dict[str, object] = {"state": "NO_SAMPLE"}
        self._depth_received_monotonic: float | None = None
        self._depth_frames: dict[tuple[int, int], dict[str, object]] = {}
        self._depth_stamp_occurrences: dict[tuple[int, int], int] = {}
        self._depth_retained_bytes = 0
        self._camera_info_observation: dict[str, object] = {"state": "NO_SAMPLE"}
        self._camera_info_received_monotonic: float | None = None
        self._point_cloud_observation: dict[str, object] = {"state": "NO_SAMPLE"}
        self._point_cloud_received_monotonic: float | None = None
        detector_weights = os.environ.get("CONTROL_STATION_HEAD_YOLO_WEIGHTS", "").strip()
        detector_prompts = os.environ.get(
            "CONTROL_STATION_HEAD_YOLO_WORLD_PROMPTS", ""
        ).strip()
        self._detector = (
            _YoloDetector(detector_weights, "CONTROL_STATION_HEAD_YOLO")
            if detector_weights or detector_prompts
            else None
        )
        self._detector_thread: threading.Thread | None = None
        if self._detector is not None:
            self._detector_thread = threading.Thread(
                target=self._detector.load,
                name="mujoco-fixed-camera-detector-loader",
                daemon=True,
            )
            self._detector_thread.start()
        self._closed = threading.Event()
        self._subscription = self._node.create_subscription(
            Image, self.IMAGE_TOPIC, self._on_image, 1
        )
        self._depth_subscription = self._node.create_subscription(
            Image, self.DEPTH_TOPIC, self._on_depth, 1
        )
        self._camera_info_subscription = self._node.create_subscription(
            CameraInfo, self.CAMERA_INFO_TOPIC, self._on_camera_info, 1
        )
        self._point_cloud_subscription = self._node.create_subscription(
            PointCloud2, self.POINT_CLOUD_TOPIC, self._on_point_cloud, 1
        )
        self._point_cloud_float32 = PointField.FLOAT32
        self._thread = threading.Thread(
            target=self._executor.spin,
            name="mujoco-camera-observer",
            daemon=True,
        )
        self._thread.start()

    def _on_image(self, message) -> None:  # type: ignore[no-untyped-def]
        try:
            import cv2
            import numpy as np

            width = int(message.width)
            height = int(message.height)
            step = int(message.step)
            if (
                message.encoding != "rgb8"
                or str(message.header.frame_id) != self.FRAME_ID
                or width <= 0
                or height <= 0
                or width > 8192
                or height > 8192
                or step < width * 3
                or step * height > MAX_OBSERVATION_IMAGE_BYTES
                or len(message.data) != step * height
            ):
                raise ValueError("지원하지 않거나 크기가 유효하지 않은 ROS RGB8 frame")
            rows = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
            rgb = rows[:, : width * 3].reshape(height, width, 3)
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            raw_pixels = bgr.tobytes(order="C")
            frame_sha256 = hashlib.sha256(raw_pixels).hexdigest()
            encoded, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not encoded:
                raise RuntimeError("ROS RGB frame의 JPEG 변환에 실패했습니다")
            captured_at = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            captured_monotonic = time.monotonic()
            source_stamp = self._message_stamp(message)
            source_stamp_key = (source_stamp["sec"], source_stamp["nanosec"])

            with self._feed.lock:
                stamp_occurrence = self._increment_stamp_occurrence(
                    self._rgb_stamp_occurrences, source_stamp_key
                )
                self._feed.sequence += 1
                sequence = self._feed.sequence
                self._feed.frame_width = width
                self._feed.frame_height = height
                self._feed.frame_sha256 = frame_sha256
                self._feed.raw_frames[sequence] = (
                    frame_sha256,
                    raw_pixels,
                    width,
                    height,
                    captured_at,
                    captured_monotonic,
                )
                self._rgb_frame_source_headers[sequence] = {
                    "source_stamp": source_stamp,
                    "source_frame_id": str(message.header.frame_id),
                    "stamp_occurrence": stamp_occurrence,
                }
                while len(self._feed.raw_frames) > 5:
                    expired_sequence = min(self._feed.raw_frames)
                    del self._feed.raw_frames[expired_sequence]
                    self._rgb_frame_source_headers.pop(expired_sequence, None)
                self._feed.frame = jpeg.tobytes()
                self._feed.last_frame_at = captured_at
                self._feed.last_frame_monotonic = captured_monotonic
                self._feed.frame_times.append(captured_monotonic)
                self._feed.frame_times = [
                    value for value in self._feed.frame_times
                    if captured_monotonic - value <= 2.0
                ]
                self._feed.connection_state = CameraConnectionState.CONNECTED
                self._feed.stream_state = CameraStreamState.STREAMING
                self._feed.error_message = None
                self._source_stamp = source_stamp
                self._source_frame_id = str(message.header.frame_id)
        except Exception as exc:
            with self._feed.lock:
                self._feed.stream_state = CameraStreamState.ERROR
                self._feed.error_message = f"MuJoCo ROS RGB frame 거부: {exc}"[:512]

    @staticmethod
    def _message_stamp(message) -> dict[str, int]:  # type: ignore[no-untyped-def]
        result = {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        }
        if not 0 <= result["nanosec"] < 1_000_000_000:
            raise ValueError("ROS source stamp nanosec 범위가 유효하지 않습니다")
        return result

    @staticmethod
    def _increment_stamp_occurrence(
        history: dict[tuple[int, int], int], key: tuple[int, int]
    ) -> int:
        occurrence = history.pop(key, 0) + 1
        history[key] = occurrence
        while len(history) > MAX_SOURCE_STAMP_HISTORY:
            history.pop(next(iter(history)))
        return occurrence

    @staticmethod
    def _rgb_header_match(
        observation: dict[str, object],
        *,
        rgb_stamp: dict[str, int] | None,
        rgb_frame_id: str | None,
        rgb_width: int | None,
        rgb_height: int | None,
    ) -> str:
        if observation.get("state") != "OBSERVED":
            return "SOURCE_UNAVAILABLE"
        if rgb_stamp is None or rgb_frame_id is None or rgb_width is None or rgb_height is None:
            return "RGB_UNAVAILABLE"
        if (
            observation.get("source_stamp") != rgb_stamp
            or observation.get("source_frame_id") != rgb_frame_id
        ):
            return "HEADER_MISMATCH"
        if observation.get("width") != rgb_width or observation.get("height") != rgb_height:
            return "DIMENSIONS_MISMATCH"
        return "HEADER_AND_DIMENSIONS_MATCH"

    def _on_depth(self, message) -> None:  # type: ignore[no-untyped-def]
        try:
            width, height, step = int(message.width), int(message.height), int(message.step)
            encoding = str(message.encoding)
            source_stamp = self._message_stamp(message)
            stamp_key = (source_stamp["sec"], source_stamp["nanosec"])
            bytes_per_pixel = {"32FC1": 4, "16UC1": 2}.get(encoding)
            if (
                encoding not in {"32FC1", "16UC1"}
                or str(message.header.frame_id) != self.FRAME_ID
                or width <= 0
                or height <= 0
                or width > 8192
                or height > 8192
                or bytes_per_pixel is None
                or step < width * bytes_per_pixel
                or step % bytes_per_pixel != 0
                or step * height > MAX_OBSERVATION_IMAGE_BYTES
                or len(message.data) != step * height
            ):
                raise ValueError("지원하지 않거나 크기/좌표계가 유효하지 않은 depth frame")
            observation = {
                "state": "OBSERVED",
                "topic": self.DEPTH_TOPIC,
                "encoding": encoding,
                "width": width,
                "height": height,
                "step_bytes": step,
                "data_bytes": len(message.data),
                "is_bigendian": bool(message.is_bigendian),
                "source_stamp": source_stamp,
                "source_frame_id": str(message.header.frame_id),
            }
            retain_for_review = len(message.data) <= MAX_RETAINED_DEPTH_BYTES
            observation["review_frame_retained"] = retain_for_review
            with self._feed.lock:
                stamp_occurrence = self._increment_stamp_occurrence(
                    self._depth_stamp_occurrences, stamp_key
                )
            observation["source_stamp_occurrence"] = stamp_occurrence
            payload = bytes(message.data) if retain_for_review else None
            retained = (
                {
                    "encoding": encoding,
                    "width": width,
                    "height": height,
                    "step_bytes": step,
                    "data": payload,
                    "is_bigendian": bool(message.is_bigendian),
                    "source_stamp": source_stamp,
                    "source_frame_id": str(message.header.frame_id),
                    "source_stamp_occurrence": stamp_occurrence,
                    "received_monotonic": time.monotonic(),
                }
                if payload is not None
                else None
            )
            with self._feed.lock:
                previous = self._depth_frames.pop(stamp_key, None)
                if previous is not None:
                    self._depth_retained_bytes -= len(previous["data"])
                if retained is not None:
                    self._depth_frames[stamp_key] = retained
                    self._depth_retained_bytes += len(message.data)
                    while (
                        len(self._depth_frames) > 5
                        or self._depth_retained_bytes > MAX_RETAINED_DEPTH_BYTES
                    ):
                        oldest_key = next(iter(self._depth_frames))
                        oldest = self._depth_frames.pop(oldest_key)
                        self._depth_retained_bytes -= len(oldest["data"])
                self._depth_observation = observation
                self._depth_received_monotonic = time.monotonic()
        except Exception as exc:
            try:
                failed_stamp_key = (
                    int(message.header.stamp.sec),
                    int(message.header.stamp.nanosec),
                )
            except Exception:
                failed_stamp_key = None
            with self._feed.lock:
                if failed_stamp_key is not None:
                    previous = self._depth_frames.pop(failed_stamp_key, None)
                    if previous is not None:
                        self._depth_retained_bytes -= len(previous["data"])
                self._depth_observation = {
                    "state": "INVALID",
                    "topic": self.DEPTH_TOPIC,
                    "message": str(exc)[:256],
                }
                self._depth_received_monotonic = None

    def summarize_detection_depth(
        self, camera: CameraSnapshot, candidates: list[dict[str, object]]
    ) -> dict[str, object]:
        """Summarize raw depth samples only for detections on the exact RGB header."""
        result: dict[str, object] = {
            "schema_version": 1,
            "policy_version": DEPTH_REGION_SUMMARY_POLICY_VERSION,
            "candidate_limit": MAX_DEPTH_SUMMARY_CANDIDATES,
            "summary_pixel_limit": MAX_DEPTH_SUMMARY_TOTAL_PIXELS,
            "review_required": True,
            "used_for_selection": False,
            "target_pose_use": False,
            "camera_id": camera.id,
            "rgb_frame_sequence": camera.frame_sequence,
            "rgb_frame_sha256": camera.frame_sha256,
            "rgb_source_stamp": camera.source_stamp,
            "rgb_source_frame_id": camera.source_frame_id,
            "candidates": [],
        }
        if camera.position != "FIXED" or camera.id != self.CAMERA_ID:
            result["state"] = "CAMERA_NOT_SUPPORTED"
            return result
        if camera.frame_freshness.value != "FRESH" or camera.source_stamp is None:
            result["state"] = "RGB_FRAME_UNAVAILABLE"
            return result
        if not candidates:
            result["state"] = "NO_CANDIDATES"
            return result
        if len(candidates) > MAX_DEPTH_SUMMARY_CANDIDATES:
            result["state"] = "CANDIDATE_LIMIT_EXCEEDED"
            return result
        with self._feed.lock:
            rgb_frame = self._feed.raw_frames.get(camera.frame_sequence)
            if rgb_frame is None or rgb_frame[0] != camera.frame_sha256:
                result["state"] = "RGB_FRAME_NOT_RETAINED"
                return result
            rgb_header = self._rgb_frame_source_headers.get(camera.frame_sequence)
            if (
                rgb_header is None
                or rgb_header.get("source_stamp") != camera.source_stamp
                or rgb_header.get("source_frame_id") != camera.source_frame_id
            ):
                result["state"] = "RGB_HEADER_MISMATCH"
                return result
            if rgb_header.get("stamp_occurrence") != 1:
                result["state"] = "RGB_STAMP_NOT_UNIQUE"
                return result
            rgb_age = time.monotonic() - rgb_frame[5]
            if rgb_age > 2.0:
                result["state"] = "RGB_FRAME_STALE"
                return result
        stamp = camera.source_stamp
        stamp_key = (int(stamp["sec"]), int(stamp["nanosec"]))
        with self._feed.lock:
            depth_frame = self._depth_frames.get(stamp_key)
            if depth_frame is None:
                result["state"] = "NO_EXACT_DEPTH_FRAME"
                return result
            frame = dict(depth_frame)
        if time.monotonic() - float(frame["received_monotonic"]) > 2.0:
            result["state"] = "DEPTH_FRAME_STALE"
            return result
        if (
            frame["source_stamp"] != camera.source_stamp
            or frame["source_frame_id"] != camera.source_frame_id
        ):
            result["state"] = "DEPTH_HEADER_MISMATCH"
            return result
        if frame["source_stamp_occurrence"] != 1:
            result["state"] = "DEPTH_STAMP_NOT_UNIQUE"
            return result
        if frame["width"] != camera.frame_width or frame["height"] != camera.frame_height:
            result["state"] = "DEPTH_DIMENSIONS_MISMATCH"
            return result

        try:
            import numpy as np
        except ImportError:
            result["state"] = "NUMPY_UNAVAILABLE"
            return result
        encoding = str(frame["encoding"])
        bytes_per_pixel = {"32FC1": 4, "16UC1": 2}.get(encoding)
        width = int(frame["width"])
        height = int(frame["height"])
        step = int(frame["step_bytes"])
        if bytes_per_pixel is None or step % bytes_per_pixel != 0:
            result["state"] = "DEPTH_ENCODING_UNSUPPORTED"
            return result
        byte_order = ">" if frame["is_bigendian"] else "<"
        dtype = np.dtype(f"{byte_order}{'f4' if encoding == '32FC1' else 'u2'}")
        row_values = step // bytes_per_pixel
        image = np.frombuffer(frame["data"], dtype=dtype).reshape(height, row_values)[:, :width]
        pixel_summaries: list[dict[str, object]] = []
        summarized_pixel_count = 0
        for index, candidate in enumerate(candidates):
            if (
                candidate.get("camera_id") != camera.id
                or candidate.get("frame_sequence") != camera.frame_sequence
                or candidate.get("captured_at") != camera.last_frame_at
                or candidate.get("frame_sha256") != camera.frame_sha256
                or candidate.get("pixel_transform_id") != camera.pixel_transform_id
                or candidate.get("frame_width") != width
                or candidate.get("frame_height") != height
            ):
                pixel_summaries.append(
                    {"candidate_index": index, "state": "CANDIDATE_PROVENANCE_MISMATCH"}
                )
                continue
            bbox = candidate.get("bbox_xyxy_px")
            if (
                not isinstance(bbox, list)
                or len(bbox) != 4
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    for value in bbox
                )
            ):
                pixel_summaries.append({"candidate_index": index, "state": "INVALID_BBOX"})
                continue
            x1, y1, x2, y2 = (float(value) for value in bbox)
            if (
                x1 < 0
                or y1 < 0
                or x2 > width
                or y2 > height
                or x2 <= x1
                or y2 <= y1
            ):
                pixel_summaries.append({"candidate_index": index, "state": "BBOX_OUT_OF_FRAME"})
                continue
            left, top = math.floor(x1), math.floor(y1)
            right, bottom = math.ceil(x2), math.ceil(y2)
            bbox_pixel_count = (right - left) * (bottom - top)
            if summarized_pixel_count + bbox_pixel_count > MAX_DEPTH_SUMMARY_TOTAL_PIXELS:
                pixel_summaries.append(
                    {"candidate_index": index, "state": "SUMMARY_PIXEL_BUDGET_EXCEEDED"}
                )
                continue
            summarized_pixel_count += bbox_pixel_count
            region = image[top:bottom, left:right]
            finite = np.isfinite(region)
            finite_values = region[finite]
            positive_values = finite_values[finite_values > 0]
            zero_count = int(np.count_nonzero(finite_values == 0))
            summary: dict[str, object] = {
                "candidate_index": index,
                "state": (
                    "POSITIVE_FINITE_SAMPLES"
                    if positive_values.size
                    else "NO_POSITIVE_FINITE_SAMPLES"
                ),
                "bbox_xyxy_px": [left, top, right, bottom],
                "pixel_count": int(region.size),
                "finite_sample_count": int(finite_values.size),
                "positive_sample_count": int(positive_values.size),
                "nonpositive_sample_count": int(finite_values.size - positive_values.size),
                "zero_sample_count": zero_count,
                "nonfinite_sample_count": int(region.size - finite_values.size),
                "value_encoding": encoding,
                "numeric_unit_status": "NOT_INTERPRETED",
                "sample_policy": "FINITE_POSITIVE_RAW_VALUES_ONLY",
                "review_required": True,
                "used_for_selection": False,
                "target_pose_use": False,
            }
            if positive_values.size:
                summary.update(
                    {
                        "raw_value_min": float(np.min(positive_values)),
                        "raw_value_median": float(np.median(positive_values)),
                        "raw_value_max": float(np.max(positive_values)),
                    }
                )
            pixel_summaries.append(summary)
        result.update(
            {
                "state": "SUMMARIZED",
                "depth_source_stamp": frame["source_stamp"],
                "depth_source_frame_id": frame["source_frame_id"],
                "depth_source_stamp_occurrence": frame["source_stamp_occurrence"],
                "depth_width": width,
                "depth_height": height,
                "depth_encoding": encoding,
                "depth_is_bigendian": bool(frame["is_bigendian"]),
                "summarized_pixel_count": summarized_pixel_count,
                "rgb_frame_age_seconds": round(rgb_age, 3),
                "depth_received_age_seconds": round(
                    time.monotonic() - float(frame["received_monotonic"]), 3
                ),
                "depth_data_sha256": hashlib.sha256(frame["data"]).hexdigest(),
                "candidates": pixel_summaries,
            }
        )
        return result

    def _on_camera_info(self, message) -> None:  # type: ignore[no-untyped-def]
        try:
            width, height = int(message.width), int(message.height)
            distortion_model = str(message.distortion_model)
            d = [float(value) for value in message.d]
            k = [float(value) for value in message.k]
            r = [float(value) for value in message.r]
            p = [float(value) for value in message.p]
            binning_x, binning_y = int(message.binning_x), int(message.binning_y)
            roi_x, roi_y = int(message.roi.x_offset), int(message.roi.y_offset)
            roi_width, roi_height = int(message.roi.width), int(message.roi.height)
            if (
                str(message.header.frame_id) != self.FRAME_ID
                or width <= 0
                or height <= 0
                or width > 8192
                or height > 8192
                or len(distortion_model) > 64
                or len(d) > 64
                or len(k) != 9
                or len(r) != 9
                or len(p) != 12
                or not all(math.isfinite(value) for value in [*d, *k, *r, *p])
                or binning_x < 0
                or binning_y < 0
                or min(roi_x, roi_y, roi_width, roi_height) < 0
                or (roi_width > 0 and roi_x + roi_width > width)
                or (roi_height > 0 and roi_y + roi_height > height)
            ):
                raise ValueError("CameraInfo 크기, 계수 또는 좌표계가 유효하지 않습니다")
            observation = {
                "state": "OBSERVED",
                "topic": self.CAMERA_INFO_TOPIC,
                "width": width,
                "height": height,
                "distortion_model": distortion_model,
                "d": d,
                "k": k,
                "r": r,
                "p": p,
                "binning_x": binning_x,
                "binning_y": binning_y,
                "roi": {
                    "x_offset": roi_x,
                    "y_offset": roi_y,
                    "height": roi_height,
                    "width": roi_width,
                    "do_rectify": bool(message.roi.do_rectify),
                },
                "source_stamp": self._message_stamp(message),
                "source_frame_id": str(message.header.frame_id),
                "calibration_use": "OBSERVATION_ONLY_UNVALIDATED",
            }
            with self._feed.lock:
                self._camera_info_observation = observation
                self._camera_info_received_monotonic = time.monotonic()
        except Exception as exc:
            with self._feed.lock:
                self._camera_info_observation = {
                    "state": "INVALID",
                    "topic": self.CAMERA_INFO_TOPIC,
                    "message": str(exc)[:256],
                    "calibration_use": "OBSERVATION_ONLY_UNVALIDATED",
                }
                self._camera_info_received_monotonic = None

    def _on_point_cloud(self, message) -> None:  # type: ignore[no-untyped-def]
        try:
            width, height = int(message.width), int(message.height)
            point_step, row_step = int(message.point_step), int(message.row_step)
            data_bytes = len(message.data)
            if len(message.fields) > 64:
                raise ValueError("PointCloud2 field 개수가 제한을 초과했습니다")
            fields = {str(field.name): field for field in message.fields}
            if (
                str(message.header.frame_id) != self.FRAME_ID
                or width <= 0
                or height <= 0
                or width > 8192
                or height > 8192
                or point_step <= 0
                or row_step < width * point_step
                or row_step * height > MAX_OBSERVATION_IMAGE_BYTES
                or data_bytes != row_step * height
                or len(fields) != len(message.fields)
                or any(
                    name not in fields
                    or int(fields[name].datatype) != self._point_cloud_float32
                    or int(fields[name].count) != 1
                    or int(fields[name].offset) < 0
                    or int(fields[name].offset) + 4 > point_step
                    for name in ("x", "y", "z")
                )
            ):
                raise ValueError("지원하지 않거나 크기/필드/좌표계가 유효하지 않은 PointCloud2")
            observation = {
                "state": "OBSERVED",
                "topic": self.POINT_CLOUD_TOPIC,
                "width": width,
                "height": height,
                "fields": [
                    {
                        "name": str(field.name),
                        "offset": int(field.offset),
                        "datatype": int(field.datatype),
                        "count": int(field.count),
                    }
                    for field in message.fields
                ],
                "point_step_bytes": point_step,
                "row_step_bytes": row_step,
                "data_bytes": data_bytes,
                "is_bigendian": bool(message.is_bigendian),
                "is_dense": bool(message.is_dense),
                "source_stamp": self._message_stamp(message),
                "source_frame_id": str(message.header.frame_id),
                "coordinates_use": "SIMULATION_CAMERA_FRAME_OBSERVATION_ONLY",
            }
            with self._feed.lock:
                self._point_cloud_observation = observation
                self._point_cloud_received_monotonic = time.monotonic()
        except Exception as exc:
            with self._feed.lock:
                self._point_cloud_observation = {
                    "state": "INVALID",
                    "topic": self.POINT_CLOUD_TOPIC,
                    "message": str(exc)[:256],
                    "coordinates_use": "SIMULATION_CAMERA_FRAME_OBSERVATION_ONLY",
                }
                self._point_cloud_received_monotonic = None

    def snapshots(self) -> list[CameraSnapshot]:
        with self._feed.lock:
            now = time.monotonic()
            age = (
                now - self._feed.last_frame_monotonic
                if self._feed.last_frame_monotonic is not None
                else None
            )
            freshness = (
                FrameFreshness.UNKNOWN
                if age is None
                else FrameFreshness.FRESH if age <= 2.0 else FrameFreshness.STALE
            )
            frame_rate = None
            if len(self._feed.frame_times) >= 2:
                elapsed = self._feed.frame_times[-1] - self._feed.frame_times[0]
                if elapsed > 0:
                    frame_rate = round((len(self._feed.frame_times) - 1) / elapsed, 1)
            depth_observation = dict(self._depth_observation)
            depth_observation["rgb_header_dimension_match"] = self._rgb_header_match(
                depth_observation,
                rgb_stamp=self._source_stamp,
                rgb_frame_id=self._source_frame_id,
                rgb_width=self._feed.frame_width,
                rgb_height=self._feed.frame_height,
            )
            if self._depth_received_monotonic is not None:
                depth_observation["freshness"] = (
                    "FRESH" if now - self._depth_received_monotonic <= 2.0 else "STALE"
                )
            camera_info_observation = dict(self._camera_info_observation)
            camera_info_observation["rgb_header_dimension_match"] = self._rgb_header_match(
                camera_info_observation,
                rgb_stamp=self._source_stamp,
                rgb_frame_id=self._source_frame_id,
                rgb_width=self._feed.frame_width,
                rgb_height=self._feed.frame_height,
            )
            if self._camera_info_received_monotonic is not None:
                camera_info_observation["freshness"] = (
                    "FRESH" if now - self._camera_info_received_monotonic <= 2.0 else "STALE"
                )
            point_cloud_observation = dict(self._point_cloud_observation)
            point_cloud_observation["rgb_header_dimension_match"] = self._rgb_header_match(
                point_cloud_observation,
                rgb_stamp=self._source_stamp,
                rgb_frame_id=self._source_frame_id,
                rgb_width=self._feed.frame_width,
                rgb_height=self._feed.frame_height,
            )
            if self._point_cloud_received_monotonic is not None:
                point_cloud_observation["freshness"] = (
                    "FRESH" if now - self._point_cloud_received_monotonic <= 2.0 else "STALE"
                )
            tf_observation = self._camera_to_base_tf_observation(
                rgb_stamp=self._source_stamp,
                rgb_frame_id=self._source_frame_id,
                rgb_received_monotonic=self._feed.last_frame_monotonic,
            )
            return [
                CameraSnapshot(
                    id=self.CAMERA_ID,
                    robot_id="arm-01",
                    position="FIXED",
                    connection_state=self._feed.connection_state,
                    stream_state=(
                        self._feed.stream_state
                        if self._feed.frame is not None
                        or self._feed.stream_state == CameraStreamState.ERROR
                        else CameraStreamState.NO_FRAMES
                    ),
                    frame_freshness=freshness,
                    last_frame_at=self._feed.last_frame_at,
                    frame_sequence=self._feed.sequence,
                    frame_width=self._feed.frame_width,
                    frame_height=self._feed.frame_height,
                    frame_sha256=self._feed.frame_sha256,
                    pixel_transform_id=MUJOCO_ROS_PIXEL_TRANSFORM_ID,
                    source_stamp=self._source_stamp,
                    source_frame_id=self._source_frame_id,
                    depth_observation=depth_observation,
                    camera_info_observation=camera_info_observation,
                    point_cloud_observation=point_cloud_observation,
                    tf_observation=tf_observation,
                    frame_rate=frame_rate,
                    error_message=(
                        self._feed.error_message
                        or (self._detector.error_message if self._detector else None)
                    ),
                    detector_state=(self._detector.state if self._detector else None),
                    detector_model_type=(
                        self._detector.detector_model_type if self._detector else None
                    ),
                    detector_prompts=(
                        list(self._detector.prompts) if self._detector else None
                    ),
                    detector_classes=(
                        list(self._detector.class_names)
                        if self._detector and self._detector.state == "ready"
                        else None
                    ),
                    detector_confidence_threshold=(
                        self._detector.confidence_threshold if self._detector else None
                    ),
                    detector_weights_sha256=(
                        self._detector.weights_sha256 if self._detector else None
                    ),
                    detector_inference_image_size=(
                        self._detector.inference_image_size if self._detector else None
                    ),
                    detector_device=(self._detector.device if self._detector else None),
                    detector_ultralytics_version=(
                        self._detector.ultralytics_version if self._detector else None
                    ),
                    stream_url=(
                        f"/streams/{self.CAMERA_ID}.mjpg"
                        if self._feed.frame is not None else None
                    ),
                )
            ]

    def _camera_to_base_tf_observation(
        self,
        *,
        rgb_stamp: dict[str, int] | None,
        rgb_frame_id: str | None,
        rgb_received_monotonic: float | None,
    ) -> dict[str, object]:
        base = {
            "target_frame_id": self.TARGET_FRAME_ID,
            "source_frame_id": rgb_frame_id,
            "source_stamp": rgb_stamp,
            "transform_basis": "SIMULATION_ONLY_PLACEHOLDER",
            "target_pose_use": False,
        }
        if rgb_stamp is None or rgb_frame_id is None or rgb_received_monotonic is None:
            return {"state": "NO_RGB", **base}
        age = time.monotonic() - rgb_received_monotonic
        if age > 2.0:
            return {"state": "RGB_STALE", "rgb_age_seconds": round(age, 3), **base}
        try:
            from rclpy.duration import Duration
            from rclpy.time import Time
            from builtin_interfaces.msg import Time as TimeMessage

            stamp_message = TimeMessage(
                sec=rgb_stamp["sec"], nanosec=rgb_stamp["nanosec"]
            )
            transform = self._tf_buffer.lookup_transform(
                self.TARGET_FRAME_ID,
                rgb_frame_id,
                Time.from_msg(stamp_message),
                timeout=Duration(seconds=0.0),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            values = [
                float(translation.x),
                float(translation.y),
                float(translation.z),
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("TF returned non-finite transform values")
            return {
                "state": "AVAILABLE",
                **base,
                "transform_stamp": self._message_stamp(transform),
                "translation_xyz_m": values[:3],
                "rotation_xyzw": values[3:],
                "rgb_age_seconds": round(age, 3),
            }
        except Exception as exc:
            return {
                "state": "LOOKUP_FAILED",
                **base,
                "error_type": type(exc).__name__,
                "rgb_age_seconds": round(age, 3),
            }

    def frame_for(self, camera_id: str) -> bytes | None:
        if camera_id != self.CAMERA_ID:
            return None
        with self._feed.lock:
            return self._feed.frame

    def detect_for_observation(self, camera: CameraSnapshot) -> CameraSnapshot:
        """Run the configured detector on the exact retained frame in a task snapshot."""
        if camera.id != self.CAMERA_ID:
            raise ValueError("camera stream not found")
        detector = self._detector
        if detector is None:
            raise RuntimeError("MuJoCo fixed camera detector is not configured")
        if detector.state != "ready":
            raise RuntimeError(detector.error_message or "MuJoCo detector is not ready")
        if (
            isinstance(camera.frame_sequence, bool)
            or not isinstance(camera.frame_sequence, int)
            or camera.frame_sequence <= 0
            or not isinstance(camera.frame_sha256, str)
        ):
            raise ValueError("requested observation has no frame sequence/hash")
        with self._feed.lock:
            captured = self._feed.raw_frames.get(camera.frame_sequence)
            if captured is None or captured[0] != camera.frame_sha256:
                raise RuntimeError("requested observation frame is no longer retained")
            frame_sha256, raw_pixels, width, height, captured_at, captured_monotonic = captured
        if (
            captured_at != camera.last_frame_at
            or width != camera.frame_width
            or height != camera.frame_height
        ):
            raise RuntimeError("requested observation frame provenance changed")
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("NumPy is required for detector inference") from exc
        image = np.frombuffer(raw_pixels, dtype=np.uint8).reshape((height, width, 3)).copy()
        detections = detector.detect(image)
        for detection in detections:
            detection.update(
                {
                    "camera_id": camera.id,
                    "captured_at": captured_at,
                    "frame_sequence": camera.frame_sequence,
                    "frame_sha256": frame_sha256,
                    "pixel_transform_id": camera.pixel_transform_id,
                }
            )
        freshness = (
            FrameFreshness.FRESH
            if time.monotonic() - captured_monotonic <= 2.0
            else FrameFreshness.STALE
        )
        with self._feed.lock:
            current_connection_state = self._feed.connection_state
            current_stream_state = self._feed.stream_state
        return replace(
            camera,
            connection_state=current_connection_state,
            stream_state=current_stream_state,
            frame_freshness=freshness,
            detector_state=detector.state,
            detector_model_type=detector.detector_model_type,
            detector_prompts=list(detector.prompts),
            detector_classes=list(detector.class_names),
            detector_confidence_threshold=detector.confidence_threshold,
            detector_weights_sha256=detector.weights_sha256,
            detector_inference_image_size=detector.inference_image_size,
            detector_device=detector.device,
            detector_ultralytics_version=detector.ultralytics_version,
            detector_inference_frame_sequence=camera.frame_sequence,
            detections=detections,
        )

    def detect_for_prompt(self, camera: CameraSnapshot, prompt: str) -> CameraSnapshot:
        if camera.id != self.CAMERA_ID:
            raise ValueError("camera stream not found")
        detector = self._detector
        if detector is None or detector.detector_model_type != "YOLO_WORLD_OPEN_VOCAB":
            raise RuntimeError("MuJoCo fixed camera has no YOLO-World detector")
        if (
            isinstance(camera.frame_sequence, bool)
            or not isinstance(camera.frame_sequence, int)
            or camera.frame_sequence <= 0
            or not isinstance(camera.frame_sha256, str)
        ):
            raise ValueError("requested observation has no frame sequence/hash")
        with self._feed.lock:
            captured = self._feed.raw_frames.get(camera.frame_sequence)
            if captured is None or captured[0] != camera.frame_sha256:
                raise RuntimeError("requested observation frame is no longer retained")
            frame_sha256, raw_pixels, width, height, captured_at, captured_monotonic = captured
        if (
            captured_at != camera.last_frame_at
            or width != camera.frame_width
            or height != camera.frame_height
        ):
            raise RuntimeError("requested observation frame provenance changed")
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("NumPy is required for request-prompt inference") from exc
        image = np.frombuffer(raw_pixels, dtype=np.uint8).reshape((height, width, 3)).copy()
        detections = detector.detect_for_prompt(image, prompt)
        for detection in detections:
            detection.update(
                {
                    "camera_id": camera.id,
                    "captured_at": captured_at,
                    "frame_sequence": camera.frame_sequence,
                    "frame_sha256": frame_sha256,
                    "pixel_transform_id": camera.pixel_transform_id,
                }
            )
        freshness = (
            FrameFreshness.FRESH
            if time.monotonic() - captured_monotonic <= 2.0
            else FrameFreshness.STALE
        )
        with self._feed.lock:
            current_connection_state = self._feed.connection_state
            current_stream_state = self._feed.stream_state
        return replace(
            camera,
            connection_state=current_connection_state,
            stream_state=current_stream_state,
            frame_freshness=freshness,
            detector_state=detector.state,
            detector_model_type=detector.detector_model_type,
            detector_prompts=list(detector.prompts),
            detector_classes=[prompt.strip()],
            detector_confidence_threshold=detector.confidence_threshold,
            detector_weights_sha256=detector.weights_sha256,
            detector_inference_image_size=detector.inference_image_size,
            detector_device=detector.device,
            detector_ultralytics_version=detector.ultralytics_version,
            detector_inference_frame_sequence=camera.frame_sequence,
            detections=detections,
        )

    def sequence_for(self, camera_id: str) -> int:
        if camera_id != self.CAMERA_ID:
            return -1
        with self._feed.lock:
            return self._feed.sequence

    def save_observation_frame(
        self, camera_id: str, frame_sequence: int, frame_sha256: str, task_id: str
    ) -> dict[str, object]:
        if camera_id != self.CAMERA_ID:
            raise ValueError("camera stream not found")
        return OpenCVArmCameraStreamManager.save_observation_frame(
            self, camera_id, frame_sequence, frame_sha256, task_id
        )

    def delete_observation_frame(self, relative_path: str) -> None:
        OpenCVArmCameraStreamManager.delete_observation_frame(self, relative_path)

    def close(self) -> None:
        self._closed.set()
        self._executor.shutdown(timeout_sec=2.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        self._thread.join(timeout=2.0)
        if self._detector_thread is not None:
            self._detector_thread.join(timeout=2.0)
        self._context.try_shutdown()
