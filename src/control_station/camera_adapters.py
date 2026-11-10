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
from typing import Callable, Protocol

from control_station.models import (
    CameraConnectionState,
    CameraSnapshot,
    CameraStreamState,
    FrameFreshness,
)


PIXEL_TRANSFORM_ID = "opencv-v4l2-capture-no-app-geometry-v1"
MAX_OBSERVATION_IMAGE_BYTES = 64 * 1024 * 1024
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
    # Set while this process holds no open capture, so another process may open the device.
    released: threading.Event = field(default_factory=threading.Event)
    # Requested capture size. Some sensors crop the field of view at 4:3, so a wider mode may show more.
    capture_size: tuple[int, int] = (640, 480)


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
            CameraSnapshot(
                id="front_camera",
                robot_id="arm-01",
                position="FRONT",
                connection_state=CameraConnectionState.UNKNOWN,
                stream_state=CameraStreamState.UNKNOWN,
                error_message="Front Camera 장치 경로 미설정 (CONTROL_STATION_FRONT_CAMERA)",
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

    def __init__(
        self, joint_enabled: bool = True, policy_active: Callable[[], bool] | None = None
    ) -> None:
        # The learned-policy client opens the Front Camera itself, and V4L2 lets only one
        # process stream it, so the dashboard gives it up while a policy run is requested or alive.
        self._policy_active = policy_active
        self._front_policy_hold = False
        self._policy_active_cache = (0.0, False)
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
        front_device = os.environ.get(
            "CONTROL_STATION_FRONT_CAMERA",
            os.environ.get("CONTROL_STATION_POLICY_FRONT_CAMERA", ""),
        ).strip()
        front_weights = os.environ.get("CONTROL_STATION_FRONT_YOLO_WEIGHTS", "").strip()
        front_prompts = os.environ.get("CONTROL_STATION_FRONT_YOLO_WORLD_PROMPTS", "").strip()
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
            "front_camera": _CameraFeed(
                "front_camera",
                "FRONT",
                front_device or None,
                detector=(
                    _YoloDetector(front_weights, "CONTROL_STATION_FRONT_YOLO")
                    if front_weights or front_prompts
                    else None
                ),
            ),
        }
        front_feed = self._feeds["front_camera"]
        if not front_feed.device:
            front_feed.error_message = "Front Camera 장치 경로 미설정 (CONTROL_STATION_FRONT_CAMERA)"
        by_device: dict[str, list[_CameraFeed]] = {}
        for feed in self._feeds.values():
            if feed.device:
                by_device.setdefault(os.path.realpath(feed.device), []).append(feed)
        for shared in by_device.values():
            if len(shared) < 2:
                continue
            labels = "와 ".join(f"{feed.position.title()} Camera" for feed in shared)
            message = (
                f"{labels} 경로가 같은 V4L2 장치로 연결됩니다. 실제 장착을 확인한 뒤 "
                "CONTROL_STATION_HEAD_CAMERA, CONTROL_STATION_JOINT_CAMERA, "
                "CONTROL_STATION_FRONT_CAMERA를 서로 다른 장치로 설정하세요."
            )
            for feed in shared:
                feed.device = None
                feed.connection_state = CameraConnectionState.DISCONNECTED
                feed.stream_state = CameraStreamState.ERROR
                feed.error_message = message
        for feed in self._feeds.values():
            feed.released.set()
            env_name = f"CONTROL_STATION_{feed.position}_CAMERA_RESOLUTION"
            raw_size = os.environ.get(env_name, "").strip()
            size_match = re.fullmatch(r"(\d{2,5})x(\d{2,5})", raw_size)
            if size_match:
                feed.capture_size = (int(size_match.group(1)), int(size_match.group(2)))
            elif raw_size and feed.device:
                feed.device = None
                feed.connection_state = CameraConnectionState.DISCONNECTED
                feed.stream_state = CameraStreamState.ERROR
                feed.error_message = f"{env_name} 형식이 잘못되었습니다: {raw_size} (예: 1280x720)"
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

    def _held_for_policy(self, feed: _CameraFeed) -> bool:
        """Return whether the learned-policy client should own this camera instead."""
        if feed.camera_id != "front_camera":
            return False
        if self._front_policy_hold:
            return True
        if self._policy_active is None:
            return False
        checked_at, active = self._policy_active_cache
        now = time.monotonic()
        if now - checked_at >= 0.2:
            try:
                active = bool(self._policy_active())
            except Exception:
                active = False
            self._policy_active_cache = (now, active)
        return active

    def hold_front_for_policy(self, hold: bool, timeout: float = 5.0) -> bool:
        """Close the dashboard Front Camera capture before a policy run opens the device.

        Returns False when the capture was not confirmed closed within ``timeout``.
        """
        self._front_policy_hold = hold
        self._policy_active_cache = (0.0, False)
        feed = self._feeds["front_camera"]
        if not hold or not feed.device or feed.thread is None:
            return True
        return feed.released.wait(timeout)

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
            # Clear before checking the hold so a concurrent hold request waits for this capture.
            feed.released.clear()
            if self._held_for_policy(feed):
                feed.released.set()
                with feed.lock:
                    feed.detections = []  # no live frame backs the last detections while held
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.STOPPED
                    feed.error_message = (
                        "학습 정책 실행이 Front Camera를 사용하는 동안 관제 화면 수신을 멈췄습니다"
                    )
                self._closed.wait(0.25)
                continue
            resolved_device = os.path.realpath(device)
            video_match = re.fullmatch(r"/dev/video(\d+)", resolved_device)
            if not os.path.exists(resolved_device):
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = f"카메라 장치 경로가 없습니다: {device}"
                feed.released.set()
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
                feed.released.set()
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
                feed.released.set()
                self._closed.wait(2)
                continue

            try:
                # Both physical cameras share a USB 2.0 hub. Request compressed
                # MJPEG explicitly so simultaneous streams do not reserve the
                # bus bandwidth required by uncompressed YUYV.
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, feed.capture_size[0])
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, feed.capture_size[1])
                capture.set(cv2.CAP_PROP_FPS, 30)
            except Exception as exc:
                release_error = self._release_capture(capture)
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = f"카메라 형식 설정 실패: {exc}"
                    if release_error:
                        feed.error_message += f" · {release_error}"
                feed.released.set()
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
                    if self._held_for_policy(feed):
                        break
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
                feed.released.set()
            if self._held_for_policy(feed) and not capture_failure:
                continue  # the next pass reports the hold and waits without an error state
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
