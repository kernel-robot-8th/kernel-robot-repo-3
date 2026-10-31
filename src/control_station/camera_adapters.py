"""Camera capture ownership and MJPEG frame access for the dashboard."""

from __future__ import annotations

import os
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Protocol

from control_station.models import (
    CameraConnectionState,
    CameraSnapshot,
    CameraStreamState,
    FrameFreshness,
)


class CameraStreamManager(Protocol):
    def snapshots(self) -> list[CameraSnapshot]: ...

    def frame_for(self, camera_id: str) -> bytes | None: ...

    def close(self) -> None: ...


@dataclass
class _CameraFeed:
    camera_id: str
    position: str
    device: str | None
    frame: bytes | None = None
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
    """Draw YOLO detections on camera frames for display only.

    Results are shown on the dashboard and are not used as a robot-control signal.
    """

    def __init__(self, weights: str) -> None:
        self.weights = weights
        self.confidence = float(os.environ.get("CONTROL_STATION_HEAD_YOLO_CONF", "0.25"))
        self.device = os.environ.get("CONTROL_STATION_HEAD_YOLO_DEVICE", "").strip() or None
        self.state = "loading"
        self.error_message: str | None = None
        self._model = None

    def load(self) -> bool:
        if not os.path.isfile(self.weights):
            self.state, self.error_message = "error", f"YOLO 가중치 파일이 없습니다: {self.weights}"
            return False
        try:
            from ultralytics import YOLO
        except ImportError:
            self.state = "error"
            self.error_message = "ultralytics 미설치: requirements-vision.txt를 설치하세요"
            return False
        self._model = YOLO(self.weights)
        self.state = "ready"
        return True

    def annotate(self, image):  # type: ignore[no-untyped-def]
        result = self._model.predict(
            source=image, conf=self.confidence, imgsz=640, device=self.device, verbose=False
        )[0]
        detections = [
            {
                "class_name": result.names[int(box.cls.item())],
                "confidence": round(float(box.conf.item()), 3),
                "center_px": [round(value, 1) for value in box.xywh[0][:2].tolist()],
            }
            for box in result.boxes
        ]
        return result.plot(), detections


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

    def close(self) -> None:
        return


class OpenCVArmCameraStreamManager:
    """Own configured V4L2 devices once and expose their latest JPEG frames."""

    def __init__(self, joint_enabled: bool = True) -> None:
        joint_device = os.environ.get(
            "CONTROL_STATION_JOINT_CAMERA",
            os.environ.get("CONTROL_STATION_ARM_CAMERA", "/dev/video4"),
        ).strip()
        head_device = os.environ.get(
            "CONTROL_STATION_HEAD_CAMERA",
            "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0",
        ).strip()
        head_weights = os.environ.get("CONTROL_STATION_HEAD_YOLO_WEIGHTS", "").strip()
        # The joint camera sees the same objects, so it reuses the head model unless overridden.
        joint_weights = os.environ.get("CONTROL_STATION_JOINT_YOLO_WEIGHTS", head_weights).strip()
        joint_feed = _CameraFeed(
            "joint_camera",
            "JOINT",
            (joint_device or None) if joint_enabled else None,
            detector=_YoloDetector(joint_weights) if joint_enabled and joint_weights else None,
        )
        if not joint_enabled:
            joint_feed.error_message = "팔 카메라 스트림을 사용하려면 --arm 모드로 실행하세요"
        self._feeds = {
            "head_camera": _CameraFeed(
                "head_camera",
                "HEAD",
                head_device or None,
                detector=_YoloDetector(head_weights) if head_weights else None,
            ),
            "joint_camera": joint_feed,
        }
        self._closed = threading.Event()
        for feed in self._feeds.values():
            if feed.device:
                feed.thread = threading.Thread(
                    target=self._capture_loop,
                    args=(feed,),
                    name=f"camera-{feed.camera_id}",
                    daemon=True,
                )
                feed.thread.start()

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
            capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = f"카메라를 열 수 없습니다: {feed.device} (다른 프로그램 점유 여부 확인)"
                self._closed.wait(2)
                continue

            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            capture.set(cv2.CAP_PROP_FPS, 30)
            with feed.lock:
                feed.connection_state = CameraConnectionState.CONNECTED
                feed.stream_state = CameraStreamState.NO_FRAMES
                feed.error_message = None

            failed_reads = 0
            try:
                while not self._closed.is_set():
                    ok, image = capture.read()
                    if not ok:
                        failed_reads += 1
                        if failed_reads >= 10:
                            break
                        self._closed.wait(0.05)
                        continue
                    failed_reads = 0
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
                    now = time.monotonic()
                    with feed.lock:
                        feed.frame = jpeg.tobytes()
                        feed.detections = detections
                        feed.last_frame_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
                        feed.last_frame_monotonic = now
                        feed.frame_times.append(now)
                        feed.frame_times = [stamp for stamp in feed.frame_times if now - stamp <= 2]
                        feed.sequence += 1
                        feed.connection_state = CameraConnectionState.CONNECTED
                        feed.stream_state = CameraStreamState.STREAMING
                        feed.error_message = None
            finally:
                capture.release()
            if not self._closed.is_set():
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = f"카메라 프레임 수신 중단: {feed.device}"
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
                        frame_rate=frame_rate,
                        error_message=(
                            feed.error_message
                            or (feed.detector.error_message if feed.detector else None)
                            or (None if configured else "카메라 장치 경로 미설정")
                        ),
                        detector_state=feed.detector.state if feed.detector else None,
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

    def sequence_for(self, camera_id: str) -> int:
        feed = self._feeds.get(camera_id)
        if feed is None:
            return -1
        with feed.lock:
            return feed.sequence

    def close(self) -> None:
        self._closed.set()
        for feed in self._feeds.values():
            if feed.thread is not None:
                feed.thread.join(timeout=2)
