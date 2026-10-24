"""Camera capture ownership and MJPEG frame access for the dashboard."""

from __future__ import annotations

import os
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
    lock: threading.Lock = field(default_factory=threading.Lock)
    thread: threading.Thread | None = None


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

    def __init__(self) -> None:
        joint_device = os.environ.get(
            "CONTROL_STATION_JOINT_CAMERA",
            os.environ.get("CONTROL_STATION_ARM_CAMERA", "/dev/video4"),
        ).strip()
        head_device = os.environ.get(
            "CONTROL_STATION_HEAD_CAMERA",
            "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0",
        ).strip()
        self._feeds = {
            "head_camera": _CameraFeed("head_camera", "HEAD", head_device or None),
            "joint_camera": _CameraFeed("joint_camera", "JOINT", joint_device or None),
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
                feed.error_message = "OpenCV 미설치: LeRobot 가상환경에서 실행하세요"
            return

        while not self._closed.is_set():
            capture = cv2.VideoCapture(feed.device, cv2.CAP_V4L2)
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
                    encoded, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not encoded:
                        continue
                    now = time.monotonic()
                    with feed.lock:
                        feed.frame = jpeg.tobytes()
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
                        error_message=feed.error_message or (None if configured else "카메라 장치 경로 미설정"),
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
