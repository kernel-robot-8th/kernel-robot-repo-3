"""Camera capture ownership and MJPEG frame access for the dashboard.

With two arms there are six cameras (docs/implementation/runbook.md, 카메라 6대):
- head_camera: the ceiling camera that checks Pinky's parking; the parking launch opens it, this reads its topic
- shelf_camera: the shelf camera that counts stock
- arm01_gripper / arm02_gripper: each arm's gripper (joint) camera
- arm01_overhead / arm02_overhead: each arm's overhead camera, an input of the ACT policies recorded on that arm
The arm cameras come from arm_devices.py (CONTROL_STATION_ARM01_*_CAMERA, ARM02_*). A gripper camera is streamed
like the older joint camera; an overhead camera belongs to the policy and is only checked for presence unless
CONTROL_STATION_OVERHEAD_CAMERA_PREVIEW=1. While a policy runs, the feeds on its devices are closed (V4L2 lets only
one program stream a device). All six share one USB controller, so every device is opened as MJPG.
"""

from __future__ import annotations

import hashlib
from importlib.metadata import version as package_version
import math
import os
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable, Mapping

from control_station.arm_devices import ROLE_LABELS, ArmDevices, arm_devices, env_prefix
from control_station.arms import ArmConfigError, load_arms
from control_station.models import (
    CameraConnectionState,
    CameraSnapshot,
    CameraStreamState,
    FrameFreshness,
)


PIXEL_TRANSFORM_ID = "opencv-v4l2-capture-no-app-geometry-v1"


def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass
class _CameraFeed:
    camera_id: str
    position: str
    device: str | None
    frame: bytes | None = None
    frame_width: int | None = None
    frame_height: int | None = None
    frame_sha256: str | None = None
    connection_state: CameraConnectionState = CameraConnectionState.UNKNOWN
    stream_state: CameraStreamState = CameraStreamState.UNKNOWN
    last_frame_at: str | None = None
    last_frame_monotonic: float | None = None
    frame_times: list[float] = field(default_factory=list)
    error_message: str | None = None
    sequence: int = 0
    detector: "_YoloDetector | None" = None
    robot_id: str | None = "arm-01"
    detections: list[dict[str, object]] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)
    thread: threading.Thread | None = None
    # Set while another program (the arm worker running a LeRobot policy) needs this V4L2 device: the capture
    # loop closes the device, sets `handed_over`, and reopens it once `lent` is cleared.
    lent: threading.Event = field(default_factory=threading.Event)
    handed_over: threading.Event = field(default_factory=threading.Event)
    lent_reason: str = ""
    # Set while this process holds no open capture, so another process may open the device.
    released: threading.Event = field(default_factory=threading.Event)
    # Requested capture size. Some sensors crop the field of view at 4:3, so a wider mode may show more.
    capture_size: tuple[int, int] = (640, 480)
    role: str | None = None
    opened_by: str = "control_station"
    setting: str | None = None
    # Only check that the device exists; the policy opens it (an overhead camera without preview).
    watch_only: bool = False


class _YoloDetector:
    """Run YOLO on camera frames and draw the detections.

    The detections are shown on the dashboard, placed into shelf slots (shelf_slot_zones.py) and counted for the
    order flow's stock and load checks. They are camera evidence only; they never move a robot by themselves.
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

    def _predict(self, image):  # type: ignore[no-untyped-def]
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
                "frame_width": int(width),
                "frame_height": int(height),
            }
            for box in result.boxes
        ]
        return result.plot(), detections

    def annotate(self, image):  # type: ignore[no-untyped-def]
        with self._inference_lock:
            if self.state != "ready" or self._model is None:
                raise RuntimeError("YOLO detector is no longer ready")
            return self._predict(image)


class UnconfiguredArmCameraStreamManager:
    """Report the camera slots without opening any physical device."""

    def snapshots(self) -> list[CameraSnapshot]:
        slots = [("head_camera", None, "HEAD", "핑키 정차 확인 (천장)", "CONTROL_STATION_HEAD_CAMERA"),
                 ("shelf_camera", None, "SHELF", "선반 재고", "CONTROL_STATION_SHELF_CAMERA")]
        for arm_id, label in (("arm-01", "왼쪽 팔"), ("arm-02", "오른쪽 팔")):
            for role in ("GRIPPER", "OVERHEAD"):
                slots.append((_arm_camera_id(arm_id, role), arm_id, role, _arm_role(label, role),
                              f"{env_prefix(arm_id)}{role}_CAMERA"))
        return [
            CameraSnapshot(
                id=camera_id, robot_id=robot_id, position=position,
                connection_state=CameraConnectionState.UNKNOWN, stream_state=CameraStreamState.UNKNOWN,
                error_message="카메라 장치 경로 미설정", role=role, link_label="연결 전", setting=setting,
                opened_by="policy" if position == "OVERHEAD" else "control_station",
            )
            for camera_id, robot_id, position, role, setting in slots
        ]

    def frame_for(self, camera_id: str) -> bytes | None:
        return None

    def close(self) -> None:
        return


def _arm_camera_id(arm_id: str, role: str) -> str:
    """arm-01, GRIPPER -> arm01_gripper"""
    return f"{arm_id.replace('-', '')}_{role.lower()}"


def _arm_role(label: str, role: str) -> str:
    return f"{label} {ROLE_LABELS[role]}" + (" (ACT 정책)" if role == "OVERHEAD" else "")


def _load_arm_devices() -> dict[str, ArmDevices]:
    try:
        return arm_devices(load_arms().arms)
    except ArmConfigError:
        return {}


# CONTROL_STATION_*_CAMERA=ros:<image topic> reads that camera from a ROS camera driver node instead of V4L2.
ROS_SOURCE_PREFIX = "ros:"


class _RosImageSource:
    """Latest frame of one sensor_msgs Image or CompressedImage topic, as BGR with its capture time."""

    def __init__(self, topic: str) -> None:
        import numpy as np
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, Image

        raw_domain = os.environ.get("CONTROL_STATION_MOBILE_DOMAIN", os.environ.get("ROS_DOMAIN_ID"))
        if raw_domain is None:
            raise RuntimeError("ROS domain ID(CONTROL_STATION_MOBILE_DOMAIN 또는 ROS_DOMAIN_ID)가 필요합니다")
        self._np = np
        self._compressed = topic.endswith("/compressed")
        self._latest: tuple[object, str, float] | None = None
        self._ready = threading.Condition()
        self._context = Context()
        self._context.init(domain_id=int(raw_domain))
        name = "control_station_camera_" + re.sub(r"[^A-Za-z0-9_]", "_", topic.strip("/"))
        self._node = Node(name[:200], context=self._context)
        # Best effort subscribes to both reliable and best-effort drivers and never queues stale frames.
        self._node.create_subscription(
            CompressedImage if self._compressed else Image, topic, self._on_image, qos_profile_sensor_data
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, name=f"ros-camera-{topic}", daemon=True)
        self._thread.start()

    def _to_bgr(self, message):  # type: ignore[no-untyped-def]
        import cv2

        np = self._np
        if self._compressed:
            return cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        pixels = np.frombuffer(message.data, dtype=np.uint8)
        if message.encoding in ("bgr8", "rgb8"):
            image = pixels.reshape(message.height, message.step)[:, : message.width * 3].reshape(
                message.height, message.width, 3
            )
            return image[:, :, ::-1].copy() if message.encoding == "rgb8" else image.copy()
        if message.encoding == "mono8":
            gray = pixels.reshape(message.height, message.step)[:, : message.width]
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"지원하지 않는 영상 encoding: {message.encoding}")

    def _on_image(self, message) -> None:  # type: ignore[no-untyped-def]
        received_monotonic = time.monotonic()
        received_wall = time.time()
        image = self._to_bgr(message)
        if image is None:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        # A frame counts as captured at its stamp but never later than it arrived, like the C++ parking node.
        age = max(0.0, received_wall - stamp) if stamp > 0 else 0.0
        captured_wall = received_wall - age
        captured_at = datetime.fromtimestamp(captured_wall, tz=timezone.utc).isoformat(timespec="milliseconds")
        with self._ready:
            self._latest = (image, captured_at, received_monotonic - age)
            self._ready.notify_all()

    def next_frame(self, timeout_s: float) -> tuple[object, str, float] | None:
        with self._ready:
            if self._latest is None:
                self._ready.wait(timeout_s)
            frame, self._latest = self._latest, None
        return frame

    def close(self) -> None:
        with self._ready:
            self._ready.notify_all()
        self._executor.shutdown()
        self._node.destroy_node()
        self._context.try_shutdown()
        self._thread.join(timeout=2)


class OpenCVArmCameraStreamManager:
    """Own configured V4L2 devices once and expose their latest JPEG frames."""

    def __init__(
        self,
        joint_enabled: bool = True,
        policy_active: Callable[[], bool] | None = None,
        *,
        policy_devices: Callable[[], list[str]] | None = None,
        arm_cameras: Mapping[str, ArmDevices] | None = None,
    ) -> None:
        # The learned-policy client opens the Front Camera itself, and V4L2 lets only one
        # process stream it, so the dashboard gives it up while a policy run is requested or alive.
        self._policy_active = policy_active
        self._front_policy_hold = False
        self._policy_active_cache = (0.0, False)
        # Devices of the live policy run (any arm) and devices held for a run about to start: closed meanwhile.
        self._policy_devices = policy_devices
        self._policy_devices_cache: tuple[float, frozenset[str]] = (0.0, frozenset())
        self._held_devices: frozenset[str] = frozenset()
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
            "joint_camera", "JOINT", (joint_device or None) if joint_enabled else None,
            role="그리퍼 캠 (예전 단일 팔 설정)", setting="CONTROL_STATION_JOINT_CAMERA",
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
                role="핑키 정차 확인 (천장)",
                opened_by="ros_node" if head_device.startswith(ROS_SOURCE_PREFIX) else "control_station",
                setting="CONTROL_STATION_HEAD_CAMERA",
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
                role="정면 캠 (예전 정책 위 카메라 설정)",
                setting="CONTROL_STATION_FRONT_CAMERA",
                detector=(
                    _YoloDetector(front_weights, "CONTROL_STATION_FRONT_YOLO")
                    if front_weights or front_prompts
                    else None
                ),
            ),
        }
        # Optional shelf camera looking at the shelf and arms from the front; its id matches the
        # shelf_camera intrinsic calibration. Its YOLO detections are shown like the other feeds, but it has
        # no Pick & Place task observation step.
        shelf_device = os.environ.get("CONTROL_STATION_SHELF_CAMERA", "").strip()
        if shelf_device:
            shelf_weights = os.environ.get("CONTROL_STATION_SHELF_YOLO_WEIGHTS", "").strip()
            shelf_prompts = os.environ.get("CONTROL_STATION_SHELF_YOLO_WORLD_PROMPTS", "").strip()
            shelf_feed = _CameraFeed(
                "shelf_camera",
                "SHELF",
                shelf_device,
                detector=(
                    _YoloDetector(shelf_weights, "CONTROL_STATION_SHELF_YOLO")
                    if shelf_weights or shelf_prompts
                    else None
                ),
                robot_id=None,  # a fixed view of the shelf, not a camera on the arm
                role="선반 재고",
                opened_by="ros_node" if shelf_device.startswith(ROS_SOURCE_PREFIX) else "control_station",
                setting="CONTROL_STATION_SHELF_CAMERA",
            )
            self._feeds["shelf_camera"] = shelf_feed
        front_feed = self._feeds["front_camera"]
        if not front_feed.device:
            front_feed.error_message = "Front Camera 장치 경로 미설정 (CONTROL_STATION_FRONT_CAMERA)"
        self._add_arm_feeds(_load_arm_devices() if arm_cameras is None else arm_cameras, joint_enabled)
        by_device: dict[str, list[_CameraFeed]] = {}
        for feed in self._feeds.values():
            if feed.device:
                by_device.setdefault(os.path.realpath(feed.device), []).append(feed)
        for shared in by_device.values():
            if len(shared) < 2:
                continue
            labels = "와 ".join(feed.role or f"{feed.position.title()} Camera" for feed in shared)
            settings = ", ".join(feed.setting or feed.camera_id for feed in shared)
            message = (
                f"{labels} 경로가 같은 V4L2 장치로 연결됩니다. 실제 장착을 확인한 뒤 {settings}를 서로 다른 "
                "장치로 설정하세요. 같은 모델 카메라는 /dev/v4l/by-id 이름이 겹칠 수 있으니 꽂은 USB 자리로 정해지는 "
                "/dev/v4l/by-path 경로를 쓰세요."
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
        self._ros_sources: list[_RosImageSource] = []
        if any(feed.device and feed.device.startswith(ROS_SOURCE_PREFIX) for feed in self._feeds.values()):
            try:
                # Import here, before the capture threads: rclpy's first import is not safe from two threads
                # (another thread may be importing it for the parking node at the same time).
                import rclpy.node  # noqa: F401
            except ImportError:
                pass  # each ros: feed reports the missing ROS environment itself
        for feed in self._feeds.values():
            if feed.device and not feed.watch_only:
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

    def _add_arm_feeds(self, arm_cameras: Mapping[str, ArmDevices], joint_enabled: bool) -> None:
        """One feed per arm camera. An older single-arm feed on the same device (joint/front) gives way to it."""
        preview = os.environ.get("CONTROL_STATION_OVERHEAD_CAMERA_PREVIEW", "").strip().lower() in {"1", "true", "yes"}
        added: list[_CameraFeed] = []
        for devices in arm_cameras.values():
            for role in ("GRIPPER", "OVERHEAD"):
                device = devices.cameras.get(role)
                streamed = joint_enabled if role == "GRIPPER" else preview
                feed = _CameraFeed(
                    _arm_camera_id(devices.arm_id, role), role, device or None, robot_id=devices.arm_id,
                    role=_arm_role(devices.label, role), opened_by="control_station" if streamed else "policy",
                    setting=f"{env_prefix(devices.arm_id)}{role}_CAMERA", watch_only=bool(device) and not streamed,
                )
                if not device:
                    feed.error_message = f"장치 경로 미설정 ({feed.setting})"
                added.append(feed)
        taken = {os.path.realpath(feed.device) for feed in added if feed.device}
        for legacy_id in ("joint_camera", "front_camera"):
            legacy = self._feeds.get(legacy_id)
            if legacy is not None and legacy.device and os.path.realpath(legacy.device) in taken:
                del self._feeds[legacy_id]
        legacy_joint = self._feeds.get("joint_camera")
        if taken and legacy_joint is not None and not legacy_joint.device:
            del self._feeds["joint_camera"]  # an empty older slot next to the arms' own gripper cameras
        self._feeds.update((feed.camera_id, feed) for feed in added)

    def lend_devices(self, devices: list[str], reason: str, timeout_s: float = 3.0) -> list[str]:
        """Close the V4L2 feeds that open any of `devices` so another program can open them. Returns the ids of
        the feeds that were closed in time; a feed still open after `timeout_s` is left lent and not listed."""
        wanted = {os.path.realpath(device) for device in devices if device and not device.startswith(ROS_SOURCE_PREFIX)}
        feeds = [
            feed for feed in self._feeds.values()
            if feed.thread is not None and feed.device and not feed.device.startswith(ROS_SOURCE_PREFIX)
            and os.path.realpath(feed.device) in wanted
        ]
        for feed in feeds:
            feed.lent_reason = reason
            feed.lent.set()
        deadline = time.monotonic() + timeout_s
        return [feed.camera_id for feed in feeds if feed.handed_over.wait(max(0.0, deadline - time.monotonic()))]

    def take_back_devices(self) -> list[str]:
        """Reopen every lent feed."""
        lent = [feed for feed in self._feeds.values() if feed.lent.is_set()]
        for feed in lent:
            feed.lent.clear()
        return [feed.camera_id for feed in lent]

    @staticmethod
    def _release_capture(capture) -> str | None:  # type: ignore[no-untyped-def]
        try:
            capture.release()
        except Exception as exc:
            return f"카메라 자원 해제 실패: {exc}"
        return None

    def _held_for_policy(self, feed: _CameraFeed) -> bool:
        """Return whether the learned-policy client should own this camera instead."""
        if not feed.device or feed.device.startswith(ROS_SOURCE_PREFIX):
            return False
        path = os.path.realpath(feed.device)
        if path in self._held_devices:
            return True
        now = time.monotonic()
        if feed.camera_id == "front_camera":
            # The older setting: the Front Camera is given up whenever any policy run is alive.
            if self._front_policy_hold:
                return True
            if self._policy_active is not None:
                checked_at, active = self._policy_active_cache
                if now - checked_at >= 0.2:
                    try:
                        active = bool(self._policy_active())
                    except Exception:
                        active = False
                    self._policy_active_cache = (now, active)
                if active:
                    return True
        if self._policy_devices is None:
            return False
        checked_at, devices = self._policy_devices_cache
        if now - checked_at >= 0.2:
            try:
                devices = frozenset(os.path.realpath(device) for device in self._policy_devices() if device)
            except Exception:
                devices = frozenset()
            self._policy_devices_cache = (now, devices)
        return path in devices

    def hold_for_policy(self, devices: list[str], hold: bool, timeout: float = 5.0) -> bool:
        """Close the feeds on `devices` (and the older Front Camera) before a policy run opens them.

        Returns False when a capture was not confirmed closed within ``timeout``."""
        self._front_policy_hold = hold
        self._policy_active_cache = (0.0, False)
        self._policy_devices_cache = (0.0, frozenset())
        self._held_devices = frozenset(
            os.path.realpath(device) for device in devices if device and not device.startswith(ROS_SOURCE_PREFIX)
        ) if hold else frozenset()
        if not hold:
            return True
        deadline = time.monotonic() + timeout
        feeds = [
            feed for feed in self._feeds.values()
            if feed.thread is not None and feed.device and (
                feed.camera_id == "front_camera" or os.path.realpath(feed.device) in self._held_devices)
        ]
        return all(feed.released.wait(max(0.0, deadline - time.monotonic())) for feed in feeds)

    def hold_front_for_policy(self, hold: bool, timeout: float = 5.0) -> bool:
        """Close the dashboard Front Camera capture before a policy run opens the device (older single-arm path)."""
        return self.hold_for_policy([], hold, timeout)

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

        if feed.device and feed.device.startswith(ROS_SOURCE_PREFIX):
            self._ros_capture_loop(feed, feed.device.removeprefix(ROS_SOURCE_PREFIX), cv2)
            return

        while not self._closed.is_set():
            device = feed.device
            if not device:
                return
            if feed.lent.is_set():
                # The device is closed here; wait until it is given back.
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.STOPPED
                    feed.error_message = feed.lent_reason
                feed.released.set()
                feed.handed_over.set()
                while feed.lent.is_set() and not self._closed.is_set():
                    self._closed.wait(0.2)
                feed.handed_over.clear()
                continue
            # Clear before checking the hold so a concurrent hold request waits for this capture.
            feed.released.clear()
            if self._held_for_policy(feed):
                feed.released.set()
                with feed.lock:
                    feed.detections = []  # no live frame backs the last detections while held
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.STOPPED
                    feed.error_message = (
                        "학습 정책 실행이 이 카메라를 쓰는 동안 관제 화면 수신을 멈췄습니다"
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
                while not self._closed.is_set() and not feed.lent.is_set():
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
                    self._publish_frame(
                        feed,
                        image,
                        datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                        time.monotonic(),
                        cv2,
                    )
            except Exception as exc:
                capture_failure = f"카메라 프레임 처리 실패: {exc}"
            finally:
                release_error = self._release_capture(capture)
                capture_failure = capture_failure or release_error
                feed.released.set()
            if self._held_for_policy(feed) and not capture_failure:
                continue  # the next pass reports the hold and waits without an error state
            if feed.lent.is_set():
                continue  # closed on purpose; the top of the loop reports it and waits
            if not self._closed.is_set():
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = (
                        capture_failure or f"카메라 프레임 수신 중단: {feed.device}"
                    )
                self._closed.wait(2)

    def _publish_frame(self, feed: _CameraFeed, image, captured_at: str, captured_monotonic: float, cv2) -> None:  # type: ignore[no-untyped-def]
        """Run YOLO on one BGR frame and make it the feed's latest JPEG."""
        frame_sha256 = hashlib.sha256(image.tobytes(order="C")).hexdigest()
        detections: list[dict[str, object]] = []
        detector = feed.detector
        if detector is not None and detector.state == "ready":
            try:
                image, detections = detector.annotate(image)
            except Exception as exc:  # keep streaming raw frames if inference fails
                detector.state, detector.error_message = "error", f"YOLO 추론 실패: {exc}"
        encoded, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not encoded:
            return
        frame_height, frame_width = image.shape[:2]
        with feed.lock:
            feed.sequence += 1
            frame_sequence = feed.sequence
            feed.frame_width = int(frame_width)
            feed.frame_height = int(frame_height)
            feed.frame_sha256 = frame_sha256
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

    def _ros_capture_loop(self, feed: _CameraFeed, topic: str, cv2) -> None:  # type: ignore[no-untyped-def]
        """Take the feed's frames from a ROS image topic that a camera driver node publishes.

        The driver owns the device, so the control station and the C++ parking node can share one camera
        (docs/plans/pinky-parking-ros2-node.md). The geometry is the same 640x480 MJPEG capture with no crop or
        flip, so detections keep PIXEL_TRANSFORM_ID. A topic ending in /compressed is decoded here; it is the one
        to use across processes, since raw 640x480 images were mostly dropped between processes on 2026-11-02.
        """
        try:
            source = _RosImageSource(topic)
        except Exception as exc:
            with feed.lock:
                feed.connection_state = CameraConnectionState.DISCONNECTED
                feed.stream_state = CameraStreamState.ERROR
                feed.error_message = f"ROS 영상 구독 실패 ({topic}): {exc}"
            return
        self._ros_sources.append(source)
        with feed.lock:
            feed.connection_state = CameraConnectionState.CONNECTED
            feed.stream_state = CameraStreamState.NO_FRAMES
            feed.error_message = None
        while not self._closed.is_set():
            frame = source.next_frame(timeout_s=2.0)
            if frame is None:
                if not self._closed.is_set():
                    with feed.lock:
                        feed.stream_state = CameraStreamState.NO_FRAMES
                        feed.error_message = f"ROS 영상 토픽에서 2초 동안 프레임이 없습니다: {topic}"
                continue
            image, captured_at, captured_monotonic = frame
            try:
                self._publish_frame(feed, image, captured_at, captured_monotonic, cv2)
            except Exception as exc:
                with feed.lock:
                    feed.error_message = f"카메라 프레임 처리 실패: {exc}"

    @staticmethod
    def _watch(feed: _CameraFeed) -> None:
        """A policy-owned camera: only whether the device is plugged in."""
        if not feed.device:
            return  # refused at start (e.g. the same device as another camera); the error says why
        present = os.path.exists(os.path.realpath(feed.device))
        feed.connection_state = CameraConnectionState.CONNECTED if present else CameraConnectionState.DISCONNECTED
        feed.stream_state = CameraStreamState.STOPPED if present else CameraStreamState.ERROR
        how_to_view = ("CONTROL_STATION_OVERHEAD_CAMERA_PREVIEW=1" if feed.position == "OVERHEAD"
                       else "팔 원격 조작 CONTROL_STATION_ARM_ENABLED=1")
        feed.error_message = (
            f"정책 실행 때 정책이 직접 엽니다 (관제 화면에서 보려면 {how_to_view})"
            if present else f"카메라 장치 경로가 없습니다: {feed.device}"
        )

    def snapshots(self) -> list[CameraSnapshot]:
        result = []
        for feed in self._feeds.values():
            if feed.camera_id == "front_camera" and not feed.device:
                continue  # not configured on this PC: not shown (set CONTROL_STATION_FRONT_CAMERA to use it)
            with feed.lock:
                if feed.watch_only:
                    self._watch(feed)
                configured = feed.device is not None
                # Closed for a policy run still counts as plugged in.
                linked = configured and (
                    feed.connection_state == CameraConnectionState.CONNECTED
                    or (feed.stream_state == CameraStreamState.STOPPED and os.path.exists(os.path.realpath(feed.device)))
                )
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
                        robot_id=feed.robot_id,
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
                        role=feed.role,
                        opened_by=feed.opened_by,
                        link_label="연결됨" if linked else "연결 전",
                        setting=feed.setting,
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
        for source in self._ros_sources:
            source.close()
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
