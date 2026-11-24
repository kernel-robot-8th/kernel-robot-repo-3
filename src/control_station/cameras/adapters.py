"""카메라 장치를 잡고(capture 소유) 대시보드에 MJPEG 프레임과 YOLO 검출을 내준다.

팔이 두 대라 카메라는 여섯 대다(docs/implementation/runbook.md, 카메라 6대):
- head_camera: 핑키 주차를 확인하는 천장 카메라. 주차 launch가 장치를 열고, 여기서는 그 토픽을 읽는다
- shelf_camera: 재고를 세는 선반 카메라
- arm01_gripper / arm02_gripper: 팔마다 그리퍼(관절) 카메라
- arm01_overhead / arm02_overhead: 팔마다 위 카메라. 그 팔에서 녹화한 ACT 정책의 입력이다
팔 카메라는 arms/config.py(CONTROL_STATION_ARM01_*_CAMERA, ARM02_*)에서 온다. 그리퍼 카메라는 예전 관절 카메라처럼
스트리밍한다. 위 카메라는 정책 몫이라 CONTROL_STATION_OVERHEAD_CAMERA_PREVIEW=1이 아니면 꽂혀 있는지만 확인한다.
정책이 도는 동안에는 그 정책이 쓰는 장치의 피드를 닫는다(V4L2는 한 장치를 한 프로그램만 스트리밍하게 한다).
여섯 대가 USB 컨트롤러 하나를 같이 쓰므로 모든 장치를 MJPG로 연다.

파일 구성(위에서 아래로):
1) 공통 도구와 탑재 검출 합치기: `_sha256_file`, `_merge_load_detections`
2) 피드 하나의 상태: `_CameraFeed`
3) YOLO 검출기: `_YoloDetector`
4) 장치를 열지 않는 관리자와 팔 카메라 이름: `UnconfiguredArmCameraStreamManager`, `_arm_camera_id`
5) ROS 영상 토픽 소스(`ros:` 장치): `_RosImageSource`
6) 카메라 관리자 `OpenCVArmCameraStreamManager`
   6a) 피드 구성: 환경 변수 → 피드, 같은 장치 겹침 검사, 캡처 스레드 시작
   6b) 정책 실행에 장치 빌려주기·되찾기
   6c) V4L2 캡처 루프
   6d) 프레임 처리: ArUco 마커, YOLO, JPEG
   6e) ROS 토픽 캡처 루프
   6f) 스냅샷과 프레임 내주기, 종료

스레드: 피드마다 캡처 스레드("camera-<id>", 데몬)가 하나씩 돌며 장치(또는 ROS 토픽)에서 프레임을 읽고 YOLO를 돌려,
결과를 `feed.lock` 아래에서 피드에 쓴다. HTTP 스레드는 같은 잠금 아래에서 snapshots()/frame_for()로 읽는다. ROS
소스는 자기 rclpy executor를 따로 데몬 스레드에서 돌리고, 최신 프레임 하나만 Condition으로 캡처 스레드에 넘긴다.

위치: app.py가 만들어 server.camera_manager로 둔다. 스냅샷은 api/cameras.py(카드, MJPEG 스트림), cameras/counts.py
(개수 세기), cameras/shelf_slots.py(칸 영역)가 쓰고, api/arm.py가 정책 실행 전후로 hold_for_policy / lend_devices를
부른다.
"""

from __future__ import annotations

import hashlib
import math
import os
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from importlib.metadata import version as package_version
from typing import Callable, Mapping

from control_station.arms.config import ROLE_LABELS, ArmConfigError, ArmDevices, arm_devices, env_prefix, load_arms
from control_station.storage.models import CameraConnectionState, CameraSnapshot, CameraStreamState, FrameFreshness

# 검출 좌표가 어떤 픽셀 변환을 거쳤는지 나타내는 ID: OpenCV V4L2로 받은 그대로이고 앱에서 자르거나 뒤집지 않는다.
# 칸 영역(shelf_slots.py)은 보정 때와 이 값이 같을 때만 쓴다.
PIXEL_TRANSFORM_ID = "opencv-v4l2-capture-no-app-geometry-v1"


# ──── 1) 공통 도구와 탑재 검출 합치기 ────────────────────────────────────────────────────────────


def _sha256_file(path: str) -> str:
    """파일의 SHA-256(1 MiB씩 읽음). 어떤 가중치 파일을 올렸는지 기록하는 데 쓴다."""
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _same_load_object(first: dict[str, object], second: dict[str, object]) -> bool:
    """두 모델의 검출이 같은 물건인지 박스 IoU로 본다. 같은 클래스면 0.35, 클래스가 다르면(두 모델이 이름을 다르게
    붙임) 더 많이 겹쳐야 하므로 0.55 이상. 한쪽이 pinky/car(핑키 몸체)인데 클래스가 다르면 같은 물건이 아니다."""
    first_name, second_name = first["class_name"], second["class_name"]
    if first_name != second_name and ({first_name, second_name} & {"pinky", "car"}):
        return False
    ax1, ay1, ax2, ay2 = first["bbox_xyxy_px"]
    bx1, by1, bx2, by2 = second["bbox_xyxy_px"]
    intersection = max(0, min(ax2, bx2) - max(ax1, bx1)) * max(0, min(ay2, by2) - max(ay1, by1))
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - intersection
    threshold = 0.35 if first_name == second_name else 0.55
    return union > 0 and intersection / union >= threshold


def _merge_load_detections(
    dedicated: list[dict[str, object]], general: list[dict[str, object]]
) -> list[dict[str, object]]:
    """탑재 전용 YOLO(dedicated)와 피드의 일반 YOLO(general) 검출을 합친다. 둘 다 신뢰도 0.5 이상만 쓰고, 일반
    모델 검출은 전용 모델 검출과 겹치지 않는 것만 덧붙인다."""
    # 두 모델이 같은 물건을 보면 전문 모델(탑재 전용)의 클래스와 박스를 쓴다.
    primary = [item for item in dedicated if float(item["confidence"]) >= 0.5]
    additional = [
        item.copy() for item in general if float(item["confidence"]) >= 0.5
        and not any(_same_load_object(item, matched) for matched in primary)
    ]
    return primary + additional


# ──── 2) 피드 하나의 상태 ────────────────────────────────────────────────────────────────────────


@dataclass
class _CameraFeed:
    """카메라 하나의 설정과 최신 상태.

    캡처 스레드가 쓰고 HTTP 스레드가 읽으므로, 프레임·검출·상태 필드는 `lock` 아래에서 다룬다. Event 필드(lent,
    handed_over, released)는 그 자체로 스레드 안전하며 캡처 스레드와 정책 쪽 호출이 주고받는 신호다.
    """

    camera_id: str
    position: str  # HEAD, SHELF, FRONT, GRIPPER, OVERHEAD 등. 해상도 환경 변수 이름에도 쓴다
    device: str | None  # V4L2 경로, `ros:<토픽>`, 또는 None(미설정이거나 열지 않기로 함)
    frame: bytes | None = None  # 최신 JPEG
    frame_width: int | None = None
    frame_height: int | None = None
    frame_sha256: str | None = None  # 그리기 전 원본 BGR 프레임의 SHA-256
    connection_state: CameraConnectionState = CameraConnectionState.UNKNOWN
    stream_state: CameraStreamState = CameraStreamState.UNKNOWN
    last_frame_at: str | None = None  # 캡처 시각(UTC ISO)
    last_frame_monotonic: float | None = None  # 신선도 계산용 monotonic 시각(초)
    frame_times: list[float] = field(default_factory=list)  # 최근 2초의 프레임 시각(fps 계산용)
    error_message: str | None = None
    sequence: int = 0  # 내보낸 프레임 수 = 최신 프레임 번호
    detector: "_YoloDetector | None" = None
    # 핑키 탑재 전용 YOLO(CONTROL_STATION_LOAD_YOLO_WEIGHTS). 적재 카메라 피드에만 붙는다.
    load_detector: "_YoloDetector | None" = None
    load_detections: list[dict[str, object]] = field(default_factory=list)
    robot_id: str | None = "arm-01"  # 이 카메라가 딸린 팔(예전 단일 팔 설정이 기본). 고정 카메라는 None
    detections: list[dict[str, object]] = field(default_factory=list)
    markers: list[dict[str, object]] = field(default_factory=list)  # ArUco 마커(표시용, _find_markers)
    lock: threading.Lock = field(default_factory=threading.Lock)
    thread: threading.Thread | None = None
    # 다른 프로그램(LeRobot 정책을 돌리는 팔 워커)이 이 V4L2 장치를 써야 하는 동안 세워진다. 캡처 루프가 장치를
    # 닫고 `handed_over`를 세우며, `lent`가 지워지면 다시 연다.
    lent: threading.Event = field(default_factory=threading.Event)
    handed_over: threading.Event = field(default_factory=threading.Event)
    lent_reason: str = ""
    # 이 프로세스가 열린 캡처를 갖고 있지 않은 동안 세워진다. 그동안 다른 프로세스가 장치를 열어도 된다.
    released: threading.Event = field(default_factory=threading.Event)
    # 요청하는 캡처 크기(px). 어떤 센서는 4:3에서 화각을 잘라 내므로 더 넓은 모드가 더 많이 보일 수 있다.
    capture_size: tuple[int, int] = (640, 480)
    role: str | None = None  # 화면에 보일 역할 이름
    opened_by: str = "control_station"  # 장치를 여는 쪽: control_station, policy, ros_node
    setting: str | None = None  # 장치 경로를 정하는 환경 변수 이름(오류 안내용)
    # 장치가 있는지만 확인한다. 장치는 정책이 연다(미리보기를 켜지 않은 위 카메라).
    watch_only: bool = False


# ──── 3) YOLO 검출기 ─────────────────────────────────────────────────────────────────────────────


class _YoloDetector:
    """카메라 프레임에 YOLO를 돌리고 검출을 그린다.

    검출은 대시보드에 보이고, 선반 칸에 배정되며(cameras/shelf_slots.py), 주문 흐름의 재고·탑재 확인에서 세어진다.
    검출은 카메라 근거일 뿐이고, 스스로 로봇을 움직이는 일은 없다.

    설정(env_prefix 예: CONTROL_STATION_HEAD_YOLO): {PREFIX}_CONF(기본 0.25), {PREFIX}_WORLD_PROMPTS(있으면 YOLO-World
    열린 어휘 모델), {PREFIX}_DEVICE(추론 장치), {PREFIX}_CLASS_CONF(클래스별 기준). 설정이 틀리면 state를 "error"로
    두고 load()가 아무것도 하지 않는다. 상태: loading → ready 또는 error.
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
        self._inference_lock = threading.RLock()  # 추론은 한 번에 하나씩
        self.class_names: list[str] = []
        self.weights_sha256: str | None = None
        self.confidence_threshold: float | None = None
        self.inference_image_size = 640  # 추론 입력 크기(px, 긴 변)
        self.ultralytics_version: str | None = None
        # 프롬프트는 1~100개, 각 1~80자, 대소문자를 무시하고 중복 없이.
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
        # 클래스별 기준값. 예: {PREFIX}_CLASS_CONF=tissue=0.15 (흰 물티슈 팩은 흰 핑키 쟁반 위에서 점수가 낮게 나온다).
        # 이름을 대지 않은 클래스는 {PREFIX}_CONF를 쓴다.
        self.class_confidence: dict[str, float] = {}
        for part in os.environ.get(f"{env_prefix}_CLASS_CONF", "").split(","):
            name, _, value = part.partition("=")
            if not name.strip():
                continue
            try:
                threshold = float(value)
            except ValueError:
                threshold = math.nan
            if not math.isfinite(threshold) or not 0.0 <= threshold <= 1.0:
                self.state = "error"
                self.error_message = f"{env_prefix}_CLASS_CONF는 '클래스=0~1 숫자'를 쉼표로 이어 써야 합니다 ({part.strip()})"
                break
            self.class_confidence[name.strip()] = threshold

    def load(self) -> bool:
        """가중치를 읽어 모델을 준비한다. 성공하면 state="ready"이고 True.

        피드의 캡처 스레드가 시작할 때 부른다. 그래서 무거운 ultralytics import와 모델 로드가 서버 시작을 막지 않는다.
        """
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
            # 로드 앞뒤의 해시를 비교해, 읽는 도중 파일이 바뀌지 않았음을 확인한다. 그래야 기록하는 weights_sha256이
            # 실제로 올린 모델과 같다.
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

    def _predict(self, image, *, render: bool):  # type: ignore[no-untyped-def]
        """모델을 한 번 돌려 (그린 이미지 또는 원본, 검출 목록)을 준다. 좌표는 원본 프레임의 px.

        모든 기준값 가운데 가장 낮은 값으로 돌린 뒤, 클래스별 기준으로 다시 거른다.
        """
        floor = min([self.confidence, *self.class_confidence.values()])
        result = self._model.predict(
            source=image,
            conf=floor,
            imgsz=self.inference_image_size,
            device=self.device,
            verbose=False,
        )[0]
        if self.class_confidence:  # 박스마다 자기 클래스 기준 이상일 때만 남긴다(그리는 박스도 같이)
            keep = [index for index, box in enumerate(result.boxes)
                    if float(box.conf.item()) >= self.class_confidence.get(result.names[int(box.cls.item())], self.confidence)]
            result = result[keep]
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
        return (result.plot() if render else image), detections

    def annotate(self, image, *, render: bool = True):  # type: ignore[no-untyped-def]
        """잠금 아래에서 추론한다. render=False면 그리지 않고 원본을 돌려준다(탑재 검출용). 준비 전이면 RuntimeError."""
        with self._inference_lock:
            if self.state != "ready" or self._model is None:
                raise RuntimeError("YOLO detector is no longer ready")
            return self._predict(image, render=render)


# ──── 4) 장치를 열지 않는 관리자와 팔 카메라 이름 ──────────────────────────────────────────────────


class UnconfiguredArmCameraStreamManager:
    """물리 장치를 하나도 열지 않고 카메라 자리만 알린다.

    app.py가 카메라 설정이 하나도 없을 때 쓴다. sequence_for가 없으므로 스트림은 404다(api/cameras.py).
    """

    def snapshots(self) -> list[CameraSnapshot]:
        """여섯 카메라 자리를 모두 '장치 경로 미설정'으로 알린다."""
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
    """팔 id와 역할로 카메라 id를 만든다. 예: arm-01, GRIPPER -> arm01_gripper"""
    return f"{arm_id.replace('-', '')}_{role.lower()}"


def _arm_role(label: str, role: str) -> str:
    """화면 역할 이름(예: "왼쪽 팔 그리퍼 캠"). 위 카메라에는 "(ACT 정책)"을 붙인다."""
    return f"{label} {ROLE_LABELS[role]}" + (" (ACT 정책)" if role == "OVERHEAD" else "")


def _load_arm_devices() -> dict[str, ArmDevices]:
    """config/arms.json의 팔별 카메라 장치. 팔 설정이 없거나 틀리면 빈 dict(팔 카메라 없이 시작)."""
    try:
        return arm_devices(load_arms().arms)
    except ArmConfigError:
        return {}


# ──── 5) ROS 영상 토픽 소스 ───────────────────────────────────────────────────────────────────────

# CONTROL_STATION_*_CAMERA=ros:<영상 토픽>이면 그 카메라를 V4L2 대신 ROS 카메라 드라이버 노드에서 읽는다.
ROS_SOURCE_PREFIX = "ros:"


class _RosImageSource:
    """sensor_msgs Image 또는 CompressedImage 토픽 하나의 최신 프레임을, 캡처 시각과 함께 BGR로 준다.

    자체 rclpy Context와 executor를 데몬 스레드("ros-camera-<토픽>")에서 돌린다. 콜백은 최신 프레임 하나만 덮어
    두고(쌓지 않음) Condition으로 기다리는 캡처 스레드를 깨운다. 도메인: CONTROL_STATION_MOBILE_DOMAIN → ROS_DOMAIN_ID.
    """

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
        self._latest: tuple[object, str, float] | None = None  # (BGR 이미지, captured_at ISO, captured monotonic)
        self._ready = threading.Condition()
        self._context = Context()
        self._context.init(domain_id=int(raw_domain))
        # 노드 이름은 토픽에서 만든다(ROS 이름에 쓸 수 없는 글자는 _로, 길이는 200자까지).
        name = "control_station_camera_" + re.sub(r"[^A-Za-z0-9_]", "_", topic.strip("/"))
        self._node = Node(name[:200], context=self._context)
        # best effort(qos_profile_sensor_data)로 구독하면 reliable·best-effort 드라이버 모두에 붙고, 오래된 프레임을
        # 쌓아 두지 않는다.
        self._node.create_subscription(
            CompressedImage if self._compressed else Image, topic, self._on_image, qos_profile_sensor_data
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, name=f"ros-camera-{topic}", daemon=True)
        self._thread.start()

    def _to_bgr(self, message):  # type: ignore[no-untyped-def]
        """ROS 영상 메시지를 BGR numpy 배열로. compressed는 cv2로 디코드하고(실패하면 None), raw는 bgr8·rgb8·mono8만
        받는다. raw는 한 줄 바이트 수(step)에 붙은 패딩을 잘라 낸다."""
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
        """executor 스레드에서 불리는 구독 콜백. 최신 프레임 하나를 덮어 두고 기다리는 쪽을 깨운다."""
        received_monotonic = time.monotonic()
        received_wall = time.time()
        image = self._to_bgr(message)
        if image is None:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        # 프레임의 캡처 시각은 메시지 stamp로 하되 도착 시각보다 늦을 수는 없다(C++ 주차 노드와 같은 규칙).
        # stamp가 0이면 도착 시각을 쓴다. 신선도용 monotonic 시각도 같은 나이(age, 초)만큼 앞당긴다.
        age = max(0.0, received_wall - stamp) if stamp > 0 else 0.0
        captured_wall = received_wall - age
        captured_at = datetime.fromtimestamp(captured_wall, tz=timezone.utc).isoformat(timespec="milliseconds")
        with self._ready:
            self._latest = (image, captured_at, received_monotonic - age)
            self._ready.notify_all()

    def next_frame(self, timeout_s: float) -> tuple[object, str, float] | None:
        """새 프레임을 최대 timeout_s(초) 기다려 꺼낸다. 꺼낸 프레임은 비우므로 같은 프레임을 두 번 주지 않는다.
        없으면 None."""
        with self._ready:
            if self._latest is None:
                self._ready.wait(timeout_s)
            frame, self._latest = self._latest, None
        return frame

    def close(self) -> None:
        """기다리는 캡처 스레드를 깨우고, executor·노드·Context를 정리한 뒤 스레드를 최대 2초 기다린다."""
        with self._ready:
            self._ready.notify_all()
        self._executor.shutdown()
        self._node.destroy_node()
        self._context.try_shutdown()
        self._thread.join(timeout=2)


# ──── 6) 카메라 관리자 ───────────────────────────────────────────────────────────────────────────


class OpenCVArmCameraStreamManager:
    """설정된 V4L2 장치를 하나씩 한 번만 열어 소유하고, 최신 JPEG 프레임과 검출을 내준다.

    피드마다 캡처 스레드 하나. 정책 실행이 장치를 써야 하면 그 피드를 닫았다가(hold_for_policy, lend_devices) 끝나면
    다시 연다. `ros:` 장치는 V4L2 대신 ROS 토픽에서 받으므로 정책과 다투지 않는다.
    """

    def __init__(
        self,
        joint_enabled: bool = True,
        head_enabled: bool = True,
        policy_active: Callable[[], bool] | None = None,
        *,
        policy_devices: Callable[[], list[str]] | None = None,
        arm_cameras: Mapping[str, ArmDevices] | None = None,
    ) -> None:
        # ── 6a) 피드 구성 ──
        # 학습 정책 클라이언트가 Front Camera를 직접 열고 V4L2는 한 프로세스만 스트리밍하게 하므로, 정책 실행이
        # 요청됐거나 살아 있는 동안 대시보드가 그 카메라를 내놓는다.
        self._policy_active = policy_active
        self._front_policy_hold = False
        self._policy_active_cache = (0.0, False)  # (확인한 monotonic 시각, 결과): 0.2초 캐시
        # 살아 있는 정책 실행(어느 팔이든)의 장치와, 곧 시작할 실행을 위해 잡아 둔 장치: 그동안 닫는다.
        self._policy_devices = policy_devices
        self._policy_devices_cache: tuple[float, frozenset[str]] = (0.0, frozenset())
        self._held_devices: frozenset[str] = frozenset()
        # 예전 단일 팔 설정의 관절(그리퍼) 카메라. CONTROL_STATION_ARM_CAMERA는 그보다 더 오래된 이름이다.
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
        ).strip() if head_enabled else ""
        # 예전 팔별 미리보기 피드(LEFT/RIGHT_CEILING, LEFT/RIGHT_JOINT). 지금은 팔 설정의 카메라가 우선한다.
        left_ceiling_device = os.environ.get("CONTROL_STATION_LEFT_CEILING_CAMERA", "").strip()
        right_ceiling_device = os.environ.get("CONTROL_STATION_RIGHT_CEILING_CAMERA", "").strip()
        left_joint_device = os.environ.get("CONTROL_STATION_LEFT_JOINT_CAMERA", "").strip()
        right_joint_device = os.environ.get("CONTROL_STATION_RIGHT_JOINT_CAMERA", "").strip()
        front_device = os.environ.get(
            "CONTROL_STATION_FRONT_CAMERA",
            os.environ.get("CONTROL_STATION_POLICY_FRONT_CAMERA", ""),
        ).strip()
        # 카메라별 YOLO: 가중치나 YOLO-World 프롬프트가 있을 때만 검출기를 붙인다.
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
            joint_feed.error_message = (
                "관절 카메라를 보려면 CONTROL_STATION_JOINT_CAMERA를 설정하세요"
            )
        elif joint_weights or joint_prompts:
            joint_feed.detector = _YoloDetector(
                joint_weights, "CONTROL_STATION_JOINT_YOLO"
            )
        self._feeds = {
            "left_ceiling_camera": _CameraFeed(
                "left_ceiling_camera", "LEFT_CEILING", left_ceiling_device or None, robot_id=None
            ),
            "right_ceiling_camera": _CameraFeed(
                "right_ceiling_camera", "RIGHT_CEILING", right_ceiling_device or None, robot_id=None
            ),
            "left_joint_camera": _CameraFeed(
                "left_joint_camera", "LEFT_JOINT", left_joint_device or None, robot_id="arm-01"
            ),
            "right_joint_camera": _CameraFeed(
                "right_joint_camera", "RIGHT_JOINT", right_joint_device or None, robot_id="arm-02"
            ),
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
        # 선택: 정면에서 선반과 팔을 보는 선반 카메라. id가 shelf_camera 내부 보정(intrinsic calibration)과 맞는다.
        # YOLO 검출은 다른 피드처럼 보이지만, Pick & Place 작업 관측 단계는 없다.
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
                robot_id=None,  # 팔에 달린 카메라가 아니라 선반을 고정해서 보는 카메라
                role="선반 재고",
                opened_by="ros_node" if shelf_device.startswith(ROS_SOURCE_PREFIX) else "control_station",
                setting="CONTROL_STATION_SHELF_CAMERA",
            )
            self._feeds["shelf_camera"] = shelf_feed
        front_feed = self._feeds["front_camera"]
        if not front_feed.device:
            front_feed.error_message = "Front Camera 장치 경로 미설정 (CONTROL_STATION_FRONT_CAMERA)"
        self._add_arm_feeds(_load_arm_devices() if arm_cameras is None else arm_cameras, joint_enabled)
        # 핑키 탑재 전용 YOLO(선택): 적재 카메라(CONTROL_STATION_LOAD_CAMERA, 기본 head_camera) 피드에만 붙인다.
        # load_snapshots()가 이 검출을 준다.
        load_weights = os.environ.get("CONTROL_STATION_LOAD_YOLO_WEIGHTS", "").strip()
        load_camera_id = os.environ.get("CONTROL_STATION_LOAD_CAMERA", "").strip() or "head_camera"
        if load_weights and load_camera_id in self._feeds:
            self._feeds[load_camera_id].load_detector = _YoloDetector(load_weights, "CONTROL_STATION_LOAD_YOLO")
        # 같은 실제 장치(realpath)를 두 피드가 열려고 하면 둘 다 열지 않고 오류로 알린다. 어느 쪽이 맞는 설정인지
        # 관제는 알 수 없기 때문이다.
        by_device: dict[str, list[_CameraFeed]] = {}
        for feed in self._feeds.values():
            # 정책 몫 피드는 꽂혔는지만 보고 카메라를 열지 않는다. 그래서 미리보기 피드와 같은 경로를 써도 V4L2 장치를
            # 두고 다투지 않는다.
            if feed.device and not feed.watch_only:
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
        # 피드마다 CONTROL_STATION_<POSITION>_CAMERA_RESOLUTION=WxH(예: 1280x720)로 캡처 크기를 바꿀 수 있다.
        # 형식이 틀리면 그 피드를 열지 않는다. 아직 아무 장치도 열지 않았으므로 released로 시작한다.
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
        self._closed = threading.Event()  # 서버 종료: 모든 캡처 루프가 이것을 보고 끝난다
        self._ros_sources: list[_RosImageSource] = []
        if any(feed.device and feed.device.startswith(ROS_SOURCE_PREFIX) for feed in self._feeds.values()):
            try:
                # 캡처 스레드보다 먼저 여기서 import한다. rclpy의 첫 import는 두 스레드에서 동시에 하면 안전하지
                # 않다(다른 스레드가 같은 때 주차 노드용으로 import하고 있을 수 있다).
                import rclpy.node  # noqa: F401
            except ImportError:
                pass  # ros: 피드마다 ROS 환경이 없다고 스스로 알린다
        # 설정된(watch_only가 아닌) 피드마다 캡처 스레드 하나. 데몬이라 서버 종료를 막지 않는다.
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
        """팔 카메라마다 피드 하나. 같은 장치를 쓰는 예전 단일 팔 피드(joint/front)는 자리를 내준다.

        그리퍼 캠은 관절 카메라를 켰으면(joint_enabled), 위 캠은 미리보기(CONTROL_STATION_OVERHEAD_CAMERA_PREVIEW)를
        켰을 때만 관제가 연다. 아니면 정책 몫(watch_only)으로 꽂혔는지만 본다.
        """
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
                if device and streamed:
                    # 예전 팔별 미리보기 피드(left_joint_camera 등)가 같은 장치를 가리키는지 본다.
                    side = "left" if devices.arm_id == "arm-01" else "right"
                    preview_id = f"{side}_{'joint' if role == 'GRIPPER' else 'ceiling'}_camera"
                    preview_feed = self._feeds.get(preview_id)
                    if (preview_feed and preview_feed.device
                            and os.path.realpath(preview_feed.device) == os.path.realpath(device)):
                        # V4L2 캡처 주인은 하나로 두면서, 정책 카메라의 연결(꽂힘) 표시는 남긴다.
                        preview_feed.watch_only = True
        # 팔 피드가 쓰는 장치를 예전 joint/front 피드도 가리키면 예전 피드를 지운다.
        taken = {os.path.realpath(feed.device) for feed in added if feed.device}
        for legacy_id in ("joint_camera", "front_camera"):
            legacy = self._feeds.get(legacy_id)
            if legacy is not None and legacy.device and os.path.realpath(legacy.device) in taken:
                del self._feeds[legacy_id]
        legacy_joint = self._feeds.get("joint_camera")
        if taken and legacy_joint is not None and not legacy_joint.device:
            del self._feeds["joint_camera"]  # 팔마다 그리퍼 카메라가 있는데 장치도 없는 예전 자리는 지운다
        self._feeds.update((feed.camera_id, feed) for feed in added)

    # ── 6b) 정책 실행에 장치 빌려주기·되찾기 ──

    def lend_devices(self, devices: list[str], reason: str, timeout_s: float = 3.0) -> list[str]:
        """`devices` 중 하나라도 여는 V4L2 피드를 닫아 다른 프로그램(정책을 돌리는 팔 워커)이 열 수 있게 한다.
        제때 닫힌 피드의 id를 돌려준다. `timeout_s`(초) 뒤에도 열려 있는 피드는 빌려준 상태로 두고 목록에서 뺀다.
        api/arm.py의 sync_policy_cameras가 정책 집기 명령이 도는 동안 부른다. 피드는 캡처 스레드가 닫는다."""
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
        """빌려준 피드를 모두 다시 연다(lent를 지우면 캡처 루프가 다시 연다). 다시 열 피드 id를 돌려준다."""
        lent = [feed for feed in self._feeds.values() if feed.lent.is_set()]
        for feed in lent:
            feed.lent.clear()
        return [feed.camera_id for feed in lent]

    @staticmethod
    def _release_capture(capture) -> str | None:  # type: ignore[no-untyped-def]
        """capture.release()를 하고, 실패하면 예외 대신 오류 문장을 돌려준다."""
        try:
            capture.release()
        except Exception as exc:
            return f"카메라 자원 해제 실패: {exc}"
        return None

    def _held_for_policy(self, feed: _CameraFeed) -> bool:
        """이 카메라를 관제 대신 학습 정책 클라이언트가 가져야 하는지 돌려준다.

        캡처 루프가 프레임마다 부르므로 콜백 결과는 0.2초 캐시한다. 여러 캡처 스레드가 잠금 없이 캐시를 쓰지만,
        엇갈려도 한 번 더 묻는 것뿐이다.
        """
        if not feed.device or feed.device.startswith(ROS_SOURCE_PREFIX):
            return False
        path = os.path.realpath(feed.device)
        if path in self._held_devices:
            return True
        now = time.monotonic()
        if feed.camera_id == "front_camera":
            # 예전 설정: 어떤 정책 실행이든 살아 있으면 Front Camera를 내놓는다.
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
        """정책 실행이 `devices`(와 예전 Front Camera)를 열기 전에 그 피드들을 닫는다.

        ``timeout``(초) 안에 캡처가 닫힌 것을 확인하지 못하면 False. hold=False면 잡아 둔 것을 풀고 바로 True를
        돌려준다(캡처 루프가 다음 확인에서 다시 연다)."""
        # 캐시를 비워 캡처 루프가 다음 확인에서 바로 새 상태를 보게 한다.
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
        """정책 실행이 장치를 열기 전에 대시보드의 Front Camera 캡처를 닫는다(예전 단일 팔 경로)."""
        return self.hold_for_policy([], hold, timeout)

    # ── 6c) V4L2 캡처 루프 ──

    def _capture_loop(self, feed: _CameraFeed) -> None:
        """피드 하나의 캡처 스레드 본문. 서버가 닫힐 때(_closed)까지 돈다.

        장치를 MJPG로 열어 프레임을 읽고 _publish_frame에 넘긴다. 열기·읽기가 실패하면 장치를 닫고 2초 쉰 뒤 다시
        연다. 빌려줌(lent)이나 정책 잡기(hold) 동안은 장치를 닫고 기다린다. 장치를 닫을 때마다 released를 세워
        hold_for_policy가 기다릴 수 있게 한다. `ros:` 장치는 _ros_capture_loop로 넘긴다.
        """
        try:
            import cv2
        except ImportError:
            with feed.lock:
                feed.connection_state = CameraConnectionState.DISCONNECTED
                feed.stream_state = CameraStreamState.ERROR
                feed.error_message = "OpenCV 미설치: 관제 서버 가상환경에서 requirements.txt를 설치하세요"
            return

        # 검출기는 이 스레드에서 올린다(무거운 모델 로드가 서버 시작을 막지 않게).
        for detector in (feed.detector, feed.load_detector):
            if detector is not None:
                detector.load()

        if feed.device and feed.device.startswith(ROS_SOURCE_PREFIX):
            self._ros_capture_loop(feed, feed.device.removeprefix(ROS_SOURCE_PREFIX), cv2)
            return

        while not self._closed.is_set():
            device = feed.device
            if not device:
                return
            if feed.lent.is_set():
                # 여기서는 장치가 닫혀 있다. 돌려받을 때까지 기다린다.
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
            # hold 확인 전에 released를 먼저 지운다. 그래야 동시에 들어온 hold 요청이 이 캡처가 닫힐 때까지 기다린다.
            feed.released.clear()
            if self._held_for_policy(feed):
                feed.released.set()
                with feed.lock:
                    feed.detections = []  # 잡혀 있는 동안에는 마지막 검출을 뒷받침하는 실시간 프레임이 없다
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

            # OpenCV의 V4L2 백엔드는 장치 이름 문자열이 아니라 카메라 번호를 기대한다. /dev/videoN이면 N을 쓴다.
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
                # 실제 카메라들이 USB 2.0 허브 하나를 같이 쓴다. 압축된 MJPEG를 명시적으로 요청해, 여러 스트림을 동시에
                # 열어도 압축하지 않은 YUYV에 필요한 버스 대역폭을 잡아 두지 않게 한다. 30 fps를 요청한다.
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
                    # 읽기가 10번 연속 실패하면(USB가 빠짐 등) 장치를 닫고 다시 연다.
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
                continue  # 다음 차례가 잡힘을 알리고 오류 상태 없이 기다린다
            if feed.lent.is_set():
                continue  # 일부러 닫았다. 루프 맨 위가 그것을 알리고 기다린다
            if not self._closed.is_set():
                with feed.lock:
                    feed.connection_state = CameraConnectionState.DISCONNECTED
                    feed.stream_state = CameraStreamState.ERROR
                    feed.error_message = (
                        capture_failure or f"카메라 프레임 수신 중단: {feed.device}"
                    )
                self._closed.wait(2)

    # ── 6d) 프레임 처리: ArUco 마커, YOLO, JPEG ──

    def _find_markers(self, feed: _CameraFeed, image, cv2) -> list[dict[str, object]]:  # type: ignore[no-untyped-def]
        """CONTROL_STATION_MARKER_CAMERAS(기본 head_camera)에 든 카메라에서 ArUco 마커(DICT_4X4_50)를 찾는다.

        표시 전용: 적재 카메라 카드의 "핑키 도착" 줄. 주차 판정은 여전히 C++ 노드가 맡는다. 마커마다 id, 중심과
        네 꼭짓점(px)을 돌려준다. 검출기는 처음 쓸 때 한 번 만들어 둔다.
        """
        cameras = {name.strip() for name in os.environ.get("CONTROL_STATION_MARKER_CAMERAS", "head_camera").split(",")}
        if feed.camera_id not in cameras or not hasattr(cv2, "aruco"):
            return []
        detector = getattr(self, "_aruco_detector", None)
        if detector is None:
            detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                               cv2.aruco.DetectorParameters())
            self._aruco_detector = detector
        try:
            corners, ids, _ = detector.detectMarkers(image)
        except Exception:  # 마커 찾기가 실패해도 스트림은 멈추지 않는다
            return []
        if ids is None:
            return []
        found = []
        for marker_corners, marker_id in zip(corners, ids.ravel().tolist()):
            points = marker_corners.reshape(4, 2)
            found.append({"id": int(marker_id), "center_px": [round(float(v), 1) for v in points.mean(axis=0)],
                          "corners_px": [[round(float(x), 1), round(float(y), 1)] for x, y in points]})
        return found

    def _publish_frame(self, feed: _CameraFeed, image, captured_at: str, captured_monotonic: float, cv2) -> None:  # type: ignore[no-untyped-def]
        """BGR 프레임 하나에 YOLO를 돌리고 피드의 최신 JPEG로 만든다.

        무거운 처리(마커, YOLO, JPEG 인코딩)는 잠금 밖에서 하고, 결과를 피드에 쓰는 것만 feed.lock 아래에서 한다.
        그래서 HTTP 스레드가 추론 시간만큼 막히지 않는다.
        """
        frame_sha256 = hashlib.sha256(image.tobytes(order="C")).hexdigest()
        detections: list[dict[str, object]] = []
        markers = self._find_markers(feed, image, cv2)  # YOLO가 그리기 전의 원본 프레임에서
        load_detections: list[dict[str, object]] = []
        load_detector = feed.load_detector
        if load_detector is not None and load_detector.state == "ready":
            try:
                _, load_detections = load_detector.annotate(image, render=False)
            except Exception as exc:
                load_detector.state, load_detector.error_message = "error", f"탑재 YOLO 추론 실패: {exc}"
        detector = feed.detector
        if detector is not None and detector.state == "ready":
            try:
                image, detections = detector.annotate(image)
            except Exception as exc:  # 추론이 실패해도 원본 프레임은 계속 내보낸다
                detector.state, detector.error_message = "error", f"YOLO 추론 실패: {exc}"
        # 탑재 검출에, 일반 검출 가운데 겹치지 않는 것을 덧붙인다.
        if load_detector is not None and load_detector.state == "ready":
            load_detections = _merge_load_detections(load_detections, detections)
        encoded, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not encoded:
            return  # 인코딩에 실패한 프레임은 버린다(프레임 번호도 올리지 않음)
        frame_height, frame_width = image.shape[:2]
        with feed.lock:
            feed.sequence += 1
            frame_sequence = feed.sequence
            feed.frame_width = int(frame_width)
            feed.frame_height = int(frame_height)
            feed.frame_sha256 = frame_sha256
            # 검출마다 어느 프레임에서 나왔는지(카메라, 시각, 번호, 해시, 픽셀 변환)를 붙인다. 팔 명령의 카메라 근거가 된다.
            for detection in (*detections, *load_detections):
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
            feed.load_detections = load_detections
            feed.markers = markers
            feed.last_frame_at = captured_at
            feed.last_frame_monotonic = captured_monotonic
            feed.frame_times.append(captured_monotonic)
            # 최근 2초만 남겨 fps를 계산한다.
            feed.frame_times = [
                stamp for stamp in feed.frame_times if captured_monotonic - stamp <= 2
            ]
            feed.connection_state = CameraConnectionState.CONNECTED
            feed.stream_state = CameraStreamState.STREAMING
            feed.error_message = None

    # ── 6e) ROS 토픽 캡처 루프 ──

    def _ros_capture_loop(self, feed: _CameraFeed, topic: str, cv2) -> None:  # type: ignore[no-untyped-def]
        """카메라 드라이버 노드가 발행하는 ROS 영상 토픽에서 피드의 프레임을 받는다.

        드라이버가 장치를 소유하므로 관제와 C++ 주차 노드가 카메라 하나를 같이 쓸 수 있다
        (docs/plans/pinky-parking-ros2-node.md). 기하는 같은 640x480 MJPEG 캡처에 자르기·뒤집기가 없으므로 검출은
        PIXEL_TRANSFORM_ID를 그대로 쓴다. /compressed로 끝나는 토픽은 여기서 디코드한다. 프로세스 사이에서는 이
        토픽을 써야 한다. 2026-11-02에 640x480 raw 영상은 프로세스 사이에서 대부분 버려졌다.
        2초 동안 프레임이 없으면 NO_FRAMES로 알리고 계속 기다린다(드라이버가 살아나면 이어서 받는다).
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
        """정책 몫 카메라: 장치가 꽂혀 있는지만 본다. _snapshots가 feed.lock 아래에서 부른다."""
        if not feed.device:
            return  # 시작할 때 거부됨(예: 다른 카메라와 같은 장치). 오류 문장이 이유를 말한다
        present = os.path.exists(os.path.realpath(feed.device))
        feed.connection_state = CameraConnectionState.CONNECTED if present else CameraConnectionState.DISCONNECTED
        feed.stream_state = CameraStreamState.STOPPED if present else CameraStreamState.ERROR
        how_to_view = ("CONTROL_STATION_OVERHEAD_CAMERA_PREVIEW=1" if feed.position == "OVERHEAD"
                       else "팔 원격 조작 CONTROL_STATION_ARM_ENABLED=1")
        feed.error_message = (
            f"정책 실행 때 정책이 직접 엽니다 (관제 화면에서 보려면 {how_to_view})"
            if present else f"카메라 장치 경로가 없습니다: {feed.device}"
        )

    # ── 6f) 스냅샷과 프레임 내주기, 종료 ──

    def snapshots(self) -> list[CameraSnapshot]:
        """화면·개수 세기·칸 판정용 카메라 스냅샷(피드의 일반 YOLO 검출)."""
        return self._snapshots(for_load=False)

    def load_snapshots(self) -> list[CameraSnapshot]:
        """적재 카메라에 탑재 전용 YOLO가 있으면 그 검출(일반 검출과 합친 것)을 싣는 스냅샷. 핑키 탑재 세기
        (cameras/counts.py PinkyLoadCounter)가 쓴다. 탑재 YOLO가 없는 피드는 snapshots()와 같다."""
        return self._snapshots(for_load=True)

    def _snapshots(self, *, for_load: bool) -> list[CameraSnapshot]:
        """피드마다 CameraSnapshot 하나를 feed.lock 아래에서 만든다."""
        result = []
        for feed in self._feeds.values():
            if feed.camera_id == "front_camera" and not feed.device:
                continue  # 이 PC에 설정되지 않음: 보이지 않는다(쓰려면 CONTROL_STATION_FRONT_CAMERA를 설정)
            with feed.lock:
                if feed.watch_only:
                    self._watch(feed)
                use_load_detector = for_load and feed.load_detector is not None
                detector = feed.load_detector if use_load_detector else feed.detector
                detections = feed.load_detections if use_load_detector else feed.detections
                configured = feed.device is not None
                # 정책 실행 때문에 닫혀 있어도 꽂혀 있는 것으로 친다.
                linked = configured and (
                    feed.connection_state == CameraConnectionState.CONNECTED
                    or (feed.stream_state == CameraStreamState.STOPPED and os.path.exists(os.path.realpath(feed.device)))
                )
                # 마지막 프레임이 2초 안이면 FRESH. 개수 세기와 칸 선택은 FRESH 프레임만 쓴다.
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
                            or (detector.error_message if detector else None)
                            or (None if configured else "카메라 장치 경로 미설정")
                        ),
                        detector_state=detector.state if detector else None,
                        detector_model_type=(
                            detector.detector_model_type if detector else None
                        ),
                        detector_prompts=(
                            list(detector.prompts) if detector else None
                        ),
                        detector_classes=(
                            list(detector.class_names)
                            if detector and detector.state == "ready"
                            else None
                        ),
                        detector_confidence_threshold=(
                            detector.confidence_threshold if detector else None
                        ),
                        detector_weights_sha256=(
                            detector.weights_sha256 if detector else None
                        ),
                        detector_inference_image_size=(
                            detector.inference_image_size if detector else None
                        ),
                        detector_device=(detector.device if detector else None),
                        detector_ultralytics_version=(
                            detector.ultralytics_version if detector else None
                        ),
                        detections=list(detections),
                        markers=list(feed.markers),
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
        """카메라의 최신 JPEG(없으면 None). MJPEG 스트림이 부른다."""
        feed = self._feeds.get(camera_id)
        if feed is None:
            return None
        with feed.lock:
            return feed.frame

    def sequence_for(self, camera_id: str) -> int:
        """카메라의 최신 프레임 번호(모르는 카메라는 -1). 스트림이 새 프레임이 왔는지 볼 때 쓴다."""
        feed = self._feeds.get(camera_id)
        if feed is None:
            return -1
        with feed.lock:
            return feed.sequence

    def close(self) -> None:
        """서버 종료: 캡처 루프와 ROS 소스를 멈춘다. 2초 안에 끝나지 않은 (데몬) 스레드는 두고 상태도 건드리지 않는다."""
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
