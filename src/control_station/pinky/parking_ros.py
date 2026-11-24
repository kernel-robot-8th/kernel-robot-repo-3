"""C++ ROS 2 노드가 내린 핑키 주차(적재 자리 도착) 판정을 관제로 가져온다(docs/plans/pinky-parking-ros2-node.md).

판정은 노드(cpp/parking_vision_ros)가 맡고, 관제는 자기 주차 판정을 갖지 않는다. 이 모듈은 /pinky_parking/state를
구독하고(실시간으로 따라가는 다른 자리의 노드도 구독한다. 핑키 등록마다 하나), /pinky_parking/check를 호출해, 둘을
주차 계약의 JSON(interface-contracts.md: GET /api/parking-spots/{spot_id}/live 와 /check)으로 바꾼다. 웹 패널과
VERIFY_ARRIVAL이 이 JSON을 쓴다. 시간 안에 답이 없거나 노드에 닿지 못한 확인은 판정하지 않은 것으로 하고, 다시
시도하지 않는다. 여기서는 로봇 명령을 하나도 보내지 않는다.

두 경로:
- 실시간 상태(/live, `snapshot`): 노드가 10 Hz로 발행하는 ParkingState. 표시 전용(display_only,
  usable_for_robot_control False)이고, 적재 자리 진입 안내(pinky/docking.py DockGuideRelay)의 조향에만 쓴다.
- 확인(/check, `check`): CheckParking 서비스. 노드가 새 프레임을 모아 한 번 판정한다. 도착(ARRIVED 등)은 이것으로만 정한다.

스레드: rclpy executor가 데몬 스레드("parking-ros")에서 돌고, 상태 콜백은 그 스레드에서 `_states`를 잠금 아래
갱신한다. HTTP 요청 스레드와 주문 흐름은 `snapshot`/`check`로 읽는다. 같은 프로세스의 다른 rclpy 사용과 섞이지 않게
자체 Context를 쓴다. 위치: app.py가 CONTROL_STATION_PARKING_SPOT이 있을 때 하나 만든다.
"""

from __future__ import annotations

import re
import threading
import time
from datetime import datetime, timezone
from functools import partial
from typing import Any, Callable, Iterable

OBSERVATION_SCHEMA_VERSION = 1
PRIMARY_NODE = "pinky_parking"
STATE_TOPIC = f"/{PRIMARY_NODE}/state"
CHECK_SERVICE = f"/{PRIMARY_NODE}/check"
# 노드는 10 Hz로 발행한다. 이보다 오래된(초) 상태는 노드나 그 타이머가 멈췄다는 뜻이다.
STALE_STATE_S = 2.0
# 확인 한 번은 최대 3초 동안 프레임을 모은다. 나머지(초)는 자리 파일을 다시 읽는 시간과 서비스 왕복 몫이다.
CHECK_TIMEOUT_S = 6.0
NODE_UNAVAILABLE = "NODE_UNAVAILABLE"
# CheckParking의 error_code -> 관제 프로세스 안 API의 HTTP 상태(interface-contracts.md). 여기 없는 코드는 502.
HTTP_STATUS = {
    "PARKING_SPOT_ID_INVALID": 400,
    "PARKING_SPOT_NOT_FOUND": 404,
    "PARKING_SPOT_INVALID": 409,
    "PARKING_CAMERA_UNAVAILABLE": 503,
    "PARKING_OBSERVATION_ERROR": 503,
    "PARKING_CHECK_BUSY": 429,
}


class ParkingObservationError(Exception):
    """판정하지 못한 확인. API가 돌려줄 HTTP 상태와 error_code를 함께 담는다."""

    def __init__(self, http_status: int, error_code: str, message: str) -> None:
        super().__init__(message)
        self.http_status = http_status
        self.error_code = error_code
        self.message = message


def iso_time(stamp: Any) -> str | None:  # builtin_interfaces/Time 메시지
    """ROS 시각을 UTC ISO 8601(밀리초) 문자열로 바꾼다. 0(아직 시각 없음)이면 None."""
    nanoseconds = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if nanoseconds == 0:
        return None
    moment = datetime.fromtimestamp(nanoseconds / 1e9, tz=timezone.utc)
    return moment.isoformat(timespec="milliseconds")


def _points(values: Any) -> list[list[float]]:
    """노드가 주는 평탄한 배열 [x0, y0, x1, y1, ...]을 [[x, y], ...] 픽셀 점 목록(소수 첫째 자리)으로 바꾼다."""
    flat = [float(v) for v in values]
    return [[round(flat[i], 1), round(flat[i + 1], 1)] for i in range(0, len(flat) - 1, 2)]


def judgment_fields(message: Any) -> dict[str, object]:
    """/live와 /check가 함께 쓰는 어긋남·허용 오차·벗어남 필드. ParkingState와 CheckParking 어느 쪽에서든 만든다.

    offset: 등록 자리 마커 축 기준 dx_mm·dy_mm·distance_mm(mm)와 dyaw_deg(도). 자리에 marker_forward_axis가 등록돼
    있으면(has_pinky_frame) 핑키 자신의 축 기준 forward_mm·left_mm도 넣는다. 이 값이 redock과 진입 안내에 쓰인다.
    position_ratio·yaw_ratio·deviation_level은 노드가 허용 오차에 견준 어긋남 정도이고, view_shift_px는 카메라 화면이
    등록 때보다 밀린 정도(px)다(view_check가 ON일 때).
    """
    offset: dict[str, object] | None = None
    axis = message.marker_forward_axis or None
    if message.has_offset:
        offset = {
            "frame": "REGISTERED_SPOT_MARKER_AXES",
            "dx_mm": round(message.dx_mm, 1),
            "dy_mm": round(message.dy_mm, 1),
            "distance_mm": round(message.distance_mm, 1),
            "dyaw_deg": round(message.dyaw_deg, 2),
            "marker_forward_axis": axis,
        }
        if message.has_pinky_frame:
            offset.update(forward_mm=round(message.forward_mm, 1), left_mm=round(message.left_mm, 1))
    return {
        "offset": offset,
        "tolerance": {"position_mm": message.tolerance_position_mm, "yaw_deg": message.tolerance_yaw_deg},
        "position_ratio": round(message.position_ratio, 2) if message.has_ratios else None,
        "yaw_ratio": round(message.yaw_ratio, 2) if message.has_ratios else None,
        "deviation_level": message.deviation_level if message.has_ratios else None,
        "view_check": "ON" if message.view_check else "OFF",
        "view_shift_px": round(message.view_shift_px, 1) if message.has_view_shift else None,
    }


def image_overlay(message: Any) -> dict[str, object]:
    """스트림 위에 그릴 원본 영상 픽셀 좌표(표시 전용). ParkingState와 CheckParking 어느 쪽에서든 만든다."""
    return {
        "marker_forward_axis": message.marker_forward_axis or None,
        "view_shift_xy_px": (
            [round(message.view_shift_x_px, 1), round(message.view_shift_y_px, 1)] if message.has_view_shift_xy else None
        ),
        "image_size_px": {"width": int(message.image_width), "height": int(message.image_height)},
        "registered_corners_px": _points(message.registered_corners_px),
        "marker_corners_px": _points(message.marker_corners_px) if message.has_marker_corners else None,
        "anchor_centers_px": _points(message.anchor_centers_px),
    }


def parking_node_name(spot_id: str, primary_spot_id: str) -> str:
    """spot_id를 실시간으로 따라가는 주차 노드 이름. 노드 하나는 자리 하나만 따라가고, 핑키마다 적재 자리 등록이
    따로 있으므로(pinky/markers.py) ceiling_parking.launch.py가 등록된 자리마다 노드를 하나씩 같은 규칙으로 띄운다:
    주 자리(launch의 spot_id 인자, 여기서는 CONTROL_STATION_PARKING_SPOT)는 'pinky_parking', 나머지는
    'pinky_parking_<자리>'이고, 글자·숫자가 아닌 문자는 모두 '_'로 바꾼다. 두 쪽 규칙이 어긋나면 구독이 비게 된다."""
    if spot_id == primary_spot_id:
        return PRIMARY_NODE
    return PRIMARY_NODE + "_" + re.sub(r"[^A-Za-z0-9]", "_", spot_id)


def state_topic(spot_id: str, primary_spot_id: str) -> str:
    """spot_id를 따라가는 노드의 상태 토픽(/pinky_parking/state 또는 /pinky_parking_<자리>/state)."""
    return f"/{parking_node_name(spot_id, primary_spot_id)}/state"


def live_payload(message: Any, feed_camera_id: str | None, topic: str = STATE_TOPIC) -> dict[str, object]:
    """ParkingState 메시지로 만든 /live 응답.

    feed_camera_id는 같은 카메라를 보여 주는 관제 피드다(웹 패널이 그것을 스트리밍한다). 노드 자신의 영상 토픽은
    node_image_topic으로 남긴다. 자리가 아직 정해지지 않은 상태(has_spot False)면 어긋남·그리기 필드를 넣지 않는다.
    """
    payload: dict[str, object] = {
        "state": message.state,
        "reason": message.reason,
        "spot_id": message.spot_id,
        "feed_camera_id": feed_camera_id,
        "node_image_topic": message.feed_camera_id,
        "frame_sequence": int(message.frame_sequence) if message.has_frame else None,
        "captured_at": iso_time(message.header.stamp) if message.has_frame else None,
        "updated_at": iso_time(message.updated_at),
        "display_only": True,
        "usable_for_robot_control": False,
        "judgment_source": "ros:" + topic,
    }
    if message.has_spot:
        payload.update(judgment_fields(message))
        payload["image_overlay"] = image_overlay(message)
    payload["since"] = iso_time(message.since)
    return payload


def observation_payload(response: Any, feed_camera_id: str | None) -> dict[str, object]:
    """성공한 CheckParking 응답으로 만든 /check 관측. feed_camera_id는 live_payload()와 같다.

    어느 프레임 구간으로 판정했는지, 자리 등록·내부 보정·OpenCV 버전의 해시까지 실어, 나중에 판정 근거를 따질 수
    있게 한다. 이 관측도 usable_for_robot_control은 False다(로봇을 직접 움직이는 조건이 아니라 주문 흐름의 확인 근거).
    """
    return {
        "observation_schema_version": OBSERVATION_SCHEMA_VERSION,
        "spot_id": response.spot_id,
        "marker_id": int(response.marker_id),
        "status": response.status,
        "reason": response.reason,
        **judgment_fields(response),
        "frames_used": int(response.frames_used),
        "frames_examined": int(response.frames_examined),
        "feed_camera_id": feed_camera_id,
        "node_image_topic": response.feed_camera_id,
        "calibration_camera_id": response.calibration_camera_id,
        "frame_sequence_range": [int(response.first_frame_sequence), int(response.last_frame_sequence)],
        "captured_from": iso_time(response.captured_from),
        "captured_until": iso_time(response.captured_until),
        "requested_at": iso_time(response.requested_at),
        "spot_registration_sha256": response.spot_registration_sha256,
        "intrinsic_calibration_sha256": response.intrinsic_calibration_sha256,
        "opencv_version": response.opencv_version,
        "registration_opencv_version": response.registration_opencv_version or None,
        "image_overlay": image_overlay(response),
        "usable_for_robot_control": False,
        "judgment_source": "ros:" + CHECK_SERVICE,
    }


class RosParkingSource:
    """주차 노드들의 실시간 상태와 확정 판정을 계약의 JSON 모양으로 준다. spot_id는 주 자리이고, extra_spot_ids는
    실시간으로 따라가는 다른 자리들이다(자리마다 자기 노드, parking_node_name). 확인(check)은 주 노드로 가고, 주 노드는
    등록된 어느 자리든 판정한다."""

    def __init__(
        self,
        domain_id: int,
        spot_id: str,
        feed_camera_id: str | None = None,
        clock: Callable[[], float] = time.monotonic,
        extra_spot_ids: Iterable[str] = (),
    ) -> None:
        try:
            from parking_vision_interfaces.msg import ParkingState
            from parking_vision_interfaces.srv import CheckParking
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        except ImportError as exc:
            raise RuntimeError(
                "CONTROL_STATION_PARKING_SPOT에는 ROS 2 Jazzy와 cpp/install/setup.bash를 source해야 합니다"
                " (parking_vision_interfaces)."
            ) from exc
        self.spot_id = spot_id
        # 자리 -> 상태 토픽. dict.fromkeys로 순서를 지키며 중복을 없앤다(주 자리가 맨 앞).
        self.topics = {spot: state_topic(spot, spot_id) for spot in dict.fromkeys((spot_id, *extra_spot_ids))}
        self.feed_camera_id = feed_camera_id
        self._clock = clock
        self._request_type = CheckParking.Request
        self._lock = threading.Lock()
        self._states: dict[str, tuple[float, Any]] = {}  # 자리 -> (받은 clock 시각, 마지막 ParkingState)
        self._context = Context()
        self._context.init(domain_id=domain_id)
        self._node = Node("control_station_parking", context=self._context)
        # 노드의 발행자와 QoS를 맞춘다(RELIABLE, TRANSIENT_LOCAL, depth 1). 그래서 관제가 나중에 켜져도 마지막 상태를 받는다.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for spot, topic in self.topics.items():
            self._node.create_subscription(ParkingState, topic, partial(self._on_state, spot), qos)
        self._client = self._node.create_client(CheckParking, CHECK_SERVICE)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, name="parking-ros", daemon=True)
        self._thread.start()

    @property
    def live_spots(self) -> tuple[str, ...]:
        """실시간으로 따라가는 자리들. 주 자리가 맨 앞."""
        return tuple(self.topics)

    def _on_state(self, spot_id: str, message: Any) -> None:
        """executor 스레드에서 불린다. 자리별로 받은 시각과 메시지를 잠금 아래 보관한다."""
        with self._lock:
            self._states[spot_id] = (self._clock(), message)

    def _unavailable(self, reason: str, spot_id: str | None = None, topic: str = STATE_TOPIC) -> dict[str, object]:
        """노드 상태를 쓸 수 없을 때의 /live 응답(state NODE_UNAVAILABLE). 화면은 reason을 그대로 보여 준다."""
        now = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        return {
            "state": NODE_UNAVAILABLE,
            "reason": reason,
            "spot_id": spot_id or self.spot_id,
            "feed_camera_id": self.feed_camera_id,
            "frame_sequence": None,
            "captured_at": None,
            "updated_at": now,
            "display_only": True,
            "usable_for_robot_control": False,
            "judgment_source": "ros:" + topic,
            "since": now,
        }

    def snapshot(self, spot_id: str | None = None) -> dict[str, object]:
        """spot_id 자리의 실시간 상태(None이면 주 자리). 따라가지 않는 자리, 아직 못 받은 상태, 오래된 상태, 다른
        자리를 따라가는 노드의 상태는 모두 NODE_UNAVAILABLE로 돌려준다."""
        spot_id = spot_id or self.spot_id
        topic = self.topics.get(spot_id)
        if topic is None:
            return self._unavailable(f"이 주차 위치는 실시간 추적 중이 아닙니다: {spot_id!r}", spot_id)
        with self._lock:
            state = self._states.get(spot_id)
        if state is None:
            return self._unavailable(f"주차 판정 노드 상태를 받지 못했습니다: {topic}", spot_id, topic)
        received_at, message = state
        age = self._clock() - received_at
        # 멈춘 노드가 마지막으로 보낸 IN_POSITION을 지금 상태처럼 보여 주지 않는다.
        if age > STALE_STATE_S:
            return self._unavailable(f"주차 판정 노드 상태가 {age:.1f}초째 갱신되지 않습니다: {topic}", spot_id, topic)
        if message.spot_id != spot_id:
            # launch 인자와 관제 설정이 달라 노드가 다른 자리를 따라가면, 그 값을 이 자리 값으로 쓰지 않는다.
            return self._unavailable(
                f"주차 판정 노드가 다른 주차 위치를 추적합니다: {message.spot_id!r} (관제 설정 {spot_id!r})", spot_id, topic
            )
        return live_payload(message, self.feed_camera_id, topic)

    def check(self, spot_id: str, timeout_s: float = CHECK_TIMEOUT_S) -> dict[str, object]:
        """주 노드에 spot_id 자리의 새 판정을 요청해 /check 관측을 돌려준다.

        서비스가 1초 안에 보이지 않으면 PARKING_NODE_UNAVAILABLE, timeout_s(초) 안에 답이 없으면 PARKING_NODE_TIMEOUT,
        노드가 판정을 거절하면 그 error_code와 HTTP_STATUS의 상태로 ParkingObservationError를 올린다. 부른 스레드는
        답을 기다리며 막힌다(응답은 executor 스레드에서 오고, Event로 이 스레드를 깨운다).
        """
        if not self._client.wait_for_service(timeout_sec=1.0):
            raise ParkingObservationError(
                503, "PARKING_NODE_UNAVAILABLE", f"주차 판정 노드에 연결할 수 없습니다: {CHECK_SERVICE}"
            )
        request = self._request_type()
        request.spot_id = spot_id
        done = threading.Event()
        future = self._client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            # 판정하지 않은 것으로 하고 다시 시도하지 않는다. 이 요청에 늦게 온 응답은 버린다.
            self._client.remove_pending_request(future)
            raise ParkingObservationError(
                503, "PARKING_NODE_TIMEOUT", f"주차 판정 노드가 {timeout_s:g}초 안에 응답하지 않았습니다: {CHECK_SERVICE}"
            )
        response = future.result()
        if response is None:
            raise ParkingObservationError(503, "PARKING_NODE_UNAVAILABLE", f"주차 판정 응답이 없습니다: {CHECK_SERVICE}")
        if not response.ok:
            raise ParkingObservationError(HTTP_STATUS.get(response.error_code, 502), response.error_code, response.message)
        return observation_payload(response, self.feed_camera_id)

    def close(self) -> None:
        """서버 종료 때: executor를 멈추고 노드와 Context를 정리한 뒤 스레드를 최대 2초 기다린다."""
        self._executor.shutdown()
        self._node.destroy_node()
        self._context.try_shutdown()
        self._thread.join(timeout=2)
