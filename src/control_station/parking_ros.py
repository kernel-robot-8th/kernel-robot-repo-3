"""Pinky parking judgment from the C++ ROS 2 node (docs/plans/pinky-parking-ros2-node.md).

Used when CONTROL_STATION_PARKING_SOURCE=ros. The node (cpp/parking_vision_ros) owns the judgment; this module
subscribes to /pinky_parking/state, calls /pinky_parking/check and turns both into the same JSON as the in-process
judgment (parking_observation.observe_parking, parking_tracker.ParkingTracker.snapshot), so the API, the web panel
and VERIFY_ARRIVAL stay as they are. A check that times out or cannot reach the node is not judged and is not
retried. Nothing here sends a robot command.
"""

from __future__ import annotations

import threading
import time
from datetime import datetime, timezone
from typing import Any, Callable

from control_station.parking_observation import OBSERVATION_SCHEMA_VERSION, ParkingObservationError

STATE_TOPIC = "/pinky_parking/state"
CHECK_SERVICE = "/pinky_parking/check"
# The node publishes at 10 Hz; a state older than this means the node or its timer stopped.
STALE_STATE_S = 2.0
# A check collects for at most 3 s; the rest covers reloading the spot and the service round trip.
CHECK_TIMEOUT_S = 6.0
NODE_UNAVAILABLE = "NODE_UNAVAILABLE"
# CheckParking error_code -> HTTP status of the in-process API (interface-contracts.md).
HTTP_STATUS = {
    "PARKING_SPOT_ID_INVALID": 400,
    "PARKING_SPOT_NOT_FOUND": 404,
    "PARKING_SPOT_INVALID": 409,
    "PARKING_CAMERA_UNAVAILABLE": 503,
    "PARKING_OBSERVATION_ERROR": 503,
}


def iso_time(stamp: Any) -> str | None:  # builtin_interfaces/Time
    nanoseconds = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if nanoseconds == 0:
        return None
    moment = datetime.fromtimestamp(nanoseconds / 1e9, tz=timezone.utc)
    return moment.isoformat(timespec="milliseconds")


def _points(values: Any) -> list[list[float]]:
    flat = [float(v) for v in values]
    return [[round(flat[i], 1), round(flat[i + 1], 1)] for i in range(0, len(flat) - 1, 2)]


def judgment_fields(message: Any) -> dict[str, object]:
    """parking_observation.judgment_fields() from ParkingState or a CheckParking response."""
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
    """parking_observation.image_overlay() from ParkingState or a CheckParking response."""
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


def live_payload(message: Any, feed_camera_id: str | None) -> dict[str, object]:
    """ParkingTracker.snapshot() from a ParkingState message.

    feed_camera_id is the control station feed that shows the same camera (the web panel streams it); the node's
    own image topic is kept as node_image_topic.
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
        "judgment_source": "ros:" + STATE_TOPIC,
    }
    if message.has_spot:
        payload.update(judgment_fields(message))
        payload["image_overlay"] = image_overlay(message)
    payload["since"] = iso_time(message.since)
    return payload


def observation_payload(response: Any, feed_camera_id: str | None) -> dict[str, object]:
    """observe_parking() from a successful CheckParking response; feed_camera_id as in live_payload()."""
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
    """Live state and confirmed checks of one spot from the parking node, in the in-process JSON shapes."""

    def __init__(
        self,
        domain_id: int,
        spot_id: str,
        feed_camera_id: str | None = None,
        clock: Callable[[], float] = time.monotonic,
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
                "CONTROL_STATION_PARKING_SOURCE=ros에는 ROS 2 Jazzy와 cpp/install/setup.bash를 source해야 합니다"
                " (parking_vision_interfaces)."
            ) from exc
        self.spot_id = spot_id
        self.feed_camera_id = feed_camera_id
        self._clock = clock
        self._request_type = CheckParking.Request
        self._lock = threading.Lock()
        self._state: tuple[float, Any] | None = None
        self._context = Context()
        self._context.init(domain_id=domain_id)
        self._node = Node("control_station_parking", context=self._context)
        # Matches the node's publisher, so a control station that starts later still gets the last state.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._node.create_subscription(ParkingState, STATE_TOPIC, self._on_state, qos)
        self._client = self._node.create_client(CheckParking, CHECK_SERVICE)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, name="parking-ros", daemon=True)
        self._thread.start()

    def _on_state(self, message: Any) -> None:
        with self._lock:
            self._state = (self._clock(), message)

    def _unavailable(self, reason: str) -> dict[str, object]:
        now = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        return {
            "state": NODE_UNAVAILABLE,
            "reason": reason,
            "spot_id": self.spot_id,
            "feed_camera_id": self.feed_camera_id,
            "frame_sequence": None,
            "captured_at": None,
            "updated_at": now,
            "display_only": True,
            "usable_for_robot_control": False,
            "judgment_source": "ros:" + STATE_TOPIC,
            "since": now,
        }

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            state = self._state
        if state is None:
            return self._unavailable(f"주차 판정 노드 상태를 받지 못했습니다: {STATE_TOPIC}")
        received_at, message = state
        age = self._clock() - received_at
        # Never show a stopped node's last IN_POSITION as current.
        if age > STALE_STATE_S:
            return self._unavailable(f"주차 판정 노드 상태가 {age:.1f}초째 갱신되지 않습니다: {STATE_TOPIC}")
        if message.spot_id != self.spot_id:
            return self._unavailable(
                f"주차 판정 노드가 다른 주차 위치를 추적합니다: {message.spot_id!r} (관제 설정 {self.spot_id!r})"
            )
        return live_payload(message, self.feed_camera_id)

    def check(self, spot_id: str, timeout_s: float = CHECK_TIMEOUT_S) -> dict[str, object]:
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
            # Not judged and not retried; a late response to this request is dropped.
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
        self._executor.shutdown()
        self._node.destroy_node()
        self._context.try_shutdown()
        self._thread.join(timeout=2)
