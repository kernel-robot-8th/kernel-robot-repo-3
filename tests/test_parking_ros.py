from __future__ import annotations

import threading
import unittest
from types import SimpleNamespace

from control_station.parking_observation import ParkingObservationError
from control_station.parking_ros import (
    NODE_UNAVAILABLE,
    STALE_STATE_S,
    RosParkingSource,
    live_payload,
    observation_payload,
)

# Keys of ParkingTracker._state() for a state with a spot, and of observe_parking().
LIVE_KEYS = {
    "state", "reason", "spot_id", "feed_camera_id", "frame_sequence", "captured_at", "updated_at", "display_only",
    "usable_for_robot_control", "offset", "tolerance", "position_ratio", "yaw_ratio", "deviation_level",
    "view_check", "view_shift_px", "image_overlay", "since",
}
OBSERVATION_KEYS = {
    "observation_schema_version", "spot_id", "marker_id", "status", "reason", "offset", "tolerance",
    "position_ratio", "yaw_ratio", "deviation_level", "view_check", "view_shift_px", "frames_used",
    "frames_examined", "feed_camera_id", "calibration_camera_id", "frame_sequence_range", "captured_from",
    "captured_until", "requested_at", "spot_registration_sha256", "intrinsic_calibration_sha256",
    "opencv_version", "registration_opencv_version", "image_overlay", "usable_for_robot_control",
}


def time(seconds: float) -> SimpleNamespace:
    return SimpleNamespace(sec=int(seconds), nanosec=int(round((seconds % 1) * 1e9)))


def judged(**fields: object) -> dict[str, object]:
    """Fields ParkingState and the CheckParking response share, for a judged MISALIGNED/NEAR result."""
    base: dict[str, object] = dict(
        spot_id="pinky_dock_1", marker_id=21,
        has_offset=True, dx_mm=44.04, dy_mm=3.36, distance_mm=44.168, dyaw_deg=-0.6712,
        marker_forward_axis="-x", has_pinky_frame=True, forward_mm=-44.04, left_mm=-3.36,
        has_ratios=True, position_ratio=1.7667, yaw_ratio=0.1342, deviation_level="NEAR",
        tolerance_position_mm=25.0, tolerance_yaw_deg=5.0, view_check=True, has_view_shift=True, view_shift_px=6.24,
        has_view_shift_xy=True, view_shift_x_px=-6.11, view_shift_y_px=1.38, image_width=640, image_height=480,
        registered_corners_px=[441.71, 215.93, 439.2, 174.6, 481.3, 175.6, 484.9, 216.9],
        has_marker_corners=True, marker_corners_px=[427.9, 163.0, 425.9, 121.6, 467.9, 122.6, 471.6, 163.9],
        anchor_centers_px=[375.59, 435.48],
    )
    base.update(fields)
    return base


def state_message(**fields: object) -> SimpleNamespace:
    values = judged(
        header=SimpleNamespace(stamp=time(1793700000.25), frame_id="ceiling_camera"),
        state="OFF_POSITION", reason="1초 이상 멈춰 있음", since=time(1793699990.0), updated_at=time(1793700000.3),
        feed_camera_id="/ceiling_camera/image_raw", has_frame=True, frame_sequence=342, has_spot=True,
    )
    values.update(fields)
    return SimpleNamespace(**values)


def check_response(**fields: object) -> SimpleNamespace:
    values = judged(
        ok=True, error_code="", message="", observation_schema_version=1, status="MISALIGNED",
        reason="position 44 mm > 25 mm", frames_used=15, frames_examined=15,
        feed_camera_id="/ceiling_camera/image_raw", calibration_camera_id="ceiling_camera",
        first_frame_sequence=10, last_frame_sequence=24, requested_at=time(1793700000.0),
        captured_from=time(1793700000.03), captured_until=time(1793700000.5),
        spot_registration_sha256="a" * 64, intrinsic_calibration_sha256="b" * 64,
        opencv_version="4.6.0", registration_opencv_version="",
    )
    values.update(fields)
    return SimpleNamespace(**values)


class PayloadTests(unittest.TestCase):
    def test_live_payload_has_the_tracker_fields(self) -> None:
        payload = live_payload(state_message(), "head_camera")
        self.assertEqual(set(payload) - {"judgment_source", "node_image_topic"}, LIVE_KEYS)
        self.assertEqual(payload["feed_camera_id"], "head_camera")  # the web panel streams /streams/head_camera.mjpg
        self.assertEqual(payload["node_image_topic"], "/ceiling_camera/image_raw")
        self.assertEqual(payload["state"], "OFF_POSITION")
        self.assertEqual(payload["offset"], {
            "frame": "REGISTERED_SPOT_MARKER_AXES", "dx_mm": 44.0, "dy_mm": 3.4, "distance_mm": 44.2,
            "dyaw_deg": -0.67, "marker_forward_axis": "-x", "forward_mm": -44.0, "left_mm": -3.4,
        })
        self.assertEqual((payload["position_ratio"], payload["yaw_ratio"]), (1.77, 0.13))
        self.assertEqual(payload["view_check"], "ON")
        self.assertEqual(payload["view_shift_px"], 6.2)
        self.assertEqual(payload["captured_at"], "2026-11-03T10:00:00.250+00:00")
        overlay = payload["image_overlay"]
        self.assertEqual(overlay["registered_corners_px"][0], [441.7, 215.9])
        self.assertEqual(overlay["view_shift_xy_px"], [-6.1, 1.4])
        self.assertEqual(overlay["anchor_centers_px"], [[375.6, 435.5]])
        self.assertTrue(payload["display_only"])
        self.assertFalse(payload["usable_for_robot_control"])

    def test_unjudged_state_has_nulls_like_the_tracker(self) -> None:
        payload = live_payload(state_message(
            state="VIEW_UNVERIFIED", has_offset=False, has_ratios=False, has_marker_corners=False, has_view_shift_xy=False,
        ), "head_camera")
        self.assertIsNone(payload["offset"])
        self.assertIsNone(payload["deviation_level"])
        self.assertIsNone(payload["position_ratio"])
        self.assertIsNone(payload["image_overlay"]["marker_corners_px"])

    def test_spot_invalid_state_has_no_spot_fields(self) -> None:
        payload = live_payload(state_message(state="SPOT_INVALID", has_spot=False, has_frame=False), "head_camera")
        self.assertNotIn("tolerance", payload)
        self.assertNotIn("image_overlay", payload)
        self.assertIsNone(payload["frame_sequence"])

    def test_observation_payload_has_the_check_fields(self) -> None:
        payload = observation_payload(check_response(), "head_camera")
        self.assertEqual(set(payload) - {"judgment_source", "node_image_topic"}, OBSERVATION_KEYS)
        self.assertEqual(payload["feed_camera_id"], "head_camera")
        self.assertEqual(payload["status"], "MISALIGNED")
        self.assertEqual(payload["frame_sequence_range"], [10, 24])
        self.assertIsNone(payload["registration_opencv_version"])
        self.assertEqual(payload["requested_at"], "2026-11-03T10:00:00.000+00:00")
        self.assertFalse(payload["usable_for_robot_control"])


class FakeFuture:
    def __init__(self, response: object | None, finish: bool) -> None:
        self.response, self.finish = response, finish

    def add_done_callback(self, callback) -> None:  # type: ignore[no-untyped-def]
        if self.finish:
            callback(self)

    def result(self) -> object | None:
        return self.response


class FakeClient:
    def __init__(self, ready: bool = True, response: object | None = None, finish: bool = True) -> None:
        self.ready, self.response, self.finish = ready, response, finish
        self.removed = False
        self.calls = 0

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.ready

    def call_async(self, request: object) -> FakeFuture:
        self.calls += 1
        return FakeFuture(self.response, self.finish)

    def remove_pending_request(self, future: object) -> None:
        self.removed = True


def source(client: FakeClient | None = None, now: float = 100.0) -> RosParkingSource:
    """A RosParkingSource without rclpy: only the parts snapshot() and check() use."""
    instance = object.__new__(RosParkingSource)
    instance.spot_id = "pinky_dock_1"
    instance.feed_camera_id = "head_camera"
    instance._clock = lambda: now
    instance._lock = threading.Lock()
    instance._state = None
    instance._client = client
    instance._request_type = SimpleNamespace
    return instance


class SnapshotTests(unittest.TestCase):
    def test_no_state_yet_is_node_unavailable(self) -> None:
        payload = source().snapshot()
        self.assertEqual(payload["state"], NODE_UNAVAILABLE)
        self.assertTrue(payload["display_only"])

    def test_stale_state_is_not_shown_as_current(self) -> None:
        stale = source(now=100.0)
        stale._state = (100.0 - STALE_STATE_S - 0.1, state_message(state="IN_POSITION"))
        self.assertEqual(stale.snapshot()["state"], NODE_UNAVAILABLE)

    def test_state_of_another_spot_is_not_shown(self) -> None:
        other = source()
        other._state = (100.0, state_message(spot_id="other_dock"))
        self.assertEqual(other.snapshot()["state"], NODE_UNAVAILABLE)

    def test_fresh_state(self) -> None:
        fresh = source()
        fresh._state = (99.9, state_message())
        self.assertEqual(fresh.snapshot()["state"], "OFF_POSITION")


class CheckTests(unittest.TestCase):
    def assert_error(self, client: FakeClient, http_status: int, error_code: str) -> None:
        with self.assertRaises(ParkingObservationError) as raised:
            source(client).check("pinky_dock_1", timeout_s=0.05)
        self.assertEqual((raised.exception.http_status, raised.exception.error_code), (http_status, error_code))

    def test_judged_response(self) -> None:
        self.assertEqual(source(FakeClient(response=check_response())).check("pinky_dock_1")["status"], "MISALIGNED")

    def test_node_errors_keep_the_http_contract(self) -> None:
        for error_code, http_status in (
            ("PARKING_SPOT_ID_INVALID", 400), ("PARKING_SPOT_NOT_FOUND", 404), ("PARKING_SPOT_INVALID", 409),
            ("PARKING_CAMERA_UNAVAILABLE", 503),
        ):
            self.assert_error(FakeClient(response=check_response(ok=False, error_code=error_code)), http_status, error_code)

    def test_unreachable_node_is_not_judged(self) -> None:
        self.assert_error(FakeClient(ready=False), 503, "PARKING_NODE_UNAVAILABLE")

    def test_timeout_is_not_retried(self) -> None:
        client = FakeClient(response=check_response(), finish=False)
        self.assert_error(client, 503, "PARKING_NODE_TIMEOUT")
        self.assertEqual(client.calls, 1)
        self.assertTrue(client.removed)


if __name__ == "__main__":
    unittest.main()
