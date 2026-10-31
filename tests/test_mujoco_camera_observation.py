from __future__ import annotations

import hashlib
import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np

from control_station.app import ControlStationHandler
from control_station.camera_adapters import (
    MUJOCO_ROS_PIXEL_TRANSFORM_ID,
    Ros2SimulationCameraStreamManager,
)
from control_station.models import (
    CameraConnectionState,
    CameraSnapshot,
    CameraStreamState,
    FrameFreshness,
)


class _StubDetector:
    state = "ready"
    detector_model_type = "YOLO_CLOSED_SET"
    prompts: list[str] = []
    class_names = ["box"]
    confidence_threshold = 0.25
    weights_sha256 = "a" * 64
    inference_image_size = 640
    device = None
    ultralytics_version = "test"

    def detect(self, image: np.ndarray) -> list[dict[str, object]]:
        if image.shape != (2, 3, 3):
            raise AssertionError("detector received unexpected frame dimensions")
        return [
            {
                "class_name": "box",
                "confidence": 0.9,
                "center_px": [1.5, 1.0],
                "bbox_xyxy_px": [0, 0, 3, 2],
            }
        ]


class MujocoCameraObservationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.pixels = np.zeros((2, 3, 3), dtype=np.uint8)
        self.raw_pixels = self.pixels.tobytes()
        self.frame_sha256 = hashlib.sha256(self.raw_pixels).hexdigest()
        self.captured_at = "2026-10-31T00:00:00.000+00:00"
        self.manager = object.__new__(Ros2SimulationCameraStreamManager)
        self.manager._detector = _StubDetector()
        self.manager._feed = SimpleNamespace(
            lock=threading.Lock(),
            raw_frames={
                7: (
                    self.frame_sha256,
                    self.raw_pixels,
                    3,
                    2,
                    self.captured_at,
                    time.monotonic(),
                )
            },
            connection_state=CameraConnectionState.CONNECTED,
            stream_state=CameraStreamState.STREAMING,
        )

    def _camera(self, *, sequence: int = 7) -> CameraSnapshot:
        return CameraSnapshot(
            id=Ros2SimulationCameraStreamManager.CAMERA_ID,
            robot_id="arm-01",
            position="FIXED",
            connection_state=CameraConnectionState.CONNECTED,
            stream_state=CameraStreamState.STREAMING,
            frame_freshness=FrameFreshness.FRESH,
            last_frame_at=self.captured_at,
            frame_sequence=sequence,
            frame_width=3,
            frame_height=2,
            frame_sha256=self.frame_sha256,
            pixel_transform_id=MUJOCO_ROS_PIXEL_TRANSFORM_ID,
            source_stamp={"sec": 12, "nanosec": 34},
            source_frame_id="placeholder_fixed_camera_optical_frame",
            detector_state="ready",
            detector_classes=["box"],
        )

    def test_inference_preserves_exact_frame_provenance(self) -> None:
        result = self.manager.detect_for_observation(self._camera())

        self.assertEqual(result.detector_inference_frame_sequence, 7)
        self.assertEqual(len(result.detections), 1)
        detection = result.detections[0]
        self.assertEqual(detection["camera_id"], result.id)
        self.assertEqual(detection["frame_sequence"], 7)
        self.assertEqual(detection["frame_sha256"], self.frame_sha256)
        self.assertEqual(detection["pixel_transform_id"], MUJOCO_ROS_PIXEL_TRANSFORM_ID)

    def test_unretained_frame_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "no longer retained"):
            self.manager.detect_for_observation(self._camera(sequence=6))

    def test_fixed_get_distinguishes_not_run_from_empty_detection(self) -> None:
        camera = self._camera()
        not_run = ControlStationHandler._target_observation(camera, "box")
        self.assertEqual(not_run["status"], "DETECTOR_NOT_RUN")
        self.assertIsNone(not_run["candidate_count"])
        self.assertEqual(not_run["source_stamp"], camera.source_stamp)
        self.assertEqual(
            not_run["source_stamp_clock_status"],
            "ROS_SOURCE_HEADER_NOT_ALIGNED_WITH_HOST_CLOCK",
        )

        inferred = CameraSnapshot(
            **{**camera.__dict__, "detector_inference_frame_sequence": camera.frame_sequence}
        )
        no_target = ControlStationHandler._target_observation(inferred, "box")
        self.assertEqual(no_target["status"], "TARGET_NOT_FOUND")
        self.assertEqual(no_target["candidate_count"], 0)

        stale_detection = CameraSnapshot(
            **{
                **camera.__dict__,
                "detector_inference_frame_sequence": camera.frame_sequence - 1,
                "detections": [
                    {
                        "class_name": "box",
                        "camera_id": camera.id,
                        "captured_at": camera.last_frame_at,
                        "frame_sequence": camera.frame_sequence - 1,
                        "frame_sha256": camera.frame_sha256,
                        "pixel_transform_id": camera.pixel_transform_id,
                    }
                ],
            }
        )
        stale = ControlStationHandler._target_observation(stale_detection, "box")
        self.assertEqual(stale["status"], "DETECTOR_NOT_RUN")
        self.assertEqual(stale["candidates"], [])


if __name__ == "__main__":
    unittest.main()
