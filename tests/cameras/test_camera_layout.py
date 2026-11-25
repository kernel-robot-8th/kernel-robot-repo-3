"""The six cameras of two arms: slots, presence of policy-owned cameras, duplicate devices, policy holds.

Device paths point at temporary files, and every feed here is presence-only or unset, so no device is opened.
"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from control_station.arms.config import arm_devices
from control_station.cameras.adapters import OpenCVArmCameraStreamManager, UnconfiguredArmCameraStreamManager

ARMS = (
    {"arm_id": "arm-01", "label": "왼쪽 팔", "board_serial": "F58AE68650323952582E3120FF05260E"},
    {"arm_id": "arm-02", "label": "오른쪽 팔", "board_serial": "A9F9354350323952582E3120FF070727"},
)


class CameraLayoutTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def device(self, name: str) -> str:
        path = self.root / name
        path.touch()
        return str(path)

    def manager(self, **env: str) -> OpenCVArmCameraStreamManager:
        cleared = {name: "" for name in os.environ if name.startswith("CONTROL_STATION_")}
        settings = {**cleared, "CONTROL_STATION_HEAD_CAMERA": "", **env}
        with mock.patch.dict(os.environ, settings):
            manager = OpenCVArmCameraStreamManager(
                joint_enabled=False, arm_cameras=arm_devices(ARMS, settings),
                policy_devices=lambda: self.running,
            )
        self.addCleanup(manager.close)
        return manager

    running: list[str] = []

    @staticmethod
    def by_id(manager: OpenCVArmCameraStreamManager) -> dict:
        return {camera.id: camera for camera in manager.snapshots()}

    def test_load_combines_detections_on_the_same_frame(self) -> None:
        manager = self.manager(CONTROL_STATION_LOAD_YOLO_WEIGHTS="cargo.pt")
        head = manager._feeds["head_camera"]
        self.assertEqual(head.load_detector.weights, "cargo.pt")

        raw = mock.Mock(shape=(480, 640, 3))
        raw.tobytes.return_value = b"raw pixels"
        annotated = mock.Mock(shape=raw.shape)
        head.detector = mock.Mock(state="ready", prompts=[], class_names=[])
        def detection(name, confidence, box):
            return {"class_name": name, "confidence": confidence, "bbox_xyxy_px": box}

        kiwi = [400, 140, 425, 165]
        head.detector.annotate.return_value = (annotated, [
            detection("tissue", 0.8, [455, 145, 492, 170]),
            detection("kiwi", 0.8, kiwi),
            detection("apple", 0.9, kiwi),
            detection("carrot", 0.49, [430, 200, 455, 230]),
            detection("kiwi", 0.7, [400, 200, 425, 225]),
        ])
        head.load_detector = mock.Mock(state="ready", prompts=[], class_names=[])
        head.load_detector.annotate.return_value = (raw, [detection("kiwi", 0.7, kiwi)])
        cv2 = mock.Mock()
        cv2.imencode.return_value = True, mock.Mock()
        with mock.patch.object(manager, "_find_markers", return_value=[]):
            manager._publish_frame(head, raw, "2026-11-24T00:00:00Z", 0.0, cv2)

        head.load_detector.annotate.assert_called_once_with(raw, render=False)
        head.detector.annotate.assert_called_once_with(raw)
        self.assertIs(cv2.imencode.call_args.args[1], annotated)
        normal = self.by_id(manager)["head_camera"]
        load = next(camera for camera in manager.load_snapshots() if camera.id == "head_camera")
        self.assertEqual(normal.detections[0]["class_name"], "tissue")
        self.assertEqual([item["class_name"] for item in load.detections], ["kiwi", "tissue", "kiwi"])
        self.assertEqual(len(normal.detections), 5)
        self.assertEqual(normal.frame_sequence, load.frame_sequence)
        head.load_detector.state = "error"
        load = next(camera for camera in manager.load_snapshots() if camera.id == "head_camera")
        self.assertEqual(load.detector_state, "error")
        self.assertEqual(self.by_id(manager)["head_camera"].detector_state, "ready")

    def test_four_arm_cameras_with_role_arm_and_link_state(self) -> None:
        manager = self.manager(
            CONTROL_STATION_ARM01_GRIPPER_CAMERA=self.device("left_gripper"),
            CONTROL_STATION_ARM01_OVERHEAD_CAMERA=self.device("left_top"),
            CONTROL_STATION_ARM02_GRIPPER_CAMERA=str(self.root / "right_gripper_not_plugged"),
        )
        cameras = self.by_id(manager)
        self.assertEqual({key for key in cameras if key.startswith("arm")},
                         {"arm01_gripper", "arm01_overhead", "arm02_gripper", "arm02_overhead"})
        top = cameras["arm01_overhead"]
        self.assertEqual((top.robot_id, top.position, top.role, top.opened_by, top.link_label),
                         ("arm-01", "OVERHEAD", "왼쪽 팔 탑 캠 (ACT 정책)", "policy", "연결됨"))
        self.assertEqual(top.setting, "CONTROL_STATION_ARM01_OVERHEAD_CAMERA")
        self.assertEqual(cameras["arm02_gripper"].link_label, "연결 전")
        self.assertIn("장치 경로가 없습니다", cameras["arm02_gripper"].error_message)
        self.assertEqual((cameras["arm02_overhead"].link_label, cameras["head_camera"].link_label), ("연결 전", "연결 전"))

    def test_two_settings_on_one_device_are_refused_with_the_by_path_hint(self) -> None:
        shared = self.device("same_camera")
        alias = self.root / "alias"
        alias.symlink_to(shared)
        manager = self.manager(CONTROL_STATION_ARM01_GRIPPER_CAMERA=shared, CONTROL_STATION_ARM02_GRIPPER_CAMERA=str(alias))
        cameras = self.by_id(manager)
        for camera_id in ("arm01_gripper", "arm02_gripper"):
            self.assertEqual(cameras[camera_id].link_label, "연결 전")
            self.assertIn("by-path", cameras[camera_id].error_message)
            self.assertIn("CONTROL_STATION_ARM02_GRIPPER_CAMERA", cameras[camera_id].error_message)

    def test_an_older_single_arm_feed_gives_way_to_the_arm_camera_on_its_device(self) -> None:
        top = self.device("top")
        manager = self.manager(CONTROL_STATION_FRONT_CAMERA=top, CONTROL_STATION_ARM02_OVERHEAD_CAMERA=top)
        cameras = self.by_id(manager)
        self.assertNotIn("front_camera", cameras)
        self.assertEqual(cameras["arm02_overhead"].link_label, "연결됨")

    def test_policy_runs_close_only_the_cameras_they_use(self) -> None:
        left, right = self.device("left_gripper"), self.device("right_gripper")
        manager = self.manager(CONTROL_STATION_ARM01_GRIPPER_CAMERA=left, CONTROL_STATION_ARM02_GRIPPER_CAMERA=right)
        feeds = manager._feeds
        self.running = [left]
        self.assertTrue(manager._held_for_policy(feeds["arm01_gripper"]))
        self.assertFalse(manager._held_for_policy(feeds["arm02_gripper"]))
        self.running = []
        manager._policy_devices_cache = (0.0, frozenset())
        self.assertTrue(manager.hold_for_policy([right], True))  # no capture is open here, so nothing to wait for
        self.assertTrue(manager._held_for_policy(feeds["arm02_gripper"]))
        manager.hold_for_policy([right], False)
        self.assertFalse(manager._held_for_policy(feeds["arm02_gripper"]))

    def test_unconfigured_station_lists_the_six_slots(self) -> None:
        cameras = UnconfiguredArmCameraStreamManager().snapshots()
        self.assertEqual([camera.id for camera in cameras],
                         ["head_camera", "shelf_camera", "arm01_gripper", "arm01_overhead", "arm02_gripper", "arm02_overhead"])
        self.assertEqual({camera.link_label for camera in cameras}, {"연결 전"})


if __name__ == "__main__":
    unittest.main()
