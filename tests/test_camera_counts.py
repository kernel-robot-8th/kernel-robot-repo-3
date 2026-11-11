"""Counting shelf stock and the Pinky load from camera detections (median over fresh frames, slot and tray filters)."""

from __future__ import annotations

import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from control_station.act_policies import load_act_policies
from control_station.arms import load_arms
from control_station.camera_counts import (
    CountUnavailable, PinkyLoadCounter, ShelfStockCounter, sample_frames, tray_region_from_spot,
)
from control_station.mock_world import MockWorld
from control_station.models import CameraConnectionState, CameraSnapshot, CameraStreamState, FrameFreshness
from control_station.order_flow import OrderDevices, OrderFlow
from control_station.shelf_slot_map import load_shelf_slot_map
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator


def detection(name: str, box: list[float], slot: str | None = None) -> dict:
    item: dict = {"class_name": name, "confidence": 0.9, "bbox_xyxy_px": box}
    if slot is not None:  # what the fake slot zones below will assign
        item["fake_slot"] = slot
    return item


class FakeCamera:
    """Each call is a new fresh frame with the next detection list (the last one repeats)."""

    def __init__(self, camera_id: str, frames: list[list[dict]], *, fresh: bool = True, detector: str = "ready") -> None:
        self.camera_id, self.frames, self.fresh, self.detector, self.sequence = camera_id, frames, fresh, detector, 0

    def snapshots(self) -> list[CameraSnapshot]:
        self.sequence += 1
        detections = self.frames[min(self.sequence - 1, len(self.frames) - 1)]
        return [CameraSnapshot(
            id=self.camera_id, robot_id=None, position="SHELF", connection_state=CameraConnectionState.CONNECTED,
            stream_state=CameraStreamState.STREAMING,
            frame_freshness=FrameFreshness.FRESH if self.fresh else FrameFreshness.STALE,
            frame_sequence=self.sequence, frame_width=640, frame_height=480, detector_state=self.detector,
            detections=detections,
        )]


class FakeSlotZones:
    def __init__(self, status: str = "READY") -> None:
        self.status = status

    def annotate(self, camera: dict) -> dict:
        detections = []
        for item in camera["detections"]:
            slot = item.get("fake_slot")
            detections.append({**item, "slot_assignment": {"status": "IN_SLOT", "slot_id": slot} if slot else {"status": "OUTSIDE"}})
        return {**camera, "slot_zones": {"status": self.status}, "detections": detections}


class CountTests(unittest.TestCase):
    def test_median_over_frames_ignores_one_missed_or_doubled_detection(self) -> None:
        apple = detection("apple", [10, 10, 20, 20], "slot_a_1_1")
        frames = [[apple, apple], [apple], [apple, apple], [apple, apple, apple], [apple, apple]]
        camera = FakeCamera("shelf_camera", frames)
        counter = ShelfStockCounter(camera.snapshots, FakeSlotZones(), load_shelf_slot_map(), load_arms(), max_wait_s=2)
        observation = counter()
        self.assertEqual(observation["shelves"], {"shelf_a": {"apple": 2}, "shelf_b": {}})
        self.assertFalse(observation["simulated"])
        self.assertEqual(len(observation["frames"]["shelf_camera"]), 5)

    def test_stock_counts_only_objects_in_a_slot_of_each_shelf(self) -> None:
        frame = [
            detection("carrot", [0, 0, 5, 5], "slot_a_2_1"),
            detection("apple", [0, 0, 5, 5], "slot_b_2_2"),
            detection("apple", [0, 0, 5, 5], "slot_b_2_1"),
            detection("apple", [300, 300, 320, 320]),  # on the Pinky or in the gripper: not stock
        ]
        counter = ShelfStockCounter(FakeCamera("shelf_camera", [frame]).snapshots, FakeSlotZones(),
                                    load_shelf_slot_map(), load_arms(), max_wait_s=2)
        self.assertEqual(counter()["shelves"], {"shelf_a": {"carrot": 1}, "shelf_b": {"apple": 2}})

    def test_no_count_without_fresh_frames_a_ready_detector_or_slot_zones(self) -> None:
        cases = (
            (FakeCamera("shelf_camera", [[]], fresh=False), FakeSlotZones(), "COUNT_CAMERA_NOT_FRESH"),
            (FakeCamera("shelf_camera", [[]], detector="loading"), FakeSlotZones(), "COUNT_DETECTOR_NOT_READY"),
            (FakeCamera("other", [[]]), FakeSlotZones(), "COUNT_CAMERA_NOT_FOUND"),
            (FakeCamera("shelf_camera", [[]]), FakeSlotZones("SLOT_MAP_MISMATCH"), "COUNT_SLOT_ZONES_NOT_READY"),
        )
        for camera, zones, code in cases:
            with self.subTest(code=code), self.assertRaises(CountUnavailable) as raised:
                ShelfStockCounter(camera.snapshots, zones, load_shelf_slot_map(), load_arms(), max_wait_s=0.3)()
            self.assertEqual(raised.exception.error_code, code)

    def test_load_counts_inside_the_tray_and_skips_ignored_classes(self) -> None:
        frame = [
            detection("apple", [400, 150, 420, 170]),   # center (410, 160): on the tray
            detection("carrot", [100, 100, 120, 120]),  # on the shelf
            detection("car", [400, 150, 480, 230]),     # the Pinky itself
        ]
        counter = PinkyLoadCounter(FakeCamera("head_camera", [frame]).snapshots, "head_camera", [380, 120, 500, 250],
                                   max_wait_s=2)
        observation = counter("10")
        self.assertEqual(observation["counts"], {"apple": 1})
        self.assertEqual(observation["region_px"], [380, 120, 500, 250])
        with self.assertRaises(CountUnavailable) as raised:
            PinkyLoadCounter(FakeCamera("head_camera", [frame]).snapshots, "head_camera", None)("10")
        self.assertEqual(raised.exception.error_code, "COUNT_TRAY_REGION_UNKNOWN")

    def test_tray_region_comes_from_the_registered_pinky_marker(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            spot = Path(directory) / "spot.json"
            spot.write_text(json.dumps({"reference_corners_px": [[420, 200], [420, 160], [460, 160], [460, 200]]}))
            self.assertEqual(tray_region_from_spot(spot, 1.5), [380.0, 120.0, 500.0, 240.0])
            self.assertIsNone(tray_region_from_spot(Path(directory) / "missing.json", 1.5))

    def test_sample_frames_waits_for_distinct_frames(self) -> None:
        camera = FakeCamera("shelf_camera", [[]])
        frames = sample_frames(camera.snapshots, "shelf_camera", 3, 1.0)
        self.assertEqual([frame["frame_sequence"] for frame in frames], [1, 2, 3])

    def test_an_order_stops_when_the_shelf_cannot_be_counted(self) -> None:
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        store = TaskStore(str(Path(directory.name) / "t.sqlite3"))
        self.addCleanup(store.close)
        world = MockWorld(drive_s=0.1, pick_s=0.05)
        stale = ShelfStockCounter(FakeCamera("shelf_camera", [[]], fresh=False).snapshots, FakeSlotZones(),
                                  load_shelf_slot_map(), load_arms(), max_wait_s=0.2)
        devices = OrderDevices(stale, world.pinky_load, world.fleet_status, world.fleet_confirm, world.parking,
                               world.pick, world.dispatch)
        orchestrator = MockTaskOrchestrator(store, order_flow=OrderFlow(load_arms(), load_act_policies(), devices,
                                                                         dwell_s=0.0, poll_s=0.02))
        self.addCleanup(orchestrator.close)
        task = store.create_task("핑키 적재 위치", "주문", order_items=[{"object_class": "carrot", "quantity": 1}])
        orchestrator.start(task["id"])
        deadline = time.monotonic() + 5
        while store.get_task(task["id"])["status"] not in {"SUCCEEDED", "FAILED"} and time.monotonic() < deadline:
            time.sleep(0.05)
        failed = store.get_task(task["id"])
        self.assertEqual((failed["status"], failed["error"]["code"]), ("FAILED", "COUNT_CAMERA_NOT_FRESH"))
        self.assertIn("선반 재고", failed["error"]["message"])


if __name__ == "__main__":
    unittest.main()
