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
    CountUnavailable, PinkyLoadCounter, ShelfStockCounter, camera_inventory, head_middle_view_from_env, head_shelf_view_from_env, sample_frames, tray_region_from_spot,
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
        self.markers: list[dict] = []  # ArUco markers the next frames carry (set by a test)

    def snapshots(self) -> list[CameraSnapshot]:
        self.sequence += 1
        detections = self.frames[min(self.sequence - 1, len(self.frames) - 1)]
        return [CameraSnapshot(
            id=self.camera_id, robot_id=None, position="SHELF", connection_state=CameraConnectionState.CONNECTED,
            stream_state=CameraStreamState.STREAMING,
            frame_freshness=FrameFreshness.FRESH if self.fresh else FrameFreshness.STALE,
            frame_sequence=self.sequence, frame_width=640, frame_height=480, detector_state=self.detector,
            detections=detections, markers=list(self.markers),
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

    def test_live_load_lists_the_newest_frame_on_the_tray_for_display(self) -> None:
        frame = [
            detection("apple", [400, 150, 420, 170]),   # on the tray
            detection("carrot", [440, 200, 460, 220]),  # on the tray
            detection("carrot", [100, 100, 120, 120]),  # on the shelf
            detection("car", [400, 150, 480, 230]),     # the Pinky itself
        ]
        region = [380, 120, 500, 250]
        live = PinkyLoadCounter(FakeCamera("head_camera", [frame]).snapshots, "head_camera", region).live()
        self.assertEqual(live["state"], "READY")
        self.assertEqual(live["counts"], {"apple": 1, "carrot": 1})
        self.assertEqual([item["class_name"] for item in live["items"]], ["apple", "carrot"])
        self.assertFalse(live["usable_for_robot_control"])
        empty = PinkyLoadCounter(FakeCamera("head_camera", [[]]).snapshots, "head_camera", region).live()
        self.assertEqual((empty["state"], empty["counts"]), ("READY", {}))

    def test_camera_inventory_uses_max_per_class_and_filters_regions(self) -> None:
        apple = detection("apple", [30, 20, 50, 40])
        head = FakeCamera("head_camera", [[apple, detection("carrot", [400, 100, 420, 120])]])
        front = FakeCamera("front_camera", [[apple, detection("kiwi", [80, 20, 100, 40]),
                            {**detection("watermelon", [30, 50, 50, 70]), "confidence": 0.49}]])
        rows = camera_inventory(lambda: head.snapshots() + front.snapshots(),
                                [20, 0, 155, 480], [0, 0, 640, 480])
        self.assertEqual([(row["label"], row["count"]) for row in rows],
                         [("사과", 1), ("수박", 0), ("당근", 0), ("키위", 1)])
        self.assertTrue(all(row["count"] is None for row in camera_inventory(lambda: [], None, None)))

    def test_head_display_filters_region_and_confidence(self) -> None:
        shelf = {**detection("apple", [40, 20, 60, 40]), "confidence": 0.5}
        low = {**detection("kiwi", [40, 40, 60, 60]), "confidence": 0.49}
        tray = detection("carrot", [410, 120, 430, 140])
        car = detection("car", [90, 260, 130, 290])
        pinky = detection("pinky", [40, 60, 60, 80])
        middle = detection("kiwi", [230, 60, 250, 80])
        camera = FakeCamera("head_camera", [[shelf, low, tray, car, pinky, middle]])
        with mock.patch.dict(os.environ, {"CONTROL_STATION_HEAD_SHELF_REGION": "20,0,155,480"}):
            view = head_shelf_view_from_env(camera.snapshots)
        self.assertEqual(view.live()["counts"], {"apple": 1, "car": 1})
        with mock.patch.dict(os.environ, {"CONTROL_STATION_HEAD_MIDDLE_REGION": "156,0,365,480"}):
            workbench = head_middle_view_from_env(camera.snapshots)
        self.assertEqual(workbench.live()["counts"], {"kiwi": 1})
        load = PinkyLoadCounter(camera.snapshots, "head_camera", [366, 108, 522, 264])
        self.assertEqual(load.live()["counts"], {"carrot": 1})

    def test_live_load_keeps_an_object_that_flickers_below_the_threshold_in_some_frames(self) -> None:
        melon = detection("watermelon", [410, 170, 430, 190])
        camera = FakeCamera("head_camera", [[melon], [], [], [melon], []])  # seen in 2 of 5 frames
        live = PinkyLoadCounter(camera.snapshots, "head_camera", [366, 108, 522, 264]).live()
        self.assertEqual(live["counts"], {"watermelon": 1})
        self.assertEqual(len(live["frames"]), 5)

    def test_live_load_holds_an_object_for_a_few_seconds_after_it_was_last_seen(self) -> None:
        melon = detection("watermelon", [410, 170, 430, 190])
        camera = FakeCamera("head_camera", [[melon]] * 5 + [[]])  # seen, then gone in every later frame
        counter = PinkyLoadCounter(camera.snapshots, "head_camera", [366, 108, 522, 264])
        counter.live_hold_s = 5
        self.assertEqual(counter.live()["counts"], {"watermelon": 1})
        held = counter.live()
        self.assertEqual(held["counts"], {"watermelon": 1})  # not seen now, still within 5 s
        self.assertFalse(held["items"][0]["seen_now"])
        with mock.patch("control_station.camera_counts.time.monotonic", return_value=time.monotonic() + 6):
            self.assertEqual(counter.live()["counts"], {})  # 6 s later it is dropped

    def test_live_load_reports_why_it_cannot_count_instead_of_zero(self) -> None:
        region = [380, 120, 500, 250]
        cases = [
            (PinkyLoadCounter(FakeCamera("head_camera", [[]]).snapshots, "head_camera", None), "NO_TRAY_REGION"),
            (PinkyLoadCounter(FakeCamera("other", [[]]).snapshots, "head_camera", region), "NO_CAMERA"),
            (PinkyLoadCounter(FakeCamera("head_camera", [[]], detector="loading").snapshots, "head_camera", region),
             "DETECTOR_NOT_READY"),
            (PinkyLoadCounter(FakeCamera("head_camera", [[]], fresh=False).snapshots, "head_camera", region), "NOT_FRESH"),
        ]
        for counter, state in cases:
            with self.subTest(state=state):
                live = counter.live()
                self.assertEqual(live["state"], state)
                self.assertEqual(live["counts"], {})
                self.assertTrue(live["message"])

    def test_live_load_marks_the_pinky_arrived_by_its_marker_and_keeps_it_while_cargo_hides_it(self) -> None:
        camera = FakeCamera("head_camera", [[detection("apple", [430, 170, 450, 190])]])
        arrivals = [{"pinky_id": "12", "name": "P2", "marker_id": 21, "spot_id": "pinky_dock_1", "spot_center_px": [444.0, 185.0]}]
        counter = PinkyLoadCounter(camera.snapshots, "head_camera", [366, 108, 522, 264], arrivals=arrivals, arrival_tolerance_px=30)
        self.assertEqual(counter.live()["pinky"]["state"], "NOT_SEEN")
        camera.markers = [{"id": 21, "center_px": [520.0, 185.0]}]  # 76 px from the spot
        far = counter.live()["pinky"]
        self.assertEqual((far["state"], far["offset_px"], far["name"]), ("NOT_AT_SPOT", 76.0, "P2"))
        camera.markers = [{"id": 21, "center_px": [450.0, 193.0]}]
        arrived = counter.live()
        self.assertEqual((arrived["pinky"]["state"], arrived["pinky"]["offset_px"]), ("ARRIVED", 10.0))
        self.assertEqual(arrived["counts"], {"apple": 1})
        camera.markers = []  # cargo covers the marker: still arrived
        hidden = counter.live()["pinky"]
        self.assertEqual((hidden["state"], hidden["marker_visible"]), ("ARRIVED", False))
        camera.markers = [{"id": 21, "center_px": [444.0, 400.0]}, {"id": 1, "center_px": [359.0, 430.0]}]  # driving off
        self.assertEqual(counter.live()["pinky"]["state"], "NOT_AT_SPOT")
        camera.markers = []
        self.assertEqual(counter.live()["pinky"]["state"], "NOT_SEEN")
        self.assertEqual(PinkyLoadCounter(camera.snapshots, "head_camera", [0, 0, 1, 1]).live()["pinky"]["state"], "NOT_CONFIGURED")

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
