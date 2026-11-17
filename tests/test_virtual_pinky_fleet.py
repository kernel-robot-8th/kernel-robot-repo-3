from __future__ import annotations

import json
import math
import sys
import threading
import time
import unittest
from pathlib import Path

from control_station.pinky_fleet import PinkyFleetClient, PinkyFleetError

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FLEET_DIR = REPOSITORY_ROOT / "simulation" / "virtual_pinky_fleet"
sys.path.insert(0, str(FLEET_DIR))

from virtual_pinky_fleet import VirtualFleet, read_map, serve  # noqa: E402

ZONE_A = json.loads((REPOSITORY_ROOT / "config" / "receiving-zones.json").read_text(encoding="utf-8"))["zones"][0]["route_labels"]


class VirtualFleetTests(unittest.TestCase):
    def start(self, shared_waypoints: bool = False) -> PinkyFleetClient:
        fleet = VirtualFleet(FLEET_DIR, speed_mps=0.15, time_scale=200.0, shared_waypoints=shared_waypoints)
        server = serve(fleet, "127.0.0.1", 0)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(fleet.close)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return PinkyFleetClient(f"http://127.0.0.1:{server.server_address[1]}", status_max_age_s=0.0)

    def wait_phase(self, client: PinkyFleetClient, pinky_id: str, phase: str) -> dict:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            status = client.fleet_status(pinky_id)
            if status["phase"] == phase:
                return status
            time.sleep(0.02)
        self.fail(f"Pinky {pinky_id} never reached {phase}; last {status}")

    def test_order_mission_goes_through_every_phase_and_says_simulated(self) -> None:
        client = self.start()
        self.assertEqual(client.fleet_status("11")["phase"], "parked")
        dispatched = client.dispatch("11", ZONE_A)
        self.assertTrue(dispatched["accepted"] and dispatched["simulated"])
        self.assertEqual(dispatched["labels"], ["주차장", *ZONE_A, "주차장"])

        docking = self.wait_phase(client, "11", "docking_manual")
        self.assertTrue(docking["simulated"])
        with self.assertRaises(PinkyFleetError) as rejected:
            client.fleet_confirm("11", "dock_complete", "not-the-token")
        self.assertEqual(rejected.exception.error_code, "PINKY_SIGNAL_REJECTED")
        with self.assertRaises(PinkyFleetError):  # right token, wrong signal for this phase
            client.fleet_confirm("11", "received", docking["mission_token"])
        self.assertTrue(client.fleet_confirm("11", "dock_complete", docking["mission_token"])["simulated"])

        docked = self.wait_phase(client, "11", "docked_manual")
        self.assertNotEqual(docked["mission_token"], docking["mission_token"])  # a fresh token per waiting phase
        client.fleet_confirm("11", "received", docked["mission_token"])

        course = self.wait_phase(client, "11", "course_complete")
        self.assertEqual(course["location"], ZONE_A[-1])
        client.mission_complete("11", course["mission_token"])
        self.wait_phase(client, "11", "parked")
        pose = client.robots()["11"]["topics"]["tf_pose"]["value"]
        self.assertAlmostEqual(pose["x_m"], 0.7, places=3)
        self.assertAlmostEqual(pose["y_m"], 0.4, places=3)

    def test_route_needs_the_pinkys_own_waypoints_like_the_real_dashboard(self) -> None:
        client = self.start()
        with self.assertRaises(PinkyFleetError) as rejected:  # Pinky 10 has recorded only 5 and two midpoints
            client.dispatch("10", ZONE_A)
        self.assertIn("저장되지 않은 웨이포인트", str(rejected.exception))
        self.assertTrue(self.start(shared_waypoints=True).dispatch("10", ZONE_A)["accepted"])

    def test_stop_cancels_the_route_and_a_second_route_waits_for_the_first(self) -> None:
        client = self.start()
        client.dispatch("12", ZONE_A)
        with self.assertRaises(PinkyFleetError):
            client.dispatch("12", ZONE_A)
        status, answer = client.stop("12")
        self.assertEqual((status, answer["simulated"]), (202, True))
        self.assertEqual(client.fleet_status("12")["route_status"], "cancelled")
        self.assertEqual(client.fleet_status("12")["phase"], "parked")

    def test_map_is_read_like_nav2_map_server(self) -> None:
        grid = read_map(FLEET_DIR / "map-proto.yaml")
        self.assertEqual((grid["width"], grid["height"], grid["resolution_m"]), (44, 51, 0.05))
        width = grid["width"]
        row = lambda index: grid["cells"][index * width:(index + 1) * width]  # noqa: E731
        # Row 0 is the bottom of the map (y = origin): unknown, then the wall; the obstacles are near the top.
        self.assertTrue(all(cell == -1 for cell in row(0)))
        self.assertTrue(all(cell == 100 for cell in row(1)[1:]))
        self.assertIn(100, row(grid["height"] - 2)[2:-2])
        self.assertNotIn(100, row(2)[2:-2])
        parking = (0.7, 0.4)
        col = math.floor((parking[0] - grid["origin_x_m"]) / grid["resolution_m"])
        row = math.floor((parking[1] - grid["origin_y_m"]) / grid["resolution_m"])
        self.assertEqual(grid["cells"][row * width + col], 0)  # P1's parking spot is free space


if __name__ == "__main__":
    unittest.main()
