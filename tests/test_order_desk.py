"""Order desk: receive from the order window, preview the assignment, start with a chosen Pinky, reject."""

from __future__ import annotations

import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from control_station.act_policies import load_act_policies
from control_station.arms import load_arms
from control_station.mock_world import MockWorld
from control_station.order_desk import OrderDesk, OrderDeskError
from control_station.order_flow import OrderFlow
from control_station.receiving_zones import ReceivingZone
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator, TaskExecutionConflict
from test_api_routes import ApiTestCase
from test_order_flow import CARROT_APPLE, TERMINAL, devices

ZONES = {"zone_a": ReceivingZone("zone_a", "수령 구역 A", ("1", "5", "9"))}


def make_desk(store: TaskStore, *, zones=ZONES) -> tuple[OrderDesk, MockWorld, MockTaskOrchestrator]:
    world = MockWorld(drive_s=0.2, pick_s=0.05)
    flow = OrderFlow(load_arms(), load_act_policies(), devices(world), dwell_s=0.0, poll_s=0.02,
                     arrival_timeout_s=10.0, signal_timeout_s=2.0)
    orchestrator = MockTaskOrchestrator(store, order_flow=flow)
    desk = OrderDesk(store, orchestrator, devices(world), load_arms(), load_act_policies(), zones,
                     pick_stats=lambda: store.pick_success_stats(simulated=True))
    return desk, world, orchestrator


class OrderDeskTest(unittest.TestCase):
    def setUp(self) -> None:
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.store = TaskStore(str(Path(directory.name) / "t.sqlite3"))
        self.addCleanup(self.store.close)
        self.desk, self.world, orchestrator = make_desk(self.store)
        self.addCleanup(orchestrator.close)

    def wait(self, order_id: str) -> dict:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            order = self.store.get_task(order_id)
            if order["status"] in TERMINAL:
                return order
            time.sleep(0.02)
        self.fail(f"order did not finish: {self.store.get_task(order_id)['status']}")

    def test_received_order_waits_without_a_pinky(self) -> None:
        order = self.desk.receive([{"object_class": "사과", "quantity": 2}], None)  # the only zone is the default
        self.assertEqual((order["status"], order["pinky_id"], order["receiving_zone"]), ("PENDING", None, "zone_a"))
        self.assertEqual((order["destination"], order["arm_job"]), ("수령 구역 A", "사과 2 적재"))
        self.assertEqual({step["robot_id"] for step in order["steps"] if step["step_type"] == "CONFIRM_DOCKING"}, {None})
        self.assertEqual([o["id"] for o in self.desk.pending()], [order["id"]])
        self.assertNotIn("10", self.store.busy_pinkies())  # a waiting order holds no Pinky

    def test_catalog_lists_products_stock_and_zones(self) -> None:
        catalog = self.desk.catalog()
        stock = {item["object_class"]: item["available"] for item in catalog["products"]}
        self.assertEqual(stock, {"carrot": 4, "tissue": 3, "apple": 4})  # mock stock over both shelves
        self.assertEqual([zone["zone_id"] for zone in catalog["zones"]], ["zone_a"])

    def test_bad_items_and_unknown_zone_are_refused(self) -> None:
        with self.assertRaises(OrderDeskError) as bad_item:
            self.desk.receive([{"object_class": "바나나", "quantity": 1}], "zone_a")
        self.assertEqual((bad_item.exception.error_code, bad_item.exception.status), ("ORDER_INVALID", 400))
        with self.assertRaises(OrderDeskError) as bad_zone:
            self.desk.receive(CARROT_APPLE, "zone_z")
        self.assertEqual(bad_zone.exception.error_code, "RECEIVING_ZONE_UNKNOWN")

    def test_preview_explains_and_changes_nothing(self) -> None:
        order = self.desk.receive(CARROT_APPLE, "zone_a")
        preview = self.desk.preview(order["id"])
        self.assertEqual(preview["pinky"]["pinky_id"], "10")  # mock batteries: P0 92 %, P1 78 %, P2 64 %
        self.assertEqual(preview["pinky"]["summary"], "P0 · 배터리 90%대 · 오늘 0회")
        self.assertEqual(preview["arms"]["summary"], "오른쪽 팔 2개 → 왼쪽 팔 1개")
        self.assertIsNone(self.store.get_task(order["id"])["pinky_id"])

    def test_start_assigns_the_pinky_and_runs_the_order(self) -> None:
        first = self.desk.receive(CARROT_APPLE, "zone_a")
        second = self.desk.receive([{"object_class": "carrot", "quantity": 1}], "zone_a")
        self.desk.start(first["id"])
        self.desk.start(second["id"])  # P0 is held by the first order now
        done = self.wait(first["id"])
        self.assertEqual((done["status"], done["pinky_id"]), ("SUCCEEDED", "10"))
        assigned = [event for event in done["events"] if event["event_type"] == "PINKY_ASSIGNED"]
        self.assertEqual(assigned[0]["payload"]["assignment"]["pinky_id"], "10")
        self.assertEqual({step["robot_id"] for step in done["steps"] if step["step_type"] == "CONFIRM_DOCKING"},
                         {"pinky-10"})
        self.assertEqual(self.wait(second["id"])["pinky_id"], "11")

    def test_no_free_pinky_keeps_the_order_waiting(self) -> None:
        for pinky_id in ("10", "11", "12"):
            self.store.create_task("A", "B", order_items=CARROT_APPLE, pinky_id=pinky_id)
        order = self.desk.receive(CARROT_APPLE, "zone_a")
        with self.assertRaises(OrderDeskError) as error:
            self.desk.start(order["id"])
        self.assertEqual(error.exception.error_code, "PINKY_UNAVAILABLE")
        self.assertEqual([v["reason"][:3] for v in error.exception.details["assignment"]["verdicts"]], ["주문 "] * 3)
        self.assertEqual(self.store.get_task(order["id"])["status"], "PENDING")

    def test_an_order_cannot_start_without_its_pinky_by_the_old_route(self) -> None:
        order = self.desk.receive(CARROT_APPLE, "zone_a")
        with self.assertRaises(TaskExecutionConflict):
            self.desk.orchestrator.start(order["id"])

    def test_large_order_is_planned_over_two_pinkys_but_not_started(self) -> None:
        # Tray capacity 4 (config/fleet-planning.json): six units go as two trips of three.
        order = self.desk.receive([{"object_class": "carrot", "quantity": 3}, {"object_class": "apple", "quantity": 3}],
                                  "zone_a")
        fleet = self.desk.preview(order["id"])["fleet"]
        self.assertEqual(fleet["summary"], "핑키 2대 · P0 3, P1 3")
        self.assertEqual([trip["pinky_id"] for trip in fleet["trips"]], ["10", "11"])
        for trip in fleet["trips"]:
            schedule = trip["schedule"]
            self.assertEqual(len(schedule["picks"]), 3)
            # ACT runs grasp -> place -> rest in one go, so overlap is off: one whole pick after another.
            self.assertEqual((schedule["overlap"], schedule["total_s"]), (False, schedule["one_arm_at_a_time_s"]))
        with self.assertRaises(OrderDeskError) as error:
            self.desk.start(order["id"])
        self.assertEqual(error.exception.error_code, "MULTI_PINKY_NOT_SUPPORTED")
        self.assertIsNone(self.store.get_task(order["id"])["pinky_id"])

    def test_reject_only_waiting_orders(self) -> None:
        order = self.desk.receive(CARROT_APPLE, "zone_a")
        self.assertEqual(self.desk.reject(order["id"])["status"], "CANCELED")
        with self.assertRaises(OrderDeskError) as error:
            self.desk.reject(order["id"])
        self.assertEqual(error.exception.error_code, "ORDER_NOT_PENDING")


class OrderApiTest(ApiTestCase):
    def setUp(self) -> None:
        super().setUp()
        self.server.order_desk, _, orchestrator = make_desk(self.task_store)
        self.addCleanup(orchestrator.close)

    def test_order_window_round_trip(self) -> None:
        status, catalog, _ = self.request("GET", "/api/catalog")
        self.assertEqual((status, catalog["zones"][0]["label"]), (200, "수령 구역 A"))
        status, created, _ = self.request("POST", "/api/orders", {"items": CARROT_APPLE, "receiving_zone": "zone_a"})
        self.assertEqual(status, 201)
        order_id = created["order"]["id"]
        status, pending, _ = self.request("GET", "/api/orders")
        self.assertEqual((status, [order["id"] for order in pending["orders"]]), (200, [order_id]))
        status, preview, _ = self.request("GET", f"/api/orders/{order_id}/assignment")
        self.assertEqual((status, preview["pinky"]["pinky_id"]), (200, "10"))
        status, started, _ = self.request("POST", f"/api/orders/{order_id}/start")
        self.assertEqual((status, started["order"]["pinky_id"]), (202, "10"))

    def test_errors_carry_codes(self) -> None:
        status, body, _ = self.request("POST", "/api/orders", {"items": CARROT_APPLE, "note": "x"})
        self.assertEqual((status, body["error_code"]), (400, "ORDER_INVALID"))
        status, body, _ = self.request("GET", "/api/orders/TASK-NOPE/assignment")
        self.assertEqual((status, body["error_code"]), (404, "ORDER_NOT_FOUND"))
        status, _, _ = self.request("POST", "/api/orders", {"items": CARROT_APPLE}, origin="http://evil.example")
        self.assertEqual(status, 403)


if __name__ == "__main__":
    unittest.main()
