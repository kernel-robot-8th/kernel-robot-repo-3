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
TWO_ZONES = {**ZONES, "zone_b": ReceivingZone("zone_b", "수령 구역 B", ("1", "5", "7", "10"))}


def finish(desk: OrderDesk, store: TaskStore, order_id: str, *, pickup: bool = True) -> dict:
    """Run an order to its end, confirming pickup when its Pinky waits at the receiving zone."""
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        order = store.get_task(order_id)
        if order["status"] in TERMINAL:
            return order
        active = next((step for step in order["steps"] if step["id"] == order.get("active_step_id")), None)
        if pickup and active is not None and active["step_type"] == "CONFIRM_PICKUP" and desk.flow_waiting(order_id):
            desk.confirm_pickup(order_id)
        time.sleep(0.02)
    raise AssertionError(f"order did not finish: {store.get_task(order_id)['status']}")


def make_desk(store: TaskStore, *, zones=ZONES) -> tuple[OrderDesk, MockWorld, MockTaskOrchestrator]:
    world = MockWorld(drive_s=0.2, pick_s=0.05)
    flow = OrderFlow(load_arms(), load_act_policies(), devices(world), dwell_s=0.0, poll_s=0.02,
                     arrival_timeout_s=10.0, signal_timeout_s=2.0,
                     route_for=lambda zone_id: list(zones[zone_id].route_labels) if zone_id in zones else None)
    orchestrator = MockTaskOrchestrator(store, order_flow=flow)
    desk = OrderDesk(store, orchestrator, devices(world), load_arms(), load_act_policies(), zones,
                     pick_stats=lambda: store.pick_success_stats(simulated=True), pickup=flow.confirm_pickup)
    desk.flow_waiting = lambda order_id: order_id in flow._pickups  # test helper: is the step waiting yet
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

    def test_received_order_waits_without_a_pinky_or_a_zone(self) -> None:
        order = self.desk.receive([{"object_class": "사과", "quantity": 2}])  # the customer app sends no zone
        self.assertEqual((order["status"], order["pinky_id"], order["receiving_zone"]), ("PENDING", None, None))
        self.assertEqual((order["destination"], order["arm_job"]), ("수령 구역 (시작 때 배정)", "사과 2 적재"))
        self.assertEqual({step["robot_id"] for step in order["steps"] if step["step_type"] == "CONFIRM_DOCKING"}, {None})
        # The zone steps are there from the start; the zone itself comes with the start.
        self.assertIn("CONFIRM_PICKUP", [step["step_type"] for step in order["steps"]])
        track = self.desk.track(order["id"])
        self.assertEqual((track["zone"], track["zone_note"]), (None, "받을 곳은 로봇이 출발할 때 알려 드려요"))
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

    def use_two_zones(self) -> None:
        self.desk, self.world, orchestrator = make_desk(self.store, zones=TWO_ZONES)
        self.addCleanup(orchestrator.close)

    def test_start_assigns_the_zone_and_the_pinky_and_runs_the_order(self) -> None:
        self.use_two_zones()
        first = self.desk.receive(CARROT_APPLE)
        second = self.desk.receive([{"object_class": "carrot", "quantity": 1}])
        self.assertEqual(self.desk.preview(first["id"])["zone"]["summary"], "수령 구역 A · 오늘 0회 · 경로 3곳")
        self.desk.start(first["id"])
        preview = self.desk.preview(second["id"])["zone"]  # zone A is the first order's until its pickup
        self.assertEqual([(v["zone_id"], v["eligible"]) for v in preview["verdicts"]],
                         [("zone_a", False), ("zone_b", True)])
        self.desk.start(second["id"])  # P0 and zone A are held by the first order now
        done = finish(self.desk, self.store, first["id"])
        self.assertEqual((done["status"], done["pinky_id"], done["receiving_zone"], done["destination"]),
                         ("SUCCEEDED", "10", "zone_a", "수령 구역 A"))
        assigned = [event for event in done["events"] if event["event_type"] == "PINKY_ASSIGNED"]
        self.assertEqual(assigned[0]["payload"]["assignment"]["pinky_id"], "10")
        zoned = [event for event in done["events"] if event["event_type"] == "ZONE_ASSIGNED"]
        self.assertEqual(zoned[0]["payload"]["assignment"]["zone_id"], "zone_a")
        self.assertEqual({step["robot_id"] for step in done["steps"] if step["step_type"] == "CONFIRM_DOCKING"},
                         {"pinky-10"})
        second_done = finish(self.desk, self.store, second["id"])
        self.assertEqual((second_done["pinky_id"], second_done["receiving_zone"]), ("11", "zone_b"))
        self.assertEqual(self.desk.track(second["id"])["zone"], "수령 구역 B")
        # Both zones are free again; B was used as often as A today and has the longer route, so A comes first.
        third = self.desk.receive([{"object_class": "apple", "quantity": 1}])
        self.assertEqual(self.desk.preview(third["id"])["zone"]["zone_id"], "zone_a")

    def test_a_zone_in_use_keeps_the_next_order_waiting(self) -> None:
        first = self.desk.receive([{"object_class": "carrot", "quantity": 1}])
        second = self.desk.receive([{"object_class": "apple", "quantity": 1}])
        self.desk.start(first["id"])
        with self.assertRaises(OrderDeskError) as busy:
            self.desk.start(second["id"])
        self.assertEqual((busy.exception.error_code, busy.exception.status), ("ZONE_BUSY", 409))
        self.assertIn("수령 구역이 모두 사용 중이에요", str(busy.exception))
        waiting = self.store.get_task(second["id"])
        self.assertEqual((waiting["status"], waiting["pinky_id"], waiting["receiving_zone"]), ("PENDING", None, None))
        finish(self.desk, self.store, first["id"])  # the pickup frees zone A
        self.desk.start(second["id"])
        self.assertEqual(finish(self.desk, self.store, second["id"])["receiving_zone"], "zone_a")

    def test_an_operators_zone_is_kept(self) -> None:
        self.use_two_zones()
        first = self.desk.receive([{"object_class": "carrot", "quantity": 1}], "zone_b")  # A would rank first
        second = self.desk.receive([{"object_class": "apple", "quantity": 1}], "zone_b")
        self.assertEqual(first["destination"], "수령 구역 B")
        self.assertEqual(self.desk.preview(first["id"])["zone"]["summary"], "수령 구역 B · 작업자 지정")
        self.desk.start(first["id"])
        with self.assertRaises(OrderDeskError) as busy:
            self.desk.start(second["id"])  # zone A is free, but the operator's zone B is not swapped for it
        self.assertEqual(busy.exception.error_code, "ZONE_BUSY")
        self.assertEqual(finish(self.desk, self.store, first["id"])["receiving_zone"], "zone_b")

    def test_the_pinky_goes_to_the_receiving_zone_and_home_after_pickup(self) -> None:
        order = self.desk.receive([{"object_class": "carrot", "quantity": 1}], "zone_a")
        with self.assertRaises(OrderDeskError) as early:
            self.desk.confirm_pickup(order["id"])
        self.assertEqual(early.exception.error_code, "PICKUP_NOT_READY")
        self.desk.start(order["id"])
        deadline = time.monotonic() + 15
        while not self.desk.flow_waiting(order["id"]) and time.monotonic() < deadline:
            time.sleep(0.02)
        track = self.desk.track(order["id"])
        self.assertEqual((track["stage"], track["can_confirm_pickup"], track["title"]),
                         ("arrived", True, "수령 구역 A에 도착했어요"))
        self.assertEqual(self.world.fleet_status("10")["phase"], "course_complete")
        self.desk.confirm_pickup(order["id"])
        done = finish(self.desk, self.store, order["id"], pickup=False)
        self.assertEqual(done["status"], "SUCCEEDED")
        self.assertEqual(self.world.fleet_status("10")["phase"], "parked")
        actions = [e["payload"]["action"] for e in done["events"] if e["event_type"] == "PINKY_SIGNAL_SENT"]
        self.assertEqual(actions, ["dock_complete", "received", "mission-complete"])
        self.assertEqual(self.desk.track(order["id"])["stage"], "done")

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

    def test_the_window_sells_only_what_is_available(self) -> None:
        def inventory(apple_status: str = "available", apples: int | None = 2) -> dict:
            return {"state": "live" if apple_status != "unknown" else "unknown", "reason": "선반 카메라의 새 값이 없습니다",
                    "products": [{"object_class": "apple", "label": "사과", "available": apples, "status": apple_status},
                                 {"object_class": "carrot", "label": "당근", "available": 0, "status": "sold_out"},
                                 {"object_class": "tissue", "label": "물티슈", "available": 0, "status": "closed"}]}
        self.desk.inventory = inventory
        catalog = self.desk.catalog()
        self.assertEqual([p["notice"] for p in catalog["products"]], [None, "품절", "판매 중지"])
        self.assertEqual(self.desk.receive([{"object_class": "apple", "quantity": 2}], "zone_a")["status"], "PENDING")
        with self.assertRaises(OrderDeskError) as short:
            self.desk.receive([{"object_class": "apple", "quantity": 3}, {"object_class": "carrot", "quantity": 1}],
                              "zone_a")
        self.assertEqual((short.exception.error_code, str(short.exception)),
                         ("OUT_OF_STOCK", "사과는 2개 남았습니다 · 당근은 품절입니다"))
        with self.assertRaises(OrderDeskError) as closed:
            self.desk.receive([{"object_class": "tissue", "quantity": 1}], "zone_a")
        self.assertEqual((closed.exception.error_code, str(closed.exception)), ("ITEM_CLOSED", "물티슈는 지금 판매하지 않습니다"))
        self.desk.inventory = lambda: inventory("unknown", None)
        self.assertIn("재고를 확인하지 못해", self.desk.catalog()["notice"])
        with self.assertRaises(OrderDeskError) as unknown:
            self.desk.receive([{"object_class": "apple", "quantity": 1}], "zone_a")
        self.assertEqual((unknown.exception.error_code, str(unknown.exception)),
                         ("ITEM_CLOSED", "사과는 재고를 확인하지 못해 주문을 닫았습니다"))

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
        status, created, _ = self.request("POST", "/api/orders", {"items": CARROT_APPLE})  # no zone from the app
        self.assertEqual((status, created["order"]["receiving_zone"]), (201, None))
        order_id = created["order"]["id"]
        status, pending, _ = self.request("GET", "/api/orders")
        self.assertEqual((status, [order["id"] for order in pending["orders"]]), (200, [order_id]))
        status, preview, _ = self.request("GET", f"/api/orders/{order_id}/assignment")
        self.assertEqual((status, preview["pinky"]["pinky_id"], preview["zone"]["zone_id"]), (200, "10", "zone_a"))
        status, started, _ = self.request("POST", f"/api/orders/{order_id}/start")
        self.assertEqual((status, started["order"]["pinky_id"], started["order"]["receiving_zone"]),
                         (202, "10", "zone_a"))
        status, created, _ = self.request("POST", "/api/orders", {"items": [{"object_class": "apple", "quantity": 1}]})
        status, body, _ = self.request("POST", f"/api/orders/{created['order']['id']}/start")
        self.assertEqual((status, body["error_code"], body["zone"]["zone_id"]), (409, "ZONE_BUSY", None))

    def test_errors_carry_codes(self) -> None:
        status, body, _ = self.request("POST", "/api/orders", {"items": CARROT_APPLE, "note": "x"})
        self.assertEqual((status, body["error_code"]), (400, "ORDER_INVALID"))
        status, body, _ = self.request("GET", "/api/orders/TASK-NOPE/assignment")
        self.assertEqual((status, body["error_code"]), (404, "ORDER_NOT_FOUND"))
        status, _, _ = self.request("POST", "/api/orders", {"items": CARROT_APPLE}, origin="http://evil.example")
        self.assertEqual(status, 403)


if __name__ == "__main__":
    unittest.main()
