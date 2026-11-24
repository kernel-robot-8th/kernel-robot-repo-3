"""Automatic dispatch: waiting orders start oldest first, each as soon as the Pinky before it was sent off."""

from __future__ import annotations

import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from control_station.arms.policies import load_act_policies
from control_station.arms.config import load_arms
from control_station.orders.dispatcher import BEFORE_DEPARTURE, OrderDispatcher, own_problem, step_before_spot, step_in
from control_station.mock_world import MockWorld
from control_station.orders.desk import OrderDesk
from control_station.orders.flow import OrderFlow
from control_station.orders.receiving_zones import ReceivingZone
from control_station.storage.store import TaskStore
from control_station.orders.task_runner import MockTaskOrchestrator
from test_api_routes import ApiTestCase
from test_order_flow import devices

ZONES = {"zone_a": ReceivingZone("zone_a", "수령 구역 A", ("5", "수령 1")),
         "zone_b": ReceivingZone("zone_b", "수령 구역 B", ("5", "수령 2"))}
APPLE = [{"object_class": "사과", "quantity": 1}]
CARROT = [{"object_class": "당근", "quantity": 1}]


class DispatcherTest(unittest.TestCase):
    def setUp(self) -> None:
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.store = TaskStore(str(Path(directory.name) / "t.sqlite3"))
        self.addCleanup(self.store.close)
        # A long drive: the first Pinky is on its way while the test looks at the second order.
        self.world = MockWorld(drive_s=1.0, pick_s=0.05)
        self.flow = OrderFlow(load_arms(), load_act_policies(), devices(self.world), dwell_s=0.0, poll_s=0.02,
                              arrival_timeout_s=10.0, signal_timeout_s=2.0,
                              route_for=lambda zone_id: list(ZONES[zone_id].route_labels) if zone_id in ZONES else None)
        orchestrator = MockTaskOrchestrator(self.store, order_flow=self.flow)
        self.addCleanup(orchestrator.close)
        self.desk = OrderDesk(self.store, orchestrator, devices(self.world), load_arms(), load_act_policies(), ZONES,
                              pick_stats=lambda: self.store.pick_success_stats(simulated=True),
                              pickup=self.flow.confirm_pickup)
        self.flow.choose_shipment_pinky = self.desk.assign_next_shipment
        self.dispatcher = OrderDispatcher(self.desk, self.store, enabled=True)

    def status(self, order_id: str) -> str:
        return self.store.get_task(order_id)["status"]

    def wait_until(self, condition, message: str, timeout_s: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(0.02)
        self.fail(message)

    def at_spot(self, order_id: str) -> bool:
        """The order's Pinky was confirmed at the loading spot (or the order went on past it)."""
        order = self.store.get_task(order_id)
        return order["status"] != "PENDING" and step_before_spot(order) is None

    def sent_off(self, order_id: str) -> bool:
        """The order's Pinky was dispatched (or the order ended before that)."""
        order = self.store.get_task(order_id)
        return order["status"] != "PENDING" and step_in(order, BEFORE_DEPARTURE) is None

    def dispatched(self, order_id: str) -> str:
        """When the order's Pinky was sent off (when its PINKY_DISPATCHED event was received)."""
        return next(event["received_at"] for event in self.store.get_task(order_id)["events"]
                    if event["event_type"] == "PINKY_DISPATCHED")

    def test_off_starts_nothing(self) -> None:
        order = self.desk.receive(APPLE)
        state = OrderDispatcher(self.desk, self.store).tick()
        self.assertEqual((state["auto"], self.status(order["id"])), (False, "PENDING"))

    def test_orders_leave_one_after_another_in_the_order_they_came(self) -> None:
        first, second = self.desk.receive(APPLE), self.desk.receive(CARROT)
        state = self.dispatcher.tick()
        # The oldest order starts; the next one waits only until that Pinky has been sent off, not for its arrival.
        self.assertNotEqual(self.status(first["id"]), "PENDING")
        self.assertEqual(self.status(second["id"]), "PENDING")
        self.assertEqual((state["blocked"]["code"], state["blocked"]["order_id"], state["blocked"]["waiting_for"]),
                         ("DEPARTURE_ORDER", second["id"], first["id"]))
        self.wait_until(lambda: self.sent_off(first["id"]), "the first Pinky was not sent off")
        state = self.dispatcher.tick()
        started = self.store.get_task(second["id"])
        self.assertNotEqual(started["status"], "PENDING")
        self.assertIsNotNone(step_before_spot(self.store.get_task(first["id"])))  # the first is still on its way
        self.assertNotEqual(started["pinky_id"], self.store.get_task(first["id"])["pinky_id"])
        self.assertEqual({self.store.get_task(order["id"])["receiving_zone"] for order in (first, second)},
                         {"zone_a", "zone_b"})
        self.assertEqual([entry["order_id"] for entry in state["started"]], [second["id"], first["id"]])
        self.wait_until(lambda: self.sent_off(second["id"]), "the second Pinky was not sent off")
        self.assertLess(self.dispatched(first["id"]), self.dispatched(second["id"]))

    def test_with_wait_for_spot_the_next_order_waits_for_the_arrival(self) -> None:
        dispatcher = OrderDispatcher(self.desk, self.store, enabled=True, wait_for_spot=True)
        first, second = self.desk.receive(APPLE), self.desk.receive(CARROT)
        dispatcher.tick()
        self.wait_until(lambda: self.sent_off(first["id"]), "the first Pinky was not sent off")
        state = dispatcher.tick()
        self.assertEqual(self.status(second["id"]), "PENDING")
        self.assertEqual((state["blocked"]["code"], state["blocked"]["waiting_for"]), ("SPOT_ORDER", first["id"]))
        self.wait_until(lambda: self.at_spot(first["id"]), "the first Pinky did not reach the loading spot")
        dispatcher.tick()
        self.assertNotEqual(self.status(second["id"]), "PENDING")

    def test_the_line_waits_when_every_zone_is_in_use(self) -> None:
        orders = [self.desk.receive(APPLE), self.desk.receive(CARROT), self.desk.receive(APPLE)]
        self.dispatcher.tick()
        self.wait_until(lambda: self.sent_off(orders[0]["id"]), "the first Pinky was not sent off")
        self.dispatcher.tick()
        self.wait_until(lambda: self.sent_off(orders[1]["id"]), "the second Pinky was not sent off")
        state = self.dispatcher.tick()
        # Two zones, two orders on their way to them: the third keeps waiting and no Pinky is taken for it.
        self.assertEqual((state["blocked"]["code"], state["blocked"]["order_id"]), ("ZONE_BUSY", orders[2]["id"]))
        third = self.store.get_task(orders[2]["id"])
        self.assertEqual((third["status"], third["pinky_id"], state["waiting"]), ("PENDING", None, 1))

    def test_an_order_that_cannot_be_picked_is_passed_over(self) -> None:
        short, fine = self.desk.receive(APPLE), self.desk.receive(CARROT)
        for counts in self.world.shelves.values():  # the apples went off the shelves after the order came in
            counts.pop("apple", None)
        state = self.dispatcher.tick()
        self.assertEqual(self.status(short["id"]), "PENDING")
        self.assertNotEqual(self.status(fine["id"]), "PENDING")
        self.assertEqual([(entry["order_id"], entry["code"]) for entry in state["skipped"]],
                         [(short["id"], "SHELF_STOCK_SHORT")])
        self.assertIsNone(self.store.get_task(short["id"])["pinky_id"])

    def test_a_stopped_order_holds_the_line_until_the_alert_is_acknowledged(self) -> None:
        self.world.inject("PICK_FAILED")
        failed = self.desk.receive(APPLE)
        self.dispatcher.tick()
        self.wait_until(lambda: self.status(failed["id"]) == "FAILED", "the order did not stop")
        waiting = self.desk.receive(CARROT)
        state = self.dispatcher.tick()
        self.assertEqual((state["blocked"]["code"], state["blocked"]["waiting_for"]), ("STOPPED_ORDER", failed["id"]))
        self.assertEqual(self.status(waiting["id"]), "PENDING")
        self.store.acknowledge_alert(failed["id"], "현장 확인")
        self.flow.free_spot(failed["id"])
        self.dispatcher.tick()
        self.assertNotEqual(self.status(waiting["id"]), "PENDING")

    def test_turning_it_off_clears_the_reasons_and_starts_nothing(self) -> None:
        first, second = self.desk.receive(APPLE), self.desk.receive(CARROT)
        self.dispatcher.tick()
        state = self.dispatcher.set_enabled(False)
        self.assertEqual((state["auto"], state["blocked"], state["skipped"]), (False, None, []))
        self.wait_until(lambda: self.sent_off(first["id"]), "the first Pinky was not sent off")
        self.dispatcher.tick()
        self.assertEqual(self.status(second["id"]), "PENDING")

    def test_own_problem_reads_every_shipment(self) -> None:
        ok = {"arms": {"picks": []}}
        self.assertIsNone(own_problem({"fleet": {"trips": [ok, ok]}}))
        self.assertEqual(own_problem({"fleet": {"trips": [ok, {"arms": {"error": {"code": "SHELF_STOCK_SHORT",
                                                                                 "message": "부족"}}}]}})["code"],
                         "SHELF_STOCK_SHORT")
        self.assertEqual(own_problem({"fleet": {"error": {"code": "ITEM_TOO_BIG_FOR_PINKY", "message": "큼"}}})["code"],
                         "ITEM_TOO_BIG_FOR_PINKY")


class DispatchApiTest(ApiTestCase):
    def setUp(self) -> None:
        super().setUp()
        world = MockWorld(drive_s=0.2, pick_s=0.05)
        flow = OrderFlow(load_arms(), load_act_policies(), devices(world), dwell_s=0.0, poll_s=0.02)
        self.server.task_orchestrator.order_flow = flow
        self.server.order_desk = OrderDesk(self.task_store, self.server.task_orchestrator, devices(world), load_arms(),
                                           load_act_policies(), ZONES, pick_stats=dict)
        self.server.order_dispatcher = OrderDispatcher(self.server.order_desk, self.task_store)

    def test_switch_and_view(self) -> None:
        status, body, _ = self.request("GET", "/api/dispatch")
        self.assertEqual((status, body["dispatch"]["auto"]), (200, False))
        status, body, _ = self.request("GET", "/api/orders")
        self.assertEqual((status, body["orders"], body["dispatch"]["auto"]), (200, [], False))
        status, body, _ = self.request("POST", "/api/dispatch", {"auto": True})
        self.assertEqual((status, body["dispatch"]["auto"]), (200, True))
        status, body, _ = self.request("POST", "/api/dispatch", {"auto": "yes"})
        self.assertEqual((status, body["error_code"]), (400, "DISPATCH_INVALID"))
        status, body, _ = self.request("POST", "/api/dispatch", {"auto": False, "more": 1})
        self.assertEqual((status, body["error_code"]), (400, "DISPATCH_INVALID"))


if __name__ == "__main__":
    unittest.main()
