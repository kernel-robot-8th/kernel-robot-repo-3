"""Order flow against the mock world: stock, arm plan by shelf, Pinky arrival, per-unit transfer, total, signal."""

from __future__ import annotations

import json
import os
import tempfile
import time
import unittest
from collections import Counter
from pathlib import Path
from unittest import mock

from control_station.act_policies import load_act_policies
from control_station.arms import load_arms
from control_station.flow_view import build_flow_view, summarize_order
from control_station.mock_world import MockWorld, parse_faults, parse_stock
from control_station.order_flow import OrderDevices, OrderFlow
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator
from test_api_routes import ApiTestCase

TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED"}
CARROT_APPLE = [{"object_class": "carrot", "quantity": 1}, {"object_class": "apple", "quantity": 2}]


def devices(world: MockWorld) -> OrderDevices:
    return OrderDevices(world.shelf_stock, world.pinky_load, world.fleet_status, world.fleet_confirm, world.parking,
                        world.pick, world.dispatch)


class OrderFlowTests(unittest.TestCase):
    def setUp(self) -> None:
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.store = TaskStore(str(self.directory / "t.sqlite3"))
        self.addCleanup(self.store.close)

    def start(self, faults: str = "", *, stock: str | None = None, policies=None, arrival_timeout_s: float = 10.0,
              drive_s: float = 0.2) -> None:
        self.world = MockWorld(parse_stock(stock) if stock else None, drive_s=drive_s, pick_s=0.05,
                               faults=parse_faults(faults))
        self.flow = OrderFlow(load_arms(), policies or load_act_policies(), devices(self.world), dwell_s=0.0,
                              poll_s=0.02, arrival_timeout_s=arrival_timeout_s, signal_timeout_s=2.0)
        self.orchestrator = MockTaskOrchestrator(self.store, order_flow=self.flow)
        self.addCleanup(self.orchestrator.close)

    def order(self, items=None, pinky_id: str | None = None) -> str:
        task = self.store.create_task("핑키 적재 위치", "주문", order_items=items or CARROT_APPLE, pinky_id=pinky_id)
        self.orchestrator.start(task["id"])
        return task["id"]

    def wait(self, task_id: str) -> dict:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            task = self.store.get_task(task_id)
            if task["status"] in TERMINAL:
                return task
            time.sleep(0.02)
        self.fail(f"order did not finish: {self.store.get_task(task_id)['status']}")

    @staticmethod
    def events(task: dict, event_type: str) -> list[dict]:
        return [event for event in task["events"] if event["event_type"] == event_type]

    @staticmethod
    def step(task: dict, step_type: str, index: int = 0) -> dict:
        return [step for step in task["steps"] if step["step_type"] == step_type][index]

    def test_order_loads_every_unit_and_signals_the_pinky(self) -> None:
        self.start()
        task = self.wait(self.order())
        self.assertEqual(task["status"], "SUCCEEDED", task["error"])
        plan = self.step(task, "PLAN_ARM_SEQUENCE")["result"]["sequence"]
        # Default mock stock: left shelf carrot 3 / apple 1, right shelf apple 3 / carrot 1. The carrot goes where
        # more carrots are left (left arm); the first apple goes to the arm with fewer units (right), the second to
        # the shelf with more apples left (right again), which keeps the left shelf's only apple. The busier arm
        # goes first and finishes before the other starts.
        self.assertEqual([(unit["arm_id"], unit["object_class"]) for unit in plan],
                         [("arm-02", "apple"), ("arm-02", "apple"), ("arm-01", "carrot")])
        self.assertEqual([unit["reason"] for unit in plan], ["맡은 개수 0 vs 1", "남은 재고 2 vs 1", "남은 재고 3 vs 1"])
        picks = [step for step in task["steps"] if step["step_type"] == "PICK_ITEM"]
        self.assertEqual([step["robot_id"] for step in picks], ["arm-02", "arm-02", "arm-01"])
        self.assertEqual(Counter(self.world.loads["10"]), Counter({"carrot": 1, "apple": 2}))
        self.assertEqual([event["payload"]["action"] for event in self.events(task, "PINKY_SIGNAL_SENT")],
                         ["dock_complete", "received"])
        self.assertEqual(self.world.fleet_status("10")["phase"], "driving")
        # The arms move only after the Pinky was judged ARRIVED.
        kinds = [event["event_type"] for event in task["events"]]
        self.assertLess(kinds.index("PARKING_OBSERVED"), kinds.index("ARM_PICK_STARTED"))
        self.assertIsNone(self.flow.station_owner)

    def test_short_stock_stops_before_any_arm_moves(self) -> None:
        self.start("STOCK_SHORT:apple")
        task = self.wait(self.order())
        self.assertEqual((task["status"], task["error"]["code"]), ("FAILED", "SHELF_STOCK_SHORT"))
        self.assertIn("사과 주문 2 / 선반 0", task["error"]["message"])
        self.assertEqual(self.events(task, "ARM_PICK_STARTED"), [])
        self.assertEqual(self.step(task, "NOTIFY_PINKY_RECEIVED")["result"]["outcome"], "NOT_EXECUTED_AFTER_FAILURE")

    def test_pick_failure_and_loss_stop_the_order_without_a_signal(self) -> None:
        for fault, code in (("PICK_FAILED:carrot", "PICK_FAILED_STILL_ON_SHELF"), ("ITEM_LOST:carrot", "ITEM_LOST")):
            with self.subTest(fault=fault):
                self.start(fault)
                task = self.wait(self.order([{"object_class": "carrot", "quantity": 2}]))
                self.assertEqual((task["status"], task["error"]["code"]), ("FAILED", code))
                transfer = self.events(task, "TRANSFER_OBSERVED")[-1]["payload"]
                self.assertEqual(transfer["verdict"], code)
                self.assertEqual(self.step(task, "PICK_ITEM", 1)["status"], "CANCELED")  # no automatic retry
                self.assertEqual([event["payload"]["action"] for event in self.events(task, "PINKY_SIGNAL_SENT")],
                                 ["dock_complete"])
                pinky = task["pinky_id"]
                self.assertEqual(self.world.fleet_status(pinky)["phase"], "docked_manual")  # the Pinky keeps waiting
                self.assertEqual(self.flow.station_owner, task["id"])  # held until the operator acknowledges
                self.store.acknowledge_alert(task["id"], "확인")
                self.flow.free_spot(task["id"])

    def test_unexpected_object_on_the_pinky_fails_the_total(self) -> None:
        self.start("EXTRA_ON_PINKY:ball")
        task = self.wait(self.order([{"object_class": "carrot", "quantity": 1}]))
        self.assertEqual((task["status"], task["error"]["code"]), ("FAILED", "LOAD_EXTRA_ITEM"))

    def test_misaligned_pinky_raises_an_alert_and_the_arms_wait(self) -> None:
        self.start("PINKY_MISALIGNED:3")
        task = self.wait(self.order([{"object_class": "carrot", "quantity": 1}]))
        self.assertEqual(task["status"], "SUCCEEDED", task["error"])
        abnormal = self.events(task, "PINKY_POSITION_ABNORMAL")
        self.assertEqual(len(abnormal), 1)  # one alert while the same offset repeats
        self.assertIn("위치 이상", abnormal[0]["payload"]["message"])
        kinds = [event["event_type"] for event in task["events"]]
        self.assertLess(kinds.index("PINKY_POSITION_ABNORMAL"), kinds.index("ARM_PICK_STARTED"))

    def test_parking_errors_and_offsets_keep_waiting_then_continue_on_arrival(self) -> None:
        self.start()
        answers = iter([
            RuntimeError("주차 판정 노드에 연결할 수 없습니다"),
            {"status": "MISALIGNED", "offset": {"distance_mm": 41.2, "dyaw_deg": 3.0}},
            {"status": "MISALIGNED", "offset": {"distance_mm": 43.9, "dyaw_deg": 3.1}},  # same 1 cm band: no new alert
            {"status": "ARRIVED", "offset": {"distance_mm": 3.0, "dyaw_deg": 0.2}},
        ])

        def parking(pinky_id: str) -> dict:
            answer = next(answers)
            if isinstance(answer, Exception):
                raise answer
            return answer

        self.flow.devices.parking = parking
        task = self.wait(self.order([{"object_class": "carrot", "quantity": 1}]))
        self.assertEqual(task["status"], "SUCCEEDED", task["error"])
        alerts = [event["payload"]["message"] for event in self.events(task, "PINKY_POSITION_ABNORMAL")]
        self.assertEqual(len(alerts), 2)
        self.assertIn("PARKING_CHECK_FAILED", alerts[0])
        self.assertIn("오차 41 mm", alerts[1])
        self.assertIn("자동으로 다음 단계", alerts[1])
        self.assertEqual(self.step(task, "WAIT_PINKY_ARRIVAL")["result"]["parking"]["status"], "ARRIVED")

    def test_arrival_timeout_stops_the_order(self) -> None:
        self.start(arrival_timeout_s=0.3, drive_s=30)
        task = self.wait(self.order([{"object_class": "carrot", "quantity": 1}]))
        self.assertEqual((task["status"], task["error"]["code"]), ("FAILED", "PINKY_ARRIVAL_TIMEOUT"))

    def test_rejected_received_signal_fails_after_the_load_was_confirmed(self) -> None:
        self.start("FLEET_REJECT:received")
        task = self.wait(self.order([{"object_class": "carrot", "quantity": 1}]))
        self.assertEqual((task["status"], task["error"]["code"]), ("FAILED", "PINKY_SIGNAL_REJECTED"))
        self.assertEqual(self.step(task, "VERIFY_LOAD_TOTAL")["status"], "SUCCEEDED")

    def test_two_orders_share_the_loading_spot_one_after_the_other(self) -> None:
        self.start()
        first = self.order([{"object_class": "carrot", "quantity": 1}])
        second = self.order([{"object_class": "tissue", "quantity": 1}])
        first_task, second_task = self.wait(first), self.wait(second)
        self.assertEqual((first_task["status"], second_task["status"]), ("SUCCEEDED", "SUCCEEDED"))
        self.assertEqual((first_task["pinky_id"], second_task["pinky_id"]), ("10", "11"))
        # Whichever Pinky stopped at the station first loads first; the other waits for the received signal.
        first_loaded, then = sorted((first_task, second_task),
                                    key=lambda task: self.events(task, "ARM_PICK_STARTED")[0]["received_at"])
        received = self.events(first_loaded, "PINKY_SIGNAL_SENT")[-1]["received_at"]
        self.assertLess(received, self.events(then, "ARM_PICK_STARTED")[0]["received_at"])

    def test_a_stopped_order_keeps_the_spot_until_acknowledged(self) -> None:
        self.start("ITEM_LOST:carrot")
        stopped = self.wait(self.order([{"object_class": "carrot", "quantity": 1}]))
        self.assertEqual(stopped["status"], "FAILED")
        self.assertEqual(self.flow.station_owner, stopped["id"])  # its Pinky still stands at the station
        waiting = self.order([{"object_class": "tissue", "quantity": 1}])
        deadline = time.monotonic() + 5
        while not self.events(self.store.get_task(waiting), "STATION_BUSY") and time.monotonic() < deadline:
            time.sleep(0.02)
        busy = self.events(self.store.get_task(waiting), "STATION_BUSY")[0]["payload"]
        self.assertTrue(busy["alert"])
        self.assertIn("멈춘 주문", busy["message"])
        self.assertEqual(summarize_order(self.store.get_task(waiting))["alert"]["title"], "적재 자리 막힘")
        self.assertTrue(self.flow.free_spot(stopped["id"]))  # the operator acknowledged the alert
        self.assertEqual(self.wait(waiting)["status"], "SUCCEEDED")

    def test_object_only_on_the_side_whose_arm_has_no_policy(self) -> None:
        policies_path = self.directory / "policies.json"
        policies_path.write_text(json.dumps({"schema_version": 1, "policies": [
            {"policy_id": "carrot_right", "object_class": "carrot", "label": "당근", "aliases": [], "model": "a/b",
             "policy_type": "act", "task": "x", "cameras": {"front": "OVERHEAD", "wrist": "GRIPPER"},
             "max_run_s": 30, "arm_id": "arm-02"},
        ]}))
        self.start(stock="shelf_a:carrot=2;shelf_b:apple=1", policies=load_act_policies(policies_path))
        task = self.wait(self.order([{"object_class": "carrot", "quantity": 1}]))
        self.assertEqual((task["status"], task["error"]["code"]), ("FAILED", "POLICY_NOT_FOR_ARM"))

    def test_screen_summary_shows_stages_alerts_and_robots(self) -> None:
        self.start("ITEM_LOST:apple")
        task = self.wait(self.order())
        summary = summarize_order(task)
        states = {stage["key"]: stage["state"] for stage in summary["stages"]}
        self.assertEqual(states, {"ORDER": "done", "STOCK": "done", "PLAN": "done", "PINKY": "done",
                                  "LOAD": "failed", "TOTAL": "skipped", "SIGNAL": "skipped"})
        self.assertEqual((summary["loaded"], summary["total"]), (0, 3))  # the first pick (an apple) was lost
        self.assertEqual((summary["alert"]["level"], summary["alert"]["title"]), ("error", "물건 분실"))
        self.store.acknowledge_alert(task["id"], "확인")
        self.assertIsNone(summarize_order(self.store.get_task(task["id"]))["alert"])
        view = build_flow_view(self.store.recent_orders(), {"10": self.world.fleet_status("10")}, load_arms(), None)
        self.assertEqual([pinky["name"] for pinky in view["pinkies"]], ["P0", "P1", "P2"])
        self.assertEqual(view["pinkies"][0]["phase"], "docked_manual")
        self.assertFalse(view["pinkies"][1]["reachable"])
        self.assertEqual([arm["label"] for arm in view["arms"]], ["왼쪽 팔", "오른쪽 팔"])


class FlowRouteTests(ApiTestCase):
    def setUp(self) -> None:
        super().setUp()
        world = MockWorld(drive_s=0.2, pick_s=0.05)
        self.server.mock_world = world
        self.server.order_devices = devices(world)
        self.server.arms = load_arms()
        self.server.arm_commands.act_policies = load_act_policies()
        self.server.order_flow = OrderFlow(self.server.arms, load_act_policies(), self.server.order_devices, dwell_s=0.0, poll_s=0.02)
        self.server.task_orchestrator.order_flow = self.server.order_flow

    def test_flow_summary_mock_controls_and_alert_ack(self) -> None:
        status, view = self.get("/api/flow")
        self.assertEqual(status, 200)
        self.assertEqual(len(view["pinkies"]), 3)  # type: ignore[index]
        self.assertIn("ITEM_LOST", view["mock_fault_kinds"])  # type: ignore[index]
        self.assertEqual(self.post("/api/mock-world", {"action": "inject", "fault": "STOCK_SHORT", "argument": "carrot"})[0], 200)
        self.assertEqual(self.post("/api/mock-world", {"action": "explode"})[0], 400)
        status, body = self.post("/api/tasks", {"order_items": [{"object_class": "carrot", "quantity": 1}]})
        task_id = body["task"]["id"]  # type: ignore[index]
        self.assertEqual(self.post(f"/api/tasks/{task_id}/start", {})[0], 202)
        deadline = time.monotonic() + 10
        while self.task_store.get_task(task_id)["status"] not in TERMINAL and time.monotonic() < deadline:
            time.sleep(0.05)
        _, view = self.get("/api/flow")
        self.assertEqual(view["alerts"][0]["code"], "SHELF_STOCK_SHORT")  # type: ignore[index]
        self.assertEqual(self.post(f"/api/flow/alerts/{task_id}/ack", {})[0], 200)
        self.assertEqual(self.get("/api/flow")[1]["alerts"], [])  # type: ignore[index]
        self.assertEqual(self.post("/api/flow/alerts/TASK-00000000/ack", {})[0], 404)
        self.assertEqual(self.post("/api/mock-world", {"action": "restock"})[0], 200)


if __name__ == "__main__":
    unittest.main()
