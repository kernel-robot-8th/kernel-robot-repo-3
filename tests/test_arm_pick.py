"""Real-arm order picks: PICK_WITH_POLICY through the arm command queue, and the camera check that closes it."""

from __future__ import annotations

import itertools
import os
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from control_station.act_policies import load_act_policies
from control_station.arm_pick import HOME_REACHED_UNVERIFIED, ArmQueuePick
from control_station.arms import load_arms
from control_station.mock_world import MockWorld
from control_station.order_flow import OrderDevices, OrderFlow, OrderFlowError
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator

HOME_UNVERIFIED = {"state": "UNKNOWN", "reason_code": "PLACE_NOT_VERIFIED", "result": {"pose_reached": True}}


class FakeCommands:
    """Arm command queue stand-in: `get` walks through the given states and then repeats the last one."""

    def __init__(self, *states: dict) -> None:
        self.act_policies = load_act_policies()
        self.states = list(states)
        self.created: list[dict] = []
        self.cancelled: list[str] = []
        self.cleared: list[tuple[str, str]] = []

    def create(self, payload: dict) -> dict:
        self.created.append(payload)
        return {"command_id": "CMD-1", "state": "QUEUED"}

    def get(self, command_id: str) -> dict:
        if self.cancelled:
            return {"state": "CANCELED"}
        return self.states.pop(0) if len(self.states) > 1 else self.states[0]

    def cancel(self, command_id: str) -> None:
        self.cancelled.append(command_id)

    def clear_unknown(self, command_id: str, note: str) -> None:
        self.cleared.append((command_id, note))


def pick(commands: FakeCommands, arm_id: str = "arm-02", object_class: str = "carrot",
         cancel: threading.Event | None = None, **options) -> tuple[dict | None, list[str]]:
    messages: list[str] = []
    picker = ArmQueuePick(commands, poll_s=0, **options)  # type: ignore[arg-type]
    result = picker(arm_id, object_class, "shelf_b", "10", cancel or threading.Event(), messages.append)
    return result, messages


class ArmQueuePickTest(unittest.TestCase):
    def test_queues_the_policy_of_the_planned_arm_and_waits_until_home(self) -> None:
        commands = FakeCommands({"state": "QUEUED"}, {"state": "RUNNING"}, HOME_UNVERIFIED)
        result, messages = pick(commands)
        self.assertEqual(commands.created[0]["robot_id"], "arm-02")
        self.assertEqual((commands.created[0]["operation"], commands.created[0]["target"]),
                         ("PICK_WITH_POLICY", {"object_class": "carrot"}))
        self.assertEqual((result["outcome"], result["arm_command_id"], result["arm_id"]),
                         (HOME_REACHED_UNVERIFIED, "CMD-1", "arm-02"))
        self.assertEqual(messages[-1], "arm-02: 팔 명령 UNKNOWN")

    def test_a_policy_recorded_on_the_other_arm_moves_nothing(self) -> None:
        commands = FakeCommands(HOME_UNVERIFIED)
        with self.assertRaises(OrderFlowError) as raised:
            pick(commands, arm_id="arm-02", object_class="apple")  # the apple policy is arm-01's
        self.assertEqual(raised.exception.error_code, "POLICY_NOT_FOR_ARM")
        self.assertEqual(commands.created, [])

    def test_a_failed_command_stops_the_order_with_its_reason(self) -> None:
        commands = FakeCommands({"state": "FAILED", "reason_code": "POLICY_CRASHED", "result": {"message": "x"}})
        with self.assertRaises(OrderFlowError) as raised:
            pick(commands)
        self.assertEqual((raised.exception.error_code, raised.exception.source), ("POLICY_CRASHED", "arm"))

    def test_cancel_stops_the_arm_and_returns_nothing(self) -> None:
        cancel = threading.Event()
        cancel.set()
        commands = FakeCommands({"state": "RUNNING"})
        result, _ = pick(commands, cancel=cancel)
        self.assertEqual((result, commands.cancelled), (None, ["CMD-1"]))

    def test_no_answer_in_time_is_a_timeout(self) -> None:
        commands = FakeCommands({"state": "RUNNING"})
        clock = itertools.count(0, 100).__next__
        with self.assertRaises(OrderFlowError) as raised:
            pick(commands, extra_wait_s=0, clock=clock)
        self.assertEqual(raised.exception.error_code, "ARM_RESULT_TIMEOUT")

    def test_only_camera_evidence_closes_the_unverified_command(self) -> None:
        commands = FakeCommands(HOME_UNVERIFIED)
        picker = ArmQueuePick(commands)  # type: ignore[arg-type]
        done = {"outcome": HOME_REACHED_UNVERIFIED, "arm_command_id": "CMD-1"}
        picker.settle(done, {"task_id": "TASK-1", "source": "mock_world", "simulated": True})
        self.assertEqual(commands.cleared, [])
        picker.settle(done, {"task_id": "TASK-1", "source": "camera_yolo", "simulated": False})
        self.assertEqual(commands.cleared, [("CMD-1", "주문 TASK-1: 카메라로 옮김 확인(camera_yolo)")])


class RealPickOrderTest(unittest.TestCase):
    """The order flow with `pick_is_real`: counts must come from cameras, and each verified transfer settles."""

    def setUp(self) -> None:
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.store = TaskStore(str(Path(directory.name) / "t.sqlite3"))
        self.addCleanup(self.store.close)
        self.world = MockWorld(drive_s=0.2, pick_s=0.05)
        self.picks: list[tuple[str, str]] = []
        self.settled: list[dict] = []

    def run_order(self, *, camera_counts: bool) -> dict:
        world = self.world

        def counted(observe):  # what camera counts look like to the flow: the same numbers, not simulated
            return lambda *args: {**observe(*args), "simulated": False, "source": "camera_yolo"}

        def real_pick(arm_id, object_class, shelf_id, pinky_id, cancel_event, progress):
            self.picks.append((arm_id, object_class))
            world.pick(arm_id, object_class, shelf_id, pinky_id, cancel_event, progress)  # the object moves
            return {"source": "arm_worker", "outcome": HOME_REACHED_UNVERIFIED, "arm_command_id": f"CMD-{len(self.picks)}"}

        shelf, load = (counted(world.shelf_stock), counted(world.pinky_load)) if camera_counts else (
            world.shelf_stock, world.pinky_load)
        devices = OrderDevices(shelf, load, world.fleet_status, world.fleet_confirm, world.parking, real_pick,
                               world.dispatch, world.mission_complete, pick_is_real=True,
                               pick_settle=lambda done, evidence: self.settled.append({**done, **evidence}))
        flow = OrderFlow(load_arms(), load_act_policies(), devices, dwell_s=0.0, poll_s=0.02,
                         arrival_timeout_s=10.0, signal_timeout_s=2.0)
        orchestrator = MockTaskOrchestrator(self.store, order_flow=flow)
        self.addCleanup(orchestrator.close)
        task = self.store.create_task("핑키 적재 위치", "주문", order_items=[{"object_class": "carrot", "quantity": 1},
                                                                        {"object_class": "apple", "quantity": 1}])
        orchestrator.start(task["id"])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            current = self.store.get_task(task["id"])
            if current["status"] in {"SUCCEEDED", "FAILED", "CANCELED"}:
                return current
            time.sleep(0.02)
        self.fail(f"order did not finish: {self.store.get_task(task['id'])['status']}")

    def test_mock_counts_stop_the_order_before_a_real_arm_moves(self) -> None:
        task = self.run_order(camera_counts=False)
        self.assertEqual((task["status"], task["error"]["code"]), ("FAILED", "TRANSFER_EVIDENCE_REQUIRED"))
        self.assertEqual(self.picks, [])

    def test_camera_counted_transfers_settle_each_arm_command(self) -> None:
        task = self.run_order(camera_counts=True)
        self.assertEqual(task["status"], "SUCCEEDED", task["error"])
        self.assertEqual(len(self.picks), 2)
        self.assertIn(("arm-01", "apple"), self.picks)  # the apple policy was recorded on the left arm only
        self.assertEqual([entry["arm_command_id"] for entry in self.settled], ["CMD-1", "CMD-2"])
        self.assertEqual({entry["simulated"] for entry in self.settled}, {False})


if __name__ == "__main__":
    unittest.main()
