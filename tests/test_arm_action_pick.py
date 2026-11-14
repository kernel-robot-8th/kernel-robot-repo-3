"""Order picks as a ROS 2 action: the goal, and the arm command record kept as if a worker had reported.

The ROS side is replaced by a fake transport, so these run without ROS; the record is the real ArmCommandStore.
"""

from __future__ import annotations

import tempfile
import threading
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path

from control_station.act_policies import load_act_policies
from control_station.arm_action_ros import PickActionUnavailable, action_name
from control_station.arm_commands import ArmCommandStore
from control_station.arm_pick import HOME_REACHED_UNVERIFIED, ArmActionPick
from control_station.order_flow import OrderFlowError
from control_station.shelf_slot_map import load_shelf_slot_map
from control_station.store import TaskStore

MOVING = ["CHECKING", "STARTING_POLICY", "RUNNING_POLICY", "RETURNING_HOME"]
HOME = {"outcome": "HOME_REACHED_PLACE_NOT_VERIFIED", "reason_code": "PLACE_NOT_VERIFIED", "message": "home",
        "pose_reached": True, "simulated": True, "run_s": 3.0}


class FakeGoal:
    """Delivers its feedback on the first poll and its result on the next (or on the same poll with `together`).
    A goal with no result waits until it is canceled, then ends CANCELED."""

    def __init__(self, on_feedback, phases: list[str], result: dict | None, *, accepted: bool = True,
                 together: bool = False, on_poll=None) -> None:
        self.accepted = accepted
        self.on_feedback, self.phases, self.final, self.together, self.on_poll = on_feedback, phases, result, together, on_poll
        self.polls = 0
        self.canceled = False

    def cancel(self) -> None:
        self.canceled = True

    def result(self, timeout_s: float) -> dict | None:
        self.polls += 1
        if self.on_poll:
            self.on_poll(self.polls)
        if self.polls == 1:
            for phase in self.phases:
                self.on_feedback({"phase": phase, "elapsed_s": 0.1, "message": "", "simulated": True})
            if not self.together:
                return None
        if self.final is not None:
            return self.final
        if self.canceled:
            return {**HOME, "outcome": "CANCELED", "reason_code": "", "pose_reached": False}
        return None


class FakeTransport:
    def __init__(self, phases: list[str], result: dict | None, **goal_options) -> None:
        self.phases, self.result, self.goal_options = phases, result, goal_options
        self.targets: list[dict] = []
        self.goal: FakeGoal | None = None
        self.unavailable = False

    def send_goal(self, arm_id: str, target: dict, on_feedback) -> FakeGoal:
        if self.unavailable:
            raise PickActionUnavailable(f"집기 액션 서버에 연결할 수 없습니다: {action_name(arm_id)}")
        self.targets.append({"arm_id": arm_id, **target})
        self.goal = FakeGoal(on_feedback, self.phases, self.result, **self.goal_options)
        return self.goal


class ArmActionPickTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.task_store = TaskStore(str(Path(directory.name) / "tasks.sqlite3"))
        self.addCleanup(self.task_store.close)
        self.policies = load_act_policies()
        self.store = ArmCommandStore(self.task_store.session_factory, slot_map=load_shelf_slot_map(),
                                     act_policies=self.policies, ack_timeout_s=10)

    def pick(self, transport: FakeTransport, *, arm_id: str = "arm-02", object_class: str = "carrot",
             cancel: threading.Event | None = None):
        messages: list[str] = []
        picker = ArmActionPick(self.store, transport, poll_s=0)  # type: ignore[arg-type]
        result = picker(arm_id, object_class, "shelf_b", "10", cancel or threading.Event(), messages.append)
        return result, messages, picker

    def only_command(self) -> dict:
        commands = self.store.list_recent()["commands"]
        self.assertEqual(len(commands), 1)
        return self.store.get(commands[0]["command_id"])

    def test_the_goal_carries_the_policy_and_the_record_follows_the_action(self) -> None:
        transport = FakeTransport(MOVING, HOME)
        result, messages, picker = self.pick(transport)
        command = self.only_command()
        target = transport.targets[0]
        policy = self.policies.resolve("carrot", "arm-02")
        self.assertEqual((target["arm_id"], target["object_class"], target["policy_id"], target["policy_model"]),
                         ("arm-02", "carrot", policy["policy_id"], policy["model"]))
        self.assertEqual(target["policy_registry_version"], self.policies.version)
        # The goal may not start past the record's ACK window, so the arm never moves on an expired record.
        self.assertLessEqual(datetime.fromisoformat(target["valid_until"]),
                             datetime.now(timezone.utc) + timedelta(seconds=10))
        self.assertEqual((result["outcome"], result["arm_command_id"], result["source"], result["simulated"]),
                         (HOME_REACHED_UNVERIFIED, command["command_id"], "arm_action", True))
        self.assertEqual((command["state"], command["reason_code"], command["worker_id"]),
                         ("UNKNOWN", "PLACE_NOT_VERIFIED", "ros2-action.arm-02"))
        self.assertEqual([event["event_type"] for event in command["events"]],
                         ["CREATED", "DISPATCHED", "ACCEPTED", "PROGRESS", "RESULT_UNKNOWN"])
        self.assertIn("arm-02: 정책 실행 중 (모의)", messages)
        # The camera check closes it like a worker's, and the arm takes the next command.
        picker.settle(result, {"task_id": "TASK-1", "source": "camera_yolo", "simulated": False})
        self.assertFalse(self.store.get(command["command_id"])["blocks_new_commands"])

    def test_a_result_that_overtakes_its_feedback_is_still_recorded_in_order(self) -> None:
        result, _, _ = self.pick(FakeTransport(MOVING, HOME, together=True))
        self.assertEqual(result["outcome"], HOME_REACHED_UNVERIFIED)
        self.assertEqual([event["event_type"] for event in self.only_command()["events"]][-3:],
                         ["ACCEPTED", "PROGRESS", "RESULT_UNKNOWN"])

    def test_refused_checks_move_nothing_and_close_the_record(self) -> None:
        refused = {**HOME, "outcome": "REJECTED", "reason_code": "POLICY_WRONG_ARM", "pose_reached": False}
        with self.assertRaises(OrderFlowError) as raised:
            self.pick(FakeTransport(["CHECKING"], refused))
        self.assertEqual(raised.exception.error_code, "POLICY_WRONG_ARM")
        command = self.only_command()
        self.assertEqual((command["state"], command["reason_code"]), ("REJECTED", "POLICY_WRONG_ARM"))
        self.assertFalse(command["blocks_new_commands"])

    def test_no_server_and_a_busy_server_send_nothing_to_the_arm(self) -> None:
        transport = FakeTransport(MOVING, HOME)
        transport.unavailable = True
        with self.assertRaises(OrderFlowError) as raised:
            self.pick(transport)
        self.assertEqual(raised.exception.error_code, "ACTION_SERVER_UNAVAILABLE")
        self.assertEqual(self.only_command()["state"], "REJECTED")

    def test_a_goal_the_server_does_not_take(self) -> None:
        with self.assertRaises(OrderFlowError) as raised:
            self.pick(FakeTransport(MOVING, HOME, accepted=False))
        self.assertEqual(raised.exception.error_code, "ACTION_GOAL_REJECTED")

    def test_cancelling_the_order_cancels_the_goal(self) -> None:
        cancel = threading.Event()
        cancel.set()
        transport = FakeTransport(MOVING, None)
        result, _, _ = self.pick(transport, cancel=cancel)
        self.assertIsNone(result)
        self.assertTrue(transport.goal.canceled)
        self.assertEqual(self.only_command()["state"], "CANCELED")

    def test_cancel_on_the_screen_reaches_the_arm(self) -> None:
        def screen_cancel(poll: int) -> None:
            if poll == 2:
                command = self.store.list_recent()["commands"][0]
                self.store.cancel(command["command_id"])

        transport = FakeTransport(MOVING, None, on_poll=screen_cancel)
        with self.assertRaises(OrderFlowError):  # the order did not ask: it stops with the arm's CANCELED
            self.pick(transport)
        self.assertTrue(transport.goal.canceled)
        self.assertEqual(self.only_command()["state"], "CANCELED")

    def test_a_failure_after_the_start_stops_the_order_with_the_arm_reason(self) -> None:
        failed = {**HOME, "outcome": "FAILED", "reason_code": "POLICY_TIMEOUT_NOT_AT_REST", "pose_reached": False}
        with self.assertRaises(OrderFlowError) as raised:
            self.pick(FakeTransport(MOVING, failed))
        self.assertEqual(raised.exception.error_code, "POLICY_TIMEOUT_NOT_AT_REST")
        self.assertEqual(self.only_command()["state"], "FAILED")

    def test_a_command_an_http_worker_took_is_not_sent_again(self) -> None:
        create = self.store.create

        def create_and_lose_it_to_http(payload: dict) -> dict:  # an HTTP worker of the arm polls in between
            made = create(payload)
            taken = self.store.claim_next("arm-02", "http-worker", 0)
            base = {"robot_id": "arm-02", "worker_id": "http-worker", "simulated": False}
            self.store.apply_worker_event(taken["command_id"], {**base, "event_id": "e1", "event": "ACCEPTED"})
            self.store.apply_worker_event(taken["command_id"], {**base, "event_id": "e2", "event": "RESULT_UNKNOWN",
                                                                 "reason_code": "PLACE_NOT_VERIFIED", "pose_reached": True})
            return made

        self.store.create = create_and_lose_it_to_http  # type: ignore[method-assign]
        transport = FakeTransport(MOVING, HOME)
        result, _, _ = self.pick(transport)
        self.assertEqual(transport.targets, [])  # no second execution through the action
        self.assertEqual(result["outcome"], HOME_REACHED_UNVERIFIED)


if __name__ == "__main__":
    unittest.main()
