"""The stop-everything request (stop_all.py, POST /api/stop-all): arms and Pinkys in one request."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from control_station.api.routes import POST_ROUTES
from control_station.api import stop as stop_api
from control_station.arm_commands import ArmCommandStore
from control_station.shelf_slot_map import load_shelf_slot_map
from control_station.stop_all import stop_everything
from control_station.store import TaskStore


def command_request(robot_id: str = "arm-01") -> dict[str, object]:
    return {
        "robot_id": robot_id,
        "operation": "MOVE_TO_SLOT",
        "valid_for_s": 60,
        "target": {"object_class": "strawberry", "pick_slot_id": "slot_a_2_1"},
        "place": {"slot_id": "CART_1"},
        "source": {"kind": "OPERATOR_INPUT"},
    }


class FakeGateway:
    def __init__(self, accepted: dict[str, bool] | None = None, error: Exception | None = None) -> None:
        self.accepted = accepted or {"10": True, "11": True, "12": True}
        self.error = error
        self.calls = 0

    def stop_all(self) -> list[dict[str, object]]:
        self.calls += 1
        if self.error is not None:
            raise self.error
        return [{"pinky_id": pinky_id, "action": "stop", "ok": ok, "simulated": False} for pinky_id, ok in self.accepted.items()]


class FakeSequences:
    def __init__(self, latest: dict[str, object] | None, error: Exception | None = None) -> None:
        self._latest, self.error, self.canceled = latest, error, []

    def latest(self) -> dict[str, object] | None:
        if self.error is not None:
            raise self.error
        return self._latest

    def cancel(self, run_id: str) -> dict[str, object]:
        self.canceled.append(run_id)
        return {"sequence_run_id": run_id, "state": "CANCELING"}


class FakePolicy:
    def __init__(self, alive: bool) -> None:
        self.alive, self.stops = alive, 0

    def snapshot(self) -> dict[str, object]:
        return {"state": "RUNNING" if self.alive else "IDLE", "process_alive": self.alive}

    def stop(self) -> dict[str, object]:
        self.stops += 1
        return {"state": "STOPPING", "process_alive": True}


class CancelActiveTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.task_store = TaskStore(str(Path(directory.name) / "tasks.sqlite3"))
        self.addCleanup(self.task_store.close)
        self.store = ArmCommandStore(self.task_store.session_factory, slot_map=load_shelf_slot_map(), ack_timeout_s=10)

    def test_queued_ends_at_once_and_in_flight_gets_the_cancel_flag(self) -> None:
        self.store.create(command_request("arm-01"))
        in_flight = self.store.claim_next("arm-01", "w1", 0)
        queued = self.store.create(command_request("arm-02"))

        canceled = {command["command_id"]: command for command in self.store.cancel_active()}

        self.assertEqual(set(canceled), {in_flight["command_id"], queued["command_id"]})
        self.assertEqual(canceled[queued["command_id"]]["state"], "CANCELED")
        self.assertEqual(canceled[in_flight["command_id"]]["state"], "DISPATCHED")  # the worker ends it
        self.assertTrue(canceled[in_flight["command_id"]]["cancel_requested"])

    def test_finished_commands_are_left_alone_and_a_second_request_changes_nothing(self) -> None:
        finished = self.store.create(command_request("arm-01"))
        self.store.cancel(finished["command_id"])
        self.assertEqual(self.store.cancel_active(), [])

        self.store.create(command_request("arm-01"))
        claimed = self.store.claim_next("arm-01", "w1", 0)
        first = self.store.cancel_active()
        second = self.store.cancel_active()
        self.assertEqual([command["command_id"] for command in second], [claimed["command_id"]])
        self.assertEqual(first[0]["cancel_requested_at"], second[0]["cancel_requested_at"])


class StopEverythingTests(unittest.TestCase):
    def app(self, **parts: object) -> SimpleNamespace:
        return SimpleNamespace(**parts)

    def test_asks_the_sequence_the_commands_the_policy_run_and_every_pinky(self) -> None:
        commands = SimpleNamespace(cancel_active=lambda: [{"command_id": "c1", "robot_id": "arm-01", "state": "RUNNING",
                                                           "cancel_requested": True}])
        sequences = FakeSequences({"sequence_run_id": "run-1", "state": "RUNNING"})
        policy, gateway = FakePolicy(alive=True), FakeGateway()

        result = stop_everything(self.app(arm_sequences=sequences, arm_commands=commands,
                                          policy_run_supervisor=policy, pinky_gateway=gateway))

        self.assertTrue(result["ok"])
        self.assertEqual(sequences.canceled, ["run-1"])
        self.assertEqual(result["arm_sequence"]["state"], "CANCELING")
        self.assertEqual([command["command_id"] for command in result["arm_commands"]], ["c1"])
        self.assertEqual((policy.stops, result["policy_run"]["state"]), (1, "STOPPING"))
        self.assertEqual([entry["pinky_id"] for entry in result["pinkies"]], ["10", "11", "12"])
        self.assertEqual(result["errors"], [])

    def test_nothing_running_on_the_arms_still_stops_the_pinkies(self) -> None:
        sequences, policy, gateway = FakeSequences({"sequence_run_id": "run-0", "state": "SUCCEEDED"}), FakePolicy(False), FakeGateway()

        result = stop_everything(self.app(arm_sequences=sequences, arm_commands=SimpleNamespace(cancel_active=lambda: []),
                                          policy_run_supervisor=policy, pinky_gateway=gateway))

        self.assertTrue(result["ok"])
        self.assertEqual((sequences.canceled, policy.stops, gateway.calls), ([], 0, 1))
        self.assertIsNone(result["arm_sequence"])
        self.assertIsNone(result["policy_run"])

    def test_a_failing_arm_part_does_not_keep_the_pinkies_from_being_asked(self) -> None:
        gateway = FakeGateway()

        def broken() -> list[dict[str, object]]:
            raise RuntimeError("database is locked")

        result = stop_everything(self.app(arm_sequences=FakeSequences(None, error=RuntimeError("no table")),
                                          arm_commands=SimpleNamespace(cancel_active=broken), pinky_gateway=gateway))

        self.assertEqual(gateway.calls, 1)
        self.assertEqual(len(result["pinkies"]), 3)
        self.assertFalse(result["ok"])  # the arms were not confirmed asked
        self.assertEqual([error["part"] for error in result["errors"]], ["arm_sequence", "arm_commands"])
        self.assertEqual(result["arm_commands"], [])

    def test_a_refused_pinky_or_a_gateway_failure_is_not_ok(self) -> None:
        refused = stop_everything(self.app(pinky_gateway=FakeGateway({"10": True, "11": False, "12": True})))
        self.assertFalse(refused["ok"])
        self.assertEqual([entry["ok"] for entry in refused["pinkies"]], [True, False, True])

        failed = stop_everything(self.app(pinky_gateway=FakeGateway(error=OSError("connection refused"))))
        self.assertFalse(failed["ok"])
        self.assertEqual(failed["errors"], [{"part": "pinkies", "message": "connection refused"}])

    def test_the_route_answers_202_when_all_accepted_and_207_otherwise(self) -> None:
        route = next(route for route in POST_ROUTES if route.matches("/api/stop-all", ["api", "stop-all"]))
        self.assertIs(route.handle, stop_api.post_stop_all)
        self.assertFalse(route.reads_body)
        self.assertIsNotNone(route.cross_origin_message)

        for accepted, expected in (({"10": True, "11": True, "12": True}, 202), ({"10": True, "11": False, "12": True}, 207)):
            sent: list[tuple[int, dict[str, object]]] = []
            handler = SimpleNamespace(app=self.app(pinky_gateway=FakeGateway(accepted)),
                                      _json=lambda status, body: sent.append((status, body)))
            stop_api.post_stop_all(handler, "/api/stop-all", ["api", "stop-all"], None)
            self.assertEqual(sent[0][0], expected)
            self.assertEqual(len(sent[0][1]["pinkies"]), 3)


if __name__ == "__main__":
    unittest.main()
