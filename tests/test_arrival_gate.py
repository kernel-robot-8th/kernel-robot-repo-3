from __future__ import annotations

import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from control_station.parking_ros import ParkingObservationError
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator

TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED", "NEEDS_INTERVENTION"}


def observation(status: str) -> dict[str, object]:
    offset = None if status in {"NOT_FOUND", "AMBIGUOUS", "VIEW_UNVERIFIED"} else {
        "dx_mm": -60.8, "dy_mm": 12.0, "distance_mm": 62.0, "dyaw_deg": 1.3,
    }
    return {"spot_id": "pinky_dock_1", "status": status, "reason": "test", "offset": offset}


class ArrivalGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.store = TaskStore(str(Path(self.directory.name) / "tasks.sqlite3"))
        self.addCleanup(self.store.close)
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_task(self, arrival_check) -> dict[str, object]:
        orchestrator = MockTaskOrchestrator(self.store, arrival_check=arrival_check)
        self.addCleanup(orchestrator.close)
        task = self.store.create_task("작업대 A", "상자 이동")
        orchestrator.start(task["id"])
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            task = self.store.get_task(task["id"])
            if task["status"] in TERMINAL:
                return task
            time.sleep(0.02)
        self.fail(f"task did not finish: {task['status']}")

    @staticmethod
    def step(task: dict[str, object], step_type: str) -> dict[str, object]:
        return next(step for step in task["steps"] if step["step_type"] == step_type)

    @staticmethod
    def parking_events(task: dict[str, object]) -> list[dict[str, object]]:
        return [event for event in task["events"] if event["event_type"] == "PARKING_OBSERVED"]

    def test_arrived_records_observation_and_runs_arm_step(self) -> None:
        task = self.run_task(lambda: observation("ARRIVED"))
        self.assertEqual(task["status"], "SUCCEEDED")
        verify = self.step(task, "VERIFY_ARRIVAL")
        self.assertEqual(verify["result"]["outcome"], "PARKING_ARRIVED")
        events = self.parking_events(task)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["step_id"], verify["id"])
        self.assertEqual(events[0]["payload"]["observation"]["status"], "ARRIVED")
        self.assertFalse(events[0]["payload"]["arm_command_sent"])
        self.assertEqual(self.step(task, "EXECUTE_ARM_TASK")["status"], "SUCCEEDED")

    def test_misaligned_fails_before_arm_step(self) -> None:
        task = self.run_task(lambda: observation("MISALIGNED"))
        self.assertEqual(task["status"], "FAILED")
        self.assertEqual(task["error"]["code"], "PARKING_MISALIGNED")
        self.assertEqual(self.step(task, "VERIFY_ARRIVAL")["status"], "FAILED")
        arm = self.step(task, "EXECUTE_ARM_TASK")
        self.assertEqual(arm["status"], "CANCELED")
        self.assertEqual(arm["result"]["outcome"], "NOT_EXECUTED_AFTER_FAILURE")
        self.assertEqual(len(self.parking_events(task)), 1)

    def test_unverified_view_is_not_treated_as_arrived(self) -> None:
        for status in ("NOT_FOUND", "AMBIGUOUS", "VIEW_UNVERIFIED"):
            with self.subTest(status=status):
                task = self.run_task(lambda status=status: observation(status))
                self.assertEqual(task["status"], "FAILED")
                self.assertEqual(task["error"]["code"], "PARKING_NOT_VERIFIED")
                self.assertEqual(self.step(task, "EXECUTE_ARM_TASK")["status"], "CANCELED")

    def test_check_that_cannot_start_fails_without_observation_event(self) -> None:
        def unavailable() -> dict[str, object]:
            raise ParkingObservationError(503, "PARKING_CAMERA_UNAVAILABLE", "no frames")

        task = self.run_task(unavailable)
        self.assertEqual(task["status"], "FAILED")
        self.assertEqual(task["error"]["code"], "PARKING_CAMERA_UNAVAILABLE")
        self.assertEqual(self.parking_events(task), [])
        self.assertEqual(self.step(task, "EXECUTE_ARM_TASK")["status"], "CANCELED")

    def test_without_arrival_check_verify_arrival_stays_mock(self) -> None:
        task = self.run_task(None)
        self.assertEqual(task["status"], "SUCCEEDED")
        self.assertEqual(self.step(task, "VERIFY_ARRIVAL")["result"]["outcome"], "SIMULATED_SUCCESS")
        self.assertEqual(self.parking_events(task), [])


if __name__ == "__main__":
    unittest.main()
