from __future__ import annotations

import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from control_station.load_check import MockLoadObserver, judge_load, mock_load_observer_from_env
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator

TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED", "NEEDS_INTERVENTION"}


class LoadGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.store = TaskStore(str(Path(self.directory.name) / "tasks.sqlite3"))
        self.addCleanup(self.store.close)
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_task(self, load_check) -> dict[str, object]:
        orchestrator = MockTaskOrchestrator(self.store, load_check=load_check)
        self.addCleanup(orchestrator.close)
        task = self.store.create_task("작업대 A", "상자 적재")
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
    def load_events(task: dict[str, object]) -> list[dict[str, object]]:
        return [event for event in task["events"] if event["event_type"] == "LOAD_OBSERVED"]

    def test_verify_load_runs_after_arm_result_and_before_finalize(self) -> None:
        task = self.store.create_task("작업대 A", "상자 적재")
        order = [step["step_type"] for step in task["steps"]]
        self.assertEqual(order.index("VERIFY_LOAD"), order.index("VERIFY_ARM_RESULT") + 1)
        self.assertEqual(order[-1], "FINALIZE_TASK")
        self.assertIsNone(self.step(task, "VERIFY_LOAD")["robot_id"])

    def test_expected_count_on_cart_confirms_load(self) -> None:
        task = self.run_task(MockLoadObserver(1))
        self.assertEqual(task["status"], "SUCCEEDED")
        verify = self.step(task, "VERIFY_LOAD")
        self.assertEqual(verify["result"]["outcome"], "LOAD_CONFIRMED")
        self.assertEqual(verify["result"]["source"], "mock")
        events = self.load_events(task)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["step_id"], verify["id"])
        self.assertEqual(events[0]["payload"]["status"], "LOADED")
        self.assertTrue(events[0]["payload"]["observation"]["simulated"])

    def test_missing_or_extra_items_fail_before_finalize(self) -> None:
        for observed, code in ((0, "LOAD_INCOMPLETE"), (2, "LOAD_EXCESS")):
            with self.subTest(observed=observed):
                task = self.run_task(MockLoadObserver(observed))
                self.assertEqual(task["status"], "FAILED")
                self.assertEqual(task["error"]["code"], code)
                self.assertEqual(self.step(task, "VERIFY_LOAD")["status"], "FAILED")
                finalize = self.step(task, "FINALIZE_TASK")
                self.assertEqual(finalize["status"], "CANCELED")
                self.assertEqual(finalize["result"]["outcome"], "NOT_EXECUTED_AFTER_FAILURE")
                self.assertEqual(len(self.load_events(task)), 1)

    def test_observation_without_a_count_is_not_treated_as_loaded(self) -> None:
        task = self.run_task(lambda: {"source": "ceiling_yolo", "observed_count": None, "reason": "no frame"})
        self.assertEqual(task["status"], "FAILED")
        self.assertEqual(task["error"]["code"], "LOAD_NOT_VERIFIED")

    def test_check_that_cannot_start_fails_without_observation_event(self) -> None:
        class Unavailable(RuntimeError):
            error_code = "LOAD_CAMERA_UNAVAILABLE"

        def unavailable() -> dict[str, object]:
            raise Unavailable("no frames")

        task = self.run_task(unavailable)
        self.assertEqual(task["status"], "FAILED")
        self.assertEqual(task["error"]["code"], "LOAD_CAMERA_UNAVAILABLE")
        self.assertEqual(self.load_events(task), [])

    def test_without_load_check_verify_load_stays_mock(self) -> None:
        task = self.run_task(None)
        self.assertEqual(task["status"], "SUCCEEDED")
        self.assertEqual(self.step(task, "VERIFY_LOAD")["result"]["outcome"], "SIMULATED_SUCCESS")
        self.assertEqual(self.load_events(task), [])


class JudgeLoadTests(unittest.TestCase):
    def test_counts(self) -> None:
        self.assertEqual(judge_load({"observed_count": 1}, 1), "LOADED")
        self.assertEqual(judge_load({"observed_count": 0}, 1), "INCOMPLETE")
        self.assertEqual(judge_load({"observed_count": 3}, 1), "EXCESS")
        for count in (None, -1, True, 1.0, "1"):
            with self.subTest(count=count):
                self.assertEqual(judge_load({"observed_count": count}, 1), "NOT_VERIFIED")

    def test_env_observer(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(mock_load_observer_from_env())
        with mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_LOAD_OBSERVED": "0"}):
            self.assertEqual(mock_load_observer_from_env()()["observed_count"], 0)
        for raw in ("-1", "two"):
            with self.subTest(raw=raw), mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_LOAD_OBSERVED": raw}):
                with self.assertRaises(ValueError):
                    mock_load_observer_from_env()


if __name__ == "__main__":
    unittest.main()
