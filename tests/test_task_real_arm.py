"""EXECUTE_ARM_TASK sending a real PICK_WITH_POLICY arm command (CONTROL_STATION_TASK_REAL_ARM=1)."""

from __future__ import annotations

import os
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from control_station.act_policies import load_act_policies
from control_station.arm_commands import ArmCommandStore
from control_station.load_check import MockLoadObserver
from control_station.shelf_slot_map import load_shelf_slot_map
from control_station.shelf_slot_zones import SlotSelectionError
from control_station.store import TaskStore
from control_station.task_orchestrator import MockTaskOrchestrator, policy_for_arm_job

TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED", "NEEDS_INTERVENTION"}


def seen_in_slot(object_class: str) -> dict[str, object]:
    """Front-camera evidence as select_slot_for_class returns it (values are test fixtures, not measurements)."""
    return {
        "camera_id": "shelf_camera", "captured_at": "2026-11-11T00:00:00+00:00", "frame_sequence": 7,
        "frame_sha256": "0" * 64, "pixel_transform_id": "test", "class_name": object_class, "confidence": 0.8,
        "bbox_xyxy_px": [1.0, 2.0, 3.0, 4.0], "anchor_px": [2.0, 4.0], "position_shelf_m": None,
        "lateral_offset_m": None, "slot_id": "slot_b_2_1", "slot_label": "오른쪽 선반 아래 단 좌",
        "shelf_pose_id": "pose", "shelf_pose_sha256": "1" * 64, "detector_weights_sha256": "2" * 64,
    }


def not_seen(object_class: str) -> dict[str, object]:
    raise SlotSelectionError("TARGET_NOT_FOUND", f"정면 카메라에서 칸 안의 {object_class}을(를) 찾지 못했습니다")


class FakeArmWorker(threading.Thread):
    """Claims arm commands like the OMX worker and answers with a fixed final event."""

    def __init__(self, store: ArmCommandStore, final: dict[str, object]) -> None:
        super().__init__(daemon=True)
        self.store, self.final, self.stop, self.seen = store, final, threading.Event(), []

    def run(self) -> None:
        while not self.stop.is_set():
            command = self.store.claim_next("arm-01", "fake", 0.2)
            if command is None:
                continue
            self.seen.append(command)
            base = {"robot_id": "arm-01", "worker_id": "fake", "simulated": True}
            self.store.apply_worker_event(command["command_id"], {**base, "event_id": "a", "event": "ACCEPTED"})
            self.store.apply_worker_event(
                command["command_id"], {**base, "event_id": "p", "event": "PROGRESS", "phase": "APPROACH_TARGET"}
            )
            self.store.apply_worker_event(command["command_id"], {**base, "event_id": "f", **self.final})


class RealArmTaskTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.store = TaskStore(str(Path(directory.name) / "tasks.sqlite3"))
        self.addCleanup(self.store.close)
        self.arm = ArmCommandStore(
            self.store.session_factory, slot_map=load_shelf_slot_map(), act_policies=load_act_policies()
        )
        patcher = mock.patch.dict(os.environ, {"CONTROL_STATION_MOCK_STEP_DELAY": "0.05"})
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_task(
        self, arm_job: str, final: dict[str, object] | None, load_check=None, evidence=seen_in_slot
    ) -> tuple[dict, list]:
        worker = None
        if final is not None:
            worker = FakeArmWorker(self.arm, final)
            worker.start()
            self.addCleanup(worker.join, 2)  # cleanups run last-in first-out: stop, then join, then the DB goes
            self.addCleanup(worker.stop.set)
        orchestrator = MockTaskOrchestrator(
            self.store, load_check=load_check, arm_commands=self.arm, target_evidence=evidence
        )
        self.addCleanup(orchestrator.close)
        task = self.store.create_task("Pinky 정차 위치", arm_job)
        orchestrator.start(task["id"])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            task = self.store.get_task(task["id"])
            if task["status"] in TERMINAL:
                return task, (worker.seen if worker else [])
            time.sleep(0.05)
        self.fail(f"task did not finish: {task['status']}")

    @staticmethod
    def step(task: dict, step_type: str) -> dict:
        return next(step for step in task["steps"] if step["step_type"] == step_type)

    def test_arm_job_text_chooses_the_policy(self) -> None:
        registry = load_act_policies()
        self.assertEqual(policy_for_arm_job(registry, "당근 집기")["policy_id"], "carrot_act_v2")
        self.assertEqual(policy_for_arm_job(registry, "물티슈 하나 올려줘")["policy_id"], "tissue_act_v1")
        self.assertIsNone(policy_for_arm_job(registry, "상자 적재"))

    def test_arm_back_home_passes_and_leaves_placement_to_the_load_check(self) -> None:
        final = {"event": "RESULT_UNKNOWN", "reason_code": "PLACE_NOT_VERIFIED", "pose_reached": True}
        task, seen = self.run_task("당근 집기", final, MockLoadObserver(1))
        self.assertEqual(task["status"], "SUCCEEDED")
        self.assertEqual(seen[0]["operation"], "PICK_WITH_POLICY")
        self.assertEqual(seen[0]["target"]["policy"]["policy_id"], "carrot_act_v2")
        # The worker receives the front-camera observation as the command's source (SR-13).
        self.assertEqual(seen[0]["source"]["kind"], "CAMERA")
        self.assertEqual(seen[0]["source"]["class_name"], "carrot")
        self.assertEqual(seen[0]["target"]["pick_slot_id"], "slot_b_2_1")
        arm_step = self.step(task, "EXECUTE_ARM_TASK")
        events = [event["event_type"] for event in task["events"]]
        self.assertLess(events.index("TARGET_CONFIRMED"), events.index("ARM_COMMAND_CREATED"))
        self.assertEqual(arm_step["result"]["outcome"], "ARM_HOME_PLACEMENT_NOT_VERIFIED")
        self.assertEqual(self.step(task, "VERIFY_ARM_RESULT")["result"]["outcome"], "ARM_HOME_PLACEMENT_LEFT_TO_VERIFY_LOAD")
        # The mock load count is not evidence, so the arm command stays UNKNOWN for the operator.
        command = self.arm.get(arm_step["result"]["arm_command_id"])
        self.assertEqual(command["state"], "UNKNOWN")
        self.assertIsNone(command["operator_cleared_at"])

    def test_policy_timeout_fails_the_task_without_a_load_check(self) -> None:
        final = {"event": "FAILED", "reason_code": "POLICY_TIMEOUT_NOT_AT_REST"}
        task, _ = self.run_task("당근 집기", final, MockLoadObserver(1))
        self.assertEqual(task["status"], "FAILED")
        arm_step = self.step(task, "EXECUTE_ARM_TASK")
        self.assertEqual((arm_step["status"], arm_step["error"]["code"]), ("FAILED", "POLICY_TIMEOUT_NOT_AT_REST"))
        verify_load = self.step(task, "VERIFY_LOAD")  # never run: marked not executed after the failure
        self.assertEqual((verify_load["status"], verify_load["result"]["outcome"]), ("CANCELED", "NOT_EXECUTED_AFTER_FAILURE"))

    def test_object_not_seen_by_the_front_camera_fails_before_any_arm_command(self) -> None:
        task, seen = self.run_task("당근 집기", {"event": "FAILED", "reason_code": "SHOULD_NOT_RUN"}, evidence=not_seen)
        self.assertEqual(task["status"], "FAILED")
        self.assertEqual(self.step(task, "EXECUTE_ARM_TASK")["error"]["code"], "TARGET_NOT_FOUND")
        self.assertEqual(self.arm.list_recent()["commands"], [])
        self.assertEqual(seen, [])
        # The failure is the camera's, not the arm worker's.
        failure = next(event for event in task["events"] if event["payload"].get("event") == "ARM_TASK_NOT_CONFIRMED")
        self.assertEqual(failure["payload"]["source"], "front_camera")

    def test_no_camera_evidence_path_fails_before_any_arm_command(self) -> None:
        task, _ = self.run_task("당근 집기", None, evidence=None)
        self.assertEqual(task["status"], "FAILED")
        self.assertEqual(self.step(task, "EXECUTE_ARM_TASK")["error"]["code"], "TARGET_EVIDENCE_NOT_CONFIGURED")
        self.assertEqual(self.arm.list_recent()["commands"], [])

    def test_unknown_arm_job_fails_before_any_arm_command(self) -> None:
        task, _ = self.run_task("상자 적재", None)
        self.assertEqual(task["status"], "FAILED")
        self.assertEqual(self.step(task, "EXECUTE_ARM_TASK")["error"]["code"], "ARM_JOB_POLICY_NOT_FOUND")
        self.assertEqual(self.arm.list_recent()["commands"], [])


if __name__ == "__main__":
    unittest.main()
