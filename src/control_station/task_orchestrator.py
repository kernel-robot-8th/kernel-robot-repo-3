"""Task execution on top of the persisted task/step state machine.

Every step is simulated (`source: mock`) except, when enabled with CONTROL_STATION_TASK_REAL_ARM=1,
EXECUTE_ARM_TASK: it queues one PICK_WITH_POLICY arm command for the policy named by the task's arm job
(e.g. "당근 집기") and waits for the arm worker's result. The command is made only when the front camera sees
the object in exactly one shelf slot (SR-13); that observation is stored with the command as its CAMERA source,
and without it the step fails before any arm command exists. The worker cannot see whether the object was grasped
and placed, so its best outcome is "back home, placement not verified"; VERIFY_ARM_RESULT records that the
decision is left to VERIFY_LOAD, and the arm command stays UNKNOWN for the operator unless a non-mock load
observation confirmed the load.
"""

from __future__ import annotations

import os
import threading
import time
import uuid
from typing import Any, Callable

from control_station.arm_commands import IN_FLIGHT, ArmCommandConflict, ArmCommandError, ArmCommandStore
from control_station.load_check import EXPECTED_LOAD_COUNT, LOAD_ERROR_CODES, LoadObserver, judge_load
from control_station.models import StepStatus, TaskStatus
from control_station.shelf_slot_zones import SlotSelectionError
from control_station.store import (
    DEFAULT_TASK_STEPS,
    InvalidStateTransition,
    PICK_AND_PLACE_STEPS,
    ROBOT_ID_BY_KIND,
    StateTransitionConflict,
    TaskNotFoundError,
    TaskStore,
)


class MockScenarioFailure(RuntimeError):
    """An explicitly configured mock-only failure at one workflow step."""


# Returns one parking observation (interface-contracts "Pinky 주차 정렬 관측"); raises with
# an error_code attribute when the observation could not start.
ArrivalCheck = Callable[[], dict[str, Any]]
# Front-camera evidence for one object class: the selection record of shelf_slot_zones.select_slot_for_class.
# Raises SlotSelectionError (error_code, details) when the camera does not single out one slot.
TargetEvidence = Callable[[str], dict[str, Any]]


class ArrivalNotConfirmed(RuntimeError):
    """The parking observation after navigation did not show Pinky on its registered spot."""

    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


class LoadNotConfirmed(RuntimeError):
    """The load observation after the arm step did not show the expected items on the cart."""

    def __init__(self, error_code: str, message: str, source: str) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.source = source


class ArmTaskNotConfirmed(RuntimeError):
    """The arm command for EXECUTE_ARM_TASK could not be sent or did not end back home."""

    def __init__(self, error_code: str, message: str, source: str = "arm_worker") -> None:
        super().__init__(message)
        self.error_code = error_code
        # Which side could not confirm: the front camera (no command made) or the arm worker / control station.
        self.source = source


def policy_for_arm_job(registry: Any, arm_job: str) -> dict[str, Any] | None:
    """The trained policy whose object name appears in the arm job text ("당근 집기" -> carrot policy)."""
    if registry is None:
        return None
    exact = registry.resolve(arm_job)
    if exact is not None:
        return exact
    text = arm_job.casefold()
    for policy in registry.policies.values():
        if any(name.casefold() in text for name in {policy["object_class"], policy["label"], *policy["aliases"]}):
            return policy
    return None


class TaskExecutionConflict(RuntimeError):
    def __init__(
        self,
        message: str,
        task: dict[str, Any] | None = None,
        *,
        error_code: str = "TASK_EXECUTION_CONFLICT",
    ) -> None:
        super().__init__(message)
        self.task = task
        self.error_code = error_code


class MockTaskOrchestrator:
    """Run the persisted workflow without opening a device or ROS connection."""

    def __init__(
        self,
        task_store: TaskStore,
        mode: str = "mock",
        arrival_check: ArrivalCheck | None = None,
        load_check: LoadObserver | None = None,
        arm_commands: ArmCommandStore | None = None,
        target_evidence: TargetEvidence | None = None,
    ) -> None:
        self.task_store = task_store
        self.mode = mode
        # When set, EXECUTE_ARM_TASK sends a real PICK_WITH_POLICY arm command instead of a mock success.
        self.arm_commands = arm_commands
        # Required for that real arm command: no front-camera evidence, no command (SR-13).
        self.target_evidence = target_evidence
        self._arm_results: dict[str, dict[str, Any]] = {}
        # When set, VERIFY_ARRIVAL succeeds only on an ARRIVED parking observation.
        self.arrival_check = arrival_check
        # When set, VERIFY_LOAD succeeds only when the observed count on the cart equals the expected count.
        self.load_check = load_check
        self.step_delay = self._read_step_delay()
        self.fail_step = self._read_failure_step() if mode == "mock" else None
        self._lock = threading.RLock()
        self._cancel_events: dict[str, threading.Event] = {}
        self._threads: dict[str, threading.Thread] = {}

    @staticmethod
    def _read_step_delay() -> float:
        raw = os.environ.get("CONTROL_STATION_MOCK_STEP_DELAY", "0.25")
        try:
            return min(2.0, max(0.05, float(raw)))
        except ValueError:
            return 0.25

    @staticmethod
    def _read_failure_step() -> str | None:
        raw = os.environ.get("CONTROL_STATION_MOCK_FAIL_STEP", "").strip()
        if not raw:
            return None
        valid_steps = {step_type for step_type, _ in DEFAULT_TASK_STEPS + PICK_AND_PLACE_STEPS}
        if raw not in valid_steps:
            raise ValueError(
                "CONTROL_STATION_MOCK_FAIL_STEP must match a configured task step type"
            )
        return raw

    def start(self, task_id: str) -> tuple[dict[str, Any], bool]:
        if self.mode != "mock":
            raise TaskExecutionConflict(
                "작업 실행은 모의 모드에서만 제공됩니다. 장치 명령은 보내지 않았습니다.",
                error_code="EXECUTION_MODE_UNSUPPORTED",
            )
        with self._lock:
            try:
                task, claimed = self.task_store.claim_task(task_id)
            except TaskNotFoundError:
                raise
            except (InvalidStateTransition, StateTransitionConflict) as exc:
                raise TaskExecutionConflict(
                    str(exc), error_code="TASK_STATE_CONFLICT"
                ) from exc
            if not claimed:
                if task["status"] in {TaskStatus.VALIDATING.value, TaskStatus.RUNNING.value}:
                    # A repeated start is idempotent and never creates a second worker.
                    if task_id in self._threads:
                        return task, False
                    raise TaskExecutionConflict(
                        "Task is active in storage but has no in-process mock worker; operator review is required",
                        task,
                        error_code="TASK_WORKER_OWNERSHIP_UNCERTAIN",
                    )
                raise TaskExecutionConflict(
                    f"Task {task_id} is already active or requires operator review",
                    task,
                    error_code="TASK_NOT_STARTABLE",
                )
            cancel_event = threading.Event()
            worker = threading.Thread(
                target=self._run,
                args=(task_id, cancel_event),
                name=f"mock-task-{task_id}",
                daemon=True,
            )
            self._cancel_events[task_id] = cancel_event
            self._threads[task_id] = worker
            try:
                worker.start()
            except RuntimeError as exc:
                self._cancel_events.pop(task_id, None)
                self._threads.pop(task_id, None)
                if task["steps"]:
                    self.task_store.transition_step(
                        task_id,
                        task["steps"][0]["id"],
                        StepStatus.FAILED,
                        expected_status=StepStatus.PENDING,
                        error_code="MOCK_WORKER_START_FAILED",
                        error_message=str(exc),
                        event_payload={
                            "source": "mock",
                            "event": "MOCK_WORKER_START_FAILED",
                            "task_id": task_id,
                            "step_id": task["steps"][0]["id"],
                        },
                    )
                self._mark_pending_steps_not_executed(task_id, reason="FAILURE")
                failed = self.task_store.transition_task(
                    task_id,
                    TaskStatus.FAILED,
                    expected_status=TaskStatus.VALIDATING,
                    payload={
                        "source": "mock",
                        "event": "MOCK_WORKER_START_FAILED",
                        "error": str(exc),
                    },
                )
                raise TaskExecutionConflict(
                    "Mock worker could not start",
                    failed,
                    error_code="MOCK_WORKER_START_FAILED",
                ) from exc
            return self.task_store.get_task(task_id) or task, True

    def cancel(self, task_id: str) -> tuple[dict[str, Any], bool]:
        if self.mode != "mock":
            raise TaskExecutionConflict(
                "작업 취소는 모의 모드에서만 제공됩니다. 장치 명령은 보내지 않았습니다.",
                error_code="EXECUTION_MODE_UNSUPPORTED",
            )
        with self._lock:
            task = self.task_store.get_task(task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            status = TaskStatus(task["status"])
            if status == TaskStatus.PENDING:
                try:
                    updated = self.task_store.cancel_pending_task(task_id)
                    return updated, True
                except StateTransitionConflict:
                    latest = self.task_store.get_task(task_id)
                    raise TaskExecutionConflict(
                        "Task state changed; reload before retrying",
                        latest,
                        error_code="TASK_STATE_CONFLICT",
                    )
            if status == TaskStatus.CANCELED:
                return task, False
            if status == TaskStatus.CANCELING:
                if task_id in self._threads:
                    return task, False
                raise TaskExecutionConflict(
                    "Task is canceling in storage but has no in-process mock worker; operator review is required",
                    task,
                    error_code="TASK_WORKER_OWNERSHIP_UNCERTAIN",
                )
            if status not in {TaskStatus.VALIDATING, TaskStatus.RUNNING}:
                raise TaskExecutionConflict(
                    f"Task cannot be canceled from {status.value}",
                    task,
                    error_code="TASK_NOT_CANCELLABLE",
                )
            event = self._cancel_events.get(task_id)
            if event is None:
                raise TaskExecutionConflict(
                    "No in-process mock worker owns this active task; operator review is required",
                    task,
                    error_code="TASK_WORKER_OWNERSHIP_UNCERTAIN",
                )
            try:
                updated = self.task_store.transition_task(
                    task_id,
                    TaskStatus.CANCELING,
                    expected_status=status,
                    payload={"source": "mock", "event": "CANCEL_REQUESTED"},
                )
            except StateTransitionConflict as exc:
                latest = self.task_store.get_task(task_id)
                if latest and latest["status"] in {
                    TaskStatus.CANCELING.value,
                    TaskStatus.CANCELED.value,
                }:
                    return latest, False
                raise TaskExecutionConflict(
                    "Task state changed; reload before retrying",
                    latest,
                    error_code="TASK_STATE_CONFLICT",
                ) from exc
            event.set()
            return updated, True

    def local_worker_state(self, task_id: str) -> str:
        """Report local ownership without inferring or changing persisted task state."""
        with self._lock:
            task = self.task_store.get_task(task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if task["status"] not in {
                TaskStatus.VALIDATING.value,
                TaskStatus.RUNNING.value,
                TaskStatus.CANCELING.value,
            }:
                return "NOT_ACTIVE"
            if self.mode != "mock":
                return "MODE_NOT_MANAGED"
            worker = self._threads.get(task_id)
            return (
                "LOCAL_WORKER_ACTIVE"
                if worker is not None and worker.is_alive()
                else "NO_LOCAL_WORKER"
            )

    def close(self) -> None:
        with self._lock:
            workers = list(self._threads.values())
            for task_id, event in self._cancel_events.items():
                task = self.task_store.get_task(task_id)
                if task and task["status"] in {
                    TaskStatus.VALIDATING.value,
                    TaskStatus.RUNNING.value,
                }:
                    try:
                        self.task_store.transition_task(
                            task_id,
                            TaskStatus.CANCELING,
                            expected_status=TaskStatus(task["status"]),
                            payload={"source": "mock", "event": "SERVER_SHUTDOWN"},
                        )
                    except StateTransitionConflict:
                        pass
                event.set()
        for worker in workers:
            worker.join(timeout=3.0)

    def _run(self, task_id: str, cancel_event: threading.Event) -> None:
        active_step_id: str | None = None
        try:
            task = self.task_store.get_task(task_id)
            if task is None:
                return
            steps = task["steps"]
            if not steps:
                raise RuntimeError("Task has no persisted steps")

            # Readiness is simulated explicitly and labeled as mock evidence.
            active_step_id = steps[0]["id"]
            if not self._run_step(task_id, steps[0], cancel_event):
                self._finish_cancel(task_id, active_step_id)
                return
            current = self.task_store.get_task(task_id)
            if current is None or current["status"] == TaskStatus.CANCELING.value:
                self._finish_cancel(task_id, active_step_id)
                return
            self.task_store.transition_task(
                task_id,
                TaskStatus.RUNNING,
                expected_status=TaskStatus.VALIDATING,
                payload={"source": "mock", "event": "MOCK_VALIDATION_PASSED"},
            )

            for step in steps[1:]:
                active_step_id = step["id"]
                if cancel_event.is_set() or not self._run_step(task_id, step, cancel_event):
                    self._finish_cancel(task_id, active_step_id)
                    return

            current = self.task_store.get_task(task_id)
            if current is None or current["status"] == TaskStatus.CANCELING.value:
                self._finish_cancel(task_id, active_step_id)
                return
            self.task_store.transition_task(
                task_id,
                TaskStatus.SUCCEEDED,
                expected_status=TaskStatus.RUNNING,
                payload={"source": "mock", "event": "MOCK_TASK_COMPLETED"},
            )
        except Exception as exc:
            self._finish_error(task_id, active_step_id, exc)
        finally:
            with self._lock:
                self._threads.pop(task_id, None)
                self._cancel_events.pop(task_id, None)
                self._arm_results.pop(task_id, None)

    def _run_step(
        self,
        task_id: str,
        step: dict[str, Any],
        cancel_event: threading.Event,
    ) -> bool:
        step_id = step["id"]
        expected_task_status = (
            TaskStatus.VALIDATING
            if step.get("sequence") == 1
            else TaskStatus.RUNNING
        )
        command_id = f"CMD-{uuid.uuid4().hex[:12].upper()}"
        real_arm = self.arm_commands is not None and step.get("step_type") in {"EXECUTE_ARM_TASK", "VERIFY_ARM_RESULT"}
        base = {
            "source": "arm_worker" if real_arm else "mock",
            "robot_id": self._mock_robot_id(step),
            "task_id": task_id,
            "step_id": step_id,
            "command_id": command_id,
        }
        with self._lock:
            if cancel_event.is_set():
                return False
            task = self.task_store.get_task(task_id)
            if task is None or task["status"] == TaskStatus.CANCELING.value:
                return False
            if task["status"] != expected_task_status.value:
                raise StateTransitionConflict(
                    "Mock step cannot start unless the task is in its expected active state"
                )
            self.task_store.transition_step(
                task_id,
                step_id,
                StepStatus.COMMAND_SENT,
                expected_status=StepStatus.PENDING,
                command_id=command_id,
                event_payload={**base, "event": "COMMAND_SENT"},
            )
            self.task_store.transition_step(
                task_id,
                step_id,
                StepStatus.ACCEPTED,
                expected_status=StepStatus.COMMAND_SENT,
                event_payload={**base, "event": "COMMAND_ACCEPTED"},
            )
            self.task_store.transition_step(
                task_id,
                step_id,
                StepStatus.RUNNING,
                expected_status=StepStatus.ACCEPTED,
                event_payload={**base, "event": "EXECUTION_STARTED"},
            )
        if real_arm and step.get("step_type") == "EXECUTE_ARM_TASK":
            arm_result = self._execute_arm_policy(task_id, step_id, command_id, base, cancel_event)
            if arm_result is None:
                return False
            self._arm_results[task_id] = arm_result
            self.task_store.transition_step(
                task_id, step_id, StepStatus.SUCCEEDED, expected_status=StepStatus.RUNNING,
                result_payload=arm_result, event_payload={**base, "event": "ARM_COMMAND_FINISHED"},
            )
            return True
        if real_arm and step.get("step_type") == "VERIFY_ARM_RESULT":
            arm_result = self._arm_results.get(task_id) or {}
            self.task_store.transition_step(
                task_id, step_id, StepStatus.SUCCEEDED, expected_status=StepStatus.RUNNING,
                result_payload={
                    "source": "control_station",
                    "outcome": "ARM_HOME_PLACEMENT_LEFT_TO_VERIFY_LOAD",
                    "arm_command_id": arm_result.get("arm_command_id"),
                    "message": "팔은 home으로 돌아왔다고 보고했지만 집기·적재는 확인하지 않았습니다. 적재 확인 단계가 판단합니다.",
                },
                event_payload={**base, "event": "ARM_RESULT_RECORDED"},
            )
            return True
        progress_delay = self.step_delay / 2
        if cancel_event.wait(progress_delay):
            return False
        self.task_store.record_step_event(
            task_id,
            step_id,
            command_id,
            "PROGRESS",
            {
                **base,
                "event": "PROGRESS",
                "progress": 50,
                "message": "모의 단계 진행 중",
            },
            expected_task_status=expected_task_status,
        )
        if cancel_event.wait(self.step_delay - progress_delay):
            return False
        if step.get("step_type") == self.fail_step:
            raise MockScenarioFailure(
                f"Configured mock scenario failure at step {self.fail_step}"
            )
        result_payload: dict[str, Any] = {"source": "mock", "outcome": "SIMULATED_SUCCESS"}
        if step.get("step_type") == "VERIFY_ARRIVAL" and self.arrival_check is not None:
            result_payload = self._verify_arrival(task_id, step_id, command_id, base)
        if step.get("step_type") == "VERIFY_LOAD" and self.load_check is not None:
            result_payload = self._verify_load(task_id, step_id, command_id, base)
            self._close_arm_command_after_load(task_id, result_payload)
        self.task_store.transition_step(
            task_id,
            step_id,
            StepStatus.SUCCEEDED,
            expected_status=StepStatus.RUNNING,
            result_payload=result_payload,
            event_payload={**base, "event": "MOCK_STEP_COMPLETED"},
        )
        return True

    def _execute_arm_policy(
        self,
        task_id: str,
        step_id: str,
        command_id: str,
        base: dict[str, Any],
        cancel_event: threading.Event,
    ) -> dict[str, Any] | None:
        """Queue PICK_WITH_POLICY for the task's arm job and wait for the worker. None means canceled."""
        assert self.arm_commands is not None
        task = self.task_store.get_task(task_id) or {}
        policy = policy_for_arm_job(self.arm_commands.act_policies, str(task.get("arm_job") or ""))
        if policy is None:
            raise ArmTaskNotConfirmed(
                "ARM_JOB_POLICY_NOT_FOUND", f"팔 작업 '{task.get('arm_job')}'에 맞는 학습 정책이 없습니다",
                source="control_station",
            )
        # SR-13: the object must be seen in exactly one slot by the front camera before the arm is asked to pick.
        if self.target_evidence is None:
            raise ArmTaskNotConfirmed(
                "TARGET_EVIDENCE_NOT_CONFIGURED", "정면 카메라 확인 경로가 없어 팔 명령을 만들지 않았습니다",
                source="control_station",
            )
        try:
            evidence = self.target_evidence(policy["object_class"])
        except SlotSelectionError as exc:
            raise ArmTaskNotConfirmed(
                exc.error_code, f"{exc} — 팔 명령을 만들지 않았습니다", source="front_camera"
            ) from exc
        self.task_store.record_step_event(
            task_id, step_id, command_id, "TARGET_CONFIRMED",
            {**base, "event": "TARGET_CONFIRMED", "camera_id": evidence.get("camera_id"),
             "object_class": evidence.get("class_name"), "slot_id": evidence.get("slot_id"),
             "confidence": evidence.get("confidence"), "captured_at": evidence.get("captured_at"),
             "message": f"정면 카메라: {evidence.get('class_name')} → {evidence.get('slot_label')}"},
            expected_task_status=TaskStatus.RUNNING,
        )
        try:
            arm_command = self.arm_commands.create(
                {
                    "robot_id": base.get("robot_id") or "arm-01",
                    "operation": "PICK_WITH_POLICY",
                    "valid_for_s": 60,
                    "target": {"object_class": policy["object_class"], "pick_slot_id": evidence.get("slot_id")},
                    "place": {"slot_id": "CART_1"},
                    "source": {"kind": "OPERATOR_INPUT"},  # replaced by the camera evidence
                },
                camera_source=evidence,
            )
        except (ArmCommandError, ArmCommandConflict) as exc:
            raise ArmTaskNotConfirmed(
                getattr(exc, "error_code", "ARM_COMMAND_INVALID"), str(exc), source="control_station"
            ) from exc
        arm_id = arm_command["command_id"]
        self.task_store.record_step_event(
            task_id, step_id, command_id, "ARM_COMMAND_CREATED",
            {**base, "event": "ARM_COMMAND_CREATED", "arm_command_id": arm_id, "policy_id": policy["policy_id"],
             "slot_id": evidence.get("slot_id")},
            expected_task_status=TaskStatus.RUNNING,
        )
        # Waiting longer than the policy run plus the queue, ACK and home move means the result is unclear.
        deadline = time.monotonic() + policy["max_run_s"] + 180
        last_state, cancel_sent = None, False
        while True:
            current = self.arm_commands.get(arm_id)
            state = current["state"]
            if state != last_state and not cancel_event.is_set():
                last_state = state
                try:
                    self.task_store.record_step_event(
                        task_id, step_id, command_id, "PROGRESS",
                        {**base, "event": "PROGRESS", "arm_command_id": arm_id, "arm_state": state,
                         "message": f"팔 명령 {arm_id}: {state}"},
                        expected_task_status=TaskStatus.RUNNING,
                    )
                except StateTransitionConflict:
                    pass
            if state != "QUEUED" and state not in {s.value for s in IN_FLIGHT}:
                break
            if cancel_event.is_set() and not cancel_sent:
                try:
                    self.arm_commands.cancel(arm_id)  # the worker stops the arm where it is
                except ArmCommandConflict:
                    pass
                cancel_sent = True
            if time.monotonic() > deadline:
                raise ArmTaskNotConfirmed(
                    "ARM_RESULT_TIMEOUT", f"팔 명령 {arm_id}의 결과가 오지 않았습니다. 팔을 현장에서 확인하세요."
                )
            time.sleep(0.5)
        if cancel_sent:
            return None
        result = current.get("result") or {}
        if state == "UNKNOWN" and current.get("reason_code") == "PLACE_NOT_VERIFIED" and result.get("pose_reached") is True:
            return {
                "source": "arm_worker",
                "outcome": "ARM_HOME_PLACEMENT_NOT_VERIFIED",
                "arm_command_id": arm_id,
                "policy_id": policy["policy_id"],
                "simulated": result.get("simulated"),
            }
        raise ArmTaskNotConfirmed(
            current.get("reason_code") or state,
            f"팔 명령 {arm_id}이(가) {state}({current.get('reason_code')})로 끝났습니다. {result.get('message') or ''}".strip(),
        )

    def _close_arm_command_after_load(self, task_id: str, load_result: dict[str, Any]) -> None:
        """A real (non-mock) load confirmation is the evidence the arm worker lacked; record it on the arm command
        so the next command is allowed. A mock load count leaves the command UNKNOWN for the operator."""
        arm_result = self._arm_results.pop(task_id, None)
        if self.arm_commands is None or not arm_result or load_result.get("source") == "mock":
            return
        try:
            self.arm_commands.clear_unknown(
                arm_result["arm_command_id"],
                f"작업 {task_id} 적재 확인({load_result.get('source')}): 관측 {load_result.get('observed_count')}개",
            )
        except (ArmCommandConflict, ArmCommandError):
            pass

    def _verify_arrival(
        self,
        task_id: str,
        step_id: str,
        command_id: str,
        base: dict[str, Any],
    ) -> dict[str, Any]:
        # Navigation has already succeeded, so the check judges only frames captured from now on.
        assert self.arrival_check is not None
        try:
            observation = self.arrival_check()
        except Exception as exc:
            raise ArrivalNotConfirmed(
                getattr(exc, "error_code", "PARKING_CHECK_ERROR"),
                f"주차 판정을 시작하지 못했습니다: {exc}",
            ) from exc
        self.task_store.record_step_event(
            task_id,
            step_id,
            command_id,
            "PARKING_OBSERVED",
            {
                **base,
                "source": "parking_check",
                "event": "PARKING_OBSERVED",
                "observation": observation,
                "task_state_changed": False,
                "arm_command_sent": False,
            },
            expected_task_status=TaskStatus.RUNNING,
        )
        status = observation.get("status")
        if status != "ARRIVED":
            # No automatic re-drive: the operator sees the offset and decides (pinky-arrival-gate plan).
            raise ArrivalNotConfirmed(
                "PARKING_MISALIGNED" if status == "MISALIGNED" else "PARKING_NOT_VERIFIED",
                f"주차 판정 {status}: {observation.get('reason')}",
            )
        return {
            "source": "parking_check",
            "outcome": "PARKING_ARRIVED",
            "spot_id": observation.get("spot_id"),
        }

    def _verify_load(
        self,
        task_id: str,
        step_id: str,
        command_id: str,
        base: dict[str, Any],
    ) -> dict[str, Any]:
        assert self.load_check is not None
        try:
            observation = self.load_check()
        except Exception as exc:
            raise LoadNotConfirmed(
                getattr(exc, "error_code", "LOAD_CHECK_ERROR"),
                f"적재 확인을 시작하지 못했습니다: {exc}",
                "load_check",
            ) from exc
        source = str(observation.get("source") or "load_check")
        status = judge_load(observation, EXPECTED_LOAD_COUNT)
        self.task_store.record_step_event(
            task_id,
            step_id,
            command_id,
            "LOAD_OBSERVED",
            {
                **base,
                "source": source,
                "event": "LOAD_OBSERVED",
                "observation": observation,
                "expected_count": EXPECTED_LOAD_COUNT,
                "status": status,
                "task_state_changed": False,
            },
            expected_task_status=TaskStatus.RUNNING,
        )
        if status != "LOADED":
            # Pinky does not leave with a load that was not confirmed; the operator decides what happens next.
            raise LoadNotConfirmed(
                LOAD_ERROR_CODES[status],
                f"적재 확인 {status}: 기대 {EXPECTED_LOAD_COUNT}개, 관측 {observation.get('observed_count')}개"
                f" ({observation.get('reason')})",
                source,
            )
        return {
            "source": source,
            "outcome": "LOAD_CONFIRMED",
            "expected_count": EXPECTED_LOAD_COUNT,
            "observed_count": observation.get("observed_count"),
        }

    def _finish_cancel(self, task_id: str, active_step_id: str | None) -> None:
        task = self.task_store.get_task(task_id)
        if task is None:
            return
        if active_step_id is not None:
            step = next((item for item in task["steps"] if item["id"] == active_step_id), None)
            if step and step["status"] in {
                StepStatus.PENDING.value,
                StepStatus.COMMAND_SENT.value,
                StepStatus.ACCEPTED.value,
                StepStatus.RUNNING.value,
            }:
                self.task_store.transition_step(
                    task_id,
                    active_step_id,
                    StepStatus.CANCELED,
                    expected_status=StepStatus(step["status"]),
                    result_payload={"source": "mock", "outcome": "CANCEL_CONFIRMED"},
                    event_payload={
                        "source": "mock",
                        "robot_id": self._mock_robot_id(step),
                        "event": "CANCEL_CONFIRMED",
                        "task_id": task_id,
                        "step_id": active_step_id,
                        "command_id": step.get("command_id"),
                    },
                )
        self._mark_pending_steps_not_executed(task_id, reason="CANCEL")
        task = self.task_store.get_task(task_id)
        if task and task["status"] == TaskStatus.CANCELING.value:
            self.task_store.transition_task(
                task_id,
                TaskStatus.CANCELED,
                expected_status=TaskStatus.CANCELING,
                payload={"source": "mock", "event": "MOCK_STOP_CONFIRMED"},
            )

    def _mark_pending_steps_not_executed(self, task_id: str, *, reason: str) -> None:
        if reason not in {"CANCEL", "FAILURE"}:
            raise ValueError(f"Unsupported pending-step closure reason: {reason}")
        outcome = f"NOT_EXECUTED_AFTER_{reason}"
        task = self.task_store.get_task(task_id)
        if task is None:
            return
        for step in task["steps"]:
            if step["status"] == StepStatus.PENDING.value:
                self.task_store.transition_step(
                    task_id,
                    step["id"],
                    StepStatus.CANCELED,
                    expected_status=StepStatus.PENDING,
                    result_payload={"source": "mock", "outcome": outcome},
                    event_payload={
                        "source": "mock",
                        "robot_id": self._mock_robot_id(step),
                        "event": outcome,
                        "task_id": task_id,
                        "step_id": step["id"],
                        "command_id": None,
                    },
                )

    def _finish_error(
        self,
        task_id: str,
        active_step_id: str | None,
        error: Exception,
    ) -> None:
        # Serialize terminal failure handling with a concurrent cancel request.
        # Whichever acquires the lock first determines the recorded terminal path.
        with self._lock:
            self._finish_error_locked(task_id, active_step_id, error)

    def _finish_error_locked(
        self,
        task_id: str,
        active_step_id: str | None,
        error: Exception,
    ) -> None:
        task = self.task_store.get_task(task_id)
        if task is None:
            return
        source = "mock"
        if isinstance(error, ArrivalNotConfirmed):
            source, error_code, event_name = "parking_check", error.error_code, "ARRIVAL_NOT_CONFIRMED"
        elif isinstance(error, LoadNotConfirmed):
            source, error_code, event_name = error.source, error.error_code, "LOAD_NOT_CONFIRMED"
        elif isinstance(error, ArmTaskNotConfirmed):
            source, error_code, event_name = error.source, error.error_code, "ARM_TASK_NOT_CONFIRMED"
        elif isinstance(error, MockScenarioFailure):
            error_code = event_name = "MOCK_SCENARIO_FAILURE"
        else:
            error_code, event_name = "MOCK_EXECUTION_ERROR", "MOCK_EXECUTION_FAILED"
        if task["status"] == TaskStatus.CANCELING.value:
            self._finish_cancel(task_id, active_step_id)
            return
        if active_step_id is not None:
            step = next((item for item in task["steps"] if item["id"] == active_step_id), None)
            if step and step["status"] in {
                StepStatus.PENDING.value,
                StepStatus.COMMAND_SENT.value,
                StepStatus.ACCEPTED.value,
                StepStatus.RUNNING.value,
            }:
                self.task_store.transition_step(
                    task_id,
                    active_step_id,
                    StepStatus.FAILED,
                    expected_status=StepStatus(step["status"]),
                    error_code=error_code,
                    error_message=str(error),
                    event_payload={
                        "source": source,
                        "robot_id": self._mock_robot_id(step),
                        "event": event_name,
                        "task_id": task_id,
                        "step_id": active_step_id,
                        "command_id": step.get("command_id"),
                        "error_code": error_code,
                        "error": str(error),
                    },
                )
        self._mark_pending_steps_not_executed(task_id, reason="FAILURE")
        task = self.task_store.get_task(task_id)
        if task and task["status"] in {
            TaskStatus.VALIDATING.value,
            TaskStatus.RUNNING.value,
        }:
            self.task_store.transition_task(
                task_id,
                TaskStatus.FAILED,
                expected_status=TaskStatus(task["status"]),
                payload={
                    "source": source,
                    "error_code": error_code,
                    "error": str(error),
                    "event": event_name,
                },
            )

    @staticmethod
    def _mock_robot_id(step: dict[str, Any]) -> str | None:
        """Resolve legacy robot kinds while preserving configured robot IDs."""
        robot_kind = step.get("robot_id")
        return ROBOT_ID_BY_KIND.get(robot_kind, robot_kind) if isinstance(robot_kind, str) else None
