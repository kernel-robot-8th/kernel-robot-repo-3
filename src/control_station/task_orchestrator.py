"""Mock-only task execution on top of the persisted task/step state machine."""

from __future__ import annotations

import os
import threading
import uuid
from typing import Any

from control_station.models import StepStatus, TaskStatus
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

    def __init__(self, task_store: TaskStore, mode: str = "mock") -> None:
        self.task_store = task_store
        self.mode = mode
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
        base = {
            "source": "mock",
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
        self.task_store.transition_step(
            task_id,
            step_id,
            StepStatus.SUCCEEDED,
            expected_status=StepStatus.RUNNING,
            result_payload={"source": "mock", "outcome": "SIMULATED_SUCCESS"},
            event_payload={**base, "event": "MOCK_STEP_COMPLETED"},
        )
        return True

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
        error_code = (
            "MOCK_SCENARIO_FAILURE"
            if isinstance(error, MockScenarioFailure)
            else "MOCK_EXECUTION_ERROR"
        )
        event_name = (
            "MOCK_SCENARIO_FAILURE"
            if isinstance(error, MockScenarioFailure)
            else "MOCK_EXECUTION_FAILED"
        )
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
                        "source": "mock",
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
                    "source": "mock",
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
