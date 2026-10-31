"""SQLAlchemy-backed persistence for server-created task records."""

from __future__ import annotations

import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from threading import Lock
from typing import Any

from alembic import command
from alembic.config import Config
from sqlalchemy import select, update
from sqlalchemy.engine import Engine
from sqlalchemy.orm import sessionmaker

from control_station.database import (
    TaskEventRecord,
    TaskRecord,
    TaskStepRecord,
    create_database_engine,
    database_path,
    make_session_factory,
)
from control_station.models import StepStatus, TaskStatus


ROBOT_ID_BY_KIND = {"MOBILE": "mobile-01", "ARM": "arm-01"}


DEFAULT_TASK_STEPS = (
    ("VALIDATE_ROBOTS", None),
    ("NAVIGATE_TO_DESTINATION", ROBOT_ID_BY_KIND["MOBILE"]),
    ("VERIFY_ARRIVAL", ROBOT_ID_BY_KIND["MOBILE"]),
    ("EXECUTE_ARM_TASK", ROBOT_ID_BY_KIND["ARM"]),
    ("VERIFY_ARM_RESULT", ROBOT_ID_BY_KIND["ARM"]),
    ("FINALIZE_TASK", None),
)

PICK_AND_PLACE_STEPS = (
    ("VALIDATE_REQUEST", None),
    ("DETECT_TARGETS", None),
    ("SELECT_TARGET", None),
    ("ESTIMATE_TARGET_POSE", None),
    ("VALIDATE_GRASP", None),
    ("APPROACH_TARGET", ROBOT_ID_BY_KIND["ARM"]),
    ("REFINE_TARGET", ROBOT_ID_BY_KIND["ARM"]),
    ("GRASP_TARGET", ROBOT_ID_BY_KIND["ARM"]),
    ("VERIFY_GRASP", ROBOT_ID_BY_KIND["ARM"]),
    ("LIFT_TARGET", ROBOT_ID_BY_KIND["ARM"]),
    ("MOVE_TO_DESTINATION", ROBOT_ID_BY_KIND["ARM"]),
    ("PLACE_TARGET", ROBOT_ID_BY_KIND["ARM"]),
    ("VERIFY_PLACEMENT", ROBOT_ID_BY_KIND["ARM"]),
    ("FINALIZE_TASK", None),
)

TASK_TRANSITIONS = {
    TaskStatus.PENDING: {TaskStatus.VALIDATING, TaskStatus.CANCELED},
    TaskStatus.VALIDATING: {
        TaskStatus.RUNNING,
        TaskStatus.FAILED,
        TaskStatus.CANCELING,
        TaskStatus.CANCELED,
        TaskStatus.NEEDS_INTERVENTION,
    },
    TaskStatus.RUNNING: {
        TaskStatus.SUCCEEDED,
        TaskStatus.FAILED,
        TaskStatus.CANCELING,
        TaskStatus.NEEDS_INTERVENTION,
    },
    TaskStatus.CANCELING: {
        TaskStatus.CANCELED,
        TaskStatus.FAILED,
        TaskStatus.NEEDS_INTERVENTION,
    },
    TaskStatus.NEEDS_INTERVENTION: {
        TaskStatus.CANCELED,
        TaskStatus.FAILED,
        TaskStatus.RUNNING,
    },
    TaskStatus.SUCCEEDED: set(),
    TaskStatus.FAILED: set(),
    TaskStatus.CANCELED: set(),
}

STEP_TRANSITIONS = {
    StepStatus.PENDING: {StepStatus.COMMAND_SENT, StepStatus.CANCELED, StepStatus.FAILED},
    StepStatus.COMMAND_SENT: {
        StepStatus.ACCEPTED,
        StepStatus.RUNNING,
        StepStatus.SUCCEEDED,
        StepStatus.FAILED,
        StepStatus.TIMED_OUT,
        StepStatus.CANCELED,
        StepStatus.UNKNOWN,
    },
    StepStatus.ACCEPTED: {
        StepStatus.RUNNING,
        StepStatus.SUCCEEDED,
        StepStatus.FAILED,
        StepStatus.TIMED_OUT,
        StepStatus.CANCELED,
        StepStatus.UNKNOWN,
    },
    StepStatus.RUNNING: {
        StepStatus.SUCCEEDED,
        StepStatus.FAILED,
        StepStatus.TIMED_OUT,
        StepStatus.CANCELED,
        StepStatus.UNKNOWN,
    },
    StepStatus.TIMED_OUT: {
        StepStatus.SUCCEEDED,
        StepStatus.FAILED,
        StepStatus.CANCELED,
        StepStatus.UNKNOWN,
    },
    StepStatus.UNKNOWN: {
        StepStatus.SUCCEEDED,
        StepStatus.FAILED,
        StepStatus.CANCELED,
    },
    StepStatus.SUCCEEDED: set(),
    StepStatus.FAILED: set(),
    StepStatus.CANCELED: set(),
}


class TaskNotFoundError(LookupError):
    pass


class StepNotFoundError(LookupError):
    pass


class InvalidStateTransition(ValueError):
    pass


class StateTransitionConflict(InvalidStateTransition):
    pass


class TaskStore:
    def __init__(self, db_path: str | None = None) -> None:
        self.path = Path(db_path) if db_path else database_path()
        self.engine: Engine = create_database_engine(self.path)
        self._sessions: sessionmaker = make_session_factory(self.engine)
        # The control station is a single local server process. Serialize task
        # claims in that process while the database CAS protects each task row.
        self._claim_lock = Lock()
        self._migrate()

    def claim_task(self, task_id: str) -> tuple[dict[str, Any], bool]:
        """Atomically reserve one pending task for this server process.

        Returns (task, started). Repeated requests for an already active task
        are idempotent; another active task prevents a new claim.
        """
        with self._claim_lock:
            with self._sessions() as session:
                task = session.get(TaskRecord, task_id)
                if task is None:
                    raise TaskNotFoundError(task_id)
                current = TaskStatus(task.status)
                if current in {
                    TaskStatus.VALIDATING,
                    TaskStatus.RUNNING,
                    TaskStatus.CANCELING,
                    TaskStatus.NEEDS_INTERVENTION,
                }:
                    return self._present_with_history(task), False
                if current != TaskStatus.PENDING:
                    raise InvalidStateTransition(
                        f"Task cannot start from {current.value}"
                    )
                active = session.scalars(
                    select(TaskRecord).where(
                        TaskRecord.status.in_(
                            [
                                TaskStatus.VALIDATING.value,
                                TaskStatus.RUNNING.value,
                                TaskStatus.CANCELING.value,
                                TaskStatus.NEEDS_INTERVENTION.value,
                            ]
                        )
                    )
                ).first()
                if active is not None:
                    raise StateTransitionConflict(
                        f"Task {active.id} is already active"
                    )
            claimed = self.transition_task(
                task_id,
                TaskStatus.VALIDATING,
                expected_status=TaskStatus.PENDING,
                payload={"source": "mock", "event": "START_REQUESTED"},
            )
            return claimed, True

    def cancel_pending_task(self, task_id: str) -> dict[str, Any]:
        """Atomically cancel an unstarted task and all of its pending steps."""
        started_at = datetime.now(timezone.utc)
        with self._sessions.begin() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if task.status != TaskStatus.PENDING.value:
                raise StateTransitionConflict(
                    f"Expected task state PENDING, found {task.status}"
                )
            pending_steps = [
                step for step in task.steps if step.status == StepStatus.PENDING.value
            ]
            if len(pending_steps) != len(task.steps):
                raise StateTransitionConflict(
                    "Pending task contains a non-pending step; operator review is required"
                )
            task_finished_at = (
                started_at + timedelta(microseconds=len(pending_steps))
            ).isoformat(timespec="microseconds")

            result = session.execute(
                update(TaskRecord)
                .where(
                    TaskRecord.id == task_id,
                    TaskRecord.status == TaskStatus.PENDING.value,
                )
                .values(status=TaskStatus.CANCELED.value, finished_at=task_finished_at)
            )
            if result.rowcount != 1:
                raise StateTransitionConflict(
                    "Task state changed concurrently; reload before retrying"
                )
            session.add(
                TaskEventRecord(
                    id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                    task_id=task_id,
                    event_type="TASK_STATUS_CHANGED",
                    previous_status=TaskStatus.PENDING.value,
                    new_status=TaskStatus.CANCELED.value,
                    occurred_at=task_finished_at,
                    received_at=task_finished_at,
                    payload={"source": "mock", "event": "CANCEL_CONFIRMED"},
                )
            )
            for index, step in enumerate(pending_steps):
                step_finished_at = (
                    started_at + timedelta(microseconds=index)
                ).isoformat(timespec="microseconds")
                step.status = StepStatus.CANCELED.value
                step.finished_at = step_finished_at
                step.result = {"source": "mock", "outcome": "NOT_EXECUTED"}
                session.add(
                    TaskEventRecord(
                        id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                        task_id=task_id,
                        step_id=step.id,
                        event_type="STEP_STATUS_CHANGED",
                        previous_status=StepStatus.PENDING.value,
                        new_status=StepStatus.CANCELED.value,
                        occurred_at=step_finished_at,
                        received_at=step_finished_at,
                        payload={
                            "source": "mock",
                            "event": "NOT_EXECUTED_AFTER_CANCEL",
                            "task_id": task_id,
                            "step_id": step.id,
                            "command_id": None,
                        },
                    )
                )
            session.flush()
            session.refresh(task)
            session.expire(task, ["steps", "events"])
            return self._present_with_history(task)

    def _migrate(self) -> None:
        project_root = Path(__file__).resolve().parents[2]
        config = Config(str(project_root / "alembic.ini"))
        database_url = self.engine.url.render_as_string(hide_password=False)
        config.set_main_option("sqlalchemy.url", database_url.replace("%", "%%"))
        command.upgrade(config, "head")

    @staticmethod
    def _present(task: TaskRecord) -> dict[str, Any]:
        return {
            "id": task.id,
            "destination": task.destination,
            "arm_job": task.arm_job,
            "pick_request": task.pick_request,
            "workflow": "PICK_AND_PLACE" if task.pick_request else "TRANSPORT_AND_ARM",
            "status": task.status,
            "created_at": task.created_at,
            "started_at": task.started_at,
            "finished_at": task.finished_at,
            "active_step_id": task.active_step_id,
            "error": (
                {"code": task.error_code, "message": task.error_message}
                if task.error_code is not None or task.error_message is not None
                else None
            ),
        }

    @staticmethod
    def _present_step(step: TaskStepRecord) -> dict[str, Any]:
        robot_id = step.robot_id
        if robot_id is not None:
            robot_id = ROBOT_ID_BY_KIND.get(robot_id, robot_id)
        return {
            "id": step.id,
            "task_id": step.task_id,
            "sequence": step.sequence,
            "step_type": step.step_type,
            "robot_id": robot_id,
            "status": step.status,
            "command_id": step.command_id,
            "started_at": step.started_at,
            "finished_at": step.finished_at,
            "result": step.result,
            "error": (
                {"code": step.error_code, "message": step.error_message}
                if step.error_code is not None or step.error_message is not None
                else None
            ),
        }

    @staticmethod
    def _present_event(event: TaskEventRecord) -> dict[str, Any]:
        return {
            "id": event.id,
            "task_id": event.task_id,
            "step_id": event.step_id,
            "event_type": event.event_type,
            "previous_status": event.previous_status,
            "new_status": event.new_status,
            "occurred_at": event.occurred_at,
            "received_at": event.received_at,
            "payload": event.payload,
        }

    @classmethod
    def _present_with_history(cls, task: TaskRecord) -> dict[str, Any]:
        result = cls._present(task)
        result["steps"] = [cls._present_step(step) for step in task.steps]
        result["events"] = [cls._present_event(event) for event in task.events]
        return result

    def list_tasks(self) -> list[dict[str, Any]]:
        with self._sessions() as session:
            tasks = session.scalars(
                select(TaskRecord).order_by(TaskRecord.created_at.desc(), TaskRecord.id.desc())
            ).all()
            return [self._present(task) for task in tasks]

    def get_task(self, task_id: str) -> dict[str, Any] | None:
        with self._sessions() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                return None
            return self._present_with_history(task)

    def record_task_event(
        self,
        task_id: str,
        event_type: str,
        payload: dict[str, object],
        *,
        expected_status: TaskStatus,
        step_id: str | None = None,
    ) -> dict[str, Any]:
        """Append task/optional-step evidence without changing task or step state."""
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._sessions.begin() as session:
            result = session.execute(
                update(TaskRecord)
                .where(
                    TaskRecord.id == task_id,
                    TaskRecord.status == expected_status.value,
                )
                .values(status=expected_status.value)
            )
            if result.rowcount != 1:
                task = session.get(TaskRecord, task_id)
                if task is None:
                    raise TaskNotFoundError(task_id)
                raise StateTransitionConflict(
                    f"Expected task state {expected_status.value}, found {task.status}"
                )
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if step_id is not None:
                step = session.get(TaskStepRecord, step_id)
                if step is None or step.task_id != task_id:
                    raise StepNotFoundError(step_id)
                if step.status != StepStatus.PENDING.value:
                    raise StateTransitionConflict(
                        f"Expected step state PENDING, found {step.status}"
                    )
            session.add(
                TaskEventRecord(
                    id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                    task_id=task_id,
                    step_id=step_id,
                    event_type=event_type,
                    previous_status=task.status,
                    new_status=task.status,
                    occurred_at=now,
                    received_at=now,
                    payload={"source": "camera_observation", **payload},
                )
            )
            session.flush()
            session.expire(task, ["events"])
            return self._present_with_history(task)

    def record_target_selection_event(
        self,
        task_id: str,
        payload: dict[str, object],
        *,
        expected_status: TaskStatus,
        step_id: str,
    ) -> tuple[dict[str, Any], str, bool]:
        """Atomically append or retrieve an identical operator selection event."""
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._sessions.begin() as session:
            result = session.execute(
                update(TaskRecord)
                .where(
                    TaskRecord.id == task_id,
                    TaskRecord.status == expected_status.value,
                )
                .values(status=expected_status.value)
            )
            if result.rowcount != 1:
                task = session.get(TaskRecord, task_id)
                if task is None:
                    raise TaskNotFoundError(task_id)
                raise StateTransitionConflict(
                    f"Expected task state {expected_status.value}, found {task.status}"
                )
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)

            existing_events = session.scalars(
                select(TaskEventRecord).where(
                    TaskEventRecord.task_id == task_id,
                    TaskEventRecord.event_type == "TARGET_SELECTED",
                )
            ).all()
            matching_events = [
                event
                for event in existing_events
                if event.step_id == step_id and event.payload == payload
            ]
            if matching_events:
                existing = matching_events[-1]
                return self._present_with_history(task), existing.id, True

            step = session.get(TaskStepRecord, step_id)
            if step is None or step.task_id != task_id:
                raise StepNotFoundError(step_id)
            if step.status != StepStatus.PENDING.value:
                raise StateTransitionConflict(
                    f"Expected step state PENDING, found {step.status}"
                )
            event = TaskEventRecord(
                id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                task_id=task_id,
                step_id=step_id,
                event_type="TARGET_SELECTED",
                previous_status=task.status,
                new_status=task.status,
                occurred_at=now,
                received_at=now,
                payload={"source": "operator_review", **payload},
            )
            session.add(event)
            session.flush()
            session.expire(task, ["events"])
            return self._present_with_history(task), event.id, False

    def record_step_event(
        self,
        task_id: str,
        step_id: str,
        command_id: str,
        event_type: str,
        payload: dict[str, object],
        *,
        expected_task_status: TaskStatus,
        expected_step_status: StepStatus = StepStatus.RUNNING,
    ) -> dict[str, Any]:
        """Append evidence only while the same task step and command remain active."""
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._sessions.begin() as session:
            result = session.execute(
                update(TaskRecord)
                .where(
                    TaskRecord.id == task_id,
                    TaskRecord.status == expected_task_status.value,
                )
                .values(status=expected_task_status.value)
            )
            if result.rowcount != 1:
                task = session.get(TaskRecord, task_id)
                if task is None:
                    raise TaskNotFoundError(task_id)
                raise StateTransitionConflict(
                    f"Expected task state {expected_task_status.value}, found {task.status}"
                )
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if task.active_step_id != step_id:
                raise StateTransitionConflict(
                    "Step is no longer the task's active step; reload before recording event"
                )
            step = session.get(TaskStepRecord, step_id)
            if step is None or step.task_id != task_id:
                raise StepNotFoundError(step_id)
            if step.status != expected_step_status.value:
                raise StateTransitionConflict(
                    f"Expected step state {expected_step_status.value}, found {step.status}"
                )
            if step.command_id != command_id:
                raise StateTransitionConflict("Step command ID changed; reload before recording event")

            session.add(
                TaskEventRecord(
                    id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                    task_id=task_id,
                    step_id=step_id,
                    event_type=event_type,
                    previous_status=task.status,
                    new_status=task.status,
                    occurred_at=now,
                    received_at=now,
                    payload={"source": "control_station", **payload},
                )
            )
            session.flush()
            session.expire(task, ["events"])
            return self._present_with_history(task)

    def record_projection_draft_event(
        self,
        task_id: str,
        payload: dict[str, object],
        *,
        expected_status: TaskStatus,
        step_id: str,
    ) -> tuple[dict[str, Any], str, bool]:
        """Atomically append one projection review event per draft ID and artifact hash."""
        draft_id = payload.get("draft_id")
        artifact_sha256 = payload.get("artifact_sha256")
        if not isinstance(draft_id, str) or not isinstance(artifact_sha256, str):
            raise ValueError("projection event requires draft ID and artifact hash")
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._sessions.begin() as session:
            result = session.execute(
                update(TaskRecord)
                .where(
                    TaskRecord.id == task_id,
                    TaskRecord.status == expected_status.value,
                )
                .values(status=expected_status.value)
            )
            if result.rowcount != 1:
                task = session.get(TaskRecord, task_id)
                if task is None:
                    raise TaskNotFoundError(task_id)
                raise StateTransitionConflict(
                    f"Expected task state {expected_status.value}, found {task.status}"
                )
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            existing_events = session.scalars(
                select(TaskEventRecord).where(
                    TaskEventRecord.task_id == task_id,
                    TaskEventRecord.event_type == "TARGET_PLANE_PROJECTION_RECORDED",
                )
            ).all()
            matching_events = [
                event
                for event in existing_events
                if isinstance(event.payload, dict) and event.payload.get("draft_id") == draft_id
            ]
            if matching_events:
                existing = matching_events[-1]
                if (
                    existing.payload.get("artifact_sha256") != artifact_sha256
                    or existing.step_id != step_id
                ):
                    raise StateTransitionConflict(
                        "Projection draft ID is already linked to a different artifact or step"
                    )
                return self._present_with_history(task), existing.id, True

            step = session.get(TaskStepRecord, step_id)
            if step is None or step.task_id != task_id:
                raise StepNotFoundError(step_id)
            if step.status != StepStatus.PENDING.value:
                raise StateTransitionConflict(
                    f"Expected step state PENDING, found {step.status}"
                )
            event = TaskEventRecord(
                id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                task_id=task_id,
                step_id=step_id,
                event_type="TARGET_PLANE_PROJECTION_RECORDED",
                previous_status=task.status,
                new_status=task.status,
                occurred_at=now,
                received_at=now,
                payload={"source": "projection_draft_review", **payload},
            )
            session.add(event)
            session.flush()
            session.expire(task, ["events"])
            return self._present_with_history(task), event.id, False

    def transition_task(
        self,
        task_id: str,
        new_status: TaskStatus,
        *,
        expected_status: TaskStatus | None = None,
        payload: dict[str, object] | None = None,
        resume_confirmed: bool = False,
    ) -> dict[str, Any]:
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._sessions.begin() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)

            previous = TaskStatus(task.status)
            if expected_status is not None and previous != expected_status:
                raise StateTransitionConflict(
                    f"Expected task state {expected_status.value}, found {previous.value}"
                )
            if new_status not in TASK_TRANSITIONS[previous]:
                raise InvalidStateTransition(
                    f"Task cannot transition from {previous.value} to {new_status.value}"
                )
            if previous == TaskStatus.NEEDS_INTERVENTION and new_status == TaskStatus.RUNNING and not resume_confirmed:
                raise InvalidStateTransition("Resuming requires explicit operator confirmation")

            values: dict[str, object] = {"status": new_status.value}
            if new_status == TaskStatus.VALIDATING and task.started_at is None:
                values["started_at"] = now
            if new_status in {TaskStatus.SUCCEEDED, TaskStatus.FAILED, TaskStatus.CANCELED}:
                values["finished_at"] = now
            if new_status in {TaskStatus.FAILED, TaskStatus.NEEDS_INTERVENTION} and payload:
                if "error_code" in payload:
                    values["error_code"] = payload["error_code"]
                if "error" in payload:
                    values["error_message"] = payload["error"]
            if (
                previous == TaskStatus.NEEDS_INTERVENTION
                and new_status == TaskStatus.RUNNING
                and resume_confirmed
            ):
                values["error_code"] = None
                values["error_message"] = None

            result = session.execute(
                update(TaskRecord)
                .where(TaskRecord.id == task_id, TaskRecord.status == previous.value)
                .values(**values)
            )
            if result.rowcount != 1:
                raise StateTransitionConflict("Task state changed concurrently; reload before retrying")

            session.add(
                TaskEventRecord(
                    id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                    task_id=task_id,
                    event_type="TASK_STATUS_CHANGED",
                    previous_status=previous.value,
                    new_status=new_status.value,
                    occurred_at=now,
                    received_at=now,
                    payload={"source": "control_station", **(payload or {})},
                )
            )
            session.flush()
            session.refresh(task)
            session.expire(task, ["steps", "events"])
            return self._present_with_history(task)

    def transition_step(
        self,
        task_id: str,
        step_id: str,
        new_status: StepStatus,
        *,
        expected_status: StepStatus | None = None,
        command_id: str | None = None,
        result_payload: dict[str, object] | None = None,
        error_code: str | None = None,
        error_message: str | None = None,
        event_payload: dict[str, object] | None = None,
    ) -> dict[str, Any]:
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._sessions.begin() as session:
            step = session.get(TaskStepRecord, step_id)
            if step is None or step.task_id != task_id:
                raise StepNotFoundError(step_id)

            previous = StepStatus(step.status)
            if expected_status is not None and previous != expected_status:
                raise StateTransitionConflict(
                    f"Expected step state {expected_status.value}, found {previous.value}"
                )
            if new_status not in STEP_TRANSITIONS[previous]:
                raise InvalidStateTransition(
                    f"Step cannot transition from {previous.value} to {new_status.value}"
                )

            values: dict[str, object] = {"status": new_status.value}
            if command_id is not None:
                values["command_id"] = command_id
            if result_payload is not None:
                values["result"] = result_payload
            if error_code is not None or error_message is not None:
                values["error_code"] = error_code
                values["error_message"] = error_message
            if new_status in {StepStatus.COMMAND_SENT, StepStatus.ACCEPTED, StepStatus.RUNNING} and step.started_at is None:
                values["started_at"] = now
            if new_status in {
                StepStatus.SUCCEEDED,
                StepStatus.FAILED,
                StepStatus.TIMED_OUT,
                StepStatus.CANCELED,
            }:
                values["finished_at"] = now

            update_result = session.execute(
                update(TaskStepRecord)
                .where(
                    TaskStepRecord.id == step_id,
                    TaskStepRecord.task_id == task_id,
                    TaskStepRecord.status == previous.value,
                )
                .values(**values)
            )
            if update_result.rowcount != 1:
                raise StateTransitionConflict("Step state changed concurrently; reload before retrying")

            session.add(
                TaskEventRecord(
                    id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                    task_id=task_id,
                    step_id=step_id,
                    event_type="STEP_STATUS_CHANGED",
                    previous_status=previous.value,
                    new_status=new_status.value,
                    occurred_at=now,
                    received_at=now,
                    payload={"source": "control_station", **(event_payload or {})},
                )
            )
            session.flush()
            task = session.get(TaskRecord, task_id)
            assert task is not None
            if new_status in {
                StepStatus.COMMAND_SENT,
                StepStatus.ACCEPTED,
                StepStatus.RUNNING,
            }:
                task.active_step_id = step_id
            elif task.active_step_id == step_id:
                task.active_step_id = None
            session.expire(task, ["steps", "events"])
            return self._present_with_history(task)

    def create_task(
        self,
        destination: str,
        arm_job: str,
        pick_request: dict[str, object] | None = None,
    ) -> dict[str, Any]:
        created_at = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        task = TaskRecord(
            id=f"TASK-{uuid.uuid4().hex[:8].upper()}",
            destination=destination,
            arm_job=arm_job,
            pick_request=pick_request,
            status=TaskStatus.PENDING.value,
            created_at=created_at,
        )
        step_definitions = PICK_AND_PLACE_STEPS if pick_request else DEFAULT_TASK_STEPS
        task.steps = [
            TaskStepRecord(
                id=f"STEP-{uuid.uuid4().hex[:12].upper()}",
                sequence=sequence,
                step_type=step_type,
                robot_id=robot_id,
                status="PENDING",
            )
            for sequence, (step_type, robot_id) in enumerate(step_definitions, start=1)
        ]
        task.events = [
            TaskEventRecord(
                id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                event_type="TASK_CREATED",
                previous_status=None,
                new_status=TaskStatus.PENDING.value,
                occurred_at=created_at,
                received_at=created_at,
                payload={
                    "step_ids": [step.id for step in task.steps],
                    "pick_request": pick_request,
                },
            )
        ]

        with self._sessions.begin() as session:
            session.add(task)
            session.flush()
            return self._present_with_history(task)

    def close(self) -> None:
        self.engine.dispose()
