"""SQLAlchemy-backed persistence for server-created task records."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone
from pathlib import Path
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


DEFAULT_TASK_STEPS = (
    ("VALIDATE_ROBOTS", None),
    ("NAVIGATE_TO_DESTINATION", "MOBILE"),
    ("VERIFY_ARRIVAL", "MOBILE"),
    ("EXECUTE_ARM_TASK", "ARM"),
    ("VERIFY_ARM_RESULT", "ARM"),
    ("FINALIZE_TASK", None),
)

TASK_TRANSITIONS = {
    TaskStatus.PENDING: {TaskStatus.VALIDATING, TaskStatus.CANCELED},
    TaskStatus.VALIDATING: {
        TaskStatus.RUNNING,
        TaskStatus.FAILED,
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
        self._migrate()

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
        return {
            "id": step.id,
            "task_id": step.task_id,
            "sequence": step.sequence,
            "step_type": step.step_type,
            "robot_id": step.robot_id,
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

    def transition_task(
        self,
        task_id: str,
        new_status: TaskStatus,
        *,
        expected_status: TaskStatus | None = None,
        payload: dict[str, object] | None = None,
        resume_confirmed: bool = False,
    ) -> dict[str, Any]:
        now = datetime.now(timezone.utc).isoformat(timespec="seconds")
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
                    payload={**(payload or {}), "source": "control_station"},
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
        now = datetime.now(timezone.utc).isoformat(timespec="seconds")
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
                    payload={**(event_payload or {}), "source": "control_station"},
                )
            )
            session.flush()
            task = session.get(TaskRecord, task_id)
            assert task is not None
            session.expire(task, ["steps", "events"])
            return self._present_with_history(task)

    def create_task(self, destination: str, arm_job: str) -> dict[str, Any]:
        created_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
        task = TaskRecord(
            id=f"TASK-{uuid.uuid4().hex[:8].upper()}",
            destination=destination,
            arm_job=arm_job,
            status=TaskStatus.PENDING.value,
            created_at=created_at,
        )
        task.steps = [
            TaskStepRecord(
                id=f"STEP-{uuid.uuid4().hex[:12].upper()}",
                sequence=sequence,
                step_type=step_type,
                robot_id=robot_id,
                status="PENDING",
            )
            for sequence, (step_type, robot_id) in enumerate(DEFAULT_TASK_STEPS, start=1)
        ]
        task.events = [
            TaskEventRecord(
                id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                event_type="TASK_CREATED",
                previous_status=None,
                new_status=TaskStatus.PENDING.value,
                occurred_at=created_at,
                received_at=created_at,
                payload={"step_ids": [step.id for step in task.steps]},
            )
        ]

        with self._sessions.begin() as session:
            session.add(task)
            session.flush()
            return self._present_with_history(task)

    def close(self) -> None:
        self.engine.dispose()
