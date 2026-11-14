"""SQLAlchemy-backed persistence for server-created task records."""

from __future__ import annotations

from collections import Counter

import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from threading import Lock
from typing import Any

from alembic import command
from alembic.config import Config
from sqlalchemy import exists, or_, select, update
from sqlalchemy.engine import Engine
from sqlalchemy.orm import sessionmaker

from control_station.database import (
    OrderRequestRecord,
    TaskEventRecord,
    TaskRecord,
    TaskStepRecord,
    create_database_engine,
    database_path,
    make_session_factory,
)
from control_station.models import StepStatus, TaskStatus


ROBOT_ID_BY_KIND = {"MOBILE": "mobile-01", "ARM": "arm-01"}
# The three Pinky robots by their ROS domain, as the Pinky fleet dashboard names them (P0=10, P1=11, P2=12).
PINKY_IDS = ("10", "11", "12")
PINKY_NAMES = {"10": "P0", "11": "P1", "12": "P2"}
# An order holds its Pinky from creation until it ends, so a pending order also keeps the Pinky busy.
_ORDER_HOLDING_STATUSES = (
    TaskStatus.PENDING.value,
    TaskStatus.VALIDATING.value,
    TaskStatus.RUNNING.value,
    TaskStatus.CANCELING.value,
    TaskStatus.NEEDS_INTERVENTION.value,
)


DEFAULT_TASK_STEPS = (
    ("VALIDATE_ROBOTS", None),
    ("NAVIGATE_TO_DESTINATION", ROBOT_ID_BY_KIND["MOBILE"]),
    ("VERIFY_ARRIVAL", ROBOT_ID_BY_KIND["MOBILE"]),
    ("EXECUTE_ARM_TASK", ROBOT_ID_BY_KIND["ARM"]),
    ("VERIFY_ARM_RESULT", ROBOT_ID_BY_KIND["ARM"]),
    ("VERIFY_LOAD", None),
    ("FINALIZE_TASK", None),
)

# Order steps carried out by (and so tagged with) the order's Pinky.
PINKY_STEP_TYPES = frozenset({"WAIT_PINKY_ARRIVAL", "CONFIRM_DOCKING", "NOTIFY_PINKY_RECEIVED",
                              "WAIT_AT_RECEIVING_ZONE", "CONFIRM_PICKUP", "WAIT_PINKY_PARKED"})


def pinky_robot_id(pinky_id: str | None) -> str | None:
    return None if pinky_id is None else f"pinky-{pinky_id}"


def order_step_definitions(
    order_items: list[dict[str, object]], pinky_id: str | None, *, receiving_zone: bool = False,
    shipments: list[list[dict[str, object]]] | None = None,
) -> list[tuple[str, str | None, dict[str, object] | None]]:
    """Steps of one order: (step_type, robot_id, detail). One PICK_ITEM + VERIFY_TRANSFER pair per object unit;
    the arm of each pair is filled in by PLAN_ARM_SEQUENCE after the shelf stock check. Without a Pinky yet (an
    order waiting in the intake), the Pinky steps carry no robot until assign_pinky fills them in.

    `shipments` splits a larger order into Pinky loads (assignment.split_into_shipments, unit dicts). They run one
    after another: the loading-to-parking steps repeat per shipment, each tagged {"shipment": k, "shipments": n},
    and shipment k > 1 opens with START_SHIPMENT, which waits for the previous Pinky to be parked, picks the Pinky
    for this load and sends it off. Only one Pinky drives at a time and the order keeps its receiving zone until
    the last load is picked up. A shipped order needs the receiving-zone steps (each load is picked up there)."""
    pinky = pinky_robot_id(pinky_id)
    units = [
        {"object_class": item["object_class"], "unit": index, "of": item["quantity"]}
        for item in order_items
        for index in range(1, int(item["quantity"]) + 1)  # type: ignore[call-overload]
    ]
    loads = [list(load) for load in shipments] if shipments else [units]
    if len(loads) > 1 and not receiving_zone:
        raise ValueError("an order split into shipments needs the receiving-zone steps")
    steps: list[tuple[str, str | None, dict[str, object] | None]] = [
        ("VALIDATE_ORDER", None, None),
        ("CHECK_SHELF_STOCK", None, None),
        ("PLAN_ARM_SEQUENCE", None, None),
    ]
    for number, load in enumerate(loads, start=1):
        tag: dict[str, object] | None = {"shipment": number, "shipments": len(loads)} if len(loads) > 1 else None
        robot = pinky if number == 1 else None  # later loads get their Pinky when they start
        if number > 1:
            steps.append(("START_SHIPMENT", None, dict(tag or {})))
        steps += [("WAIT_PINKY_ARRIVAL", robot, _tagged(None, tag)), ("CONFIRM_DOCKING", robot, _tagged(None, tag))]
        for unit in load:
            steps.append(("PICK_ITEM", None, _tagged(unit, tag)))
            steps.append(("VERIFY_TRANSFER", None, _tagged(unit, tag)))
        steps += [
            ("VERIFY_LOAD_TOTAL", None, _tagged(None, tag)),
            ("NOTIFY_PINKY_RECEIVED", robot, _tagged(None, tag)),
        ]
        if receiving_zone:
            # The Pinky carries the load on to the receiving zone, waits for the customer, then returns home.
            steps += [("WAIT_AT_RECEIVING_ZONE", robot, _tagged(None, tag)), ("CONFIRM_PICKUP", robot, _tagged(None, tag)),
                      ("WAIT_PINKY_PARKED", robot, _tagged(None, tag))]
    steps.append(("FINALIZE_TASK", None, None))
    return steps


def _tagged(detail: dict[str, object] | None, tag: dict[str, object] | None) -> dict[str, object] | None:
    if detail is None and tag is None:
        return None
    return {**(detail or {}), **(tag or {})}


def shipment_of(step: dict[str, Any] | Any) -> int:
    """Which shipment (1, 2, ...) a step belongs to; 1 for orders that are not split and for shared steps."""
    detail = step.get("detail") if isinstance(step, dict) else getattr(step, "detail", None)
    return int((detail or {}).get("shipment") or 1)


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


def _pinky_holding_orders():  # type: ignore[no-untyped-def]
    """Orders that keep their Pinky: unfinished ones, and stopped (FAILED) ones until the operator acknowledged the
    alert, because a stopped order's Pinky may still stand at the loading spot."""
    acknowledged = exists().where(
        TaskEventRecord.task_id == TaskRecord.id, TaskEventRecord.event_type == "ALERT_ACKNOWLEDGED"
    )
    return select(TaskRecord.pinky_id, TaskRecord.id).where(
        TaskRecord.pinky_id.is_not(None),
        or_(
            TaskRecord.status.in_(_ORDER_HOLDING_STATUSES),
            (TaskRecord.status == TaskStatus.FAILED.value) & ~acknowledged,
        ),
    )


def _zone_holding_orders():  # type: ignore[no-untyped-def]
    """Orders that keep their receiving zone: started and not yet picked up there (every shipment of a split
    order: the zone is kept between them). A waiting (PENDING) order holds no zone; a stopped (FAILED) one keeps it
    until the operator acknowledged the alert, like its Pinky."""
    acknowledged = exists().where(
        TaskEventRecord.task_id == TaskRecord.id, TaskEventRecord.event_type == "ALERT_ACKNOWLEDGED"
    )
    pickup_left = exists().where(
        TaskStepRecord.task_id == TaskRecord.id, TaskStepRecord.step_type == "CONFIRM_PICKUP",
        TaskStepRecord.status != StepStatus.SUCCEEDED.value,
    )
    started = tuple(status for status in _ORDER_HOLDING_STATUSES if status != TaskStatus.PENDING.value)
    return select(TaskRecord.receiving_zone, TaskRecord.id).where(
        TaskRecord.receiving_zone.is_not(None),
        TaskRecord.order_items.is_not(None),
        pickup_left,
        or_(
            TaskRecord.status.in_(started),
            (TaskRecord.status == TaskStatus.FAILED.value) & ~acknowledged,
        ),
    )


class PinkyUnavailable(RuntimeError):
    """Every Pinky (or the requested one) already carries an unfinished order."""


class ZoneUnavailable(RuntimeError):
    """The receiving zone is kept by another started order."""


class TaskStore:
    def __init__(self, db_path: str | None = None) -> None:
        self.path = Path(db_path) if db_path else database_path()
        self.engine: Engine = create_database_engine(self.path)
        self._sessions: sessionmaker = make_session_factory(self.engine)
        # The control station is a single local server process. Serialize task
        # claims in that process while the database CAS protects each task row.
        self._claim_lock = Lock()
        self._migrate()

    @property
    def session_factory(self) -> sessionmaker:
        """Sessions on the migrated task database, shared with the arm command queue."""
        return self._sessions

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
                if task.order_items is not None and task.pinky_id is None:
                    raise InvalidStateTransition("핑키가 배정되지 않은 주문입니다. 주문 접수 창에서 시작하세요")
                running = select(TaskRecord).where(
                    TaskRecord.status.in_(
                        [
                            TaskStatus.VALIDATING.value,
                            TaskStatus.RUNNING.value,
                            TaskStatus.CANCELING.value,
                            TaskStatus.NEEDS_INTERVENTION.value,
                        ]
                    )
                )
                if task.pinky_id is not None:
                    # Orders for different Pinkys run side by side (the loading spot is shared through the flow's
                    # station lock); a task without a Pinky still runs alone.
                    running = running.where(
                        (TaskRecord.pinky_id.is_(None)) | (TaskRecord.pinky_id == task.pinky_id)
                    )
                active = session.scalars(running).first()
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
            "order_items": task.order_items,
            "pinky_id": task.pinky_id,
            "receiving_zone": task.receiving_zone,
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
            "detail": step.detail,
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

    def reserved_units(self) -> Counter:
        """Units that open orders still need from the shelves: ordered minus already moved onto a Pinky.

        Waiting and running orders reserve; a finished, failed or canceled order releases what it did not take."""
        open_statuses = [TaskStatus.PENDING.value, TaskStatus.VALIDATING.value, TaskStatus.RUNNING.value,
                         TaskStatus.CANCELING.value, TaskStatus.NEEDS_INTERVENTION.value]
        reserved: Counter = Counter()
        with self._sessions() as session:
            orders = session.scalars(
                select(TaskRecord).where(TaskRecord.order_items.is_not(None), TaskRecord.status.in_(open_statuses))
            ).all()
            for order in orders:
                need = Counter({str(item["object_class"]): int(item["quantity"]) for item in order.order_items or []})
                moved = Counter(
                    str((step.detail or {}).get("object_class")) for step in order.steps
                    if step.step_type == "VERIFY_TRANSFER" and step.status == StepStatus.SUCCEEDED.value
                )
                reserved.update(need - moved)
        return reserved

    def confirmed_transfers(self) -> list[tuple[str, str, str]]:
        """(step id, shelf, object) of every unit the order flow confirmed moved from a shelf onto a Pinky."""
        with self._sessions() as session:
            steps = session.scalars(
                select(TaskStepRecord).where(TaskStepRecord.step_type == "VERIFY_TRANSFER",
                                             TaskStepRecord.status == StepStatus.SUCCEEDED.value)
            ).all()
            return [(step.id, str(step.detail["shelf_id"]), str(step.detail["object_class"]))
                    for step in steps if (step.detail or {}).get("shelf_id") and (step.detail or {}).get("object_class")]

    def pending_orders(self) -> list[dict[str, Any]]:
        """Orders waiting in the intake (not started yet), oldest first."""
        with self._sessions() as session:
            tasks = session.scalars(
                select(TaskRecord)
                .where(TaskRecord.order_items.is_not(None), TaskRecord.status == TaskStatus.PENDING.value)
                .order_by(TaskRecord.created_at, TaskRecord.id)
            ).all()
            return [self._present(task) for task in tasks]

    def recent_orders(self, limit: int = 12) -> list[dict[str, Any]]:
        """The latest orders (tasks with order_items) with their steps and events, newest first."""
        with self._sessions() as session:
            tasks = session.scalars(
                select(TaskRecord)
                .where(TaskRecord.order_items.is_not(None))
                .order_by(TaskRecord.created_at.desc(), TaskRecord.id.desc())
                .limit(limit)
            ).all()
            return [self._present_with_history(task) for task in tasks]

    def acknowledge_alert(self, task_id: str, note: str) -> dict[str, Any]:
        """Record that the operator saw a stopped order's alert; the task state does not change."""
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._sessions.begin() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            session.add(
                TaskEventRecord(
                    id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                    task_id=task_id,
                    step_id=None,
                    event_type="ALERT_ACKNOWLEDGED",
                    previous_status=task.status,
                    new_status=task.status,
                    occurred_at=now,
                    received_at=now,
                    payload={"source": "operator", "event": "ALERT_ACKNOWLEDGED", "note": note},
                )
            )
            session.flush()
            session.expire(task, ["events"])
            return self._present_with_history(task)

    def get_task(self, task_id: str) -> dict[str, Any] | None:
        with self._sessions() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                return None
            return self._present_with_history(task)

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

    def set_step_plan(self, task_id: str, step_id: str, robot_id: str | None, detail: dict[str, object]) -> None:
        """Fill in the planned arm and object unit of a step that has not started yet."""
        with self._sessions.begin() as session:
            step = session.get(TaskStepRecord, step_id)
            if step is None or step.task_id != task_id:
                raise StepNotFoundError(step_id)
            if step.status != StepStatus.PENDING.value:
                raise StateTransitionConflict(f"Expected step state PENDING, found {step.status}")
            step.robot_id = robot_id
            step.detail = detail

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

    def busy_pinkies(self) -> dict[str, str]:
        """Pinky ID -> the unfinished order holding it."""
        with self._sessions() as session:
            rows = session.execute(_pinky_holding_orders()).all()
            return {pinky_id: task_id for pinky_id, task_id in rows}

    def create_task(
        self,
        destination: str,
        arm_job: str,
        pick_request: dict[str, object] | None = None,
        order_items: list[dict[str, object]] | None = None,
        pinky_id: str | None = None,
        *,
        receiving_zone: str | None = None,
        defer_pinky: bool = False,
        zone_steps: bool | None = None,
        idempotency: tuple[str, str] | None = None,
        shipments: list[list[dict[str, object]]] | None = None,
    ) -> dict[str, Any]:
        """Store a task. An order (order_items) is given a free Pinky, or the requested one if it is free.

        With `defer_pinky` the order waits in the intake without a Pinky; assign_pinky gives it one when it starts,
        so the choice is made with the fleet state of that moment. An order gets the receiving-zone steps when it
        names its zone or with `zone_steps` (the zone is then given at start by assign_zone).

        `idempotency` is (Idempotency-Key, request hash): it is stored with the task in the same transaction, so
        a key is never left pointing at nothing and two requests with one key cannot both make a task."""
        created_at = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        task = TaskRecord(
            id=f"TASK-{uuid.uuid4().hex[:8].upper()}",
            destination=destination,
            arm_job=arm_job,
            pick_request=pick_request,
            order_items=order_items,
            pinky_id=None,
            receiving_zone=receiving_zone,
            status=TaskStatus.PENDING.value,
            created_at=created_at,
        )
        step_definitions: list[tuple[str, str | None, dict[str, object] | None]] = [
            (step_type, robot_id, None) for step_type, robot_id in DEFAULT_TASK_STEPS
        ]
        # The claim lock also serializes Pinky assignment inside this single server process.
        with self._claim_lock, self._sessions.begin() as session:
            if order_items is not None and not defer_pinky:
                busy = {pinky_id for pinky_id, _ in session.execute(_pinky_holding_orders()).all()}
                if pinky_id is not None:
                    if pinky_id in busy:
                        raise PinkyUnavailable(f"핑키 {pinky_id}는 끝나지 않은 주문이 있습니다")
                    task.pinky_id = pinky_id
                else:
                    free = [candidate for candidate in PINKY_IDS if candidate not in busy]
                    if not free:
                        raise PinkyUnavailable("모든 핑키에 끝나지 않은 주문이 있습니다")
                    task.pinky_id = free[0]
            if order_items is not None:
                # An order runs its own step list; its Pinky steps are tagged once the Pinky is known.
                with_zone = receiving_zone is not None if zone_steps is None else zone_steps
                step_definitions = order_step_definitions(order_items, task.pinky_id, receiving_zone=with_zone,
                                                          shipments=shipments)
            task.steps = [
                TaskStepRecord(
                    id=f"STEP-{uuid.uuid4().hex[:12].upper()}",
                    sequence=sequence,
                    step_type=step_type,
                    robot_id=robot_id,
                    status="PENDING",
                    detail=detail,
                )
                for sequence, (step_type, robot_id, detail) in enumerate(step_definitions, start=1)
            ]
            payload: dict[str, object] = {
                "step_ids": [step.id for step in task.steps],
                "pick_request": pick_request,
                "order_items": order_items,
            }
            if order_items is not None:
                payload["pinky_id"] = task.pinky_id
                payload["receiving_zone"] = receiving_zone
            task.events = [
                TaskEventRecord(
                    id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                    event_type="TASK_CREATED",
                    previous_status=None,
                    new_status=TaskStatus.PENDING.value,
                    occurred_at=created_at,
                    received_at=created_at,
                    payload=payload,
                )
            ]
            session.add(task)
            session.flush()  # the task row first: the key row points at it
            if idempotency is not None:
                key, request_hash = idempotency
                session.add(OrderRequestRecord(
                    idempotency_key=key, request_hash=request_hash, task_id=task.id, created_at=created_at))
                session.flush()
            return self._present_with_history(task)

    def order_request(self, idempotency_key: str) -> tuple[str, str] | None:
        """(request hash, order id) stored for an Idempotency-Key, or None when the key is new."""
        with self._sessions() as session:
            record = session.get(OrderRequestRecord, idempotency_key)
            return None if record is None else (record.request_hash, record.task_id)

    def assign_pinky(self, task_id: str, pinky_id: str, assignment: dict[str, object]) -> dict[str, Any]:
        """Give a waiting order its Pinky, tag the Pinky steps with it and record why it was chosen."""
        if pinky_id not in PINKY_IDS:
            raise ValueError(f"unknown Pinky {pinky_id}")
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._claim_lock, self._sessions.begin() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if task.order_items is None or task.status != TaskStatus.PENDING.value:
                raise InvalidStateTransition("접수 상태의 주문에만 핑키를 배정할 수 있습니다")
            if task.pinky_id is not None:
                raise InvalidStateTransition(f"이미 핑키 {task.pinky_id}가 배정된 주문입니다")
            holder = dict(session.execute(_pinky_holding_orders()).all()).get(pinky_id)
            if holder is not None:
                raise PinkyUnavailable(f"핑키 {pinky_id}는 끝나지 않은 주문 {holder}가 있습니다")
            task.pinky_id = pinky_id
            for step in task.steps:
                if step.step_type in PINKY_STEP_TYPES and shipment_of(step) == 1:  # later loads choose when they start
                    step.robot_id = pinky_robot_id(pinky_id)
            task.events.append(TaskEventRecord(
                id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                event_type="PINKY_ASSIGNED",
                previous_status=task.status,
                new_status=task.status,
                occurred_at=now,
                received_at=now,
                payload={"pinky_id": pinky_id, "assignment": assignment, "shipment": 1},
            ))
            session.flush()
            return self._present_with_history(task)

    def assign_shipment_pinky(self, task_id: str, shipment: int, pinky_id: str,
                              assignment: dict[str, object]) -> dict[str, Any]:
        """Give the next shipment of a running order its Pinky: the order now holds that Pinky (it held the previous
        one, which is parked again), its steps of that shipment are tagged with it, and the choice is recorded."""
        if pinky_id not in PINKY_IDS:
            raise ValueError(f"unknown Pinky {pinky_id}")
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._claim_lock, self._sessions.begin() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if task.order_items is None or task.status not in {TaskStatus.RUNNING.value, TaskStatus.VALIDATING.value}:
                raise InvalidStateTransition("진행 중인 주문의 다음 운송분에만 핑키를 배정할 수 있습니다")
            holder = dict(session.execute(_pinky_holding_orders()).all()).get(pinky_id)
            if holder is not None and holder != task_id:
                raise PinkyUnavailable(f"핑키 {pinky_id}는 끝나지 않은 주문 {holder}가 있습니다")
            task.pinky_id = pinky_id
            for step in task.steps:
                if step.step_type in PINKY_STEP_TYPES and shipment_of(step) == shipment:
                    step.robot_id = pinky_robot_id(pinky_id)
            task.events.append(TaskEventRecord(
                id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                event_type="PINKY_ASSIGNED",
                previous_status=task.status,
                new_status=task.status,
                occurred_at=now,
                received_at=now,
                payload={"pinky_id": pinky_id, "assignment": assignment, "shipment": shipment},
            ))
            session.flush()
            return self._present_with_history(task)

    def busy_zones(self) -> dict[str, str]:
        """Receiving zone ID -> the started order still using it (not yet picked up there)."""
        with self._sessions() as session:
            return {zone_id: task_id for zone_id, task_id in session.execute(_zone_holding_orders()).all()}

    def zone_usage(self, since: str) -> dict[str, int]:
        """Receiving zone ID -> orders started since `since` (ISO time) that went there."""
        with self._sessions() as session:
            rows = session.execute(
                select(TaskRecord.receiving_zone).where(
                    TaskRecord.receiving_zone.is_not(None), TaskRecord.order_items.is_not(None),
                    TaskRecord.started_at.is_not(None), TaskRecord.started_at >= since,
                )
            ).scalars().all()
        usage: dict[str, int] = {}
        for zone_id in rows:
            usage[zone_id] = usage.get(zone_id, 0) + 1
        return usage

    def assign_zone(self, task_id: str, zone_id: str, label: str, assignment: dict[str, object]) -> dict[str, Any]:
        """Give a waiting order its receiving zone (and the zone's name as its destination), recording why."""
        now = datetime.now(timezone.utc).isoformat(timespec="microseconds")
        with self._claim_lock, self._sessions.begin() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if task.order_items is None or task.status != TaskStatus.PENDING.value:
                raise InvalidStateTransition("접수 상태의 주문에만 수령 구역을 배정할 수 있습니다")
            if task.receiving_zone is not None:
                raise InvalidStateTransition(f"이미 수령 구역 {task.receiving_zone}이 정해진 주문입니다")
            holder = dict(session.execute(_zone_holding_orders()).all()).get(zone_id)
            if holder is not None:
                raise ZoneUnavailable(f"수령 구역({label})은 주문 {holder}이 사용 중입니다")
            task.receiving_zone = zone_id
            task.destination = label
            task.events.append(TaskEventRecord(
                id=f"EVT-{uuid.uuid4().hex[:12].upper()}",
                event_type="ZONE_ASSIGNED",
                previous_status=task.status,
                new_status=task.status,
                occurred_at=now,
                received_at=now,
                payload={"receiving_zone": zone_id, "label": label, "assignment": assignment},
            ))
            session.flush()
            return self._present_with_history(task)

    def pinky_usage(self, since: str) -> dict[str, dict[str, object]]:
        """Per Pinky: orders it started since `since` (ISO time) and when its last order ended."""
        usage: dict[str, dict[str, object]] = {pinky: {"trips": 0, "last_finished_at": None} for pinky in PINKY_IDS}
        with self._sessions() as session:
            rows = session.execute(
                select(TaskRecord.pinky_id, TaskRecord.started_at, TaskRecord.finished_at).where(
                    TaskRecord.pinky_id.is_not(None), TaskRecord.order_items.is_not(None)
                )
            ).all()
        for pinky_id, started_at, finished_at in rows:
            entry = usage.setdefault(pinky_id, {"trips": 0, "last_finished_at": None})
            if started_at is not None and started_at >= since:
                entry["trips"] = int(entry["trips"]) + 1  # type: ignore[call-overload]
            if finished_at is not None and (entry["last_finished_at"] is None or finished_at > entry["last_finished_at"]):
                entry["last_finished_at"] = finished_at
        return usage

    def pick_success_stats(self, *, simulated: bool) -> dict[tuple[str, str], tuple[int, int]]:
        """(policy_id, arm_id) -> (successes, attempts) from finished transfer checks.

        A check counts as mock or real by the pick right before it, so mock runs never shape the choice of a
        real arm. A failed check is an attempt without success."""
        finished = [StepStatus.SUCCEEDED.value, StepStatus.FAILED.value]
        stats: dict[tuple[str, str], tuple[int, int]] = {}
        with self._sessions() as session:
            steps = session.scalars(
                select(TaskStepRecord)
                .where(TaskStepRecord.step_type.in_(["PICK_ITEM", "VERIFY_TRANSFER"]))
                .order_by(TaskStepRecord.task_id, TaskStepRecord.sequence)
            ).all()
            picks = {(step.task_id, step.sequence): step for step in steps if step.step_type == "PICK_ITEM"}
            for check in steps:
                pick = picks.get((check.task_id, check.sequence - 1))
                if check.step_type != "VERIFY_TRANSFER" or check.status not in finished or pick is None:
                    continue
                detail = check.detail or {}
                if bool((pick.result or {}).get("simulated")) != simulated or not detail.get("policy_id"):
                    continue
                key = (str(detail["policy_id"]), str(detail.get("arm_id")))
                successes, attempts = stats.get(key, (0, 0))
                stats[key] = (successes + (check.status == StepStatus.SUCCEEDED.value), attempts + 1)
        return stats

    def close(self) -> None:
        self.engine.dispose()
