"""서버가 만든 작업 기록을 SQLAlchemy(SQLite)로 저장한다: TaskStore.

TaskStore는 작업·주문을 만들고, 상태를 전이표(TASK_TRANSITIONS / STEP_TRANSITIONS)대로 바꾸고, 모든 변화를
task_events에 이벤트로 남긴다. 핑키·수령 구역 배정, 재고 예약, 집기 성공률 같은 집계도 여기서 DB를 읽어 만든다.
부르는 쪽: orders/task_runner.py(단계 실행), orders/desk.py(주문 접수·시작), orders/dispatcher.py(자동 출발),
orders/inventory.py(예약·옮김 장부), API. 반환값은 ORM 객체가 아니라 dict다(_present*).

동시성 가정:
- 관제는 서버 프로세스 하나다. 작업 시작(claim)과 핑키·구역 배정은 프로세스 안의 `_claim_lock`으로 줄 세운다.
- 상태 변경은 "기대 상태일 때만 UPDATE"(compare-and-set)로 하고 바뀐 행이 1개인지 확인한다. 다른 스레드가 먼저
  바꿨으면 StateTransitionConflict를 올리고, 부른 쪽이 다시 읽고 판단한다.
- 서버가 시작할 때 `alembic upgrade head`로 스키마를 최신으로 맞춘다(_migrate).
"""

from __future__ import annotations

import uuid
from collections import Counter
from datetime import datetime, timedelta, timezone
from pathlib import Path
from threading import Lock
from typing import Any

from alembic import command
from alembic.config import Config
from sqlalchemy import exists, or_, select, update
from sqlalchemy.engine import Engine
from sqlalchemy.orm import sessionmaker

from control_station import REPO_ROOT, utc_now_iso
from control_station.storage.database import (
    OrderRequestRecord,
    TaskEventRecord,
    TaskRecord,
    TaskStepRecord,
    create_database_engine,
    database_path,
    make_session_factory,
)
from control_station.storage.models import StepStatus, TaskStatus

# 이전 형식 작업(이동 + 팔 7단계)의 로봇 종류 → 로봇 id.
ROBOT_ID_BY_KIND = {"MOBILE": "mobile-01", "ARM": "arm-01"}
# 핑키 세 대를 ROS 도메인 번호로 부른다. 핑키 관제 화면의 이름과 같다(P0=10, P1=11, P2=12).
PINKY_IDS = ("10", "11", "12")
PINKY_NAMES = {"10": "P0", "11": "P1", "12": "P2"}
# 주문은 핑키를 받은 때부터 끝날 때까지 붙잡으므로, 접수 대기(PENDING) 주문도 핑키가 있으면 그 핑키를 쓰는 중으로
# 본다. 핑키 없이 접수된 주문(defer_pinky)은 pinky_id가 없어 아무것도 붙잡지 않는다.
_ORDER_HOLDING_STATUSES = (
    TaskStatus.PENDING.value,
    TaskStatus.VALIDATING.value,
    TaskStatus.RUNNING.value,
    TaskStatus.CANCELING.value,
    TaskStatus.NEEDS_INTERVENTION.value,
)


# 이전 형식 작업(order_items 없음)의 고정 7단계: (단계 종류, 로봇 id). task_runner가 모의로 돌린다.
DEFAULT_TASK_STEPS = (
    ("VALIDATE_ROBOTS", None),
    ("NAVIGATE_TO_DESTINATION", ROBOT_ID_BY_KIND["MOBILE"]),
    ("VERIFY_ARRIVAL", ROBOT_ID_BY_KIND["MOBILE"]),
    ("EXECUTE_ARM_TASK", ROBOT_ID_BY_KIND["ARM"]),
    ("VERIFY_ARM_RESULT", ROBOT_ID_BY_KIND["ARM"]),
    ("VERIFY_LOAD", None),
    ("FINALIZE_TASK", None),
)

# 주문의 핑키가 하는 단계. 그래서 그 핑키가 정해지면 robot_id에 그 핑키를 적는다.
PINKY_STEP_TYPES = frozenset({"WAIT_PINKY_ARRIVAL", "CONFIRM_DOCKING", "NOTIFY_PINKY_RECEIVED",
                              "WAIT_AT_RECEIVING_ZONE", "CONFIRM_PICKUP", "WAIT_PINKY_PARKED"})


def pinky_robot_id(pinky_id: str | None) -> str | None:
    """핑키 번호 → 단계의 robot_id("10" → "pinky-10"). None이면 None."""
    return None if pinky_id is None else f"pinky-{pinky_id}"


def order_step_definitions(
    order_items: list[dict[str, object]], pinky_id: str | None, *, receiving_zone: bool = False,
    shipments: list[list[dict[str, object]]] | None = None,
) -> list[tuple[str, str | None, dict[str, object] | None]]:
    """주문 하나의 단계 목록: (step_type, robot_id, detail). 물체 한 개(단위)마다 PICK_ITEM + VERIFY_TRANSFER
    한 쌍이 들어가고, 각 쌍의 팔은 선반 재고 확인 뒤 PLAN_ARM_SEQUENCE가 채운다. 아직 핑키가 없으면(접수 대기 주문)
    핑키 단계는 로봇 없이 두고, assign_pinky가 나중에 채운다.

    `shipments`는 큰 주문을 핑키 적재분으로 나눈 것이다(assignment.split_into_shipments의 단위 dict 목록). 운송분은
    차례로 돈다: 적재부터 주차 복귀까지의 단계가 운송분마다 되풀이되고 각각 {"shipment": k, "shipments": n} 표시가
    붙는다. k > 1인 운송분은 START_SHIPMENT로 시작하는데, 앞 핑키가 주차장에 돌아오기를 기다렸다가 이 적재분의
    핑키를 골라 출발시킨다. 한 번에 핑키 한 대만 달리고, 주문은 마지막 적재분을 수령할 때까지 수령 구역을 쥔다.
    나눈 주문에는 수령 구역 단계가 꼭 있어야 한다(적재분마다 거기서 수령하므로). 없으면 ValueError."""
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
        robot = pinky if number == 1 else None  # 두 번째 운송분부터는 시작할 때 핑키를 정한다
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
            # 핑키가 짐을 수령 구역으로 싣고 가서 손님을 기다린 뒤 주차장으로 돌아온다.
            steps += [("WAIT_AT_RECEIVING_ZONE", robot, _tagged(None, tag)), ("CONFIRM_PICKUP", robot, _tagged(None, tag)),
                      ("WAIT_PINKY_PARKED", robot, _tagged(None, tag))]
    steps.append(("FINALIZE_TASK", None, None))
    return steps


def _tagged(detail: dict[str, object] | None, tag: dict[str, object] | None) -> dict[str, object] | None:
    """단계 detail에 운송분 표시를 합친다. 둘 다 없으면 None."""
    if detail is None and tag is None:
        return None
    return {**(detail or {}), **(tag or {})}


def shipment_of(step: dict[str, Any] | Any) -> int:
    """단계가 속한 운송분 번호(1, 2, ...). 나누지 않은 주문과 공통 단계(재고 확인·계획·마무리)는 1이다.

    단계 dict와 TaskStepRecord 둘 다 받는다."""
    detail = step.get("detail") if isinstance(step, dict) else getattr(step, "detail", None)
    return int((detail or {}).get("shipment") or 1)


# 작업 상태 전이표: 지금 상태 → 갈 수 있는 상태. 끝 상태(SUCCEEDED/FAILED/CANCELED)에서는 나갈 수 없다.
# NEEDS_INTERVENTION → RUNNING은 운영자 확인(resume_confirmed)이 있어야 한다(transition_task).
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

# 단계 상태 전이표. TIMED_OUT·UNKNOWN은 결과를 확정하지 못한 상태라, 나중에 확인한 결과로 닫을 수 있다.
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
    """없는 작업 id."""

    pass


class StepNotFoundError(LookupError):
    """없는 단계 id이거나 다른 작업의 단계."""

    pass


class InvalidStateTransition(ValueError):
    """전이표가 허용하지 않는 상태 변경이거나, 지금 상태에서는 할 수 없는 요청."""

    pass


class StateTransitionConflict(InvalidStateTransition):
    """기대한 상태가 아니다: 다른 요청·스레드가 먼저 바꿨다. 다시 읽고 판단해야 한다."""

    pass


def _pinky_holding_orders():  # type: ignore[no-untyped-def]
    """핑키를 붙잡고 있는 주문들을 고르는 쿼리: (pinky_id, task id). 끝나지 않은 주문, 그리고 운영자가 경보를
    확인하기 전의 멈춘(FAILED) 주문이다. 멈춘 주문의 핑키는 아직 적재 자리에 서 있을 수 있기 때문이다."""
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
    """수령 구역을 붙잡고 있는 주문들을 고르는 쿼리: (receiving_zone, task id). 시작했고 아직 거기서 수령되지
    않은 주문이다(CONFIRM_PICKUP이 하나라도 남음. 나눈 주문은 운송분 사이에도 구역을 쥔다). 접수 대기(PENDING)
    주문은 구역을 쥐지 않는다. 멈춘(FAILED) 주문은 핑키처럼 운영자가 경보를 확인할 때까지 쥔다."""
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
    """모든 핑키(또는 요청한 핑키)에 이미 끝나지 않은 주문이 있다."""


class ZoneUnavailable(RuntimeError):
    """그 수령 구역을 이미 시작한 다른 주문이 쓰고 있다."""


class TaskStore:
    """작업·단계·이벤트 저장소. 만들 때 DB 파일을 열고 스키마를 최신으로 올린다(db_path가 없으면 database_path())."""

    def __init__(self, db_path: str | None = None) -> None:
        self.path = Path(db_path) if db_path else database_path()
        self.engine: Engine = create_database_engine(self.path)
        self._sessions: sessionmaker = make_session_factory(self.engine)
        # 관제는 로컬 서버 프로세스 하나다. 작업 선점(claim)은 이 프로세스 안에서 락으로 줄 세우고,
        # 각 작업 행은 DB의 CAS(기대 상태일 때만 UPDATE)가 지킨다.
        self._claim_lock = Lock()
        self._migrate()

    @property
    def session_factory(self) -> sessionmaker:
        """마이그레이션을 마친 작업 DB의 세션 팩토리. 팔 명령 큐와 같이 쓴다."""
        return self._sessions

    def claim_task(self, task_id: str) -> tuple[dict[str, Any], bool]:
        """접수 대기(PENDING) 작업 하나를 이 서버 프로세스가 원자적으로 선점해 VALIDATING으로 바꾼다.

        (task, started)를 돌려준다. 이미 진행 중인 작업을 다시 시작하라는 요청은 그 작업을 그대로 돌려준다(멱등,
        started=False). 겹치는 다른 작업이 진행 중이면 새로 선점하지 않는다(StateTransitionConflict).
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
                # 핑키 없이 접수된 주문은 /api/tasks/{id}/start로 시작할 수 없다: 주문 접수 창(OrderDesk.start)이
                # 핑키를 정한 뒤 시작한다.
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
                    # 핑키가 다른 주문끼리는 나란히 돈다(적재 자리는 주문 흐름의 station 잠금으로 나눠 쓴다).
                    # 핑키가 없는 작업(이전 형식)과 같은 핑키의 작업만 막는다. 핑키가 없는 작업은 여전히 혼자 돈다.
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
        """시작하지 않은 작업과 그 대기 단계를 한 트랜잭션으로 모두 취소한다.

        이벤트 순서가 단계 순서와 같도록 단계마다 1µs씩 다른 시각을 붙이고, 작업의 CANCELED 이벤트는 그 뒤 시각에 둔다."""
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
            # PENDING 작업인데 이미 움직인 단계가 있으면 상태가 꼬인 것이니 자동으로 취소하지 않고 운영자에게 넘긴다.
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
        """`alembic upgrade head`로 이 DB의 스키마를 최신으로 맞춘다. alembic.ini의 URL은 이 DB 경로로 바꿔 넣는다."""
        project_root = REPO_ROOT
        config = Config(str(project_root / "alembic.ini"))
        database_url = self.engine.url.render_as_string(hide_password=False)
        # alembic.ini는 ConfigParser로 읽혀 %를 보간 문자로 보므로, 경로의 %를 %%로 이스케이프한다.
        config.set_main_option("sqlalchemy.url", database_url.replace("%", "%%"))
        command.upgrade(config, "head")

    @staticmethod
    def _present(task: TaskRecord) -> dict[str, Any]:
        """작업 행 → API dict(단계·이벤트 없이). error는 코드나 메시지가 있을 때만 채운다."""
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
        """단계 행 → API dict. 이전 형식 작업의 로봇 종류("MOBILE"/"ARM")는 로봇 id로 바꿔 보여 준다."""
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
        """이벤트 행 → API dict."""
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
        """작업 dict에 단계(순번 순)와 이벤트(받은 순)를 붙인다."""
        result = cls._present(task)
        result["steps"] = [cls._present_step(step) for step in task.steps]
        result["events"] = [cls._present_event(event) for event in task.events]
        return result

    def list_tasks(self) -> list[dict[str, Any]]:
        """모든 작업(단계·이벤트 없이), 최근에 만든 것부터."""
        with self._sessions() as session:
            tasks = session.scalars(
                select(TaskRecord).order_by(TaskRecord.created_at.desc(), TaskRecord.id.desc())
            ).all()
            return [self._present(task) for task in tasks]

    def reserved_units(self) -> Counter:
        """열린 주문이 아직 선반에서 가져가야 하는 개수(물체별): 주문 수량 − 이미 핑키 위로 옮긴 수량.

        대기 중이거나 진행 중인 주문이 예약한다. 끝났거나 멈췄거나 취소된 주문은 가져가지 않은 몫을 풀어 준다.
        재고 장부(orders/inventory.py)가 '팔 수 있는 수량 = 선반 − 예약'을 계산할 때 쓴다."""
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
        """주문 흐름이 선반에서 핑키 위로 옮겼다고 확인한 모든 단위의 (단계 id, 선반, 물체).

        재고 장부가 단계 id로 아직 반영하지 않은 옮김을 골라 선반 개수를 바로 하나씩 줄인다."""
        with self._sessions() as session:
            steps = session.scalars(
                select(TaskStepRecord).where(TaskStepRecord.step_type == "VERIFY_TRANSFER",
                                             TaskStepRecord.status == StepStatus.SUCCEEDED.value)
            ).all()
            return [(step.id, str(step.detail["shelf_id"]), str(step.detail["object_class"]))
                    for step in steps if (step.detail or {}).get("shelf_id") and (step.detail or {}).get("object_class")]

    def pending_orders(self) -> list[dict[str, Any]]:
        """접수되어 시작을 기다리는 주문들, 오래된 것부터."""
        with self._sessions() as session:
            tasks = session.scalars(
                select(TaskRecord)
                .where(TaskRecord.order_items.is_not(None), TaskRecord.status == TaskStatus.PENDING.value)
                .order_by(TaskRecord.created_at, TaskRecord.id)
            ).all()
            return [self._present(task) for task in tasks]

    def running_orders(self) -> list[dict[str, Any]]:
        """시작했고 아직 끝나지 않은 주문들과 그 단계, 먼저 시작한 것부터. 자동 출발(dispatcher)이 이 중 어느
        주문의 핑키가 아직 적재 자리로 가는 중인지 읽는다."""
        with self._sessions() as session:
            tasks = session.scalars(
                select(TaskRecord)
                .where(TaskRecord.order_items.is_not(None),
                       TaskRecord.status.in_((TaskStatus.VALIDATING.value, TaskStatus.RUNNING.value)))
                .order_by(TaskRecord.started_at, TaskRecord.id)
            ).all()
            return [{**self._present(task), "steps": [self._present_step(step) for step in task.steps]}
                    for task in tasks]

    def recent_orders(self, limit: int = 12) -> list[dict[str, Any]]:
        """최근 주문(order_items가 있는 작업)을 단계·이벤트와 함께, 최신부터 limit개."""
        with self._sessions() as session:
            tasks = session.scalars(
                select(TaskRecord)
                .where(TaskRecord.order_items.is_not(None))
                .order_by(TaskRecord.created_at.desc(), TaskRecord.id.desc())
                .limit(limit)
            ).all()
            return [self._present_with_history(task) for task in tasks]

    def acknowledge_alert(self, task_id: str, note: str) -> dict[str, Any]:
        """운영자가 멈춘 주문의 경보를 확인했다고 기록한다(ALERT_ACKNOWLEDGED). 작업 상태는 바꾸지 않는다.

        이 이벤트가 생기면 그 주문은 더 이상 핑키·수령 구역을 붙잡지 않는다(_pinky_holding_orders)."""
        now = utc_now_iso("microseconds")
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
        """작업 하나를 단계·이벤트와 함께. 없으면 None."""
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
        """같은 작업·단계·명령이 아직 활성일 때만 근거(이벤트)를 덧붙인다. 상태는 바꾸지 않는다.

        작업이 expected_task_status가 아니거나, 그 단계가 더 이상 활성 단계가 아니거나, 단계 상태·명령 id가 바뀌었으면
        StateTransitionConflict다(취소된 뒤 늦게 온 기록이 끼어들지 않게)."""
        now = utc_now_iso("microseconds")
        with self._sessions.begin() as session:
            # 상태를 같은 값으로 UPDATE한다. 값은 그대로지만, 기대 상태인지 확인하면서 SQLite 쓰기 잠금을 먼저 잡아
            # 아래 확인과 기록 사이에 다른 쓰기가 끼지 못하게 한다.
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
        """아직 시작하지 않은(PENDING) 단계에 계획된 팔과 물체 단위를 채운다(PLAN_ARM_SEQUENCE가 부른다)."""
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
        """작업 상태를 바꾸고 TASK_STATUS_CHANGED 이벤트를 남긴다.

        expected_status를 주면 그 상태일 때만 바꾼다(아니면 StateTransitionConflict). 전이표에 없는 변경은
        InvalidStateTransition이고, NEEDS_INTERVENTION → RUNNING은 resume_confirmed=True(운영자 확인)일 때만 된다.
        처음 VALIDATING이 될 때 started_at, 끝 상태가 될 때 finished_at을 적는다. FAILED/NEEDS_INTERVENTION이면
        payload의 error_code/error를 작업 오류로 남기고, 운영자 확인으로 다시 RUNNING이 되면 오류를 지운다."""
        now = utc_now_iso("microseconds")
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

            # 읽은 뒤 다른 쪽이 상태를 바꿨다면 바뀐 행이 0개가 된다(compare-and-set).
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
        """단계 상태를 바꾸고 STEP_STATUS_CHANGED 이벤트를 남긴다. 규칙은 transition_task와 같다(전이표, CAS).

        명령을 보낸 때부터 started_at, 끝 상태(TIMED_OUT 포함)가 되면 finished_at을 적는다. 명령을 보냈거나 실행 중인
        단계는 작업의 활성 단계(active_step_id)가 되고, 그 단계가 끝나면 비운다."""
        now = utc_now_iso("microseconds")
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
        """핑키 번호 → 그 핑키를 붙잡고 있는 주문(끝나지 않았거나 경보 확인 전의 멈춘 주문)."""
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
        """작업을 저장한다. 주문(order_items)은 빈 핑키를, 또는 요청한 핑키가 비어 있으면 그 핑키를 받는다.

        `defer_pinky`면 주문은 핑키 없이 접수 상태로 기다리고, 시작할 때 assign_pinky가 핑키를 준다. 그래서 그때의
        핑키 상태로 고른다. 주문이 수령 구역을 지정했거나 `zone_steps`를 주면 수령 구역 단계가 들어간다(구역은
        시작할 때 assign_zone이 정한다).

        `idempotency`는 (Idempotency-Key, 요청 해시)다. 작업과 같은 트랜잭션에 저장하므로, 키가 없는 작업을 가리키는
        일이 없고 같은 키의 두 요청이 둘 다 작업을 만들 수도 없다(키가 기본 키라 두 번째는 실패한다)."""
        created_at = utc_now_iso("microseconds")
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
        # 선점 락은 이 서버 프로세스 안의 핑키 배정도 줄 세운다(빈 핑키 확인과 배정 사이에 다른 배정이 못 낀다).
        with self._claim_lock, self._sessions.begin() as session:
            if order_items is not None and not defer_pinky:
                busy = {pinky_id for pinky_id, _ in session.execute(_pinky_holding_orders()).all()}
                if pinky_id is not None:
                    if pinky_id in busy:
                        raise PinkyUnavailable(f"핑키 {pinky_id}는 끝나지 않은 주문이 있습니다")
                    task.pinky_id = pinky_id
                else:
                    # 이전 경로(POST /api/tasks): 배정 규칙 없이 번호가 앞선 빈 핑키를 준다.
                    free = [candidate for candidate in PINKY_IDS if candidate not in busy]
                    if not free:
                        raise PinkyUnavailable("모든 핑키에 끝나지 않은 주문이 있습니다")
                    task.pinky_id = free[0]
            if order_items is not None:
                # 주문은 자기 단계 목록으로 돈다. 핑키 단계에는 핑키가 정해진 뒤 그 핑키를 적는다.
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
            session.flush()  # 작업 행을 먼저 쓴다: 키 행이 그 행을 가리킨다
            if idempotency is not None:
                key, request_hash = idempotency
                session.add(OrderRequestRecord(
                    idempotency_key=key, request_hash=request_hash, task_id=task.id, created_at=created_at))
                session.flush()
            return self._present_with_history(task)

    def order_request(self, idempotency_key: str) -> tuple[str, str] | None:
        """Idempotency-Key에 저장된 (요청 해시, 주문 id). 처음 보는 키면 None."""
        with self._sessions() as session:
            record = session.get(OrderRequestRecord, idempotency_key)
            return None if record is None else (record.request_hash, record.task_id)

    def assign_pinky(self, task_id: str, pinky_id: str, assignment: dict[str, object]) -> dict[str, Any]:
        """접수 대기 주문에 핑키를 주고, 첫 운송분의 핑키 단계에 그 핑키를 적고, 고른 이유를 PINKY_ASSIGNED로 남긴다.

        그 사이 다른 주문이 그 핑키를 가져갔으면 PinkyUnavailable."""
        if pinky_id not in PINKY_IDS:
            raise ValueError(f"unknown Pinky {pinky_id}")
        now = utc_now_iso("microseconds")
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
                if step.step_type in PINKY_STEP_TYPES and shipment_of(step) == 1:  # 뒤 운송분은 시작할 때 고른다
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
        """진행 중인 주문의 다음 운송분에 핑키를 준다. 주문은 이제 그 핑키를 쥔다(앞 운송분의 핑키는 주차장에 돌아와
        있다). 그 운송분의 핑키 단계에 핑키를 적고, 고른 이유를 PINKY_ASSIGNED(shipment 포함)로 남긴다."""
        if pinky_id not in PINKY_IDS:
            raise ValueError(f"unknown Pinky {pinky_id}")
        now = utc_now_iso("microseconds")
        with self._claim_lock, self._sessions.begin() as session:
            task = session.get(TaskRecord, task_id)
            if task is None:
                raise TaskNotFoundError(task_id)
            if task.order_items is None or task.status not in {TaskStatus.RUNNING.value, TaskStatus.VALIDATING.value}:
                raise InvalidStateTransition("진행 중인 주문의 다음 운송분에만 핑키를 배정할 수 있습니다")
            holder = dict(session.execute(_pinky_holding_orders()).all()).get(pinky_id)
            # 이 주문 자신이 쥔 핑키(앞 운송분의 핑키)는 다시 보내도 된다.
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
        """수령 구역 id → 그 구역을 아직 쓰는(거기서 수령되지 않은) 시작한 주문."""
        with self._sessions() as session:
            return {zone_id: task_id for zone_id, task_id in session.execute(_zone_holding_orders()).all()}

    def zone_usage(self, since: str) -> dict[str, int]:
        """수령 구역 id → `since`(ISO 시각) 이후 시작해 그 구역으로 간 주문 수. ISO 문자열이라 문자열 비교로 거른다."""
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
        """접수 대기 주문에 수령 구역을 주고(목적지 글자도 구역 이름으로 바꾼다), 이유를 ZONE_ASSIGNED로 남긴다.

        그 사이 다른 주문이 그 구역에서 시작했으면 ZoneUnavailable."""
        now = utc_now_iso("microseconds")
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
        """핑키마다: `since`(ISO 시각) 이후 시작한 운행 횟수와 마지막 운행이 끝난 시각.

        운행 하나는 PINKY_ASSIGNED 이벤트 하나다. 큰 주문은 운송분마다 하나씩 보내고 핑키가 다를 수도 있다. 운송분의
        운행은 다음 운송분이 핑키를 가져갈 때 끝나고, 마지막 운송분은 주문이 끝날 때 끝난다. 이 이벤트가 생기기 전에
        만든 작업(만들 때 핑키를 준 경우)은 작업 자신의 시작·끝으로 한 번 센다. 배정 규칙(오늘 덜 달린 핑키, 오래 쉰
        핑키 먼저)이 이 값을 쓴다."""
        usage: dict[str, dict[str, object]] = {pinky: {"trips": 0, "last_finished_at": None} for pinky in PINKY_IDS}

        def count(pinky_id: str, started_at: str | None, ended_at: str | None) -> None:
            entry = usage.setdefault(pinky_id, {"trips": 0, "last_finished_at": None})
            if started_at is not None and started_at >= since:
                entry["trips"] = int(entry["trips"]) + 1  # type: ignore[call-overload]
            if ended_at is not None and (entry["last_finished_at"] is None or ended_at > entry["last_finished_at"]):
                entry["last_finished_at"] = ended_at

        with self._sessions() as session:
            tasks = session.execute(
                select(TaskRecord.id, TaskRecord.pinky_id, TaskRecord.started_at, TaskRecord.finished_at).where(
                    TaskRecord.pinky_id.is_not(None), TaskRecord.order_items.is_not(None)
                )
            ).all()
            assigned: dict[str, list[tuple[str, str]]] = {}
            for task_id, payload, occurred_at in session.execute(
                select(TaskEventRecord.task_id, TaskEventRecord.payload, TaskEventRecord.occurred_at)
                .where(TaskEventRecord.event_type == "PINKY_ASSIGNED")
                .order_by(TaskEventRecord.occurred_at, TaskEventRecord.id)
            ).all():
                pinky_id = (payload or {}).get("pinky_id")
                if pinky_id:
                    assigned.setdefault(task_id, []).append((str(pinky_id), occurred_at))
        for task_id, pinky_id, started_at, finished_at in tasks:
            trips = assigned.get(task_id)
            if not trips:
                count(pinky_id, started_at, finished_at)
                continue
            for index, (trip_pinky, trip_started) in enumerate(trips):
                ended = trips[index + 1][1] if index + 1 < len(trips) else finished_at
                count(trip_pinky, trip_started, ended)
        return usage

    def pick_success_stats(self, *, simulated: bool) -> dict[tuple[str, str], tuple[int, int]]:
        """(policy_id, arm_id) → (성공 수, 시도 수). 끝난 옮김 확인(VERIFY_TRANSFER)에서 센다.

        확인 하나가 모의인지 실제인지는 바로 앞 집기(PICK_ITEM) 결과의 simulated로 정한다. 그래서 모의 실행이 실제
        팔을 고르는 데 끼어들지 않는다. 실패한 확인은 성공 없는 시도 하나다."""
        finished = [StepStatus.SUCCEEDED.value, StepStatus.FAILED.value]
        stats: dict[tuple[str, str], tuple[int, int]] = {}
        with self._sessions() as session:
            steps = session.scalars(
                select(TaskStepRecord)
                .where(TaskStepRecord.step_type.in_(["PICK_ITEM", "VERIFY_TRANSFER"]))
                .order_by(TaskStepRecord.task_id, TaskStepRecord.sequence)
            ).all()
            # PICK_ITEM 바로 다음 순번이 그 단위의 VERIFY_TRANSFER다(order_step_definitions가 쌍으로 만든다).
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
        """엔진의 연결을 모두 닫는다."""
        self.engine.dispose()
