"""작업 기록용 SQLAlchemy 모델(테이블 정의)과 DB 설정.

TaskStore(store.py), 팔 명령 큐(arms/commands.py), 정책 연속 실행(arms/sequences.py)이 이 모델을 쓴다.
테이블은 create_all이 아니라 Alembic 마이그레이션(migrations/versions/)으로 만든다. 서버가 시작할 때
`alembic upgrade head`를 돌리므로(TaskStore._migrate), 여기 모델을 바꾸면 마이그레이션도 같이 추가해야 한다.

- 시각 열은 모두 ISO 8601 문자열(UTC)이다. 같은 형식이라 문자열 비교가 시간 순서와 같다.
- JSON 열의 실제 열 이름은 `*_json`이고 파이썬 속성은 접미사 없이 쓴다(pick_request ↔ pick_request_json).
"""

from __future__ import annotations

import os
from pathlib import Path

from sqlalchemy import ForeignKey, Index, String, UniqueConstraint, create_engine, event
from sqlalchemy.engine import URL, Engine
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship, sessionmaker
from sqlalchemy.types import JSON

from control_station import REPO_ROOT


class Base(DeclarativeBase):
    """모든 ORM 모델의 기반. migrations/env.py가 이 메타데이터를 마이그레이션 비교 대상으로 쓴다."""

    pass


class TaskRecord(Base):
    """작업 한 건(tasks). order_items가 있으면 주문이고, 없으면 이전 형식 작업(이동 + 팔 7단계)이다."""

    __tablename__ = "tasks"

    id: Mapped[str] = mapped_column(String, primary_key=True)
    destination: Mapped[str] = mapped_column(String, nullable=False)
    arm_job: Mapped[str] = mapped_column(String, nullable=False)
    pick_request: Mapped[dict[str, object] | None] = mapped_column("pick_request_json", JSON)
    # [{"object_class", "quantity"}] 목록. 주문이 여러 물체를 담기 전에 만든 작업은 None이다.
    order_items: Mapped[list[dict[str, object]] | None] = mapped_column("order_items_json", JSON)
    # 주문을 나르는 핑키의 ROS 도메인 번호("10"/"11"/"12"). 접수만 된 주문은 None. 큰 주문은 운송분마다 바뀐다.
    pinky_id: Mapped[str | None] = mapped_column(String)
    # 수령 구역 id(config/receiving-zones.json). 시작 때 배정되기 전이거나 이전 형식 작업이면 None.
    receiving_zone: Mapped[str | None] = mapped_column(String)
    status: Mapped[str] = mapped_column(String, nullable=False)
    created_at: Mapped[str] = mapped_column(String, nullable=False)
    started_at: Mapped[str | None] = mapped_column(String)
    finished_at: Mapped[str | None] = mapped_column(String)
    # 지금 명령을 보냈거나 실행 중인 단계. 단계와 단계 사이, 또는 끝난 뒤에는 None.
    active_step_id: Mapped[str | None] = mapped_column(String)
    error_code: Mapped[str | None] = mapped_column(String)
    error_message: Mapped[str | None] = mapped_column(String)

    steps: Mapped[list[TaskStepRecord]] = relationship(
        back_populates="task",
        cascade="all, delete-orphan",
        order_by="TaskStepRecord.sequence",
    )
    # 받은 시각 순. 시각이 같으면 id로 순서를 고정한다.
    events: Mapped[list[TaskEventRecord]] = relationship(
        back_populates="task",
        cascade="all, delete-orphan",
        order_by="(TaskEventRecord.received_at, TaskEventRecord.id)",
    )


class TaskStepRecord(Base):
    """작업의 단계 하나(task_steps). 같은 작업 안에서 sequence 순서대로 실행한다."""

    __tablename__ = "task_steps"
    __table_args__ = (
        UniqueConstraint("task_id", "sequence", name="uq_task_steps_task_sequence"),
        Index("idx_task_steps_task_sequence", "task_id", "sequence"),
    )

    id: Mapped[str] = mapped_column(String, primary_key=True)
    task_id: Mapped[str] = mapped_column(ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False)
    sequence: Mapped[int] = mapped_column(nullable=False)
    step_type: Mapped[str] = mapped_column(String, nullable=False)
    # 단계를 맡는 로봇(arm-01, pinky-10 등). 아직 정해지지 않았거나 관제 자신이 하는 단계는 None.
    robot_id: Mapped[str | None] = mapped_column(String)
    status: Mapped[str] = mapped_column(String, nullable=False)
    command_id: Mapped[str | None] = mapped_column(String)  # 이 단계를 실행할 때 만든 명령 id(CMD-...)
    started_at: Mapped[str | None] = mapped_column(String)
    finished_at: Mapped[str | None] = mapped_column(String)
    result: Mapped[dict[str, object] | None] = mapped_column("result_json", JSON)
    error_code: Mapped[str | None] = mapped_column(String)
    error_message: Mapped[str | None] = mapped_column(String)
    # 주문 단계에만 있다: 이 단계가 다루는 물체 단위와 계획된 팔.
    detail: Mapped[dict[str, object] | None] = mapped_column("detail_json", JSON)

    task: Mapped[TaskRecord] = relationship(back_populates="steps")


class TaskEventRecord(Base):
    """작업·단계에 일어난 일 하나(task_events). 상태 변화와 관측·신호 기록을 덧붙이기만 하고 고치지 않는다."""

    __tablename__ = "task_events"
    __table_args__ = (Index("idx_task_events_task_received", "task_id", "received_at", "id"),)

    id: Mapped[str] = mapped_column(String, primary_key=True)
    task_id: Mapped[str] = mapped_column(ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False)
    # 작업 전체에 대한 이벤트는 None. 단계가 지워져도 이벤트는 남는다(SET NULL).
    step_id: Mapped[str | None] = mapped_column(
        ForeignKey("task_steps.id", ondelete="SET NULL")
    )
    event_type: Mapped[str] = mapped_column(String, nullable=False)
    # 상태 변화 이벤트면 전·후 상태. 상태를 바꾸지 않는 기록(관측, 신호)은 둘이 같다.
    previous_status: Mapped[str | None] = mapped_column(String)
    new_status: Mapped[str | None] = mapped_column(String)
    occurred_at: Mapped[str] = mapped_column(String, nullable=False)  # 일이 일어난 시각
    received_at: Mapped[str] = mapped_column(String, nullable=False)  # 관제가 기록한 시각(정렬 기준)
    payload: Mapped[dict[str, object]] = mapped_column("payload_json", JSON, nullable=False)

    task: Mapped[TaskRecord] = relationship(back_populates="events")


class ArmCommandRecord(Base):
    """HTTP로 외부 워커(예: LeRobot OMX 노트북)에게 넘긴 팔 명령 하나. 상태 규칙은 arms/commands.py에 있다."""

    __tablename__ = "arm_commands"
    __table_args__ = (Index("idx_arm_commands_robot_created", "robot_id", "created_at"),)

    id: Mapped[str] = mapped_column(String, primary_key=True)
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    operation: Mapped[str] = mapped_column(String, nullable=False)
    state: Mapped[str] = mapped_column(String, nullable=False)
    request: Mapped[dict[str, object]] = mapped_column("request_json", JSON, nullable=False)
    created_at: Mapped[str] = mapped_column(String, nullable=False)
    valid_until: Mapped[str] = mapped_column(String, nullable=False)  # 이때까지 워커가 안 가져가면 EXPIRED
    dispatched_at: Mapped[str | None] = mapped_column(String)  # 워커가 가져간 시각
    worker_id: Mapped[str | None] = mapped_column(String)
    accepted_at: Mapped[str | None] = mapped_column(String)
    cancel_requested_at: Mapped[str | None] = mapped_column(String)
    finished_at: Mapped[str | None] = mapped_column(String)
    last_event_at: Mapped[str | None] = mapped_column(String)
    # UNKNOWN으로 끝난 명령을 운영자가 현장 확인 뒤 닫은 시각. 닫기 전에는 그 팔이 다음 명령을 받지 않는다.
    operator_cleared_at: Mapped[str | None] = mapped_column(String)
    reason_code: Mapped[str | None] = mapped_column(String)  # 그 상태가 된 이유(예: ACK_TIMEOUT, PLACE_NOT_VERIFIED)
    result: Mapped[dict[str, object] | None] = mapped_column("result_json", JSON)

    events: Mapped[list[ArmCommandEventRecord]] = relationship(
        back_populates="command",
        cascade="all, delete-orphan",
        order_by="(ArmCommandEventRecord.received_at, ArmCommandEventRecord.id)",
    )


class ArmCommandEventRecord(Base):
    """팔 명령 하나에 일어난 일(워커가 보낸 진행·결과, 서버가 정한 만료·시간 초과 등)."""

    __tablename__ = "arm_command_events"
    __table_args__ = (
        # 워커가 붙인 이벤트 id로 재시도한 POST를 한 번만 저장한다(멱등). 서버가 남기는 이벤트는 NULL로 둔다
        # (SQLite는 NULL끼리는 겹친다고 보지 않으므로 서버 이벤트끼리 부딪히지 않는다).
        UniqueConstraint("command_id", "worker_event_id", name="uq_arm_command_events_worker_event"),
        Index("idx_arm_command_events_command_received", "command_id", "received_at", "id"),
    )

    id: Mapped[str] = mapped_column(String, primary_key=True)
    command_id: Mapped[str] = mapped_column(
        ForeignKey("arm_commands.id", ondelete="CASCADE"), nullable=False
    )
    worker_event_id: Mapped[str | None] = mapped_column(String)
    event_type: Mapped[str] = mapped_column(String, nullable=False)
    previous_state: Mapped[str | None] = mapped_column(String)
    new_state: Mapped[str | None] = mapped_column(String)
    occurred_at: Mapped[str | None] = mapped_column(String)
    received_at: Mapped[str] = mapped_column(String, nullable=False)
    payload: Mapped[dict[str, object]] = mapped_column("payload_json", JSON, nullable=False)

    command: Mapped[ArmCommandRecord] = relationship(back_populates="events")


class OrderRequestRecord(Base):
    """주문 창이 POST /api/orders에 함께 보낸 Idempotency-Key. 다시 보낸 요청(응답이 오는 길에 사라진 경우)은
    두 번째 주문을 만드는 대신 이미 만든 주문을 돌려받는다."""
    __tablename__ = "order_requests"

    idempotency_key: Mapped[str] = mapped_column(String, primary_key=True)
    request_hash: Mapped[str] = mapped_column(String, nullable=False)  # 본문의 sha256. 같은 키의 다른 본문을 잡는다
    task_id: Mapped[str] = mapped_column(ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False)
    created_at: Mapped[str] = mapped_column(String, nullable=False)


class ArmSequenceRecord(Base):
    """운영자가 요청한 정책 집기 연속 실행. 서버를 다시 켜도 남는다(실행 규칙은 arms/sequences.py)."""

    __tablename__ = "arm_sequences"

    id: Mapped[str] = mapped_column(String, primary_key=True)
    sequence_key: Mapped[str] = mapped_column(String, nullable=False)
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    state: Mapped[str] = mapped_column(String, nullable=False)
    current_step: Mapped[int] = mapped_column(nullable=False)
    steps: Mapped[list[dict[str, object]]] = mapped_column("steps_json", JSON, nullable=False)
    results: Mapped[list[dict[str, object]]] = mapped_column("results_json", JSON, nullable=False)
    current_command_id: Mapped[str | None] = mapped_column(String)
    created_at: Mapped[str] = mapped_column(String, nullable=False)
    updated_at: Mapped[str] = mapped_column(String, nullable=False)


def database_path() -> Path:
    """DB 파일 위치: CONTROL_STATION_DB가 있으면 그 경로, 없으면 저장소의 data/tasks.sqlite3."""
    default_path = REPO_ROOT / "data" / "tasks.sqlite3"
    return Path(os.environ.get("CONTROL_STATION_DB", default_path))


def create_database_engine(path: Path) -> Engine:
    """SQLite 엔진을 만든다. 폴더가 없으면 만들고, 연결마다 외래 키 검사를 켠다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    engine = create_engine(
        URL.create("sqlite", database=str(path)),
        # 다른 연결이 쓰기 잠금을 쥐고 있으면 최대 5초 기다린 뒤 "database is locked"로 실패한다.
        connect_args={"timeout": 5},
    )

    @event.listens_for(engine, "connect")
    def _enable_foreign_keys(dbapi_connection: object, _connection_record: object) -> None:
        """SQLite는 연결마다 외래 키 검사가 꺼진 채로 시작한다. 켜야 ON DELETE CASCADE/SET NULL이 동작한다."""
        cursor = dbapi_connection.cursor()  # type: ignore[attr-defined]
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.close()

    return engine


def make_session_factory(engine: Engine) -> sessionmaker:
    """세션 팩토리. expire_on_commit=False라 커밋한 뒤에도 객체 속성을 DB에서 다시 읽지 않고 그대로 쓸 수 있다."""
    return sessionmaker(bind=engine, expire_on_commit=False)
