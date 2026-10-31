"""SQLAlchemy models and database setup for task history."""

from __future__ import annotations

import os
from pathlib import Path

from sqlalchemy import ForeignKey, Index, String, UniqueConstraint, create_engine, event
from sqlalchemy.engine import Engine, URL
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship, sessionmaker
from sqlalchemy.types import JSON


class Base(DeclarativeBase):
    pass


class TaskRecord(Base):
    __tablename__ = "tasks"

    id: Mapped[str] = mapped_column(String, primary_key=True)
    destination: Mapped[str] = mapped_column(String, nullable=False)
    arm_job: Mapped[str] = mapped_column(String, nullable=False)
    pick_request: Mapped[dict[str, object] | None] = mapped_column("pick_request_json", JSON)
    status: Mapped[str] = mapped_column(String, nullable=False)
    created_at: Mapped[str] = mapped_column(String, nullable=False)
    started_at: Mapped[str | None] = mapped_column(String)
    finished_at: Mapped[str | None] = mapped_column(String)
    active_step_id: Mapped[str | None] = mapped_column(String)
    error_code: Mapped[str | None] = mapped_column(String)
    error_message: Mapped[str | None] = mapped_column(String)

    steps: Mapped[list[TaskStepRecord]] = relationship(
        back_populates="task",
        cascade="all, delete-orphan",
        order_by="TaskStepRecord.sequence",
    )
    events: Mapped[list[TaskEventRecord]] = relationship(
        back_populates="task",
        cascade="all, delete-orphan",
        order_by="(TaskEventRecord.received_at, TaskEventRecord.id)",
    )


class TaskStepRecord(Base):
    __tablename__ = "task_steps"
    __table_args__ = (
        UniqueConstraint("task_id", "sequence", name="uq_task_steps_task_sequence"),
        Index("idx_task_steps_task_sequence", "task_id", "sequence"),
    )

    id: Mapped[str] = mapped_column(String, primary_key=True)
    task_id: Mapped[str] = mapped_column(ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False)
    sequence: Mapped[int] = mapped_column(nullable=False)
    step_type: Mapped[str] = mapped_column(String, nullable=False)
    robot_id: Mapped[str | None] = mapped_column(String)
    status: Mapped[str] = mapped_column(String, nullable=False)
    command_id: Mapped[str | None] = mapped_column(String)
    started_at: Mapped[str | None] = mapped_column(String)
    finished_at: Mapped[str | None] = mapped_column(String)
    result: Mapped[dict[str, object] | None] = mapped_column("result_json", JSON)
    error_code: Mapped[str | None] = mapped_column(String)
    error_message: Mapped[str | None] = mapped_column(String)

    task: Mapped[TaskRecord] = relationship(back_populates="steps")


class TaskEventRecord(Base):
    __tablename__ = "task_events"
    __table_args__ = (Index("idx_task_events_task_received", "task_id", "received_at", "id"),)

    id: Mapped[str] = mapped_column(String, primary_key=True)
    task_id: Mapped[str] = mapped_column(ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False)
    step_id: Mapped[str | None] = mapped_column(
        ForeignKey("task_steps.id", ondelete="SET NULL")
    )
    event_type: Mapped[str] = mapped_column(String, nullable=False)
    previous_status: Mapped[str | None] = mapped_column(String)
    new_status: Mapped[str | None] = mapped_column(String)
    occurred_at: Mapped[str] = mapped_column(String, nullable=False)
    received_at: Mapped[str] = mapped_column(String, nullable=False)
    payload: Mapped[dict[str, object]] = mapped_column("payload_json", JSON, nullable=False)

    task: Mapped[TaskRecord] = relationship(back_populates="events")


def database_path() -> Path:
    default_path = Path(__file__).resolve().parents[2] / "data" / "tasks.sqlite3"
    return Path(os.environ.get("CONTROL_STATION_DB", default_path))


def create_database_engine(path: Path) -> Engine:
    path.parent.mkdir(parents=True, exist_ok=True)
    engine = create_engine(
        URL.create("sqlite", database=str(path)),
        connect_args={"timeout": 5},
    )

    @event.listens_for(engine, "connect")
    def _enable_foreign_keys(dbapi_connection: object, _connection_record: object) -> None:
        cursor = dbapi_connection.cursor()  # type: ignore[attr-defined]
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.close()

    return engine


def make_session_factory(engine: Engine) -> sessionmaker:
    return sessionmaker(bind=engine, expire_on_commit=False)
