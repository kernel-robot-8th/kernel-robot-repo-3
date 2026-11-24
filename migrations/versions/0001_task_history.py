"""작업 단계(task_steps)와 이벤트 기록(task_events) 테이블을 추가하되, 이미 있던 이전 작업(tasks)은 보존한다.

첫 마이그레이션이다. 이것보다 먼저 Alembic 없이 tasks 테이블을 만들어 쓰던 DB가 있을 수 있어서, 테이블·열·인덱스를
만들기 전에 이미 있는지 먼저 확인(inspect)하고 없는 것만 만든다. 이미 있던 tasks에는 빠진 열만 nullable로 붙인다.
시각 열은 모두 ISO 8601 문자열이다.
"""

from __future__ import annotations

from alembic import op
import sqlalchemy as sa
from sqlalchemy import inspect


revision = "0001_task_history"
down_revision = None
branch_labels = None
depends_on = None


def _existing_tables() -> set[str]:
    """지금 연결된 DB에 있는 테이블 이름들."""
    return set(inspect(op.get_bind()).get_table_names())


def upgrade() -> None:
    tables = _existing_tables()
    if "tasks" not in tables:
        op.create_table(
            "tasks",
            sa.Column("id", sa.String(), primary_key=True),
            sa.Column("destination", sa.String(), nullable=False),
            sa.Column("arm_job", sa.String(), nullable=False),
            sa.Column("status", sa.String(), nullable=False),
            sa.Column("created_at", sa.String(), nullable=False),
            sa.Column("started_at", sa.String()),
            sa.Column("finished_at", sa.String()),
            sa.Column("active_step_id", sa.String()),
            sa.Column("error_code", sa.String()),
            sa.Column("error_message", sa.String()),
        )
    else:
        # 예전 tasks 테이블: 행은 그대로 두고 이 버전에서 생긴 열만 덧붙인다.
        existing_columns = {column["name"] for column in inspect(op.get_bind()).get_columns("tasks")}
        for name in ("started_at", "finished_at", "active_step_id", "error_code", "error_message"):
            if name not in existing_columns:
                op.add_column("tasks", sa.Column(name, sa.String(), nullable=True))

    # 앞에서 테이블을 만들었을 수 있으니 목록을 다시 읽는다.
    tables = _existing_tables()
    if "task_steps" not in tables:
        op.create_table(
            "task_steps",
            sa.Column("id", sa.String(), primary_key=True),
            sa.Column("task_id", sa.String(), sa.ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False),
            sa.Column("sequence", sa.Integer(), nullable=False),
            sa.Column("step_type", sa.String(), nullable=False),
            sa.Column("robot_id", sa.String()),
            sa.Column("status", sa.String(), nullable=False),
            sa.Column("command_id", sa.String()),
            sa.Column("started_at", sa.String()),
            sa.Column("finished_at", sa.String()),
            sa.Column("result_json", sa.JSON()),
            sa.Column("error_code", sa.String()),
            sa.Column("error_message", sa.String()),
            # 한 작업 안에서 단계 순번은 겹치지 않는다.
            sa.UniqueConstraint("task_id", "sequence", name="uq_task_steps_task_sequence"),
        )
    # 테이블이 이미 있었어도 인덱스는 없을 수 있어 따로 확인한다.
    if "idx_task_steps_task_sequence" not in {
        index["name"] for index in inspect(op.get_bind()).get_indexes("task_steps")
    }:
        op.create_index("idx_task_steps_task_sequence", "task_steps", ["task_id", "sequence"])

    tables = _existing_tables()
    if "task_events" not in tables:
        op.create_table(
            "task_events",
            sa.Column("id", sa.String(), primary_key=True),
            sa.Column("task_id", sa.String(), sa.ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False),
            # 단계가 지워져도 이벤트 기록은 남기고 단계 연결만 끊는다.
            sa.Column("step_id", sa.String(), sa.ForeignKey("task_steps.id", ondelete="SET NULL")),
            sa.Column("event_type", sa.String(), nullable=False),
            sa.Column("previous_status", sa.String()),
            sa.Column("new_status", sa.String()),
            sa.Column("occurred_at", sa.String(), nullable=False),
            sa.Column("received_at", sa.String(), nullable=False),
            sa.Column("payload_json", sa.JSON(), nullable=False),
        )
    # 작업별 이벤트를 받은 순서대로 읽는 인덱스. id는 받은 시각이 같을 때 순서를 고정한다.
    if "idx_task_events_task_received" not in {
        index["name"] for index in inspect(op.get_bind()).get_indexes("task_events")
    }:
        op.create_index("idx_task_events_task_received", "task_events", ["task_id", "received_at", "id"])


def downgrade() -> None:
    """이 버전이 만든 단계·이벤트 테이블과 인덱스만 지운다. tasks와 거기 붙인 열은 이전 데이터 보존을 위해 둔다."""
    op.drop_index("idx_task_events_task_received", table_name="task_events")
    op.drop_table("task_events")
    op.drop_index("idx_task_steps_task_sequence", table_name="task_steps")
    op.drop_table("task_steps")
