"""Add task steps and event history while preserving legacy tasks."""

from __future__ import annotations

from alembic import op
import sqlalchemy as sa
from sqlalchemy import inspect


revision = "0001_task_history"
down_revision = None
branch_labels = None
depends_on = None


def _existing_tables() -> set[str]:
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
        existing_columns = {column["name"] for column in inspect(op.get_bind()).get_columns("tasks")}
        for name in ("started_at", "finished_at", "active_step_id", "error_code", "error_message"):
            if name not in existing_columns:
                op.add_column("tasks", sa.Column(name, sa.String(), nullable=True))

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
            sa.UniqueConstraint("task_id", "sequence", name="uq_task_steps_task_sequence"),
        )
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
            sa.Column("step_id", sa.String(), sa.ForeignKey("task_steps.id", ondelete="SET NULL")),
            sa.Column("event_type", sa.String(), nullable=False),
            sa.Column("previous_status", sa.String()),
            sa.Column("new_status", sa.String()),
            sa.Column("occurred_at", sa.String(), nullable=False),
            sa.Column("received_at", sa.String(), nullable=False),
            sa.Column("payload_json", sa.JSON(), nullable=False),
        )
    if "idx_task_events_task_received" not in {
        index["name"] for index in inspect(op.get_bind()).get_indexes("task_events")
    }:
        op.create_index("idx_task_events_task_received", "task_events", ["task_id", "received_at", "id"])


def downgrade() -> None:
    op.drop_index("idx_task_events_task_received", table_name="task_events")
    op.drop_table("task_events")
    op.drop_index("idx_task_steps_task_sequence", table_name="task_steps")
    op.drop_table("task_steps")
