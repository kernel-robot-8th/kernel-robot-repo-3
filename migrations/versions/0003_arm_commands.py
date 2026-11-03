"""Store arm commands handed to an external arm worker and their lifecycle events."""

from __future__ import annotations

from alembic import op
import sqlalchemy as sa


revision = "0003_arm_commands"
down_revision = "0002_pick_request"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.create_table(
        "arm_commands",
        sa.Column("id", sa.String(), primary_key=True),
        sa.Column("robot_id", sa.String(), nullable=False),
        sa.Column("operation", sa.String(), nullable=False),
        sa.Column("state", sa.String(), nullable=False),
        sa.Column("request_json", sa.JSON(), nullable=False),
        sa.Column("created_at", sa.String(), nullable=False),
        sa.Column("valid_until", sa.String(), nullable=False),
        sa.Column("dispatched_at", sa.String()),
        sa.Column("worker_id", sa.String()),
        sa.Column("accepted_at", sa.String()),
        sa.Column("cancel_requested_at", sa.String()),
        sa.Column("finished_at", sa.String()),
        sa.Column("last_event_at", sa.String()),
        sa.Column("operator_cleared_at", sa.String()),
        sa.Column("reason_code", sa.String()),
        sa.Column("result_json", sa.JSON()),
    )
    op.create_index("idx_arm_commands_robot_created", "arm_commands", ["robot_id", "created_at"])
    op.create_table(
        "arm_command_events",
        sa.Column("id", sa.String(), primary_key=True),
        sa.Column(
            "command_id",
            sa.String(),
            sa.ForeignKey("arm_commands.id", ondelete="CASCADE"),
            nullable=False,
        ),
        sa.Column("worker_event_id", sa.String()),
        sa.Column("event_type", sa.String(), nullable=False),
        sa.Column("previous_state", sa.String()),
        sa.Column("new_state", sa.String()),
        sa.Column("occurred_at", sa.String()),
        sa.Column("received_at", sa.String(), nullable=False),
        sa.Column("payload_json", sa.JSON(), nullable=False),
        sa.UniqueConstraint(
            "command_id", "worker_event_id", name="uq_arm_command_events_worker_event"
        ),
    )
    op.create_index(
        "idx_arm_command_events_command_received",
        "arm_command_events",
        ["command_id", "received_at", "id"],
    )


def downgrade() -> None:
    op.drop_index("idx_arm_command_events_command_received", table_name="arm_command_events")
    op.drop_table("arm_command_events")
    op.drop_index("idx_arm_commands_robot_created", table_name="arm_commands")
    op.drop_table("arm_commands")
