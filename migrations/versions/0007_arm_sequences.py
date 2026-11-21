"""Persist operator-requested sequential policy runs."""

from alembic import op
import sqlalchemy as sa


revision = "0007_arm_sequences"
down_revision = "0006_order_requests"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.create_table(
        "arm_sequences",
        sa.Column("id", sa.String(), primary_key=True),
        sa.Column("sequence_key", sa.String(), nullable=False),
        sa.Column("robot_id", sa.String(), nullable=False),
        sa.Column("state", sa.String(), nullable=False),
        sa.Column("current_step", sa.Integer(), nullable=False),
        sa.Column("steps_json", sa.JSON(), nullable=False),
        sa.Column("results_json", sa.JSON(), nullable=False),
        sa.Column("current_command_id", sa.String()),
        sa.Column("created_at", sa.String(), nullable=False),
        sa.Column("updated_at", sa.String(), nullable=False),
    )
    op.create_index("idx_arm_sequences_created", "arm_sequences", ["created_at"])


def downgrade() -> None:
    op.drop_index("idx_arm_sequences_created", table_name="arm_sequences")
    op.drop_table("arm_sequences")
