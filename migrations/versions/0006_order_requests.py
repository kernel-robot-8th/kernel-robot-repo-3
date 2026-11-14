"""Idempotency-Key of each order request, so a resent POST /api/orders returns the order it already made."""

from __future__ import annotations

from alembic import op
import sqlalchemy as sa


revision = "0006_order_requests"
down_revision = "0005_receiving_zone"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.create_table(
        "order_requests",
        sa.Column("idempotency_key", sa.String(), primary_key=True),
        sa.Column("request_hash", sa.String(), nullable=False),
        sa.Column("task_id", sa.String(), sa.ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False),
        sa.Column("created_at", sa.String(), nullable=False),
    )


def downgrade() -> None:
    op.drop_table("order_requests")
