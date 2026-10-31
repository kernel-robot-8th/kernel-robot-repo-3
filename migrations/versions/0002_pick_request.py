"""Store an optional structured object request with each task."""

from __future__ import annotations

from alembic import op
import sqlalchemy as sa


revision = "0002_pick_request"
down_revision = "0001_task_history"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.add_column("tasks", sa.Column("pick_request_json", sa.JSON(), nullable=True))


def downgrade() -> None:
    op.drop_column("tasks", "pick_request_json")
