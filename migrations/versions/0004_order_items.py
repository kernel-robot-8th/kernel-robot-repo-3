"""Store an order's items (object and quantity), the Pinky assigned to carry it, and per-step detail."""

from __future__ import annotations

from alembic import op
import sqlalchemy as sa


revision = "0004_order_items"
down_revision = "0003_arm_commands"
branch_labels = None
depends_on = None


def upgrade() -> None:
    with op.batch_alter_table("tasks") as batch:
        batch.add_column(sa.Column("order_items_json", sa.JSON(), nullable=True))
        batch.add_column(sa.Column("pinky_id", sa.String(), nullable=True))
    with op.batch_alter_table("task_steps") as batch:
        # Which object unit and arm a step handles (PICK_ITEM carrot #1 on arm-01); None for fixed steps.
        batch.add_column(sa.Column("detail_json", sa.JSON(), nullable=True))


def downgrade() -> None:
    with op.batch_alter_table("task_steps") as batch:
        batch.drop_column("detail_json")
    with op.batch_alter_table("tasks") as batch:
        batch.drop_column("pinky_id")
        batch.drop_column("order_items_json")
