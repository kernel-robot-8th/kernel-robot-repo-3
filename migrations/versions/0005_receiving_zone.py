"""Where the customer collects an order (receiving zone id); None for tasks and older orders."""

from __future__ import annotations

from alembic import op
import sqlalchemy as sa


revision = "0005_receiving_zone"
down_revision = "0004_order_items"
branch_labels = None
depends_on = None


def upgrade() -> None:
    with op.batch_alter_table("tasks") as batch:
        batch.add_column(sa.Column("receiving_zone", sa.String(), nullable=True))


def downgrade() -> None:
    with op.batch_alter_table("tasks") as batch:
        batch.drop_column("receiving_zone")
