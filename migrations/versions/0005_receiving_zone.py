"""손님이 주문을 받는 곳(수령 구역 id, config/receiving-zones.json의 zone_id). 이전 형식 작업과 예전 주문은 None이다.

2026-11-14 결정 이후로는 주문을 시작할 때 서버가 정한다(orders/assignment.py의 choose_zone).
"""

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
