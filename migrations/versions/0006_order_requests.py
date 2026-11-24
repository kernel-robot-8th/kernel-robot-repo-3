"""주문 요청마다 받은 Idempotency-Key를 저장해, 다시 보낸 POST /api/orders가 이미 만든 주문을 돌려주게 한다.

응답이 오는 길에 사라져 주문 창이 같은 요청을 다시 보내도 주문이 두 개 생기지 않게 하려는 것이다.
"""

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
        # 요청 본문의 sha256. 같은 키에 다른 본문이 오면 거절하려고 둔다(IDEMPOTENCY_KEY_REUSED).
        sa.Column("request_hash", sa.String(), nullable=False),
        sa.Column("task_id", sa.String(), sa.ForeignKey("tasks.id", ondelete="CASCADE"), nullable=False),
        sa.Column("created_at", sa.String(), nullable=False),
    )


def downgrade() -> None:
    op.drop_table("order_requests")
