"""주문의 물체 목록(물체와 수량), 그 주문을 나를 핑키, 단계별 세부 정보를 저장한다.

- tasks.order_items_json: [{"object_class", "quantity"}]. 값이 있으면 그 작업은 주문이다.
- tasks.pinky_id: 핑키의 ROS 도메인 번호("10"/"11"/"12"). 접수만 되고 아직 배정 전이면 NULL.
- task_steps.detail_json: 단계가 다루는 물체 단위와 팔.
SQLite에서 열을 안전하게 붙이고 떼려고 batch_alter_table을 쓴다.
"""

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
        # 단계가 다루는 물체 단위와 팔(예: arm-01의 PICK_ITEM 당근 #1). 고정 단계는 None이다.
        batch.add_column(sa.Column("detail_json", sa.JSON(), nullable=True))


def downgrade() -> None:
    with op.batch_alter_table("task_steps") as batch:
        batch.drop_column("detail_json")
    with op.batch_alter_table("tasks") as batch:
        batch.drop_column("pinky_id")
        batch.drop_column("order_items_json")
