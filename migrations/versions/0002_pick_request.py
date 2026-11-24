"""작업마다 구조화된 물체 요청(선택)을 저장한다: tasks.pick_request_json.

예: {"category": "box", "color": "red", "shelf_level": 2}. 이전 형식 작업을 만드는 POST /api/tasks의 pick_request
(api/common.validate_pick_request로 검사한 값)를 그대로 저장한다. 명령 문장 해석(orders/command_parser.py)이 이 모양의
초안을 만들고 운영자가 검토해 보낸다. 요청이 없는 작업은 NULL이다.
"""

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
