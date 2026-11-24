"""운영자가 요청한 정책 연속 실행(여러 ACT 정책을 차례로 돌리기)을 저장한다.

실행 규칙은 control_station/arms/sequences.py에 있다. 서버가 다시 켜져도 어느 단계까지 했는지 남는다.
"""

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
        sa.Column("sequence_key", sa.String(), nullable=False),  # 정의 파일(config/omx-act-sequences.json)의 sequence_id
        sa.Column("robot_id", sa.String(), nullable=False),
        sa.Column("state", sa.String(), nullable=False),
        sa.Column("current_step", sa.Integer(), nullable=False),  # 지금 하는 단계 번호(0부터)
        sa.Column("steps_json", sa.JSON(), nullable=False),  # 시작할 때 복사해 둔 단계 목록
        sa.Column("results_json", sa.JSON(), nullable=False),  # 끝난 단계마다 결과 하나씩
        sa.Column("current_command_id", sa.String()),  # 지금 단계의 팔 명령
        sa.Column("created_at", sa.String(), nullable=False),
        sa.Column("updated_at", sa.String(), nullable=False),
    )
    op.create_index("idx_arm_sequences_created", "arm_sequences", ["created_at"])


def downgrade() -> None:
    op.drop_index("idx_arm_sequences_created", table_name="arm_sequences")
    op.drop_table("arm_sequences")
