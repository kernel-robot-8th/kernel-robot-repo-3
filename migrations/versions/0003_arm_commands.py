"""외부 팔 워커에게 넘긴 팔 명령(arm_commands)과 그 명령의 생애 이벤트(arm_command_events)를 저장한다.

팔 명령은 관제가 큐에 넣고(QUEUED) 워커(예: LeRobot OMX 노트북)가 HTTP로 가져가 실행한다. 상태 규칙은
control_station/arms/commands.py에 있고, 여기서는 그 상태와 시각, 근거를 남길 자리만 만든다.
"""

from __future__ import annotations

from alembic import op
import sqlalchemy as sa


revision = "0003_arm_commands"
down_revision = "0002_pick_request"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.create_table(
        "arm_commands",
        sa.Column("id", sa.String(), primary_key=True),
        sa.Column("robot_id", sa.String(), nullable=False),
        sa.Column("operation", sa.String(), nullable=False),
        sa.Column("state", sa.String(), nullable=False),
        sa.Column("request_json", sa.JSON(), nullable=False),
        sa.Column("created_at", sa.String(), nullable=False),
        # 이 시각까지 워커가 가져가지 않으면 명령은 EXPIRED가 된다.
        sa.Column("valid_until", sa.String(), nullable=False),
        sa.Column("dispatched_at", sa.String()),  # 워커가 가져간 시각
        sa.Column("worker_id", sa.String()),  # 가져간 워커
        sa.Column("accepted_at", sa.String()),
        sa.Column("cancel_requested_at", sa.String()),
        sa.Column("finished_at", sa.String()),
        sa.Column("last_event_at", sa.String()),  # 워커가 마지막으로 이벤트를 보낸 시각
        # 결과가 UNKNOWN인 명령을 운영자가 현장 확인 뒤 닫은 시각. 닫기 전에는 그 팔이 다음 명령을 받지 않는다.
        sa.Column("operator_cleared_at", sa.String()),
        sa.Column("reason_code", sa.String()),  # 왜 그 상태로 끝났는지(예: EXPIRED_BEFORE_DISPATCH, ACK_TIMEOUT)
        sa.Column("result_json", sa.JSON()),
    )
    op.create_index("idx_arm_commands_robot_created", "arm_commands", ["robot_id", "created_at"])
    op.create_table(
        "arm_command_events",
        sa.Column("id", sa.String(), primary_key=True),
        sa.Column(
            "command_id",
            sa.String(),
            sa.ForeignKey("arm_commands.id", ondelete="CASCADE"),
            nullable=False,
        ),
        # 워커가 붙인 이벤트 id. 서버가 남기는 이벤트는 NULL이다.
        sa.Column("worker_event_id", sa.String()),
        sa.Column("event_type", sa.String(), nullable=False),
        sa.Column("previous_state", sa.String()),
        sa.Column("new_state", sa.String()),
        sa.Column("occurred_at", sa.String()),
        sa.Column("received_at", sa.String(), nullable=False),
        sa.Column("payload_json", sa.JSON(), nullable=False),
        # 워커가 같은 이벤트를 재전송해도 한 번만 저장된다(멱등). SQLite는 NULL끼리는 겹친다고 보지 않는다.
        sa.UniqueConstraint(
            "command_id", "worker_event_id", name="uq_arm_command_events_worker_event"
        ),
    )
    op.create_index(
        "idx_arm_command_events_command_received",
        "arm_command_events",
        ["command_id", "received_at", "id"],
    )


def downgrade() -> None:
    op.drop_index("idx_arm_command_events_command_received", table_name="arm_command_events")
    op.drop_table("arm_command_events")
    op.drop_index("idx_arm_commands_robot_created", table_name="arm_commands")
    op.drop_table("arm_commands")
