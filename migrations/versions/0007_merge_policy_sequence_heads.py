"""Join order-history migrations with the previously deployed arm-sequence revision."""

revision = "0007_merge_policy_sequence_heads"
down_revision = ("0006_order_requests", "0005_arm_sequences")
branch_labels = None
depends_on = None


def upgrade() -> None:
    pass


def downgrade() -> None:
    pass
