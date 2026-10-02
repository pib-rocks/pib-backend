"""personality filler and per-chat first-token latency

Revision ID: b7e1c4a92d10
Revises: f1a9c3e74b20
Create Date: 2026-09-30 07:10:00.000000

thinking_filler stays null. A personality with no authored text does not
gain a phrase. first_token_latency_ms stays null until a turn is measured.

"""

import sqlalchemy as sa
from alembic import op

revision = "b7e1c4a92d10"
down_revision = "f1a9c3e74b20"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column("thinking_filler", sa.String(length=255), nullable=True)
        )
    with op.batch_alter_table("chat", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column("first_token_latency_ms", sa.Float(), nullable=True)
        )


def downgrade():
    with op.batch_alter_table("chat", schema=None) as batch_op:
        batch_op.drop_column("first_token_latency_ms")
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.drop_column("thinking_filler")
