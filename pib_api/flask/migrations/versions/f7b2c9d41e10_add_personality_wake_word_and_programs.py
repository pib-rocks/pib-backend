"""add personality wake word and wake up / go sleep program columns (PR-1153)

Revision ID: f7b2c9d41e10
Revises: e8f1a2b3c4d5
Create Date: 2026-11-06 00:15:00.000000

"""

import sqlalchemy as sa
from alembic import op

revision = "f7b2c9d41e10"
down_revision = "e8f1a2b3c4d5"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "use_wake_word",
                sa.Boolean(),
                nullable=False,
                server_default="0",
            )
        )
        batch_op.add_column(
            sa.Column("wake_up_program", sa.String(length=255), nullable=True)
        )
        batch_op.add_column(
            sa.Column("go_sleep_program", sa.String(length=255), nullable=True)
        )


def downgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.drop_column("go_sleep_program")
        batch_op.drop_column("wake_up_program")
        batch_op.drop_column("use_wake_word")
