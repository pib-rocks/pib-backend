"""personality chat channel, smart or direct

Revision ID: d7c1e4a90b22
Revises: c4e8a1b27d90
Create Date: 2026-09-30 03:00:00.000000

"""

import sqlalchemy as sa
from alembic import op

revision = "d7c1e4a90b22"
down_revision = "c4e8a1b27d90"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "channel",
                sa.String(length=255),
                nullable=True,
                server_default="smart",
            )
        )
    op.execute("UPDATE personality SET channel = 'smart' WHERE channel IS NULL")
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.alter_column(
            "channel",
            existing_type=sa.String(length=255),
            nullable=False,
            server_default="smart",
        )


def downgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.drop_column("channel")
