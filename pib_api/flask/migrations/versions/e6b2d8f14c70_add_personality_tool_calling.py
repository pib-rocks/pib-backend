"""personality tool-calling switch, on by default

Revision ID: e6b2d8f14c70
Revises: d7c1e4a90b22
Create Date: 2026-09-30 04:00:00.000000

"""

import sqlalchemy as sa
from alembic import op

revision = "e6b2d8f14c70"
down_revision = "d7c1e4a90b22"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "tool_calling",
                sa.Boolean(),
                nullable=True,
                server_default=sa.text("1"),
            )
        )
    op.execute("UPDATE personality SET tool_calling = 1 WHERE tool_calling IS NULL")
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.alter_column(
            "tool_calling",
            existing_type=sa.Boolean(),
            nullable=False,
            server_default=sa.text("1"),
        )


def downgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.drop_column("tool_calling")
