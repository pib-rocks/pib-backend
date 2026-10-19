"""chat message metadata (answering transport, model, stage timings)

Revision ID: a7d2e5c81b34
Revises: c3a8e1d94b70
Create Date: 2026-10-18 23:40:00.000000

"""

import sqlalchemy as sa
from alembic import op

revision = "a7d2e5c81b34"
down_revision = "c3a8e1d94b70"
branch_labels = None
depends_on = None


def upgrade():
    """Add the nullable JSON column; existing rows keep no metadata."""
    with op.batch_alter_table("chatMessage", schema=None) as batch_op:
        batch_op.add_column(sa.Column("meta", sa.JSON(), nullable=True))


def downgrade():
    with op.batch_alter_table("chatMessage", schema=None) as batch_op:
        batch_op.drop_column("meta")
