"""nullable per-personality reasoning effort

Revision ID: c3a8e1d94b70
Revises: e4b8c1d90a72
Create Date: 2026-10-09 12:00:00.000000

NULL means unmanaged: the personality's Hermes profile is left as it is.
Existing rows are not backfilled. A new personality is created with "none"
by the service, not by a server default.

"""

import sqlalchemy as sa
from alembic import op

revision = "c3a8e1d94b70"
down_revision = "e4b8c1d90a72"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column("reasoning_effort", sa.String(length=32), nullable=True)
        )


def downgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.drop_column("reasoning_effort")
