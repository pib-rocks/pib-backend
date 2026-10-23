"""replace motor invert flag with upper_endpoint mapping

Revision ID: e8f1a2b3c4d5
Revises: d7a1c4e29b36
Create Date: 2026-10-23 08:00:00.000000

PR-2030: physical endpoint naming replaces the invert boolean. Existing rows
keep the same drive sign: invert=True becomes upper_endpoint='min'.
"""

import sqlalchemy as sa
from alembic import op

revision = "e8f1a2b3c4d5"
down_revision = "d7a1c4e29b36"
branch_labels = None
depends_on = None


def upgrade():
    op.add_column(
        "motor",
        sa.Column(
            "upper_endpoint",
            sa.String(length=4),
            nullable=False,
            server_default="max",
        ),
    )
    op.execute(
        "UPDATE motor SET upper_endpoint = CASE WHEN invert = 1 THEN 'min' ELSE 'max' END"
    )
    with op.batch_alter_table("motor", schema=None) as batch_op:
        batch_op.drop_column("invert")


def downgrade():
    op.add_column(
        "motor",
        sa.Column("invert", sa.Boolean(), nullable=False, server_default=sa.false()),
    )
    op.execute(
        "UPDATE motor SET invert = CASE WHEN upper_endpoint = 'min' THEN 1 ELSE 0 END"
    )
    with op.batch_alter_table("motor", schema=None) as batch_op:
        batch_op.drop_column("upper_endpoint")
