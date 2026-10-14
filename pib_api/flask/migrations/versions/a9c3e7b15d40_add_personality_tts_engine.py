"""personality text-to-speech backend, local Supertone by default

Revision ID: a9c3e7b15d40
Revises: e6b2d8f14c70
Create Date: 2026-09-30 05:00:00.000000

"""

import sqlalchemy as sa
from alembic import op

revision = "a9c3e7b15d40"
down_revision = "e6b2d8f14c70"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "tts_engine",
                sa.String(length=255),
                nullable=True,
                server_default="supertone",
            )
        )
    op.execute(
        "UPDATE personality SET tts_engine = 'supertone' WHERE tts_engine IS NULL"
    )
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.alter_column(
            "tts_engine",
            existing_type=sa.String(length=255),
            nullable=False,
            server_default="supertone",
        )


def downgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.drop_column("tts_engine")
