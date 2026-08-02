"""add stt_engine

Revision ID: e2f38192ab14
Revises: d4d024ef3a32
Create Date: 2026-07-27 10:00:00.000000

"""

from alembic import op
import sqlalchemy as sa

# revision identifiers, used by Alembic.
revision = "e2f38192ab14"
down_revision = "d4d024ef3a32"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "stt_engine",
                sa.String(length=255),
                nullable=False,
                server_default="local_whisper",
            )
        )


def downgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.drop_column("stt_engine")
