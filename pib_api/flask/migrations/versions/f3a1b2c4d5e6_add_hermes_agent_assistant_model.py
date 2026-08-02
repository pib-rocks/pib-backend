"""add hermes-agent assistant model

Revision ID: f3a1b2c4d5e6
Revises: e2f38192ab14
Create Date: 2026-07-31 15:00:00.000000
"""

from alembic import op
import sqlalchemy as sa

# revision identifiers, used by Alembic.
revision = "f3a1b2c4d5e6"
down_revision = "e2f38192ab14"
branch_labels = None
depends_on = None


def upgrade():
    conn = op.get_bind()
    result = conn.execute(sa.text("SELECT COUNT(*) FROM assistant_model"))
    count = result.scalar()

    if count > 0:
        conn.execute(
            sa.text(
                """
                INSERT OR IGNORE INTO assistant_model (api_name, visual_name, has_image_support)
                VALUES ('hermes-agent', 'Hermes Agent (selbstlernend)', true)
                """
            )
        )


def downgrade():
    conn = op.get_bind()
    conn.execute(
        sa.text(
            "DELETE FROM assistant_model WHERE visual_name = 'Hermes Agent (selbstlernend)'"
        )
    )
