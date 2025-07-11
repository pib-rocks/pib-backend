"""add gemini assistant model

Revision ID: ba9e8ed1e18c
Revises: ddd0daa8e6f5
Create Date: 2025-07-25 12:00:00.000000

"""

from alembic import op
import sqlalchemy as sa

# revision identifiers, used by Alembic.
revision = "ba9e8ed1e18c"
down_revision = "ddd0daa8e6f5"
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
                VALUES ('gemini-2.5-flash', 'Gemini 2.5 Flash', false)
                """
            )
        )


def downgrade():
    conn = op.get_bind()
    conn.execute(
        sa.text(
            "DELETE FROM assistant_model WHERE visual_name = 'Gemini 2.5 Flash'"
        )
    )
