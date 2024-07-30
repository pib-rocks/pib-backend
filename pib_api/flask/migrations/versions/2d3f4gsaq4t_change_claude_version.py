"""change_claude_version

Revision ID: 2d3f4gsaq4t
Revises: 7160a4f6951f
Create Date: 2024-07-23 13:32:24.207287

"""

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision = "2d3f4gsaq4t"
down_revision = "7160a4f6951f"
branch_labels = None
depends_on = None


def upgrade():

    conn = op.get_bind()

    # only modify/add new Claude 3.5 Sonnet migrations if the table is not empty - otherwise, the commands.py will fill in all default values
    result = conn.execute(sa.text("SELECT COUNT(*) FROM assistant_model"))
    count = result.scalar()

    if count > 0:
        conn.execute(
            sa.text(
                """
                UPDATE assistant_model
                SET api_name = 'anthropic.claude-3-5-sonnet-20240620-v1:0', visual_name = 'Claude 3.5 Sonnet [Vision]', has_image_support = true
                WHERE api_name = 'anthropic.claude-3-sonnet-20240229-v1:0';
                """
            )
        )
        conn.execute(
            sa.text(
            """
            INSERT OR IGNORE INTO assistant_model (api_name, visual_name, has_image_support)
            VALUES ('anthropic.claude-3-5-sonnet-20240620-v1:0', 'Claude 3.5 Sonnet [Text]', false)
            """
            )
        )

def downgrade():
        
    conn = op.get_bind()

    result = conn.execute(sa.text("SELECT COUNT(*) FROM assistant_model"))
    count = result.scalar()

    if count > 0:
        conn.execute(
            sa.text(
                """
                DELETE FROM assistant_model WHERE visual_name = 'Claude 3.5 Sonnet [Text]'
                """
            )
        )
        conn.execute(
            sa.text(
                """
                UPDATE assistant_model
                SET api_name = 'anthropic.claude-3-sonnet-20240229-v1:0', visual_name = 'Claude 3 Sonnet [Vision]', has_image_support = true
                WHERE api_name = 'anthropic.claude-3-5-sonnet-20240620-v1:0';
                """
            )
        )