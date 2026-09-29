"""add registered system properties

Revision ID: b17461748001
Revises: a1638c0de001
Create Date: 2026-09-17 16:20:00
"""

from alembic import op
import sqlalchemy as sa

revision = "b17461748001"
down_revision = "a1638c0de001"
branch_labels = None
depends_on = None


def upgrade():
    system_property = op.create_table(
        "system_property",
        sa.Column("key", sa.String(length=100), nullable=False),
        sa.Column("value", sa.Text(), nullable=False),
        sa.Column("value_type", sa.String(length=20), nullable=False),
        sa.Column("source", sa.String(length=20), nullable=False),
        sa.Column(
            "created_at",
            sa.DateTime(),
            nullable=False,
            server_default=sa.func.current_timestamp(),
        ),
        sa.Column(
            "updated_at",
            sa.DateTime(),
            nullable=False,
            server_default=sa.func.current_timestamp(),
        ),
        sa.PrimaryKeyConstraint("key"),
    )
    op.bulk_insert(
        system_property,
        [
            {
                "key": "hardware.variant",
                "value": "pib5edu",
                "value_type": "str",
                "source": "default",
            }
        ],
    )


def downgrade():
    op.drop_table("system_property")
