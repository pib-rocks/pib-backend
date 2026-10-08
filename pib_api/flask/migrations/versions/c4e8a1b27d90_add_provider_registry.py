"""provider registry with capability flags

Revision ID: c4e8a1b27d90
Revises: b17461748001
Create Date: 2026-09-30 00:00:00.000000

"""

import json

import sqlalchemy as sa
from alembic import op

from provider_registry import capabilities_for, is_registry_default

# revision identifiers, used by Alembic.
revision = "c4e8a1b27d90"
down_revision = "b17461748001"
branch_labels = None
depends_on = None


def upgrade():
    op.create_table(
        "provider",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("api_name", sa.String(length=255), nullable=False),
        sa.Column("visual_name", sa.String(length=255), nullable=False),
        sa.Column("has_image_support", sa.Boolean(), nullable=False),
        sa.Column("endpoint_base", sa.String(length=1024), nullable=True),
        sa.Column("capabilities", sa.JSON(), nullable=False),
        sa.Column("credential_ref", sa.String(length=255), nullable=True),
        sa.Column("is_default", sa.Boolean(), nullable=False),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("visual_name"),
    )
    op.create_index(
        "uq_provider_single_default",
        "provider",
        ["is_default"],
        unique=True,
        sqlite_where=sa.text("is_default = 1"),
    )

    conn = op.get_bind()
    rows = conn.execute(
        sa.text(
            "SELECT id, api_name, visual_name, has_image_support FROM assistant_model"
        )
    ).fetchall()
    for row in rows:
        images = bool(row.has_image_support)
        conn.execute(
            sa.text("""
                INSERT INTO provider (
                    id, api_name, visual_name, has_image_support,
                    endpoint_base, capabilities, credential_ref, is_default
                )
                VALUES (
                    :id, :api_name, :visual_name, :has_image_support,
                    NULL, :capabilities, NULL, :is_default
                )
                """),
            {
                "id": row.id,
                "api_name": row.api_name,
                "visual_name": row.visual_name,
                "has_image_support": images,
                "capabilities": json.dumps(capabilities_for(row.api_name, images)),
                "is_default": 1 if is_registry_default(row.api_name) else 0,
            },
        )

    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column("provider_ref", sa.String(length=255), nullable=True)
        )

    conn.execute(sa.text("""
            UPDATE personality
            SET provider_ref = CAST(assistant_model_id AS TEXT)
            WHERE provider_ref IS NULL
            """))

    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.alter_column(
            "provider_ref",
            existing_type=sa.String(length=255),
            nullable=False,
        )
        batch_op.alter_column(
            "assistant_model_id",
            existing_type=sa.Integer(),
            nullable=True,
        )


def downgrade():
    conn = op.get_bind()
    conn.execute(sa.text("""
            UPDATE personality
            SET assistant_model_id = (
                SELECT id FROM provider WHERE is_default = 1
            )
            WHERE assistant_model_id IS NULL
            """))
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.alter_column(
            "assistant_model_id",
            existing_type=sa.Integer(),
            nullable=False,
        )
        batch_op.drop_column("provider_ref")
    op.drop_index("uq_provider_single_default", table_name="provider")
    op.drop_table("provider")
