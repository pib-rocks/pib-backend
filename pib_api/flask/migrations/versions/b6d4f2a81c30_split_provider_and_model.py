"""split each flat registry row into a provider and one model

Revision ID: b6d4f2a81c30
Revises: a8c3e1d74f20
Create Date: 2026-10-03 15:00:00.000000

A provider keeps the credential, the endpoint, and the flags that hold for
every model it has. The model keeps the chat id, the display name, and its
own flags, and it keeps the id a personality already stores. Rows that share
a catalogue provider share one provider row.

"""

import json

import sqlalchemy as sa
from alembic import op

from provider_registry import (
    CAPABILITY_KEYS,
    capabilities_held_by_all,
    provider_name_for,
)

revision = "b6d4f2a81c30"
down_revision = "a8c3e1d74f20"
branch_labels = None
depends_on = None


def _capabilities(raw):
    if isinstance(raw, str):
        loaded = json.loads(raw)
        if isinstance(loaded, dict):
            return loaded
        return {key: False for key in CAPABILITY_KEYS}
    if isinstance(raw, dict):
        return raw
    return {key: False for key in CAPABILITY_KEYS}


def upgrade():
    conn = op.get_bind()
    op.create_table(
        "provider_account",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("name", sa.String(length=255), nullable=False),
        sa.Column("endpoint_base", sa.String(length=1024), nullable=True),
        sa.Column("capabilities", sa.JSON(), nullable=False),
        sa.Column("credential_ref", sa.String(length=255), nullable=True),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("name"),
    )
    op.create_table(
        "registry_model",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("provider_id", sa.Integer(), nullable=False),
        sa.Column("api_name", sa.String(length=255), nullable=False),
        sa.Column("visual_name", sa.String(length=255), nullable=False),
        sa.Column("has_image_support", sa.Boolean(), nullable=False),
        sa.Column("capabilities", sa.JSON(), nullable=False),
        sa.Column("is_default", sa.Boolean(), nullable=False),
        sa.Column("live_model", sa.String(length=255), nullable=True),
        sa.Column("live_model_checked_on", sa.Date(), nullable=True),
        sa.ForeignKeyConstraint(["provider_id"], ["provider_account.id"]),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("visual_name"),
    )
    op.create_index(
        "uq_registry_model_single_default",
        "registry_model",
        ["is_default"],
        unique=True,
        sqlite_where=sa.text("is_default = 1"),
    )

    rows = conn.execute(sa.text("""
            SELECT id, api_name, visual_name, has_image_support, endpoint_base,
                   capabilities, credential_ref, is_default, live_model,
                   live_model_checked_on
            FROM provider
            """)).fetchall()
    grouped: dict[str, dict] = {}
    for row in rows:
        name = provider_name_for(row.api_name, row.visual_name)
        group = grouped.setdefault(
            name,
            {"endpoint": None, "credential": None, "capabilities": [], "rows": []},
        )
        if row.endpoint_base and group["endpoint"] is None:
            group["endpoint"] = row.endpoint_base
        if row.credential_ref and group["credential"] is None:
            group["credential"] = row.credential_ref
        group["capabilities"].append(_capabilities(row.capabilities))
        group["rows"].append(row)

    name_to_id: dict[str, int] = {}
    for name, group in grouped.items():
        conn.execute(
            sa.text("""
                INSERT INTO provider_account (
                    name, endpoint_base, capabilities, credential_ref
                )
                VALUES (:name, :endpoint_base, :capabilities, :credential_ref)
                """),
            {
                "name": name,
                "endpoint_base": group["endpoint"],
                "capabilities": json.dumps(
                    capabilities_held_by_all(group["capabilities"])
                ),
                "credential_ref": group["credential"],
            },
        )
        name_to_id[name] = conn.execute(
            sa.text("SELECT id FROM provider_account WHERE name = :name"),
            {"name": name},
        ).scalar()

    for name, group in grouped.items():
        provider_id = name_to_id[name]
        for row in group["rows"]:
            conn.execute(
                sa.text("""
                    INSERT INTO registry_model (
                        id, provider_id, api_name, visual_name, has_image_support,
                        capabilities, is_default, live_model, live_model_checked_on
                    )
                    VALUES (
                        :id, :provider_id, :api_name, :visual_name, :has_image_support,
                        :capabilities, :is_default, :live_model, :checked_on
                    )
                    """),
                {
                    "id": row.id,
                    "provider_id": provider_id,
                    "api_name": row.api_name,
                    "visual_name": row.visual_name,
                    "has_image_support": row.has_image_support,
                    "capabilities": (
                        row.capabilities
                        if isinstance(row.capabilities, str)
                        else json.dumps(_capabilities(row.capabilities))
                    ),
                    "is_default": row.is_default,
                    "live_model": row.live_model,
                    "checked_on": row.live_model_checked_on,
                },
            )

    op.drop_index("uq_provider_single_default", table_name="provider")
    op.drop_table("provider")
    op.rename_table("provider_account", "provider")


def downgrade():
    conn = op.get_bind()
    op.create_table(
        "provider_flat",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("api_name", sa.String(length=255), nullable=False),
        sa.Column("visual_name", sa.String(length=255), nullable=False),
        sa.Column("has_image_support", sa.Boolean(), nullable=False),
        sa.Column("endpoint_base", sa.String(length=1024), nullable=True),
        sa.Column("capabilities", sa.JSON(), nullable=False),
        sa.Column("credential_ref", sa.String(length=255), nullable=True),
        sa.Column("is_default", sa.Boolean(), nullable=False),
        sa.Column("live_model", sa.String(length=255), nullable=True),
        sa.Column("live_model_checked_on", sa.Date(), nullable=True),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("visual_name"),
    )
    rows = conn.execute(sa.text("""
            SELECT registry_model.id, registry_model.api_name,
                   registry_model.visual_name, registry_model.has_image_support,
                   provider.endpoint_base, registry_model.capabilities,
                   provider.credential_ref, registry_model.is_default,
                   registry_model.live_model, registry_model.live_model_checked_on
            FROM registry_model
            JOIN provider ON provider.id = registry_model.provider_id
            """)).fetchall()
    for row in rows:
        conn.execute(
            sa.text("""
                INSERT INTO provider_flat (
                    id, api_name, visual_name, has_image_support, endpoint_base,
                    capabilities, credential_ref, is_default, live_model,
                    live_model_checked_on
                )
                VALUES (
                    :id, :api_name, :visual_name, :has_image_support, :endpoint_base,
                    :capabilities, :credential_ref, :is_default, :live_model,
                    :checked_on
                )
                """),
            {
                "id": row[0],
                "api_name": row[1],
                "visual_name": row[2],
                "has_image_support": row[3],
                "endpoint_base": row[4],
                "capabilities": (
                    row[5] if isinstance(row[5], str) else json.dumps(row[5])
                ),
                "credential_ref": row[6],
                "is_default": row[7],
                "live_model": row[8],
                "checked_on": row[9],
            },
        )
    op.drop_index("uq_registry_model_single_default", table_name="registry_model")
    op.drop_table("registry_model")
    op.drop_table("provider")
    op.rename_table("provider_flat", "provider")
    op.create_index(
        "uq_provider_single_default",
        "provider",
        ["is_default"],
        unique=True,
        sqlite_where=sa.text("is_default = 1"),
    )
