"""remove registry rows that are not in the catalogue

Revision ID: e4b8c1d90a72
Revises: b6d4f2a81c30
Create Date: 2026-10-03 17:30:00.000000

The split left each model id where a personality already points. This
revision deletes again from the catalogue as it stands now, from
registry_model and assistant_model. A personality that pointed at a deleted
row keeps its provider_ref. Only the foreign key is cleared. It is not moved
onto another model, onto the provider, or onto the default. A catalogue model
that has no row is added under its provider, with an id above every id that
already existed, so a dangling reference cannot land on it. The catalogue
default stays the only default row. A provider with no model left is deleted.
One that still has a model keeps its credential.

"""

import json

import sqlalchemy as sa
from alembic import op

from provider_registry import (
    DEFAULT_PROVIDER_API_NAME,
    active_api_names,
    active_entries,
    capabilities_for,
    capabilities_held_by_all,
)

revision = "e4b8c1d90a72"
down_revision = "b6d4f2a81c30"
branch_labels = None
depends_on = None


def _max_id(conn, table: str) -> int:
    value = conn.execute(sa.text(f"SELECT MAX(id) FROM {table}")).scalar()
    return int(value or 0)


def _provider_id(conn, name: str) -> int:
    found = conn.execute(
        sa.text("SELECT id FROM provider WHERE name = :name"),
        {"name": name},
    ).scalar()
    if found is not None:
        return int(found)
    conn.execute(
        sa.text("""
            INSERT INTO provider (name, endpoint_base, capabilities, credential_ref)
            VALUES (:name, NULL, :capabilities, NULL)
            """),
        {
            "name": name,
            "capabilities": json.dumps(capabilities_held_by_all(())),
        },
    )
    return int(
        conn.execute(
            sa.text("SELECT id FROM provider WHERE name = :name"),
            {"name": name},
        ).scalar()
    )


def _refresh_provider_capabilities(conn) -> None:
    """Store the flags that are true on every model the provider still has."""
    providers = conn.execute(sa.text("SELECT id FROM provider")).fetchall()
    for (provider_id,) in providers:
        raw = conn.execute(
            sa.text("""
                SELECT capabilities FROM registry_model WHERE provider_id = :id
                """),
            {"id": provider_id},
        ).fetchall()
        conn.execute(
            sa.text("""
                UPDATE provider SET capabilities = :capabilities WHERE id = :id
                """),
            {
                "id": provider_id,
                "capabilities": json.dumps(
                    capabilities_held_by_all(row[0] for row in raw)
                ),
            },
        )


def upgrade():
    conn = op.get_bind()
    assistant_count = conn.execute(
        sa.text("SELECT COUNT(*) FROM assistant_model")
    ).scalar()
    registry_count = conn.execute(
        sa.text("SELECT COUNT(*) FROM registry_model")
    ).scalar()
    if not assistant_count and not registry_count:
        # An empty database is seeded by commands.py.
        return

    # Taken before the delete, so a new catalogue row cannot reuse an id a
    # personality still stores.
    next_id = max(_max_id(conn, "assistant_model"), _max_id(conn, "registry_model")) + 1

    supported = sorted(active_api_names())
    names = {f"name_{index}": name for index, name in enumerate(supported)}
    placeholders = ", ".join(f":{key}" for key in names)

    conn.execute(
        sa.text(f"""
            UPDATE personality
            SET assistant_model_id = NULL
            WHERE assistant_model_id IN (
                SELECT id FROM assistant_model
                WHERE api_name NOT IN ({placeholders})
            )
            """),
        names,
    )
    conn.execute(
        sa.text(f"""
            UPDATE personality
            SET assistant_model_id = NULL
            WHERE assistant_model_id IN (
                SELECT id FROM registry_model
                WHERE api_name NOT IN ({placeholders})
            )
            """),
        names,
    )
    conn.execute(
        sa.text(f"DELETE FROM registry_model WHERE api_name NOT IN ({placeholders})"),
        names,
    )
    conn.execute(
        sa.text(f"DELETE FROM assistant_model WHERE api_name NOT IN ({placeholders})"),
        names,
    )

    assistant_ids = {
        row[1]: row[0]
        for row in conn.execute(sa.text("SELECT id, api_name FROM assistant_model"))
    }
    registry_ids = {
        row[1]: row[0]
        for row in conn.execute(sa.text("SELECT id, api_name FROM registry_model"))
    }
    for entry in active_entries():
        api_name = entry.api_name
        if api_name in assistant_ids and api_name in registry_ids:
            continue
        if api_name in assistant_ids:
            model_id = assistant_ids[api_name]
        elif api_name in registry_ids:
            model_id = registry_ids[api_name]
        else:
            model_id = next_id
            next_id += 1
        if api_name not in assistant_ids:
            conn.execute(
                sa.text("""
                    INSERT INTO assistant_model (
                        id, api_name, visual_name, has_image_support
                    )
                    VALUES (:id, :api_name, :visual_name, :has_image_support)
                    """),
                {
                    "id": model_id,
                    "api_name": api_name,
                    "visual_name": entry.visual_name,
                    "has_image_support": entry.images,
                },
            )
            assistant_ids[api_name] = model_id
        if api_name not in registry_ids:
            conn.execute(
                sa.text("""
                    INSERT INTO registry_model (
                        id, provider_id, api_name, visual_name, has_image_support,
                        capabilities, is_default, live_model, live_model_checked_on
                    )
                    VALUES (
                        :id, :provider_id, :api_name, :visual_name,
                        :has_image_support, :capabilities, 0, NULL, NULL
                    )
                    """),
                {
                    "id": model_id,
                    "provider_id": _provider_id(conn, entry.provider),
                    "api_name": api_name,
                    "visual_name": entry.visual_name,
                    "has_image_support": entry.images,
                    "capabilities": json.dumps(
                        capabilities_for(api_name, entry.images)
                    ),
                },
            )
            registry_ids[api_name] = model_id

    conn.execute(sa.text("""
            DELETE FROM provider
            WHERE id NOT IN (SELECT provider_id FROM registry_model)
            """))
    _refresh_provider_capabilities(conn)

    conn.execute(sa.text("UPDATE registry_model SET is_default = 0"))
    conn.execute(
        sa.text("UPDATE registry_model SET is_default = 1 WHERE api_name = :api_name"),
        {"api_name": DEFAULT_PROVIDER_API_NAME},
    )


def downgrade():
    # The deleted rows are not recreated: their models are not supported.
    pass
