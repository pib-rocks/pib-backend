"""remove provider rows that left the catalogue

Revision ID: a8c3e1d74f20
Revises: d9f2a6c41e88
Create Date: 2026-10-02 15:00:00.000000

The previous purge deleted rows that were outside the catalogue at that
time. hermes-agent was still listed then, so an installation that already
ran that revision still has the row. This revision deletes again from the
catalogue as it stands now. A personality that pointed at a deleted row
keeps its provider_ref. Only the foreign key is cleared. It is not moved
onto another model. Catalogue models without a row are added, and the
catalogue default stays the only default row.

"""

import json

import sqlalchemy as sa
from alembic import op

from pib_hermes_config.live_session import (
    GEMINI_LIVE_MODEL,
    GEMINI_LIVE_MODEL_CHECKED_ON,
)
from provider_registry import (
    DEFAULT_PROVIDER_API_NAME,
    active_api_names,
    active_entries,
    capabilities_for,
    pins_gemini_live_model,
)

revision = "a8c3e1d74f20"
down_revision = "d9f2a6c41e88"
branch_labels = None
depends_on = None


def upgrade():
    conn = op.get_bind()
    count = conn.execute(sa.text("SELECT COUNT(*) FROM assistant_model")).scalar()
    if not count:
        # An empty database is seeded by commands.py.
        return

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
        sa.text(f"DELETE FROM provider WHERE api_name NOT IN ({placeholders})"),
        names,
    )
    conn.execute(
        sa.text(f"DELETE FROM assistant_model WHERE api_name NOT IN ({placeholders})"),
        names,
    )

    present = {
        row.api_name
        for row in conn.execute(sa.text("SELECT api_name FROM assistant_model"))
    }
    for entry in active_entries():
        if entry.api_name in present:
            continue
        conn.execute(
            sa.text("""
                INSERT INTO assistant_model (api_name, visual_name, has_image_support)
                VALUES (:api_name, :visual_name, :has_image_support)
                """),
            {
                "api_name": entry.api_name,
                "visual_name": entry.visual_name,
                "has_image_support": entry.images,
            },
        )
        model_id = conn.execute(
            sa.text("SELECT id FROM assistant_model WHERE visual_name = :visual_name"),
            {"visual_name": entry.visual_name},
        ).scalar()
        pinned = pins_gemini_live_model(entry.api_name)
        conn.execute(
            sa.text("""
                INSERT INTO provider (
                    id, api_name, visual_name, has_image_support, endpoint_base,
                    capabilities, credential_ref, is_default,
                    live_model, live_model_checked_on
                )
                VALUES (
                    :id, :api_name, :visual_name, :has_image_support, NULL,
                    :capabilities, NULL, 0,
                    :live_model, :checked_on
                )
                """),
            {
                "id": model_id,
                "api_name": entry.api_name,
                "visual_name": entry.visual_name,
                "has_image_support": entry.images,
                "capabilities": json.dumps(
                    capabilities_for(entry.api_name, entry.images)
                ),
                "live_model": GEMINI_LIVE_MODEL if pinned else None,
                "checked_on": GEMINI_LIVE_MODEL_CHECKED_ON if pinned else None,
            },
        )

    conn.execute(sa.text("UPDATE provider SET is_default = 0"))
    conn.execute(
        sa.text("UPDATE provider SET is_default = 1 WHERE api_name = :api_name"),
        {"api_name": DEFAULT_PROVIDER_API_NAME},
    )


def downgrade():
    # The deleted rows are not recreated: their models are not supported.
    pass
