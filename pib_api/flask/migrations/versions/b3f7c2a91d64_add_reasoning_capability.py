"""add the reasoning capability flag to stored rows

Revision ID: b3f7c2a91d64
Revises: a7d2e5c81b34
Create Date: 2026-10-19 10:00:00.000000

The catalogue gained one more capability flag, ``reasoning``. pib.Cloud
sets its own effort and answers HTTP 400 on ``reasoning_effort``, so its
flag is off; the other cloud chat models carry it on and the on-device row
carries it off.

A fresh seed writes the new flags. An installation that already has rows
would keep the old key set, so this revision rewrites the capability JSON
of every existing ``registry_model`` row and recomputes each ``provider``
row's shared flags from its models, exactly as ``sync_shared_capabilities``
does at startup.
"""

import json

import sqlalchemy as sa
from alembic import op

from provider_registry import CAPABILITY_KEYS, capabilities_for, catalogue_entry

revision = "b3f7c2a91d64"
down_revision = "a7d2e5c81b34"
branch_labels = None
depends_on = None


def _parsed(raw: object) -> dict:
    """The stored capability JSON as a dict. Empty when it cannot be read."""
    if isinstance(raw, dict):
        return dict(raw)
    if not isinstance(raw, str):
        return {}
    try:
        loaded = json.loads(raw)
    except (TypeError, ValueError):
        return {}
    return dict(loaded) if isinstance(loaded, dict) else {}


def _flags(raw: object) -> dict:
    """Every shared capability key, keeping unrelated keys such as ``offline``."""
    stored = _parsed(raw)
    flags = {key: bool(stored.get(key)) for key in CAPABILITY_KEYS}
    for key, value in stored.items():
        if key not in flags:
            flags[key] = value
    return flags


def _refresh_model_rows(conn) -> None:
    rows = conn.execute(
        sa.text(
            "SELECT id, api_name, has_image_support, capabilities FROM registry_model"
        )
    ).fetchall()
    for row in rows:
        entry = catalogue_entry(row.api_name)
        flags = (
            capabilities_for(row.api_name, bool(row.has_image_support))
            if entry is not None
            else _flags(row.capabilities)
        )
        conn.execute(
            sa.text(
                "UPDATE registry_model SET capabilities = :capabilities WHERE id = :id"
            ),
            {"capabilities": json.dumps(flags), "id": row.id},
        )


def _refresh_provider_capabilities(conn) -> None:
    shared: dict[object, dict[str, bool]] = {}
    for row in conn.execute(
        sa.text("SELECT provider_id, capabilities FROM registry_model")
    ):
        flags = _flags(row.capabilities)
        aggregate = shared.setdefault(
            row.provider_id, {key: True for key in CAPABILITY_KEYS}
        )
        for key in CAPABILITY_KEYS:
            aggregate[key] = bool(aggregate[key] and flags.get(key))
    for row in conn.execute(sa.text("SELECT id FROM provider")):
        flags = shared.get(row.id, {key: False for key in CAPABILITY_KEYS})
        conn.execute(
            sa.text("UPDATE provider SET capabilities = :capabilities WHERE id = :id"),
            {"capabilities": json.dumps(flags), "id": row.id},
        )


def upgrade():
    conn = op.get_bind()
    count = conn.execute(sa.text("SELECT COUNT(*) FROM registry_model")).scalar()
    if not count:
        # An empty database is seeded by commands.py.
        return
    _refresh_model_rows(conn)
    _refresh_provider_capabilities(conn)


def downgrade():
    conn = op.get_bind()
    for table in ("registry_model", "provider"):
        for row in conn.execute(sa.text(f"SELECT id, capabilities FROM {table}")):
            stored = _parsed(row.capabilities)
            if "reasoning" not in stored:
                continue
            stored.pop("reasoning")
            conn.execute(
                sa.text(
                    f"UPDATE {table} SET capabilities = :capabilities WHERE id = :id"
                ),
                {"capabilities": json.dumps(stored), "id": row.id},
            )
