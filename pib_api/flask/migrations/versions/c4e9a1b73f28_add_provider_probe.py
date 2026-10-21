"""add the provider probe wiring

Revision ID: c4e9a1b73f28
Revises: b3f7c2a91d64
Create Date: 2026-10-21 12:10:00.000000

The status strip asks pib-backend whether a personality's provider is usable
(PC-115). That probe may not name a host, an auth header or a vendor status
page itself: production code carries no provider wiring, and
``test_production_models_are_in_the_catalogue`` rejects strings that look like
catalogue model ids. The wiring lives in ``provider_probes`` and is stored per
``provider`` row instead.

A fresh seed writes it. An installation that already has rows would keep a
null column, so this revision adds it and fills every known provider; rows this
build knows nothing about stay null and are reported as unknown, never red.
"""

import json

import sqlalchemy as sa
from alembic import op

from provider_probes import probe_for

revision = "c4e9a1b73f28"
down_revision = "b3f7c2a91d64"
branch_labels = None
depends_on = None


def _column_exists(conn) -> bool:
    rows = conn.execute(sa.text("PRAGMA table_info(provider)")).fetchall()
    return any(row[1] == "probe" for row in rows)


def upgrade():
    conn = op.get_bind()
    if not _column_exists(conn):
        op.add_column("provider", sa.Column("probe", sa.JSON(), nullable=True))
    count = conn.execute(sa.text("SELECT COUNT(*) FROM provider")).scalar()
    if not count:
        # An empty database is seeded by commands.py.
        return
    for row in conn.execute(sa.text("SELECT id, name FROM provider")).fetchall():
        probe = probe_for(row.name)
        if not probe:
            continue
        conn.execute(
            sa.text("UPDATE provider SET probe = :probe WHERE id = :id"),
            {"probe": json.dumps(probe), "id": row.id},
        )


def downgrade():
    conn = op.get_bind()
    if _column_exists(conn):
        op.drop_column("provider", "probe")
