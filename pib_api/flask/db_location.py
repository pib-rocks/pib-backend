"""Relocate legacy SQLite DB from the Flask bind mount to persistent storage."""

from __future__ import annotations

import os
import sqlite3
from pathlib import Path

LEGACY_DB_PATH = Path("/app/pibdata.db")


def sqlite_uri_to_path(uri: str) -> Path | None:
    if not uri.startswith("sqlite:"):
        return None
    if uri.startswith("sqlite:////"):
        return Path(uri.removeprefix("sqlite://"))
    if uri.startswith("sqlite:///"):
        return Path("/") / uri.removeprefix("sqlite:///")
    return None


def relocate_legacy_sqlite_db() -> bool:
    """Copy legacy /app/pibdata.db to the configured URI path when target is missing.

    Returns True when a migration was performed, False when skipped.
    """
    uri = os.environ.get("SQLALCHEMY_DATABASE_URI", "")
    target = sqlite_uri_to_path(uri)
    if target is None:
        return False

    if target.is_file():
        return False

    legacy = LEGACY_DB_PATH
    if not legacy.is_file() or legacy.stat().st_size == 0:
        return False

    target.parent.mkdir(parents=True, exist_ok=True)

    source_conn = sqlite3.connect(str(legacy))
    dest_conn = sqlite3.connect(str(target))
    try:
        source_conn.backup(dest_conn)
        result = dest_conn.execute("PRAGMA integrity_check").fetchone()[0]
        if result != "ok":
            raise RuntimeError(f"relocated database integrity check failed: {result}")
    finally:
        dest_conn.close()
        source_conn.close()

    for path in (
        legacy,
        legacy.with_name(legacy.name + "-wal"),
        legacy.with_name(legacy.name + "-shm"),
    ):
        if path.is_file():
            path.unlink()

    return True


if __name__ == "__main__":
    migrated = relocate_legacy_sqlite_db()
    if migrated:
        print("relocate_legacy_sqlite_db: migrated legacy database to configured path")
