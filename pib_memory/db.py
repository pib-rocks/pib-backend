"""SQLite schema and connection helpers for episodic memory."""

from __future__ import annotations

import sqlite3
from pathlib import Path

SCHEMA_VERSION = 1

_BUSY_TIMEOUT_MS = 15000

_DDL = """
CREATE TABLE IF NOT EXISTS meta (
    key TEXT PRIMARY KEY,
    value TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS episodes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts_ms INTEGER NOT NULL,
    place_id TEXT,
    participants_json TEXT NOT NULL DEFAULT '[]',
    task TEXT,
    outcome TEXT,
    failure_reason TEXT,
    evidence_ref TEXT NOT NULL,
    consent_scope TEXT,
    created_ms INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS object_memory (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    object_id TEXT NOT NULL UNIQUE,
    label TEXT,
    place_id TEXT,
    last_seen_ms INTEGER NOT NULL,
    confidence REAL NOT NULL DEFAULT 0.0,
    frame_ref TEXT
);

CREATE TABLE IF NOT EXISTS person_memory (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    display_name TEXT NOT NULL UNIQUE,
    embedding_ref TEXT,
    consent TEXT,
    preferences_json TEXT NOT NULL DEFAULT '{}',
    created_ms INTEGER NOT NULL
);
"""


def connect(db_path: str | Path) -> sqlite3.Connection:
    path = Path(db_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(path), check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=ON")
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute(f"PRAGMA busy_timeout={_BUSY_TIMEOUT_MS}")
    return conn


def init_schema(conn: sqlite3.Connection) -> None:
    conn.executescript(_DDL)
    conn.execute(
        """
        INSERT INTO meta(key, value) VALUES ('schema_version', ?)
        ON CONFLICT(key) DO NOTHING
        """,
        (str(SCHEMA_VERSION),),
    )
    conn.commit()
