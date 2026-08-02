"""Unit tests for SQLite WAL mode and busy_timeout under concurrency (PR-1517)."""

from __future__ import annotations

import sqlite3
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

from sqlalchemy import text

from app.app import db
from config import Config


def test_config_sets_sqlite_connect_timeout():
    options = Config.SQLALCHEMY_ENGINE_OPTIONS
    assert options["connect_args"]["timeout"] == 15.0


def test_sqlite_connection_uses_wal_and_busy_timeout(app):
    with app.app_context():
        with db.engine.connect() as conn:
            journal_mode = conn.execute(text("PRAGMA journal_mode")).scalar()
            busy_timeout = conn.execute(text("PRAGMA busy_timeout")).scalar()

    assert str(journal_mode).lower() == "wal"
    assert int(busy_timeout) == 15000


def test_concurrent_multi_threaded_writes_do_not_raise_database_is_locked(app):
    """Many threads writing at once should wait on locks instead of failing."""
    uri = app.config["SQLALCHEMY_DATABASE_URI"]
    assert uri.startswith("sqlite:///"), uri
    db_path = uri.replace("sqlite:///", "", 1)

    with app.app_context():
        with db.engine.begin() as conn:
            conn.execute(
                text(
                    "CREATE TABLE IF NOT EXISTS concurrency_probe ("
                    "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                    "thread_id INTEGER NOT NULL, "
                    "seq INTEGER NOT NULL)"
                )
            )

    errors: list[BaseException] = []
    lock = threading.Lock()
    writers = 8
    rows_per_writer = 25

    def _write(thread_id: int) -> int:
        # Use the same engine the Flask app configured (WAL + busy_timeout).
        written = 0
        for seq in range(rows_per_writer):
            with app.app_context():
                with db.engine.begin() as conn:
                    conn.execute(
                        text(
                            "INSERT INTO concurrency_probe "
                            "(thread_id, seq) VALUES (:thread_id, :seq)"
                        ),
                        {"thread_id": thread_id, "seq": seq},
                    )
            written += 1
        return written

    with ThreadPoolExecutor(max_workers=writers) as pool:
        futures = [pool.submit(_write, i) for i in range(writers)]
        for future in as_completed(futures):
            try:
                future.result()
            except BaseException as exc:  # noqa: BLE001 — collect all failures
                with lock:
                    errors.append(exc)

    assert errors == [], f"concurrent writes raised: {errors!r}"

    with app.app_context():
        engine_db_path = db.engine.url.database
        with db.engine.connect() as conn:
            count = conn.execute(
                text("SELECT COUNT(*) FROM concurrency_probe")
            ).scalar()
            journal_mode = conn.execute(text("PRAGMA journal_mode")).scalar()
            busy_timeout = conn.execute(text("PRAGMA busy_timeout")).scalar()

    assert engine_db_path == db_path
    assert count == writers * rows_per_writer
    assert str(journal_mode).lower() == "wal"
    assert int(busy_timeout) == 15000

    # Confirm the on-disk DB is actually in WAL mode for other connections too.
    with sqlite3.connect(engine_db_path, timeout=15.0) as raw:
        mode = raw.execute("PRAGMA journal_mode").fetchone()[0]
    assert str(mode).lower() == "wal"
