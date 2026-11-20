"""Unit tests for Flask SQLite WAL auto-checkpoint, truncation, and session teardown."""

from __future__ import annotations

import os
import sqlite3
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
FLASK_ROOT = REPO_ROOT / "pib_api" / "flask"


def test_sqlite_wal_pragmas_checkpoint_and_teardown(tmp_path: Path, monkeypatch):
    db_file = tmp_path / "test_pibdata.db"
    code_dir = tmp_path / "python_code"
    code_dir.mkdir()

    monkeypatch.setenv("PIB_DATABASE_PATH", str(db_file))
    monkeypatch.setenv("PYTHON_CODE_DIR", str(code_dir))

    # Ensure fresh import of flask app bound to temp SQLite file
    if str(FLASK_ROOT) not in sys.path:
        sys.path.insert(0, str(FLASK_ROOT))

    # Import only the SQLite helpers/event logic on a standalone SQLAlchemy app
    from flask import Flask
    from flask_sqlalchemy import SQLAlchemy
    from sqlalchemy import event, text
    from sqlalchemy.engine import Engine

    # Load app.py source constants/functions cleanly without mutating global prod DB
    import importlib.util

    # Temporarily stub config.Config pointing to tmp_path so app.py never touches repo DB
    import types

    fake_config = types.ModuleType("config")

    class _TmpConfig:
        SQLALCHEMY_DATABASE_URI = f"sqlite:///{db_file}"
        SQLALCHEMY_TRACK_MODIFICATIONS = False
        PYTHON_CODE_DIR = str(code_dir)

    fake_config.Config = _TmpConfig  # type: ignore[attr-defined]
    fake_commands = types.ModuleType("commands")
    import click

    @click.command("seed-db")
    def _noop1():
        pass

    @click.command("seed-hardware")
    def _noop2():
        pass

    @click.command("reconcile-system-properties")
    def _noop3():
        pass

    fake_commands.seed_db = _noop1  # type: ignore[attr-defined]
    fake_commands.seed_hardware = _noop2  # type: ignore[attr-defined]
    fake_commands.reconcile_system_properties = _noop3  # type: ignore[attr-defined]

    monkeypatch.setitem(sys.modules, "config", fake_config)
    monkeypatch.setitem(sys.modules, "commands", fake_commands)

    spec = importlib.util.spec_from_file_location(
        "pib_flask_app_wal_test", FLASK_ROOT / "app" / "app.py"
    )
    assert spec and spec.loader
    app_mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = app_mod
    spec.loader.exec_module(app_mod)

    flask_app = app_mod.app
    db = app_mod.db

    with flask_app.app_context():
        # Verify WAL pragmas on SQLAlchemy engine connections
        journal_mode = db.session.execute(text("PRAGMA journal_mode")).scalar()
        busy_timeout = db.session.execute(text("PRAGMA busy_timeout")).scalar()
        autocheckpoint = db.session.execute(text("PRAGMA wal_autocheckpoint")).scalar()
        size_limit = db.session.execute(text("PRAGMA journal_size_limit")).scalar()

        assert str(journal_mode).lower() == "wal"
        assert int(busy_timeout) == app_mod.SQLITE_BUSY_TIMEOUT_MS
        assert int(autocheckpoint) == app_mod.SQLITE_WAL_AUTOCHECKPOINT_PAGES
        assert int(size_limit) == app_mod.SQLITE_JOURNAL_SIZE_LIMIT_BYTES

        # Generate WAL frames via table writes
        db.session.execute(
            text(
                "CREATE TABLE IF NOT EXISTS wal_probe (id INTEGER PRIMARY KEY, payload TEXT)"
            )
        )
        for i in range(40):
            db.session.execute(
                text("INSERT INTO wal_probe (payload) VALUES (:p)"),
                {"p": f"row-{i}-" + ("x" * 200)},
            )
        db.session.commit()

    # Simulate an aborted request leaving an uncommitted transaction; teardown must release it
    @flask_app.route("/_test_boom")
    def _test_boom():
        db.session.execute(text("SELECT COUNT(*) FROM wal_probe")).scalar()
        raise RuntimeError("simulated request failure")

    client = flask_app.test_client()
    resp = client.get("/_test_boom")
    assert resp.status_code == 500

    # Because teardown released the session, TRUNCATE checkpoint must succeed with busy == 0
    wal_path = Path(f"{db_file}-wal")
    result = app_mod.checkpoint_sqlite_wal(flask_app, mode="TRUNCATE")
    assert result is not None
    busy, _log_frames, _checkpointed = result
    assert busy == 0
    assert not wal_path.exists() or wal_path.stat().st_size == 0

    # Verify rows are durably flushed into the main .db file
    direct = sqlite3.connect(db_file)
    try:
        count = direct.execute("SELECT COUNT(*) FROM wal_probe").fetchone()[0]
        assert count == 40
    finally:
        direct.close()
