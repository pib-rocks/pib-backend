"""PR-2118: Gunicorn runtime and SQLite storage outside the Git checkout."""

from __future__ import annotations

import os
import sqlite3
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
FLASK_DIR = REPO_ROOT / "pib_api" / "flask"
COMPOSE_FILE = REPO_ROOT / "docker-compose.yaml"
UPDATE_RUNNER = REPO_ROOT / "setup" / "update_runner.sh"

sys.path.insert(0, str(FLASK_DIR))
import db_location  # noqa: E402


def _create_sqlite_with_table(path: Path, marker: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(path)
    try:
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("CREATE TABLE migration_marker (value TEXT NOT NULL)")
        connection.execute("INSERT INTO migration_marker (value) VALUES (?)", (marker,))
        connection.commit()
    finally:
        connection.close()


def _read_marker(path: Path) -> str:
    connection = sqlite3.connect(path)
    try:
        row = connection.execute("SELECT value FROM migration_marker").fetchone()
        return row[0]
    finally:
        connection.close()


class TestRelocateLegacySqliteDb:
    def test_migrates_legacy_database_and_removes_legacy_files(
        self, tmp_path, monkeypatch
    ):
        legacy = tmp_path / "app" / "pibdata.db"
        target = tmp_path / "data" / "pibdata.db"
        monkeypatch.setattr(db_location, "LEGACY_DB_PATH", legacy)
        monkeypatch.setenv("SQLALCHEMY_DATABASE_URI", f"sqlite:////{target}")

        _create_sqlite_with_table(legacy, "wal-safe-marker")
        wal_path = legacy.with_name(legacy.name + "-wal")
        wal_path.write_bytes(b"wal")

        assert db_location.relocate_legacy_sqlite_db() is True
        assert _read_marker(target) == "wal-safe-marker"
        assert not legacy.is_file()
        assert not wal_path.is_file()

    def test_no_op_when_target_already_exists(self, tmp_path, monkeypatch):
        legacy = tmp_path / "app" / "pibdata.db"
        target = tmp_path / "data" / "pibdata.db"
        monkeypatch.setattr(db_location, "LEGACY_DB_PATH", legacy)
        monkeypatch.setenv("SQLALCHEMY_DATABASE_URI", f"sqlite:////{target}")

        _create_sqlite_with_table(legacy, "legacy")
        _create_sqlite_with_table(target, "existing")

        assert db_location.relocate_legacy_sqlite_db() is False
        assert legacy.is_file()
        assert _read_marker(target) == "existing"


class TestComposeAndPackaging:
    def test_requirements_include_gunicorn(self):
        requirements = (FLASK_DIR / "requirements.txt").read_text(encoding="utf-8")
        assert "gunicorn" in requirements

    def test_flask_app_service_uses_gunicorn_and_data_mount(self):
        compose = yaml.safe_load(COMPOSE_FILE.read_text(encoding="utf-8"))
        flask = compose["services"]["flask-app"]
        command = flask["command"]
        assert "gunicorn --workers 1 --threads 8" in command
        assert "python3 -m db_location" in command
        env = flask["environment"]
        assert "SQLALCHEMY_DATABASE_URI=sqlite:////data/pibdata.db" in env
        assert any(
            v.rstrip("/") == "/home/pib/app/data:/data" for v in flask["volumes"]
        )


class TestUpdateRunnerBackupDatabase:
    def test_backup_database_resolves_sqlalchemy_uri(self):
        script = UPDATE_RUNNER.read_text(encoding="utf-8")
        start = script.index("backup_database()")
        end = script.index("\n\n# Prints the APP_VERSION", start)
        block = script[start:end]
        assert 'os.environ.get("SQLALCHEMY_DATABASE_URI"' in block
        assert "/data/pibdata.db" in block
        assert "/app/pibdata.db" in block
        assert 'sqlite3.connect("/app/pibdata.db")' not in block
