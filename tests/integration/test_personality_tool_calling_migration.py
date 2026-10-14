"""Existing personalities gain tool calling, switched on."""

import os
import sqlite3
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
FLASK_DIR = REPO_ROOT / "pib_api" / "flask"
IMPORT_DIRS = (
    FLASK_DIR,
    REPO_ROOT / "pib_blockly" / "pib_blockly_client",
    REPO_ROOT / "pib_api" / "client",
    REPO_ROOT / "public_api_client",
    REPO_ROOT / "pib_hermes_config",
)


def _upgrade(database: Path, revision: str) -> None:
    environment = os.environ.copy()
    environment["SQLALCHEMY_DATABASE_URI"] = f"sqlite:///{database}"
    environment["PYTHONPATH"] = os.pathsep.join(map(str, IMPORT_DIRS))
    environment["PYTHON_CODE_DIR"] = str(database.parent / "programs")
    environment["TRYB_URL_PREFIX"] = "http://localhost/test"
    subprocess.run(
        [sys.executable, "-m", "flask", "--app", "run", "db", "upgrade", revision],
        cwd=FLASK_DIR,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )


def test_upgrade_defaults_existing_personalities_to_tool_calling(tmp_path):
    database = tmp_path / "existing.db"
    (tmp_path / "programs").mkdir()
    _upgrade(database, "d7c1e4a90b22")

    with sqlite3.connect(database) as connection:
        connection.execute("""
            INSERT INTO personality (
                id, name, personality_id, gender, description, pause_threshold,
                message_history, assistant_model_id, stt_engine, provider_ref,
                channel
            )
            VALUES (
                31, 'Existing', 'person-tools', 'Female', 'the one identity',
                0.8, 5, NULL, 'local_whisper', 'default', 'direct'
            )
            """)
        connection.commit()

    _upgrade(database, "head")

    with sqlite3.connect(database) as connection:
        row = connection.execute("""
            SELECT channel, description, tool_calling
            FROM personality WHERE id = 31
            """).fetchone()

    assert row[0] == "direct"
    assert row[1] == "the one identity"
    assert row[2] == 1
