"""Existing rows gain a null filler and a null first-token measurement."""

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


def test_upgrade_leaves_filler_and_latency_empty(tmp_path):
    database = tmp_path / "existing.db"
    (tmp_path / "programs").mkdir()
    _upgrade(database, "f1a9c3e74b20")

    with sqlite3.connect(database) as connection:
        connection.execute("""
            INSERT INTO personality (
                id, name, personality_id, gender, description, pause_threshold,
                message_history, assistant_model_id, stt_engine, tts_engine,
                provider_ref, channel, tool_calling, voice_mode, live_idle_timeout
            )
            VALUES (
                51, 'Existing', 'person-turn', 'Female', 'the one identity',
                0.8, 5, NULL, 'local_whisper', 'supertone',
                'default', 'smart', 1, 'live', 60
            )
            """)
        connection.execute("""
            INSERT INTO chat (id, chat_id, topic, personality_id)
            VALUES (7, 'chat-turn', 'topic', 'person-turn')
            """)
        connection.commit()

    _upgrade(database, "head")

    with sqlite3.connect(database) as connection:
        personality = connection.execute("""
            SELECT pause_threshold, thinking_filler
            FROM personality WHERE id = 51
            """).fetchone()
        chat = connection.execute("""
            SELECT first_token_latency_ms FROM chat WHERE id = 7
            """).fetchone()

    assert personality[0] == 0.8
    assert personality[1] is None
    assert chat[0] is None
