import os
from pathlib import Path
import sqlite3
import subprocess
import sys

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
        [
            sys.executable,
            "-m",
            "flask",
            "--app",
            "run",
            "db",
            "upgrade",
            revision,
        ],
        cwd=FLASK_DIR,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )


def test_upgrade_existing_database_backfills_variant_and_preserves_rows(tmp_path):
    database = tmp_path / "existing.db"
    (tmp_path / "programs").mkdir()
    _upgrade(database, "a1638c0de001")

    with sqlite3.connect(database) as connection:
        connection.execute("""
            INSERT INTO controller (id, kind, number)
            VALUES (101, 'tinkerforge_bricklet', 101)
            """)
        connection.execute("""
            INSERT INTO pose (id, pose_id, name, deletable)
            VALUES (102, 'pose-existing', 'Existing pose', 1)
            """)
        connection.execute("""
            INSERT INTO program (id, name, code_visual, program_number)
            VALUES (103, 'existing-program', '{}', 'program-existing')
            """)
        connection.execute("""
            INSERT INTO chat (id, chat_id, topic, personality_id)
            VALUES (104, 'chat-existing', 'Existing chat', 'person-existing')
            """)
        before = {
            "controller": connection.execute(
                "SELECT id, kind, number FROM controller WHERE id = 101"
            ).fetchone(),
            "pose": connection.execute(
                "SELECT id, pose_id, name FROM pose WHERE id = 102"
            ).fetchone(),
            "program": connection.execute(
                "SELECT id, name, code_visual, program_number "
                "FROM program WHERE id = 103"
            ).fetchone(),
            "chat": connection.execute(
                "SELECT id, chat_id, topic, personality_id FROM chat WHERE id = 104"
            ).fetchone(),
        }

    _upgrade(database, "head")

    with sqlite3.connect(database) as connection:
        tables = {
            row[0]
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type = 'table'"
            )
        }
        variant = connection.execute("""
            SELECT key, value, value_type, source
            FROM system_property
            """).fetchall()
        after = {
            "controller": connection.execute(
                "SELECT id, kind, number FROM controller WHERE id = 101"
            ).fetchone(),
            "pose": connection.execute(
                "SELECT id, pose_id, name FROM pose WHERE id = 102"
            ).fetchone(),
            "program": connection.execute(
                "SELECT id, name, code_visual, program_number "
                "FROM program WHERE id = 103"
            ).fetchone(),
            "chat": connection.execute(
                "SELECT id, chat_id, topic, personality_id FROM chat WHERE id = 104"
            ).fetchone(),
        }

    assert "system_property" in tables
    assert variant == [("hardware.variant", "pib5edu", "str", "default")]
    assert after == before
