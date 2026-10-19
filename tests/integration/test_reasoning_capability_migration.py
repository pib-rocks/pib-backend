"""An existing database gains the reasoning flag on its stored speech rows."""

import json
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

OLD_FLAGS = {
    "tools": True,
    "images": True,
    "live": False,
    "stt": True,
    "tts": False,
}


def _run_flask(database: Path, *args: str) -> None:
    environment = os.environ.copy()
    environment["SQLALCHEMY_DATABASE_URI"] = f"sqlite:///{database}"
    environment["PYTHONPATH"] = os.pathsep.join(map(str, IMPORT_DIRS))
    environment["PYTHON_CODE_DIR"] = str(database.parent / "programs")
    environment["TRYB_URL_PREFIX"] = "http://localhost/test"
    subprocess.run(
        [sys.executable, "-m", "flask", "--app", "run", "db", *args],
        cwd=FLASK_DIR,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )


def _insert_rows(database: Path) -> None:
    with sqlite3.connect(database) as connection:
        connection.execute(
            """
            INSERT INTO provider (id, name, endpoint_base, credential_ref, capabilities)
            VALUES
                (1, 'pib.Cloud', NULL, NULL, :cloud),
                (2, 'Local', NULL, NULL, :local)
            """,
            {
                "cloud": json.dumps(OLD_FLAGS),
                "local": json.dumps(
                    {
                        "tools": False,
                        "images": False,
                        "live": False,
                        "stt": False,
                        "tts": False,
                    }
                ),
            },
        )
        connection.execute(
            """
            INSERT INTO registry_model (
                id, provider_id, api_name, visual_name, has_image_support,
                capabilities, is_default
            )
            VALUES
                (1, 1, 'pib-cloud', 'pib.Cloud', 1, :cloud, 1),
                (2, 2, 'qwen-fast', 'Local (qwen-fast)', 0, :local, 0)
            """,
            {
                "cloud": json.dumps(OLD_FLAGS),
                "local": json.dumps(
                    {
                        "tools": False,
                        "images": False,
                        "live": False,
                        "stt": False,
                        "tts": False,
                        "offline": True,
                    }
                ),
            },
        )
        connection.commit()


def test_upgrade_adds_reasoning_to_stored_rows_and_keeps_the_offline_marker(
    tmp_path,
):
    database = tmp_path / "existing.db"
    (tmp_path / "programs").mkdir()
    _run_flask(database, "upgrade", "a7d2e5c81b34")
    _insert_rows(database)

    _run_flask(database, "upgrade", "head")

    with sqlite3.connect(database) as connection:
        models = {
            row[0]: json.loads(row[1])
            for row in connection.execute(
                "SELECT api_name, capabilities FROM registry_model"
            )
        }
        providers = {
            row[0]: json.loads(row[1])
            for row in connection.execute("SELECT name, capabilities FROM provider")
        }

    # pib.Cloud is a speech backend in both directions and sets its own effort.
    assert models["pib-cloud"]["stt"] is True
    assert models["pib-cloud"]["tts"] is True
    assert models["pib-cloud"]["reasoning"] is False
    # The on-device row carries the flag off and keeps its own marker.
    assert models["qwen-fast"]["reasoning"] is False
    assert models["qwen-fast"]["offline"] is True
    # The provider's shared flags follow its models.
    assert providers["pib.Cloud"] == {
        "tools": True,
        "images": True,
        "live": False,
        "stt": True,
        "tts": True,
        "reasoning": False,
    }
    assert providers["Local"]["reasoning"] is False
    assert "offline" not in providers["Local"]


def test_downgrade_removes_the_reasoning_flag(tmp_path):
    database = tmp_path / "existing.db"
    (tmp_path / "programs").mkdir()
    _run_flask(database, "upgrade", "a7d2e5c81b34")
    _insert_rows(database)
    _run_flask(database, "upgrade", "head")

    _run_flask(database, "downgrade", "a7d2e5c81b34")

    with sqlite3.connect(database) as connection:
        model_flags = [
            json.loads(row[0])
            for row in connection.execute("SELECT capabilities FROM registry_model")
        ]
        provider_flags = [
            json.loads(row[0])
            for row in connection.execute("SELECT capabilities FROM provider")
        ]
    assert all("reasoning" not in flags for flags in model_flags + provider_flags)
