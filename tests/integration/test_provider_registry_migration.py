"""Upgrade an existing database into the provider registry."""

import json
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


def test_upgrade_copies_assistant_model_ids_onto_the_provider_registry(tmp_path):
    database = tmp_path / "existing.db"
    (tmp_path / "programs").mkdir()
    _upgrade(database, "b17461748001")

    with sqlite3.connect(database) as connection:
        connection.execute("""
            INSERT INTO assistant_model (id, api_name, visual_name, has_image_support)
            VALUES
                (11, 'gpt-4o', 'GPT-4o [Vision]', 1),
                (12, 'gpt-4o', 'GPT-4o [Text]', 0),
                (13, 'gemini-3.5-flash', 'Gemini 3.5 Flash', 0),
                (14, 'hermes-agent', 'Hermes Agent (selbstlernend)', 1)
            """)
        connection.execute("""
            INSERT INTO personality (
                id, name, personality_id, gender, description, pause_threshold,
                message_history, assistant_model_id, stt_engine
            )
            VALUES (
                21, 'Existing', 'person-existing', 'Female', 'kept', 0.8,
                5, 12, 'local_whisper'
            )
            """)
        connection.commit()

    _upgrade(database, "head")

    with sqlite3.connect(database) as connection:
        providers = {row[0]: row for row in connection.execute("""
                SELECT id, api_name, visual_name, has_image_support,
                       capabilities, is_default, endpoint_base, credential_ref
                FROM provider
                ORDER BY id
                """)}
        personality = connection.execute("""
            SELECT assistant_model_id, provider_ref
            FROM personality WHERE id = 21
            """).fetchone()

    assert set(providers) == {11, 12, 13, 14}
    assert providers[12][1] == "gpt-4o"
    assert providers[12][2] == "GPT-4o [Text]"
    assert json.loads(providers[11][4])["images"] is True
    assert json.loads(providers[12][4])["images"] is False
    assert json.loads(providers[13][4])["images"] is False
    assert json.loads(providers[14][4])["images"] is True
    assert providers[14][5] == 1
    assert providers[11][5] == 0
    assert providers[11][6] is None
    assert providers[11][7] is None
    assert personality == (12, "12")

    from sqlalchemy import Column, Integer, JSON, create_engine
    from sqlalchemy.orm import DeclarativeBase, Session

    class Base(DeclarativeBase):
        pass

    class ProviderRow(Base):
        __tablename__ = "provider"
        id = Column(Integer, primary_key=True)
        capabilities = Column(JSON)

    engine = create_engine(f"sqlite:///{database}")
    with Session(engine) as session:
        vision = session.get(ProviderRow, 11)
        assert isinstance(vision.capabilities, dict)
        assert vision.capabilities["images"] is True
        assert vision.capabilities["tools"] is True
