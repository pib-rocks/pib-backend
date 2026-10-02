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
            VALUES
                (
                    21, 'Existing', 'person-existing', 'Female', 'kept', 0.8,
                    5, 12, 'local_whisper'
                ),
                (
                    22, 'OnHermes', 'person-hermes', 'Male', 'kept', 0.8,
                    5, 14, 'local_whisper'
                )
            """)
        connection.commit()

    # The registry is created from the legacy rows first, keeping their ids.
    _upgrade(database, "c4e8a1b27d90")

    with sqlite3.connect(database) as connection:
        copied = {
            row[0]: row
            for row in connection.execute(
                "SELECT id, api_name, visual_name, has_image_support FROM provider"
            )
        }
        personality = connection.execute("""
            SELECT assistant_model_id, provider_ref
            FROM personality WHERE id = 21
            """).fetchone()
        on_hermes = connection.execute("""
            SELECT assistant_model_id, provider_ref
            FROM personality WHERE id = 22
            """).fetchone()
    assert set(copied) == {11, 12, 13, 14}
    assert copied[12][1:] == ("gpt-4o", "GPT-4o [Text]", 0)
    assert personality == (12, "12")
    assert on_hermes == (14, "14")

    # At head, only the catalogue's models are left. Nothing old survives.
    _upgrade(database, "head")

    with sqlite3.connect(database) as connection:
        providers = {row[1]: row for row in connection.execute("""
                SELECT id, api_name, visual_name, has_image_support,
                       capabilities, is_default, endpoint_base, credential_ref,
                       live_model, live_model_checked_on
                FROM provider
                ORDER BY id
                """)}
        assistant_models = {
            row[1]: row
            for row in connection.execute(
                "SELECT id, api_name, visual_name, has_image_support FROM assistant_model"
            )
        }
        personality = connection.execute("""
            SELECT assistant_model_id, provider_ref
            FROM personality WHERE id = 21
            """).fetchone()
        on_hermes = connection.execute("""
            SELECT assistant_model_id, provider_ref
            FROM personality WHERE id = 22
            """).fetchone()
        voice = connection.execute("""
            SELECT voice_mode, live_idle_timeout FROM personality WHERE id = 21
            """).fetchone()

    supported = {
        "gemini-3.8-flash",
        "gpt-6",
        "claude-sonnet-5-5",
        "pib-cloud",
    }
    assert set(providers) == supported
    assert set(assistant_models) == supported
    for api_name in supported:
        assert providers[api_name][0] == assistant_models[api_name][0]
        assert providers[api_name][2] == assistant_models[api_name][2]
        assert providers[api_name][6] is None
        assert providers[api_name][7] is None
    assert "hermes-agent" not in providers
    assert "hermes-agent" not in assistant_models
    assert {providers[name][0] for name in supported} & {11, 12, 13, 14} == set()
    assert [name for name, row in providers.items() if row[5] == 1] == ["pib-cloud"]
    assert providers["pib-cloud"][2] == "pib.Cloud"
    assert json.loads(providers["pib-cloud"][4])["images"] is True
    assert json.loads(providers["gemini-3.8-flash"][4])["live"] is True
    assert providers["gemini-3.8-flash"][8:] == (
        "gemini-3.8-live",
        "2026-10-01",
    )
    assert providers["gpt-6"][8:] == (None, None)
    # The personality is not moved onto another model. Its reference stays and
    # now points at nothing, which is what reports it as needing a new model.
    assert personality == (None, "12")
    # The personality that pointed at hermes-agent keeps that reference.
    assert on_hermes == (None, "14")
    assert voice == ("live", 60)

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
        pib_cloud = session.get(ProviderRow, providers["pib-cloud"][0])
        assert isinstance(pib_cloud.capabilities, dict)
        assert pib_cloud.capabilities["images"] is True
        assert pib_cloud.capabilities["tools"] is True
