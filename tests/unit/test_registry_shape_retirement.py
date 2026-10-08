"""Personalities keep their model when the registry is a provider with many models.

A row the catalogue no longer lists is deleted. The personality keeps the
reference it had, so the existing retirement path asks for a new model and
refuses the chat. The reference is not rewritten onto a sibling model, onto
the provider, or onto the default.
"""

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

SPLIT_REVISION = "b6d4f2a81c30"

#: Ids a personality already stores. The migration must not hand these to a
#: catalogue row that it inserts afterwards.
REMOVED_MODEL_IDS = (12, 13, 14)


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


def _flags(**overrides: bool) -> str:
    flags = {
        "tools": False,
        "images": False,
        "live": False,
        "stt": False,
        "tts": False,
    }
    flags.update(overrides)
    return json.dumps(flags)


def _insert_personality(
    connection: sqlite3.Connection,
    row_id: int,
    name: str,
    description: str,
    model_id: int | None,
    provider_ref: str,
    voice_mode: str,
) -> None:
    connection.execute(
        """
        INSERT INTO personality (
            id, name, personality_id, gender, description, pause_threshold,
            message_history, assistant_model_id, stt_engine, tts_engine,
            provider_ref, channel, tool_calling, voice_mode, live_idle_timeout
        )
        VALUES (
            ?, ?, ?, 'Female', ?, 0.8,
            5, ?, 'local_whisper', 'supertone',
            ?, 'smart', 1, ?, 60
        )
        """,
        (
            row_id,
            name,
            f"person-{row_id}",
            description,
            model_id,
            provider_ref,
            voice_mode,
        ),
    )


def test_personalities_keep_their_model_and_a_gone_row_is_not_remapped(tmp_path):
    """A database already on the split schema. Head must retire, not reassign."""
    database = tmp_path / "split.db"
    (tmp_path / "programs").mkdir()
    _upgrade(database, SPLIT_REVISION)

    flash = _flags(tools=True, images=True)
    live = _flags(tools=True, live=True)
    gpt6 = _flags(tools=True, images=True)
    cloud = _flags(tools=True, images=True)
    with sqlite3.connect(database) as connection:
        connection.execute(
            """
            INSERT INTO provider (id, name, endpoint_base, capabilities, credential_ref)
            VALUES
                (1, 'Google', NULL, ?, 'google-kept'),
                (2, 'OpenAI', NULL, ?, 'openai-kept'),
                (3, 'pib.Cloud', NULL, ?, NULL),
                (12, 'GPT-4o [Text]', NULL, ?, 'orphan-key'),
                (14, 'Hermes Agent (selbstlernend)', NULL, ?, NULL)
            """,
            (
                _flags(tools=True, images=True, live=True),
                _flags(),
                _flags(tools=True, images=True),
                _flags(),
                _flags(),
            ),
        )
        connection.executemany(
            """
            INSERT INTO assistant_model (id, api_name, visual_name, has_image_support)
            VALUES (?, ?, ?, ?)
            """,
            [
                (7, "gemini-3.8-flash", "Gemini 3.8 Flash", 1),
                (8, "gemini-3.8-live", "Gemini 3.8 Live", 0),
                (9, "gpt-6", "GPT-6", 1),
                (10, "pib-cloud", "pib.Cloud", 1),
                (12, "gpt-4o", "GPT-4o [Text]", 0),
                (13, "gpt-3.5-turbo", "GPT-3.5 [Text]", 0),
                (14, "hermes-agent", "Hermes Agent (selbstlernend)", 1),
            ],
        )
        connection.executemany(
            """
            INSERT INTO registry_model (
                id, provider_id, api_name, visual_name, has_image_support,
                capabilities, is_default, live_model, live_model_checked_on
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, NULL, NULL)
            """,
            [
                (7, 1, "gemini-3.8-flash", "Gemini 3.8 Flash", 1, flash, 0),
                (8, 1, "gemini-3.8-live", "Gemini 3.8 Live", 0, live, 0),
                (9, 2, "gpt-6", "GPT-6", 1, gpt6, 0),
                (10, 3, "pib-cloud", "pib.Cloud", 1, cloud, 1),
                (12, 12, "gpt-4o", "GPT-4o [Text]", 0, _flags(), 0),
                (13, 2, "gpt-3.5-turbo", "GPT-3.5 [Text]", 0, _flags(), 0),
                (
                    14,
                    14,
                    "hermes-agent",
                    "Hermes Agent (selbstlernend)",
                    1,
                    _flags(),
                    0,
                ),
            ],
        )
        _insert_personality(connection, 31, "OnFlash", "on-flash", 7, "7", "turn_based")
        _insert_personality(connection, 32, "OnLive", "on-live", 8, "8", "live")
        _insert_personality(
            connection, 33, "OnDefault", "on-default", None, "default", "turn_based"
        )
        _insert_personality(
            connection, 34, "OnGpt4o", "on-gpt4o", 12, "12", "turn_based"
        )
        _insert_personality(
            connection, 35, "OnTurbo", "on-turbo", 13, "13", "turn_based"
        )
        _insert_personality(
            connection, 36, "OnHermes", "on-hermes", 14, "14", "turn_based"
        )
        connection.commit()

    _upgrade(database, "head")

    with sqlite3.connect(database) as connection:
        people = {row[0]: row[1:] for row in connection.execute("""
                SELECT id, assistant_model_id, provider_ref, description, voice_mode
                FROM personality
                """)}
        models = {row[1]: row for row in connection.execute("""
                SELECT id, api_name, provider_id, capabilities, is_default, visual_name
                FROM registry_model
                """)}
        accounts = {
            row[0]: row
            for row in connection.execute(
                "SELECT id, name, credential_ref, capabilities FROM provider"
            )
        }
        assistant_names = {
            row[0] for row in connection.execute("SELECT api_name FROM assistant_model")
        }

    # Each personality still names the model it used. Nothing was collapsed
    # onto the other Gemini row, and 'default' is still a pointer.
    assert people[31] == (7, "7", "on-flash", "turn_based")
    assert people[32] == (8, "8", "on-live", "live")
    assert people[33] == (None, "default", "on-default", "turn_based")
    assert models["gemini-3.8-flash"][0] == 7
    assert models["gemini-3.8-live"][0] == 8
    assert models["gemini-3.8-flash"][2] == models["gemini-3.8-live"][2] == 1
    assert models["gpt-6"][0] == 9
    assert models["pib-cloud"][0] == 10
    assert [name for name, row in models.items() if row[4] == 1] == ["pib-cloud"]

    # The row is gone. The reference stays, which is what refuses the chat
    # instead of starting it on another model.
    assert people[34] == (None, "12", "on-gpt4o", "turn_based")
    assert people[35] == (None, "13", "on-turbo", "turn_based")
    assert people[36] == (None, "14", "on-hermes", "turn_based")
    remaining_ids = {row[0] for row in models.values()}
    assert set(REMOVED_MODEL_IDS).isdisjoint(remaining_ids)
    for gone in (people[34], people[35], people[36]):
        assert gone[1] not in {str(model_id) for model_id in remaining_ids}
        assert gone[1] != "default"

    from provider_registry import active_api_names, capabilities_for

    assert set(models) == set(active_api_names()) == assistant_names
    claude = models["claude-sonnet-5-5"]
    assert claude[0] not in REMOVED_MODEL_IDS
    assert claude[0] == 15
    assert claude[5] == "Claude Sonnet 5.5"
    assert json.loads(claude[3]) == capabilities_for("claude-sonnet-5-5", True)
    assert accounts[claude[2]][1] == "Anthropic"

    google = accounts[1]
    openai = accounts[2]
    assert google[1:][:2] == ("Google", "google-kept")
    assert openai[1:][:2] == ("OpenAI", "openai-kept")
    assert json.loads(google[3]) == {
        "tools": True,
        "images": False,
        "live": False,
        "stt": False,
        "tts": False,
    }
    assert json.loads(openai[3]) == json.loads(models["gpt-6"][3])
    assert json.loads(models["gpt-6"][3])["tools"] is True
    assert json.loads(models["gpt-6"][3])["images"] is True
    assert "GPT-4o [Text]" not in {row[1] for row in accounts.values()}
    assert "Hermes Agent (selbstlernend)" not in {row[1] for row in accounts.values()}
