"""PC-109: pib.Cloud speaks too, and the catalogue carries a reasoning flag.

A personality on the pib.Cloud row defaults to cloud whisper for STT and
cloud speech for TTS. An explicit choice is stored as sent. Existing rows
gain the new reasoning flag through the migration and through seed_db.
"""

from unittest.mock import MagicMock

import pytest
from click.testing import CliRunner

from commands import seed_db
from model.personality_model import Personality
from model.provider_model import Provider, RegistryModel
from pib_hermes_config.local_model import model_capabilities
from pib_hermes_config.voice_backends import (
    CLOUD_STT_ID,
    CLOUD_TTS_ID,
    LOCAL_STT_ID,
    LOCAL_TTS_ID,
    normalize_tts_choice,
    resolve_stt_route,
    resolve_tts_route,
)
from provider_registry import (
    CAPABILITY_KEYS,
    CATALOGUE,
    PIB_CLOUD_API_NAME,
    capabilities_for,
)
from service import personality_service, provider_service

PERSONALITY_URL = "/voice-assistant/personality"
EVA_PERSONALITY_ID = "8f73b580-927e-41c2-98ac-e5df070e7288"


@pytest.fixture(autouse=True)
def profile_factory_is_up(monkeypatch):
    monkeypatch.setattr(
        personality_service, "_provision_profile", lambda *_a, **_k: {"ok": True}
    )


@pytest.fixture()
def client(app):
    return app.test_client()


def _row_id(api_name: str) -> int:
    return RegistryModel.query.filter_by(api_name=api_name).one().id


def _never(_model_id: int) -> bool:
    return False


def _create(client, **body):
    return client.post(PERSONALITY_URL, json={"name": "CloudSpeech", **body})


# --- catalogue -----------------------------------------------------------------


def test_pib_cloud_carries_both_speech_flags_and_no_reasoning():
    cloud = next(entry for entry in CATALOGUE if entry.api_name == PIB_CLOUD_API_NAME)
    assert cloud.stt is True
    assert cloud.tts is True
    assert cloud.reasoning is False
    flags = capabilities_for(PIB_CLOUD_API_NAME, True)
    assert flags["stt"] is True
    assert flags["tts"] is True
    assert flags["reasoning"] is False
    assert set(flags) == set(CAPABILITY_KEYS)


def test_the_other_cloud_chat_models_own_their_reasoning_effort():
    by_name = {entry.api_name: entry for entry in CATALOGUE if entry.api_name}
    for api_name in ("gemini-3.8-flash", "gpt-6", "claude-sonnet-5-5"):
        assert by_name[api_name].reasoning is True, api_name
        assert capabilities_for(api_name, True)["reasoning"] is True, api_name
    # A live model and the unconfirmed lines carry no reasoning flag.
    assert by_name["gemini-3.8-live"].reasoning is False
    assert by_name["gpt-realtime"].reasoning is False


def test_the_on_device_row_carries_no_reasoning_flag():
    assert model_capabilities()["reasoning"] is False
    assert capabilities_for("qwen-fast", False)["reasoning"] is False


# --- engine id round-trip ------------------------------------------------------


def test_cloud_speech_is_accepted_only_while_pib_cloud_has_tts():
    assert normalize_tts_choice(CLOUD_TTS_ID, _never, 5) == CLOUD_TTS_ID
    with pytest.raises(ValueError, match="pib.Cloud"):
        normalize_tts_choice(CLOUD_TTS_ID, _never, None)


def test_the_bare_pib_cloud_row_id_is_stored_as_cloud_speech():
    assert normalize_tts_choice("5", lambda model_id: model_id == 5, 5) == CLOUD_TTS_ID
    # Without the capability the bare id is an ordinary rejected row.
    with pytest.raises(ValueError):
        normalize_tts_choice("5", _never, None)


def test_local_and_other_capable_rows_keep_their_meaning():
    assert normalize_tts_choice(LOCAL_TTS_ID, _never, 5) == LOCAL_TTS_ID
    assert normalize_tts_choice("supertone", _never, 5) == LOCAL_TTS_ID
    assert normalize_tts_choice("7", lambda model_id: model_id == 7, 5) == "7"
    assert normalize_tts_choice(None, _never, 5, default=CLOUD_TTS_ID) == CLOUD_TTS_ID


def test_cloud_speech_resolves_to_the_cloud_route():
    assert resolve_tts_route(CLOUD_TTS_ID) == "cloud"
    assert resolve_tts_route(LOCAL_TTS_ID) == "local"
    assert resolve_tts_route("") == "local"
    assert resolve_tts_route("5") == "unsupported"
    # The whisper id is not a text-to-speech engine.
    assert resolve_tts_route(CLOUD_STT_ID) == "unsupported"


# --- personality defaults ------------------------------------------------------


@pytest.mark.parametrize("reference", ["policy", "modelRef", "providerRef", "id"])
def test_a_new_personality_on_pib_cloud_stores_both_cloud_engines(
    client, app, reference
):
    with app.app_context():
        cloud_id = _row_id(PIB_CLOUD_API_NAME)
    body = {
        "policy": {},
        "modelRef": {"modelRef": f"model:{cloud_id}"},
        "providerRef": {"providerRef": str(cloud_id)},
        "id": {"assistantModelId": cloud_id},
    }[reference]
    created = _create(client, **body)
    assert created.status_code == 201
    stored = created.get_json()
    assert stored["sttEngine"] == CLOUD_STT_ID
    assert stored["ttsEngine"] == CLOUD_TTS_ID
    assert resolve_stt_route(stored["sttEngine"]) == "cloud"
    assert resolve_tts_route(stored["ttsEngine"]) == "cloud"


def test_a_true_cloud_row_carries_cloud_speech(client, app):
    with app.app_context():
        cloud_id = _row_id(PIB_CLOUD_API_NAME)
        assert provider_service.cloud_tts_model_id() == cloud_id
        assert provider_service.cloud_stt_model_id() == cloud_id
    response = _create(client, modelRef=f"model:{cloud_id}")
    assert response.status_code == 201
    assert response.get_json()["ttsEngine"] == CLOUD_TTS_ID


def test_another_model_keeps_local_speech(client, app):
    with app.app_context():
        gpt6_id = _row_id("gpt-6")
    created = _create(client, modelRef=f"model:{gpt6_id}")
    assert created.status_code == 201
    assert created.get_json()["sttEngine"] == LOCAL_STT_ID
    assert created.get_json()["ttsEngine"] == LOCAL_TTS_ID


@pytest.mark.parametrize("local", [LOCAL_TTS_ID, "supertone"])
def test_an_explicit_local_choice_on_pib_cloud_is_stored_as_sent(client, app, local):
    with app.app_context():
        cloud_id = _row_id(PIB_CLOUD_API_NAME)
    created = _create(client, modelRef=f"model:{cloud_id}", ttsEngine=local)
    assert created.status_code == 201
    assert created.get_json()["ttsEngine"] == LOCAL_TTS_ID
    assert resolve_tts_route(created.get_json()["ttsEngine"]) == "local"


def test_an_explicit_local_choice_survives_unrelated_edits(client, app):
    created = _create(client, ttsEngine=LOCAL_TTS_ID).get_json()
    url = f"{PERSONALITY_URL}/{created['personalityId']}"
    assert created["ttsEngine"] == LOCAL_TTS_ID
    for edit in ({"pauseThreshold": 1.2}, {"description": "new soul text"}):
        response = client.put(url, json=edit)
        assert response.status_code == 200, edit
        assert response.get_json()["ttsEngine"] == LOCAL_TTS_ID, edit
    with app.app_context():
        stored = Personality.query.filter_by(
            personality_id=created["personalityId"]
        ).one()
        assert stored.tts_engine == LOCAL_TTS_ID


def test_the_bare_pib_cloud_id_is_sent_back_as_cloud_speech(client, app):
    with app.app_context():
        cloud_id = _row_id(PIB_CLOUD_API_NAME)
        gpt6_id = _row_id("gpt-6")
    created = _create(client, modelRef=f"model:{gpt6_id}", ttsEngine=str(cloud_id))
    assert created.status_code == 201
    assert created.get_json()["ttsEngine"] == CLOUD_TTS_ID


def test_a_blank_update_resets_to_the_default_of_the_chosen_model(client, app):
    with app.app_context():
        cloud_id = _row_id(PIB_CLOUD_API_NAME)
    created = _create(client, ttsEngine=LOCAL_TTS_ID).get_json()
    url = f"{PERSONALITY_URL}/{created['personalityId']}"
    reset = client.put(url, json={"ttsEngine": ""})
    assert reset.status_code == 200
    assert reset.get_json()["ttsEngine"] == CLOUD_TTS_ID
    with app.app_context():
        assert provider_service.cloud_tts_model_id() == cloud_id


def test_an_existing_personality_keeps_its_stored_engine(client, app):
    body = client.get(f"{PERSONALITY_URL}/{EVA_PERSONALITY_ID}").get_json()
    assert body["ttsEngine"] == LOCAL_TTS_ID
    edited = client.put(
        f"{PERSONALITY_URL}/{EVA_PERSONALITY_ID}", json={"pauseThreshold": 0.9}
    )
    assert edited.status_code == 200
    assert edited.get_json()["ttsEngine"] == LOCAL_TTS_ID


# --- stored rows gain the new flag ---------------------------------------------


def test_seed_db_refreshes_the_reasoning_flag_on_existing_rows(app):
    """A fresh seed is not enough: a stored row gains reasoning idempotently."""
    with app.app_context():
        from app.app import db

        cloud = RegistryModel.query.filter_by(api_name=PIB_CLOUD_API_NAME).one()
        cloud.capabilities = {
            key: value
            for key, value in cloud.capabilities.items()
            if key != "reasoning"
        }
        gpt6 = RegistryModel.query.filter_by(api_name="gpt-6").one()
        gpt6.capabilities = {
            key: value for key, value in gpt6.capabilities.items() if key != "reasoning"
        }
        provider = Provider.query.filter_by(name="OpenAI").one()
        provider.capabilities = {
            key: value
            for key, value in provider.capabilities.items()
            if key != "reasoning"
        }
        db.session.commit()
        assert "reasoning" not in cloud.capabilities
        assert "reasoning" not in provider.capabilities

        result = CliRunner().invoke(seed_db, [])
        assert result.exception is None, result.output
        db.session.remove()

        cloud = RegistryModel.query.filter_by(api_name=PIB_CLOUD_API_NAME).one()
        assert cloud.capabilities["reasoning"] is False
        assert cloud.capabilities["tts"] is True
        assert cloud.capabilities["stt"] is True
        assert (
            RegistryModel.query.filter_by(api_name="gpt-6")
            .one()
            .capabilities["reasoning"]
            is True
        )
        assert (
            Provider.query.filter_by(name="OpenAI").one().capabilities["reasoning"]
            is True
        )
        assert (
            Provider.query.filter_by(name="pib.Cloud").one().capabilities["reasoning"]
            is False
        )


def test_seed_db_reaches_every_catalogue_row_and_keeps_personalities(app):
    with app.app_context():
        from app.app import db

        before = {row.api_name: row.capabilities for row in RegistryModel.query.all()}
        for row in RegistryModel.query.all():
            row.capabilities = {
                key: value
                for key, value in row.capabilities.items()
                if key != "reasoning"
            }
        engines_before = {
            p.personality_id: (p.stt_engine, p.tts_engine)
            for p in Personality.query.all()
        }
        db.session.commit()

        result = CliRunner().invoke(seed_db, [])
        assert result.exception is None, result.output
        db.session.remove()

        for row in RegistryModel.query.all():
            assert set(row.capabilities) >= set(CAPABILITY_KEYS), row.api_name
            assert row.capabilities["reasoning"] == before[row.api_name]["reasoning"]
        assert {
            p.personality_id: (p.stt_engine, p.tts_engine)
            for p in Personality.query.all()
        } == engines_before
