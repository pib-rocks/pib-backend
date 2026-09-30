"""Catalogue consistency, and retirement of a personality's selected model."""

from unittest.mock import MagicMock

from model.chat_model import Chat
from model.provider_model import Provider
from service import personality_service

EVA_PERSONALITY_ID = "8f73b580-927e-41c2-98ac-e5df070e7288"


def test_catalogue_entries_are_consistent():
    from provider_registry import (
        CAPABILITY_KEYS,
        CATALOGUE,
        DEFAULT_PROVIDER_API_NAME,
        STATUS_ACTIVE,
        STATUS_UNCONFIRMED,
        is_retired,
    )

    active = [entry for entry in CATALOGUE if entry.status == STATUS_ACTIVE]
    assert active
    for entry in active:
        assert entry.api_name
        assert entry.visual_name
        flags = entry.capabilities()
        assert set(flags) == set(CAPABILITY_KEYS)
        assert all(isinstance(flags[key], bool) for key in CAPABILITY_KEYS)
    api_names = [entry.api_name for entry in CATALOGUE if entry.api_name]
    assert len(api_names) == len(set(api_names))
    defaults = [entry for entry in CATALOGUE if entry.is_default]
    assert len(defaults) == 1
    assert defaults[0].api_name == DEFAULT_PROVIDER_API_NAME
    assert defaults[0].status == STATUS_ACTIVE
    by_provider = {entry.provider: entry for entry in CATALOGUE}
    assert by_provider["Mistral"].api_name is None
    assert by_provider["pib.Cloud"].api_name is None
    assert by_provider["Mistral"].status == STATUS_UNCONFIRMED
    assert by_provider["pib.Cloud"].status == STATUS_UNCONFIRMED
    realtime = next(entry for entry in CATALOGUE if entry.api_name == "gpt-realtime")
    assert realtime.status == STATUS_UNCONFIRMED
    assert realtime.live is True
    assert is_retired("gpt-4o") is True
    assert is_retired("anthropic.claude-3-sonnet-20240229-v1:0") is True
    assert is_retired("hermes-agent") is False
    assert is_retired("gemini-3.8-flash") is False


def test_retired_model_is_reported_on_the_personality(app):
    client = app.test_client()
    body = client.get(f"/voice-assistant/personality/{EVA_PERSONALITY_ID}").get_json()
    assert body["needsNewModel"] is True
    assert body["providerRef"] == str(body["assistantModelId"])
    provider = client.get(f"/provider/{body['assistantModelId']}").get_json()
    assert provider["status"] == "retired"
    assert provider["apiName"] == "anthropic.claude-3-sonnet-20240229-v1:0"


def test_starting_a_chat_on_a_retired_model_is_refused(app):
    client = app.test_client()
    personality = client.get(
        f"/voice-assistant/personality/{EVA_PERSONALITY_ID}"
    ).get_json()
    visual_name = client.get(f"/provider/{personality['assistantModelId']}").get_json()[
        "visualName"
    ]
    with app.app_context():
        before = Chat.query.filter_by(personality_id=EVA_PERSONALITY_ID).count()

    refused = client.post(
        "/voice-assistant/chat",
        json={"topic": "retired", "personalityId": EVA_PERSONALITY_ID},
    )
    assert refused.status_code == 422
    assert refused.get_json()["error"] == (
        f"This personality's model ({visual_name}) has been retired. "
        "Choose a current model in settings before starting a chat."
    )
    with app.app_context():
        after = Chat.query.filter_by(personality_id=EVA_PERSONALITY_ID).count()
    assert after == before


def test_active_model_personality_is_unaffected(app, monkeypatch):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    client = app.test_client()
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "CurrentModel",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
        },
    )
    assert created.status_code == 201
    body = created.get_json()
    assert body["needsNewModel"] is False
    assert body["providerRef"] == "default"
    started = client.post(
        "/voice-assistant/chat",
        json={"topic": "current", "personalityId": body["personalityId"]},
    )
    assert started.status_code == 201

    with app.app_context():
        gemini = Provider.query.filter_by(api_name="gemini-3.8-flash").one()
        gemini_id = gemini.id
        assert gemini.capabilities["live"] is True
        assert gemini.capabilities["images"] is True
    explicit = client.post(
        "/voice-assistant/personality",
        json={
            "name": "GeminiCurrent",
            "gender": "Male",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": gemini_id,
        },
    )
    assert explicit.status_code == 201
    explicit_body = explicit.get_json()
    assert explicit_body["needsNewModel"] is False
    provider = client.get(f"/provider/{gemini_id}").get_json()
    assert provider["status"] == "active"
    started = client.post(
        "/voice-assistant/chat",
        json={
            "topic": "gemini",
            "personalityId": explicit_body["personalityId"],
        },
    )
    assert started.status_code == 201
