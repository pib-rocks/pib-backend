"""Voice backends per personality: local defaults, capability filter."""

from unittest.mock import MagicMock

from model.provider_model import Provider
from pib_hermes_config.voice_backends import LIVE_VOICE_NOTE
from service import key_store_service, personality_service, provider_service


def test_offer_is_local_engines_until_a_capability_is_set(app):
    client = app.test_client()
    body = client.get("/provider/voice-backends").get_json()
    assert [row["id"] for row in body["speechToText"]] == ["local_whisper"]
    assert body["speechToText"][0]["engine"] == "faster-whisper"
    assert [row["id"] for row in body["textToSpeech"]] == ["supertone"]
    assert body["textToSpeech"][0]["engine"] == "supertone"
    assert body["liveVoiceNote"] == LIVE_VOICE_NOTE
    offered = " ".join(
        f"{row['id']} {row['engine']} {row['label']}"
        for row in body["speechToText"] + body["textToSpeech"]
    )
    assert "elevenlabs" not in offered.lower()

    with app.app_context():
        from app.app import db

        gemini = Provider.query.filter_by(api_name="gemini-3.8-flash").one()
        gpt6 = Provider.query.filter_by(api_name="gpt-6").one()
        gemini_id, gpt6_id = gemini.id, gpt6.id
        gemini.capabilities = {**gemini.capabilities, "stt": True}
        gpt6.capabilities = {**gpt6.capabilities, "tts": True}
        db.session.commit()

    body = client.get("/provider/voice-backends").get_json()
    stt_ids = [row["id"] for row in body["speechToText"]]
    tts_ids = [row["id"] for row in body["textToSpeech"]]
    assert stt_ids == ["local_whisper", str(gemini_id)]
    assert str(gpt6_id) not in stt_ids
    assert tts_ids == ["supertone", str(gpt6_id)]
    assert str(gemini_id) not in tts_ids
    assert "elevenlabs" not in str(body).lower()


def test_new_personality_defaults_to_local_voice_before_the_password(app, monkeypatch):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    key_store_service.lock()
    assert key_store_service.operating_mode() == "degraded"
    response = app.test_client().post(
        "/voice-assistant/personality",
        json={
            "name": "OfflineVoice",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
        },
    )
    assert response.status_code == 201
    body = response.get_json()
    assert body["sttEngine"] == "local_whisper"
    assert body["ttsEngine"] == "supertone"
    assert body["localVoiceApplies"] is True
    assert body["liveVoiceNote"] is None


def test_live_provider_states_that_the_local_voice_does_not_apply(app, monkeypatch):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    with app.app_context():
        gemini = Provider.query.filter_by(api_name="gemini-3.8-flash").one()
        gemini_id = gemini.id
        assert gemini.capabilities["live"] is True
    response = app.test_client().post(
        "/voice-assistant/personality",
        json={
            "name": "LiveVoice",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": gemini_id,
        },
    )
    assert response.status_code == 201
    body = response.get_json()
    assert body["localVoiceApplies"] is False
    assert body["liveVoiceNote"] == LIVE_VOICE_NOTE


def test_only_a_capable_provider_can_be_stored(app, monkeypatch):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    client = app.test_client()
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "Chooser",
            "gender": "Male",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
        },
    )
    assert created.status_code == 201
    personality_id = created.get_json()["personalityId"]

    with app.app_context():
        text = Provider.query.filter_by(api_name="gpt-6").one()
        text_id = text.id

    rejected = client.put(
        f"/voice-assistant/personality/{personality_id}",
        json={"sttEngine": str(text_id), "ttsEngine": "elevenlabs"},
    )
    assert rejected.status_code == 400
    unchanged = client.get(f"/voice-assistant/personality/{personality_id}").get_json()
    assert unchanged["sttEngine"] == "local_whisper"
    assert unchanged["ttsEngine"] == "supertone"

    named = client.put(
        f"/voice-assistant/personality/{personality_id}",
        json={"sttEngine": "tryb_api"},
    )
    assert named.status_code == 400

    with app.app_context():
        from app.app import db

        text = Provider.query.filter_by(id=text_id).one()
        text.capabilities = {**text.capabilities, "stt": True, "tts": True}
        db.session.commit()

    accepted = client.put(
        f"/voice-assistant/personality/{personality_id}",
        json={"sttEngine": str(text_id), "ttsEngine": str(text_id)},
    )
    assert accepted.status_code == 200
    stored = accepted.get_json()
    assert stored["sttEngine"] == str(text_id)
    assert stored["ttsEngine"] == str(text_id)
    with app.app_context():
        listed = provider_service.speech_backends()
    assert str(text_id) in {row["id"] for row in listed["speechToText"]}
    assert str(text_id) in {row["id"] for row in listed["textToSpeech"]}
