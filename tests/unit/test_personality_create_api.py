"""Creating a personality asks for a name only; the rest is defaulted.

Cerebra's Add dialog sends the name and whatever the Advanced dialog set.
Every other value comes from the defaults and can be changed afterwards.
A catalogue provider is named by its row id, or by the sentinel "default".
The catalogue api_name is not a provider reference.
"""

from pathlib import Path

import pytest
from marshmallow import ValidationError
from model.provider_model import RegistryModel
from service import personality_service

API_YAML = Path(__file__).resolve().parents[2] / "pib_api" / "pib-api.yaml"

PERSONALITY_URL = "/voice-assistant/personality"


@pytest.fixture()
def client(app):
    return app.test_client()


@pytest.fixture(autouse=True)
def profile_factory_is_up(monkeypatch):
    monkeypatch.setattr(
        personality_service, "_provision_profile", lambda *_a, **_k: {"ok": True}
    )


def test_name_only_create_succeeds_with_the_documented_defaults(client, app_ctx):
    response = client.post(PERSONALITY_URL, json={"name": "Nur Name"})

    assert response.status_code == 201
    created = response.get_json()
    assert created["name"] == "Nur Name"
    assert created["gender"] == "Female"
    assert created["pauseThreshold"] == 0.8
    assert created["messageHistory"] == 5
    assert created["toolCalling"] is True
    assert created["voiceMode"] == "live"
    assert created["liveIdleTimeout"] == 60
    assert created["sttEngine"] == "local_whisper"
    assert created["ttsEngine"] == "supertone"
    assert created["providerRef"] == "default"
    assert created["channel"] == "smart"
    assert created["thinkingFiller"] is None
    assert "Nur Name" in created["description"]


def test_name_only_create_persists_and_is_readable_and_updateable(client, app_ctx):
    created = client.post(PERSONALITY_URL, json={"name": "Spaeter"}).get_json()
    personality_id = created["personalityId"]

    listed = client.get(PERSONALITY_URL).get_json()["voiceAssistantPersonalities"]
    assert personality_id in {p["personalityId"] for p in listed}

    read = client.get(f"{PERSONALITY_URL}/{personality_id}")
    assert read.status_code == 200
    assert read.get_json()["pauseThreshold"] == 0.8
    assert read.get_json()["messageHistory"] == 5

    updated = client.put(
        f"{PERSONALITY_URL}/{personality_id}",
        json={"gender": "Male", "pauseThreshold": 1.4, "messageHistory": 12},
    )
    assert updated.status_code == 200
    assert updated.get_json()["gender"] == "Male"
    assert updated.get_json()["pauseThreshold"] == 1.4
    assert updated.get_json()["messageHistory"] == 12

    again = client.get(f"{PERSONALITY_URL}/{personality_id}").get_json()
    assert again["gender"] == "Male"
    assert again["pauseThreshold"] == 1.4
    assert again["messageHistory"] == 12


def test_create_pointing_at_a_catalogue_provider_on_the_direct_channel(client, app_ctx):
    gemini = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
    response = client.post(
        PERSONALITY_URL,
        json={
            "name": "Gemini Test",
            "gender": "Female",
            "providerRef": str(gemini.id),
            "channel": "direct",
            "pauseThreshold": 0.5,
            "messageHistory": 10,
        },
    )

    assert response.status_code == 201
    created = response.get_json()
    assert created["name"] == "Gemini Test"
    assert created["providerRef"] == str(gemini.id)
    assert created["assistantModelId"] == gemini.id
    assert created["channel"] == "direct"
    assert created["pauseThreshold"] == 0.5
    assert created["messageHistory"] == 10
    personality_id = created["personalityId"]

    read = client.get(f"{PERSONALITY_URL}/{personality_id}")
    assert read.status_code == 200
    assert read.get_json()["providerRef"] == str(gemini.id)
    assert read.get_json()["channel"] == "direct"

    updated = client.put(
        f"{PERSONALITY_URL}/{personality_id}",
        json={"pauseThreshold": 1.2},
    )
    assert updated.status_code == 200
    assert updated.get_json()["providerRef"] == str(gemini.id)
    assert updated.get_json()["channel"] == "direct"
    assert updated.get_json()["pauseThreshold"] == 1.2

    again = client.get(f"{PERSONALITY_URL}/{personality_id}").get_json()
    assert again["providerRef"] == str(gemini.id)
    assert again["channel"] == "direct"
    assert again["pauseThreshold"] == 1.2


def test_create_with_the_formerly_required_fields_still_honours_them(client, app_ctx):
    response = client.post(
        PERSONALITY_URL,
        json={
            "name": "Voll",
            "gender": "Male",
            "pauseThreshold": 1.0,
            "messageHistory": 15,
            "channel": "direct",
            "toolCalling": False,
            "description": "",
        },
    )

    assert response.status_code == 201
    created = response.get_json()
    assert created["gender"] == "Male"
    assert created["pauseThreshold"] == 1.0
    assert created["messageHistory"] == 15
    assert created["channel"] == "direct"
    assert created["toolCalling"] is False


def test_service_create_from_a_name_only_dto(app_ctx):
    from app.app import db

    personality = personality_service.create_personality({"name": "Dienst"})
    db.session.commit()

    assert personality.gender == "Female"
    assert personality.pause_threshold == 0.8
    assert personality.message_history == 5
    assert personality.provider_ref == "default"
    assert personality.channel == "smart"


# Each case is otherwise complete, so the one bad value is the only reason
# to reject it. A schema without the range would accept these.
COMPLETE = {"gender": "Female", "pauseThreshold": 0.8, "messageHistory": 5}


@pytest.mark.parametrize(
    "bad",
    [
        {"name": "Fremd", "favouriteColour": "blue"},
        {"name": "Kanal", "channel": "loud"},
        {"name": "Zu lang", "pauseThreshold": 3.5},
        {"name": "Zu kurz", "pauseThreshold": 0.0},
        {"name": "Negativ", "messageHistory": -1},
        {"name": "ApiName", "providerRef": "gemini-3.8-flash"},
        {"name": "MissingRow", "providerRef": "99999"},
        {"name": "Snake", "provider_ref": "1", "pause_threshold": 0.5},
        {"name": None},
    ],
    ids=[
        "unknown-field",
        "channel",
        "threshold-high",
        "threshold-low",
        "history",
        "api-name",
        "unknown-id",
        "snake-case",
        "no-name",
    ],
)
def test_an_invalid_create_is_still_rejected(client, app_ctx, bad):
    payload = {**COMPLETE, **bad}
    if payload["name"] is None:
        del payload["name"]
    response = client.post(PERSONALITY_URL, json=payload)

    assert response.status_code == 400
    listed = client.get(PERSONALITY_URL).get_json()["voiceAssistantPersonalities"]
    assert all(p["name"] != payload.get("name") for p in listed)


def test_an_api_name_is_not_a_provider_reference(app_ctx):
    with pytest.raises(ValidationError) as caught:
        personality_service.create_personality(
            {"name": "ApiName", "provider_ref": "gemini-3.8-flash"}
        )

    assert caught.value.messages["providerRef"] == [
        "Provider reference must be 'default' or the id of a model row."
    ]
    listed = personality_service.get_all_personalities()
    assert all(personality.name != "ApiName" for personality in listed)


def test_the_create_description_states_the_provider_reference_forms():
    text = API_YAML.read_text(encoding="utf-8")
    post = text.split("  /voice-assistant/personality:", 1)[1].split("    get:", 1)[0]
    assert "PostVoiceAssistantPersonality" in post
    body = text.split("PostVoiceAssistantPersonality:", 1)[1].split(
        "VoiceAssistantPersonalities:", 1
    )[0]
    assert "providerRef:" in body
    assert 'sentinel "default"' in body
    assert "decimal id of a model row" in body
    assert "The provider follows from that model." in body
    assert "gemini-3.8-flash, is not a reference" in body
    assert "enum: [smart, direct]" in body


def test_an_update_still_validates_what_it_is_given(client, app_ctx):
    created = client.post(PERSONALITY_URL, json={"name": "Streng"}).get_json()
    url = f"{PERSONALITY_URL}/{created['personalityId']}"

    assert client.put(url, json={"pauseThreshold": 9.0}).status_code == 400
    assert client.put(url, json={"messageHistory": -3}).status_code == 400
    assert client.put(url, json={"channel": "loud"}).status_code == 400
    assert client.put(url, json={"unknownField": 1}).status_code == 400

    unchanged = client.get(url).get_json()
    assert unchanged["pauseThreshold"] == 0.8
    assert unchanged["messageHistory"] == 5
