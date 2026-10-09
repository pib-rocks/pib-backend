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
    assert created["voiceMode"] == "turn_based"
    assert created["liveIdleTimeout"] == 60
    assert created["sttEngine"] == "local_whisper"
    assert created["ttsEngine"] == "supertone"
    assert created["providerRef"] == "default"
    assert created["channel"] == "smart"
    assert created["reasoningEffort"] == "none"
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


# The on-device row is offered only while Ollama lists it, so its id is not
# the default row's id. A create that carries the model's own id must select
# that row even when the body also carries a provider account id, as the
# catalogue dumps it next to the model.
LOCAL_MODEL_TAGS = {
    "models": [{"name": "qwen-fast:latest", "model": "qwen-fast:latest"}]
}


def test_a_create_carrying_the_model_id_selects_that_row(client, app_ctx, monkeypatch):
    from service import local_model_service

    monkeypatch.setattr(local_model_service, "fetch_tags", lambda: LOCAL_MODEL_TAGS)
    models = client.get("/assistant-model").get_json()["assistantModels"]
    local = next(row for row in models if row["apiName"] == "qwen-fast")
    assert local["providerId"] != local["id"]

    response = client.post(
        PERSONALITY_URL,
        json={
            "name": "LocalModel",
            "channel": "direct",
            "assistantModelId": local["id"],
            "providerRef": str(local["providerId"]),
        },
    )

    assert response.status_code == 201
    created = response.get_json()
    assert created["assistantModelId"] == local["id"]
    assert created["providerRef"] == str(local["id"])


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


def test_the_create_description_states_the_typed_reference():
    text = API_YAML.read_text(encoding="utf-8")
    post = text.split("  /voice-assistant/personality:", 1)[1].split("    get:", 1)[0]
    assert "PostVoiceAssistantPersonality" in post
    body = text.split("PostVoiceAssistantPersonality:", 1)[1].split(
        "VoiceAssistantPersonalities:", 1
    )[0]
    assert "modelRef:" in body
    assert "providerRef:" in body
    assert "deprecated: true" in body
    assert 'sentinel "default"' in body
    assert "model:<id>" in body
    assert "v0.8.0" in body
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


# ---------------------------------------------------------------------------
# PR-1930 (option D2): typed, unambiguous model references.
#
# The canonical field is modelRef: "default" or "model:<id>". providerRef (a
# bare id as text) and assistantModelId (a bare id as a number) are deprecated
# aliases kept for the migration window and removed in v0.8.0.
# ---------------------------------------------------------------------------


def test_create_with_a_typed_model_reference_round_trips(client, app_ctx):
    gemini = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()

    response = client.post(
        PERSONALITY_URL,
        json={"name": "Typed", "channel": "direct", "modelRef": f"model:{gemini.id}"},
    )

    assert response.status_code == 201
    created = response.get_json()
    assert created["modelRef"] == f"model:{gemini.id}"
    # the deprecated aliases keep reporting the same model
    assert created["assistantModelId"] == gemini.id
    assert created["providerRef"] == str(gemini.id)

    pid = created["personalityId"]
    read = client.get(f"{PERSONALITY_URL}/{pid}").get_json()
    assert read["modelRef"] == f"model:{gemini.id}"
    assert read["assistantModelId"] == gemini.id
    assert read["providerRef"] == str(gemini.id)

    updated = client.put(f"{PERSONALITY_URL}/{pid}", json={"modelRef": "default"})
    assert updated.status_code == 200
    assert updated.get_json()["modelRef"] == "default"
    assert updated.get_json()["assistantModelId"] is None
    assert updated.get_json()["providerRef"] == "default"


def test_a_typed_reference_and_an_agreeing_alias_are_accepted(client, app_ctx):
    gemini = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()

    response = client.post(
        PERSONALITY_URL,
        json={
            "name": "Agree",
            "channel": "direct",
            "modelRef": f"model:{gemini.id}",
            "providerRef": str(gemini.id),
            "assistantModelId": gemini.id,
        },
    )

    assert response.status_code == 201
    assert response.get_json()["modelRef"] == f"model:{gemini.id}"


@pytest.mark.parametrize(
    "bad",
    [
        {"modelRef": "6"},
        {"modelRef": "account:5"},
        {"modelRef": "provider:5"},
        {"modelRef": "gemini-3.8-flash"},
        {"modelRef": "model:0"},
        {"modelRef": "model:99999"},
        {"modelRef": "model:"},
    ],
    ids=[
        "bare-id",
        "account-namespace",
        "provider-namespace",
        "api-name",
        "zero",
        "unknown-row",
        "empty-id",
    ],
)
def test_an_invalid_typed_reference_is_rejected(client, app_ctx, bad):
    response = client.post(PERSONALITY_URL, json={"name": "BadRef", **bad})

    assert response.status_code == 400
    listed = client.get(PERSONALITY_URL).get_json()["voiceAssistantPersonalities"]
    assert all(personality["name"] != "BadRef" for personality in listed)


def test_a_typed_reference_conflicting_with_an_alias_is_rejected(client, app_ctx):
    gemini = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
    other = RegistryModel.query.filter_by(api_name="gpt-6").one()

    # The measured defect shape: two references that do not name the same
    # model. It is rejected instead of one silently winning.
    response = client.post(
        PERSONALITY_URL,
        json={
            "name": "Conflict",
            "channel": "direct",
            "modelRef": f"model:{gemini.id}",
            "assistantModelId": other.id,
        },
    )

    assert response.status_code == 400
    listed = client.get(PERSONALITY_URL).get_json()["voiceAssistantPersonalities"]
    assert all(personality["name"] != "Conflict" for personality in listed)


def test_a_provider_account_id_is_never_read_as_a_model_row(client, app_ctx):
    """The measured defect: model-row ids and account ids overlap."""
    from model.provider_model import Provider

    account = Provider.query.order_by(Provider.id).first()
    # The collision is real only when the account id also names a model row.
    if RegistryModel.query.filter_by(id=account.id).first() is None:
        pytest.skip("no provider-account/model-row id collision in this catalogue")

    response = client.post(
        PERSONALITY_URL,
        json={"name": "Collision", "channel": "direct", "modelRef": str(account.id)},
    )

    assert response.status_code == 400


def test_service_accepts_a_typed_reference(app_ctx):
    gemini = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()

    personality = personality_service.create_personality(
        {"name": "DienstTyped", "model_ref": f"model:{gemini.id}"}
    )

    assert personality.provider_ref == str(gemini.id)
    assert personality.assistant_model_id == gemini.id


def test_service_rejects_a_bare_reference_in_the_typed_field(app_ctx):
    with pytest.raises(ValidationError) as caught:
        personality_service.create_personality({"name": "Fremd", "model_ref": "5"})

    assert caught.value.messages["modelRef"] == [
        "modelRef must be 'default' or a model reference like 'model:6'."
    ]


def test_service_rejects_an_unknown_typed_reference(app_ctx):
    with pytest.raises(ValidationError) as caught:
        personality_service.create_personality(
            {"name": "Fremd", "model_ref": "model:99999"}
        )

    assert caught.value.messages["modelRef"] == ["modelRef names no model row."]


def test_an_existing_stored_reference_needs_no_conversion(app_ctx):
    """AC5: the stored column is already a validated model-row id.

    A row written with the deprecated spelling maps to the same typed
    reference, so no row has to be converted and the model it had is kept.
    """
    from app.app import db
    from schema.personality_schema import personality_schema

    gemini = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
    made = personality_service.create_personality(
        {"name": "LegacyStored", "provider_ref": str(gemini.id)}
    )
    db.session.commit()

    body = personality_schema.dump(made)
    assert body["modelRef"] == f"model:{gemini.id}"
    assert body["assistantModelId"] == gemini.id
    assert body["providerRef"] == str(gemini.id)
