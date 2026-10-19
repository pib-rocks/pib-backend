"""PC-43: cloud whisper is selectable, and pib.Cloud makes it the default.

Both engines stay selectable. pib.Cloud only changes the default: an explicit
choice always wins and is kept, another provider keeps local faster-whisper,
and a stored engine is never rewritten.
"""

import pytest
from click.testing import CliRunner
from marshmallow import ValidationError
from commands import seed_db
from model.personality_model import Personality
from model.provider_model import RegistryModel
from pib_hermes_config.voice_backends import (
    CLOUD_STT_ID,
    LEGACY_CLOUD_STT,
    LOCAL_STT_ID,
    normalize_stt_choice,
    resolve_stt_route,
)
from provider_registry import CATALOGUE, PIB_CLOUD_API_NAME, capabilities_for
from service import personality_service, provider_service

PERSONALITY_URL = "/voice-assistant/personality"
EVA_PERSONALITY_ID = "8f73b580-927e-41c2-98ac-e5df070e7288"
CLOUD_REJECTED = "Cloud whisper needs pib.Cloud with the stt capability."


@pytest.fixture(autouse=True)
def profile_factory_is_up(monkeypatch):
    monkeypatch.setattr(
        personality_service, "_provision_profile", lambda *_a, **_k: {"ok": True}
    )


@pytest.fixture()
def client(app):
    return app.test_client()


def _row_id(api_name):
    return RegistryModel.query.filter_by(api_name=api_name).one().id


@pytest.fixture()
def ids(app):
    with app.app_context():
        return {"cloud": _row_id(PIB_CLOUD_API_NAME), "gpt6": _row_id("gpt-6")}


def _disable_cloud_stt(app):
    with app.app_context():
        from app.app import db

        row = RegistryModel.query.filter_by(api_name=PIB_CLOUD_API_NAME).one()
        row.capabilities = {**row.capabilities, "stt": False}
        db.session.commit()


def _create(client, **body):
    return client.post(PERSONALITY_URL, json={"name": "Stt", **body})


def _stt(client, personality_id):
    return client.get(f"{PERSONALITY_URL}/{personality_id}").get_json()["sttEngine"]


# --- pure helpers ------------------------------------------------------------


def _never(_model_id):
    return False


def test_cloud_whisper_is_accepted_only_while_pib_cloud_has_stt():
    assert normalize_stt_choice(CLOUD_STT_ID, _never, 5) == CLOUD_STT_ID
    with pytest.raises(ValueError, match="pib.Cloud"):
        normalize_stt_choice(CLOUD_STT_ID, _never, None)


def test_the_bare_pib_cloud_row_id_is_stored_as_cloud_whisper():
    stored = normalize_stt_choice("5", lambda model_id: model_id == 5, 5)
    assert stored == CLOUD_STT_ID
    # Without the capability the bare id is an ordinary rejected row.
    with pytest.raises(ValueError):
        normalize_stt_choice("5", _never, None)


def test_other_capable_rows_and_local_aliases_keep_their_meaning():
    assert normalize_stt_choice("7", lambda model_id: model_id == 7, 5) == "7"
    assert normalize_stt_choice("faster-whisper", _never, 5) == LOCAL_STT_ID
    assert normalize_stt_choice(LOCAL_STT_ID, _never, 5) == LOCAL_STT_ID
    with pytest.raises(ValueError):
        normalize_stt_choice(LEGACY_CLOUD_STT, _never, 5)


@pytest.mark.parametrize("missing", [None, "", "  "])
def test_no_choice_stores_the_given_default(missing):
    assert normalize_stt_choice(missing, _never, 5, CLOUD_STT_ID) == CLOUD_STT_ID
    assert normalize_stt_choice(missing, _never, 5) == LOCAL_STT_ID


def test_cloud_whisper_routes_to_cloud_and_not_to_the_legacy_client():
    assert CLOUD_STT_ID != LEGACY_CLOUD_STT
    assert resolve_stt_route(CLOUD_STT_ID) == "cloud"
    assert resolve_stt_route(LEGACY_CLOUD_STT) == "tryb"
    assert resolve_stt_route(LOCAL_STT_ID) == "local"
    assert resolve_stt_route("") == "local"
    assert resolve_stt_route("5") == "unsupported"


# --- catalogue -----------------------------------------------------------------


def test_only_the_pib_cloud_catalogue_line_carries_stt():
    flagged = [entry.api_name for entry in CATALOGUE if entry.stt]
    assert flagged == [PIB_CLOUD_API_NAME]
    cloud = next(entry for entry in CATALOGUE if entry.api_name == PIB_CLOUD_API_NAME)
    assert cloud.tts is False
    assert cloud.is_default is True
    assert capabilities_for(PIB_CLOUD_API_NAME, True)["stt"] is True


def test_seeding_refreshes_the_flag_and_keeps_stored_engines(app):
    """An existing deployment gets stt through seed-db; no personality moves."""
    _disable_cloud_stt(app)
    with app.app_context():
        from app.app import db

        eva = Personality.query.filter_by(personality_id=EVA_PERSONALITY_ID).one()
        assert eva.stt_engine == LOCAL_STT_ID
        before = {p.personality_id: p.stt_engine for p in Personality.query.all()}

        result = CliRunner().invoke(seed_db, [])
        assert result.exception is None, result.output
        db.session.remove()

        row = RegistryModel.query.filter_by(api_name=PIB_CLOUD_API_NAME).one()
        assert row.capabilities["stt"] is True
        after = {p.personality_id: p.stt_engine for p in Personality.query.all()}
        assert after == before


# --- offered options -----------------------------------------------------------


def test_pib_cloud_offers_exactly_local_and_cloud_whisper(app, ids):
    with app.app_context():
        listed = provider_service.speech_backends()["speechToText"]
    assert [row["id"] for row in listed] == [LOCAL_STT_ID, CLOUD_STT_ID]
    assert [row["engine"] for row in listed] == ["faster-whisper", CLOUD_STT_ID]
    assert [row["label"] for row in listed] == [
        "Local faster-whisper",
        "pib.Cloud Whisper",
    ]
    assert str(ids["cloud"]) not in {row["id"] for row in listed}


def test_every_offered_id_is_accepted_back(client):
    listed = client.get("/provider/voice-backends").get_json()["speechToText"]
    for option in listed:
        created = _create(client, sttEngine=option["id"])
        assert created.status_code == 201, option
        assert created.get_json()["sttEngine"] == option["id"]


def test_without_the_capability_cloud_whisper_is_neither_offered_nor_stored(
    app, client, ids
):
    _disable_cloud_stt(app)
    listed = client.get("/provider/voice-backends").get_json()["speechToText"]
    assert [row["id"] for row in listed] == [LOCAL_STT_ID]

    assert _create(client, sttEngine=CLOUD_STT_ID).status_code == 400
    assert _create(client, sttEngine=str(ids["cloud"])).status_code == 400
    with app.app_context():
        with pytest.raises(ValidationError) as caught:
            personality_service.create_personality(
                {"name": "Off", "stt_engine": CLOUD_STT_ID}
            )
    assert caught.value.messages == {"sttEngine": [CLOUD_REJECTED]}

    omitted = _create(client)
    assert omitted.status_code == 201
    assert omitted.get_json()["sttEngine"] == LOCAL_STT_ID


def test_an_unknown_engine_is_rejected(client):
    assert _create(client, sttEngine="whisper-somewhere").status_code == 400
    assert _create(client, sttEngine=LEGACY_CLOUD_STT).status_code == 400


# --- AC1: pib.Cloud without a choice reports cloud whisper ---------------------


@pytest.mark.parametrize("reference", ["policy", "modelRef", "providerRef", "id"])
def test_pib_cloud_without_a_choice_is_cloud_whisper(client, ids, reference):
    body = {
        "policy": {},
        "modelRef": {"modelRef": f"model:{ids['cloud']}"},
        "providerRef": {"providerRef": str(ids["cloud"])},
        "id": {"assistantModelId": ids["cloud"]},
    }[reference]
    created = _create(client, **body)
    assert created.status_code == 201
    assert created.get_json()["sttEngine"] == CLOUD_STT_ID
    assert _stt(client, created.get_json()["personalityId"]) == CLOUD_STT_ID


def test_a_blank_choice_on_pib_cloud_is_no_choice(client):
    created = _create(client, sttEngine="")
    assert created.status_code == 201
    assert created.get_json()["sttEngine"] == CLOUD_STT_ID


def test_the_service_applies_the_default_without_the_schema(app_ctx):
    made = personality_service.create_personality({"name": "Dienst"})
    assert made.stt_engine == CLOUD_STT_ID


def test_the_bare_pib_cloud_id_is_sent_back_as_cloud_whisper(client, ids):
    created = _create(
        client, modelRef=f"model:{ids['gpt6']}", sttEngine=str(ids["cloud"])
    )
    assert created.status_code == 201
    assert created.get_json()["sttEngine"] == CLOUD_STT_ID


# --- AC2: an explicit local choice wins and is kept ----------------------------


@pytest.mark.parametrize("local", [LOCAL_STT_ID, "faster-whisper"])
def test_pib_cloud_with_an_explicit_local_choice_is_local(client, ids, local):
    created = _create(client, modelRef=f"model:{ids['cloud']}", sttEngine=local)
    assert created.status_code == 201
    assert created.get_json()["sttEngine"] == LOCAL_STT_ID


def test_an_explicit_local_choice_survives_model_and_unrelated_edits(client, ids):
    created = _create(client, sttEngine=LOCAL_STT_ID).get_json()
    url = f"{PERSONALITY_URL}/{created['personalityId']}"

    for edit in (
        {"pauseThreshold": 1.2},
        {"modelRef": f"model:{ids['gpt6']}"},
        {"modelRef": f"model:{ids['cloud']}"},
        {"modelRef": "default"},
        {"providerRef": str(ids["cloud"])},
    ):
        response = client.put(url, json=edit)
        assert response.status_code == 200, edit
        assert response.get_json()["sttEngine"] == LOCAL_STT_ID, edit
    assert _stt(client, created["personalityId"]) == LOCAL_STT_ID


def test_switching_engines_explicitly_is_free_in_both_directions(client):
    created = _create(client).get_json()
    url = f"{PERSONALITY_URL}/{created['personalityId']}"
    assert created["sttEngine"] == CLOUD_STT_ID

    to_local = client.put(url, json={"sttEngine": LOCAL_STT_ID})
    assert to_local.get_json()["sttEngine"] == LOCAL_STT_ID
    back = client.put(url, json={"sttEngine": CLOUD_STT_ID})
    assert back.get_json()["sttEngine"] == CLOUD_STT_ID
    assert resolve_stt_route(_stt(client, created["personalityId"])) == "cloud"


def test_a_rejected_engine_leaves_the_stored_one(client):
    created = _create(client, sttEngine=LOCAL_STT_ID).get_json()
    url = f"{PERSONALITY_URL}/{created['personalityId']}"
    assert client.put(url, json={"sttEngine": "bogus"}).status_code == 400
    assert _stt(client, created["personalityId"]) == LOCAL_STT_ID


# --- AC3: another provider keeps local -----------------------------------------


def test_another_provider_defaults_to_local(client, ids):
    created = _create(client, modelRef=f"model:{ids['gpt6']}")
    assert created.status_code == 201
    assert created.get_json()["sttEngine"] == LOCAL_STT_ID


def test_moving_to_pib_cloud_later_does_not_switch_the_engine(client, ids):
    created = _create(client, modelRef=f"model:{ids['gpt6']}").get_json()
    url = f"{PERSONALITY_URL}/{created['personalityId']}"
    moved = client.put(url, json={"modelRef": f"model:{ids['cloud']}"})
    assert moved.status_code == 200
    assert moved.get_json()["sttEngine"] == LOCAL_STT_ID


def test_another_provider_may_still_choose_cloud_whisper(client, ids):
    created = _create(client, modelRef=f"model:{ids['gpt6']}", sttEngine=CLOUD_STT_ID)
    assert created.status_code == 201
    assert created.get_json()["sttEngine"] == CLOUD_STT_ID


def test_a_blank_update_resets_to_the_default_of_the_chosen_model(client, ids):
    created = _create(
        client, modelRef=f"model:{ids['gpt6']}", sttEngine=CLOUD_STT_ID
    ).get_json()
    url = f"{PERSONALITY_URL}/{created['personalityId']}"
    reset = client.put(url, json={"sttEngine": ""})
    assert reset.status_code == 200
    assert reset.get_json()["sttEngine"] == LOCAL_STT_ID


LOCAL_MODEL_TAGS = {
    "models": [{"name": "qwen-fast:latest", "model": "qwen-fast:latest"}]
}


def test_default_policy_on_the_local_model_stays_local(client, monkeypatch):
    from service import local_model_service

    monkeypatch.setattr(local_model_service, "fetch_tags", lambda: LOCAL_MODEL_TAGS)
    client.get("/assistant-model")
    created = _create(client, channel="direct")
    assert created.status_code == 201
    assert created.get_json()["sttEngine"] == LOCAL_STT_ID


# --- AC4: an existing personality keeps its stored engine ----------------------


def test_an_existing_pib_cloud_personality_still_reports_local(client):
    body = client.get(f"{PERSONALITY_URL}/{EVA_PERSONALITY_ID}").get_json()
    assert body["sttEngine"] == LOCAL_STT_ID
    edited = client.put(
        f"{PERSONALITY_URL}/{EVA_PERSONALITY_ID}", json={"pauseThreshold": 0.9}
    )
    assert edited.status_code == 200
    assert edited.get_json()["sttEngine"] == LOCAL_STT_ID
    listed = client.get(PERSONALITY_URL).get_json()["voiceAssistantPersonalities"]
    eva = next(p for p in listed if p["personalityId"] == EVA_PERSONALITY_ID)
    assert eva["sttEngine"] == LOCAL_STT_ID
