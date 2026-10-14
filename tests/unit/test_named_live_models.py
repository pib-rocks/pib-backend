"""Live speech is a named catalogue model. The client does not toggle a mode."""

from click.testing import CliRunner

from app.app import db
from commands import seed_db
from model.provider_model import RegistryModel
from pib_hermes_config.live_session import GEMINI_LIVE_MODEL, VOICE_MODE_LIVE
from provider_registry import catalogue_entry
from service import personality_service


def test_catalogue_lists_gemini_live_beside_flash():
    flash = catalogue_entry("gemini-3.8-flash")
    live = catalogue_entry(GEMINI_LIVE_MODEL)
    assert flash is not None and live is not None
    assert flash.visual_name == "Gemini 3.8 Flash"
    assert flash.live is False
    assert live.visual_name == "Gemini 3.8 Live"
    assert live.provider == flash.provider == "Google"
    assert live.live is True
    assert live.api_name == GEMINI_LIVE_MODEL


def test_voice_mode_is_the_chosen_model_and_a_switch_is_refused(app, monkeypatch):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        lambda *_args, **_kwargs: {"ok": True},
    )
    with app.app_context():
        flash_id = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one().id
        live_id = RegistryModel.query.filter_by(api_name=GEMINI_LIVE_MODEL).one().id
    client = app.test_client()
    flash = client.post(
        "/voice-assistant/personality",
        json={
            "name": "FlashOnly",
            "gender": "Female",
            "assistantModelId": flash_id,
        },
    )
    assert flash.status_code == 201
    flash_body = flash.get_json()
    assert flash_body["voiceMode"] == "turn_based"
    assert flash_body["liveModel"] is None
    assert flash_body["voiceStartMode"] == "turn_based"

    switched = client.put(
        f"/voice-assistant/personality/{flash_body['personalityId']}",
        json={"voiceMode": VOICE_MODE_LIVE},
    )
    assert switched.status_code == 400
    kept = client.get(
        f"/voice-assistant/personality/{flash_body['personalityId']}"
    ).get_json()
    assert kept["voiceMode"] == "turn_based"
    assert kept["providerRef"] == str(flash_id)

    live = client.post(
        "/voice-assistant/personality",
        json={
            "name": "LiveOnly",
            "gender": "Male",
            "assistantModelId": live_id,
        },
    )
    assert live.status_code == 201
    live_body = live.get_json()
    assert live_body["voiceMode"] == VOICE_MODE_LIVE
    assert live_body["liveModel"] == GEMINI_LIVE_MODEL
    assert live_body["voiceStartMode"] == VOICE_MODE_LIVE
    assert live_body["providerRef"] == str(live_id)


def test_reseed_replaces_a_hidden_pin_with_the_named_live_model(app):
    with app.app_context():
        flash = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
        flash.capabilities = {**flash.capabilities, "live": True}
        flash.live_model = GEMINI_LIVE_MODEL
        db.session.commit()

        result = CliRunner().invoke(seed_db, [])
        assert result.exception is None, result.output
        db.session.remove()

        flash = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
        live = RegistryModel.query.filter_by(api_name=GEMINI_LIVE_MODEL).one()
        assert flash.capabilities["live"] is False
        assert flash.live_model is None
        assert live.visual_name == "Gemini 3.8 Live"
        assert live.capabilities["live"] is True
        assert live.provider_id == flash.provider_id
        assert live.live_model is None
