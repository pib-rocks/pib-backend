"""Provider registry: seeded ids, image filter, default pointer."""

from unittest.mock import MagicMock

from model.assistant_model import AssistantModel
from model.personality_model import Personality
from model.provider_model import Provider
from pib_api_client.voice_assistant_client import model_endpoint_for
from provider_registry import DEFAULT_PROVIDER_REF, has_images_capability
from service import personality_service, provider_service


def test_seed_copies_assistant_models_without_losing_ids(app_ctx):
    expected = {
        "gemini-3.5-flash": "Gemini 3.5 Flash",
        "hermes-agent": "Hermes Agent (selbstlernend)",
    }
    for api_name, visual_name in expected.items():
        assistant = AssistantModel.query.filter_by(visual_name=visual_name).one()
        provider = Provider.query.filter_by(visual_name=visual_name).one()
        assert assistant.api_name == api_name
        assert provider.id == assistant.id
        assert provider.api_name == assistant.api_name
        assert provider.has_image_support == assistant.has_image_support

    for visual_name in ("GPT-4o [Vision]", "GPT-4o [Text]"):
        assistant = AssistantModel.query.filter_by(visual_name=visual_name).one()
        provider = Provider.query.filter_by(visual_name=visual_name).one()
        assert assistant.api_name == "gpt-4o"
        assert provider.id == assistant.id
        assert (
            has_images_capability(provider.capabilities) is assistant.has_image_support
        )

    for personality in Personality.query.all():
        assert personality.assistant_model_id is not None
        assert personality.provider_ref == str(personality.assistant_model_id)


def test_selection_offers_only_rows_with_images_capability(app):
    with app.app_context():
        text = Provider.query.filter_by(visual_name="GPT-4o [Text]").one()
        vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
        gemini = Provider.query.filter_by(api_name="gemini-3.5-flash").one()
        hermes = Provider.query.filter_by(api_name="hermes-agent").one()
        assert text.api_name == vision.api_name
        text_id, vision_id, gemini_id, hermes_id = (
            text.id,
            vision.id,
            gemini.id,
            hermes.id,
        )

    client = app.test_client()
    providers = client.get("/provider").get_json()["providers"]
    offered = {row["id"] for row in providers}
    assert set(providers[0]["capabilities"]) == {
        "tools",
        "images",
        "live",
        "stt",
        "tts",
    }
    assistant_offered = {
        row["id"]
        for row in client.get("/assistant-model").get_json()["assistantModels"]
    }
    assert offered == assistant_offered
    assert vision_id in offered
    assert hermes_id in offered
    assert text_id not in offered
    assert gemini_id not in offered

    # Same api name as a selectable row. The flag decides, not the name.
    with app.app_context():
        from app.app import db

        text = Provider.query.filter_by(id=text_id).one()
        text.capabilities = {**text.capabilities, "images": True}
        hermes = Provider.query.filter_by(id=hermes_id).one()
        hermes.capabilities = {**hermes.capabilities, "images": False}
        db.session.commit()

    offered = {row["id"] for row in client.get("/provider").get_json()["providers"]}
    assert text_id in offered
    assert hermes_id not in offered
    by_id = client.get(f"/provider/{gemini_id}")
    assert by_id.status_code == 200
    assert by_id.get_json()["capabilities"]["images"] is False


def test_new_personality_stores_default_pointer(app_ctx, monkeypatch):
    from app.app import db

    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    created = personality_service.create_personality(
        {
            "name": "DefaultPointer",
            "gender": "Female",
            "pause_threshold": 0.8,
            "message_history": 5,
        }
    )
    db.session.commit()
    assert created.provider_ref == DEFAULT_PROVIDER_REF
    assert created.assistant_model_id is None

    original_default = provider_service.get_default_provider()
    assert provider_service.resolve_provider(created.provider_ref).id == (
        original_default.id
    )
    stored_ref = created.provider_ref

    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    original_default.is_default = False
    db.session.flush()
    vision.is_default = True
    db.session.commit()

    reloaded = Personality.query.filter_by(personality_id=created.personality_id).one()
    assert reloaded.provider_ref == stored_ref
    assert reloaded.assistant_model_id is None
    assert provider_service.resolve_provider(reloaded.provider_ref).id == vision.id


def test_api_create_without_model_stores_default(app, monkeypatch):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    response = app.test_client().post(
        "/voice-assistant/personality",
        json={
            "name": "ApiDefault",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
        },
    )
    assert response.status_code == 201
    body = response.get_json()
    assert body["providerRef"] == DEFAULT_PROVIDER_REF
    assert body["assistantModelId"] is None


def test_explicit_model_id_is_stored_as_that_id(app_ctx, monkeypatch):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    created = personality_service.create_personality(
        {
            "name": "ExplicitModel",
            "gender": "Male",
            "pause_threshold": 0.8,
            "message_history": 5,
            "assistant_model_id": vision.id,
        }
    )
    assert created.provider_ref == str(vision.id)
    assert created.assistant_model_id == vision.id
    assert created.provider_ref != DEFAULT_PROVIDER_REF


def test_model_endpoint_for_keeps_default_as_a_pointer():
    assert model_endpoint_for({"providerRef": "default", "assistantModelId": 4}) == (
        "default",
        None,
    )
    assert model_endpoint_for({"providerRef": "7"}) == ("id", 7)
    assert model_endpoint_for({"assistantModelId": 3}) == ("id", 3)
    assert model_endpoint_for({}) == ("default", None)
