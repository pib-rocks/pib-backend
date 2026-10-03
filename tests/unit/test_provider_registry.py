"""Provider registry: seeded ids, image filter, default pointer."""

from unittest.mock import MagicMock

from model.assistant_model import AssistantModel
from model.personality_model import Personality
from model.provider_model import Provider, RegistryModel
from pib_api_client.voice_assistant_client import model_endpoint_for
from provider_registry import (
    CAPABILITY_KEYS,
    DEFAULT_PROVIDER_REF,
    active_entries,
    capabilities_held_by_all,
    has_images_capability,
)
from service import personality_service, provider_service


def test_seed_copies_assistant_models_without_losing_ids(app_ctx):
    for entry in active_entries():
        assistant = AssistantModel.query.filter_by(visual_name=entry.visual_name).one()
        model = RegistryModel.query.filter_by(visual_name=entry.visual_name).one()
        assert assistant.api_name == entry.api_name
        assert model.id == assistant.id
        assert model.api_name == assistant.api_name
        assert model.has_image_support == assistant.has_image_support
        assert model.provider.name == entry.provider
        assert model.provider.endpoint_base is None
        assert model.provider.credential_ref is None
        assert has_images_capability(model.capabilities) is assistant.has_image_support
        siblings = [row.capabilities for row in model.provider.models]
        assert model.provider.capabilities == capabilities_held_by_all(siblings)

    for personality in Personality.query.all():
        assert personality.assistant_model_id is not None
        assert personality.provider_ref == str(personality.assistant_model_id)


def _listed_model_ids(client) -> set[int]:
    providers = client.get("/provider").get_json()["providers"]
    return {model["id"] for provider in providers for model in provider["models"]}


def test_selection_offers_only_rows_with_images_capability(app):
    with app.app_context():
        from app.app import db

        gpt6 = RegistryModel.query.filter_by(api_name="gpt-6").one()
        gemini = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
        live = RegistryModel.query.filter_by(api_name="gemini-3.8-live").one()
        claude = RegistryModel.query.filter_by(api_name="claude-sonnet-5-5").one()
        gpt6_id, gemini_id, live_id, claude_id = gpt6.id, gemini.id, live.id, claude.id
        all_ids = {row.id for row in RegistryModel.query.all()}
        # Clear one image flag so the filter shows. The live model has none.
        gemini.capabilities = {**gemini.capabilities, "images": False}
        db.session.commit()

    client = app.test_client()
    providers = client.get("/provider").get_json()["providers"]
    google = next(row for row in providers if row["name"] == "Google")
    assert set(google["models"][0]["capabilities"]) == {
        "tools",
        "images",
        "live",
        "stt",
        "tts",
    }
    offered = _listed_model_ids(client)
    assistant_offered = {
        row["id"]
        for row in client.get("/assistant-model").get_json()["assistantModels"]
    }
    # The assistant list is the image filter. The provider list also keeps
    # the named live model, which has no images.
    assert assistant_offered == all_ids - {gemini_id, live_id}
    assert offered == all_ids - {gemini_id}
    assert gpt6_id in offered
    assert claude_id in offered
    assert live_id in offered
    assert gemini_id not in offered
    assert live_id not in assistant_offered

    # The flag decides, not the name.
    with app.app_context():
        gemini = RegistryModel.query.filter_by(id=gemini_id).one()
        gemini.capabilities = {**gemini.capabilities, "images": True}
        claude = RegistryModel.query.filter_by(id=claude_id).one()
        claude.capabilities = {**claude.capabilities, "images": False}
        db.session.commit()

    offered = _listed_model_ids(client)
    assert gemini_id in offered
    assert live_id in offered
    assert claude_id not in offered
    by_id = client.get(f"/provider/{claude_id}")
    assert by_id.status_code == 200
    assert by_id.get_json()["capabilities"]["images"] is False
    assert "liveModel" not in by_id.get_json()


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

    original_default = provider_service.get_default_model()
    assert provider_service.resolve_model(created.provider_ref).id == (
        original_default.id
    )
    assert provider_service.resolve_provider(created.provider_ref).id == (
        original_default.provider_id
    )
    stored_ref = created.provider_ref

    gpt6 = RegistryModel.query.filter_by(api_name="gpt-6").one()
    original_default.is_default = False
    db.session.flush()
    gpt6.is_default = True
    db.session.commit()

    reloaded = Personality.query.filter_by(personality_id=created.personality_id).one()
    assert reloaded.provider_ref == stored_ref
    assert reloaded.assistant_model_id is None
    assert provider_service.resolve_model(reloaded.provider_ref).id == gpt6.id
    assert provider_service.resolve_provider(reloaded.provider_ref).id == (
        gpt6.provider_id
    )


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
    claude = RegistryModel.query.filter_by(api_name="claude-sonnet-5-5").one()
    created = personality_service.create_personality(
        {
            "name": "ExplicitModel",
            "gender": "Male",
            "pause_threshold": 0.8,
            "message_history": 5,
            "assistant_model_id": claude.id,
        }
    )
    assert created.provider_ref == str(claude.id)
    assert created.assistant_model_id == claude.id
    assert created.provider_ref != DEFAULT_PROVIDER_REF
    assert provider_service.resolve_provider(created.provider_ref).id == (
        claude.provider_id
    )


def test_one_provider_owns_many_models_and_the_personality_follows_the_model(
    app_ctx, monkeypatch
):
    """A provider holds the credential and the shared flags. Each model is its own row."""
    from app.app import db

    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    openai = Provider.query.filter_by(name="OpenAI").one()
    gpt6 = RegistryModel.query.filter_by(api_name="gpt-6").one()
    assert gpt6.provider_id == openai.id
    assert openai.endpoint_base is None
    assert openai.credential_ref is None
    assert openai.capabilities == gpt6.capabilities

    realtime = RegistryModel(
        provider_id=openai.id,
        api_name="gpt-realtime",
        visual_name="GPT Realtime",
        has_image_support=False,
        capabilities={
            "tools": False,
            "images": False,
            "live": True,
            "stt": False,
            "tts": False,
        },
        is_default=False,
    )
    db.session.add(realtime)
    db.session.flush()
    provider_service.sync_shared_capabilities(openai)
    openai.endpoint_base = "https://api.openai.com/v1"
    openai.credential_ref = "provider-openai"
    db.session.commit()

    assert realtime.provider_id == gpt6.provider_id
    assert realtime.capabilities["live"] is True
    assert gpt6.capabilities["images"] is True
    assert gpt6.capabilities["tools"] is True
    assert openai.capabilities == {key: False for key in CAPABILITY_KEYS}
    assert gpt6.provider.endpoint_base == "https://api.openai.com/v1"
    assert realtime.provider.credential_ref == "provider-openai"

    created = personality_service.create_personality(
        {
            "name": "Realtime",
            "gender": "Female",
            "pause_threshold": 0.8,
            "message_history": 5,
            "assistant_model_id": realtime.id,
        }
    )
    assert created.provider_ref == str(realtime.id)
    assert created.assistant_model_id == realtime.id
    assert provider_service.resolve_model(created.provider_ref).api_name == (
        "gpt-realtime"
    )
    assert provider_service.resolve_provider(created.provider_ref).id == openai.id
    assert provider_service.resolve_provider(created.provider_ref).name == "OpenAI"


def test_model_endpoint_for_keeps_default_as_a_pointer():
    assert model_endpoint_for({"providerRef": "default", "assistantModelId": 4}) == (
        "default",
        None,
    )
    assert model_endpoint_for({"providerRef": "7"}) == ("id", 7)
    assert model_endpoint_for({"assistantModelId": 3}) == ("id", 3)
    assert model_endpoint_for({}) == ("default", None)
