"""The catalogue is the only source of models, and a removed model is gone."""

from unittest.mock import MagicMock

from click.testing import CliRunner

from app.app import db
from commands import seed_db
from model.assistant_model import AssistantModel
from model.chat_model import Chat
from model.personality_model import Personality
from model.provider_model import Provider
from provider_registry import (
    CAPABILITY_KEYS,
    CATALOGUE,
    DEFAULT_PROVIDER_API_NAME,
    MISSING_MODEL_CHAT_MESSAGE,
    PIB_CLOUD_API_NAME,
    STATUS_ACTIVE,
    STATUS_UNCONFIRMED,
    active_api_names,
    capabilities_for,
)
from service import personality_service, provider_service

EVA_PERSONALITY_ID = "8f73b580-927e-41c2-98ac-e5df070e7288"
THOMAS_PERSONALITY_ID = "8b310f95-92cd-4512-b42a-d3fe29c4bb8a"

#: The maintained catalogue, in the order the file keeps it.
SUPPORTED_API_NAMES = (
    "gemini-3.8-flash",
    "gpt-6",
    "claude-sonnet-5-5",
    PIB_CLOUD_API_NAME,
)

REMOVED_API_NAME = "hermes-agent"

#: Rows the robot carries today. None of them may survive.
OLD_ROWS = (
    ("gpt-4o", "GPT-4o [Vision]", True),
    ("gpt-4o", "GPT-4o [Text]", False),
    ("gpt-3.5-turbo", "GPT-3.5 [Text]", False),
    ("anthropic.claude-3-sonnet-20240229-v1:0", "Claude 3 Sonnet [Vision]", True),
    ("gemini-3.5-flash", "Gemini 3.5 Flash", False),
)
OLD_API_NAMES = frozenset(row[0] for row in OLD_ROWS)


def _insert_old_rows() -> list[int]:
    """Put the legacy rows back into a seeded database, as the robot has them."""
    ids = []
    for api_name, visual_name, images in OLD_ROWS:
        model = AssistantModel(
            api_name=api_name, visual_name=visual_name, has_image_support=images
        )
        db.session.add(model)
        db.session.flush()
        db.session.add(
            Provider(
                id=model.id,
                api_name=api_name,
                visual_name=visual_name,
                has_image_support=images,
                capabilities=capabilities_for(api_name, images),
                is_default=False,
            )
        )
        ids.append(model.id)
    db.session.commit()
    return ids


def _stub_provisioning(monkeypatch) -> None:
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )


def test_catalogue_keeps_only_the_supported_models_in_order():
    active = [entry for entry in CATALOGUE if entry.status == STATUS_ACTIVE]
    assert tuple(entry.api_name for entry in active) == SUPPORTED_API_NAMES
    assert active_api_names() == frozenset(SUPPORTED_API_NAMES)
    assert not active_api_names() & OLD_API_NAMES
    for entry in active:
        assert entry.visual_name
        flags = entry.capabilities()
        assert set(flags) == set(CAPABILITY_KEYS)
        assert all(isinstance(flags[key], bool) for key in CAPABILITY_KEYS)
    listed = [entry.api_name for entry in CATALOGUE if entry.api_name]
    assert len(listed) == len(set(listed))
    assert not set(listed) & OLD_API_NAMES

    # No status other than active and unconfirmed exists. Nothing is retired.
    assert {entry.status for entry in CATALOGUE} == {
        STATUS_ACTIVE,
        STATUS_UNCONFIRMED,
    }
    unconfirmed = [entry for entry in CATALOGUE if entry.status == STATUS_UNCONFIRMED]
    assert [entry.provider for entry in unconfirmed] == ["OpenAI", "Mistral"]
    assert unconfirmed[0].api_name == "gpt-realtime"
    assert unconfirmed[1].api_name is None

    defaults = [entry for entry in CATALOGUE if entry.is_default]
    assert len(defaults) == 1
    assert defaults[0].provider == "pib.Cloud"
    assert defaults[0].status == STATUS_ACTIVE
    assert defaults[0].api_name == DEFAULT_PROVIDER_API_NAME == PIB_CLOUD_API_NAME
    assert defaults[0].images is True
    assert REMOVED_API_NAME not in active_api_names()
    assert REMOVED_API_NAME not in listed
    assert all(entry.provider != "hermes" for entry in CATALOGUE)


def test_no_old_model_is_offered_or_stored(app):
    client = app.test_client()
    with app.app_context():
        assistant_names = {model.api_name for model in AssistantModel.query.all()}
        provider_names = {row.api_name for row in Provider.query.all()}
    assert assistant_names == set(SUPPORTED_API_NAMES)
    assert provider_names == set(SUPPORTED_API_NAMES)

    for path, key in (
        ("/provider", "providers"),
        ("/assistant-model", "assistantModels"),
    ):
        offered = client.get(path).get_json()[key]
        assert {row["apiName"] for row in offered} == set(SUPPORTED_API_NAMES)
        assert all(row["status"] == STATUS_ACTIVE for row in offered)
    default = client.get("/provider/default").get_json()
    assert default["apiName"] == PIB_CLOUD_API_NAME
    assert default["isDefault"] is True
    assert default["visualName"] == "pib.Cloud"


def test_seeding_an_existing_database_removes_the_old_rows(app, monkeypatch):
    _stub_provisioning(monkeypatch)
    client = app.test_client()
    with app.app_context():
        old_ids = _insert_old_rows()
        pointed_at = old_ids[0]
        on_old = personality_service.create_personality(
            {
                "name": "OnOldModel",
                "gender": "Female",
                "pause_threshold": 0.8,
                "message_history": 5,
                "assistant_model_id": pointed_at,
            }
        )
        db.session.commit()
        personality_id = on_old.personality_id
        kept = Provider.query.filter_by(api_name="claude-sonnet-5-5").one()
        kept.credential_ref = "provider-claude"
        db.session.commit()
        assert {row.api_name for row in Provider.query.all()} & OLD_API_NAMES

        result = CliRunner().invoke(seed_db, [])
        assert result.exception is None, result.output
        assert "Removed model rows that are not in the catalogue" in result.output
        db.session.remove()

        assert {row.api_name for row in Provider.query.all()} == set(
            SUPPORTED_API_NAMES
        )
        assert {model.api_name for model in AssistantModel.query.all()} == set(
            SUPPORTED_API_NAMES
        )
        assert Provider.query.filter(Provider.id.in_(old_ids)).count() == 0
        assert provider_service.get_default_provider().api_name == PIB_CLOUD_API_NAME
        # Rows that stay are untouched.
        kept = Provider.query.filter_by(api_name="claude-sonnet-5-5").one()
        assert kept.credential_ref == "provider-claude"
        # The personality is not moved. It keeps pointing at the gone row.
        reloaded = Personality.query.filter_by(personality_id=personality_id).one()
        assert reloaded.provider_ref == str(pointed_at)
        assert reloaded.assistant_model_id is None

    body = client.get(f"/voice-assistant/personality/{personality_id}").get_json()
    assert body["needsNewModel"] is True
    assert body["providerRef"] == str(pointed_at)
    assert client.get(f"/provider/{pointed_at}").status_code == 404


def test_seeding_an_existing_database_moves_the_default_to_pib_cloud(app):
    with app.app_context():
        other = Provider.query.filter_by(api_name="gpt-6").one()
        pib_cloud = Provider.query.filter_by(api_name=PIB_CLOUD_API_NAME).one()
        pib_cloud.is_default = False
        db.session.flush()
        other.is_default = True
        db.session.commit()

        result = CliRunner().invoke(seed_db, [])
        assert result.exception is None, result.output
        db.session.remove()

        assert provider_service.get_default_provider().api_name == PIB_CLOUD_API_NAME
        assert Provider.query.filter_by(is_default=True).count() == 1


def test_seeding_removes_hermes_agent_and_refuses_its_chat(app, monkeypatch):
    """A personality on the removed row keeps its reference and cannot chat."""
    _stub_provisioning(monkeypatch)
    client = app.test_client()
    with app.app_context():
        existing = AssistantModel.query.filter_by(
            api_name=REMOVED_API_NAME
        ).one_or_none()
        if existing is None:
            model = AssistantModel(
                api_name=REMOVED_API_NAME,
                visual_name="Hermes Agent (selbstlernend)",
                has_image_support=True,
            )
            db.session.add(model)
            db.session.flush()
            db.session.add(
                Provider(
                    id=model.id,
                    api_name=REMOVED_API_NAME,
                    visual_name=model.visual_name,
                    has_image_support=True,
                    capabilities=capabilities_for(REMOVED_API_NAME, True),
                    is_default=False,
                )
            )
            db.session.commit()
            removed_id = model.id
        else:
            removed_id = existing.id
        on_removed = personality_service.create_personality(
            {
                "name": "OnHermes",
                "gender": "Male",
                "pause_threshold": 0.8,
                "message_history": 5,
                "assistant_model_id": removed_id,
            }
        )
        db.session.commit()
        personality_id = on_removed.personality_id
        assert on_removed.provider_ref == str(removed_id)
        assert Provider.query.filter_by(api_name=REMOVED_API_NAME).count() == 1

        result = CliRunner().invoke(seed_db, [])
        assert result.exception is None, result.output
        assert REMOVED_API_NAME in result.output
        db.session.remove()

        assert Provider.query.filter_by(api_name=REMOVED_API_NAME).count() == 0
        assert AssistantModel.query.filter_by(api_name=REMOVED_API_NAME).count() == 0
        assert {row.api_name for row in Provider.query.all()} == set(
            SUPPORTED_API_NAMES
        )
        reloaded = Personality.query.filter_by(personality_id=personality_id).one()
        assert reloaded.provider_ref == str(removed_id)
        assert reloaded.assistant_model_id is None
        assert provider_service.get_default_provider().api_name == PIB_CLOUD_API_NAME

    body = client.get(f"/voice-assistant/personality/{personality_id}").get_json()
    assert body["needsNewModel"] is True
    assert body["providerRef"] == str(removed_id)
    refused = client.post(
        "/voice-assistant/chat",
        json={"topic": "removed", "personalityId": personality_id},
    )
    assert refused.status_code == 422
    assert refused.get_json()["error"] == MISSING_MODEL_CHAT_MESSAGE


def test_a_personality_on_a_removed_row_needs_a_new_model_and_cannot_chat(
    app, monkeypatch
):
    """The row is deleted by hand. No status marker is involved."""
    _stub_provisioning(monkeypatch)
    client = app.test_client()
    with app.app_context():
        gpt6 = Provider.query.filter_by(api_name="gpt-6").one()
        gpt6_id = gpt6.id
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "OnRemovedRow",
            "gender": "Male",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": gpt6_id,
        },
    )
    assert created.status_code == 201
    personality_id = created.get_json()["personalityId"]
    assert created.get_json()["needsNewModel"] is False
    assert (
        client.post(
            "/voice-assistant/chat",
            json={"topic": "before", "personalityId": personality_id},
        ).status_code
        == 201
    )

    with app.app_context():
        db.session.delete(Provider.query.filter_by(id=gpt6_id).one())
        db.session.commit()
        before = Chat.query.filter_by(personality_id=personality_id).count()

    body = client.get(f"/voice-assistant/personality/{personality_id}").get_json()
    assert body["needsNewModel"] is True
    assert body["providerRef"] == str(gpt6_id)
    assert body["liveModel"] is None
    assert body["voiceStartMode"] == "turn_based"
    listed = client.get("/voice-assistant/personality").get_json()[
        "voiceAssistantPersonalities"
    ]
    flags = {row["personalityId"]: row["needsNewModel"] for row in listed}
    assert flags[personality_id] is True
    assert flags[EVA_PERSONALITY_ID] is False

    refused = client.post(
        "/voice-assistant/chat",
        json={"topic": "removed", "personalityId": personality_id},
    )
    assert refused.status_code == 422
    assert refused.get_json()["error"] == MISSING_MODEL_CHAT_MESSAGE
    assert "no longer available" in MISSING_MODEL_CHAT_MESSAGE
    with app.app_context():
        after = Chat.query.filter_by(personality_id=personality_id).count()
    assert after == before

    # Choosing a current model in settings is the way out.
    with app.app_context():
        claude_id = Provider.query.filter_by(api_name="claude-sonnet-5-5").one().id
    repaired = client.put(
        f"/voice-assistant/personality/{personality_id}",
        json={"assistantModelId": claude_id},
    )
    assert repaired.status_code == 200
    assert repaired.get_json()["needsNewModel"] is False
    assert repaired.get_json()["providerRef"] == str(claude_id)
    started = client.post(
        "/voice-assistant/chat",
        json={"topic": "repaired", "personalityId": personality_id},
    )
    assert started.status_code == 201


def test_example_personalities_ship_on_pib_cloud(app):
    client = app.test_client()
    with app.app_context():
        pib_cloud_id = Provider.query.filter_by(api_name=PIB_CLOUD_API_NAME).one().id
    for personality_id, name in (
        (EVA_PERSONALITY_ID, "Eva"),
        (THOMAS_PERSONALITY_ID, "Thomas"),
    ):
        body = client.get(f"/voice-assistant/personality/{personality_id}").get_json()
        assert body["name"] == name
        assert body["assistantModelId"] == pib_cloud_id
        assert body["providerRef"] == str(pib_cloud_id)
        assert body["needsNewModel"] is False
        started = client.post(
            "/voice-assistant/chat",
            json={"topic": "example", "personalityId": personality_id},
        )
        assert started.status_code == 201
    eva = client.get(f"/voice-assistant/personality/{EVA_PERSONALITY_ID}").get_json()
    assert eva["gender"] == "Female"
    assert eva["pauseThreshold"] == 0.8
    assert eva["messageHistory"] == 5
    thomas = client.get(
        f"/voice-assistant/personality/{THOMAS_PERSONALITY_ID}"
    ).get_json()
    assert thomas["gender"] == "Male"
    assert thomas["pauseThreshold"] == 1.0
    assert thomas["messageHistory"] == 15


def test_new_personality_follows_the_default_route(app, monkeypatch):
    _stub_provisioning(monkeypatch)
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
    with app.app_context():
        resolved = provider_service.resolve_provider(body["providerRef"])
        assert resolved.api_name == PIB_CLOUD_API_NAME
        gemini = Provider.query.filter_by(api_name="gemini-3.8-flash").one()
        gemini_id = gemini.id
        assert gemini.capabilities["live"] is True
        assert gemini.capabilities["images"] is True
    started = client.post(
        "/voice-assistant/chat",
        json={"topic": "current", "personalityId": body["personalityId"]},
    )
    assert started.status_code == 201

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
    assert client.get(f"/provider/{gemini_id}").get_json()["status"] == STATUS_ACTIVE
    started = client.post(
        "/voice-assistant/chat",
        json={"topic": "gemini", "personalityId": explicit_body["personalityId"]},
    )
    assert started.status_code == 201
