"""qwen-fast is offered only when Ollama's tags list it."""

import json
from unittest.mock import MagicMock

from click.testing import CliRunner

from app.app import db
from commands import seed_db
from model.provider_model import RegistryModel
from pib_hermes_config.local_model import API_NAME, VISUAL_NAME, iter_reply
from provider_registry import PIB_CLOUD_API_NAME, STATUS_ACTIVE
from service import local_model_service, personality_service

QWEN_TAGS = {
    "models": [
        {"name": "llama3:latest", "model": "llama3:latest"},
        {
            "name": "qwen-fast:latest",
            "model": "qwen-fast:latest",
            "details": {"parent_model": "qwen2.5:1.5b"},
        },
        {"name": "qwen2.5:1.5b", "model": "qwen2.5:1.5b"},
    ]
}


def _models(client):
    response = client.get("/assistant-model")
    assert response.status_code == 200
    return response.get_json()["assistantModels"]


def _by_api_name(rows):
    return {row["apiName"]: row for row in rows}


def test_catalogue_lists_the_local_model_when_tags_include_it(app, monkeypatch):
    client = app.test_client()
    before = _models(client)
    monkeypatch.setattr(local_model_service, "fetch_tags", lambda: QWEN_TAGS)
    listed = _models(client)
    again = _models(client)

    extra = [
        row["apiName"] for row in listed if row["apiName"] not in _by_api_name(before)
    ]
    assert extra == [API_NAME]
    local = _by_api_name(listed)[API_NAME]
    assert local["visualName"] == VISUAL_NAME == "Local (qwen-fast)"
    assert local["credentialRef"] is None
    assert local["hasImageSupport"] is False
    assert local["status"] == STATUS_ACTIVE
    assert local["capabilities"]["offline"] is True
    assert local["isDefault"] is False
    assert [row["id"] for row in again if row["apiName"] == API_NAME] == [local["id"]]

    before_by_name = _by_api_name(before)
    for api_name, row in _by_api_name(listed).items():
        if api_name == API_NAME:
            continue
        assert row == before_by_name[api_name]

    providers = client.get("/provider").get_json()["providers"]
    offered = [model for provider in providers for model in provider["models"]]
    assert _by_api_name(offered)[API_NAME]["credentialRef"] is None
    cloud = _by_api_name(listed)[PIB_CLOUD_API_NAME]
    assert cloud["isDefault"] is True
    assert cloud["credentialRef"] is None


def test_catalogue_is_unchanged_when_tags_omit_the_model(app, monkeypatch):
    client = app.test_client()
    baseline = client.get("/assistant-model").get_json()

    monkeypatch.setattr(
        local_model_service,
        "fetch_tags",
        lambda: {"models": [{"name": "qwen2.5:1.5b"}, {"name": "llama3:latest"}]},
    )
    omitted = client.get("/assistant-model")
    assert omitted.status_code == 200
    assert omitted.get_json() == baseline

    def unavailable():
        raise OSError("connection refused")

    monkeypatch.setattr(local_model_service, "fetch_tags", unavailable)
    failed = client.get("/assistant-model")
    assert failed.status_code == 200
    assert failed.get_json() == baseline
    assert API_NAME not in _by_api_name(failed.get_json()["assistantModels"])

    providers = client.get("/provider").get_json()
    nested = [
        model["apiName"] for row in providers["providers"] for model in row["models"]
    ]
    assert API_NAME not in nested


def test_local_model_is_the_default_only_when_no_provider_key_is_configured(
    app, monkeypatch
):
    monkeypatch.setattr(local_model_service, "fetch_tags", lambda: QWEN_TAGS)
    client = app.test_client()
    resolved = client.get("/provider/default")
    assert resolved.status_code == 200
    assert resolved.get_json()["apiName"] == API_NAME

    with app.app_context():
        cloud = RegistryModel.query.filter_by(api_name=PIB_CLOUD_API_NAME).one()
        assert cloud.is_default is True
        cloud.provider.credential_ref = "provider-cloud"
        db.session.commit()

    resolved = client.get("/provider/default")
    assert resolved.get_json()["apiName"] == PIB_CLOUD_API_NAME
    assert resolved.get_json()["isDefault"] is True
    assert API_NAME in _by_api_name(_models(client))

    with app.app_context():
        cloud = RegistryModel.query.filter_by(api_name=PIB_CLOUD_API_NAME).one()
        cloud.provider.credential_ref = None
        db.session.commit()

    monkeypatch.setattr(local_model_service, "fetch_tags", lambda: {"models": []})
    resolved = client.get("/provider/default")
    assert resolved.get_json()["apiName"] == PIB_CLOUD_API_NAME
    assert API_NAME not in _by_api_name(_models(client))


def test_a_personality_can_store_the_local_model(app, monkeypatch):
    monkeypatch.setattr(local_model_service, "fetch_tags", lambda: QWEN_TAGS)
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    client = app.test_client()
    local_id = _by_api_name(_models(client))[API_NAME]["id"]
    created = client.post(
        "/voice-assistant/personality",
        json={"name": "LocalOnly", "providerRef": str(local_id)},
    )
    assert created.status_code == 201
    body = created.get_json()
    assert body["providerRef"] == str(local_id)
    assert body["assistantModelId"] == local_id


def test_seed_keeps_the_local_row_and_cloud_credentials(app, monkeypatch):
    monkeypatch.setattr(local_model_service, "fetch_tags", lambda: QWEN_TAGS)
    client = app.test_client()
    local_id = _by_api_name(_models(client))[API_NAME]["id"]
    with app.app_context():
        claude = RegistryModel.query.filter_by(api_name="claude-sonnet-5-5").one()
        claude.provider.credential_ref = "provider-claude"
        db.session.commit()
        result = CliRunner().invoke(seed_db, [])
        assert result.exception is None, result.output
        db.session.remove()
        kept = RegistryModel.query.filter_by(api_name=API_NAME).one()
        assert kept.id == local_id
        assert kept.provider.credential_ref is None
        claude = RegistryModel.query.filter_by(api_name="claude-sonnet-5-5").one()
        assert claude.provider.credential_ref == "provider-claude"
        assert (
            RegistryModel.query.filter_by(api_name=PIB_CLOUD_API_NAME).one().is_default
            is True
        )


class _Body:
    def __init__(self, payload: bytes):
        self.payload = payload

    def read(self) -> bytes:
        return self.payload

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False


def test_the_voice_assistant_calls_the_openai_compatible_endpoint(monkeypatch):
    monkeypatch.setenv("PIB_OLLAMA_BASE_URL", "http://host.docker.internal:11434")
    seen = {}

    def opener(outgoing, timeout):
        seen["url"] = outgoing.full_url
        seen["timeout"] = timeout
        seen["headers"] = dict(outgoing.header_items())
        seen["body"] = json.loads(outgoing.data.decode("utf-8"))
        return _Body(
            json.dumps({"choices": [{"message": {"content": "Hallo."}}]}).encode(
                "utf-8"
            )
        )

    assert list(iter_reply("Du bist pib.", [], "Wie spät ist es?", opener=opener)) == [
        "Hallo."
    ]
    assert seen["url"] == "http://host.docker.internal:11434/v1/chat/completions"
    assert seen["body"]["model"] == API_NAME
    assert seen["body"]["stream"] is False
    assert "Authorization" not in {key.title() for key in seen["headers"]}
    roles = [message["role"] for message in seen["body"]["messages"]]
    assert roles == ["system", "user"]
