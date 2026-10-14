"""PR-1930b: the backend writes the personality's model into its Hermes profile.

On create and on a model change during an update, the daemon is told the model
api_name, the provider account's name and its endpoint, so the smart chat runs
the model the personality is set to.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from model.provider_model import RegistryModel
from service import personality_service


@pytest.fixture()
def provision(monkeypatch):
    calls = []

    def _provision(
        personality_id,
        personality_name=None,
        soul_text=None,
        model=None,
        provider=None,
        endpoint_base=None,
        **_kwargs,
    ):
        calls.append(
            {
                "personality_id": personality_id,
                "model": model,
                "provider": provider,
                "endpoint_base": endpoint_base,
            }
        )
        return {"ok": True}

    monkeypatch.setattr(personality_service, "_provision_profile", _provision)
    return calls


def _model(api_name):
    return RegistryModel.query.filter_by(api_name=api_name).one()


def test_create_sends_the_personalitys_model_and_provider(app_ctx, provision):
    gemini = _model("gemini-3.8-flash")

    personality_service.create_personality(
        {"name": "Gemini", "model_ref": f"model:{gemini.id}"}
    )

    assert len(provision) == 1
    assert provision[0]["model"] == "gemini-3.8-flash"
    assert provision[0]["provider"] == "Google"
    # Gemini's route is a built-in Hermes provider: no endpoint is sent.
    assert provision[0]["endpoint_base"] is None


def test_create_sends_an_openai_model(app_ctx, provision):
    model = _model("gpt-6")

    personality_service.create_personality(
        {"name": "GPT", "model_ref": f"model:{model.id}"}
    )

    assert provision[0]["model"] == "gpt-6"
    assert provision[0]["provider"] == "OpenAI"


def test_create_on_the_default_route_sends_pib_cloud(app_ctx, provision):
    personality_service.create_personality({"name": "Default"})

    assert provision[0]["model"] == "pib-cloud"
    assert provision[0]["provider"] == "pib.Cloud"


def test_create_sends_the_local_endpoint_base(app_ctx, provision, monkeypatch):
    from pib_hermes_config.local_model import openai_base_url
    from service import local_model_service

    tags = {"models": [{"name": "qwen-fast:latest", "model": "qwen-fast:latest"}]}
    monkeypatch.setattr(local_model_service, "fetch_tags", lambda: tags)
    local = local_model_service.ensure_row()

    personality_service.create_personality(
        {"name": "Lokal", "model_ref": f"model:{local.id}"}
    )

    assert provision[0]["model"] == "qwen-fast"
    assert provision[0]["provider"] == "Local"
    assert provision[0]["endpoint_base"] == openai_base_url()


def test_update_reprovisions_only_when_the_model_changed(app_ctx, provision):
    gemini = _model("gemini-3.8-flash")
    gpt = _model("gpt-6")
    personality = personality_service.create_personality(
        {"name": "Switch", "model_ref": f"model:{gemini.id}"}
    )
    provision.clear()

    # A change that leaves the model alone must not re-provision.
    personality_service.update_personality(
        personality.personality_id, {"pause_threshold": 1.1}
    )
    assert provision == []

    personality_service.update_personality(
        personality.personality_id, {"model_ref": f"model:{gpt.id}"}
    )
    assert len(provision) == 1
    assert provision[0]["model"] == "gpt-6"
    assert provision[0]["provider"] == "OpenAI"


def test_daemon_payload_receives_the_model_fields(monkeypatch):
    """_provision_profile puts the model fields on the HTTP payload."""
    captured = {}

    class _Response:
        status_code = 200

        def json(self):
            return {"ok": True}

    def fake_post(url, json=None, timeout=None):
        captured["url"] = url
        captured["json"] = json
        return _Response()

    import requests

    monkeypatch.setattr(requests, "post", fake_post)

    personality_service._provision_profile(
        "p-1",
        personality_name="Eva",
        soul_text="soul",
        model="qwen-fast",
        provider="Local",
        endpoint_base="http://host.docker.internal:11434/v1",
    )

    assert captured["url"].endswith("/profile")
    assert captured["json"]["model"] == "qwen-fast"
    assert captured["json"]["provider"] == "Local"
    assert captured["json"]["endpoint_base"] == "http://host.docker.internal:11434/v1"
