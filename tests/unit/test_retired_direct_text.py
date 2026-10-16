"""PC-48: retire only tryb Direct text; retain supported dispatch selections."""

from pathlib import Path
import importlib.util
from unittest.mock import MagicMock

import pytest
from pib_hermes_config.retired_direct_text import (
    RETIRED_DIRECT_TEXT_SENTENCE,
    legacy_direct_text_selected,
)
from pib_hermes_config.local_model import API_NAME as LOCAL_API_NAME


@pytest.mark.parametrize(
    "channel,api_name,mode,retired",
    [
        # PC-58: the tool-calling flag selects no transport any more. Direct on
        # the supported model stays, with tools off it is a plain chat.
        ("direct", "gemini-3.8-flash", "turn_based", False),
        ("direct", "gpt-6", None, True),
        ("direct", "claude-sonnet-5-5", None, True),
        ("smart", "gemini-3.8-flash", None, False),
        ("direct", "pib-cloud", None, False),
        ("direct", LOCAL_API_NAME, None, False),
        ("direct", "gemini-3.8-live", "live", False),
        ("direct", None, None, True),
    ],
)
def test_shared_dispatch_boundary(channel, api_name, mode, retired):
    assert (
        legacy_direct_text_selected(channel=channel, api_name=api_name, voice_mode=mode)
        is retired
    )


@pytest.mark.parametrize("token", [None, "obsolete-placeholder"])
def test_public_compatibility_stub_never_requests_even_without_token(
    monkeypatch, token
):
    path = (
        Path(__file__).resolve().parents[2]
        / "public_api_client/public_api_client/public_voice_client.py"
    )
    spec = importlib.util.spec_from_file_location("pc48_real_public_client", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    request = MagicMock(side_effect=AssertionError("retired text must never use HTTP"))
    monkeypatch.setattr(module.requests, "request", request)
    assert list(module.chat_completion("hello", "soul", [], token, "unused-image")) == [
        RETIRED_DIRECT_TEXT_SENTENCE
    ]
    request.assert_not_called()
    # Retaining the module's speech methods is intentional, not an endpoint-wide deletion.
    module._send_request = MagicMock()
    module._send_request.return_value.json.return_value = {"responseText": "speech"}
    assert module.speech_to_text(b"fixture", "old-speech-placeholder") == "speech"
    module._send_request.assert_called_once()


@pytest.fixture(autouse=True)
def no_real_profile_rpc(monkeypatch):
    from service import personality_service

    monkeypatch.setattr(personality_service, "_provision_profile", MagicMock())


def _model(api_name):
    from model.provider_model import RegistryModel

    return RegistryModel.query.filter_by(api_name=api_name).one()


def _create(client, api_name="gemini-3.8-flash", **extra):
    return client.post(
        "/voice-assistant/personality",
        json={
            "name": "PC-48 fixture",
            "modelRef": f"model:{_model(api_name).id}",
            "channel": "direct",
            **extra,
        },
    )


@pytest.mark.parametrize(
    "api_name,channel",
    [
        ("pib-cloud", "direct"),
        (LOCAL_API_NAME, "direct"),
        ("gemini-3.8-live", "direct"),
        ("gemini-3.8-flash", "smart"),
        # PC-58: the supported Direct model answers with tools off as a plain
        # chat instead of being sent down the retired path.
        ("gemini-3.8-flash", "direct"),
    ],
)
def test_supported_no_tool_selections_are_not_retired(
    app, app_ctx, monkeypatch, api_name, channel
):
    monkeypatch.setenv("PIB_HERMES_ENABLED", "true")
    client = app.test_client()
    if api_name == LOCAL_API_NAME:
        from service import local_model_service

        monkeypatch.setattr(
            local_model_service,
            "fetch_tags",
            lambda: {"models": [{"name": "qwen-fast:latest"}]},
        )
        assert client.get("/assistant-model").status_code == 200
    response = _create(client, api_name, channel=channel, toolCalling=False)
    assert response.status_code == 201, response.get_json()
    assert response.get_json()["toolCalling"] is False
    updated = client.put(
        "/voice-assistant/personality/" + response.get_json()["personalityId"],
        json={"name": "Still supported", "toolCalling": False},
    )
    assert updated.status_code == 200, updated.get_json()
    assert updated.get_json()["toolCalling"] is False


def test_new_legacy_selection_is_rejected_without_creating_row(app, app_ctx):
    from model.personality_model import Personality

    before = Personality.query.count()
    # PC-58: gpt-6 has no Direct transport of its own, so Direct on it is the
    # combination that is still refused. The tool-calling flag is not the cause.
    response = _create(app.test_client(), api_name="gpt-6", toolCalling=False)
    assert response.status_code == 400
    assert "channel" in str(response.get_json())
    assert "retired" in str(response.get_json()).lower()
    assert Personality.query.count() == before


def test_invalid_switch_rolls_back_row_and_does_not_write_soul(
    app, app_ctx, monkeypatch
):
    from service import soul_service

    monkeypatch.setenv("PIB_HERMES_ENABLED", "true")
    client = app.test_client()
    created = _create(client, api_name="gpt-6", channel="smart")
    assert created.status_code == 201
    body = created.get_json()
    url = "/voice-assistant/personality/" + body["personalityId"]
    write = MagicMock()
    monkeypatch.setattr(soul_service, "write_soul", write)
    response = client.put(
        url,
        json={
            "channel": "direct",
            "description": "MUST NOT WRITE",
            "name": "MUST NOT STORE",
        },
    )
    assert response.status_code == 400
    write.assert_not_called()
    stored = client.get(url).get_json()
    assert stored["channel"] == "smart"
    assert stored["name"] == body["name"]
    assert stored["description"] == body["description"]


@pytest.mark.parametrize(
    "start,change,field",
    [
        (
            {"api_name": "gpt-6", "channel": "smart"},
            {"channel": "direct"},
            "channel",
        ),
        (
            {"api_name": "gemini-3.8-flash", "channel": "direct"},
            {"model": "claude-sonnet-5-5"},
            "modelRef",
        ),
    ],
)
def test_channel_and_model_changes_cannot_reenter_legacy(
    app, app_ctx, monkeypatch, start, change, field
):
    monkeypatch.setenv("PIB_HERMES_ENABLED", "true")
    client = app.test_client()
    created = _create(client, **start)
    assert created.status_code == 201, created.get_json()
    body = created.get_json()
    url = "/voice-assistant/personality/" + body["personalityId"]
    payload = dict(change)
    if "model" in payload:
        payload["modelRef"] = f"model:{_model(payload.pop('model')).id}"
    response = client.put(url, json=payload)
    assert response.status_code == 400
    assert field in str(response.get_json())
    stored = client.get(url).get_json()
    assert stored["channel"] == body["channel"]
    assert stored["modelRef"] == body["modelRef"]


def test_old_selection_is_readable_editable_and_not_silently_migrated(app, app_ctx):
    from app.app import db
    from model.personality_model import Personality

    client = app.test_client()
    created = _create(client)
    assert created.status_code == 201
    pid = created.get_json()["personalityId"]
    row = Personality.query.filter_by(personality_id=pid).one()
    row.tool_calling = False
    db.session.commit()
    url = "/voice-assistant/personality/" + pid
    assert client.get(url).get_json()["toolCalling"] is False
    edited = client.put(url, json={"name": "Old selection, new name"})
    assert edited.status_code == 200, edited.get_json()
    assert edited.get_json()["channel"] == "direct"
    assert edited.get_json()["toolCalling"] is False
    fixed = client.put(url, json={"toolCalling": True})
    assert fixed.status_code == 200
    assert fixed.get_json()["toolCalling"] is True
