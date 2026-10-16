"""A personality's description mirrors its SOUL.md.

The SOUL template no longer advertises the MCP tools: a SOUL that named them
made the model talk about its tools or write the calls out as text, and the
agent gets the real names and schemas from the MCP server itself. The
assertions that pinned the "## Verfuegbare MCP-Werkzeuge (pib_mcp_server)"
section are therefore inverted here into assertions that it is absent, and a
description stored under the old template is stripped on the write path.
"""

import pytest
from unittest.mock import MagicMock
from pib_hermes_config import (
    MCP_TOOLS,
    MCP_TOOLS_SOUL_SECTION,
    build_default_soul_text,
)
from service import personality_service, soul_service

# The tool names that used to be documented in the SOUL and must not come back.
EXPECTED_MCP_TOOLS = (
    "mcp__pib__list_motors",
    "mcp__pib__get_state",
    "mcp__pib__list_poses",
    "mcp__pib__list_programs",
    "mcp__pib__capture_image",
    "mcp__pib__move_motor",
    "mcp__pib__apply_pose",
    "mcp__pib__run_program",
    "mcp__pib__set_led",
    "mcp__pib__set_relay",
    "mcp__pib__soul_append",
)


def _legacy_description_with_mcp_section(
    personality_name: str, custom_description: str | None = None
) -> str:
    """A description exactly as the old template stored it, tool section and all."""
    text = build_default_soul_text(personality_name, custom_description)
    return text.rstrip("\n") + "\n\n" + MCP_TOOLS_SOUL_SECTION.strip() + "\n"


def _assert_no_tool_prose(text: str) -> None:
    assert "MCP-Werkzeuge" not in text
    assert MCP_TOOLS_SOUL_SECTION.strip() not in text
    for tool in EXPECTED_MCP_TOOLS:
        assert tool not in text
    for tool, _description in MCP_TOOLS:
        assert tool not in text


@pytest.fixture()
def client(app):
    return app.test_client()


@pytest.fixture(autouse=True)
def provision_profiles_in_sandbox(monkeypatch):
    def provision(personality_id, personality_name=None, soul_text=None, **_kwargs):
        text = build_default_soul_text(personality_name or "pib", soul_text)
        soul_service.write_soul(personality_id, text, personality_name or "pib")
        return {"ok": True}

    monkeypatch.setattr(personality_service, "_provision_profile", provision)


def test_update_description_writes_soul_file(
    tmp_path, monkeypatch, app_ctx, make_personality
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    p = make_personality(description="alt")
    personality_service.update_personality(p.personality_id, {"description": "neu"})

    assert soul_service.read_soul(p.personality_id) == "neu"


def test_create_personality_populates_description_with_full_soul(
    tmp_path, monkeypatch, app_ctx, make_personality
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    p = make_personality(name="Eva", description="")
    expected = build_default_soul_text("Eva")

    assert p.description == expected
    assert soul_service.read_soul(p.personality_id) == expected
    assert "Du bist der humanoide Roboter Eva." in p.description
    _assert_no_tool_prose(p.description)


def test_create_personality_embeds_custom_description_in_soul(
    tmp_path, monkeypatch, app_ctx, make_personality
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    p = make_personality(name="Thomas", description="Sei freundlich und neugierig.")
    expected = build_default_soul_text("Thomas", "Sei freundlich und neugierig.")

    assert p.description == expected
    assert "Sei freundlich und neugierig." in p.description
    assert soul_service.read_soul(p.personality_id) == expected
    _assert_no_tool_prose(p.description)


def test_create_personality_strips_the_old_tool_section_from_a_description(
    tmp_path, monkeypatch, app_ctx, make_personality
):
    """A description written under the old template must not put the prose back.

    Such a stored description already carries the generated section; folding it
    in as-is would have re-created the SOUL this change removes it from.
    """
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    stored = "Sei freundlich.\n\n" + MCP_TOOLS_SOUL_SECTION.strip() + "\n"
    p = make_personality(name="Nora", description=stored)

    assert p.description == build_default_soul_text("Nora", "Sei freundlich.")
    assert soul_service.read_soul(p.personality_id) == p.description
    _assert_no_tool_prose(p.description)


def test_update_description_with_the_old_tool_section_writes_a_cleaned_soul(
    tmp_path, monkeypatch, app_ctx, make_personality
):
    """The SOUL.md the agent reads is cleaned even when the row still has the block."""
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    p = make_personality(name="Nora", description="alt")
    stored = _legacy_description_with_mcp_section("Nora", "Sei freundlich.")

    personality_service.update_personality(p.personality_id, {"description": stored})

    written = soul_service.read_soul(p.personality_id)
    assert written == build_default_soul_text("Nora", "Sei freundlich.")
    _assert_no_tool_prose(written)


def test_get_personality_backfills_empty_description_from_soul(
    tmp_path, monkeypatch, app_ctx, make_personality
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    from app.app import db

    p = make_personality(name="BackfillBot", description="")
    expected = p.description
    # Simulate legacy rows where description stayed empty after SOUL.md was written.
    p.description = ""
    db.session.flush()

    loaded = personality_service.get_personality(p.personality_id)
    assert loaded.description == expected
    assert "Du bist der humanoide Roboter BackfillBot." in loaded.description


def test_get_all_personalities_backfills_empty_descriptions_from_soul(
    tmp_path, monkeypatch, app_ctx, make_personality
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    from app.app import db

    p = make_personality(name="ListBot", description="")
    expected = p.description
    p.description = ""
    db.session.flush()

    all_personalities = personality_service.get_all_personalities()
    match = next(x for x in all_personalities if x.personality_id == p.personality_id)
    assert match.description == expected


def test_api_create_and_get_return_full_soul_description(
    client, app_ctx, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    from model.assistant_model import AssistantModel

    model = AssistantModel.query.first()
    create_resp = client.post(
        "/voice-assistant/personality",
        json={
            "name": "ApiSoulBot",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": model.id,
            "description": "",
        },
    )
    assert create_resp.status_code == 201
    created = create_resp.get_json()
    expected = build_default_soul_text("ApiSoulBot")
    assert created["description"] == expected
    _assert_no_tool_prose(created["description"])

    get_resp = client.get(f"/voice-assistant/personality/{created['personalityId']}")
    assert get_resp.status_code == 200
    assert get_resp.get_json()["description"] == expected


def test_api_creation_provisions_profile_immediately(client, app_ctx, monkeypatch):
    from model.assistant_model import AssistantModel

    provision = MagicMock(return_value={"ok": True})
    monkeypatch.setattr(personality_service, "_provision_profile", provision)
    model = AssistantModel.query.first()
    response = client.post(
        "/voice-assistant/personality",
        json={
            "name": "CreationBot",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": model.id,
            "description": "Custom.",
        },
    )

    assert response.status_code == 201
    body = response.get_json()
    assert body["profileProvisioned"] is True
    # The provisioning call carries the personality's model and provider
    # (PR-1930b) and its reasoning level, which a new personality stores
    # as "none".
    from provider_registry import provider_name_for

    provision.assert_called_once_with(
        body["personalityId"],
        personality_name="CreationBot",
        soul_text="Custom.",
        reasoning_effort="none",
        model=model.api_name,
        provider=provider_name_for(model.api_name),
    )


def test_api_creation_keeps_row_and_reports_profile_failure(
    client, app_ctx, monkeypatch
):
    from model.assistant_model import AssistantModel

    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("factory down")),
    )
    model = AssistantModel.query.first()
    response = client.post(
        "/voice-assistant/personality",
        json={
            "name": "Unprovisioned",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": model.id,
            "description": "",
        },
    )

    assert response.status_code == 201
    assert response.get_json()["profileProvisioned"] is False
    assert personality_service.get_personality(response.get_json()["personalityId"])
