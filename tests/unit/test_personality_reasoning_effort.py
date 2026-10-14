"""Per-personality Hermes reasoning level (PR-1955).

A new personality starts at "none". The value round-trips through the API.
An unknown level is rejected. When the profile is provisioned, the level is
written to agent.reasoning_effort. NULL leaves the profile's existing level
alone. A write to the config root is the defect this ticket fixes.
"""

import pytest
import yaml
from marshmallow import ValidationError

from pib_hermes_config import REASONING_EFFORT_ERROR
from public_api_client.hermes_agent_client import (
    DEFAULT_MAX_TOKENS,
    DEFAULT_TEMPERATURE,
    _ensure_mcp_servers_pib,
)
from service import personality_service

PERSONALITY_URL = "/voice-assistant/personality"


@pytest.fixture()
def client(app):
    return app.test_client()


@pytest.fixture()
def provision(monkeypatch):
    calls = []

    def _provision(*_args, **kwargs):
        calls.append(kwargs)
        return {"ok": True}

    monkeypatch.setattr(personality_service, "_provision_profile", _provision)
    return calls


def test_new_personality_defaults_reasoning_effort_to_none(client, app_ctx, provision):
    response = client.post(PERSONALITY_URL, json={"name": "Quiet"})

    assert response.status_code == 201
    body = response.get_json()
    assert body["reasoningEffort"] == "none"
    assert provision[0]["reasoning_effort"] == "none"

    read = client.get(f"{PERSONALITY_URL}/{body['personalityId']}")
    assert read.status_code == 200
    assert read.get_json()["reasoningEffort"] == "none"


def test_reasoning_effort_round_trips_through_create_update_and_read(
    client, app_ctx, provision
):
    created = client.post(
        PERSONALITY_URL, json={"name": "Thinker", "reasoningEffort": "high"}
    )
    assert created.status_code == 201
    personality_id = created.get_json()["personalityId"]
    assert created.get_json()["reasoningEffort"] == "high"
    assert provision[-1]["reasoning_effort"] == "high"

    unchanged = client.put(
        f"{PERSONALITY_URL}/{personality_id}", json={"pauseThreshold": 1.2}
    )
    assert unchanged.status_code == 200
    assert unchanged.get_json()["reasoningEffort"] == "high"
    assert len(provision) == 1

    updated = client.put(
        f"{PERSONALITY_URL}/{personality_id}", json={"reasoningEffort": "minimal"}
    )
    assert updated.status_code == 200
    assert updated.get_json()["reasoningEffort"] == "minimal"
    assert provision[-1]["reasoning_effort"] == "minimal"

    again = client.get(f"{PERSONALITY_URL}/{personality_id}")
    assert again.get_json()["reasoningEffort"] == "minimal"

    cleared = client.put(
        f"{PERSONALITY_URL}/{personality_id}", json={"reasoningEffort": None}
    )
    assert cleared.status_code == 200
    assert cleared.get_json()["reasoningEffort"] is None
    assert provision[-1]["reasoning_effort"] is None


def test_unknown_reasoning_effort_is_rejected(client, app_ctx, provision):
    from schema.personality_schema import upload_personality_schema

    with pytest.raises(ValidationError) as exc_info:
        upload_personality_schema.load({"name": "Nope", "reasoningEffort": "turbo"})
    messages = exc_info.value.messages["reasoningEffort"]
    assert messages == [REASONING_EFFORT_ERROR]

    rejected = client.post(
        PERSONALITY_URL, json={"name": "Nope", "reasoningEffort": "turbo"}
    )
    assert rejected.status_code == 400
    assert provision == []

    created = client.post(PERSONALITY_URL, json={"name": "Kept"}).get_json()
    personality_id = created["personalityId"]
    rejected_update = client.put(
        f"{PERSONALITY_URL}/{personality_id}", json={"reasoningEffort": "High"}
    )
    assert rejected_update.status_code == 400
    read = client.get(f"{PERSONALITY_URL}/{personality_id}")
    assert read.get_json()["reasoningEffort"] == "none"


def test_reasoning_effort_is_written_to_the_agent_block_not_the_root(tmp_path):
    """Fails when the level is stored at the config root, which Hermes ignores."""
    pdir = tmp_path / "pib_level"
    pdir.mkdir()
    with open(pdir / "config.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "model": "gemini-3.8-flash",
                "provider": "gemini",
                "agent": {"reasoning_effort": "medium", "max_turns": 4},
            },
            fh,
        )

    _ensure_mcp_servers_pib(
        str(pdir),
        reasoning_effort="none",
        personality_reasoning=True,
    )

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)

    assert cfg["agent"]["reasoning_effort"] == "none"
    assert cfg["agent"]["max_turns"] == 4
    assert cfg["agent"]["max_tokens"] == DEFAULT_MAX_TOKENS
    assert cfg["agent"]["temperature"] == DEFAULT_TEMPERATURE
    assert "reasoning_effort" not in cfg
    assert "max_tokens" not in cfg
    assert "temperature" not in cfg


def test_null_reasoning_effort_leaves_the_profile_setting_alone(tmp_path):
    pdir = tmp_path / "pib_unmanaged"
    pdir.mkdir()
    with open(pdir / "config.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "model": "gemini-3.8-flash",
                "agent": {"reasoning_effort": "medium", "max_turns": 9},
            },
            fh,
        )

    _ensure_mcp_servers_pib(
        str(pdir), reasoning_effort=None, personality_reasoning=True
    )

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)

    assert cfg["agent"]["reasoning_effort"] == "medium"
    assert cfg["agent"]["max_turns"] == 9
    assert "reasoning_effort" not in cfg
