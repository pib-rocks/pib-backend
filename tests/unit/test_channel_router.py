"""Channel setting, identity stability, and the installer flag."""

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from pib_hermes_config import profile_dir_for
from pib_hermes_config import channel as channel_mod
from pib_hermes_config.channel import (
    CHANNEL_DIRECT,
    CHANNEL_SMART,
    direct_system_prompt,
    effective_channel,
    smart_chats_enabled,
)
from service import personality_service, soul_service


def test_direct_system_prompt_is_the_soul_and_ignores_memory():
    assert direct_system_prompt("  Du bist Eva.  ") == "Du bist Eva."
    assert direct_system_prompt("") == direct_system_prompt(None)
    assert "Roboter" in direct_system_prompt(None)


def test_missing_marker_keeps_smart_available(monkeypatch, tmp_path):
    monkeypatch.delenv("PIB_SMART_CHATS", raising=False)
    monkeypatch.setattr(channel_mod, "SMART_CHATS_FILE", str(tmp_path / "missing"))

    assert smart_chats_enabled() is True
    assert effective_channel("smart") == CHANNEL_SMART
    assert effective_channel("direct") == CHANNEL_DIRECT
    assert effective_channel(None) == CHANNEL_SMART


def test_disabled_marker_forces_direct_without_rewriting_the_value(
    monkeypatch, tmp_path
):
    monkeypatch.delenv("PIB_SMART_CHATS", raising=False)
    marker = tmp_path / "pib_smart_chats"
    marker.write_text("disabled\n", encoding="utf-8")
    monkeypatch.setattr(channel_mod, "SMART_CHATS_FILE", str(marker))

    assert smart_chats_enabled() is False
    assert effective_channel(CHANNEL_SMART) == CHANNEL_DIRECT


def test_env_override_beats_the_marker_file(monkeypatch, tmp_path):
    marker = tmp_path / "pib_smart_chats"
    marker.write_text("enabled\n", encoding="utf-8")
    monkeypatch.setattr(channel_mod, "SMART_CHATS_FILE", str(marker))
    monkeypatch.setenv("PIB_SMART_CHATS", "0")

    assert smart_chats_enabled() is False


@pytest.fixture()
def client(app):
    return app.test_client()


@pytest.fixture(autouse=True)
def provision_profiles_in_sandbox(monkeypatch):
    def provision(personality_id, personality_name=None, soul_text=None, **_kwargs):
        from pib_hermes_config import build_default_soul_text

        text = build_default_soul_text(personality_name or "pib", soul_text)
        soul_service.write_soul(personality_id, text, personality_name or "pib")
        return {"ok": True}

    monkeypatch.setattr(personality_service, "_provision_profile", provision)


def test_new_personality_defaults_to_smart(app_ctx, make_personality):
    personality = make_personality(name="Eva", description="Sei freundlich.")

    assert personality.channel == CHANNEL_SMART


def test_switching_channel_does_not_change_the_identity(
    tmp_path, monkeypatch, app_ctx, make_personality
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    personality = make_personality(name="Eva", description="Sei freundlich.")
    description = personality.description
    soul = soul_service.read_soul(personality.personality_id)
    memory = (
        Path(profile_dir_for(personality.personality_id)) / "memories" / "MEMORY.md"
    )
    memory.parent.mkdir(parents=True)
    memory.write_text("remember this\n", encoding="utf-8")
    memory_bytes = memory.read_bytes()

    updated = personality_service.update_personality(
        personality.personality_id, {"channel": CHANNEL_DIRECT}
    )
    assert updated.channel == CHANNEL_DIRECT
    assert updated.description == description
    assert soul_service.read_soul(personality.personality_id) == soul
    assert memory.read_bytes() == memory_bytes

    restored = personality_service.update_personality(
        personality.personality_id, {"channel": CHANNEL_SMART}
    )
    assert restored.channel == CHANNEL_SMART
    assert restored.description == description
    assert memory.read_bytes() == memory_bytes


def test_api_exposes_both_channels_and_keeps_one_identity(client, app_ctx):
    from model.assistant_model import AssistantModel

    model = AssistantModel.query.first()
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "ChannelBot",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": model.id,
            "description": "Ein Text.",
        },
    )
    assert created.status_code == 201
    body = created.get_json()
    assert body["channel"] == CHANNEL_SMART
    assert body["effectiveChannel"] == CHANNEL_SMART
    assert body["smartChatsEnabled"] is True
    identity = body["description"]

    switched = client.put(
        f"/voice-assistant/personality/{body['personalityId']}",
        json={"channel": CHANNEL_DIRECT},
    )
    assert switched.status_code == 200
    switched_body = switched.get_json()
    assert switched_body["channel"] == CHANNEL_DIRECT
    assert switched_body["description"] == identity

    offered = client.get("/system/chat-channels")
    assert offered.status_code == 200
    assert offered.get_json() == {
        "smartChatsEnabled": True,
        "channels": [CHANNEL_SMART, CHANNEL_DIRECT],
        "defaultChannel": CHANNEL_SMART,
    }


def test_disabled_installer_offers_only_direct(client, app_ctx, monkeypatch):
    from model.assistant_model import AssistantModel

    monkeypatch.setenv("PIB_SMART_CHATS", "0")
    model = AssistantModel.query.first()
    payload = {
        "name": "DirectOnly",
        "gender": "Female",
        "pauseThreshold": 0.8,
        "messageHistory": 5,
        "assistantModelId": model.id,
        "description": "Bleibt.",
    }
    rejected = client.post(
        "/voice-assistant/personality",
        json={**payload, "channel": CHANNEL_SMART},
    )
    assert rejected.status_code == 400

    created = client.post("/voice-assistant/personality", json=payload)
    assert created.status_code == 201
    body = created.get_json()
    assert body["channel"] == CHANNEL_DIRECT
    assert body["effectiveChannel"] == CHANNEL_DIRECT
    assert body["smartChatsEnabled"] is False

    offered = client.get("/system/chat-channels").get_json()
    assert offered["channels"] == [CHANNEL_DIRECT]
    assert CHANNEL_SMART not in offered["channels"]
    assert offered["smartChatsEnabled"] is False
    assert offered["defaultChannel"] == CHANNEL_DIRECT


def test_ui_channel_path_follows_the_installer_flag(client, monkeypatch):
    """Cerebra reads /api/voice-assistant/channel. Nginx strips /api."""
    offered = client.get("/voice-assistant/channel")
    assert offered.status_code == 200
    assert offered.get_json() == client.get("/system/chat-channels").get_json()
    assert offered.get_json()["smartChatsEnabled"] is True

    monkeypatch.setenv("PIB_SMART_CHATS", "0")
    disabled = client.get("/voice-assistant/channel")
    assert disabled.status_code == 200
    body = disabled.get_json()
    assert body == client.get("/system/chat-channels").get_json()
    assert body["smartChatsEnabled"] is False
    assert body["channels"] == [CHANNEL_DIRECT]
    assert CHANNEL_SMART not in body["channels"]


def test_existing_smart_personality_is_shown_as_direct_and_restores(
    client, app_ctx, monkeypatch, tmp_path
):
    """The flag does not hide, delete, or rewrite a personality that stored Smart."""
    from model.assistant_model import AssistantModel

    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    model = AssistantModel.query.first()
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "ExistingSmart",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": model.id,
            "channel": CHANNEL_SMART,
            "description": "Ein Soul, eine Identitaet.",
        },
    )
    assert created.status_code == 201
    created_body = created.get_json()
    personality_id = created_body["personalityId"]
    identity = created_body["description"]
    soul = Path(profile_dir_for(personality_id)) / "SOUL.md"
    memory = Path(profile_dir_for(personality_id)) / "memories" / "MEMORY.md"
    memory.parent.mkdir(parents=True)
    memory.write_text("remember this\n", encoding="utf-8")
    soul_bytes = soul.read_bytes()
    memory_bytes = memory.read_bytes()

    monkeypatch.setenv("PIB_SMART_CHATS", "0")
    listed = client.get("/voice-assistant/personality").get_json()
    shown = [
        item
        for item in listed["voiceAssistantPersonalities"]
        if item["personalityId"] == personality_id
    ]
    assert len(shown) == 1
    assert shown[0]["channel"] == CHANNEL_SMART
    assert shown[0]["effectiveChannel"] == CHANNEL_DIRECT
    assert shown[0]["description"] == identity
    assert shown[0]["smartChatsEnabled"] is False
    assert soul.read_bytes() == soul_bytes
    assert memory.read_bytes() == memory_bytes

    monkeypatch.setenv("PIB_SMART_CHATS", "1")
    restored = client.get(f"/voice-assistant/personality/{personality_id}").get_json()
    assert restored["channel"] == CHANNEL_SMART
    assert restored["effectiveChannel"] == CHANNEL_SMART
    assert restored["description"] == identity
    assert soul.read_bytes() == soul_bytes
    assert memory.read_bytes() == memory_bytes


def test_disabled_flag_does_not_rewrite_a_stored_smart_channel(
    client, app_ctx, monkeypatch
):
    from model.assistant_model import AssistantModel

    model = AssistantModel.query.first()
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "StoredSmart",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": model.id,
            "channel": CHANNEL_SMART,
        },
    )
    assert created.status_code == 201
    personality_id = created.get_json()["personalityId"]

    monkeypatch.setenv("PIB_SMART_CHATS", "0")
    loaded = client.get(f"/voice-assistant/personality/{personality_id}")
    body = loaded.get_json()
    assert body["channel"] == CHANNEL_SMART
    assert body["effectiveChannel"] == CHANNEL_DIRECT

    rejected = client.put(
        f"/voice-assistant/personality/{personality_id}",
        json={"channel": CHANNEL_SMART},
    )
    assert rejected.status_code == 400
    again = client.get(f"/voice-assistant/personality/{personality_id}").get_json()
    assert again["channel"] == CHANNEL_SMART
    assert again["effectiveChannel"] == CHANNEL_DIRECT


def test_personality_client_reads_channel_apart_from_the_model(monkeypatch):
    from pib_api_client.voice_assistant_client import Personality

    model = MagicMock()
    model.api_name = "gpt-4o"
    monkeypatch.setattr(
        "pib_api_client.voice_assistant_client.get_default_provider",
        lambda: (True, model),
    )
    personality = Personality(
        {
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "description": "soul text",
            "providerRef": "default",
            "channel": "direct",
            "effectiveChannel": "direct",
        }
    )

    assert personality.channel == "direct"
    assert personality.effective_channel == "direct"
    assert personality.description == "soul text"
    assert personality.assistant_model.api_name == "gpt-4o"
