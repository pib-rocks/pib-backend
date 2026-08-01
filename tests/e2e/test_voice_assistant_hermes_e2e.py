"""Live-robot E2E for a Hermes-backed voice-assistant personality."""

from __future__ import annotations

import json
import os
import time
import uuid
from urllib.parse import urlparse

import pytest
import requests


ROBOT_URL = os.environ.get("PIB_E2E_BASE_URL", "http://192.168.1.28").rstrip("/")
API_URL = f"{ROBOT_URL}/api"
REQUEST_TIMEOUT = 5
TURN_TIMEOUT = int(os.environ.get("PIB_HERMES_E2E_TURN_TIMEOUT", "180"))


def _get_json(path: str):
    response = requests.get(f"{API_URL}{path}", timeout=REQUEST_TIMEOUT)
    response.raise_for_status()
    return response.json()


def _wait_for_new_assistant_message(chat_id: str, previous_ids: set[str]):
    deadline = time.monotonic() + TURN_TIMEOUT
    last_count = 0
    stable_ticks = 0
    while time.monotonic() < deadline:
        messages = _get_json(
            f"/voice-assistant/chat/{chat_id}/messages"
        ).get("messages", [])
        new_replies = [
            message
            for message in messages
            if not message["isUser"]
            and message["messageId"] not in previous_ids
            and message["content"].strip()
        ]
        if new_replies:
            if len(new_replies) == last_count:
                stable_ticks += 1
                combined_content = " ".join(m["content"] for m in new_replies).strip()
                if (stable_ticks >= 3 and not combined_content.endswith(":")) or stable_ticks >= 6:
                    messages = _get_json(f"/voice-assistant/chat/{chat_id}/messages").get("messages", [])
                    new_replies = [m for m in messages if not m["isUser"] and m["messageId"] not in previous_ids and m["content"].strip()]
                    combined_content = " ".join(m["content"] for m in new_replies).strip()
                    last_reply = new_replies[-1].copy()
                    last_reply["content"] = combined_content
                    return last_reply, messages
            else:
                last_count = len(new_replies)
                stable_ticks = 0
        time.sleep(1)
    pytest.fail(f"No persisted assistant reply arrived within {TURN_TIMEOUT} seconds")


def _send_chat_message(chat_id: str, content: str):
    try:
        import websocket
    except ImportError:
        pytest.skip("live Hermes E2E prerequisite absent: websocket-client is not installed")

    parsed = urlparse(ROBOT_URL)
    rosbridge_url = os.environ.get(
        "PIB_E2E_ROSBRIDGE_URL", f"ws://{parsed.hostname}:9090"
    )
    request_id = f"hermes-e2e-{uuid.uuid4()}"
    request = {
        "op": "call_service",
        "id": request_id,
        "service": "/send_chat_message",
        "type": "datatypes/srv/SendChatMessage",
        "args": {"chat_id": chat_id, "content": content},
    }

    try:
        connection = websocket.create_connection(rosbridge_url, timeout=REQUEST_TIMEOUT)
    except Exception as exc:
        pytest.skip(
            f"live Hermes E2E prerequisite absent: rosbridge is unreachable ({exc})"
        )

    try:
        connection.send(json.dumps(request))
        deadline = time.monotonic() + REQUEST_TIMEOUT
        while time.monotonic() < deadline:
            response = json.loads(connection.recv())
            if response.get("id") == request_id:
                assert response.get("result") is True
                assert response.get("values", {}).get("successful") is True
                return
        pytest.fail("rosbridge did not return the send_chat_message service response")
    finally:
        connection.close()


def test_voice_assistant_hermes_persists_reply_and_recalls_prior_fact():
    try:
        models = _get_json("/assistant-model").get("assistantModels", [])
    except (requests.RequestException, ValueError) as exc:
        pytest.skip(f"live Hermes E2E prerequisite absent: robot API is unreachable ({exc})")

    hermes_model = next(
        (model for model in models if model.get("apiName") == "hermes-agent"), None
    )
    if hermes_model is None:
        pytest.skip(
            "live Hermes E2E prerequisite absent: no hermes-agent assistant model exists"
        )

    personalities = _get_json(
        "/voice-assistant/personality"
    ).get("voiceAssistantPersonalities", [])
    requested_id = os.environ.get("PIB_HERMES_E2E_PERSONALITY_ID")
    personality = next(
        (
            item
            for item in personalities
            if requested_id is None or item["personalityId"] == requested_id
        ),
        None,
    )
    if personality is None:
        pytest.skip(
            "live Hermes E2E prerequisite absent: no usable voice-assistant personality exists"
        )

    personality_id = personality["personalityId"]
    original_model_id = personality["assistantModelId"]
    chat_id = None
    token = f"PIB-E2E-{uuid.uuid4().hex[:8].upper()}"

    try:
        update = requests.put(
            f"{API_URL}/voice-assistant/personality/{personality_id}",
            json={"assistantModelId": hermes_model["id"]},
            timeout=REQUEST_TIMEOUT,
        )
        update.raise_for_status()

        created = requests.post(
            f"{API_URL}/voice-assistant/chat",
            json={
                "topic": f"Hermes E2E {token}",
                "personalityId": personality_id,
            },
            timeout=REQUEST_TIMEOUT,
        )
        created.raise_for_status()
        chat_id = created.json()["chatId"]

        _send_chat_message(
            chat_id,
            f"My favourite fruit is {token}. Acknowledge briefly.",
        )
        first_reply, first_messages = _wait_for_new_assistant_message(chat_id, set())
        assert first_reply["content"].strip()

        first_ids = {message["messageId"] for message in first_messages}
        _send_chat_message(
            chat_id,
            "What is my favourite fruit?",
        )
        recalled_reply, _ = _wait_for_new_assistant_message(chat_id, first_ids)
        assert token in recalled_reply["content"]
    finally:
        if chat_id is not None:
            requests.delete(
                f"{API_URL}/voice-assistant/chat/{chat_id}", timeout=REQUEST_TIMEOUT
            )
        requests.put(
            f"{API_URL}/voice-assistant/personality/{personality_id}",
            json={"assistantModelId": original_model_id},
            timeout=REQUEST_TIMEOUT,
        )
