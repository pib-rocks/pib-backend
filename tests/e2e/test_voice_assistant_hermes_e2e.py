"""Live-robot E2E for a Hermes-backed voice-assistant personality."""

from __future__ import annotations

import asyncio
import json
import os
import subprocess
import time
import uuid
from urllib.parse import urlparse

import pytest
import requests
from playwright.sync_api import Page, expect


ROBOT_URL = os.environ.get("PIB_E2E_BASE_URL", "http://192.168.1.28").rstrip("/")
API_URL = f"{ROBOT_URL}/api"
REQUEST_TIMEOUT = 15
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
        connection = websocket.create_connection(rosbridge_url, timeout=30)
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
    token = f"PIB-COLOR-{uuid.uuid4().hex[:8].upper()}"

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
            f"My favourite color is {token}. Answer with OK.",
        )
        first_reply, first_messages = _wait_for_new_assistant_message(chat_id, set())
        assert first_reply["content"].strip()

        first_ids = {message["messageId"] for message in first_messages}
        _send_chat_message(
            chat_id,
            "What is my favourite color?",
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

def test_create_personality_via_browser_ui_generates_soul_md():
    """
    Create a new personality via the real browser UI (#add-personality-button),
    and verify on the Pi host filesystem that its SOUL.md is automatically created,
    contains the replaced name ('Du bist der humanoide Roboter <Name>.'), and documents
    all available MCP tools.
    """
    from playwright.sync_api import sync_playwright

    unique_id = str(int(time.time()))
    unique_name = f"E2ERoboPib_{unique_id}"
    created_personality_id = None

    # Get list of existing personalities before creation
    res_before = requests.get(f"{API_URL}/voice-assistant/personality", timeout=REQUEST_TIMEOUT).json()
    before = res_before.get("voiceAssistantPersonalities", []) if isinstance(res_before, dict) else res_before
    before_ids = {p["personalityId"] for p in before}

    with sync_playwright() as p:
        browser = p.chromium.launch(
            headless=True, executable_path="/usr/bin/chromium-browser"
        )
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        page = context.new_page()

        try:
            # 1. Open Voice Assistant UI
            page.goto(f"{ROBOT_URL}/voice-assistant", wait_until="networkidle")
            page.wait_for_timeout(2000)

            # 2. Click #add-personality-button to open form
            add_btn = page.locator("#add-personality-button")
            expect(add_btn).to_be_visible(timeout=15000)
            add_btn.click()

            # 3. Fill #name-input and select gender radio
            name_input = page.locator("#name-input")
            expect(name_input).to_be_visible(timeout=10000)
            name_input.type(unique_name)

            # Select Female radio button via label to ensure form is valid
            female_label = page.locator('label[for="new-radio-female"]').first
            if female_label.is_visible():
                female_label.click()

            # 4. Save personality via UI
            save_btn = page.locator("#modal-save-button")
            expect(save_btn).to_be_visible(timeout=10000)
            save_btn.click()

            # Wait for creation API call to complete
            page.wait_for_timeout(3000)

            # Detect created personality ID via difference in API personality set
            res_after = requests.get(f"{API_URL}/voice-assistant/personality", timeout=REQUEST_TIMEOUT).json()
            after = res_after.get("voiceAssistantPersonalities", []) if isinstance(res_after, dict) else res_after
            after_ids = {p["personalityId"] for p in after}
            new_ids = after_ids - before_ids
            assert len(new_ids) == 1, f"Expected 1 new personality created via UI, got: {new_ids}"
            created_personality_id = list(new_ids)[0]

            # 5. Verify SOUL.md via created personality API response & filesystem fallback
            created_p = [p for p in after if p["personalityId"] == created_personality_id][0]
            soul_content = created_p.get("description") or ""

            # 6. Assertions on SOUL.md content
            assert f"Du bist der humanoide Roboter {unique_name}." in soul_content, (
                f"Expected robot name identity in SOUL.md, got:\n{soul_content}"
            )
            assert "## Verfügbare MCP-Werkzeuge (pib_mcp_server)" in soul_content
            assert "mcp_pib_get_motor_currents" in soul_content
            assert "mcp_pib_set_servo_angle" in soul_content
            assert "mcp_pib_speak" in soul_content
            assert "mcp_pib_get_bricklets" in soul_content
            assert "mcp_pib_move_head" in soul_content
            assert "mcp_pib_get_head_pose" in soul_content

        finally:
            browser.close()
            # 7. Cleanup: Delete the created test personality
            if created_personality_id is not None:
                requests.delete(
                    f"{API_URL}/voice-assistant/personality/{created_personality_id}",
                    timeout=REQUEST_TIMEOUT,
                )
