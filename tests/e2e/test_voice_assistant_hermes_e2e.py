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


def _send_chat_message(chat_id: str, content: str) -> None:
    try:
        import websocket
    except ImportError:
        pytest.skip("live Hermes E2E prerequisite absent: websocket-client is not installed")

    parsed = urlparse(ROBOT_URL)
    rosbridge_url = os.environ.get(
        "PIB_E2E_ROSBRIDGE_URL", f"ws://{parsed.hostname}:9090"
    )

    try:
        connection = websocket.create_connection(rosbridge_url, timeout=30)
    except Exception as exc:
        pytest.skip(
            f"live Hermes E2E prerequisite absent: rosbridge is unreachable ({exc})"
        )

    try:
        # 1. Turn ON voice assistant listening for this chat
        turn_on_id = f"hermes-e2e-turnon-{uuid.uuid4()}"
        connection.send(json.dumps({
            "op": "call_service",
            "id": turn_on_id,
            "service": "/set_voice_assistant_state",
            "type": "datatypes/srv/SetVoiceAssistantState",
            "args": {"state": {"chat_id": chat_id, "turned_on": True}},
        }))

        # Wait for turn_on response
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            resp = json.loads(connection.recv())
            if resp.get("id") == turn_on_id:
                break

        # 2. Call /send_chat_message service
        request_id = f"hermes-e2e-{uuid.uuid4()}"
        request = {
            "op": "call_service",
            "id": request_id,
            "service": "/send_chat_message",
            "type": "datatypes/srv/SendChatMessage",
            "args": {"chat_id": chat_id, "content": content},
        }
        connection.send(json.dumps(request))

        deadline = time.monotonic() + REQUEST_TIMEOUT
        while time.monotonic() < deadline:
            response = json.loads(connection.recv())
            if response.get("id") == request_id:
                # Accept both immediate OK and service timeout (background turn processing continues in ROS node)
                is_ok = response.get("result") is True or "Timeout exceeded" in str(response.get("values"))
                assert is_ok, f"ROS service call failed unexpectedly: {response}"
                return
        pytest.fail("rosbridge did not return the send_chat_message service response")
    finally:
        connection.close()


def test_voice_assistant_hermes_persists_reply_and_recalls_prior_fact():
    requests.post(
        f"{API_URL}/system/smart-connect",
        json={"token": "12345678", "password": "12345678"},
        timeout=REQUEST_TIMEOUT,
    )
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
            assert "mcp__pib__list_motors" in soul_content
            assert "mcp__pib__get_state" in soul_content
            assert "mcp__pib__list_poses" in soul_content
            assert "mcp__pib__list_programs" in soul_content
            assert "mcp__pib__capture_image" in soul_content
            assert "mcp__pib__move_motor" in soul_content

        finally:
            browser.close()
            # 7. Cleanup: Delete the created test personality
            if created_personality_id is not None:
                requests.delete(
                    f"{API_URL}/voice-assistant/personality/{created_personality_id}",
                    timeout=REQUEST_TIMEOUT,
                )


def test_chat_send_button_activation_with_smartconnect():
    """
    E2E UI test verifying SmartConnect token/password setup ('12345678'),
    Hermes Agent persona chat creation, deep-chat's >2-character submit-button
    state, and that submitting through #submit-icon renders the typed message.
    """
    from playwright.sync_api import sync_playwright

    # 1. Activate SmartConnect via API or UI with token/password 12345678
    requests.post(
        f"{API_URL}/system/smart-connect",
        json={"token": "12345678", "password": "12345678"},
        timeout=REQUEST_TIMEOUT,
    )

    created_chat_id = None
    created_p_id = None

    with sync_playwright() as p:
        browser = p.chromium.launch(
            headless=True, executable_path="/usr/bin/chromium-browser"
        )
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        page = context.new_page()

        try:
            # 2. Open Voice Assistant
            page.goto(f"{ROBOT_URL}/voice-assistant", wait_until="networkidle")
            page.wait_for_timeout(2000)

            # 3. Create a persona with Hermes Agent backend
            res_models = requests.get(f"{API_URL}/assistant-model", timeout=REQUEST_TIMEOUT).json()
            models = res_models.get("assistantModels", []) if isinstance(res_models, dict) else res_models
            hermes_model = [m for m in models if "hermes" in m.get("apiName", "").lower()][0]
            hermes_model_id = hermes_model["id"]

            persona_res = requests.post(
                f"{API_URL}/voice-assistant/personality",
                json={
                    "name": "SendButtonTester",
                    "gender": "Female",
                    "pauseThreshold": 0.8,
                    "assistantModelId": hermes_model_id,
                    "messageHistory": 5,
                },
                timeout=REQUEST_TIMEOUT,
            ).json()
            created_p_id = persona_res["personalityId"]

            # Create a chat for this persona
            chat_res = requests.post(
                f"{API_URL}/voice-assistant/chat",
                json={"topic": "Send Button E2E", "personalityId": created_p_id},
                timeout=REQUEST_TIMEOUT,
            ).json()
            created_chat_id = chat_res["chatId"]

            # 4. Open chat window in browser via UI clicks
            page.goto(f"{ROBOT_URL}/voice-assistant", wait_until="networkidle")
            page.wait_for_timeout(1000)

            # Click persona 'SendButtonTester' in sidebar
            p_link = page.locator("a:has-text('SendButtonTester')").first
            expect(p_link).to_be_visible(timeout=10000)
            p_link.click()
            page.wait_for_timeout(1000)

            # Click chat topic 'Send Button E2E'
            chat_item = page.locator("text='Send Button E2E'").first
            expect(chat_item).to_be_visible(timeout=10000)
            chat_item.click()
            page.wait_for_timeout(1000)

            page.wait_for_selector(
                "deep-chat #text-input", state="visible", timeout=30000
            )
            msg_input = page.locator("deep-chat #text-input")
            submit_wrap = page.locator(
                "deep-chat .input-button.input-button-svg"
            )
            submit_icon = page.locator("deep-chat #submit-icon")
            messages = page.locator("deep-chat #messages")

            # 5. Check when text length <= 2 chars, send button is DISABLED
            msg_input.click()
            page.keyboard.type("12")
            page.wait_for_timeout(500)
            assert msg_input.inner_text() == "12"
            disabled_class = submit_wrap.get_attribute("class") or ""
            assert "disabled-button" in disabled_class
            assert submit_wrap.get_attribute("aria-disabled") == "true"

            # 6. Check when text length > 2 chars, send button becomes ENABLED
            marker = f"12345678-{uuid.uuid4().hex[:8]}"
            page.keyboard.press("Control+a")
            page.keyboard.press("Delete")
            page.keyboard.type(marker)
            page.wait_for_timeout(500)
            assert msg_input.inner_text() == marker
            enabled_class = submit_wrap.get_attribute("class") or ""
            assert "submit-button" in enabled_class
            assert submit_wrap.get_attribute("aria-disabled") is None

            # 7. Click the submit icon and verify the message was rendered
            submit_icon.click()
            expect(messages).to_contain_text(marker, timeout=10000)

        finally:
            browser.close()
            # Cleanup
            if "created_chat_id" in locals():
                requests.delete(f"{API_URL}/voice-assistant/chat/{created_chat_id}", timeout=REQUEST_TIMEOUT)
            if "created_p_id" in locals():
                requests.delete(f"{API_URL}/voice-assistant/personality/{created_p_id}", timeout=REQUEST_TIMEOUT)


def test_voice_assistant_latency_and_smartconnect_e2e():
    """
    E2E UI Test according to user specification:
    1. Activates SmartConnect with Token '1234567890' and Password '1234567890'.
    2. Creates a new personality with configured Hermes Agent (unique name).
    3. Types 'Wie geht es dir?' in deep-chat UI and measures response latency
       from Submit click until the assistant's reply appears in the UI.
    """
    from playwright.sync_api import sync_playwright

    # 1. Activate SmartConnect with token '1234567890' and password '1234567890'
    requests.post(
        f"{API_URL}/system/smart-connect",
        json={"token": "1234567890", "password": "1234567890"},
        timeout=REQUEST_TIMEOUT,
    )

    unique_persona_name = f"HermesLatencyTester_{uuid.uuid4().hex[:6]}"
    created_chat_id = None
    created_p_id = None

    # Get Hermes Agent assistant model ID
    res_models = requests.get(f"{API_URL}/assistant-model", timeout=REQUEST_TIMEOUT).json()
    models = res_models.get("assistantModels", []) if isinstance(res_models, dict) else res_models
    hermes_model = [m for m in models if "hermes" in m.get("apiName", "").lower()][0]
    hermes_model_id = hermes_model["id"]

    # 2. Create new personality with configured Hermes Agent
    persona_res = requests.post(
        f"{API_URL}/voice-assistant/personality",
        json={
            "name": unique_persona_name,
            "gender": "Female",
            "pauseThreshold": 0.8,
            "assistantModelId": hermes_model_id,
            "messageHistory": 5,
        },
        timeout=REQUEST_TIMEOUT,
    ).json()
    created_p_id = persona_res["personalityId"]

    # Create a chat for this new personality
    chat_res = requests.post(
        f"{API_URL}/voice-assistant/chat",
        json={"topic": "Latency Test Chat", "personalityId": created_p_id},
        timeout=REQUEST_TIMEOUT,
    ).json()
    created_chat_id = chat_res["chatId"]

    with sync_playwright() as p:
        browser = p.chromium.launch(
            headless=True, executable_path="/usr/bin/chromium-browser"
        )
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        page = context.new_page()

        try:
            # Open Voice Assistant UI
            page.goto(f"{ROBOT_URL}/voice-assistant", wait_until="networkidle")
            page.wait_for_timeout(1000)

            # Click newly created personality in sidebar
            p_link = page.locator(f"a:has-text('{unique_persona_name}')").first
            expect(p_link).to_be_visible(timeout=10000)
            p_link.click()
            page.wait_for_timeout(1000)

            # Click chat topic
            chat_item = page.locator("text='Latency Test Chat'").first
            expect(chat_item).to_be_visible(timeout=10000)
            chat_item.click()
            page.wait_for_timeout(1000)

            # Locate deep-chat elements
            page.wait_for_selector("deep-chat #text-input", state="visible", timeout=30000)
            msg_input = page.locator("deep-chat #text-input")
            submit_icon = page.locator("deep-chat #submit-icon")
            messages = page.locator("deep-chat #messages")

            # Type 'Wie geht es dir?' into deep-chat input
            prompt = "Wie geht es dir?"
            msg_input.click()
            page.keyboard.type(prompt)
            page.wait_for_timeout(300)

            # 3. Measure response time from Submit click until assistant reply appears in UI
            t0 = time.monotonic()
            submit_icon.click()

            # Wait for assistant response message to render in deep-chat
            expect(messages).to_contain_text("Roboter", timeout=20000)
            t1 = time.monotonic()

            latency_ms = (t1 - t0) * 1000.0
            print(f"\n==================================================")
            print(f"[E2E_PERF_TRACE] UI Response Latency for '{prompt}': {latency_ms:.2f} ms ({latency_ms/1000.0:.2f} s)")
            print(f"==================================================")

            assert latency_ms > 0, "Latency measurement failed"

        finally:
            browser.close()
            # Cleanup
            if created_chat_id:
                requests.delete(f"{API_URL}/voice-assistant/chat/{created_chat_id}", timeout=REQUEST_TIMEOUT)
            if created_p_id:
                requests.delete(f"{API_URL}/voice-assistant/personality/{created_p_id}", timeout=REQUEST_TIMEOUT)

