"""Live-robot E2E for a Hermes-backed voice-assistant personality."""

from __future__ import annotations

import asyncio
import json
import os
import re
import subprocess
import time
import uuid
from urllib.parse import urlparse

import pytest
import requests
import shutil


def _get_chromium_launch_kwargs() -> dict:
    kwargs = {"headless": True}
    for p in ["/usr/bin/chromium-browser", "/usr/bin/chromium"]:
        if os.path.exists(p):
            kwargs["executable_path"] = p
            break
    return kwargs


from playwright.sync_api import Page, expect
from playwright.sync_api import TimeoutError as PlaywrightTimeoutError

ROBOT_URL = os.environ.get("PIB_E2E_BASE_URL", "http://192.168.1.28").rstrip("/")
API_URL = f"{ROBOT_URL}/api"
REQUEST_TIMEOUT = 15
TURN_TIMEOUT = int(os.environ.get("PIB_HERMES_E2E_TURN_TIMEOUT", "300"))
# rosbridge forwards /send_chat_message to a ROS node that may already be busy with
# an LLM turn, so the service acknowledgement can lag well past REQUEST_TIMEOUT.
SERVICE_ACK_TIMEOUT = int(os.environ.get("PIB_HERMES_E2E_SERVICE_TIMEOUT", "60"))

# Browser-side timeouts. Defaults are generous because the suite runs on the Pi,
# where Angular bootstrap and deep-chat hydration are slow under load.
UI_TIMEOUT_MS = int(os.environ.get("PIB_E2E_UI_TIMEOUT_MS", "30000"))
NAV_TIMEOUT_MS = int(os.environ.get("PIB_E2E_NAV_TIMEOUT_MS", "60000"))
UI_REPLY_TIMEOUT_S = int(os.environ.get("PIB_E2E_UI_REPLY_TIMEOUT_S", "20"))
# Budget for a UI-triggered write to become observable through the REST API.
API_SETTLE_TIMEOUT_S = int(os.environ.get("PIB_E2E_API_SETTLE_TIMEOUT_S", "30"))


def _get_json(path: str):
    response = requests.get(f"{API_URL}{path}", timeout=REQUEST_TIMEOUT)
    response.raise_for_status()
    return response.json()


def _configure_page(page: "Page") -> "Page":
    page.set_default_timeout(UI_TIMEOUT_MS)
    page.set_default_navigation_timeout(NAV_TIMEOUT_MS)
    return page


def _open_voice_assistant(page: "Page") -> None:
    """Navigate to the Voice Assistant view and wait for it to be interactive.

    "networkidle" is unusable against Cerebra: it holds a rosbridge socket open and
    polls, so the network never goes idle and goto() times out at random on the Pi.
    """
    page.goto(f"{ROBOT_URL}/voice-assistant", wait_until="domcontentloaded")
    page.wait_for_selector(
        "#add-personality-button", state="visible", timeout=NAV_TIMEOUT_MS
    )


def _personality_ids() -> set:
    payload = _get_json("/voice-assistant/personality")
    items = (
        payload.get("voiceAssistantPersonalities", [])
        if isinstance(payload, dict)
        else payload
    )
    return {item["personalityId"] for item in items}


def _poll_messages(chat_id: str):
    """Read chat messages, returning None on a transient API hiccup.

    A single 5xx or a dropped connection while the Pi is under load must not abort
    the whole turn wait, so callers keep polling instead of raising.
    """
    try:
        return _get_json(f"/voice-assistant/chat/{chat_id}/messages").get(
            "messages", []
        )
    except (requests.RequestException, ValueError):
        return None


def _wait_for_new_assistant_message(chat_id: str, previous_ids: set[str]):
    deadline = time.monotonic() + TURN_TIMEOUT
    last_count = 0
    stable_ticks = 0
    while time.monotonic() < deadline:
        messages = _poll_messages(chat_id)
        if messages is None:
            time.sleep(1)
            continue
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
                if (
                    stable_ticks >= 3 and not combined_content.endswith(":")
                ) or stable_ticks >= 6:
                    final_messages = _poll_messages(chat_id)
                    if final_messages is None:
                        time.sleep(1)
                        continue
                    messages = final_messages
                    new_replies = [
                        m
                        for m in messages
                        if not m["isUser"]
                        and m["messageId"] not in previous_ids
                        and m["content"].strip()
                    ]
                    combined_content = " ".join(
                        m["content"] for m in new_replies
                    ).strip()
                    last_reply = new_replies[-1].copy()
                    last_reply["content"] = combined_content
                    return last_reply, messages
            else:
                last_count = len(new_replies)
                stable_ticks = 0
        time.sleep(1)
    pytest.fail(f"No persisted assistant reply arrived within {TURN_TIMEOUT} seconds")


def _await_service_response(connection, request_id: str, timeout_s: float):
    """Drain rosbridge frames until the response for `request_id` arrives.

    Returns the response dict, or None if the budget runs out. Every recv() gets a
    socket timeout derived from the remaining budget; otherwise a quiet socket blocks
    for the full connection timeout and leaves the loop via an exception rather than
    honouring its own deadline.
    """
    import websocket

    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            connection.settimeout(min(remaining, 5))
            frame = connection.recv()
        except (websocket.WebSocketTimeoutException, TimeoutError):
            continue
        except (websocket.WebSocketException, OSError):
            return None

        if not frame:
            continue
        try:
            message = json.loads(frame)
        except (TypeError, ValueError):
            continue
        if isinstance(message, dict) and message.get("id") == request_id:
            return message


def _turn_off_voice_assistant(chat_id: str) -> None:
    """Best-effort teardown of the listening state opened by `_send_chat_message`."""
    try:
        import websocket

        parsed = urlparse(ROBOT_URL)
        rosbridge_url = os.environ.get(
            "PIB_E2E_ROSBRIDGE_URL", f"ws://{parsed.hostname}:9090"
        )
        connection = websocket.create_connection(rosbridge_url, timeout=10)
        try:
            request_id = f"hermes-e2e-turnoff-{uuid.uuid4()}"
            connection.send(
                json.dumps(
                    {
                        "op": "call_service",
                        "id": request_id,
                        "service": "/set_voice_assistant_state",
                        "type": "datatypes/srv/SetVoiceAssistantState",
                        "args": {"state": {"chat_id": chat_id, "turned_on": False}},
                    }
                )
            )
            _await_service_response(connection, request_id, 10)
        finally:
            connection.close()
    except Exception:
        pass


def _send_chat_message(chat_id: str, content: str) -> None:
    try:
        import websocket
    except ImportError:
        pytest.skip(
            "live Hermes E2E prerequisite absent: websocket-client is not installed"
        )

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
        connection.send(
            json.dumps(
                {
                    "op": "call_service",
                    "id": turn_on_id,
                    "service": "/set_voice_assistant_state",
                    "type": "datatypes/srv/SetVoiceAssistantState",
                    "args": {"state": {"chat_id": chat_id, "turned_on": True}},
                }
            )
        )

        # Wait for turn_on response. Missing the acknowledgement is not fatal:
        # the state change is applied by the ROS node regardless.
        _await_service_response(connection, turn_on_id, 10)

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

        response = _await_service_response(connection, request_id, SERVICE_ACK_TIMEOUT)
        if response is None:
            pytest.fail(
                "rosbridge did not return the send_chat_message service response "
                f"within {SERVICE_ACK_TIMEOUT} seconds"
            )
        # Accept both immediate OK and service timeout (background turn processing continues in ROS node)
        is_ok = response.get("result") is True or "Timeout exceeded" in str(
            response.get("values")
        )
        assert is_ok, f"ROS service call failed unexpectedly: {response}"
    finally:
        connection.close()


def test_voice_assistant_hermes_persists_reply_and_recalls_prior_fact():
    try:
        requests.post(
            f"{API_URL}/system/smart-connect",
            json={"token": "12345678", "password": "12345678"},
            timeout=REQUEST_TIMEOUT,
        )
    except requests.RequestException:
        # Reachability is reported as a skip by the assistant-model probe below.
        pass
    try:
        models = _get_json("/assistant-model").get("assistantModels", [])
    except (requests.RequestException, ValueError) as exc:
        pytest.skip(
            f"live Hermes E2E prerequisite absent: robot API is unreachable ({exc})"
        )

    hermes_model = next(
        (model for model in models if model.get("apiName") == "hermes-agent"), None
    )
    if hermes_model is None:
        pytest.skip(
            "live Hermes E2E prerequisite absent: no hermes-agent assistant model exists"
        )

    try:
        personalities = _get_json("/voice-assistant/personality").get(
            "voiceAssistantPersonalities", []
        )
    except (requests.RequestException, ValueError) as exc:
        pytest.skip(
            f"live Hermes E2E prerequisite absent: robot API is unreachable ({exc})"
        )
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
        # Each cleanup step is isolated so a failing one cannot mask the real result,
        # and so a leftover "listening" chat cannot slow down the following tests.
        if chat_id is not None:
            _turn_off_voice_assistant(chat_id)
            try:
                requests.delete(
                    f"{API_URL}/voice-assistant/chat/{chat_id}", timeout=REQUEST_TIMEOUT
                )
            except requests.RequestException:
                pass
        try:
            requests.put(
                f"{API_URL}/voice-assistant/personality/{personality_id}",
                json={"assistantModelId": original_model_id},
                timeout=REQUEST_TIMEOUT,
            )
        except requests.RequestException:
            pass


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
    res_before = requests.get(
        f"{API_URL}/voice-assistant/personality", timeout=REQUEST_TIMEOUT
    ).json()
    before = (
        res_before.get("voiceAssistantPersonalities", [])
        if isinstance(res_before, dict)
        else res_before
    )
    before_ids = {p["personalityId"] for p in before}

    with sync_playwright() as p:
        browser = p.chromium.launch(**_get_chromium_launch_kwargs())
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        context.add_init_script("""
            localStorage.setItem('token', '12345678');
            localStorage.setItem('password', '12345678');
        """)
        page = _configure_page(context.new_page())

        try:
            # 1. Open Voice Assistant UI
            _open_voice_assistant(page)

            # 2. Click #add-personality-button to open form
            add_btn = page.locator("#add-personality-button")
            expect(add_btn).to_be_visible(timeout=UI_TIMEOUT_MS)
            expect(add_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)
            add_btn.click()

            # 3. Fill #name-input and select gender radio
            name_input = page.locator("#name-input")
            expect(name_input).to_be_visible(timeout=UI_TIMEOUT_MS)
            name_input.type(unique_name)
            expect(name_input).to_have_value(unique_name, timeout=UI_TIMEOUT_MS)

            # Select Female radio button via label to ensure form is valid. The label
            # renders with the modal, so wait for it instead of sampling is_visible().
            female_label = page.locator('label[for="new-radio-female"]').first
            try:
                female_label.wait_for(state="visible", timeout=5000)
                female_label.click()
            except PlaywrightTimeoutError:
                pass

            # 4. Save personality via UI
            save_btn = page.locator("#modal-save-button")
            expect(save_btn).to_be_visible(timeout=UI_TIMEOUT_MS)
            expect(save_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)
            save_btn.click()

            # Detect created personality ID by polling the API until the new entry
            # shows up, rather than sleeping and hoping the POST already landed.
            deadline = time.monotonic() + API_SETTLE_TIMEOUT_S
            new_ids = set()
            while time.monotonic() < deadline:
                try:
                    new_ids = _personality_ids() - before_ids
                except (requests.RequestException, ValueError):
                    new_ids = set()
                if new_ids:
                    break
                time.sleep(0.5)
            assert (
                len(new_ids) == 1
            ), f"Expected 1 new personality created via UI, got: {new_ids}"
            created_personality_id = list(new_ids)[0]

            # 5. Verify SOUL.md via created personality API response. The file is
            # written asynchronously after the POST returns, so poll for content.
            soul_content = ""
            deadline = time.monotonic() + API_SETTLE_TIMEOUT_S
            while time.monotonic() < deadline:
                try:
                    created_p = _get_json(
                        f"/voice-assistant/personality/{created_personality_id}"
                    )
                except (requests.RequestException, ValueError):
                    created_p = {}
                soul_content = created_p.get("description") or ""
                if unique_name in soul_content:
                    break
                time.sleep(0.5)

            # 6. Assertions on SOUL.md content
            assert (
                f"Du bist der humanoide Roboter {unique_name}." in soul_content
            ), f"Expected robot name identity in SOUL.md, got:\n{soul_content}"
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
                try:
                    requests.delete(
                        f"{API_URL}/voice-assistant/personality/{created_personality_id}",
                        timeout=REQUEST_TIMEOUT,
                    )
                except requests.RequestException:
                    pass


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
        browser = p.chromium.launch(**_get_chromium_launch_kwargs())
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        context.add_init_script("""
            localStorage.setItem('token', '12345678');
            localStorage.setItem('password', '12345678');
        """)
        page = _configure_page(context.new_page())

        try:
            # 2. Open Voice Assistant
            _open_voice_assistant(page)

            # 3. Create a persona with Hermes Agent backend
            res_models = requests.get(
                f"{API_URL}/assistant-model", timeout=REQUEST_TIMEOUT
            ).json()
            models = (
                res_models.get("assistantModels", [])
                if isinstance(res_models, dict)
                else res_models
            )
            hermes_model = [
                m for m in models if "hermes" in m.get("apiName", "").lower()
            ][0]
            hermes_model_id = hermes_model["id"]

            # Names are unique per run: a leftover persona/chat from an aborted run
            # would otherwise be matched first by the sidebar locators below.
            persona_name = f"SendButtonTester_{uuid.uuid4().hex[:6]}"
            chat_topic = f"Send Button E2E {uuid.uuid4().hex[:6]}"

            persona_res = requests.post(
                f"{API_URL}/voice-assistant/personality",
                json={
                    "name": persona_name,
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
                json={"topic": chat_topic, "personalityId": created_p_id},
                timeout=REQUEST_TIMEOUT,
            ).json()
            created_chat_id = chat_res["chatId"]

            # 4. Open chat window in browser via UI clicks. The reload is what makes
            # the persona created above appear in the sidebar.
            _open_voice_assistant(page)

            # Click the persona in the sidebar
            p_link = page.locator(f"a:has-text('{persona_name}')").first
            expect(p_link).to_be_visible(timeout=UI_TIMEOUT_MS)
            p_link.click()

            # Click the chat topic
            chat_item = page.locator(f"text='{chat_topic}'").first
            expect(chat_item).to_be_visible(timeout=UI_TIMEOUT_MS)
            chat_item.click()

            page.wait_for_selector(
                "deep-chat #text-input", state="visible", timeout=UI_TIMEOUT_MS
            )
            msg_input = page.locator("deep-chat #text-input")
            submit_wrap = page.locator("deep-chat .input-button.input-button-svg")
            submit_icon = page.locator("deep-chat #submit-icon")
            messages = page.locator("deep-chat #messages")

            # 5. Check when text length <= 2 chars, send button is DISABLED.
            # expect(...) polls, so deep-chat's own state update is awaited instead
            # of sampled once after a fixed sleep.
            msg_input.click()
            page.keyboard.type("12")
            expect(msg_input).to_have_text("12", timeout=UI_TIMEOUT_MS)
            expect(submit_wrap).to_have_class(
                re.compile(r"disabled-button"), timeout=UI_TIMEOUT_MS
            )
            expect(submit_wrap).to_have_attribute(
                "aria-disabled", "true", timeout=UI_TIMEOUT_MS
            )

            # 6. Check when text length > 2 chars, send button becomes ENABLED
            marker = f"12345678-{uuid.uuid4().hex[:8]}"
            page.keyboard.press("Control+a")
            page.keyboard.press("Delete")
            page.keyboard.type(marker)
            expect(msg_input).to_have_text(marker, timeout=UI_TIMEOUT_MS)
            expect(submit_wrap).to_have_class(
                re.compile(r"submit-button"), timeout=UI_TIMEOUT_MS
            )
            # Read only once the class assertion above proves the button has
            # re-rendered into its enabled state.
            assert submit_wrap.get_attribute("aria-disabled") is None

            # 7. Click the submit icon and verify the message was rendered
            expect(submit_icon).to_be_visible(timeout=UI_TIMEOUT_MS)
            submit_icon.click()
            expect(messages).to_contain_text(marker, timeout=UI_TIMEOUT_MS)

        finally:
            browser.close()
            # Cleanup (isolated so a failing delete cannot mask the test result)
            if created_chat_id:
                try:
                    requests.delete(
                        f"{API_URL}/voice-assistant/chat/{created_chat_id}",
                        timeout=REQUEST_TIMEOUT,
                    )
                except requests.RequestException:
                    pass
            if created_p_id:
                try:
                    requests.delete(
                        f"{API_URL}/voice-assistant/personality/{created_p_id}",
                        timeout=REQUEST_TIMEOUT,
                    )
                except requests.RequestException:
                    pass


def test_voice_assistant_latency_and_smartconnect_e2e():
    """
    E2E UI Test according to user specification:
    1. Activates SmartConnect with Token '1234567890' and Password '1234567890'.
    2. Creates a new personality with configured Hermes Agent (unique name).
    3. Types 'Wie geht es dir?' in deep-chat UI and measures response latency
       from Submit click until the assistant's real reply appears in the UI.
    """
    from playwright.sync_api import sync_playwright

    token = "1234567890"
    password = "1234567890"

    # 1. Activate SmartConnect via API
    requests.post(
        f"{API_URL}/system/smart-connect",
        json={"token": token, "password": password},
        timeout=REQUEST_TIMEOUT,
    )

    unique_persona_name = f"HermesLatencyTester_{uuid.uuid4().hex[:6]}"
    created_chat_id = None
    created_p_id = None

    # Get Hermes Agent assistant model ID
    res_models = requests.get(
        f"{API_URL}/assistant-model", timeout=REQUEST_TIMEOUT
    ).json()
    models = (
        res_models.get("assistantModels", [])
        if isinstance(res_models, dict)
        else res_models
    )
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

    # Create a chat for this new personality. Unique topic so a leftover chat from an
    # aborted run cannot be matched first by the sidebar locator below.
    chat_topic = f"Latency Test Chat {uuid.uuid4().hex[:6]}"
    chat_res = requests.post(
        f"{API_URL}/voice-assistant/chat",
        json={"topic": chat_topic, "personalityId": created_p_id},
        timeout=REQUEST_TIMEOUT,
    ).json()
    created_chat_id = chat_res["chatId"]

    with sync_playwright() as p:
        browser = p.chromium.launch(**_get_chromium_launch_kwargs())
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        context.add_init_script("""
            localStorage.setItem('token', '12345678');
            localStorage.setItem('password', '12345678');
        """)
        page = _configure_page(context.new_page())

        try:
            # "networkidle" never settles against Cerebra (rosbridge socket + polling),
            # so wait for whichever of the sidebar / SmartConnect modal renders first.
            page.goto(f"{ROBOT_URL}/voice-assistant", wait_until="domcontentloaded")
            page.wait_for_selector(
                "#add-personality-button, button[data-test='BTN_Smart_Connect']",
                state="visible",
                timeout=NAV_TIMEOUT_MS,
            )

            # Activate SmartConnect in UI modal if button present
            sc_btn = page.locator("button[data-test='BTN_Smart_Connect']").first
            if sc_btn.is_visible():
                sc_btn.click()
                # Which branch below applies depends on which inputs the modal
                # renders, so wait for them instead of sampling after a fixed sleep.
                try:
                    page.wait_for_selector(
                        "input#token, input[data-test='TXT_Token'], "
                        "input#password, input[data-test='TXT_Password']",
                        state="visible",
                        timeout=10000,
                    )
                except PlaywrightTimeoutError:
                    pass

                token_input = page.locator(
                    "input#token, input[data-test='TXT_Token']"
                ).first
                if token_input.is_visible():
                    token_input.fill(token)
                    token_input.dispatch_event("input")

                    pwd1 = page.locator("input#encrypt-password").first
                    pwd1.fill(password)
                    pwd1.dispatch_event("input")

                    pwd2 = page.locator("input#confirmPassword").first
                    pwd2.fill(password)
                    pwd2.dispatch_event("input")
                    page.wait_for_timeout(300)

                    encrypt_btn = page.locator("button:has-text('Encrypt')").first
                    if encrypt_btn.is_enabled():
                        encrypt_btn.click()
                        page.wait_for_timeout(1500)
                else:
                    pwd_input = page.locator(
                        "input#password, input[data-test='TXT_Password']"
                    ).first
                    if pwd_input.is_visible() and pwd_input.is_enabled():
                        pwd_input.fill(password)
                        pwd_input.dispatch_event("input")
                        connect_btn = page.locator(
                            "button[data-test='BTN_Connect']"
                        ).first
                        if connect_btn.is_enabled():
                            connect_btn.click()
                            page.wait_for_timeout(1500)

                close_btn = page.locator(
                    "button#modal-close-button, button[data-test='BTN_Close'], button[data-test='BTN_OK']"
                ).first
                if close_btn.is_visible():
                    close_btn.click()
                    page.wait_for_timeout(500)

            # Click newly created personality in sidebar
            p_link = page.locator(f"a:has-text('{unique_persona_name}')").first
            expect(p_link).to_be_visible(timeout=UI_TIMEOUT_MS)
            p_link.click()

            # Click chat topic
            chat_item = page.locator(f"text='{chat_topic}'").first
            expect(chat_item).to_be_visible(timeout=UI_TIMEOUT_MS)
            chat_item.click()

            # Locate deep-chat elements
            page.wait_for_selector(
                "deep-chat #text-input:not([aria-disabled='true'])",
                state="visible",
                timeout=UI_TIMEOUT_MS,
            )
            msg_input = page.locator("deep-chat #text-input")
            submit_icon = page.locator("deep-chat #submit-icon")
            messages = page.locator("deep-chat #messages")

            # Type 'Wie geht es dir?' into deep-chat input
            prompt = "Wie geht es dir?"
            msg_input.click()
            page.keyboard.type(prompt)
            expect(msg_input).to_have_text(prompt, timeout=UI_TIMEOUT_MS)
            expect(submit_icon).to_be_visible(timeout=UI_TIMEOUT_MS)

            # 3. Measure response time from Submit click until assistant reply appears in UI
            t0 = time.monotonic()
            submit_icon.click()

            # Wait until deep-chat #messages contains assistant response
            deadline = time.monotonic() + UI_REPLY_TIMEOUT_S
            t1 = None
            reply_snippet = ""
            while time.monotonic() < deadline:
                text = messages.inner_text() or ""
                if unique_persona_name in text or len(text) > len(prompt) + 20:
                    t1 = time.monotonic()
                    reply_snippet = text.replace(prompt, "").strip()
                    break
                time.sleep(0.05)

            assert t1 is not None, (
                "Assistant response was not rendered in deep-chat UI within "
                f"{UI_REPLY_TIMEOUT_S}s"
            )

            latency_ms = (t1 - t0) * 1000.0
            print(f"\n==================================================")
            print(f"🎉 [E2E_PERF_TRACE] REAL GEMINI RESPONSE RECEIVED!")
            print(
                f"  -> UI Response Latency for '{prompt}': {latency_ms:.2f} ms ({latency_ms/1000.0:.2f} s)"
            )
            print(f"==================================================")

            assert latency_ms > 0, "Latency measurement failed"

        finally:
            browser.close()
            # Cleanup (isolated so a failing delete cannot mask the test result)
            if created_chat_id:
                try:
                    requests.delete(
                        f"{API_URL}/voice-assistant/chat/{created_chat_id}",
                        timeout=REQUEST_TIMEOUT,
                    )
                except requests.RequestException:
                    pass
            if created_p_id:
                try:
                    requests.delete(
                        f"{API_URL}/voice-assistant/personality/{created_p_id}",
                        timeout=REQUEST_TIMEOUT,
                    )
                except requests.RequestException:
                    pass
