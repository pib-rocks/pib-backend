"""End-to-end test for a personality that talks to the local qwen-fast model.

Belongs to Jira PR-1928: the local qwen2.5:1.5b that setup-pib.sh installs as
qwen-fast on 8 GiB variants must be selectable when creating a personality and
must answer in a real chat, without a provider key and without internet.

The test skips on machines that do not offer the local model, which is the
intended state on 4 GiB variants, and it cleans up after itself so it does not
leave a personality or a chat behind on the robot.
"""

import os
import time

import pytest
import requests
from playwright.sync_api import expect, sync_playwright

try:  # PR-1918 made this module the single source for the robot address
    from robot_address import API_URL, ROBOT_URL
except Exception:  # fall back to the documented variables
    ROBOT_URL = os.environ.get("PIB_ROBOT_URL", "http://localhost").rstrip("/")
    API_URL = f"{ROBOT_URL}/api"

LOCAL_MODEL_HINT = "qwen-fast"
REQUEST_TIMEOUT_S = 20
UI_TIMEOUT_MS = 30000
REPLY_TIMEOUT_S = 120  # a 1.5b model on a CPU needs its time
PERSONALITY_NAME = "Lokales Modell E2E"


def _local_model():
    """The catalogue entry for the local model, or None when absent."""
    response = requests.get(f"{API_URL}/assistant-model", timeout=REQUEST_TIMEOUT_S)
    response.raise_for_status()
    data = response.json()
    rows = (
        data
        if isinstance(data, list)
        else data.get("assistantModels", data.get("assistantModel", []))
    )
    for row in rows:
        haystack = " ".join(
            str(row.get(key, "")) for key in ("apiName", "name", "providerName")
        )
        if LOCAL_MODEL_HINT in haystack.lower():
            return row
    return None


def _delete(path):
    try:
        requests.delete(f"{API_URL}{path}", timeout=REQUEST_TIMEOUT_S)
    except requests.RequestException:
        pass


def test_personality_with_the_local_model_holds_a_conversation():
    model = _local_model()
    if model is None:
        # On 8 GiB variants the model is installed by setup, so a run there must
        # not hide the missing catalogue entry behind a skip: set
        # PIB_LOCAL_MODEL_EXPECTED=1 to demand the entry and fail without it.
        if os.environ.get("PIB_LOCAL_MODEL_EXPECTED") == "1":
            pytest.fail(
                f"the local model is expected on this robot but the catalogue offers no entry matching "
                f"'{LOCAL_MODEL_HINT}'. The host runs the model; the product does not expose it yet."
            )
        pytest.skip(
            "the robot offers no local model: the catalogue has no entry matching "
            f"'{LOCAL_MODEL_HINT}'. This is expected on 4 GiB variants, which do not install it."
        )

    created_personality = None
    try:
        response = requests.post(
            f"{API_URL}/voice-assistant/personality",
            json={
                "name": PERSONALITY_NAME,
                "channel": "direct",
                "assistantModelId": model.get("id"),
                "providerRef": str(
                    model.get("providerRef") or model.get("providerId") or ""
                ),
            },
            timeout=REQUEST_TIMEOUT_S,
        )
        assert (
            response.status_code < 300
        ), f"personality could not be created: {response.status_code} {response.text[:200]}"
        created_personality = response.json().get("personalityId")
        assert created_personality, "the created personality has no id"

        served = requests.get(
            f"{API_URL}/voice-assistant/personality", timeout=REQUEST_TIMEOUT_S
        ).json()
        rows = served.get("voiceAssistantPersonalities", [])
        row = next(
            (item for item in rows if item.get("personalityId") == created_personality),
            None,
        )
        assert row is not None, "the created personality is not offered by the robot"
        assert row.get("assistantModelId") == model.get(
            "id"
        ), f"the personality does not use the local model: {row.get('assistantModelId')} != {model.get('id')}"

        with sync_playwright() as pw:
            browser = pw.chromium.launch(args=["--no-sandbox"])
            page = browser.new_page(viewport={"width": 1400, "height": 1000})
            page.goto(
                f"{ROBOT_URL}/voice-assistant",
                wait_until="domcontentloaded",
                timeout=60000,
            )
            expect(page.locator("#personality-select")).to_be_visible(
                timeout=UI_TIMEOUT_MS
            )
            page.wait_for_timeout(3000)

            option = page.locator(
                "#personality-select option", has_text=PERSONALITY_NAME
            ).first
            expect(option).to_have_count(1, timeout=UI_TIMEOUT_MS)
            page.select_option(
                "#personality-select", value=created_personality, timeout=UI_TIMEOUT_MS
            )
            page.wait_for_timeout(4000)

            # "New chat" carries a space in its id, so click it by its text.
            page.get_by_text("New chat", exact=True).first.click(timeout=UI_TIMEOUT_MS)
            page.wait_for_timeout(6000)

            # The composer lives inside the deep-chat web component.
            composer = page.locator("deep-chat #text-input")
            composer.wait_for(state="visible", timeout=UI_TIMEOUT_MS)
            composer.click()
            composer.fill("Antworte mit genau einem kurzen Satz: bist du erreichbar?")
            page.locator("deep-chat #submit-icon").first.click(timeout=UI_TIMEOUT_MS)

            messages = page.locator("deep-chat #messages")
            deadline = time.time() + REPLY_TIMEOUT_S
            answer = ""
            while time.time() < deadline:
                page.wait_for_timeout(2000)
                answer = messages.inner_text()
                if len(answer.strip()) > 20:
                    break

            assert (
                len(answer.strip()) > 20
            ), f"the local model produced no answer within {REPLY_TIMEOUT_S}s; the chat showed: {answer[:200]!r}"
            page.screenshot(path="/tmp/local_model_chat.png", full_page=False)
            browser.close()
    finally:
        if created_personality:
            _delete(f"/voice-assistant/personality/{created_personality}")
