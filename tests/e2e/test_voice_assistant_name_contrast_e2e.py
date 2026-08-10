"""Live-robot E2E: voice-assistant chat participant name contrast (PR-1580)."""

from __future__ import annotations

import os
import re
from typing import Optional, Tuple

import pytest
import requests
from playwright.sync_api import sync_playwright


def _get_chromium_launch_kwargs() -> dict:
    kwargs = {"headless": True}
    for p in ["/usr/bin/chromium-browser", "/usr/bin/chromium"]:
        if os.path.exists(p):
            kwargs["executable_path"] = p
            break
    return kwargs


ROBOT_URL = os.environ.get("PIB_E2E_BASE_URL", "http://192.168.1.28").rstrip("/")
API_URL = f"{ROBOT_URL}/api"
REQUEST_TIMEOUT = 15

# Verified live on the robot: both labels render in this chat.
FALLBACK_PERSONALITY_ID = "8f73b580-927e-41c2-98ac-e5df070e7288"
FALLBACK_CHAT_ID = "b4f01552-0c09-401c-8fde-fda753fb0261"
CHAT_BACKGROUND = "#041939"
MIN_CONTRAST_RATIO = 4.5


def _parse_rgb(color: str) -> Tuple[int, int, int]:
    """Parse CSS color strings like 'rgb(0, 0, 0)' or 'rgba(0, 0, 0, 1)'."""
    match = re.fullmatch(
        r"rgba?\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)(?:\s*,\s*[\d.]+)?\s*\)",
        color.strip(),
    )
    if not match:
        raise ValueError(f"Unsupported CSS color: {color!r}")
    return int(match.group(1)), int(match.group(2)), int(match.group(3))


def _srgb_channel_to_linear(channel_8bit: int) -> float:
    c = channel_8bit / 255.0
    if c <= 0.03928:
        return c / 12.92
    return ((c + 0.055) / 1.055) ** 2.4


def _relative_luminance(r: int, g: int, b: int) -> float:
    return (
        0.2126 * _srgb_channel_to_linear(r)
        + 0.7152 * _srgb_channel_to_linear(g)
        + 0.0722 * _srgb_channel_to_linear(b)
    )


def contrast_ratio(foreground: str, background_hex: str) -> float:
    """WCAG 2.1 contrast ratio between a CSS rgb()/rgba() color and a #rrggbb background."""
    fr, fg, fb = _parse_rgb(foreground)
    bg = background_hex.lstrip("#")
    if len(bg) != 6:
        raise ValueError(f"Unsupported background hex: {background_hex!r}")
    br, bg_, bb = int(bg[0:2], 16), int(bg[2:4], 16), int(bg[4:6], 16)
    l1 = _relative_luminance(fr, fg, fb)
    l2 = _relative_luminance(br, bg_, bb)
    lighter, darker = max(l1, l2), min(l1, l2)
    return (lighter + 0.05) / (darker + 0.05)


def _get_json(path: str):
    response = requests.get(f"{API_URL}{path}", timeout=REQUEST_TIMEOUT)
    response.raise_for_status()
    return response.json()


def _messages_have_both_sides(chat_id: str) -> bool:
    try:
        messages = _get_json(f"/voice-assistant/chat/{chat_id}/messages").get(
            "messages", []
        )
    except (requests.RequestException, ValueError, KeyError):
        return False
    has_user = any(m.get("isUser") for m in messages)
    has_ai = any(not m.get("isUser") for m in messages)
    return has_user and has_ai


def _discover_chat_with_both_labels() -> Optional[Tuple[str, str]]:
    """Return (personality_id, chat_id) for a chat that renders both name labels.

    Prefer API discovery via the messages endpoint. The per-personality chat-list
    endpoint has been observed returning an error, so fall back to the verified
    Eva / Nuernberg pair when needed.
    """
    env_personality = os.environ.get("PIB_NAME_CONTRAST_E2E_PERSONALITY_ID")
    env_chat = os.environ.get("PIB_NAME_CONTRAST_E2E_CHAT_ID")
    if env_personality and env_chat and _messages_have_both_sides(env_chat):
        return env_personality, env_chat

    candidates: list[Tuple[str, str]] = [
        (FALLBACK_PERSONALITY_ID, FALLBACK_CHAT_ID),
    ]

    try:
        personalities = _get_json("/voice-assistant/personality").get(
            "voiceAssistantPersonalities", []
        )
    except (requests.RequestException, ValueError) as exc:
        pytest.skip(
            f"name-contrast E2E prerequisite absent: robot API is unreachable ({exc})"
        )

    # Prefer Eva when present among personalities (verified chat ids below).
    for personality in personalities:
        if personality.get("personalityId") == FALLBACK_PERSONALITY_ID:
            candidates.insert(0, (FALLBACK_PERSONALITY_ID, FALLBACK_CHAT_ID))
            break

    seen: set[Tuple[str, str]] = set()
    for personality_id, chat_id in candidates:
        key = (personality_id, chat_id)
        if key in seen:
            continue
        seen.add(key)
        if _messages_have_both_sides(chat_id):
            return personality_id, chat_id

    return None


def test_voice_assistant_name_labels_meet_wcag_aa_contrast():
    """Participant .name labels inside deep-chat must contrast ≥ 4.5:1 on #041939."""
    requests.post(
        f"{API_URL}/system/smart-connect",
        json={"token": "12345678", "password": "12345678"},
        timeout=REQUEST_TIMEOUT,
    )

    discovered = _discover_chat_with_both_labels()
    if discovered is None:
        pytest.skip(
            "name-contrast E2E prerequisite absent: no chat with both a user and "
            "an ai message was found (tried Eva/Nuernberg fallback and env overrides)"
        )

    personality_id, chat_id = discovered
    chat_url = f"{ROBOT_URL}/voice-assistant/{personality_id}/chat/{chat_id}"

    with sync_playwright() as p:
        browser = p.chromium.launch(**_get_chromium_launch_kwargs())
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        context.add_init_script(
            """
            localStorage.setItem('token', '12345678');
            localStorage.setItem('password', '12345678');
            """
        )
        page = context.new_page()
        try:
            page.goto(chat_url, wait_until="networkidle")
            page.wait_for_selector("deep-chat", state="attached", timeout=30000)
            # Wait until both Shadow DOM name labels exist (user + ai).
            page.wait_for_function(
                """() => {
                    const host = document.querySelector('deep-chat');
                    if (!host || !host.shadowRoot) return false;
                    return host.shadowRoot.querySelectorAll('.name').length >= 2;
                }""",
                timeout=30000,
            )

            name_colors = page.evaluate(
                """() => {
                    const host = document.querySelector('deep-chat');
                    if (!host || !host.shadowRoot) {
                        return [];
                    }
                    return Array.from(host.shadowRoot.querySelectorAll('.name')).map(
                        (n) => ({
                            text: (n.textContent || '').trim(),
                            className: n.className,
                            color: getComputedStyle(n).color,
                        })
                    );
                }"""
            )

            assert len(name_colors) >= 2, (
                f"Expected at least 2 .name labels inside deep-chat Shadow DOM, "
                f"found {len(name_colors)}: {name_colors}"
            )

            for label in name_colors:
                color = label["color"]
                assert color != "rgb(0, 0, 0)", (
                    f"Name label {label!r} still computes to black on the dark chat "
                    f"background (pre-fix regression)"
                )
                ratio = contrast_ratio(color, CHAT_BACKGROUND)
                assert ratio >= MIN_CONTRAST_RATIO, (
                    f"Name label {label!r} contrast ratio {ratio:.2f}:1 against "
                    f"{CHAT_BACKGROUND} is below WCAG AA {MIN_CONTRAST_RATIO}:1"
                )
        finally:
            browser.close()
