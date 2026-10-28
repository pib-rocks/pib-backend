"""PR-2075: Direct session caches keys, HTTP connections, and system prompt bytes."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from voice_assistant import direct_tool_loop  # noqa: E402
from voice_assistant.direct_session import DirectSession  # noqa: E402
from voice_assistant.direct_tool_loop import (  # noqa: E402
    build_gemini_request,
    run_direct_turn,
)


def _gemini_json_payload(parts):
    return {"candidates": [{"content": {"parts": parts}}]}


def _sse_body(*chunks):
    lines = []
    for chunk in chunks:
        lines.append(f"data: {json.dumps(chunk)}")
    lines.append("")
    return "\n".join(lines)


class _UrlopenResponse:
    def __init__(self, *, sse_text: str):
        self._sse_text = sse_text

    def __iter__(self):
        for line in self._sse_text.splitlines(keepends=True):
            yield line.encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


@pytest.fixture(autouse=True)
def _direct_keepalive_off(monkeypatch):
    monkeypatch.setenv("DIRECT_HTTP_KEEPALIVE", "0")
    direct_tool_loop.reset_gemini_http_transport()


def test_provider_key_fetched_once_across_two_turns(monkeypatch):
    fetches = {"n": 0}
    epochs = iter([3, 3])

    def fake_epoch():
        return next(epochs)

    def fake_fetch(provider, api_name=None):
        fetches["n"] += 1
        return {"mode": "unlocked", "secret": "secret-a"}

    monkeypatch.setattr(direct_tool_loop, "_fetch_store_key", fake_fetch)
    monkeypatch.setattr(
        "voice_assistant.direct_session.read_key_store_epoch", fake_epoch
    )

    session = DirectSession(personality_id="eva")

    def fake_urlopen(request, timeout=None):
        sse = _sse_body(_gemini_json_payload([{"text": "Hi."}]))
        return _UrlopenResponse(sse_text=sse)

    monkeypatch.setattr(direct_tool_loop, "urlopen", fake_urlopen)

    for user_text in ("Hallo.", "Nochmal."):
        "".join(
            run_direct_turn(
                system_prompt="Du bist Eva.",
                user_text=user_text,
                history=[],
                tool_calling=False,
                allow_image=False,
                session=session,
            )
        )
    assert fetches["n"] == 1


def test_provider_key_refetched_after_key_epoch_change(monkeypatch):
    fetches = {"n": 0}
    epochs = iter([1, 2])

    def fake_epoch():
        return next(epochs)

    def fake_fetch(provider, api_name=None):
        fetches["n"] += 1
        return {"mode": "unlocked", "secret": f"secret-{fetches['n']}"}

    monkeypatch.setattr(direct_tool_loop, "_fetch_store_key", fake_fetch)
    monkeypatch.setattr(
        "voice_assistant.direct_session.read_key_store_epoch", fake_epoch
    )

    session = DirectSession(personality_id="eva")

    def fake_urlopen(request, timeout=None):
        sse = _sse_body(_gemini_json_payload([{"text": "Ok."}]))
        return _UrlopenResponse(sse_text=sse)

    monkeypatch.setattr(direct_tool_loop, "urlopen", fake_urlopen)

    for user_text in ("Eins.", "Zwei."):
        "".join(
            run_direct_turn(
                system_prompt="Du bist Eva.",
                user_text=user_text,
                history=[],
                tool_calling=False,
                allow_image=False,
                session=session,
            )
        )
    assert fetches["n"] == 2


def test_http_client_reused_across_two_turns(monkeypatch):
    monkeypatch.setenv("DIRECT_HTTP_KEEPALIVE", "1")
    direct_tool_loop.reset_gemini_http_transport()

    monkeypatch.setattr(
        direct_tool_loop,
        "resolve_provider_key",
        lambda *args, **kwargs: ("k", "environment"),
    )

    sse = _sse_body(_gemini_json_payload([{"text": "x"}]))

    class _FakeResponse:
        status = 200
        reason = "OK"

        def read(self):
            return b""

        def __iter__(self):
            for line in sse.splitlines(keepends=True):
                yield line.encode("utf-8")

    class _FakeConnection:
        instances: list["_FakeConnection"] = []

        def __init__(self, host, timeout=None):
            _FakeConnection.instances.append(self)

        def request(self, *args, **kwargs):
            return None

        def getresponse(self):
            return _FakeResponse()

        def close(self):
            return None

    _FakeConnection.instances = []
    monkeypatch.setattr(
        direct_tool_loop.http.client, "HTTPSConnection", _FakeConnection
    )

    for user_text in ("a", "b"):
        "".join(
            run_direct_turn(
                system_prompt="sys",
                user_text=user_text,
                history=[],
                tool_calling=False,
                allow_image=False,
            )
        )
    assert direct_tool_loop.gemini_http_connections_opened() == 1
    assert len(_FakeConnection.instances) == 1


def test_system_prompt_bytes_identical_across_two_turns():
    session = DirectSession(personality_id="eva")
    first = session.stable_system_prompt("Du bist Eva.\n\nKontext A.")
    second = session.stable_system_prompt("Du bist Eva.\n\nKontext B.")
    assert first == second
    assert first.encode("utf-8") == second.encode("utf-8")

    _, _, body_one = build_gemini_request(
        [{"role": "system", "content": first}, {"role": "user", "content": "Hi"}],
        None,
        "k",
    )
    _, _, body_two = build_gemini_request(
        [{"role": "system", "content": second}, {"role": "user", "content": "Bye"}],
        None,
        "k",
    )
    assert body_one["systemInstruction"] == body_two["systemInstruction"]


def test_identity_change_replaces_the_frozen_prompt():
    """A personality edit must not keep the old identity (PR-2075 review)."""
    session = DirectSession("p-edit")

    first = session.stable_system_prompt("Du bist Eva.", "eva-v1")
    assert session.stable_system_prompt("Du bist Eva.", "eva-v1") == first
    assert first == "Du bist Eva."

    edited = session.stable_system_prompt("Du bist Eva, jetzt knapp.", "eva-v2")
    assert edited == "Du bist Eva, jetzt knapp."
