"""PC-52: correlate cloud chat failures with on-device text fallback (observability only)."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest
import requests

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
HERMES_CONFIG_PKG = REPO_ROOT / "pib_hermes_config"
for path in (VOICE_ASSISTANT_PKG, HERMES_CONFIG_PKG):
    text = str(path)
    if text not in sys.path:
        sys.path.insert(0, text)

from voice_assistant.pib_cloud_stream import (  # noqa: E402
    CLOUD_UNAVAILABLE_NOTICE,
    LOCAL_FALLBACK_LOG_PREFIX,
    LOCAL_FALLBACK_MODEL_NAME,
    parse_local_fallback_line,
    reset_cloud_auth_state,
    stream_pib_cloud_chat,
)

_SIBLING = Path(__file__).resolve().parent / "test_pib_cloud_streaming.py"
_spec = importlib.util.spec_from_file_location("_pc52_pib_cloud_streaming", _SIBLING)
assert _spec and _spec.loader
_pc12 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pc12)
_FakeSSEResponse = _pc12._FakeSSEResponse
_sse_delta = _pc12._sse_delta


@pytest.fixture(autouse=True)
def _clean_auth_state():
    reset_cloud_auth_state()
    yield
    reset_cloud_auth_state()


def _fallback_events(caplog) -> list[dict]:
    events = []
    for record in caplog.records:
        parsed = parse_local_fallback_line(record.getMessage())
        if parsed is not None:
            events.append(parsed)
    return events


def test_502_local_fallback_success_logs_one_correlated_event(caplog):
    """Cloud 502 after retry: ids from the last attempt, one fallback event, local succeeds."""
    caplog.set_level("INFO")
    attempts = []

    def fake_post(*args, **kwargs):
        attempts.append(1)
        headers = {
            "X-Request-ID": "req-cloud-502",
            "X-Attempt-ID": f"attempt-{len(attempts)}",
        }
        return _FakeSSEResponse([], status_code=502, headers=headers)

    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            chat_id="chat-pc52-ok",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=lambda: iter(["Lokale Antwort."]),
            http_post=fake_post,
            retry_delay_s=0.0,
        )
    )
    assert len(attempts) == 2
    assert tokens == ["Lokale Antwort."]
    events = _fallback_events(caplog)
    assert len(events) == 1
    event = events[0]
    assert event["event"] == "pib_cloud_local_fallback"
    assert event["chat_id"] == "chat-pc52-ok"
    assert event["cloud_request_id"] == "req-cloud-502"
    assert event["cloud_attempt_id"] == "attempt-2"
    assert event["cloud_http_status"] == 502
    assert event["fallback_reason"] == "cloud_http_retry_exhausted"
    assert event["local_model"] == LOCAL_FALLBACK_MODEL_NAME
    assert event["outcome"] == "success"
    assert event["local_first_token_ms"] >= 0.0
    assert event["local_total_ms"] >= event["local_first_token_ms"]


def test_502_without_correlation_headers_omits_ids(caplog):
    """Mutation guard: correlation ids must come from response headers, not invented."""
    caplog.set_level("INFO")
    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            chat_id="chat-no-headers",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=lambda: iter(["Lokal."]),
            http_post=lambda *a, **kw: _FakeSSEResponse([], status_code=502),
            retry_delay_s=0.0,
        )
    )
    assert tokens == ["Lokal."]
    event = _fallback_events(caplog)[0]
    assert "cloud_request_id" not in event
    assert "cloud_attempt_id" not in event


def test_cloud_failure_with_local_fallback_error_logs_error_outcome(caplog):
    caplog.set_level("INFO")

    def broken_local():
        raise RuntimeError("ollama down")

    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            chat_id="chat-local-fail",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=broken_local,
            http_post=lambda *a, **kw: _FakeSSEResponse(
                [],
                status_code=502,
                headers={
                    "X-Request-ID": "req-fail",
                    "X-Attempt-ID": "attempt-final",
                },
            ),
            retry_delay_s=0.0,
        )
    )
    assert tokens == [CLOUD_UNAVAILABLE_NOTICE]
    events = _fallback_events(caplog)
    assert len(events) == 1
    assert events[0]["outcome"] == "error"
    assert events[0]["cloud_request_id"] == "req-fail"
    assert events[0]["fallback_reason"] == "cloud_http_retry_exhausted"


def test_transport_timeout_after_retry_logs_timeout_reason(caplog):
    caplog.set_level("INFO")
    calls = []

    def fake_post(*args, **kwargs):
        calls.append(1)
        raise requests.Timeout("read timed out")

    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            chat_id="chat-timeout",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=lambda: iter(["Nach Timeout lokal."]),
            http_post=fake_post,
            retry_delay_s=0.0,
        )
    )
    assert len(calls) == 2
    assert tokens == ["Nach Timeout lokal."]
    events = _fallback_events(caplog)
    assert len(events) == 1
    assert events[0]["fallback_reason"] == "cloud_transport_timeout"
    assert events[0]["outcome"] == "success"


def test_successful_cloud_stream_emits_no_fallback_event(caplog):
    caplog.set_level("INFO")
    response = _FakeSSEResponse(
        [_sse_delta("Hallo."), "data: [DONE]"],
        status_code=200,
        headers={"X-Request-ID": "req-ok"},
    )
    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hi",
            chat_id="chat-cloud-ok",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=lambda: iter(["soll nicht laufen"]),
            http_post=lambda *a, **kw: response,
        )
    )
    assert tokens == ["Hallo."]
    assert _fallback_events(caplog) == []
    assert LOCAL_FALLBACK_LOG_PREFIX not in caplog.text


def test_cloud_failure_without_local_model_emits_no_fallback_event(caplog):
    caplog.set_level("INFO")
    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            chat_id="chat-notice-only",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=None,
            http_post=lambda *a, **kw: _FakeSSEResponse([], status_code=502),
            retry_delay_s=0.0,
        )
    )
    assert tokens == [CLOUD_UNAVAILABLE_NOTICE]
    assert _fallback_events(caplog) == []


def test_parse_local_fallback_line_rejects_malformed_payload():
    assert parse_local_fallback_line("not a fallback line") is None
    assert parse_local_fallback_line(f"{LOCAL_FALLBACK_LOG_PREFIX} {{") is None
    good = f"{LOCAL_FALLBACK_LOG_PREFIX} " + json.dumps(
        {"event": "pib_cloud_local_fallback", "outcome": "success"}
    )
    assert parse_local_fallback_line(good)["outcome"] == "success"
