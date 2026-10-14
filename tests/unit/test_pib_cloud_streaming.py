"""Unit tests for PC-12: pib-side streaming SSE consumption and sentence-wise TTS handoff."""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
import requests

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from pib_hermes_config.turn_taking import StreamingSpeech
from voice_assistant import pib_cloud_stream
from voice_assistant.pib_cloud_stream import (
    BUDGET_EXHAUSTED_NOTICE,
    CLOUD_AUTH_FAILED_NOTICE,
    CLOUD_UNAVAILABLE_NOTICE,
    EMPTY_CONTENT_NOTICE,
    STREAM_CUTOFF_NOTICE,
    reset_cloud_auth_state,
    stream_pib_cloud_chat,
)


@pytest.fixture(autouse=True)
def _clean_auth_state():
    reset_cloud_auth_state()
    yield
    reset_cloud_auth_state()


class _FakeSSEResponse:
    def __init__(self, lines, status_code: int = 200):
        self._lines = lines
        self.status_code = status_code
        self.closed = False

    def iter_lines(self, chunk_size: int = 1, decode_unicode: bool = True):
        del chunk_size, decode_unicode
        for line in self._lines:
            if self.closed:
                break
            yield line

    def close(self):
        self.closed = True


def _sse_delta(content=None, *, reasoning=None, channel=None) -> str:
    delta = {}
    if content is not None:
        delta["content"] = content
    if reasoning is not None:
        delta["reasoning"] = reasoning
    if channel is not None:
        delta["channel"] = channel
    return "data: " + json.dumps({"id": "chatcmpl-1", "choices": [{"delta": delta}]})


def test_sse_stream_flushes_first_sentence_to_tts_before_stream_done():
    """AC1 & AC3: First sentence is handed to TTS while sentence 2 and [DONE] are still in flight."""
    allow_second_sentence = threading.Event()
    first_sentence_spoken = threading.Event()
    spoken_clauses: list[tuple[str, float]] = []
    stream_done_at: list[float] = []

    def gated_sse_lines():
        yield _sse_delta("Hallo, ")
        yield _sse_delta("ich bin ")
        yield _sse_delta("pib. ")
        # Block upstream SSE stream until the test confirms sentence 1 reached TTS!
        assert allow_second_sentence.wait(
            timeout=2.0
        ), "First sentence was not flushed to TTS while stream was still open!"
        yield _sse_delta("Wie kann ich ")
        yield _sse_delta("dir helfen?")
        stream_done_at.append(time.monotonic())
        yield "data: [DONE]"

    response = _FakeSSEResponse(gated_sse_lines(), status_code=200)
    captured_req = {}

    def fake_post(url, headers=None, json=None, stream=None, timeout=None):
        captured_req["url"] = url
        captured_req["headers"] = headers
        captured_req["json"] = json
        captured_req["stream"] = stream
        return response

    speech = StreamingSpeech()

    def run_consumer():
        tokens = stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Sag hallo.",
            endpoint_base="http://192.168.1.252:8090/v1",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "row4-secret-token"},
            http_post=fake_post,
        )
        accumulated = ""
        for tok in tokens:
            accumulated += tok
            for clause in speech.take(accumulated, False):
                spoken_clauses.append((clause, time.monotonic()))
                if "ich bin pib." in clause:
                    first_sentence_spoken.set()
                    allow_second_sentence.set()
        for clause in speech.take(accumulated, True):
            spoken_clauses.append((clause, time.monotonic()))

    worker = threading.Thread(target=run_consumer)
    worker.start()
    assert first_sentence_spoken.wait(timeout=2.0)
    worker.join(timeout=2.0)
    assert not worker.is_alive()

    assert captured_req["url"] == "http://192.168.1.252:8090/v1/chat/completions"
    assert captured_req["headers"]["Authorization"] == "Bearer row4-secret-token"
    assert captured_req["stream"] is True
    assert "model" not in captured_req["json"]
    assert [c for c, _ in spoken_clauses] == [
        "Hallo,",
        "ich bin pib.",
        "Wie kann ich dir helfen?",
    ]
    assert spoken_clauses[1][1] < stream_done_at[0]
    assert response.closed is True


def test_reasoning_frames_and_control_frames_are_ignored():
    """AC4: delta.reasoning, reasoning channels, and usage frames are never spoken."""
    lines = [
        ": keep-alive",
        _sse_delta(reasoning="Thinking step 1..."),
        _sse_delta("Should not be spoken", channel="reasoning"),
        'data: {"id":"chatcmpl-1","choices":[],"usage":{"prompt_tokens":10,"completion_tokens":5}}',
        _sse_delta(""),
        _sse_delta("Guten "),
        _sse_delta(reasoning="More internal reasoning"),
        _sse_delta("Tag!"),
        "data: [DONE]",
    ]
    response = _FakeSSEResponse(lines, status_code=200)
    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            http_post=lambda *a, **kw: response,
        )
    )
    assert "".join(tokens) == "Guten Tag!"
    assert "Thinking" not in "".join(tokens)


def test_reasoning_only_stream_ending_with_done_yields_deterministic_notice():
    """AC4: A stream that spends all tokens on reasoning and returns no content does not hang or stay silent."""
    lines = [
        _sse_delta(reasoning="Only reasoning tokens..."),
        "data: [DONE]",
    ]
    response = _FakeSSEResponse(lines, status_code=200)
    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            http_post=lambda *a, **kw: response,
        )
    )
    assert tokens == [EMPTY_CONTENT_NOTICE]


def test_402_budget_exhausted_switches_to_local_model_fallback():
    """AC5: HTTP 402 switches the turn to the local model without retrying cloud."""
    calls = 0

    def fake_post(*args, **kwargs):
        nonlocal calls
        calls += 1
        return _FakeSSEResponse([], status_code=402)

    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=lambda: iter(["Lokale Antwort nach Budgetende."]),
            http_post=fake_post,
        )
    )
    assert calls == 1
    assert tokens == ["Lokale Antwort nach Budgetende."]


def test_402_budget_exhausted_speaks_notice_when_local_model_unavailable():
    """AC5: When local model is unavailable on 402, the user hears the budget notice instead of silence."""
    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=lambda: (_ for _ in ()).throw(RuntimeError("ollama down")),
            http_post=lambda *a, **kw: _FakeSSEResponse([], status_code=402),
        )
    )
    assert tokens == [BUDGET_EXHAUSTED_NOTICE]


@pytest.mark.parametrize("status_code", [429, 502, 504])
def test_retryable_status_retries_once_and_recovers(status_code):
    """AC5: 429, 502, and 504 wait briefly and retry once."""
    attempts = []

    def fake_post(*args, **kwargs):
        attempts.append(1)
        if len(attempts) == 1:
            return _FakeSSEResponse([], status_code=status_code)
        return _FakeSSEResponse(
            [_sse_delta("Nach Retry erfolgreich."), "data: [DONE]"], status_code=200
        )

    slept = []
    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            http_post=fake_post,
            retry_delay_s=0.25,
            sleep_fn=slept.append,
        )
    )
    assert len(attempts) == 2
    assert slept == [0.25]
    assert tokens == ["Nach Retry erfolgreich."]


def test_502_falls_back_to_local_model_after_single_retry():
    """AC5: 502/504 retries once, then answers from the local model."""
    attempts = []

    def fake_post(*args, **kwargs):
        attempts.append(1)
        return _FakeSSEResponse([], status_code=502)

    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            local_fallback=lambda: iter(["Antwort vom lokalen Modell."]),
            http_post=fake_post,
            retry_delay_s=0.0,
        )
    )
    assert len(attempts) == 2
    assert tokens == ["Antwort vom lokalen Modell."]


@pytest.mark.parametrize("status_code", [401, 403])
def test_invalid_or_revoked_token_stops_cloud_and_logs_without_secret(
    status_code, caplog
):
    """AC5 & AC6: 401/403 disables cloud for that token, logs loudly without secret, and falls back locally."""
    secret = "pbc_super_secret_token_value"
    calls = 0

    def fake_post(*args, **kwargs):
        nonlocal calls
        calls += 1
        return _FakeSSEResponse([], status_code=status_code)

    first = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Turn 1",
            token_fetcher=lambda: {"mode": "unlocked", "secret": secret},
            local_fallback=lambda: iter(["Lokal 1."]),
            http_post=fake_post,
        )
    )
    second = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Turn 2",
            token_fetcher=lambda: {"mode": "unlocked", "secret": secret},
            local_fallback=lambda: iter(["Lokal 2."]),
            http_post=fake_post,
        )
    )
    # Second turn with the same rejected token must not hit the cloud endpoint again.
    assert calls == 1
    assert first == ["Lokal 1."]
    assert second == ["Lokal 2."]
    assert secret not in caplog.text


def test_mid_stream_error_event_speaks_partial_answer_and_cutoff_notice():
    """AC5: Mid-stream SSE error does not retry and appends the cutoff notice to partial text."""
    calls = 0

    def fake_post(*args, **kwargs):
        nonlocal calls
        calls += 1
        return _FakeSSEResponse(
            [
                _sse_delta("Erster Teil des Satzes."),
                "event: error",
                'data: {"error":{"code":"upstream_error","message":"stream dropped"}}',
            ],
            status_code=200,
        )

    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Erzaehl was.",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            http_post=fake_post,
        )
    )
    assert calls == 1
    assert "".join(tokens) == f"Erster Teil des Satzes. {STREAM_CUTOFF_NOTICE}"


def test_barge_in_closes_http_stream_immediately():
    """AC4: Barge-in cancellation closes the open HTTP stream and stops yielding tokens."""
    cancelled = False

    def sse_lines():
        nonlocal cancelled
        yield _sse_delta("Satz eins.")
        cancelled = True
        yield _sse_delta("Satz zwei darf nicht mehr kommen.")
        yield "data: [DONE]"

    response = _FakeSSEResponse(sse_lines(), status_code=200)
    tokens = list(
        stream_pib_cloud_chat(
            system_prompt="Du bist pib.",
            user_text="Hallo",
            token_fetcher=lambda: {"mode": "unlocked", "secret": "tok-1"},
            is_cancelled=lambda: cancelled,
            http_post=lambda *a, **kw: response,
        )
    )
    assert tokens == ["Satz eins."]
    assert response.closed is True
