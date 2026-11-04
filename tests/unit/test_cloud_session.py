"""Live session mint and usage client (pib.Cloud ``/session``)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from voice_assistant.cloud_session import (  # noqa: E402
    CascadeResult,
    CloudSession,
    CloudSessionError,
    CloudUsageResult,
    mint_session,
    report_usage,
    session_url,
)
from voice_assistant.cloud_token import CloudTokenError  # noqa: E402

BEARER = "sc-test-CLOUD-token-91"
EPHEMERAL = "ephemeral-live-token-77"
MODEL = "gemini-live-preview"


def _unlocked_fetch(secret: str = BEARER):
    def fetch():
        return {"mode": "unlocked", "secret": secret}

    return fetch


def _ok_body(**overrides):
    body = {
        "status": "ok",
        "use_cascade": False,
        "session_id": "sess-abc",
        "ephemeral_token": EPHEMERAL,
        "issued_at": "2026-01-01T00:00:00Z",
        "expires_at": "2026-01-01T00:05:00Z",
        "ttl_seconds": 300,
        "model": MODEL,
        "transport": "websocket",
        "endpoint": "wss://live.example/v1",
        "pre_hold_pib_tokens": 12.5,
        "daily_slots_used": 1,
        "daily_slots_limit": 10,
        "daily_slots_remaining": 9,
    }
    body.update(overrides)
    return json.dumps(body).encode("utf-8")


class _Response:
    def __init__(self, raw: bytes, status: int = 200) -> None:
        self.status = status
        self._raw = raw

    def read(self) -> bytes:
        return self._raw

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def _opener(raw: bytes, status: int = 200):
    captured = {}

    def open_(request, timeout):
        captured["url"] = request.full_url
        captured["headers"] = dict(request.header_items())
        captured["body"] = request.data
        captured["timeout"] = timeout
        return _Response(raw, status)

    return open_, captured


def test_mint_session_success(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    opener, captured = _opener(_ok_body())
    result = mint_session(MODEL, fetch=_unlocked_fetch(), opener=opener)
    assert isinstance(result, CloudSession)
    assert result.session_id == "sess-abc"
    assert result.ephemeral_token == EPHEMERAL
    assert result.daily_slots_remaining == 9
    # The service answers with a float; an int-only parser rejected every real mint.
    assert result.pre_hold_pib_tokens == 12.5
    assert captured["url"] == session_url("https://cloud.example/v1")
    assert captured["headers"]["Authorization"] == f"Bearer {BEARER}"
    sent = json.loads(captured["body"].decode("utf-8"))
    assert sent == {"model": MODEL}


def test_mint_session_sends_previous_session_id(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    opener, captured = _opener(_ok_body())
    mint_session(
        MODEL,
        previous_session_id="prev-1",
        fetch=_unlocked_fetch(),
        opener=opener,
    )
    sent = json.loads(captured["body"].decode("utf-8"))
    assert sent == {"model": MODEL, "previous_session_id": "prev-1"}


def test_mint_session_use_cascade():
    raw = json.dumps(
        {"status": "ok", "use_cascade": True, "reason": "slots_full"}
    ).encode()
    opener, _ = _opener(raw)
    result = mint_session(MODEL, fetch=_unlocked_fetch(), opener=opener)
    assert result == CascadeResult(reason="slots_full")


@pytest.mark.parametrize(
    "status,code,body",
    [
        (401, "unauthorized", {"error": f"bad token {BEARER}"}),
        (402, "budget_exhausted", {"error": "budget exhausted"}),
        (400, "invalid_request", {"error": "unknown model"}),
        (500, "upstream_unavailable", {"error": "internal"}),
    ],
)
def test_mint_session_http_errors(status, code, body):
    raw = json.dumps(body).encode()
    opener, _ = _opener(raw, status=status)
    with pytest.raises(CloudSessionError) as exc:
        mint_session(MODEL, fetch=_unlocked_fetch(), opener=opener)
    assert exc.value.code == code
    assert BEARER not in str(exc.value)
    assert EPHEMERAL not in str(exc.value)


def test_mint_session_malformed_json():
    opener, _ = _opener(b"not-json", status=200)
    with pytest.raises(CloudSessionError) as exc:
        mint_session(MODEL, fetch=_unlocked_fetch(), opener=opener)
    assert exc.value.code == "bad_response"


def test_mint_session_timeout():
    def opener(request, timeout):
        del request, timeout
        raise TimeoutError

    with pytest.raises(CloudSessionError) as exc:
        mint_session(MODEL, fetch=_unlocked_fetch(), opener=opener)
    assert exc.value.code == "timeout"


def test_mint_session_missing_token():
    def locked_fetch():
        return {"mode": "degraded", "secret": BEARER}

    with pytest.raises(CloudTokenError):
        mint_session(MODEL, fetch=locked_fetch)


def test_error_message_never_contains_bearer_or_ephemeral(caplog):
    caplog.set_level("INFO")
    raw = json.dumps(
        {
            "status": "ok",
            "use_cascade": False,
            "session_id": "x",
            "ephemeral_token": EPHEMERAL,
            "issued_at": "t",
            "expires_at": "t",
            "ttl_seconds": 1,
            "model": MODEL,
            "transport": "ws",
            "endpoint": "wss://x",
            "pre_hold_pib_tokens": 1,
            "daily_slots_used": 1,
            "daily_slots_limit": 1,
            "daily_slots_remaining": 0,
        }
    ).encode()
    opener, _ = _opener(raw, status=401)
    with pytest.raises(CloudSessionError) as exc:
        mint_session(MODEL, fetch=_unlocked_fetch(), opener=opener)
    visible = caplog.text + str(exc.value)
    assert BEARER not in visible
    assert EPHEMERAL not in visible


def test_report_usage_posts_metadata():
    raw = json.dumps({"status": "settled", "settled_pib_tokens": 4}).encode()
    opener, captured = _opener(raw)
    metadata = {"totalTokenCount": 99}
    result = report_usage(
        "sess-abc",
        metadata,
        fetch=_unlocked_fetch(),
        opener=opener,
    )
    assert result == CloudUsageResult(settled_pib_tokens=4, status="settled")
    sent = json.loads(captured["body"].decode("utf-8"))
    assert sent == {"session_id": "sess-abc", "usage_metadata": metadata}
