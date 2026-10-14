"""POST /v1/session and POST /v1/session/usage, with the token kept out of logs."""

from __future__ import annotations

import io
import json
import logging
import sys
from dataclasses import fields
from pathlib import Path
from urllib.error import HTTPError

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from voice_assistant.cloud_session import (  # noqa: E402
    CloudSessionClient,
    CloudSessionError,
    LiveSessionGrant,
)

BEARER = "test-pib-bearer"
EPHEMERAL = "test-ephemeral-token"
ENDPOINT = (
    "wss://generativelanguage.googleapis.com/ws/"
    "google.ai.generativelanguage.v1alpha.GenerativeService."
    "BidiGenerateContentConstrained"
)


class Response:
    def __init__(self, status: int, body: bytes):
        self.status = status
        self._body = body

    def read(self):
        return self._body

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


class ScriptedOpener:
    """urlopen stand-in. Status >= 400 is raised the way urllib raises it."""

    def __init__(self, responses: list[tuple[int, dict]]):
        self._responses = list(responses)
        self.requests = []
        self.timeouts = []

    def __call__(self, request, timeout):
        self.requests.append(request)
        self.timeouts.append(timeout)
        status, payload = self._responses.pop(0)
        raw = json.dumps(payload).encode("utf-8")
        if status >= 400:
            raise HTTPError(
                request.full_url,
                status,
                "error",
                hdrs=None,
                fp=io.BytesIO(raw),
            )
        return Response(status, raw)


def _client(opener, token: str = BEARER) -> CloudSessionClient:
    return CloudSessionClient(
        base_url="http://cloud.test/",
        token=token,
        timeout_s=4.0,
        opener=opener,
    )


def _error_body(code: str, message: str = "rejected") -> dict:
    return {
        "error": {
            "message": message,
            "type": "invalid_request_error",
            "code": code,
            "param": None,
        }
    }


def _visible(caplog) -> str:
    text = caplog.text
    for record in caplog.records:
        text += record.getMessage()
        text += repr(record.args)
    return text


def test_open_session_parses_every_grant_field():
    body = {
        "status": "ok",
        "use_cascade": False,
        "session_id": "ses_test_1",
        "ephemeral_token": EPHEMERAL,
        "issued_at": "2026-10-14T12:00:00Z",
        "expires_at": "2026-10-14T12:05:00Z",
        "ttl_seconds": 300,
        "model": "models/gemini-3.8-live",
        "transport": "websocket",
        "endpoint": ENDPOINT,
        "pre_hold_pib_tokens": 12.5,
        "daily_slots_used": 1,
        "daily_slots_limit": 20,
        "daily_slots_remaining": 19,
    }
    opener = ScriptedOpener([(200, body)])
    grant = _client(opener).open_session(
        model="models/gemini-3.8-live",
        previous_session_id="ses_prev",
    )
    assert isinstance(grant, LiveSessionGrant)
    assert grant.use_cascade is False
    assert grant.session_id == "ses_test_1"
    assert grant.token == EPHEMERAL
    assert grant.model == "models/gemini-3.8-live"
    assert grant.endpoint == ENDPOINT
    assert grant.expires_at == "2026-10-14T12:05:00Z"
    assert grant.ttl_seconds == 300
    assert grant.reason is None
    assert grant.cascade_endpoint is None
    assert grant.daily_slots_remaining == 19
    assert {field.name for field in fields(LiveSessionGrant)} == {
        "use_cascade",
        "session_id",
        "token",
        "model",
        "endpoint",
        "expires_at",
        "ttl_seconds",
        "reason",
        "cascade_endpoint",
        "daily_slots_remaining",
    }


def test_daily_quota_fallback_returns_cascade_without_raising():
    body = {
        "status": "fallback",
        "use_cascade": True,
        "reason": "daily_live_quota_exhausted",
        "cascade_endpoint": "/v1/chat/completions",
        "daily_slots_used": 20,
        "daily_slots_limit": 20,
        "daily_slots_remaining": 0,
    }
    grant = _client(ScriptedOpener([(200, body)])).open_session(
        model="models/gemini-3.8-live"
    )
    assert grant.use_cascade is True
    assert grant.reason == "daily_live_quota_exhausted"
    assert grant.cascade_endpoint == "/v1/chat/completions"
    assert grant.daily_slots_remaining == 0
    assert grant.token is None
    assert grant.session_id is None


@pytest.mark.parametrize(
    ("status", "code"),
    [
        (401, "invalid_token"),
        (403, "token_revoked"),
        (400, "invalid_request"),
        (402, "budget_exhausted"),
        (502, "upstream_unavailable"),
        (504, "upstream_timeout"),
    ],
)
def test_error_statuses_raise_with_the_typed_code(status, code):
    client = _client(ScriptedOpener([(status, _error_body(code))]))
    with pytest.raises(CloudSessionError) as caught:
        client.open_session(model="models/gemini-3.8-live")
    assert caught.value.code == code
    assert caught.value.status == status
    assert BEARER not in str(caught.value)
    assert EPHEMERAL not in str(caught.value)


def test_error_message_does_not_repeat_the_token():
    body = _error_body("invalid_token", message=f"rejected {BEARER}")
    client = _client(ScriptedOpener([(401, body)]))
    with pytest.raises(CloudSessionError) as caught:
        client.open_session()
    assert caught.value.code == "invalid_token"
    assert caught.value.status == 401
    assert BEARER not in str(caught.value)
    assert "<redacted>" in str(caught.value)


def test_grant_repr_redacts_the_token():
    grant = LiveSessionGrant(
        use_cascade=False,
        session_id="ses_test_1",
        token=EPHEMERAL,
        model="models/gemini-3.8-live",
        endpoint=ENDPOINT,
        expires_at="2026-10-14T12:05:00Z",
        ttl_seconds=300,
        daily_slots_remaining=19,
    )
    rendered = repr(grant)
    assert EPHEMERAL not in rendered
    assert "<redacted>" in rendered
    assert "ses_test_1" in rendered


def test_request_carries_authorization_and_the_json_body():
    opener = ScriptedOpener(
        [
            (
                200,
                {
                    "status": "ok",
                    "use_cascade": False,
                    "session_id": "ses_test_1",
                    "ephemeral_token": EPHEMERAL,
                    "expires_at": "2026-10-14T12:05:00Z",
                    "ttl_seconds": 300,
                    "model": "models/gemini-3.8-live",
                    "endpoint": ENDPOINT,
                    "daily_slots_remaining": 19,
                },
            ),
            (
                200,
                {
                    "status": "settled",
                    "session_id": "ses_test_1",
                    "billed_pib_tokens": 1.0,
                },
            ),
        ]
    )
    client = _client(opener)
    client.open_session(
        model="models/gemini-3.8-live",
        previous_session_id="ses_prev",
    )
    opened = opener.requests[0]
    assert opened.full_url == "http://cloud.test/v1/session"
    assert opened.get_method() == "POST"
    assert opened.get_header("Authorization") == f"Bearer {BEARER}"
    assert opened.get_header("Content-type") == "application/json"
    assert json.loads(opened.data.decode("utf-8")) == {
        "model": "models/gemini-3.8-live",
        "previous_session_id": "ses_prev",
    }
    assert BEARER not in opened.full_url
    assert BEARER not in opened.data.decode("utf-8")
    assert opener.timeouts[0] == 4.0

    usage = {
        "promptTokenCount": 3,
        "responseTokenCount": 1,
    }
    reported = client.report_usage(
        "ses_test_1",
        audio_input_tokens=3,
        audio_output_tokens=1,
        text_tokens=0,
        video_input_tokens=0,
        usage_metadata=usage,
        model="models/gemini-3.8-live",
    )
    assert reported["status"] == "settled"
    assert reported["session_id"] == "ses_test_1"
    settled = opener.requests[1]
    assert settled.full_url == "http://cloud.test/v1/session/usage"
    assert settled.get_header("Authorization") == f"Bearer {BEARER}"
    assert json.loads(settled.data.decode("utf-8")) == {
        "session_id": "ses_test_1",
        "audio_input_tokens": 3,
        "audio_output_tokens": 1,
        "video_input_tokens": 0,
        "text_tokens": 0,
        "usage_metadata": usage,
        "model": "models/gemini-3.8-live",
    }


def test_calls_do_not_log_the_token(caplog):
    caplog.set_level(logging.DEBUG)
    opener = ScriptedOpener(
        [
            (
                200,
                {
                    "status": "ok",
                    "use_cascade": False,
                    "session_id": "ses_test_1",
                    "ephemeral_token": EPHEMERAL,
                    "model": "models/gemini-3.8-live",
                    "endpoint": ENDPOINT,
                    "expires_at": "2026-10-14T12:05:00Z",
                    "ttl_seconds": 300,
                    "daily_slots_remaining": 19,
                },
            ),
            (200, {"status": "settled", "session_id": "ses_test_1"}),
            (401, _error_body("invalid_token", message=f"nope {BEARER}")),
        ]
    )
    client = _client(opener)
    grant = client.open_session(model="models/gemini-3.8-live")
    client.report_usage("ses_test_1", audio_input_tokens=2)
    with pytest.raises(CloudSessionError):
        client.open_session()
    visible = _visible(caplog) + repr(grant)
    assert BEARER not in visible
    assert EPHEMERAL not in visible
    assert caplog.records


def test_transport_failure_raises_without_the_token():
    def opener(request, timeout):
        raise TimeoutError(f"timed out with {BEARER}")

    client = _client(opener)
    with pytest.raises(CloudSessionError) as caught:
        client.open_session(model="models/gemini-3.8-live")
    assert BEARER not in str(caught.value)
    assert caught.value.status is None
    assert caught.value.code is None
