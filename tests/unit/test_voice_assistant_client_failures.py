"""PR-2022: failed chat-message API calls must not build DTOs or log request bodies."""

from __future__ import annotations

import io
import logging
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from urllib.error import HTTPError
from urllib.request import Request

REPO_ROOT = Path(__file__).resolve().parents[2]
CLIENT_PKG = REPO_ROOT / "pib_api" / "client"
if str(CLIENT_PKG) not in sys.path:
    sys.path.insert(0, str(CLIENT_PKG))

import pib_api_client
from pib_api_client import voice_assistant_client as vac

HUGE_PAYLOAD_MARKER = "PR2022_HUGE_REQUEST_BODY_SHOULD_NOT_APPEAR_IN_LOGS"
HUGE_BODY = (HUGE_PAYLOAD_MARKER + "x") * 20000


def _http_error(status: int = 400, body: str = "bad request") -> HTTPError:
    return HTTPError(
        url="http://localhost:5000/voice-assistant/chat/c/messages",
        code=status,
        msg="Bad Request",
        hdrs={},
        fp=io.BytesIO(body.encode("utf-8")),
    )


def test_create_chat_message_returns_false_none_when_send_request_fails(monkeypatch):
    monkeypatch.setattr(vac, "send_request", lambda _req: (False, None))

    successful, message = vac.create_chat_message("chat-1", "hello", True)

    assert successful is False
    assert message is None


def test_update_chat_message_returns_false_none_when_send_request_fails(monkeypatch):
    monkeypatch.setattr(vac, "send_request", lambda _req: (False, None))

    successful, message = vac.update_chat_message("chat-1", "hello", True, "msg-9")

    assert successful is False
    assert message is None


def test_create_chat_message_returns_false_none_when_payload_is_not_a_dict(
    monkeypatch,
):
    monkeypatch.setattr(vac, "send_request", lambda _req: (True, None))

    successful, message = vac.create_chat_message("chat-1", "hello", False)

    assert successful is False
    assert message is None


def test_send_request_logs_payload_length_not_body_on_http_error(monkeypatch, caplog):
    def fake_urlopen(request: Request):
        raise _http_error(body="server said no")

    monkeypatch.setattr(pib_api_client, "urlopen", fake_urlopen)

    request = Request(
        "http://localhost:5000/voice-assistant/chat/c/messages",
        data=HUGE_BODY.encode("utf-8"),
        method="POST",
    )

    with caplog.at_level(logging.ERROR):
        successful, payload = pib_api_client.send_request(request)

    assert successful is False
    assert payload is None
    assert len(caplog.records) == 1
    message = caplog.records[0].message
    assert str(len(HUGE_BODY.encode("utf-8"))) in message
    assert HUGE_PAYLOAD_MARKER not in message
    assert "server said no" in message


def test_get_chat_returns_false_none_when_send_request_fails(monkeypatch):
    monkeypatch.setattr(vac, "send_request", lambda _req: (False, None))

    successful, chat = vac.get_chat("missing-chat-id")

    assert successful is False
    assert chat is None


def test_get_chat_returns_false_none_when_payload_is_not_a_dict(monkeypatch):
    monkeypatch.setattr(vac, "send_request", lambda _req: (True, None))

    successful, chat = vac.get_chat("chat-1")

    assert successful is False
    assert chat is None


def test_get_personality_from_chat_returns_false_none_for_unknown_chat(monkeypatch):
    monkeypatch.setattr(vac, "send_request", lambda _req: (False, None))

    successful, personality = vac.get_personality_from_chat("unknown-chat")

    assert successful is False
    assert personality is None


def test_send_request_truncates_long_response_body_in_log(monkeypatch, caplog):
    long_body = "x" * 500

    def fake_urlopen(request: Request):
        raise _http_error(body=long_body)

    monkeypatch.setattr(pib_api_client, "urlopen", fake_urlopen)

    request = Request(
        "http://localhost:5000/test",
        data=b"{}",
        method="POST",
    )

    with caplog.at_level(logging.ERROR):
        pib_api_client.send_request(request)

    message = caplog.records[0].message
    assert long_body[:300] in message
    assert long_body not in message
