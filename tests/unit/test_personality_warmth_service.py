"""Cold/warm bridge to the Hermes daemon (PR-2130)."""

from __future__ import annotations

import types
from unittest.mock import MagicMock

import pytest
import requests

from service import personality_warmth_service as warmth


@pytest.fixture(autouse=True)
def daemon_url(monkeypatch):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://daemon.example:8088")


def personality(channel="smart"):
    return types.SimpleNamespace(channel=channel)


def response(status_code=200, body=None):
    entry = MagicMock()
    entry.status_code = status_code
    if body is None:
        entry.json.side_effect = ValueError("no json")
    else:
        entry.json.return_value = body
    return entry


def patch_personality(monkeypatch, channel="smart"):
    monkeypatch.setattr(
        warmth.personality_service,
        "get_personality",
        lambda personality_id: personality(channel),
    )


def patch_requests(monkeypatch, *, get=None, post=None):
    calls = {"get": [], "post": []}

    def _get(url, **kwargs):
        calls["get"].append((url, kwargs))
        if isinstance(get, Exception):
            raise get
        return get

    def _post(url, **kwargs):
        calls["post"].append((url, kwargs))
        if isinstance(post, Exception):
            raise post
        return post

    monkeypatch.setattr(requests, "get", _get)
    monkeypatch.setattr(requests, "post", _post, raising=False)
    return calls


def test_warmth_asks_the_daemon_for_the_cached_state(monkeypatch):
    patch_personality(monkeypatch)
    calls = patch_requests(
        monkeypatch, get=response(200, {"ok": True, "warm": True, "state": "warm"})
    )

    result = warmth.personality_warmth("p-1")

    assert result == {
        "personalityId": "p-1",
        "state": "warm",
        "warm": True,
        "reason": None,
        "detail": "",
        "elapsedMs": None,
    }
    url, kwargs = calls["get"][0]
    assert url == "http://daemon.example:8088/warm"
    assert kwargs["params"] == {"personality_id": "p-1"}


def test_warmth_reports_cold_with_the_daemon_reason(monkeypatch):
    patch_personality(monkeypatch)
    patch_requests(
        monkeypatch,
        get=response(200, {"ok": True, "warm": False, "state": "cold"}),
    )

    result = warmth.personality_warmth("p-1")

    assert result["state"] == "cold"
    assert result["warm"] is False


def test_warm_posts_the_personality_and_reports_the_new_state(monkeypatch):
    patch_personality(monkeypatch)
    calls = patch_requests(
        monkeypatch,
        post=response(
            200, {"ok": True, "warm": True, "state": "warm", "elapsedMs": 1980.5}
        ),
    )

    result = warmth.warm_personality("p-1")

    assert result["state"] == "warm"
    assert result["warm"] is True
    url, kwargs = calls["post"][0]
    assert url == "http://daemon.example:8088/warm"
    assert kwargs["json"] == {"personality_id": "p-1"}


def test_cold_answer_keeps_the_reason_and_the_measured_duration(monkeypatch):
    patch_personality(monkeypatch)
    patch_requests(
        monkeypatch,
        post=response(
            200,
            {
                "ok": True,
                "warm": False,
                "state": "cold",
                "reason": "no_provider_key",
                "elapsedMs": 12.5,
            },
        ),
    )

    result = warmth.warm_personality("p-1")

    assert result["state"] == "cold"
    assert result["reason"] == "no_provider_key"
    assert result["elapsedMs"] == 12.5


def test_an_unreachable_daemon_is_unknown_and_not_cold(monkeypatch):
    patch_personality(monkeypatch)
    patch_requests(monkeypatch, get=requests.ConnectionError("refused"))

    result = warmth.personality_warmth("p-1")

    assert result["state"] == "unknown"
    assert result["reason"] == warmth.REASON_DAEMON_UNREACHABLE
    assert result["warm"] is False


def test_unreachable_daemon_on_warm_is_unknown(monkeypatch):
    patch_personality(monkeypatch)
    patch_requests(monkeypatch, post=requests.Timeout("too slow"))

    result = warmth.warm_personality("p-1")

    assert result["state"] == "unknown"
    assert result["reason"] == warmth.REASON_DAEMON_UNREACHABLE


def test_a_non_200_or_invalid_body_is_unknown(monkeypatch):
    patch_personality(monkeypatch)
    patch_requests(monkeypatch, get=response(500, {"error": "boom"}))
    assert (
        warmth.personality_warmth("p-1")["reason"] == warmth.REASON_DAEMON_UNREACHABLE
    )

    patch_requests(monkeypatch, get=response(200, None))
    assert warmth.personality_warmth("p-1")["reason"] == warmth.REASON_INVALID_RESPONSE


def test_a_direct_personality_is_not_supported_and_never_calls_the_daemon(monkeypatch):
    patch_personality(monkeypatch, channel="direct")
    calls = patch_requests(monkeypatch, get=response(200, {"warm": False}))

    result = warmth.warm_personality("p-1")

    assert result["state"] == "not_supported"
    assert result["reason"] == warmth.REASON_DIRECT_CHANNEL
    assert calls["post"] == []
