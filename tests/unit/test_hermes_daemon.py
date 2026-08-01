"""Unit tests for the long-lived Hermes daemon HTTP service (PR-1535)."""

from __future__ import annotations

import json
import threading
import time
from urllib.request import Request, urlopen

import pytest

from public_api_client import hermes_daemon as hd


@pytest.fixture()
def daemon_server(monkeypatch):
    """Start an in-process daemon on an ephemeral port with a stub turn runner."""
    replies = {"value": "daemon-says-hi"}

    def runner(*, text, chat_id, personality_id=None, toolsets=None, timeout=None):
        replies["last"] = {
            "text": text,
            "chat_id": chat_id,
            "personality_id": personality_id,
            "toolsets": toolsets,
            "timeout": timeout,
        }
        return replies["value"]

    # Bind to an ephemeral port so parallel tests / leftover daemons do not clash.
    server = hd.create_server(host="127.0.0.1", port=0, turn_runner=runner)
    host, port = server.server_address
    monkeypatch.setenv("PIB_HERMES_DAEMON_HOST", host)
    monkeypatch.setenv("PIB_HERMES_DAEMON_PORT", str(port))
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", f"http://{host}:{port}")

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if hd.is_daemon_reachable(timeout=0.1):
            break
        time.sleep(0.02)
    else:
        server.shutdown()
        server.server_close()
        pytest.fail("daemon did not become reachable")

    yield server, replies

    server.shutdown()
    server.server_close()
    thread.join(timeout=2.0)


def test_health_returns_ok(daemon_server):
    server, _ = daemon_server
    host, port = server.server_address
    with urlopen(f"http://{host}:{port}/health", timeout=1) as resp:
        assert resp.status == 200
        assert json.loads(resp.read().decode()) == {"status": "ok"}


def test_turn_accepts_payload_and_returns_reply(daemon_server):
    server, replies = daemon_server
    host, port = server.server_address
    body = json.dumps(
        {
            "text": "hallo",
            "chat_id": "c-1",
            "personality_id": "p-9",
            "toolsets": "mcp",
            "timeout": 30,
        }
    ).encode()
    req = Request(
        f"http://{host}:{port}/turn",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(req, timeout=2) as resp:
        assert resp.status == 200
        payload = json.loads(resp.read().decode())

    assert payload == {"reply": "daemon-says-hi"}
    assert replies["last"] == {
        "text": "hallo",
        "chat_id": "c-1",
        "personality_id": "p-9",
        "toolsets": "mcp",
        "timeout": 30,
    }


def test_turn_rejects_missing_fields(daemon_server):
    from urllib.error import HTTPError

    server, _ = daemon_server
    host, port = server.server_address
    req = Request(
        f"http://{host}:{port}/turn",
        data=b'{"text": "hi"}',
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with pytest.raises(HTTPError) as exc_info:
        urlopen(req, timeout=2)
    assert exc_info.value.code == 400


def test_start_daemon_helper_serves_health(monkeypatch):
    """start_daemon binds and answers /health; stop_daemon tears it down."""
    hd.stop_daemon()

    # Pick a free port first.
    probe = hd.create_server(host="127.0.0.1", port=0, turn_runner=lambda **_: "x")
    host, port = probe.server_address
    probe.server_close()

    monkeypatch.setenv("PIB_HERMES_DAEMON_HOST", host)
    monkeypatch.setenv("PIB_HERMES_DAEMON_PORT", str(port))
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", f"http://{host}:{port}")

    started = hd.start_daemon(
        host=host,
        port=port,
        turn_runner=lambda **_: "x",
    )
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not hd.is_daemon_reachable(timeout=0.1):
            time.sleep(0.02)
        assert hd.is_daemon_reachable(timeout=0.5)
        assert started.server_address[1] == port
    finally:
        hd.stop_daemon()
        assert not hd.is_daemon_reachable(timeout=0.2)


def test_ensure_daemon_running_is_idempotent_when_already_up(daemon_server):
    assert hd.ensure_daemon_running(wait_seconds=1.0) is True
    assert hd.is_daemon_reachable()


def test_is_daemon_reachable_false_when_down(monkeypatch):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")
    assert hd.is_daemon_reachable(timeout=0.2) is False
