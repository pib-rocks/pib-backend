"""Unit tests for the long-lived Hermes daemon HTTP service (PR-1535)."""

from __future__ import annotations

import json
import sys
import threading
import time
import types
from unittest.mock import MagicMock, patch
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


def test_turn_emits_perf_trace_logs(daemon_server, caplog):
    """Daemon /turn path must emit [PERF_TRACE] markers for latency profiling."""
    import logging

    server, _ = daemon_server
    host, port = server.server_address
    body = json.dumps({"text": "hallo", "chat_id": "c-perf"}).encode()
    req = Request(
        f"http://{host}:{port}/turn",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with caplog.at_level(logging.INFO):
        with urlopen(req, timeout=2) as resp:
            assert resp.status == 200

    messages = [rec.getMessage() for rec in caplog.records]
    assert any("[PERF_TRACE] DAEMON_RECV" in m for m in messages)
    assert any("[PERF_TRACE] DAEMON_TURN_START" in m for m in messages)
    assert any("[PERF_TRACE] DAEMON_FIRST_TOKEN" in m for m in messages)
    assert any("[PERF_TRACE] DAEMON_DONE" in m for m in messages)


def test_client_uses_session_pooling_for_daemon(daemon_server, monkeypatch):
    """Warm-daemon turns must reuse a persistent requests.Session."""
    from public_api_client import hermes_agent_client as hac

    server, _ = daemon_server
    host, port = server.server_address
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", f"http://{host}:{port}")

    # Reset singleton so this test owns a fresh session.
    hac._daemon_http_session = None

    with patch.object(hac, "hermes_binary_available", return_value=True):
        reply1 = hac.run_turn("hi", "c1")
        session_after_first = hac._daemon_http_session
        reply2 = hac.run_turn("hi again", "c1")
        session_after_second = hac._daemon_http_session

    assert reply1 == "daemon-says-hi"
    assert reply2 == "daemon-says-hi"
    assert session_after_first is not None
    assert session_after_first is session_after_second


def test_run_turn_in_process_calls_hermes_run_agent(tmp_path, monkeypatch):
    """Default daemon turn path must invoke Hermes in-process when available."""
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    fake_run_agent = MagicMock(return_value="  in-process-reply  ")
    fake_module = types.ModuleType("hermes.run_agent")
    fake_module.run_agent = fake_run_agent

    with patch.dict(
        sys.modules,
        {
            "hermes": types.ModuleType("hermes"),
            "hermes.run_agent": fake_module,
        },
    ), patch(
        "public_api_client.hermes_agent_client.ensure_profile",
        return_value=str(tmp_path / "profiles" / "pib_pers-1"),
    ) as ensure_profile, patch(
        "public_api_client.hermes_agent_client.run_turn_subprocess",
    ) as subprocess_runner:
        reply = hd.run_turn_in_process(
            text="Hallo",
            chat_id="chat-42",
            personality_id="pers-1",
            timeout=30,
        )

    assert reply == "in-process-reply"
    ensure_profile.assert_called_once_with("pers-1")
    fake_run_agent.assert_called_once_with(
        prompt="Hallo",
        session_id="pib_chat_chat-42",
        profile="pib_pers-1",
        timeout=30,
    )
    subprocess_runner.assert_not_called()


def test_run_turn_in_process_falls_back_to_subprocess_when_import_fails():
    """Missing hermes.run_agent must fall back to the CLI subprocess path."""
    import builtins

    real_import = builtins.__import__

    def _block_hermes_run_agent(name, *args, **kwargs):
        if name == "hermes.run_agent" or name == "hermes":
            raise ImportError("no hermes package")
        return real_import(name, *args, **kwargs)

    with patch("builtins.__import__", side_effect=_block_hermes_run_agent), patch(
        "public_api_client.hermes_agent_client.run_turn_subprocess",
        return_value="subprocess-reply",
    ) as subprocess_runner:
        reply = hd.run_turn_in_process(
            text="Hallo",
            chat_id="chat-7",
            personality_id="pers-1",
            toolsets="pib",
            timeout=45,
        )

    assert reply == "subprocess-reply"
    subprocess_runner.assert_called_once_with(
        text="Hallo",
        chat_id="chat-7",
        personality_id="pers-1",
        toolsets="pib",
        timeout=45,
    )


def test_default_turn_runner_uses_in_process_path():
    """create_server default runner must be the in-process implementation."""
    with patch.object(
        hd, "run_turn_in_process", return_value="from-default"
    ) as in_process:
        reply = hd._default_turn_runner(
            text="hi",
            chat_id="c1",
            personality_id="p1",
            toolsets=None,
            timeout=10,
        )

    assert reply == "from-default"
    in_process.assert_called_once_with(
        text="hi",
        chat_id="c1",
        personality_id="p1",
        toolsets=None,
        timeout=10,
    )


def test_run_turn_in_process_returns_fallback_on_agent_error(tmp_path, monkeypatch):
    from public_api_client.hermes_agent_client import FALLBACK_REPLY

    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    fake_module = types.ModuleType("hermes.run_agent")
    fake_module.run_agent = MagicMock(side_effect=RuntimeError("boom"))

    with patch.dict(
        sys.modules,
        {
            "hermes": types.ModuleType("hermes"),
            "hermes.run_agent": fake_module,
        },
    ), patch(
        "public_api_client.hermes_agent_client.ensure_profile",
        return_value="/tmp/p",
    ):
        reply = hd.run_turn_in_process(
            text="Hi", chat_id="c", personality_id="pers-1",
        )

    assert reply == FALLBACK_REPLY
