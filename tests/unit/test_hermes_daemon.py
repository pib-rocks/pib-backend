"""Unit tests for the long-lived Hermes daemon HTTP service (PR-1535)."""

from __future__ import annotations

import errno
import importlib
import json
import logging
import os
import re
import stat
import sys
import threading
import time
import types
from contextvars import ContextVar
from pathlib import Path
from unittest.mock import MagicMock, patch
from urllib.request import Request, urlopen

import pytest

from public_api_client import hermes_daemon as hd

_REAL_DISCOVER_MCP_TOOLS = hd._discover_mcp_tools
_REAL_IN_PROCESS_AVAILABLE = hd.in_process_available


@pytest.fixture(autouse=True)
def isolate_mcp_discovery(monkeypatch):
    """Keep daemon unit tests independent of the developer's Hermes config and state.db."""
    monkeypatch.setattr(hd, "_mcp_discovery_attempted", set())
    monkeypatch.setattr(hd, "_discover_mcp_tools", MagicMock())
    monkeypatch.setattr(hd, "_registered_mcp_tool_count", MagicMock(return_value=0))
    monkeypatch.setattr(hd, "_session_dbs", {})
    monkeypatch.setattr(hd, "_session_db_attempted", set())
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=None))


@pytest.fixture(autouse=True)
def fake_hermes_home_override(monkeypatch, sandboxed_hermes_home):
    current_home = ContextVar("test_hermes_home", default=sandboxed_hermes_home)
    module = types.ModuleType("hermes_constants")
    module.get_hermes_home = current_home.get
    module.set_hermes_home_override = current_home.set
    module.reset_hermes_home_override = current_home.reset
    monkeypatch.setitem(sys.modules, "hermes_constants", module)
    return current_home


@pytest.fixture(autouse=True)
def matching_hermes_venv(monkeypatch):
    """Declare the interpreter venv-compatible so the in-process path is taken.

    ``_run_turn_in_home`` runs the agent in-process only when Hermes' own venv has
    site-packages for THIS interpreter; without one it deliberately uses the CLI
    subprocess instead (see test_no_matching_hermes_venv_uses_the_cli_subprocess).
    These tests stub ``run_agent`` in ``sys.modules`` and assert in-process
    behaviour, so they state the venv as present. ``venv_site_packages`` itself
    keeps its real implementation, so its own tests still exercise it.
    """
    monkeypatch.setattr(hd, "in_process_available", lambda: True)


class _FakeSessionDB:
    """Stand-in for ``hermes_state.SessionDB`` — no SQLite, one transcript per session."""

    def __init__(self):
        self.transcripts: dict[str, list[dict]] = {}
        self.resume_calls: list[str] = []
        self.reopened: list[str] = []
        self.closed = 0
        self.ensure_calls: list[tuple[str, str]] = []
        self.batch_appends: list[tuple[str, list[dict]]] = []

    def append_turn(self, session_id, user_message, reply):
        self.transcripts.setdefault(session_id, []).extend(
            [
                {"role": "user", "content": user_message},
                {"role": "assistant", "content": reply},
            ]
        )

    def get_resume_conversations(self, session_id):
        self.resume_calls.append(session_id)
        return list(self.transcripts.get(session_id, [])), []

    def reopen_session(self, session_id):
        self.reopened.append(session_id)

    def ensure_session(self, session_id, source="unknown", **kwargs):
        self.ensure_calls.append((session_id, source))
        self.transcripts.setdefault(session_id, [])

    def append_messages_batch(self, session_id, messages, **kwargs):
        rows = self.transcripts.setdefault(session_id, [])
        inserted = 0
        for message in messages:
            rows.append(dict(message))
            inserted += 1
        self.batch_appends.append((session_id, list(messages)))
        return inserted

    def close(self):
        self.closed += 1


def _flask_message(message_id, content, is_user=True):
    message = MagicMock()
    message.message_id = message_id
    message.content = content
    message.is_user = is_user
    return message


def _fake_agent_module(created, *, store_turns=False, reply="OK."):
    """A ``run_agent`` module whose AIAgent records the history each turn receives.

    ``store_turns`` additionally appends the turn to the store it was constructed with,
    the way a real AIAgent with a ``session_db`` persists its messages.
    """

    class FakeAgent:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.session_id = kwargs.get("session_id")
            self.histories = []
            created.append(self)

        def reset_session_state(self, **kwargs):
            pass

        def run_conversation(
            self, user_message, conversation_history=None, stream_callback=None
        ):
            self.histories.append(conversation_history)
            store = self.kwargs.get("session_db")
            if store_turns and store is not None:
                sid = getattr(self, "session_id", None) or self.kwargs.get("session_id")
                store.append_turn(sid, user_message, reply)
            return {"final_response": reply}

    module = types.ModuleType("run_agent")
    module.AIAgent = FakeAgent
    return module


@pytest.fixture()
def daemon_server(monkeypatch):
    """Start an in-process daemon on an ephemeral port with a stub turn runner."""
    replies = {"value": "daemon-says-hi"}

    def runner(
        *,
        text,
        chat_id,
        personality_id=None,
        toolsets=None,
        enabled_toolsets=None,
        max_turns=None,
        timeout=None,
        stream_callback=None,
    ):
        replies["last"] = {
            "text": text,
            "chat_id": chat_id,
            "personality_id": personality_id,
            "toolsets": toolsets,
            "enabled_toolsets": enabled_toolsets,
            "max_turns": max_turns,
            "timeout": timeout,
        }
        for delta in replies.get("deltas", []):
            stream_callback(delta)
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


def test_profile_endpoint_creates_complete_layout_idempotently(
    daemon_server, monkeypatch
):
    server, _ = daemon_server
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    host, port = server.server_address
    body = json.dumps(
        {
            "personality_id": "profile-1",
            "personality_name": "Ada",
            "soul_text": "Sei neugierig.",
        }
    ).encode()
    request = Request(
        f"http://{host}:{port}/profile",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with urlopen(request, timeout=2) as response:
        first = json.loads(response.read().decode())
    with urlopen(request, timeout=2) as response:
        second = json.loads(response.read().decode())

    profile_dir = Path(first["profile_dir"])
    assert first["created"] is True
    assert second["created"] is False
    assert all((profile_dir / dirname).is_dir() for dirname in hd.PROFILE_DIRS)
    assert (profile_dir / "config.yaml").is_file()
    assert (profile_dir / "SOUL.md").is_file()

    with urlopen(
        f"http://{host}:{port}/profile?personality_id=profile-1", timeout=2
    ) as response:
        status = json.loads(response.read().decode())
    assert status["ok"] is True
    assert status["has_config"] is True
    assert status["has_memories"] is True
    assert status["has_sessions"] is True
    assert status["has_soul"] is True


def _post_profile(server, personality_id: str, name: str = "Ada") -> dict:
    """Provision (or repair) one profile through the daemon's own endpoint."""
    host, port = server.server_address
    body = json.dumps(
        {
            "personality_id": personality_id,
            "personality_name": name,
            "soul_text": "Sei neugierig.",
        }
    ).encode()
    request = Request(
        f"http://{host}:{port}/profile",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=2) as response:
        return json.loads(response.read().decode())


def test_profile_env_is_restricted_to_0600(daemon_server, monkeypatch):
    """The guard must not quietly drop the mode change where it does apply."""
    server, _ = daemon_server
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")

    result = _post_profile(server, "cover-mode")

    env_path = Path(result["profile_dir"]) / ".env"
    assert env_path.is_file()
    assert stat.S_IMODE(env_path.stat().st_mode) == 0o600


def test_read_only_profile_env_cover_does_not_abort_provisioning(
    daemon_server, monkeypatch
):
    """PC-45: the voice container's profile .env is a read-only credential cover.

    Measured on .172 after the voice container was recreated with the filtered
    Hermes view: ``os.chmod`` on the covered ``profiles/<id>/.env`` raises
    EROFS, which aborted ``ensure_profile_home`` and made every Hermes-mediated
    voice turn answer with the fallback reply. The cover is a blank placeholder,
    so a refused chmod must not stop provisioning.
    """
    server, _ = daemon_server
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")

    env_path = Path(_post_profile(server, "cover-readonly")["profile_dir"]) / ".env"
    assert env_path.is_file()

    real_chmod = hd.os.chmod

    def read_only_cover(path, mode, *args, **kwargs):
        if str(path) == str(env_path):
            raise OSError(errno.EROFS, "Read-only file system", str(path))
        return real_chmod(path, mode, *args, **kwargs)

    monkeypatch.setattr(hd.os, "chmod", read_only_cover)

    repaired = _post_profile(server, "cover-readonly")

    # The turn path aborts on the exception, so the response and the layout are
    # the contract: a repaired profile answers ok and holds its files.
    assert repaired["ok"] is True
    profile_dir = Path(repaired["profile_dir"])
    assert (profile_dir / "config.yaml").is_file()
    assert (profile_dir / "SOUL.md").is_file()
    assert (profile_dir / "memories").is_dir()


def test_profile_endpoint_persists_the_personalitys_model_and_route(
    daemon_server, monkeypatch
):
    """PR-1930b: /profile writes the model/provider a personality is set to."""
    server, _ = daemon_server
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    host, port = server.server_address
    body = json.dumps(
        {
            "personality_id": "profile-model",
            "personality_name": "Ada",
            "soul_text": "Sei neugierig.",
            "model": "qwen-fast",
            "provider": "Local",
            "endpoint_base": "http://host.docker.internal:11434/v1",
        }
    ).encode()
    request = Request(
        f"http://{host}:{port}/profile",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with urlopen(request, timeout=2) as response:
        result = json.loads(response.read().decode())

    profile_dir = Path(result["profile_dir"])
    import yaml

    with open(profile_dir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)

    assert cfg["model"] == "qwen-fast"
    assert cfg["provider"] == "local"
    assert cfg["providers"]["local"]["base_url"] == (
        "http://host.docker.internal:11434/v1"
    )


def test_profile_endpoint_writes_reasoning_effort_into_the_agent_block(
    daemon_server, monkeypatch
):
    """The level Hermes reads is agent.reasoning_effort, not a root key."""
    server, _ = daemon_server
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    host, port = server.server_address
    body = json.dumps(
        {"personality_id": "profile-effort", "reasoning_effort": "none"}
    ).encode()
    request = Request(
        f"http://{host}:{port}/profile",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with urlopen(request, timeout=2) as response:
        result = json.loads(response.read().decode())

    import yaml

    with open(Path(result["profile_dir"]) / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)

    assert cfg["agent"]["reasoning_effort"] == "none"
    assert "reasoning_effort" not in cfg


def test_profile_endpoint_leaves_reasoning_effort_alone_when_null(
    daemon_server, monkeypatch
):
    server, _ = daemon_server
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    from pib_hermes_config import profile_dir_for

    pdir = Path(profile_dir_for("profile-unmanaged"))
    pdir.mkdir(parents=True)
    (pdir / "config.yaml").write_text(
        "agent:\n  reasoning_effort: medium\n", encoding="utf-8"
    )
    host, port = server.server_address
    body = json.dumps(
        {"personality_id": "profile-unmanaged", "reasoning_effort": None}
    ).encode()
    request = Request(
        f"http://{host}:{port}/profile",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with urlopen(request, timeout=2) as response:
        assert json.loads(response.read().decode())["ok"] is True

    import yaml

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert cfg["agent"]["reasoning_effort"] == "medium"
    assert "reasoning_effort" not in cfg


def test_profile_endpoint_rejects_an_unknown_reasoning_effort(daemon_server):
    from urllib.error import HTTPError

    server, _ = daemon_server
    host, port = server.server_address
    request = Request(
        f"http://{host}:{port}/profile",
        data=b'{"personality_id":"p","reasoning_effort":"turbo"}',
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with pytest.raises(HTTPError) as exc_info:
        urlopen(request, timeout=2)

    assert exc_info.value.code == 400
    detail = json.loads(exc_info.value.read().decode())
    assert detail["ok"] is False
    assert "none" in detail["error"]
    assert "ultra" in detail["error"]


def test_profile_endpoint_rejects_a_non_string_model(daemon_server):
    from urllib.error import HTTPError

    server, _ = daemon_server
    host, port = server.server_address
    request = Request(
        f"http://{host}:{port}/profile",
        data=b'{"personality_id":"p","model":5}',
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with pytest.raises(HTTPError) as exc_info:
        urlopen(request, timeout=2)

    assert exc_info.value.code == 400
    assert json.loads(exc_info.value.read().decode()) == {
        "ok": False,
        "error": "model must be a string",
    }


def test_profile_endpoint_surfaces_factory_failure_without_profile(
    daemon_server, monkeypatch, tmp_path
):
    from urllib.error import HTTPError

    server, _ = daemon_server
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setattr(
        hd, "ensure_profile_home", MagicMock(side_effect=RuntimeError("factory failed"))
    )
    host, port = server.server_address
    request = Request(
        f"http://{host}:{port}/profile",
        data=b'{"personality_id":"broken"}',
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with pytest.raises(HTTPError) as exc_info:
        urlopen(request, timeout=2)

    assert exc_info.value.code == 500
    assert json.loads(exc_info.value.read().decode()) == {
        "ok": False,
        "error": "factory failed",
    }
    assert not (tmp_path / "profiles" / "pib_broken").exists()


def test_turn_accepts_payload_and_returns_reply(daemon_server):
    server, replies = daemon_server
    host, port = server.server_address
    body = json.dumps(
        {
            "text": "hallo",
            "chat_id": "c-1",
            "personality_id": "p-9",
            "toolsets": "mcp",
            "enabled_toolsets": "mcp-pib,vision",
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

    assert payload == {"reply": "daemon-says-hi", "toolsUsed": []}
    assert replies["last"] == {
        "text": "hallo",
        "chat_id": "c-1",
        "personality_id": "p-9",
        "toolsets": "mcp",
        "enabled_toolsets": "mcp-pib,vision",
        "max_turns": None,
        "timeout": 30,
    }


def test_streaming_turn_emits_deltas_and_final_reply(daemon_server):
    server, replies = daemon_server
    replies["deltas"] = ["Hallo", " Welt."]
    host, port = server.server_address
    body = json.dumps({"text": "hallo", "chat_id": "c-stream", "stream": True}).encode()
    req = Request(
        f"http://{host}:{port}/turn",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with urlopen(req, timeout=2) as resp:
        chunks = [json.loads(line) for line in resp if line.strip()]

    assert chunks[:-1] == [{"delta": "Hallo"}, {"delta": " Welt."}]
    assert chunks[-1] == {"reply": "daemon-says-hi", "toolsUsed": []}


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


def test_turn_rejects_invalid_enabled_toolsets(daemon_server):
    from urllib.error import HTTPError

    server, _ = daemon_server
    host, port = server.server_address
    req = Request(
        f"http://{host}:{port}/turn",
        data=b'{"text": "hi", "chat_id": "c-1", "enabled_toolsets": ["mcp-pib"]}',
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with pytest.raises(HTTPError) as exc_info:
        urlopen(req, timeout=2)

    assert exc_info.value.code == 400
    assert json.loads(exc_info.value.read().decode()) == {
        "error": "enabled_toolsets must be a string"
    }


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

    server, replies = daemon_server
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
    assert (
        replies["last"]["toolsets"]
        == "terminal,code_execution,file,memory,session_search"
    )
    assert replies["last"]["enabled_toolsets"] == "pib,vision"
    assert replies["last"]["max_turns"] == 4


def test_run_turn_in_process_constructs_and_reuses_one_agent_per_chat(
    tmp_path, monkeypatch
):
    """Each chat owns one configured AIAgent and reuses it on later turns."""
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    created = []

    class FakeAgent:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.session_id = kwargs.get("session_id")
            self.run_conversation = MagicMock(
                return_value={"final_response": "  in-process-reply  "}
            )
            created.append(self)

        def reset_session_state(self, **kwargs):
            pass

    fake_module = types.ModuleType("run_agent")
    fake_module.AIAgent = FakeAgent
    hd.clear_agent_cache()

    with (
        patch.dict(
            sys.modules,
            {"run_agent": fake_module},
        ),
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            return_value=str(tmp_path / "profiles" / "pib_pers-1"),
        ),
        patch(
            "public_api_client.hermes_agent_client.run_turn_subprocess",
        ) as subprocess_runner,
    ):
        first = hd.run_turn_in_process(
            text="Hallo",
            chat_id="chat-42",
            personality_id="pers-1",
            toolsets="terminal,file",
            enabled_toolsets="mcp-pib,vision",
            max_turns=7,
            timeout=30,
        )
        second = hd.run_turn_in_process(
            text="Noch einmal",
            chat_id="chat-42",
            personality_id="pers-1",
            toolsets="terminal,file",
            enabled_toolsets="mcp-pib,vision",
            max_turns=7,
        )

    assert first == second == "in-process-reply"
    assert len(created) == 1
    assert created[0].kwargs["session_id"] is None
    assert created[0].session_id == "pib_chat_chat-42"
    assert created[0].kwargs["enabled_toolsets"] == ["mcp-pib", "vision"]
    assert created[0].kwargs["disabled_toolsets"] == ["terminal", "file"]
    assert created[0].kwargs["max_iterations"] == 7
    assert created[0].kwargs["skip_memory"] is True
    assert created[0].run_conversation.call_count == 2
    subprocess_runner.assert_not_called()


def test_empty_enabled_toolsets_falls_back_to_voice_default(monkeypatch):
    from public_api_client.hermes_agent_client import DEFAULT_ENABLED_TOOLSETS

    created = []
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}):
        assert (
            hd.run_turn_in_process("hi", "chat-default", enabled_toolsets="") == "OK."
        )

    assert created[0].kwargs["enabled_toolsets"] == DEFAULT_ENABLED_TOOLSETS.split(",")
    assert "pib" in created[0].kwargs["enabled_toolsets"], (
        "the pib MCP server is named by its bare mcp_servers key: the mcp-<server> "
        "alias only registers after MCP discovery, so passing it prints an "
        "'Unknown toolsets' warning onto the reply"
    )


def test_cached_agent_is_built_with_the_shared_store_and_the_chat_session_id(
    monkeypatch,
):
    """The store is opened once per personality; session_id is bound per turn (PR-2052)."""
    store = _FakeSessionDB()
    creator = MagicMock(return_value=store)
    monkeypatch.setattr(hd, "_create_session_db", creator)
    created = []
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}):
        hd.run_turn_in_process("eins", "chat-store")
        hd.run_turn_in_process("zwei", "chat-store")
        hd.run_turn_in_process("drei", "chat-other-store")

    assert len(created) == 1
    assert created[0].kwargs["session_db"] is store
    assert created[0].kwargs["session_id"] is None
    assert created[0].kwargs["skip_memory"] is True
    assert creator.call_count == 1


def test_first_turn_has_no_history_and_the_second_replays_the_stored_one(monkeypatch):
    """The regression: a codeword stored in turn one must reach the agent in turn two."""
    store = _FakeSessionDB()
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    module = _fake_agent_module(created, store_turns=True)
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": module}):
        hd.run_turn_in_process("Mein Codewort ist BLAU11.", "chat-memory")
        hd.run_turn_in_process("Wie lautet mein Codewort?", "chat-memory")

    agent = created[0]
    assert agent.histories[0] is None
    assert agent.histories[1] == [
        {"role": "user", "content": "Mein Codewort ist BLAU11."},
        {"role": "assistant", "content": "OK."},
    ]
    # Both turns reopen the row the previous turn closed, or recording would stop.
    assert store.reopened == ["pib_chat_chat-memory", "pib_chat_chat-memory"]


def test_history_is_never_taken_from_another_chat_id(monkeypatch):
    store = _FakeSessionDB()
    store.append_turn("pib_chat_chat-other", "Mein Codewort ist GRUEN22.", "OK.")
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    module = _fake_agent_module(created, store_turns=True)
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": module}):
        hd.run_turn_in_process("Mein Codewort ist ROT33.", "chat-mine")
        hd.run_turn_in_process("Wie lautet mein Codewort?", "chat-mine")

    mine = created[0]
    assert mine.histories[0] is None
    assert [message["content"] for message in mine.histories[1]] == [
        "Mein Codewort ist ROT33.",
        "OK.",
    ]
    assert store.resume_calls == ["pib_chat_chat-mine", "pib_chat_chat-mine"]


def test_stored_session_meta_rows_are_not_replayed_as_conversation(monkeypatch):
    store = _FakeSessionDB()
    store.transcripts["pib_chat_chat-meta"] = [
        {"role": "session_meta", "content": "runtime"},
        {"role": "user", "content": "Hallo"},
    ]
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}):
        hd.run_turn_in_process("Und weiter", "chat-meta")

    assert created[0].histories == [[{"role": "user", "content": "Hallo"}]]


def test_turn_still_answers_when_the_session_store_is_unavailable(monkeypatch):
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=None))
    created = []
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}),
        patch(
            "public_api_client.hermes_agent_client.run_turn_subprocess"
        ) as subprocess_runner,
    ):
        assert hd.run_turn_in_process("eins", "chat-no-store") == "OK."
        assert hd.run_turn_in_process("zwei", "chat-no-store") == "OK."

    assert created[0].kwargs["session_db"] is None
    assert created[0].histories == [None, None]
    subprocess_runner.assert_not_called()


def test_turn_still_answers_when_loading_the_history_fails(monkeypatch):
    store = _FakeSessionDB()
    store.get_resume_conversations = MagicMock(side_effect=RuntimeError("db locked"))
    store.reopen_session = MagicMock(side_effect=RuntimeError("db locked"))
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}):
        assert hd.run_turn_in_process("eins", "chat-broken-store") == "OK."

    assert created[0].histories == [None]


def test_clearing_the_agent_cache_closes_the_session_store(monkeypatch):
    store = _FakeSessionDB()
    creator = MagicMock(return_value=store)
    monkeypatch.setattr(hd, "_create_session_db", creator)
    module = _fake_agent_module([])
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": module}):
        hd.run_turn_in_process("eins", "chat-close")
        hd.clear_agent_cache()
        assert store.closed == 1

        # A turn after the shutdown opens a fresh handle instead of the closed one.
        hd.run_turn_in_process("zwei", "chat-close")

    assert creator.call_count == 2


def test_mcp_discovery_is_attempted_once_before_reused_agent_turns(monkeypatch):
    discovery = MagicMock()
    monkeypatch.setattr(hd, "_discover_mcp_tools", discovery)

    fake_module = _fake_agent_module([], reply="ok")
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": fake_module}):
        assert hd.run_turn_in_process("one", "mcp-chat") == "ok"
        assert hd.run_turn_in_process("two", "mcp-chat") == "ok"

    discovery.assert_called_once_with()


def test_personality_homes_isolate_stores_discovery_agents_and_history(
    tmp_path, monkeypatch
):
    homes = {
        "one": str(tmp_path / "profiles" / "pib_one"),
        "two": str(tmp_path / "profiles" / "pib_two"),
    }
    stores = [_FakeSessionDB(), _FakeSessionDB()]
    stores[0].append_turn("pib_chat_shared", "secret from one", "OK.")
    creator = MagicMock(side_effect=stores)
    discovery = MagicMock()
    monkeypatch.setattr(hd, "_create_session_db", creator)
    monkeypatch.setattr(hd, "_discover_mcp_tools", discovery)
    created = []
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}),
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            side_effect=lambda personality_id: homes[personality_id],
        ),
    ):
        hd.run_turn_in_process("one", "shared", personality_id="one")
        hd.run_turn_in_process("two", "shared", personality_id="two")

    assert creator.call_count == 2
    assert discovery.call_count == 2
    assert len(created) == 2
    assert created[0].kwargs["session_db"] is stores[0]
    assert created[1].kwargs["session_db"] is stores[1]
    assert created[0].histories[0][0]["content"] == "secret from one"
    assert created[1].histories[0] is None


def test_personality_home_override_is_reset_after_success_and_exception(
    tmp_path, monkeypatch, fake_hermes_home_override
):
    from pib_hermes_config import profile_dir_for

    base_home = fake_hermes_home_override.get()
    profile_dir = profile_dir_for("one")
    observed = []

    def run_inside_scope(**_kwargs):
        observed.append(str(fake_hermes_home_override.get()))
        return "ok"

    with (
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            return_value=profile_dir,
        ),
        patch.object(hd, "_run_turn_in_home", side_effect=run_inside_scope),
    ):
        assert hd.run_turn_in_process("one", "chat", personality_id="one") == "ok"
    assert observed == [profile_dir]
    assert fake_hermes_home_override.get() == base_home

    with (
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            return_value=profile_dir,
        ),
        patch.object(hd, "_run_turn_in_home", side_effect=RuntimeError("turn failed")),
        pytest.raises(RuntimeError, match="turn failed"),
    ):
        hd.run_turn_in_process("one", "chat", personality_id="one")
    assert fake_hermes_home_override.get() == base_home


def _fake_mcp_startup_module(**startup_attrs):
    hermes_cli = types.ModuleType("hermes_cli")
    hermes_cli.__path__ = []
    startup = types.ModuleType("hermes_cli.mcp_startup")
    startup.ensure_mcp_discovery_before_agent_build = MagicMock()
    startup.mcp_discovery_in_flight = MagicMock(return_value=False)
    startup.join_mcp_discovery = MagicMock(return_value=True)
    for name, value in startup_attrs.items():
        setattr(startup, name, value)
    return {"hermes_cli": hermes_cli, "hermes_cli.mcp_startup": startup}, startup


def test_mcp_discovery_uses_hermes_noninteractive_startup_path():
    modules, startup = _fake_mcp_startup_module()

    with patch.dict(sys.modules, modules):
        _REAL_DISCOVER_MCP_TOOLS()

    startup.ensure_mcp_discovery_before_agent_build.assert_called_once()
    assert (
        startup.ensure_mcp_discovery_before_agent_build.call_args.kwargs["thread_name"]
        == "pib-hermes-daemon-mcp"
    )
    startup.join_mcp_discovery.assert_not_called()


def test_mcp_discovery_joins_when_still_in_flight_after_ensure():
    modules, startup = _fake_mcp_startup_module(
        mcp_discovery_in_flight=MagicMock(return_value=True),
    )

    with patch.dict(sys.modules, modules):
        _REAL_DISCOVER_MCP_TOOLS()

    startup.join_mcp_discovery.assert_called_once_with(
        timeout=hd._DAEMON_MCP_DISCOVERY_EXTRA_WAIT_S
    )


def test_agent_construction_reports_registered_mcp_tools_after_discovery(
    monkeypatch, caplog
):
    """PR-2059: the warm agent is built only after discovery registers tools."""
    discovery_state = {"done": False, "tools": 0}

    def _finish_discovery(**_kwargs):
        discovery_state["done"] = True
        discovery_state["tools"] = 20

    modules, _startup = _fake_mcp_startup_module(
        ensure_mcp_discovery_before_agent_build=MagicMock(
            side_effect=_finish_discovery
        ),
        mcp_discovery_in_flight=MagicMock(
            side_effect=lambda: not discovery_state["done"]
        ),
    )
    monkeypatch.setattr(hd, "_discover_mcp_tools", _REAL_DISCOVER_MCP_TOOLS)
    monkeypatch.setattr(
        hd,
        "_registered_mcp_tool_count",
        lambda: discovery_state["tools"],
    )

    fake_module = _fake_agent_module([], reply="ok")
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {**modules, "run_agent": fake_module}),
        caplog.at_level(logging.INFO),
    ):
        assert hd.run_turn_in_process("one", "mcp-ready-chat") == "ok"

    messages = [record.getMessage() for record in caplog.records]
    assert not any("constructing without MCP tools" in message for message in messages)
    assert any(
        "[PERF_TRACE] HERMES_AGENT_CONSTRUCTED" in message and "mcp_tools=20" in message
        for message in messages
    )


def test_agent_construction_waits_for_in_flight_discovery_to_finish(
    monkeypatch, caplog
):
    """PR-2059: a bounded extra join runs when Hermes' first wait times out."""
    discovery_state = {"in_flight": True, "tools": 0}

    def _join(timeout=None):
        discovery_state["in_flight"] = False
        discovery_state["tools"] = 20
        return True

    modules, startup = _fake_mcp_startup_module(
        mcp_discovery_in_flight=MagicMock(
            side_effect=lambda: discovery_state["in_flight"]
        ),
        join_mcp_discovery=MagicMock(side_effect=_join),
    )
    monkeypatch.setattr(hd, "_discover_mcp_tools", _REAL_DISCOVER_MCP_TOOLS)
    monkeypatch.setattr(
        hd,
        "_registered_mcp_tool_count",
        lambda: discovery_state["tools"],
    )

    fake_module = _fake_agent_module([], reply="ok")
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {**modules, "run_agent": fake_module}),
        caplog.at_level(logging.INFO),
    ):
        assert hd.run_turn_in_process("one", "mcp-join-chat") == "ok"

    startup.join_mcp_discovery.assert_called_once()
    messages = [record.getMessage() for record in caplog.records]
    assert any("mcp_tools=20" in message for message in messages)


def test_mcp_discovery_failure_logs_once_and_turn_continues(monkeypatch, caplog):
    """PR-2059: one explicit line when discovery cannot register tools; turns still run."""
    modules, _startup = _fake_mcp_startup_module(
        mcp_discovery_in_flight=MagicMock(return_value=True),
        join_mcp_discovery=MagicMock(return_value=False),
    )
    monkeypatch.setattr(hd, "_discover_mcp_tools", _REAL_DISCOVER_MCP_TOOLS)
    monkeypatch.setattr(hd, "_registered_mcp_tool_count", MagicMock(return_value=0))

    fake_module = _fake_agent_module([], reply="ok despite missing mcp")
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {**modules, "run_agent": fake_module}),
        caplog.at_level(logging.WARNING),
    ):
        assert (
            hd.run_turn_in_process("one", "mcp-fail-chat") == "ok despite missing mcp"
        )
        assert (
            hd.run_turn_in_process("two", "mcp-fail-chat") == "ok despite missing mcp"
        )

    failure_lines = [
        record.getMessage()
        for record in caplog.records
        if "constructing without MCP tools" in record.getMessage()
    ]
    assert len(failure_lines) == 1
    assert "still in flight" in failure_lines[0]


@pytest.mark.parametrize("failure", [RuntimeError("failed"), TimeoutError("timed out")])
def test_mcp_discovery_failure_is_non_fatal(monkeypatch, failure):
    monkeypatch.setattr(hd, "_discover_mcp_tools", MagicMock(side_effect=failure))

    fake_module = _fake_agent_module([], reply="reply despite discovery failure")
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": fake_module}):
        reply = hd.run_turn_in_process("one", "failed-mcp-chat")

    assert reply == "reply despite discovery failure"


def test_agent_construction_logs_duration_and_registered_mcp_tool_count(
    monkeypatch, caplog
):
    import logging

    monkeypatch.setattr(hd, "_registered_mcp_tool_count", MagicMock(return_value=11))

    fake_module = _fake_agent_module([], reply="ok")
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": fake_module}),
        caplog.at_level(logging.INFO),
    ):
        assert hd.run_turn_in_process("one", "logged-agent") == "ok"

    messages = [record.getMessage() for record in caplog.records]
    assert any(
        "[PERF_TRACE] HERMES_AGENT_CONSTRUCTED" in message and "mcp_tools=11" in message
        for message in messages
    )


def test_run_turn_in_process_reuses_one_agent_across_chat_ids(monkeypatch):
    """PR-2052: one warm agent per personality; session_id is bound per turn."""
    created = []

    class FakeAgent:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.session_id = kwargs.get("session_id")
            created.append(self)

        def reset_session_state(self, **kwargs):
            pass

        def run_conversation(
            self, user_message, conversation_history=None, stream_callback=None
        ):
            return {"final_response": self.session_id}

    fake_module = types.ModuleType("run_agent")
    fake_module.AIAgent = FakeAgent
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": fake_module}):
        first = hd.run_turn_in_process("one", "chat-a")
        second = hd.run_turn_in_process("two", "chat-b")
        again = hd.run_turn_in_process("three", "chat-a")

    assert len(created) == 1
    assert first == again == "pib_chat_chat-a"
    assert second == "pib_chat_chat-b"
    assert created[0].kwargs.get("session_id") is None


def test_two_chats_of_one_personality_construct_the_agent_once(monkeypatch, caplog):
    """PR-2052: the second chat must not pay agent construction again."""
    import logging

    store = _FakeSessionDB()
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    module = _fake_agent_module(created, store_turns=True, reply="Antwort")
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": module}),
        caplog.at_level(logging.INFO),
    ):
        hd.run_turn_in_process("Codewort BLAU.", "chat-one")
        hd.run_turn_in_process("Was war das Codewort?", "chat-one")
        hd.run_turn_in_process("Eine andere Frage.", "chat-two")

    assert len(created) == 1
    construction_logs = [
        m for m in caplog.records if "HERMES_AGENT_CONSTRUCTED" in m.getMessage()
    ]
    assert len(construction_logs) == 1

    agent = created[0]
    assert agent.histories[0] is None
    assert [m["content"] for m in agent.histories[1]] == [
        "Codewort BLAU.",
        "Antwort",
    ]
    assert agent.histories[2] is None
    assert store.resume_calls == [
        "pib_chat_chat-one",
        "pib_chat_chat-one",
        "pib_chat_chat-two",
    ]


def test_model_change_between_turns_rebuilds_the_cached_agent(monkeypatch):
    """Changing the profile model still evicts and reconstructs the warm agent."""
    from public_api_client.hermes_agent_client import DEFAULT_MAX_TURNS

    settings_reads = [
        (
            "gemini-a",
            "gemini",
            None,
            hd._GEMINI_KEY_NAMES,
        ),
        (
            "gemini-b",
            "gemini",
            None,
            hd._GEMINI_KEY_NAMES,
        ),
    ]

    class FakeAgent:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.model = kwargs.get("model")
            self.session_id = kwargs.get("session_id")
            created.append(self)

        def reset_session_state(self, **kwargs):
            pass

        def run_conversation(
            self, user_message, conversation_history=None, stream_callback=None
        ):
            return {"final_response": self.model}

    created = []
    fake_module = types.ModuleType("run_agent")
    fake_module.AIAgent = FakeAgent
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": fake_module}):
        with patch(
            "public_api_client.hermes_agent_client.profile_hermes_settings",
            return_value=settings_reads[0],
        ):
            first = hd.run_turn_in_process(
                "one", "chat-model", max_turns=DEFAULT_MAX_TURNS
            )
        with patch(
            "public_api_client.hermes_agent_client.profile_hermes_settings",
            return_value=settings_reads[1],
        ):
            second = hd.run_turn_in_process(
                "two", "chat-model", max_turns=DEFAULT_MAX_TURNS
            )

    assert first == "gemini-a"
    assert second == "gemini-b"
    assert len(created) == 2
    assert created[0].kwargs["model"] == "gemini-a"
    assert created[1].kwargs["model"] == "gemini-b"


def test_agent_cache_evicts_the_least_recently_used_profile(monkeypatch, tmp_path):
    """LRU is over personality/runtime keys, not individual chat ids (PR-2052)."""
    created = []
    homes = {
        "one": str(tmp_path / "pib_one"),
        "two": str(tmp_path / "pib_two"),
        "three": str(tmp_path / "pib_three"),
    }

    class FakeAgent:
        def __init__(self, **kwargs):
            self.close = MagicMock()
            created.append(self)

        def reset_session_state(self, **kwargs):
            pass

        def run_conversation(
            self, user_message, conversation_history=None, stream_callback=None
        ):
            return {"final_response": "ok"}

    fake_module = types.ModuleType("run_agent")
    fake_module.AIAgent = FakeAgent
    monkeypatch.setattr(hd, "DEFAULT_AGENT_CACHE_SIZE", 2)
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": fake_module}),
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            side_effect=lambda personality_id: homes[personality_id],
        ),
    ):
        hd.run_turn_in_process("one", "chat-a", personality_id="one")
        hd.run_turn_in_process("two", "chat-b", personality_id="two")
        hd.run_turn_in_process("three", "chat-a", personality_id="one")
        hd.run_turn_in_process("four", "chat-c", personality_id="three")

    assert len(hd._agent_cache) == 2
    cached_homes = [key[0] for key in hd._agent_cache]
    assert sum("pib_one" in home for home in cached_homes) == 1
    assert sum("pib_three" in home for home in cached_homes) == 1
    assert not any("pib_two" in home for home in cached_homes)
    created[1].close.assert_called_once()


def test_run_turn_in_process_keeps_streamed_text_when_budget_ends_empty():
    class BudgetAgent:
        def __init__(self, **_kwargs):
            pass

        def run_conversation(
            self, user_message, conversation_history=None, stream_callback=None
        ):
            stream_callback("partial answer")
            return {"final_response": ""}

    fake_module = types.ModuleType("run_agent")
    fake_module.AIAgent = BudgetAgent
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": fake_module}),
        patch(
            "public_api_client.hermes_agent_client.run_turn_subprocess"
        ) as subprocess_runner,
    ):
        reply = hd.run_turn_in_process("one", "budget-chat", max_turns=1)

    assert reply == "partial answer"
    subprocess_runner.assert_not_called()


def test_run_turn_in_process_falls_back_to_subprocess_when_import_fails():
    """Missing hermes.run_agent must fall back to the CLI subprocess path."""
    import builtins

    real_import = builtins.__import__

    def _block_hermes_run_agent(name, *args, **kwargs):
        if name in ("hermes", "hermes.run_agent", "run_agent"):
            raise ImportError("no hermes package")
        return real_import(name, *args, **kwargs)

    with (
        patch.dict(
            sys.modules,
            {"run_agent": None, "hermes.run_agent": None},
        ),
        patch("builtins.__import__", side_effect=_block_hermes_run_agent),
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            return_value="/tmp/profiles/pib_pers-1",
        ),
        patch(
            "public_api_client.hermes_agent_client.run_turn_subprocess",
            return_value="subprocess-reply",
        ) as subprocess_runner,
    ):
        reply = hd.run_turn_in_process(
            text="Hallo",
            chat_id="chat-7",
            personality_id="pers-1",
            toolsets="pib",
            enabled_toolsets="mcp-pib,vision",
            timeout=45,
        )

    assert reply == "subprocess-reply"
    subprocess_runner.assert_called_once_with(
        text="Hallo",
        chat_id="chat-7",
        personality_id="pers-1",
        toolsets="pib",
        enabled_toolsets="mcp-pib,vision",
        timeout=45,
        provider_key="fixture-hermes-key",
    )


def test_tools_used_reads_the_answered_tool_messages():
    """A tool message names the tool that answered; names only, no arguments."""
    result = {
        "final_response": "done",
        "messages": [
            {"role": "user", "content": "look"},
            {
                "role": "assistant",
                "tool_calls": [
                    {"function": {"name": "capture_image", "arguments": "{secret}"}}
                ],
            },
            {"role": "tool", "name": "capture_image", "content": "{}"},
            {"role": "assistant", "content": "fertig"},
        ],
    }

    assert hd._tools_used(result) == [{"name": "capture_image", "ok": True}]
    assert "secret" not in json.dumps(hd._tools_used(result))


def test_tools_used_falls_back_to_the_requested_calls_and_ignores_other_shapes():
    requested = {
        "messages": [
            {"role": "assistant", "tool_calls": [{"function": {"name": "set_led"}}]},
            {"role": "assistant", "tool_calls": [{"name": "play_sound"}]},
        ]
    }
    assert hd._tools_used(requested) == [
        {"name": "set_led", "ok": True},
        {"name": "play_sound", "ok": True},
    ]
    assert hd._tools_used(None) == []
    assert hd._tools_used({"messages": "nope"}) == []
    assert hd._tools_used("text") == []


def test_a_turn_returns_the_tools_it_used():
    """The daemon may return more than the text (decision PC-84)."""

    def runner(**kwargs):
        kwargs["on_tool_call"](
            {"name": "capture_image", "ok": True, "durationMs": 12.5}
        )
        return "gesehen"

    server = hd.create_server(port=0, turn_runner=runner)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address
    body = json.dumps({"text": "was siehst du?", "chat_id": "c-tool"}).encode()
    req = Request(
        f"http://{host}:{port}/turn",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(req, timeout=5) as resp:
            payload = json.loads(resp.read().decode())
    finally:
        server.shutdown()
        server.server_close()

    assert payload["reply"] == "gesehen"
    assert payload["toolsUsed"] == [
        {"name": "capture_image", "ok": True, "durationMs": 12.5}
    ]


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
            enabled_toolsets="mcp-pib,vision",
            max_turns=None,
            timeout=10,
            stream_callback=None,
        )

    assert reply == "from-default"
    in_process.assert_called_once_with(
        text="hi",
        chat_id="c1",
        personality_id="p1",
        on_tool_call=None,
        toolsets=None,
        enabled_toolsets="mcp-pib,vision",
        max_turns=None,
        timeout=10,
        stream_callback=None,
        message_history=None,
    )


def _install_fake_run_agent_main(monkeypatch, main_fn):
    """Make ``from run_agent import main`` resolve to main_fn, hiding hermes.*."""
    import builtins

    real_import = builtins.__import__
    module = types.ModuleType("run_agent")
    module.main = main_fn

    def _import(name, *args, **kwargs):
        if name in ("hermes", "hermes.run_agent"):
            raise ImportError("no hermes package")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", _import)
    monkeypatch.setitem(sys.modules, "run_agent", module)


def _make_hermes_venv(home, minor: int):
    """Create the hermes-agent venv layout of one Python minor version."""
    site_packages = (
        home / "hermes-agent" / "venv" / "lib" / f"python3.{minor}" / "site-packages"
    )
    site_packages.mkdir(parents=True)
    return site_packages


def _agent_venv_site_packages(root: Path, minor: int) -> Path:
    site_packages = root / "venv" / "lib" / f"python3.{minor}" / "site-packages"
    site_packages.mkdir(parents=True)
    return site_packages


def _pretend_python(monkeypatch, minor: int):
    monkeypatch.setattr(hd, "_running_python_version", lambda: (3, minor))


def test_venv_site_packages_ignores_a_different_minor_when_it_is_the_only_tree(
    tmp_path, monkeypatch, caplog
):
    """A 3.12 process must not receive a 3.13 tree (and the reverse)."""
    import logging

    agent = tmp_path / "agent"
    _agent_venv_site_packages(agent, 13)
    _pretend_python(monkeypatch, 12)

    with caplog.at_level(logging.INFO):
        assert hd.venv_site_packages(str(agent)) == []
    assert any("inserting none" in rec.getMessage() for rec in caplog.records)

    agent_other = tmp_path / "agent-other"
    _agent_venv_site_packages(agent_other, 12)
    _pretend_python(monkeypatch, 13)

    assert hd.venv_site_packages(str(agent_other)) == []


def test_venv_site_packages_returns_the_tree_matching_the_running_interpreter(
    tmp_path, monkeypatch, caplog
):
    import logging

    agent = tmp_path / "agent"
    matching_12 = _agent_venv_site_packages(agent, 12)
    _agent_venv_site_packages(agent, 13)
    _pretend_python(monkeypatch, 12)

    with caplog.at_level(logging.INFO):
        assert hd.venv_site_packages(str(agent)) == [str(matching_12)]
    assert any(str(matching_12) in rec.getMessage() for rec in caplog.records)

    matching_13 = agent / "venv" / "lib" / "python3.13" / "site-packages"
    _pretend_python(monkeypatch, 13)
    assert hd.venv_site_packages(str(agent)) == [str(matching_13)]


def test_venv_site_packages_finds_the_matching_python_version_without_pinning(
    sandboxed_hermes_home,
):
    """Search stays version-agnostic; only the running interpreter's tree is used."""
    running_minor = sys.version_info.minor
    other_minor = 13 if running_minor != 13 else 12
    matching = _make_hermes_venv(sandboxed_hermes_home, running_minor)
    _make_hermes_venv(sandboxed_hermes_home, other_minor)

    assert hd.venv_site_packages() == [str(matching)]


def test_venv_site_packages_is_empty_when_the_layout_is_unexpected(
    sandboxed_hermes_home,
):
    """A missing or differently shaped install must not raise, just yield nothing."""
    assert hd.venv_site_packages() == []
    assert hd.venv_site_packages(str(sandboxed_hermes_home / "nope")) == []


def test_daemon_does_not_pin_a_python_minor_version_in_its_sys_path():
    """Guards the regression: the pinned python3.11 site-packages was dead on 3.13."""
    source = Path(hd.__file__).read_text(encoding="utf-8")

    assert not re.search(r"python3\.\d+[/\\]site-packages", source)


def test_run_turn_in_process_adds_the_detected_site_packages_to_sys_path(
    monkeypatch, sandboxed_hermes_home
):
    site_packages = _make_hermes_venv(sandboxed_hermes_home, sys.version_info.minor)
    monkeypatch.setattr(sys, "path", list(sys.path))

    def fake_main(query=None, model="", **kwargs):
        print("\U0001f3af FINAL RESPONSE:\n---\nAntwort")

    _install_fake_run_agent_main(monkeypatch, fake_main)

    assert hd.run_turn_in_process(text="hi", chat_id="c-1") == "Antwort"
    assert str(site_packages) in sys.path
    assert str(sandboxed_hermes_home / "hermes-agent") in sys.path


def test_run_turn_in_process_never_prepends_a_mismatched_venv_tree(
    monkeypatch, sandboxed_hermes_home
):
    """Both directions: a 3.12 process must not get 3.13 on sys.path, and vice versa."""
    mismatched_13 = _make_hermes_venv(sandboxed_hermes_home, 13)
    monkeypatch.setattr(sys, "path", list(sys.path))
    _pretend_python(monkeypatch, 12)

    fake_module = _fake_agent_module([], reply="ok")
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": fake_module}):
        assert hd.run_turn_in_process("hi", "c-mismatch-12") == "ok"

    assert str(mismatched_13) not in sys.path
    assert hd.venv_site_packages() == []

    mismatched_12 = _make_hermes_venv(sandboxed_hermes_home, 12)
    py313 = mismatched_13.parent
    mismatched_13.rmdir()
    py313.rmdir()
    _pretend_python(monkeypatch, 13)
    monkeypatch.setattr(sys, "path", list(sys.path))
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": fake_module}):
        assert hd.run_turn_in_process("hi", "c-mismatch-13") == "ok"

    assert str(mismatched_12) not in sys.path
    assert hd.venv_site_packages() == []


def test_extract_final_response_strips_decoration_and_trailing_output():
    stdout = (
        "\U0001f916 AI Agent with Tool Calling\n"
        "==================================================\n"
        "\U0001f4dd User Query: Wie geht es dir?\n"
        "\n\U0001f3af FINAL RESPONSE:\n"
        "------------------------------\n"
        "Mir geht es gut!\n"
        "Und dir?\n"
        "\n"
        "\U0001f44b Agent execution completed!\n"
    )

    assert hd.extract_final_response(stdout) == "Mir geht es gut!\nUnd dir?"


def test_extract_final_response_without_marker_is_empty():
    assert hd.extract_final_response("no banner here\n") == ""
    assert hd.extract_final_response("") == ""


def test_extract_final_response_ignores_earlier_banner_occurrence():
    stdout = (
        "\U0001f3af FINAL RESPONSE:\n----\nalte Antwort\n"
        "\U0001f44b Agent execution completed!\n"
        "\U0001f3af FINAL RESPONSE:\n----\nneue Antwort\n"
    )

    assert hd.extract_final_response(stdout) == "neue Antwort"


def test_run_turn_in_process_reads_reply_from_run_agent_stdout(tmp_path, monkeypatch):
    """run_agent.main prints its answer, so the daemon must parse stdout."""
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    calls = {}

    def fake_main(query=None, model="", **kwargs):
        calls["query"] = query
        calls["model"] = model
        calls.update(kwargs)
        print("\U0001f916 AI Agent with Tool Calling")
        print("\n\U0001f3af FINAL RESPONSE:")
        print("-" * 30)
        print("Hallo, mir geht es gut!")
        print("\n\U0001f44b Agent execution completed!")

    _install_fake_run_agent_main(monkeypatch, fake_main)

    with (
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            return_value=str(tmp_path / "profiles" / "pib_pers-1"),
        ),
        patch(
            "public_api_client.hermes_agent_client.run_turn_subprocess",
        ) as subprocess_runner,
    ):
        reply = hd.run_turn_in_process(
            text="Wie geht es dir?",
            chat_id="chat-1",
            personality_id="pers-1",
            timeout=30,
        )

    assert reply == "Hallo, mir geht es gut!"
    assert calls == {
        "query": "Wie geht es dir?",
        "model": hd.IN_PROCESS_MODEL,
        "enabled_toolsets": "pib,vision",
        "disabled_toolsets": "terminal,code_execution,file,memory,session_search",
        "max_turns": 4,
    }
    subprocess_runner.assert_not_called()


def test_run_turn_in_process_restores_stdout_after_capture(monkeypatch, capsys):
    def fake_main(query=None, model="", **kwargs):
        print("\U0001f3af FINAL RESPONSE:\n---\nAntwort")

    _install_fake_run_agent_main(monkeypatch, fake_main)

    reply = hd.run_turn_in_process(text="hi", chat_id="c-1")

    assert reply == "Antwort"
    assert sys.stdout is not None
    assert "FINAL RESPONSE" not in capsys.readouterr().out


def test_run_turn_in_process_falls_back_when_stdout_has_no_final_response(monkeypatch):
    def fake_main(query=None, model="", **kwargs):
        print("\U0001f916 AI Agent with Tool Calling")
        print("\u274c Failed to initialize agent: boom")

    _install_fake_run_agent_main(monkeypatch, fake_main)

    with patch(
        "public_api_client.hermes_agent_client.run_turn_subprocess",
        return_value="subprocess-reply",
    ) as subprocess_runner:
        reply = hd.run_turn_in_process(text="Hallo", chat_id="chat-3", timeout=45)

    assert reply == "subprocess-reply"
    subprocess_runner.assert_called_once_with(
        text="Hallo",
        chat_id="chat-3",
        personality_id=None,
        toolsets="terminal,code_execution,file,memory,session_search",
        enabled_toolsets="pib,vision",
        timeout=45,
        provider_key="fixture-hermes-key",
    )


def test_run_turn_in_process_uses_fallback_reply_when_subprocess_also_empty(
    monkeypatch,
):
    from public_api_client.hermes_agent_client import FALLBACK_REPLY

    def fake_main(query=None, model="", **kwargs):
        print("nothing useful")

    _install_fake_run_agent_main(monkeypatch, fake_main)

    with patch(
        "public_api_client.hermes_agent_client.run_turn_subprocess",
        return_value="",
    ):
        reply = hd.run_turn_in_process(text="Hallo", chat_id="chat-4")

    assert reply == FALLBACK_REPLY


def test_run_turn_in_process_returns_fallback_on_agent_error(tmp_path, monkeypatch):
    from public_api_client.hermes_agent_client import FALLBACK_REPLY

    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))

    class BrokenAgent:
        def __init__(self, **_kwargs):
            raise RuntimeError("boom")

    fake_module = types.ModuleType("run_agent")
    fake_module.AIAgent = BrokenAgent
    hd.clear_agent_cache()

    with (
        patch.dict(
            sys.modules,
            {"run_agent": fake_module},
        ),
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            return_value="/tmp/p",
        ),
    ):
        reply = hd.run_turn_in_process(
            text="Hi",
            chat_id="c",
            personality_id="pers-1",
        )

    assert reply == FALLBACK_REPLY


def test_hermes_source_is_placed_in_front_of_sys_path(tmp_path, monkeypatch):
    """Provisioning and turns must import the SAME Hermes.

    Live, provisioning imported hermes_cli from the container's site-packages first;
    the turn then failed to import run_agent and silently answered with the fallback
    sentence, because the CLI subprocess is broken in that container.
    """
    agent_dir = tmp_path / "hermes-agent"
    venv = agent_dir / "venv" / "lib" / "python3.12" / "site-packages"
    venv.mkdir(parents=True)
    monkeypatch.setattr(hd, "hermes_agent_dir", lambda: str(agent_dir))
    monkeypatch.setattr(hd, "venv_site_packages", lambda _base: [str(venv)])
    monkeypatch.setattr(hd, "_HERMES_SOURCE_PREPARED", False)
    monkeypatch.setattr(hd.sys, "path", ["/somewhere/else"])

    hd.ensure_hermes_source_on_path()
    hd.ensure_hermes_source_on_path()  # idempotent

    assert hd.sys.path[0] == str(agent_dir)
    assert hd.sys.path.count(str(agent_dir)) == 1
    assert hd.sys.path.count(str(venv)) == 1


def test_foreign_hermes_modules_are_dropped_so_the_mounted_source_wins(
    tmp_path, monkeypatch
):
    """A module imported before the path was prepared must not shadow the mounted source.

    Live, provisioning imported ``hermes_cli`` from the mounted tree while
    ``hermes_constants`` stayed cached from the container's older ``hermes-agent``. The
    version mix raised ``ImportError: cannot import name 'LOCAL_RUNTIME_ROOT_DIRS' from
    'hermes_constants'``, so no profile was provisioned and every newly created
    personality answered with the fallback sentence.
    """
    agent_dir = tmp_path / "hermes-agent"
    agent_dir.mkdir()
    (agent_dir / "hermes_constants.py").write_text(
        "LOCAL_RUNTIME_ROOT_DIRS = ('profiles',)\n"
    )
    foreign_dir = tmp_path / "dist-packages"
    foreign_dir.mkdir()
    (foreign_dir / "hermes_constants.py").write_text("OLD_VERSION = True\n")

    monkeypatch.setattr(hd, "hermes_agent_dir", lambda: str(agent_dir))
    monkeypatch.setattr(hd, "venv_site_packages", lambda _base: [])
    monkeypatch.setattr(hd, "_HERMES_SOURCE_PREPARED", False)
    monkeypatch.syspath_prepend(str(foreign_dir))
    monkeypatch.delitem(sys.modules, "hermes_constants", raising=False)
    try:
        foreign = importlib.import_module("hermes_constants")
        assert str(foreign_dir) in str(foreign.__file__ or "")
        assert not hasattr(foreign, "LOCAL_RUNTIME_ROOT_DIRS")

        hd.ensure_hermes_source_on_path()

        reimported = importlib.import_module("hermes_constants")
        assert str(reimported.__file__ or "").startswith(str(agent_dir))
        assert reimported.LOCAL_RUNTIME_ROOT_DIRS == ("profiles",)
    finally:
        sys.modules.pop("hermes_constants", None)


def test_provisioning_prepares_the_hermes_source_before_importing_the_factory(
    tmp_path, monkeypatch
):
    """The factory import must not pick a foreign hermes_cli into sys.modules."""
    prepared = MagicMock()
    monkeypatch.setattr(hd, "ensure_hermes_source_on_path", prepared)
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    def create_profile(**kwargs):
        pdir = tmp_path / kwargs["name"]
        pdir.mkdir(parents=True)
        for dirname in hd.PROFILE_DIRS:
            (pdir / dirname).mkdir()
        (pdir / "config.yaml").write_text("{}\n", encoding="utf-8")
        return pdir

    profiles_module = types.ModuleType("hermes_cli.profiles")
    profiles_module.create_profile = create_profile
    profiles_module.profile_exists = lambda _name: False
    package = types.ModuleType("hermes_cli")
    package.__path__ = []

    with patch.dict(
        sys.modules, {"hermes_cli": package, "hermes_cli.profiles": profiles_module}
    ):
        result = hd.ensure_profile_home("p-mixed")

    assert prepared.called
    assert result["ok"] is True
    assert result["created"] is True
    assert os.path.isdir(os.path.join(result["profile_dir"], "memories"))


def test_run_turn_in_process_prepares_the_hermes_source(monkeypatch):
    prepared = MagicMock()
    monkeypatch.setattr(hd, "ensure_hermes_source_on_path", prepared)
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=None))
    monkeypatch.setattr(hd, "_discover_mcp_tools", MagicMock())
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": _fake_agent_module([])}):
        hd.run_turn_in_process("hi", "chat-mixed")

    assert prepared.called


def test_in_process_available_follows_the_venv_match(monkeypatch):
    """The predicate is exactly "Hermes' venv has site-packages for this Python"."""
    monkeypatch.setattr(hd, "in_process_available", _REAL_IN_PROCESS_AVAILABLE)
    monkeypatch.setattr(hd, "venv_site_packages", lambda *args, **kwargs: [])
    assert hd.in_process_available() is False

    monkeypatch.setattr(
        hd, "venv_site_packages", lambda *args, **kwargs: ["/hermes/venv/site-packages"]
    )
    assert hd.in_process_available() is True


def test_no_matching_hermes_venv_uses_the_cli_subprocess(monkeypatch):
    """Without a Hermes venv for this interpreter the agent is never imported.

    Importing the agent source under a mismatched interpreter makes Hermes'
    bootstrap re-exec THIS process -- the ROS ``chat`` node -- with its managed
    Python, where the node's setuptools launcher cannot resolve the
    ``voice-assistant`` distribution, so the node exits and every chat is lost
    until the container restarts. The CLI subprocess is the supported path there.
    """
    monkeypatch.setattr(hd, "in_process_available", lambda: False)
    source_prepared = MagicMock()
    monkeypatch.setattr(hd, "ensure_hermes_source_on_path", source_prepared)
    subprocess_reply = MagicMock(return_value="  from-the-cli  ")
    hd.clear_agent_cache()

    agent_module = _fake_agent_module([])
    with (
        patch.dict(sys.modules, {"run_agent": agent_module}),
        patch(
            "public_api_client.hermes_agent_client.run_turn_subprocess",
            subprocess_reply,
        ),
    ):
        reply = hd.run_turn_in_process("hi", "chat-no-venv")

    subprocess_reply.assert_called_once()
    assert agent_module.AIAgent.__name__ == "FakeAgent"  # the stub was never reached
    assert not source_prepared.called, "the agent source must not be put on sys.path"
    assert reply


STORE_SECRET = "store-hermes-key"
ENV_SECRET = "env-hermes-key"
OTHER_SECRET = "other-hermes-key"


def _visible(caplog, capsys) -> str:
    captured = capsys.readouterr()
    return caplog.text + captured.out + captured.err


def _install_real_provider_key(monkeypatch, fetch) -> None:
    """Use the production store read. The suite stand-in is replaced."""
    voice = Path(__file__).resolve().parents[2] / "ros_packages" / "voice_assistant"
    if str(voice) not in sys.path:
        sys.path.insert(0, str(voice))
    from public_api_client import hermes_agent_client as client
    from voice_assistant import direct_tool_loop

    if not hasattr(direct_tool_loop, "resolve_hermes_provider_key"):
        pytest.fail("resolve_hermes_provider_key is missing")
    monkeypatch.setattr(direct_tool_loop, "_fetch_store_key", fetch)

    def real() -> str:
        key, _source = direct_tool_loop.resolve_hermes_provider_key(
            client.DEFAULT_HERMES_PROVIDER
        )
        return key

    monkeypatch.setattr(client, "provider_key_for_turn", real)


def test_unlocked_store_supplies_the_hermes_key(monkeypatch, caplog, capsys, tmp_path):
    """The agent is built with the store key. The profile does not receive a copy."""
    monkeypatch.setenv("GOOGLE_API_KEY", ENV_SECRET)
    monkeypatch.setenv("OPENAI_API_KEY", OTHER_SECRET)
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    caplog.set_level(logging.INFO)
    profile = Path(os.environ["PIB_HERMES_PROFILES_DIR"]) / "pib_pers-key"
    profile.mkdir(parents=True)
    (profile / ".env").write_text("# kept\n", encoding="utf-8")
    (profile / "config.yaml").write_text(
        "model: gemini-3.8-flash\nprovider: gemini\n", encoding="utf-8"
    )
    from public_api_client.hermes_agent_client import _ensure_mcp_servers_pib

    _ensure_mcp_servers_pib(str(profile), model="gemini-3.8-flash", provider="gemini")
    before = {
        path.name: path.read_bytes() for path in profile.iterdir() if path.is_file()
    }

    def fetch(provider):
        assert provider == "gemini"
        return {"mode": "unlocked", "secret": STORE_SECRET}

    _install_real_provider_key(monkeypatch, fetch)
    created = []
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}),
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            return_value=str(profile),
        ),
        patch(
            "public_api_client.hermes_agent_client.run_turn_subprocess"
        ) as subprocess_runner,
    ):
        reply = hd.run_turn_in_process(
            text="Hallo",
            chat_id="chat-key",
            personality_id="pers-key",
        )

    assert reply == "OK."
    assert created[0].kwargs["api_key"] == STORE_SECRET
    assert created[0].kwargs["provider"] == "gemini"
    assert created[0].kwargs["api_key"] != ENV_SECRET
    assert os.environ.get("GOOGLE_API_KEY") == ENV_SECRET
    assert "GEMINI_API_KEY" not in os.environ
    assert "hermes provider key source=key-store" in caplog.text
    assert "provider=gemini" in caplog.text
    after = {
        path.name: path.read_bytes() for path in profile.iterdir() if path.is_file()
    }
    assert after == before
    visible = _visible(caplog, capsys)
    profile_text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in profile.rglob("*")
        if path.is_file()
    )
    assert STORE_SECRET not in visible
    assert STORE_SECRET not in profile_text
    assert ENV_SECRET not in visible
    assert OTHER_SECRET not in visible
    subprocess_runner.assert_not_called()


@pytest.mark.parametrize(
    "state",
    [
        {"mode": "degraded", "secret": STORE_SECRET},
        {"mode": "unavailable", "secret": STORE_SECRET},
        {"mode": "unlocked", "secret": None},
    ],
)
def test_hermes_turn_fails_when_the_store_cannot_supply_the_key(
    monkeypatch, caplog, capsys, state
):
    """No environment key and no other provider. The message names gemini only."""
    monkeypatch.setenv("GOOGLE_API_KEY", ENV_SECRET)
    monkeypatch.setenv("OPENAI_API_KEY", OTHER_SECRET)
    caplog.set_level(logging.INFO)

    def fetch(provider):
        assert provider == "gemini"
        return state

    _install_real_provider_key(monkeypatch, fetch)
    created = []
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}),
        patch(
            "public_api_client.hermes_agent_client.run_turn_subprocess"
        ) as subprocess_runner,
        pytest.raises(Exception) as caught,
    ):
        hd.run_turn_in_process(text="Hallo", chat_id="chat-locked")

    message = str(caught.value)
    assert message == "No keys are available for provider gemini."
    assert type(caught.value).__name__ == "DirectToolLoopError"
    assert created == []
    subprocess_runner.assert_not_called()
    assert os.environ.get("GOOGLE_API_KEY") == ENV_SECRET
    visible = _visible(caplog, capsys) + message
    assert STORE_SECRET not in visible
    assert ENV_SECRET not in visible
    assert OTHER_SECRET not in visible


def test_agent_source_imports_here_probes_a_real_agent_tree(tmp_path, monkeypatch):
    """A PM-managed install has no venv/lib inside the agent dir: the import decides."""
    from public_api_client import hermes_daemon

    (tmp_path / "hermes_constants.py").write_text("VALUE = 1\n")
    (tmp_path / "hermes_bootstrap.py").write_text("READY = True\n")
    monkeypatch.setattr(hermes_daemon, "_AGENT_IMPORT_PROBE", None)

    assert hermes_daemon.agent_source_imports_here(str(tmp_path)) is True


def test_agent_source_imports_here_reports_false_when_the_import_fails(
    tmp_path, monkeypatch
):
    from public_api_client import hermes_daemon

    monkeypatch.setattr(hermes_daemon, "_AGENT_IMPORT_PROBE", None)
    # An empty directory: the probe cannot import hermes_constants.
    assert hermes_daemon.agent_source_imports_here(str(tmp_path)) is False


def test_agent_source_imports_here_caches_the_answer(tmp_path, monkeypatch):
    from public_api_client import hermes_daemon

    calls = []

    def fake_run(*args, **kwargs):
        calls.append(args)
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(hermes_daemon, "_AGENT_IMPORT_PROBE", None)
    monkeypatch.setattr(hermes_daemon.subprocess, "run", fake_run)

    assert hermes_daemon.agent_source_imports_here(str(tmp_path)) is True
    assert hermes_daemon.agent_source_imports_here(str(tmp_path)) is True
    assert len(calls) == 1


def test_second_turn_does_not_attempt_a_refused_stream_options_field(
    tmp_path, monkeypatch
):
    """After one refusal is recorded, the next turn must not plan stream_options."""
    from public_api_client.hermes_agent_client import (
        clear_provider_refused_fields_cache,
    )

    clear_provider_refused_fields_cache()
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    created = []

    class FakeAgent:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self._stream_options_unsupported = False

        def run_conversation(
            self, user_message, conversation_history=None, stream_callback=None
        ):
            self._stream_options_unsupported = True
            return {"final_response": "OK."}

    fake_module = types.ModuleType("run_agent")
    fake_module.AIAgent = FakeAgent
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": fake_module}),
        patch(
            "public_api_client.hermes_agent_client.ensure_profile",
            return_value=str(tmp_path / "profiles" / "pib_pers-1"),
        ),
        patch(
            "public_api_client.hermes_agent_client.profile_hermes_settings",
            return_value=(
                "pib-cloud",
                "pib-cloud",
                "https://api.pib.rocks/v1",
                ("OPENAI_API_KEY",),
            ),
        ),
        patch(
            "public_api_client.hermes_agent_client.profile_tool_calling_enabled",
            return_value=True,
        ),
    ):
        hd.run_turn_in_process("Hallo", "chat-a", personality_id="pers-1")
        hd._provider_field_attempts.clear()
        hd.run_turn_in_process("Nochmal", "chat-a", personality_id="pers-1")

    assert "stream_options" not in hd._provider_field_attempts[-1]
    clear_provider_refused_fields_cache()
    hd.clear_agent_cache()


def test_run_turn_repairs_stale_profile_config_before_the_model(tmp_path, monkeypatch):
    """PR-2071: an old config_version is repaired on the daemon path without a model call."""
    from pib_hermes_config import PROFILE_CONFIG_VERSION, PROFILE_CONFIG_VERSION_KEY
    from pib_hermes_config import profile_dir_for

    import yaml

    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    profile_dir = Path(profile_dir_for("stale"))
    profile_dir.mkdir(parents=True)
    for dirname in hd.PROFILE_DIRS:
        (profile_dir / dirname).mkdir()
    (profile_dir / ".env").write_text("# secrets\n", encoding="utf-8")
    (profile_dir / "config.yaml").write_text(
        yaml.safe_dump(
            {
                "model": "gemini-3.8-flash",
                "provider": "gemini",
                PROFILE_CONFIG_VERSION_KEY: PROFILE_CONFIG_VERSION - 1,
            }
        ),
        encoding="utf-8",
    )
    (profile_dir / "SOUL.md").write_text("Du bist pib.\n", encoding="utf-8")

    def read_version():
        with open(profile_dir / "config.yaml", encoding="utf-8") as fh:
            return yaml.safe_load(fh)[PROFILE_CONFIG_VERSION_KEY]

    fake_module = _fake_agent_module([])
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": fake_module}),
        patch(
            "public_api_client.hermes_agent_client.profile_hermes_settings",
            return_value=("gemini-3.8-flash", "gemini", None, ()),
        ),
        patch(
            "public_api_client.hermes_agent_client.profile_tool_calling_enabled",
            return_value=True,
        ),
        patch(
            "public_api_client.hermes_agent_client.provider_key_for_turn",
            return_value="",
        ),
    ):
        assert read_version() == PROFILE_CONFIG_VERSION - 1
        hd.run_turn_in_process("Hallo", "chat-stale", personality_id="stale")
        assert read_version() == PROFILE_CONFIG_VERSION
        with patch("yaml.safe_dump") as dump:
            hd.run_turn_in_process("Nochmal", "chat-stale", personality_id="stale")
            dump.assert_not_called()


def test_warm_turn_skips_profile_writer_when_config_version_is_current(
    tmp_path, monkeypatch
):
    from pib_hermes_config import PROFILE_CONFIG_VERSION, PROFILE_CONFIG_VERSION_KEY
    from pib_hermes_config import profile_dir_for

    import yaml

    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    profile_dir = Path(profile_dir_for("warm"))
    profile_dir.mkdir(parents=True)
    for dirname in hd.PROFILE_DIRS:
        (profile_dir / dirname).mkdir()
    (profile_dir / ".env").write_text("# secrets\n", encoding="utf-8")
    (profile_dir / "SOUL.md").write_text("Du bist pib.\n", encoding="utf-8")

    from public_api_client.hermes_agent_client import _ensure_mcp_servers_pib

    _ensure_mcp_servers_pib(
        str(profile_dir), model="gemini-3.8-flash", provider="gemini"
    )
    with open(profile_dir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert cfg[PROFILE_CONFIG_VERSION_KEY] == PROFILE_CONFIG_VERSION

    ensure_calls = {"count": 0}
    real_ensure = _ensure_mcp_servers_pib

    def counting_ensure(*args, **kwargs):
        ensure_calls["count"] += 1
        return real_ensure(*args, **kwargs)

    fake_module = _fake_agent_module([])
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": fake_module}),
        patch(
            "public_api_client.hermes_agent_client._ensure_mcp_servers_pib",
            side_effect=counting_ensure,
        ),
        patch(
            "public_api_client.hermes_agent_client.profile_hermes_settings",
            return_value=("gemini-3.8-flash", "gemini", None, ()),
        ),
        patch(
            "public_api_client.hermes_agent_client.profile_tool_calling_enabled",
            return_value=True,
        ),
        patch(
            "public_api_client.hermes_agent_client.provider_key_for_turn",
            return_value="",
        ),
    ):
        hd.run_turn_in_process("warm", "chat-warm", personality_id="warm")
        hd.run_turn_in_process("again", "chat-warm", personality_id="warm")

    assert ensure_calls["count"] == 0


def _reset_prewarm_state():
    hd._prewarm_completed = False


def test_prewarm_skipped_when_smart_chats_disabled(monkeypatch, caplog):
    _reset_prewarm_state()
    monkeypatch.setenv("PIB_SMART_CHATS", "0")
    warmed = MagicMock()
    monkeypatch.setattr(hd, "_construct_warm_agent_for_personality", warmed)

    assert hd.prewarm_default_agent_at_start() is True
    assert hd.prewarm_default_agent_at_start() is False
    warmed.assert_not_called()
    assert any("smart chats disabled" in rec.getMessage() for rec in caplog.records)


def test_prewarm_warms_explicit_personality_once(monkeypatch):
    _reset_prewarm_state()
    monkeypatch.delenv("PIB_SMART_CHATS", raising=False)
    monkeypatch.setenv(hd.PREWARM_PERSONALITY_ENV, "pers-warm")
    construct = MagicMock()
    monkeypatch.setattr(hd, "_construct_warm_agent_for_personality", construct)

    assert hd.prewarm_default_agent_at_start() is True
    assert hd.prewarm_default_agent_at_start() is False
    construct.assert_called_once_with("pers-warm")


def test_prewarm_skips_direct_only_personality_list(monkeypatch, caplog):
    _reset_prewarm_state()
    monkeypatch.delenv("PIB_SMART_CHATS", raising=False)
    monkeypatch.delenv(hd.PREWARM_PERSONALITY_ENV, raising=False)
    construct = MagicMock()
    monkeypatch.setattr(hd, "_construct_warm_agent_for_personality", construct)

    def personalities():
        return [{"personalityId": "p1", "channel": "direct"}]

    hd.prewarm_default_agent_at_start(fetch_personalities=personalities)
    construct.assert_not_called()
    assert any("no smart personality" in rec.getMessage() for rec in caplog.records)
