"""Unit tests for the long-lived Hermes daemon HTTP service (PR-1535)."""

from __future__ import annotations

import json
import re
import sys
import threading
import time
import types
from pathlib import Path
from unittest.mock import MagicMock, patch
from urllib.request import Request, urlopen

import pytest

from public_api_client import hermes_daemon as hd

_REAL_DISCOVER_MCP_TOOLS = hd._discover_mcp_tools


@pytest.fixture(autouse=True)
def isolate_mcp_discovery(monkeypatch):
    """Keep daemon unit tests independent of the developer's Hermes config and state.db."""
    monkeypatch.setattr(hd, "_mcp_discovery_attempted", False)
    monkeypatch.setattr(hd, "_discover_mcp_tools", MagicMock())
    monkeypatch.setattr(hd, "_registered_mcp_tool_count", MagicMock(return_value=0))
    monkeypatch.setattr(hd, "_session_db", None)
    monkeypatch.setattr(hd, "_session_db_attempted", False)
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=None))


class _FakeSessionDB:
    """Stand-in for ``hermes_state.SessionDB`` — no SQLite, one transcript per session."""

    def __init__(self):
        self.transcripts: dict[str, list[dict]] = {}
        self.resume_calls: list[str] = []
        self.reopened: list[str] = []
        self.closed = 0

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

    def close(self):
        self.closed += 1


def _fake_agent_module(created, *, store_turns=False, reply="OK."):
    """A ``run_agent`` module whose AIAgent records the history each turn receives.

    ``store_turns`` additionally appends the turn to the store it was constructed with,
    the way a real AIAgent with a ``session_db`` persists its messages.
    """

    class FakeAgent:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.histories = []
            created.append(self)

        def run_conversation(
            self, user_message, conversation_history=None, stream_callback=None
        ):
            self.histories.append(conversation_history)
            store = self.kwargs.get("session_db")
            if store_turns and store is not None:
                store.append_turn(self.kwargs["session_id"], user_message, reply)
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
        max_turns=None,
        timeout=None,
        stream_callback=None,
    ):
        replies["last"] = {
            "text": text,
            "chat_id": chat_id,
            "personality_id": personality_id,
            "toolsets": toolsets,
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
    assert chunks[-1] == {"reply": "daemon-says-hi"}


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
            self.run_conversation = MagicMock(
                return_value={"final_response": "  in-process-reply  "}
            )
            created.append(self)

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
            max_turns=7,
            timeout=30,
        )
        second = hd.run_turn_in_process(
            text="Noch einmal",
            chat_id="chat-42",
            toolsets="terminal,file",
            max_turns=7,
        )

    assert first == second == "in-process-reply"
    assert len(created) == 1
    assert created[0].kwargs["session_id"] == "pib_chat_chat-42"
    assert created[0].kwargs["enabled_toolsets"] is None
    assert created[0].kwargs["disabled_toolsets"] == ["terminal", "file"]
    assert created[0].kwargs["max_iterations"] == 7
    assert created[0].kwargs["skip_memory"] is True
    assert created[0].run_conversation.call_count == 2
    subprocess_runner.assert_not_called()


def test_cached_agent_is_built_with_the_shared_store_and_the_chat_session_id(
    monkeypatch,
):
    """The store is opened once per process and every chat agent gets that handle."""
    store = _FakeSessionDB()
    creator = MagicMock(return_value=store)
    monkeypatch.setattr(hd, "_create_session_db", creator)
    created = []
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": _fake_agent_module(created)}):
        hd.run_turn_in_process("eins", "chat-store")
        hd.run_turn_in_process("zwei", "chat-store")
        hd.run_turn_in_process("drei", "chat-other-store")

    assert [agent.kwargs["session_db"] for agent in created] == [store, store]
    assert [agent.kwargs["session_id"] for agent in created] == [
        "pib_chat_chat-store",
        "pib_chat_chat-other-store",
    ]
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


def test_mcp_discovery_uses_hermes_noninteractive_startup_path():
    hermes_cli = types.ModuleType("hermes_cli")
    hermes_cli.__path__ = []
    startup = types.ModuleType("hermes_cli.mcp_startup")
    startup.ensure_mcp_discovery_before_agent_build = MagicMock()
    startup.mcp_discovery_in_flight = MagicMock(return_value=False)

    with patch.dict(
        sys.modules,
        {
            "hermes_cli": hermes_cli,
            "hermes_cli.mcp_startup": startup,
        },
    ):
        _REAL_DISCOVER_MCP_TOOLS()

    startup.ensure_mcp_discovery_before_agent_build.assert_called_once()
    assert (
        startup.ensure_mcp_discovery_before_agent_build.call_args.kwargs["thread_name"]
        == "pib-hermes-daemon-mcp"
    )


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


def test_run_turn_in_process_never_shares_agents_between_chat_ids(monkeypatch):
    created = []

    class FakeAgent:
        def __init__(self, **kwargs):
            self.session_id = kwargs["session_id"]
            created.append(self)

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

    assert len(created) == 2
    assert first == again == "pib_chat_chat-a"
    assert second == "pib_chat_chat-b"
    assert created[0] is not created[1]


def test_agent_cache_evicts_the_least_recently_used_chat(monkeypatch):
    created = []

    class FakeAgent:
        def __init__(self, **kwargs):
            self.session_id = kwargs["session_id"]
            self.close = MagicMock()
            created.append(self)

        def run_conversation(
            self, user_message, conversation_history=None, stream_callback=None
        ):
            return {"final_response": "ok"}

    fake_module = types.ModuleType("run_agent")
    fake_module.AIAgent = FakeAgent
    monkeypatch.setattr(hd, "DEFAULT_AGENT_CACHE_SIZE", 2)
    hd.clear_agent_cache()

    with patch.dict(sys.modules, {"run_agent": fake_module}):
        hd.run_turn_in_process("one", "chat-a")
        hd.run_turn_in_process("two", "chat-b")
        hd.run_turn_in_process("three", "chat-a")
        hd.run_turn_in_process("four", "chat-c")

    assert len(hd._agent_cache) == 2
    assert list(hd._agent_cache) == ["chat-a", "chat-c"]
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
            "public_api_client.hermes_agent_client.run_turn_subprocess",
            return_value="subprocess-reply",
        ) as subprocess_runner,
    ):
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
            max_turns=None,
            timeout=10,
            stream_callback=None,
        )

    assert reply == "from-default"
    in_process.assert_called_once_with(
        text="hi",
        chat_id="c1",
        personality_id="p1",
        toolsets=None,
        max_turns=None,
        timeout=10,
        stream_callback=None,
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
        timeout=45,
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
