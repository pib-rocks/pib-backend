"""Unit tests for ChatNode Hermes routing and chunk streaming.

rclpy / ROS message types are stubbed so these run without a live ROS env.
"""

from __future__ import annotations

import subprocess
import sys
import threading
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))


TEXT_TYPE_SENTENCE = 0
TEXT_TYPE_CODE_VISUAL = 1


def _install_ros_stubs():
    """Install minimal rclpy/datatypes stubs before importing chat."""

    def ensure(name: str) -> types.ModuleType:
        if name in sys.modules and not isinstance(sys.modules[name], MagicMock):
            # Already a real or previously installed stub module we built.
            mod = sys.modules[name]
            if not isinstance(mod, MagicMock):
                return mod
        mod = types.ModuleType(name)
        sys.modules[name] = mod
        return mod

    # --- rclpy tree ---
    rclpy = ensure("rclpy")
    rclpy.init = MagicMock()
    rclpy.shutdown = MagicMock()
    rclpy.create_node = MagicMock()
    rclpy.spin_until_future_complete = MagicMock()

    action_mod = ensure("rclpy.action")
    action_mod.ActionServer = MagicMock()
    action_mod.CancelResponse = MagicMock()
    action_mod.CancelResponse.ACCEPT = 1

    action_server = ensure("rclpy.action.server")
    action_server.ServerGoalHandle = object

    cb_groups = ensure("rclpy.callback_groups")
    cb_groups.ReentrantCallbackGroup = MagicMock()

    executors = ensure("rclpy.executors")
    executors.MultiThreadedExecutor = MagicMock()

    node_mod = ensure("rclpy.node")

    class _FakeNode:
        def __init__(self, *args, **kwargs):
            pass

        def get_logger(self):
            return MagicMock()

        def create_publisher(self, *args, **kwargs):
            return MagicMock()

        def create_client(self, *args, **kwargs):
            return MagicMock()

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def create_service(self, *args, **kwargs):
            return MagicMock()

        def destroy_node(self):
            pass

    node_mod.Node = _FakeNode

    ensure("rclpy.publisher").Publisher = object
    ensure("rclpy.service").Service = object

    # --- std_msgs ---
    std_msgs = ensure("std_msgs")
    std_msgs_msg = ensure("std_msgs.msg")

    class _String:
        def __init__(self):
            self.data = ""

    std_msgs_msg.String = _String

    # --- datatypes ---
    ensure("datatypes")
    action = ensure("datatypes.action")

    class _Goal:
        TEXT_TYPE_SENTENCE = TEXT_TYPE_SENTENCE
        TEXT_TYPE_CODE_VISUAL = TEXT_TYPE_CODE_VISUAL

        def __init__(self):
            self.chat_id = ""
            self.text = ""
            self.generate_code = False

    class _Feedback:
        def __init__(self):
            self.text = ""
            self.text_type = TEXT_TYPE_SENTENCE

    class _Result:
        def __init__(self):
            self.text = ""
            self.text_type = TEXT_TYPE_SENTENCE

    class _Chat:
        Goal = _Goal
        Feedback = _Feedback
        Result = _Result

    action.Chat = _Chat

    msg = ensure("datatypes.msg")

    class _ChatMessage:
        pass

    msg.ChatMessage = _ChatMessage

    srv = ensure("datatypes.srv")

    class _GetCameraImage:
        class Request:
            pass

    class _VisionPrompt:
        class Request:
            pass

        class Response:
            def __init__(self):
                self.response = ""

    class _CreateOrUpdateChatMessage:
        class Request:
            pass

        class Response:
            def __init__(self):
                self.successful = False
                self.message_id = ""
                self.content = ""

    srv.GetCameraImage = _GetCameraImage
    srv.VisionPrompt = _VisionPrompt
    srv.CreateOrUpdateChatMessage = _CreateOrUpdateChatMessage

    # --- pib / public api clients (if not already importable) ---
    if "pib_api_client" not in sys.modules:
        ensure("pib_api_client")
    if "pib_api_client.voice_assistant_client" not in sys.modules:
        vac = ensure("pib_api_client.voice_assistant_client")
        vac.get_personality_from_chat = MagicMock()
        vac.get_chat_history = MagicMock()
        vac.create_chat_message = MagicMock()
        vac.update_chat_message = MagicMock()

    if "public_api_client.public_voice_client" not in sys.modules:
        pvc = ensure("public_api_client.public_voice_client")

        class PublicApiChatMessage:
            def __init__(self, content, is_user):
                self.content = content
                self.is_user = is_user

        pvc.PublicApiChatMessage = PublicApiChatMessage
        pvc.chat_completion = MagicMock()


@pytest.fixture(scope="module")
def chat_module():
    _install_ros_stubs()
    # Force re-import if a previous failed import left a partial module.
    sys.modules.pop("voice_assistant.chat", None)
    sys.modules.pop("voice_assistant", None)
    from voice_assistant import chat as chat_mod

    return chat_mod


@pytest.fixture
def chat_node(chat_module):
    node = chat_module.ChatNode.__new__(chat_module.ChatNode)
    node.executor = MagicMock()
    node.create_chat_message = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())
    node.public_voice_client_lock = MagicMock()
    node.public_voice_client_lock.__enter__ = MagicMock(return_value=None)
    node.public_voice_client_lock.__exit__ = MagicMock(return_value=False)
    node.voice_assistant_client_lock = MagicMock()
    node.voice_assistant_client_lock.__enter__ = MagicMock(return_value=None)
    node.voice_assistant_client_lock.__exit__ = MagicMock(return_value=False)
    node.token = "tok"
    node.history_length = 10
    node.get_camera_image_client = MagicMock()
    node._hermes_executor = ThreadPoolExecutor(
        max_workers=2, thread_name_prefix="test-hermes-turn"
    )
    yield node
    node._hermes_executor.shutdown(wait=False, cancel_futures=True)


def drive_like_rclpy(coro):
    """Step a coroutine the way rclpy's executor does, with no asyncio loop.

    rclpy.task.Task drives an async execute_callback by calling send() itself; it
    never installs an asyncio event loop. Using asyncio.run() in a test would
    therefore hide exactly the failure the robot hits.
    """
    assert not _asyncio_loop_running(), "this helper must run without an asyncio loop"
    try:
        while True:
            coro.send(None)
    except StopIteration as stop:
        return stop.value


def _asyncio_loop_running() -> bool:
    import asyncio

    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return False
    return True


def test_uses_hermes_backend_routing_decision():
    from public_api_client.hermes_agent_client import uses_hermes_backend

    assert uses_hermes_backend("hermes-agent") is True
    assert uses_hermes_backend("gpt-4o") is False
    assert uses_hermes_backend("gemini-3.6-flash") is False
    assert uses_hermes_backend(None) is False


def test_preflight_logs_an_error_when_the_binary_is_missing(
    chat_module, chat_node, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)

    assert chat_node._preflight_hermes_binary() is False

    logger.error.assert_called_once()
    message = logger.error.call_args[0][0]
    assert "hermes" in message and "fall back" in message


def test_preflight_accepts_an_installed_binary(
    chat_module, chat_node, installed_hermes_bin
):
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)

    assert chat_node._preflight_hermes_binary() is True

    logger.error.assert_not_called()


def test_preflight_runs_a_bounded_liveness_probe(
    chat_module, chat_node, installed_hermes_bin
):
    """The probe must execute the CLI, not just stat it, and stay bounded."""
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)
    completed = subprocess.CompletedProcess(
        args=[str(installed_hermes_bin), "--version"],
        returncode=0,
        stdout="Hermes Agent v0.18.2\nPython: 3.11.15\n",
        stderr="",
    )

    with patch("subprocess.run", return_value=completed) as run:
        assert chat_node._preflight_hermes_binary() is True

    argv, kwargs = run.call_args[0][0], run.call_args[1]
    assert argv == [str(installed_hermes_bin), "--version"]
    assert 0 < kwargs["timeout"] <= 10
    logger.error.assert_not_called()
    # The version banner belongs in the healthy log line, as the proof of life.
    assert "Hermes Agent v0.18.2" in logger.info.call_args[0][0]


def test_preflight_fails_and_reports_stderr_when_the_probe_exits_nonzero(
    chat_module, chat_node, installed_hermes_bin
):
    """The exact failure seen on the robot: wrapper present, interpreter gone.

    The captured stderr is what pinpointed the missing uv mount, so it has to
    reach the log; without it the operator only sees 'agent unavailable'.
    """
    stderr = (
        "/home/pib/.local/bin/hermes: line 4: "
        "/home/pib/.hermes/hermes-agent/venv/bin/python: "
        "No such file or directory\n"
    )
    completed = subprocess.CompletedProcess(
        args=[str(installed_hermes_bin), "--version"],
        returncode=127,
        stdout="",
        stderr=stderr,
    )
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)

    with patch("subprocess.run", return_value=completed):
        assert chat_node._preflight_hermes_binary() is False

    logger.error.assert_called_once()
    message = logger.error.call_args[0][0]
    assert "127" in message
    assert "venv/bin/python: No such file or directory" in message
    assert "fall back" in message


def test_preflight_fails_without_raising_when_the_probe_times_out(
    chat_module, chat_node, installed_hermes_bin
):
    """A wedged CLI must not turn into a node that refuses to start."""
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)

    with patch(
        "subprocess.run",
        side_effect=subprocess.TimeoutExpired(cmd="hermes --version", timeout=5),
    ):
        assert chat_node._preflight_hermes_binary() is False

    logger.error.assert_called_once()
    assert "fall back" in logger.error.call_args[0][0]


def test_ensure_hermes_daemon_logs_when_available(chat_module, chat_node):
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)

    with patch(
        "public_api_client.hermes_daemon.ensure_daemon_running", return_value=True
    ), patch(
        "public_api_client.hermes_daemon.daemon_base_url",
        return_value="http://127.0.0.1:8088",
    ):
        assert chat_node._ensure_hermes_daemon() is True

    logger.info.assert_called()
    assert "hermes daemon available" in logger.info.call_args[0][0]


def test_ensure_hermes_daemon_never_raises(chat_module, chat_node):
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)

    with patch(
        "public_api_client.hermes_daemon.ensure_daemon_running",
        side_effect=RuntimeError("bind failed"),
    ):
        assert chat_node._ensure_hermes_daemon() is False

    logger.warning.assert_called()
    assert "oneshot subprocess fallback" in logger.warning.call_args[0][0]


def test_stream_chunks_to_goal_splits_sentences(chat_module, chat_node):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False

    prev, ptype, curr = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-1", ["Hallo Welt. "]
    )

    # Last completed sentence stays in prev for Chat.Result; nothing left in curr.
    assert prev == "Hallo Welt."
    assert ptype == TEXT_TYPE_SENTENCE
    assert curr == ""
    chat_node.create_chat_message.assert_called()
    # TTFT fast-path publishes the first token immediately as feedback.
    assert goal_handle.publish_feedback.call_count == 1
    assert goal_handle.publish_feedback.call_args[0][0].text == "Hallo Welt. "


def test_stream_chunks_to_goal_writes_chunks_in_order(chat_module, chat_node):
    """A CREATE must complete before the UPDATE that targets its message id.

    Scheduling the writes as executor tasks let them overlap, so the UPDATEs
    hit the previous message and the created one kept only the first sentence.
    """
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False

    chat_node._stream_chunks_to_goal(
        goal_handle, "chat-1", ["Eins. ", "Zwei. ", "Drei."]
    )

    chat_node.executor.create_task.assert_not_called()
    updates = [call[0][3] for call in chat_node.create_chat_message.call_args_list]
    # First write creates the assistant message, every later one updates it.
    assert updates == [False, True, True]


def test_stream_chunks_to_goal_persists_text_without_terminator(
    chat_module, chat_node
):
    """The tail of a reply used to be dropped when it had no '.', '?' or '!'."""
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False

    prev, ptype, curr = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-1", ["Eins. ", "Zwei ohne Punkt"]
    )

    texts = [call[0][1] for call in chat_node.create_chat_message.call_args_list]
    assert texts == ["Eins.", "Zwei ohne Punkt"]
    # The completed sentence goes out as feedback so the tail can ride along in
    # Chat.Result, which the caller only fills from curr_text when prev is None.
    assert prev is None
    assert ptype is None
    assert curr == "Zwei ohne Punkt"
    # First call is TTFT immediate emit of "Eins. "; later call publishes prior sentence.
    feedback_texts = [
        call[0][0].text for call in goal_handle.publish_feedback.call_args_list
    ]
    assert "Eins. " in feedback_texts
    assert "Eins." in feedback_texts


def test_stream_chunks_to_goal_creates_when_only_unterminated_text_arrives(
    chat_module, chat_node
):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False

    prev, _ptype, curr = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-1", ["Antwort ohne Satzzeichen"]
    )

    chat_node.create_chat_message.assert_called_once_with(
        "chat-1", "Antwort ohne Satzzeichen", False, False, True
    )
    assert prev is None
    assert curr == "Antwort ohne Satzzeichen"
    # TTFT: first token is published immediately even without a terminator.
    assert goal_handle.publish_feedback.call_count >= 1
    assert (
        goal_handle.publish_feedback.call_args_list[0][0][0].text
        == "Antwort ohne Satzzeichen"
    )


def test_stream_chunks_to_goal_publishes_prior_sentence_as_feedback(
    chat_module, chat_node
):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False

    # Feedback for a completed sentence is published when the *next* token arrives.
    # Plus TTFT immediate emit of the first token.
    prev, ptype, curr = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-1", ["Eins. ", "Zwei."]
    )

    assert goal_handle.publish_feedback.call_count >= 2
    feedback_texts = [
        call[0][0].text for call in goal_handle.publish_feedback.call_args_list
    ]
    assert "Eins. " in feedback_texts  # TTFT first-token emit
    assert "Eins." in feedback_texts   # prior completed sentence
    assert prev == "Zwei."
    assert ptype == TEXT_TYPE_SENTENCE
    assert curr == ""


def test_stream_chunks_to_goal_extracts_pib_program(chat_module, chat_node):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False

    # Sentence completes in first token; program completes in later tokens so the
    # sentence is published as feedback (same Action contract as the legacy path).
    tokens = ["Hallo. ", "<pib-program>xml-here</pib-program>"]
    prev, ptype, curr = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-1", tokens
    )

    assert goal_handle.publish_feedback.call_count >= 2
    feedback_texts = [
        call[0][0].text for call in goal_handle.publish_feedback.call_args_list
    ]
    assert "Hallo. " in feedback_texts
    assert "Hallo." in feedback_texts
    assert prev == "xml-here"
    assert ptype == TEXT_TYPE_CODE_VISUAL
    assert curr == ""


def test_stream_chunks_to_goal_emits_perf_trace_on_first_chunk(
    chat_module, chat_node
):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)

    chat_node._stream_chunks_to_goal(goal_handle, "chat-1", ["Hallo."])

    info_messages = [call[0][0] for call in logger.info.call_args_list]
    assert any("[PERF_TRACE] FIRST_CHUNK_EMITTED" in msg for msg in info_messages)


def test_chat_routes_hermes_without_replaying_history(chat_module, chat_node):
    import os

    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat

    personality = MagicMock()
    personality.message_history = 5
    personality.description = "Du bist pib."
    personality.personality_id = "pers-1"
    personality.assistant_model.api_name = "hermes-agent"
    personality.assistant_model.has_image_support = True

    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-9"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False

    with patch.object(
        chat_module.voice_assistant_client,
        "get_personality_from_chat",
        return_value=(True, personality),
    ), patch.object(
        chat_module.voice_assistant_client,
        "get_chat_history",
    ) as get_history, patch.object(
        chat_module.public_voice_client,
        "chat_completion",
    ) as chat_completion, patch.object(
        hermes_agent_client, "is_warm_daemon_active", return_value=False
    ), patch.object(
        hermes_agent_client, "ensure_profile", return_value="/tmp/p"
    ) as ensure_profile, patch.object(
        hermes_agent_client, "run_turn", return_value="Antwort vom Agent."
    ) as run_turn, patch.object(
        chat_node,
        "_stream_chunks_to_goal",
        return_value=(None, None, "Antwort vom Agent."),
    ) as stream, patch.dict(
        os.environ, {"PIB_HERMES_TIMEOUT": "95"}, clear=False
    ):

        result = drive_like_rclpy(chat_node.chat(goal_handle))

    get_history.assert_not_called()
    chat_completion.assert_not_called()
    ensure_profile.assert_called_once_with("pers-1", soul_text="Du bist pib.")
    run_turn.assert_called_once()
    assert run_turn.call_args.kwargs["text"] == "Hi"
    assert run_turn.call_args.kwargs["chat_id"] == "chat-9"
    assert run_turn.call_args.kwargs["personality_id"] == "pers-1"
    assert run_turn.call_args.kwargs["timeout"] == 95
    stream.assert_called_once()
    streamed_tokens = stream.call_args[0][2]
    assert streamed_tokens == ["Antwort vom Agent."]
    goal_handle.succeed.assert_called_once()
    assert result.text == "Antwort vom Agent."


def test_chat_skips_ensure_profile_when_warm_daemon_active(chat_module, chat_node):
    """Warm daemon path must not re-validate profiles on the filesystem."""
    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat

    personality = MagicMock()
    personality.message_history = 5
    personality.description = "Du bist pib."
    personality.personality_id = "pers-1"
    personality.assistant_model.api_name = "hermes-agent"
    personality.assistant_model.has_image_support = False

    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-9"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False

    with patch.object(
        chat_module.voice_assistant_client,
        "get_personality_from_chat",
        return_value=(True, personality),
    ), patch.object(
        hermes_agent_client, "is_warm_daemon_active", return_value=True
    ), patch.object(
        hermes_agent_client, "ensure_profile", return_value="/tmp/p"
    ) as ensure_profile, patch.object(
        hermes_agent_client, "run_turn", return_value="schnell"
    ), patch.object(
        chat_node,
        "_stream_chunks_to_goal",
        return_value=(None, None, "schnell"),
    ):
        result = drive_like_rclpy(chat_node.chat(goal_handle))

    ensure_profile.assert_not_called()
    goal_handle.succeed.assert_called_once()
    assert result.text == "schnell"


def test_chat_emits_ros_perf_trace_logs(chat_module, chat_node):
    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)

    personality = MagicMock()
    personality.message_history = 5
    personality.description = "Du bist pib."
    personality.personality_id = "pers-1"
    personality.assistant_model.api_name = "hermes-agent"
    personality.assistant_model.has_image_support = False

    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-9"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False

    with patch.object(
        chat_module.voice_assistant_client,
        "get_personality_from_chat",
        return_value=(True, personality),
    ), patch.object(
        hermes_agent_client, "is_warm_daemon_active", return_value=True
    ), patch.object(
        hermes_agent_client, "run_turn", return_value="ok"
    ), patch.object(
        chat_node,
        "_stream_chunks_to_goal",
        return_value=(None, None, "ok"),
    ):
        drive_like_rclpy(chat_node.chat(goal_handle))

    info_messages = [call[0][0] for call in logger.info.call_args_list]
    assert any("[PERF_TRACE] ROS_SERVICE_RECV" in msg for msg in info_messages)
    assert any("[PERF_TRACE] ROS_SERVICE_DONE" in msg for msg in info_messages)


def test_run_hermes_turn_needs_no_asyncio_event_loop(
    chat_module, chat_node, installed_hermes_bin, tmp_path, monkeypatch
):
    """The robot aborted every hermes goal with 'no running event loop'.

    rclpy runs execute_callback on its own executor, so the hermes turn must work
    from a plain synchronous thread that has no asyncio loop at all.
    """
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    assert not _asyncio_loop_running()

    completed = subprocess.CompletedProcess(
        args=["hermes"], returncode=0, stdout="Antwort vom Agent.\n", stderr=""
    )
    with patch("subprocess.run", return_value=completed):
        reply = chat_node._run_hermes_turn(
            text="Hi",
            chat_id="chat-9",
            personality_id="pers-1",
            description="Du bist pib.",
        )

    assert reply == "Antwort vom Agent."


def test_run_hermes_turn_reuses_the_shared_pool(chat_module, chat_node):
    """A pool per request would leak threads under repeated chat goals."""
    seen = []

    def record(fn):
        seen.append(fn)
        future = MagicMock()
        future.result.return_value = "ok"
        return future

    with patch.object(chat_node._hermes_executor, "submit", side_effect=record):
        for _ in range(3):
            chat_node._run_hermes_turn(
                text="Hi", chat_id="c", personality_id=None, description="d"
            )

    assert len(seen) == 3


def test_chat_hermes_branch_survives_the_rclpy_task_driver(chat_module, chat_node):
    """End-to-end regression for the aborted goal, driven exactly like rclpy does."""
    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat

    personality = MagicMock()
    personality.message_history = 5
    personality.description = "Du bist pib."
    personality.personality_id = "pers-1"
    personality.assistant_model.api_name = "hermes-agent"
    personality.assistant_model.has_image_support = True

    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-9"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False

    with patch.object(
        chat_module.voice_assistant_client,
        "get_personality_from_chat",
        return_value=(True, personality),
    ), patch.object(
        hermes_agent_client, "is_warm_daemon_active", return_value=False
    ), patch.object(
        hermes_agent_client, "ensure_profile", return_value="/tmp/p"
    ), patch.object(
        hermes_agent_client, "run_turn", return_value="Antwort vom Agent."
    ), patch.object(
        chat_node,
        "_stream_chunks_to_goal",
        return_value=(None, None, "Antwort vom Agent."),
    ):
        result = drive_like_rclpy(chat_node.chat(goal_handle))

    goal_handle.abort.assert_not_called()
    goal_handle.succeed.assert_called_once()
    assert result.text == "Antwort vom Agent."


def test_run_hermes_turn_falls_back_when_the_agent_times_out(
    chat_module, chat_node, installed_hermes_bin, tmp_path, monkeypatch
):
    from public_api_client import hermes_agent_client

    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))

    with patch(
        "subprocess.run",
        side_effect=subprocess.TimeoutExpired(cmd="hermes", timeout=1),
    ):
        reply = chat_node._run_hermes_turn(
            text="Hi",
            chat_id="chat-9",
            personality_id="pers-1",
            description="Du bist pib.",
            timeout=1,
        )

    assert reply == hermes_agent_client.FALLBACK_REPLY


def test_run_hermes_turn_falls_back_when_the_worker_raises(chat_module, chat_node):
    """Even a broken profile write must yield speakable text, never an exception."""
    from public_api_client import hermes_agent_client

    with patch.object(
        hermes_agent_client, "is_warm_daemon_active", return_value=False
    ), patch.object(
        hermes_agent_client, "ensure_profile", side_effect=OSError("read-only fs")
    ):
        reply = chat_node._run_hermes_turn(
            text="Hi", chat_id="chat-9", personality_id="pers-1", description="d"
        )

    assert reply == hermes_agent_client.FALLBACK_REPLY


def test_chat_hermes_goal_succeeds_with_fallback_when_the_agent_fails(
    chat_module, chat_node
):
    """A failing agent must not abort the goal: the fallback sentence is spoken."""
    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat

    personality = MagicMock()
    personality.message_history = 5
    personality.description = "Du bist pib."
    personality.personality_id = "pers-1"
    personality.assistant_model.api_name = "hermes-agent"
    personality.assistant_model.has_image_support = False

    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-9"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False

    with patch.object(
        chat_module.voice_assistant_client,
        "get_personality_from_chat",
        return_value=(True, personality),
    ), patch.object(
        hermes_agent_client, "is_warm_daemon_active", return_value=False
    ), patch.object(
        hermes_agent_client, "ensure_profile", return_value="/tmp/p"
    ), patch.object(
        hermes_agent_client, "run_turn", side_effect=RuntimeError("agent exploded")
    ), patch.object(
        chat_node,
        "_stream_chunks_to_goal",
        return_value=(None, None, hermes_agent_client.FALLBACK_REPLY),
    ):
        result = drive_like_rclpy(chat_node.chat(goal_handle))

    goal_handle.abort.assert_not_called()
    goal_handle.succeed.assert_called_once()
    assert result.text == hermes_agent_client.FALLBACK_REPLY


def test_run_hermes_turn_gives_up_when_the_goal_is_cancelled(chat_module, chat_node):
    """Cancelling mid-subprocess returns control without crashing the node."""
    from public_api_client import hermes_agent_client

    release = threading.Event()
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = True

    def _blocking_turn(*_args, **_kwargs):
        release.wait(timeout=5)
        return "too late"

    with patch.object(hermes_agent_client, "run_turn", side_effect=_blocking_turn):
        reply = chat_node._run_hermes_turn(
            text="Hi",
            chat_id="chat-9",
            personality_id=None,
            description="d",
            goal_handle=goal_handle,
        )
    release.set()

    assert reply == ""


def test_chat_hermes_cancelled_goal_is_marked_canceled(chat_module, chat_node):
    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat

    personality = MagicMock()
    personality.message_history = 5
    personality.description = "Du bist pib."
    personality.personality_id = "pers-1"
    personality.assistant_model.api_name = "hermes-agent"
    personality.assistant_model.has_image_support = False

    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = True
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-9"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False

    with patch.object(
        chat_module.voice_assistant_client,
        "get_personality_from_chat",
        return_value=(True, personality),
    ), patch.object(
        hermes_agent_client, "is_warm_daemon_active", return_value=False
    ), patch.object(
        hermes_agent_client, "ensure_profile", return_value="/tmp/p"
    ), patch.object(
        hermes_agent_client, "run_turn", return_value="ignored"
    ), patch.object(
        chat_node, "_stream_chunks_to_goal"
    ) as stream:
        result = drive_like_rclpy(chat_node.chat(goal_handle))

    stream.assert_not_called()
    goal_handle.canceled.assert_called_once()
    goal_handle.abort.assert_not_called()
    goal_handle.succeed.assert_not_called()
    assert result.text == ""


def test_chat_module_does_not_reintroduce_asyncio_loop_lookup():
    """Guard the regression at the source level, not just behaviourally.

    Checked on the AST so the explanatory docstrings may keep naming the call
    that broke the robot.
    """
    import ast

    source = (
        REPO_ROOT / "ros_packages" / "voice_assistant" / "voice_assistant" / "chat.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)

    referenced = {
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    } | {
        alias.name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }

    assert "asyncio" not in referenced


def test_chat_legacy_path_still_uses_public_api(chat_module, chat_node):
    import asyncio

    Chat = chat_module.Chat

    personality = MagicMock()
    personality.message_history = 5
    personality.description = "legacy"
    personality.personality_id = "pers-2"
    personality.assistant_model.api_name = "gpt-4o"
    personality.assistant_model.has_image_support = False

    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-2"
    goal_handle.request.text = "Hallo"
    goal_handle.request.generate_code = False

    with patch.object(
        chat_module.voice_assistant_client,
        "get_personality_from_chat",
        return_value=(True, personality),
    ), patch.object(
        chat_module.voice_assistant_client,
        "get_chat_history",
        return_value=(True, []),
    ) as get_history, patch.object(
        chat_module.public_voice_client,
        "chat_completion",
        return_value=iter(["Hi."]),
    ) as chat_completion, patch.object(
        chat_node,
        "_stream_chunks_to_goal",
        return_value=("Hi.", TEXT_TYPE_SENTENCE, ""),
    ) as stream, patch(
        "public_api_client.hermes_agent_client.run_turn"
    ) as run_turn:

        result = asyncio.run(chat_node.chat(goal_handle))

    get_history.assert_called_once()
    chat_completion.assert_called_once()
    run_turn.assert_not_called()
    stream.assert_called_once()
    assert result.text == "Hi."
    goal_handle.succeed.assert_called_once()
