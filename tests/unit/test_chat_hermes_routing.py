"""Unit tests for ChatNode Hermes routing and chunk streaming.

rclpy / ROS message types are stubbed so these run without a live ROS env.
"""

from __future__ import annotations

import sys
import types
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
    return node


def test_uses_hermes_backend_routing_decision():
    from public_api_client.hermes_agent_client import uses_hermes_backend

    assert uses_hermes_backend("hermes-agent") is True
    assert uses_hermes_backend("gpt-4o") is False
    assert uses_hermes_backend("gemini-2.5-flash") is False
    assert uses_hermes_backend(None) is False


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
    chat_node.executor.create_task.assert_called()
    # Single sentence → no prior feedback published yet (result carries it).
    goal_handle.publish_feedback.assert_not_called()


def test_stream_chunks_to_goal_publishes_prior_sentence_as_feedback(
    chat_module, chat_node
):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False

    # Feedback for a completed sentence is published when the *next* token arrives.
    prev, ptype, curr = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-1", ["Eins. ", "Zwei."]
    )

    assert goal_handle.publish_feedback.call_count == 1
    feedback = goal_handle.publish_feedback.call_args[0][0]
    assert feedback.text == "Eins."
    assert feedback.text_type == TEXT_TYPE_SENTENCE
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

    assert goal_handle.publish_feedback.call_count == 1
    feedback = goal_handle.publish_feedback.call_args[0][0]
    assert feedback.text == "Hallo."
    assert feedback.text_type == TEXT_TYPE_SENTENCE
    assert prev == "xml-here"
    assert ptype == TEXT_TYPE_CODE_VISUAL
    assert curr == ""


def test_chat_routes_hermes_without_replaying_history(chat_module, chat_node):
    import asyncio

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
        hermes_agent_client, "ensure_profile", return_value="/tmp/p"
    ) as ensure_profile, patch.object(
        hermes_agent_client, "run_turn", return_value="Antwort vom Agent."
    ) as run_turn, patch.object(
        chat_node,
        "_stream_chunks_to_goal",
        return_value=(None, None, "Antwort vom Agent."),
    ) as stream:

        result = asyncio.run(chat_node.chat(goal_handle))

    get_history.assert_not_called()
    chat_completion.assert_not_called()
    ensure_profile.assert_called_once_with("pers-1", soul_text="Du bist pib.")
    run_turn.assert_called_once()
    assert run_turn.call_args.kwargs["text"] == "Hi"
    assert run_turn.call_args.kwargs["chat_id"] == "chat-9"
    assert run_turn.call_args.kwargs["personality_id"] == "pers-1"
    stream.assert_called_once()
    streamed_tokens = stream.call_args[0][2]
    assert streamed_tokens == ["Antwort vom Agent."]
    goal_handle.succeed.assert_called_once()
    assert result.text == "Antwort vom Agent."


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
