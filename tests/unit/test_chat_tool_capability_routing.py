"""Capability-based dispatch of a chat turn (concept D3).

The tool switch decides the transport, not the model. A pib.Cloud personality
with tool calling on runs on the Hermes path, where the agent runtime and its
MCP server live; a pib.Cloud personality without it keeps the plain cloud
stream, which carries no tools at all. Everything else routes as it did.

rclpy / ROS message types are stubbed so these run without a live ROS env,
the same way tests/unit/test_chat_hermes_routing.py does it.
"""

from __future__ import annotations

import sys
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
for _path in (
    str(VOICE_ASSISTANT_PKG),
    str(REPO_ROOT / "pib_api" / "flask"),
):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from provider_registry import PIB_CLOUD_API_NAME  # noqa: E402

TEXT_TYPE_SENTENCE = 0
TEXT_TYPE_CODE_VISUAL = 1
#: What provider_name_for resolves the pib.Cloud model to.
PIB_CLOUD_PROVIDER_NAME = "pib.Cloud"


def _install_ros_stubs() -> None:
    """Install minimal rclpy/datatypes stubs before importing chat."""

    def ensure(name: str) -> types.ModuleType:
        module = sys.modules.get(name)
        if module is not None and not isinstance(module, MagicMock):
            return module
        module = types.ModuleType(name)
        sys.modules[name] = module
        return module

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

    ensure("rclpy.action.server").ServerGoalHandle = object
    ensure("rclpy.callback_groups").ReentrantCallbackGroup = MagicMock()
    ensure("rclpy.executors").MultiThreadedExecutor = MagicMock()
    ensure("rclpy.publisher").Publisher = object
    ensure("rclpy.service").Service = object

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

    # --- std_msgs ---
    ensure("std_msgs")
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

    ensure("datatypes.msg").ChatMessage = type("_ChatMessage", (), {})

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

    # --- api clients, when they are not importable in this environment ---
    if "pib_api_client" not in sys.modules:
        ensure("pib_api_client")
    if "pib_api_client.voice_assistant_client" not in sys.modules:
        vac = ensure("pib_api_client.voice_assistant_client")
        vac.get_personality_from_chat = MagicMock()
        vac.get_chat_history = MagicMock()
        vac.create_chat_message = MagicMock()
        vac.update_chat_message = MagicMock()
        vac.record_first_token_latency = MagicMock(return_value=False)

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
    client_prefixes = ("pib_api_client", "public_api_client.public_voice_client")
    saved_clients = {
        name: module
        for name, module in sys.modules.items()
        if any(name == p or name.startswith(p + ".") for p in client_prefixes)
    }
    _install_ros_stubs()
    # Force a re-import if a previous failed import left a partial module, and
    # put the entries back afterwards so later test files still find them.
    stolen = {
        name: sys.modules.pop(name)
        for name in ("voice_assistant.chat", "voice_assistant")
        if name in sys.modules
    }
    from voice_assistant import chat as chat_mod

    yield chat_mod

    for name in tuple(sys.modules):
        if any(name == p or name.startswith(p + ".") for p in client_prefixes):
            sys.modules.pop(name, None)
    sys.modules.update(saved_clients)
    for name, module in stolen.items():
        sys.modules[name] = module


@pytest.fixture
def chat_node(chat_module):
    """A ChatNode without a live node: only the turn path under test is real."""
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
    node._remember_first_token_latency = MagicMock()
    node._key_store_mode = lambda: "unlocked"
    node.get_camera_image_client = MagicMock()
    node._hermes_executor = ThreadPoolExecutor(
        max_workers=2, thread_name_prefix="test-tool-routing"
    )
    yield node
    node._hermes_executor.shutdown(wait=False, cancel_futures=True)


def drive_like_rclpy(coro):
    """Step a coroutine the way rclpy's executor does, with no asyncio loop."""
    try:
        while True:
            coro.send(None)
    except StopIteration as stop:
        return stop.value


def _goal(chat_module, chat_id: str):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = chat_module.Chat.Goal()
    goal_handle.request.chat_id = chat_id
    goal_handle.request.text = "Hallo"
    goal_handle.request.generate_code = False
    return goal_handle


def _personality(api_name: str, *, tool_calling: bool, channel: str = "direct"):
    """A personality payload shaped like the one the API hands the node."""
    personality = MagicMock()
    personality.message_history = 5
    personality.personality_id = "pers-tools"
    personality.description = "Du bist der eine Soul."
    personality.assistant_model.api_name = api_name
    personality.assistant_model.has_image_support = False
    personality.tool_calling = tool_calling
    personality.channel = channel
    personality.effective_channel = channel
    personality.voice_mode = "turn_based"
    personality.gender = "Female"
    personality.language = "German"
    return personality


def _run_turn(chat_module, chat_node, personality, chat_id="chat-tools"):
    """Run one turn with the personality's clients and the streamers faked."""
    goal_handle = _goal(chat_module, chat_id)
    with (
        patch.object(
            chat_module.voice_assistant_client,
            "get_personality_from_chat",
            return_value=(True, personality),
        ),
        patch.object(
            chat_module.voice_assistant_client,
            "get_chat_history",
            return_value=(True, []),
        ),
        patch.object(
            chat_module.direct_tool_loop,
            "run_direct_turn",
            return_value=iter(["Direkt."]),
        ) as run_direct_turn,
        patch.object(
            chat_node, "_stream_hermes_turn", return_value=["Vom Agent."]
        ) as hermes_stream,
        patch.object(
            chat_node, "_stream_pib_cloud_turn", return_value=["Aus der Cloud."]
        ) as cloud_stream,
        patch.object(
            chat_node,
            "_stream_chunks_to_goal",
            return_value=(None, None, "Antwort."),
        ) as stream_chunks,
    ):
        result = drive_like_rclpy(chat_node.chat(goal_handle))
    return goal_handle, result, hermes_stream, cloud_stream, run_direct_turn


def test_tools_on_with_the_pib_cloud_model_routes_through_hermes(
    chat_module, chat_node
):
    """D3: the capability decides. The cloud stream cannot run a tool turn."""
    personality = _personality(PIB_CLOUD_API_NAME, tool_calling=True)

    goal_handle, result, hermes, cloud, run_direct = _run_turn(
        chat_module, chat_node, personality
    )

    cloud.assert_not_called()
    run_direct.assert_not_called()
    hermes.assert_called_once()
    kwargs = hermes.call_args.kwargs
    # Exactly the arguments a Smart turn passes.
    assert kwargs["chat_id"] == "chat-tools"
    assert kwargs["personality_id"] == "pers-tools"
    assert kwargs["description"] == "Du bist der eine Soul."
    assert kwargs["model"] == PIB_CLOUD_API_NAME
    assert kwargs["provider"] == PIB_CLOUD_PROVIDER_NAME
    assert kwargs["base_url"] == chat_module.pib_cloud_stream.pib_cloud_base_url()
    assert kwargs["text"] == "Hallo"
    assert result.text == "Antwort."
    goal_handle.succeed.assert_called_once()
    goal_handle.abort.assert_not_called()


def test_tools_on_with_the_pib_cloud_model_logs_the_decision(chat_module, chat_node):
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)
    personality = _personality(PIB_CLOUD_API_NAME, tool_calling=True)

    _run_turn(chat_module, chat_node, personality)

    infos = [call[0][0] for call in logger.info.call_args_list]
    assert any(
        message.startswith(
            "tools enabled: pib.Cloud model routed through Hermes chat=chat-tools"
        )
        for message in infos
    )


def test_tools_off_with_the_pib_cloud_model_keeps_the_cloud_stream(
    chat_module, chat_node
):
    personality = _personality(PIB_CLOUD_API_NAME, tool_calling=False)

    goal_handle, result, hermes, cloud, run_direct = _run_turn(
        chat_module, chat_node, personality
    )

    cloud.assert_called_once()
    hermes.assert_not_called()
    run_direct.assert_not_called()
    assert cloud.call_args.kwargs["chat_id"] == "chat-tools"
    assert result.text == "Antwort."
    goal_handle.succeed.assert_called_once()


def test_tools_off_with_the_pib_cloud_model_on_a_smart_channel_keeps_the_cloud_stream(
    chat_module, chat_node
):
    """The channel does not open the tools; only the switch does."""
    personality = _personality(PIB_CLOUD_API_NAME, tool_calling=False, channel="smart")

    _goal_handle, _result, hermes, cloud, _run_direct = _run_turn(
        chat_module, chat_node, personality
    )

    cloud.assert_called_once()
    hermes.assert_not_called()


def test_tools_on_with_a_smart_personality_still_uses_hermes(chat_module, chat_node):
    personality = _personality("gemini-3.8-flash", tool_calling=True, channel="smart")

    _goal_handle, _result, hermes, cloud, run_direct = _run_turn(
        chat_module, chat_node, personality
    )

    hermes.assert_called_once()
    cloud.assert_not_called()
    run_direct.assert_not_called()
    # The tool decision line is the pib.Cloud one; a Smart turn does not report it.
    assert hermes.call_args.kwargs["model"] == "gemini-3.8-flash"


def test_tools_on_with_a_direct_model_uses_the_direct_tool_loop(chat_module, chat_node):
    personality = _personality("gemini-3.8-flash", tool_calling=True)

    _goal_handle, result, hermes, cloud, run_direct = _run_turn(
        chat_module, chat_node, personality
    )

    run_direct.assert_called_once()
    assert run_direct.call_args.kwargs["tool_calling"] is True
    hermes.assert_not_called()
    cloud.assert_not_called()
    assert result.text == "Antwort."


def test_retired_direct_text_is_still_refused_before_the_tool_decision(
    chat_module, chat_node
):
    """The retirement abort keeps its place in front of every tool path."""
    from pib_hermes_config.retired_direct_text import RETIRED_DIRECT_TEXT_SENTENCE

    personality = _personality("gpt-6", tool_calling=True)

    _goal_handle, result, hermes, cloud, run_direct = _run_turn(
        chat_module, chat_node, personality
    )

    hermes.assert_not_called()
    cloud.assert_not_called()
    run_direct.assert_not_called()
    chat_node.create_chat_message.assert_any_call(
        "chat-tools", RETIRED_DIRECT_TEXT_SENTENCE, False, False, True
    )
    assert result.text == RETIRED_DIRECT_TEXT_SENTENCE


def test_key_store_refusal_still_stops_the_cloud_tool_turn(chat_module, chat_node):
    """The store check runs before the routing decision, as it did."""
    from voice_assistant.degraded_chat import refusal_sentence

    chat_node._key_store_mode = lambda: "locked"
    personality = _personality(PIB_CLOUD_API_NAME, tool_calling=True)

    goal_handle, result, hermes, cloud, run_direct = _run_turn(
        chat_module, chat_node, personality
    )

    hermes.assert_not_called()
    cloud.assert_not_called()
    run_direct.assert_not_called()
    chat_node.create_chat_message.assert_any_call(
        "chat-tools", refusal_sentence("direct"), False, False, True
    )
    assert result.text == refusal_sentence("direct")
    goal_handle.succeed.assert_called_once()
    goal_handle.abort.assert_not_called()


def test_direct_model_without_a_tool_endpoint_is_still_refused(chat_module, chat_node):
    """The explicit Direct refusal is not weakened into a silent fallback.

    Live voice mode is what keeps this turn out of the retirement sentence; the
    model still has no non-beta tool endpoint, so it aborts.
    """
    personality = _personality("gpt-6", tool_calling=True)
    personality.voice_mode = "live"

    goal_handle, result, hermes, cloud, run_direct = _run_turn(
        chat_module, chat_node, personality
    )

    run_direct.assert_not_called()
    hermes.assert_not_called()
    cloud.assert_not_called()
    goal_handle.abort.assert_called_once()
    goal_handle.succeed.assert_not_called()
    assert result.text == ""


def test_a_failing_cloud_tool_turn_is_logged_with_its_own_path(chat_module, chat_node):
    """The failure log names the path the turn took, not the stored channel.

    A pib.Cloud personality with the tool switch on runs on Hermes, so an error
    logged as "direct" would send the next diagnosis to the wrong component.
    """
    logger = MagicMock()
    chat_node.get_logger = MagicMock(return_value=logger)
    personality = _personality(PIB_CLOUD_API_NAME, tool_calling=True)
    goal_handle = _goal(chat_module, "chat-tools")

    with (
        patch.object(
            chat_module.voice_assistant_client,
            "get_personality_from_chat",
            return_value=(True, personality),
        ),
        patch.object(
            chat_module.voice_assistant_client,
            "get_chat_history",
            return_value=(True, []),
        ),
        patch.object(
            chat_node, "_stream_hermes_turn", side_effect=RuntimeError("boom")
        ),
    ):
        drive_like_rclpy(chat_node.chat(goal_handle))

    errors = [call[0][0] for call in logger.error.call_args_list]
    assert any("failed to send request to Hermes" in message for message in errors)
    assert not any("failed to send request to direct" in message for message in errors)
    goal_handle.abort.assert_called_once()
