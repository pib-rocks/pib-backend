"""Chat-node wiring: the installed emitter publishes cloud trace stages.

rclpy is stubbed the same way as tests/unit/test_chat_hermes_routing.py.
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

SECRET = "row4-secret-should-not-leak"


def _install_ros_stubs() -> None:
    def ensure(name: str) -> types.ModuleType:
        existing = sys.modules.get(name)
        if isinstance(existing, types.ModuleType) and not isinstance(
            existing, MagicMock
        ):
            return existing
        mod = types.ModuleType(name)
        sys.modules[name] = mod
        return mod

    rclpy = ensure("rclpy")
    rclpy.init = MagicMock()
    rclpy.shutdown = MagicMock()
    action_mod = ensure("rclpy.action")
    action_mod.ActionServer = MagicMock()
    action_mod.CancelResponse = MagicMock()
    action_mod.CancelResponse.ACCEPT = 1
    ensure("rclpy.action.server").ServerGoalHandle = object
    ensure("rclpy.callback_groups").ReentrantCallbackGroup = MagicMock()
    ensure("rclpy.executors").MultiThreadedExecutor = MagicMock()

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

        def create_timer(self, *args, **kwargs):
            return MagicMock()

        def destroy_node(self):
            pass

    if not hasattr(node_mod, "Node"):
        node_mod.Node = _FakeNode
    else:
        for method_name in (
            "create_publisher",
            "create_client",
            "create_subscription",
            "create_service",
            "create_timer",
            "get_logger",
            "destroy_node",
        ):
            if not hasattr(node_mod.Node, method_name):
                setattr(node_mod.Node, method_name, getattr(_FakeNode, method_name))

    ensure("rclpy.publisher").Publisher = object
    ensure("rclpy.service").Service = object

    std_msgs_msg = ensure("std_msgs.msg")
    ensure("std_msgs")

    if not hasattr(std_msgs_msg, "String"):

        class _String:
            def __init__(self):
                self.data = ""

        std_msgs_msg.String = _String

    ensure("datatypes")
    action = ensure("datatypes.action")

    class _Goal:
        TEXT_TYPE_SENTENCE = 0
        TEXT_TYPE_CODE_VISUAL = 1

        def __init__(self):
            self.chat_id = ""
            self.text = ""
            self.generate_code = False

    class _Chat:
        Goal = _Goal

        class Feedback:
            def __init__(self):
                self.text = ""
                self.text_type = 0

        class Result:
            def __init__(self):
                self.text = ""
                self.text_type = 0

    if not hasattr(action, "Chat"):
        action.Chat = _Chat

    msg = ensure("datatypes.msg")
    if not hasattr(msg, "ChatMessage"):

        class _ChatMessage:
            pass

        msg.ChatMessage = _ChatMessage

    srv = ensure("datatypes.srv")
    if not hasattr(srv, "GetCameraImage"):

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

    if "pib_api_client" not in sys.modules:
        ensure("pib_api_client")
    if "pib_api_client.voice_assistant_client" not in sys.modules:
        vac = ensure("pib_api_client.voice_assistant_client")
        vac.get_personality_from_chat = MagicMock()
        vac.get_chat_history = MagicMock()
        vac.create_chat_message = MagicMock()
        vac.update_chat_message = MagicMock()


class _Headers(dict):
    def get(self, key, default=None):
        for name, value in self.items():
            if str(name).lower() == str(key).lower():
                return value
        return default


class _FakeResponse:
    def __init__(self):
        self.status_code = 200
        self.headers = _Headers(
            {
                "X-Upstream-Ttft-Ms": "880",
                "x-usage-prompt-tokens": "120",
                "X-Usage-Completion-Tokens": "88",
                "X-Budget-Remaining-Tokens": "49880",
            }
        )
        self.closed = False

    def iter_lines(self, chunk_size=1, decode_unicode=True):
        del chunk_size, decode_unicode
        yield 'data: {"choices":[{"delta":{"content":"Hallo"}}]}'
        yield "data: [DONE]"

    def close(self):
        self.closed = True


def test_chat_node_emitter_publishes_cloud_trace_stages(monkeypatch):
    _install_ros_stubs()
    from voice_assistant import chat as chat_mod

    # The node installs the emitter on the module object chat.py imported.
    pib_cloud_stream = chat_mod.pib_cloud_stream
    published: list[str] = []
    original_create = chat_mod.Node.create_publisher

    def create_publisher(self, msg_type, topic, qos=10):
        publisher = original_create(self, msg_type, topic, qos)
        if topic == "/voice_turn_trace":

            def _publish(message):
                published.append(message.data)

            publisher.publish.side_effect = _publish
        return publisher

    monkeypatch.setattr(chat_mod.Node, "create_publisher", create_publisher)
    monkeypatch.setattr(
        chat_mod.hermes_agent_client,
        "probe_binary",
        lambda timeout=5: (True, "stub"),
    )
    monkeypatch.setattr(chat_mod, "schedule_memory_consolidation", lambda node: None)
    import public_api_client.hermes_daemon as hermes_daemon

    monkeypatch.setattr(hermes_daemon, "ensure_daemon_running", lambda: False)
    import voice_assistant.attention as attention

    monkeypatch.setattr(
        attention, "bind_attention", lambda *args, **kwargs: MagicMock()
    )

    pib_cloud_stream.set_trace_emitter(None)
    pib_cloud_stream.reset_cloud_auth_state()
    node = None
    try:
        node = chat_mod.ChatNode()
        tokens = list(
            pib_cloud_stream.stream_pib_cloud_chat(
                system_prompt="Du bist pib.",
                user_text="Sag hallo.",
                chat_id="chat-wire-1",
                endpoint_base="http://192.168.1.252:8090/v1",
                token_fetcher=lambda: {"mode": "unlocked", "secret": SECRET},
                http_post=lambda *args, **kwargs: _FakeResponse(),
                retry_delay_s=0,
            )
        )
    finally:
        if node is not None:
            node.destroy_node()
        pib_cloud_stream.set_trace_emitter(None)
        pib_cloud_stream.reset_cloud_auth_state()

    assert tokens == ["Hallo"]
    events = [json.loads(raw) for raw in published]
    by_stage = {event["stage"]: event for event in events}
    assert by_stage["request_sent"]["chat_id"] == "chat-wire-1"
    headers = by_stage["response_headers"]["detail"]
    assert headers["upstream_ttft_ms"] == 880
    assert headers["usage_prompt_tokens"] == 120
    assert headers["usage_completion_tokens"] == 88
    assert headers["budget_remaining_tokens"] == 49880
    assert by_stage["first_token"]["stage"] == "first_token"
    assert by_stage["stream_done"]["detail"]["stream_end"] == "done"
    blob = "\n".join(published)
    assert SECRET not in blob
    assert "Bearer" not in blob
