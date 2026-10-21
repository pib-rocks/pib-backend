"""Stubs for the ROS modules the voice runtime imports, so tests run without ROS.

The runtime is a ROS node: importing it pulls rclpy, std_msgs and the generated
``datatypes`` messages, none of which exist in a plain test environment. Every
test module that needs the real code paths (not a fake of the node) installs
these first, then imports ``voice_assistant.chat``.
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock

TEXT_TYPE_SENTENCE = 0
TEXT_TYPE_CODE_VISUAL = 1


def install_ros_stubs():
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
        vac.record_first_token_latency = MagicMock(return_value=False)

    if "public_api_client.public_voice_client" not in sys.modules:
        pvc = ensure("public_api_client.public_voice_client")

        class PublicApiChatMessage:
            def __init__(self, content, is_user):
                self.content = content
                self.is_user = is_user

        pvc.PublicApiChatMessage = PublicApiChatMessage
        pvc.chat_completion = MagicMock()
