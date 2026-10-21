"""PC-119: a picture a tool captured reaches the chat as a reference, not as bytes.

Two halves are covered here:

* the **collection** - only a picture is picked out of a tool result, it is scaled
  down before it leaves the robot, and an undecodable payload never takes a turn
  down (the answer is simply stored without a picture);
* the **hand-over** - the bytes are uploaded to the chat API once the answer has
  an id, and the message meta carries what the API answered, never the payload.

The scaling itself runs through Pillow, which the runtime image has and a test
environment does not; a small stand-in module exercises the size arithmetic and
the re-encode without a decoder.
"""

from __future__ import annotations

import base64
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from ros_stubs import install_ros_stubs

install_ros_stubs()


# --- a stand-in for the parts of Pillow this code uses ---------------------


class _FakeImage:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

    @property
    def size(self):
        return (self.width, self.height)

    def convert(self, mode):
        assert mode == "RGB"
        return self

    def resize(self, size, method):
        return _FakeImage(int(size[0]), int(size[1]))

    def save(self, buffer, format, quality):
        assert format == "JPEG"
        buffer.write(b"\xff\xd8\xff" + f"{self.width}x{self.height}q{quality}".encode())
        buffer.write(b"\xff\xd9")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


@pytest.fixture()
def fake_pillow(monkeypatch):
    """Install a Pillow stand-in and answer the sizes it was asked to open."""

    class _Module:
        def __init__(self):
            self.opened: list[tuple[int, int]] = []
            self.Image = types.SimpleNamespace(open=self._open, LANCZOS=1)

        def _open(self, stream):
            width, height = _decoded_size(stream.getvalue())
            self.opened.append((width, height))
            return _FakeImage(width, height)

    module = _Module()
    monkeypatch.setitem(sys.modules, "PIL", module)
    monkeypatch.setitem(sys.modules, "PIL.Image", module.Image)
    return module


def _decoded_size(raw: bytes) -> tuple[int, int]:
    """Read ``WxH`` back out of the payload a test handed to the stand-in."""
    text = raw.decode("ascii", "replace")
    if "x" not in text:
        raise ValueError("not a fixture")
    width, _, rest = text.partition("x")
    height = rest.split("q")[0]
    return int(width), int(height)


def _payload(width: int, height: int) -> str:
    return base64.b64encode(f"{width}x{height}".encode()).decode("ascii")


# --- scaling ---------------------------------------------------------------


def test_without_pillow_a_picture_is_simply_skipped(chat_module):
    """The voice matters more than the picture: no Pillow, no attachment, no crash."""
    sys.modules.pop("PIL", None)
    sys.modules.pop("PIL.Image", None)

    assert chat_module.prepare_tool_image(_payload(640, 480)) is None


def test_a_large_frame_is_scaled_down_to_the_long_edge(chat_module, fake_pillow):
    prepared = chat_module.prepare_tool_image(_payload(2048, 1536))

    assert prepared["width"] == 1024
    assert prepared["height"] == 768
    assert prepared["mimeType"] == "image/jpeg"
    assert base64.b64decode(prepared["imageBase64"]).startswith(b"\xff\xd8\xff")


def test_a_small_picture_keeps_its_size(chat_module, fake_pillow):
    prepared = chat_module.prepare_tool_image(_payload(320, 240))

    assert (prepared["width"], prepared["height"]) == (320, 240)


def test_a_portrait_frame_is_scaled_by_its_taller_edge(chat_module, fake_pillow):
    prepared = chat_module.prepare_tool_image(_payload(600, 1600))

    assert prepared["height"] == 1024
    assert prepared["width"] == 384


def test_a_payload_that_is_not_an_image_is_skipped(chat_module):
    assert chat_module.prepare_tool_image("not-base64-at-all") is None
    assert chat_module.prepare_tool_image("") is None
    assert chat_module.prepare_tool_image(None) is None


# --- only a picture is reported -------------------------------------------


def test_the_tool_reporter_hands_over_only_a_picture(tool_loop_module):
    seen: list[dict] = []

    tool_loop_module._report_tool_image(
        seen.append, "capture_image", {"imageBase64": "AAA", "mimeType": "image/jpeg"}
    )
    # A motor position, a program list, an empty answer: none of them is a picture.
    tool_loop_module._report_tool_image(
        seen.append, "move_motor", {"motorName": "elbow_right", "position": 10}
    )
    tool_loop_module._report_tool_image(
        seen.append, "capture_image", {"imageBase64": ""}
    )
    tool_loop_module._report_tool_image(seen.append, "capture_image", "a string")

    assert seen == [
        {"tool": "capture_image", "mimeType": "image/jpeg", "imageBase64": "AAA"}
    ]


def test_a_picture_without_a_mime_type_is_a_jpeg(tool_loop_module):
    seen: list[dict] = []

    tool_loop_module._report_tool_image(
        seen.append, "capture_image", {"imageBase64": "AAA"}
    )

    assert seen[0]["mimeType"] == "image/jpeg"


# --- the collection on the turn ------------------------------------------


def test_a_captured_picture_is_kept_on_the_turn(chat_module, chat_node, monkeypatch):
    monkeypatch.setattr(
        chat_module,
        "prepare_tool_image",
        lambda data, mime: {
            "imageBase64": "scaled",
            "mimeType": "image/jpeg",
            "width": 8,
            "height": 6,
        },
    )
    chat_node._reset_turn_meta()

    chat_node._record_tool_image(
        {"tool": "capture_image", "imageBase64": "raw", "mimeType": "image/jpeg"}
    )

    assert chat_node._turn_images == [
        {
            "imageBase64": "scaled",
            "mimeType": "image/jpeg",
            "width": 8,
            "height": 6,
            "tool": "capture_image",
        }
    ]


def test_a_picture_that_cannot_be_prepared_is_dropped(
    chat_module, chat_node, monkeypatch
):
    monkeypatch.setattr(chat_module, "prepare_tool_image", lambda data, mime: None)
    chat_node._reset_turn_meta()

    chat_node._record_tool_image({"tool": "capture_image", "imageBase64": "raw"})

    assert chat_node._turn_images == []


def test_the_upload_needs_the_stored_answer(
    chat_module, chat_node, chat_module_client, monkeypatch
):
    """Without a message id there is nothing to attach a picture to."""
    chat_node._reset_turn_meta()
    chat_node._turn_images = [{"imageBase64": "scaled", "mimeType": "image/jpeg"}]
    chat_node.last_pib_message_id = None

    chat_node._attach_turn_images("chat-1")

    chat_module_client.add_chat_message_image.assert_not_called()


def test_the_upload_turns_into_the_reference_the_meta_carries(
    chat_module, chat_node, chat_module_client, monkeypatch
):
    chat_node._reset_turn_meta()
    chat_node._turn_images = [
        {
            "imageBase64": "scaled-one",
            "mimeType": "image/jpeg",
            "width": 8,
            "height": 6,
            "tool": "capture_image",
        }
    ]
    chat_node.last_pib_message_id = "message-1"
    chat_module_client.add_chat_message_image.return_value = {
        "id": "image-1",
        "mimeType": "image/jpeg",
        "width": 8,
        "height": 6,
        "url": "/voice-assistant/chat/chat-1/messages/message-1/images/0",
    }

    chat_node._attach_turn_images("chat-1")
    chat_node._turn_path_token = "direct-tool-loop"
    meta = chat_node._turn_meta()

    chat_module_client.add_chat_message_image.assert_called_once_with(
        "chat-1", "message-1", "scaled-one"
    )
    assert meta["images"] == [
        {
            "id": "image-1",
            "mimeType": "image/jpeg",
            "width": 8,
            "height": 6,
            "url": "/voice-assistant/chat/chat-1/messages/message-1/images/0",
        }
    ]
    # The bytes are not what the message keeps.
    assert "scaled-one" not in str(meta)


def test_a_refused_picture_leaves_the_answer_readable(
    chat_module, chat_node, chat_module_client
):
    chat_node._reset_turn_meta()
    chat_node._turn_images = [{"imageBase64": "scaled", "mimeType": "image/jpeg"}]
    chat_node.last_pib_message_id = "message-1"
    chat_module_client.add_chat_message_image.return_value = None

    chat_node._attach_turn_images("chat-1")
    chat_node._turn_path_token = "direct-tool-loop"

    assert "images" not in chat_node._turn_meta()


def test_a_failing_upload_does_not_fail_the_turn(
    chat_module, chat_node, chat_module_client
):
    chat_node._reset_turn_meta()
    chat_node._turn_images = [{"imageBase64": "scaled", "mimeType": "image/jpeg"}]
    chat_node.last_pib_message_id = "message-1"
    chat_module_client.add_chat_message_image.side_effect = RuntimeError("api down")

    chat_node._attach_turn_images("chat-1")
    chat_node._turn_path_token = "direct-tool-loop"

    assert chat_node._turn_meta() == {"source": "direct-tool-loop"}


# --- fixtures -------------------------------------------------------------


@pytest.fixture()
def chat_module():
    for name in ("voice_assistant.chat", "voice_assistant"):
        sys.modules.pop(name, None)
    from voice_assistant import chat as module

    return module


@pytest.fixture()
def tool_loop_module():
    from voice_assistant import direct_tool_loop as module

    return module


@pytest.fixture()
def chat_module_client(chat_module, monkeypatch):
    """The client module the runtime talks to, with its image call replaced."""
    client = MagicMock()
    client.add_chat_message_image = MagicMock()
    monkeypatch.setattr(chat_module, "voice_assistant_client", client)
    return client


@pytest.fixture()
def chat_node(chat_module):
    node = chat_module.ChatNode.__new__(chat_module.ChatNode)
    node.get_logger = MagicMock(return_value=MagicMock())
    node.voice_assistant_client_lock = MagicMock()
    node.voice_assistant_client_lock.__enter__ = MagicMock(return_value=None)
    node.voice_assistant_client_lock.__exit__ = MagicMock(return_value=False)
    node._reset_turn_meta()
    return node
