"""Choosing a live model is the statement. Typed text still joins that session."""

from pathlib import Path

from pib_hermes_config.live_interaction import (
    live_turn_detection_config,
    typed_text_joins_live,
)
from pib_hermes_config.live_session import GEMINI_LIVE_MODEL

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSISTANT = (
    REPO_ROOT / "ros_packages" / "voice_assistant" / "voice_assistant" / "assistant.py"
)
AUDIO_LOOP = (
    REPO_ROOT / "ros_packages" / "voice_assistant" / "voice_assistant" / "audio_loop.py"
)


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    next_def = source.find("\n    def ", start + len(signature))
    if next_def == -1:
        return source[start:]
    return source[start:next_def]


def test_the_model_list_offers_the_named_live_model(app):
    assistant = app.test_client().get("/assistant-model").get_json()["assistantModels"]
    by_name = {row["visualName"]: row for row in assistant}
    assert by_name["Gemini 3.8 Flash"]["capabilities"]["live"] is False
    live = by_name["Gemini 3.8 Live"]
    assert live["apiName"] == GEMINI_LIVE_MODEL
    assert live["capabilities"]["live"] is True
    assert live["capabilities"]["images"] is False


def test_typed_text_joins_the_open_live_chat_and_leaves_it_interruptible():
    joined = typed_text_joins_live(
        live_open=True,
        live_chat_id="chat-live",
        message_chat_id="chat-live",
        text="  hello there  ",
    )
    assert joined == {
        "join": True,
        "interrupt": True,
        "text": "hello there",
        "keep_session": True,
    }

    other = typed_text_joins_live(
        live_open=True,
        live_chat_id="chat-live",
        message_chat_id="chat-other",
        text="hello there",
    )
    assert other["join"] is False
    assert other["keep_session"] is True

    blank = typed_text_joins_live(
        live_open=True,
        live_chat_id="chat-live",
        message_chat_id="chat-live",
        text="   ",
    )
    assert blank["join"] is False

    closed = typed_text_joins_live(
        live_open=False,
        live_chat_id="chat-live",
        message_chat_id="chat-live",
        text="hello there",
    )
    assert closed["join"] is False
    assert closed["keep_session"] is False

    detection = live_turn_detection_config()["realtime_input_config"]
    assert detection["automatic_activity_detection"]["disabled"] is False
    assert detection["activity_handling"] == "START_OF_ACTIVITY_INTERRUPTS"

    assistant = ASSISTANT.read_text(encoding="utf-8")
    send = _function_body(assistant, "def send_chat_message")
    assert "typed_text_joins_live(" in send
    assert "submit_typed_text(" in send
    assert "if self.gemini_loop.is_listening:\n            return response" not in send

    loop = AUDIO_LOOP.read_text(encoding="utf-8")
    deliver = _function_body(loop, "async def _deliver_typed_text")
    assert "on_interruption(" in deliver
    assert "send_realtime_input(text=text)" in deliver
    assert ".stop(" not in deliver
    assert "activity_start" not in loop
