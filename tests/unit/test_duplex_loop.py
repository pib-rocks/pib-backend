"""The turn cycle of the relay voice path, without ROS and without a sound card.

The cycle is the part the user feels: after every answer pib listens again, and an
interruption starts the next question instead of finishing the interrupted one.
"""

from __future__ import annotations

from voice_assistant.duplex_loop import (
    STATE_IDLE,
    STATE_LISTENING,
    STATE_SPEAKING,
    DuplexVoiceLoop,
)


class FakeSession:
    def __init__(self) -> None:
        self.said: list[str] = []
        self.started = False
        self.closed = False
        self.on_turn_end = None

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.closed = True

    def say(self, text: str) -> int:
        self.said.append(text)
        return len(self.said)

    def close(self) -> None:
        self.closed = True

    # test helpers
    def finish_turn(self, reason: str = "complete") -> None:
        if self.on_turn_end is not None:
            self.on_turn_end(reason)


class FakeRecorder:
    """Hands out the transcription callback, like the action client does."""

    def __init__(self) -> None:
        self.requests = 0
        self.callback = None

    def request(self, on_text) -> None:
        self.requests += 1
        self.callback = on_text

    def answer(self, text: str) -> None:
        callback = self.callback
        assert callback is not None, "no transcription was requested"
        self.callback = None
        callback(text)


def make(idle_timeout_s=None):
    session = FakeSession()
    recorder = FakeRecorder()
    speaking: list[bool] = []
    loop = DuplexVoiceLoop(
        session=session,  # type: ignore[arg-type]
        request_transcription=recorder.request,
        on_speaking=speaking.append,
        idle_timeout_s=idle_timeout_s,
    )
    session.on_turn_end = loop.__dict__.get("_noop")  # replaced below
    # connect the session's turn-end hook to the loop the way ros_duplex does
    session.on_turn_end = lambda reason: (
        loop.on_barge_in() if reason.startswith("barge-in") else loop.on_turn_finished()
    )
    return loop, session, recorder, speaking


def test_start_connects_and_asks_for_an_utterance() -> None:
    loop, session, recorder, _ = make()
    loop.start(chat_id="chat-1")
    assert session.started is True
    assert recorder.requests == 1
    assert loop.state == STATE_LISTENING
    assert loop.is_listening is True


def test_a_transcribed_utterance_is_said_and_then_listens_again() -> None:
    loop, session, recorder, speaking = make()
    loop.start()
    recorder.answer("Wie spät ist es?")
    assert session.said == ["Wie spät ist es?"]
    assert loop.state == STATE_SPEAKING
    assert speaking == [True]

    session.finish_turn("complete")
    assert recorder.requests == 2
    assert loop.state == STATE_LISTENING
    assert speaking == [True, False]


def test_silence_keeps_listening_instead_of_ending_the_conversation() -> None:
    loop, session, recorder, _ = make()
    loop.start()
    recorder.answer("")
    assert session.said == []
    assert recorder.requests == 2
    assert loop.is_listening is True


def test_the_idle_timeout_closes_a_silent_session() -> None:
    loop, session, recorder, _ = make(idle_timeout_s=0.0)
    loop.start()
    recorder.answer("")
    assert session.closed is True
    assert loop.is_listening is False


def test_a_confirmed_barge_in_listens_again_without_finishing_the_old_turn() -> None:
    loop, session, recorder, _ = make()
    loop.start()
    recorder.answer("Erzähl mir einen Witz")
    assert loop.state == STATE_SPEAKING

    session.finish_turn("barge-in confirmed")
    assert recorder.requests == 2
    assert loop.state == STATE_LISTENING
    assert session.said == ["Erzähl mir einen Witz"]


def test_a_typed_message_takes_the_same_path() -> None:
    loop, session, _, _ = make()
    loop.start()
    assert loop.submit_typed_text("Hallo pib") is True
    assert session.said == ["Hallo pib"]


def test_a_typed_message_before_start_is_refused() -> None:
    loop, session, _, _ = make()
    assert loop.submit_typed_text("Hallo pib") is False
    assert session.said == []


def test_stop_closes_the_session_and_does_not_listen_on() -> None:
    loop, session, recorder, _ = make()
    loop.start()
    loop.stop()
    assert session.closed is True
    assert loop.is_listening is False
    assert loop.state == STATE_IDLE
    assert recorder.requests == 1
