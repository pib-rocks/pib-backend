"""The glued voice path: relay frames in, gated audio out, microphone up.

These tests drive :class:`DuplexVoiceSession` over a fake connection - no socket,
no device, no ROS. What they pin down is the part that was nobody's job before:
which frames may reach the speaker, when the gain drops, and what the robot tells
the cloud when it hears something.
"""

from __future__ import annotations

import json
import queue
import time

import numpy as np

from voice_assistant.duplex_session import DuplexVoiceSession
from voice_assistant.pib_duplex import (
    FRAME_BYTES,
    FRAME_MIC,
    FRAME_TTS,
    encode_frame,
)

FRAME_SAMPLES = FRAME_BYTES // 2


def tone(samples: int = FRAME_SAMPLES, amplitude: int = 8000) -> bytes:
    """One frame of a square wave: loud enough that a gain change is visible."""
    half = samples // 2
    return np.concatenate(
        [
            np.full(half, amplitude, dtype="<i2"),
            np.full(samples - half, -amplitude, dtype="<i2"),
        ]
    ).tobytes()


class FakeConnection:
    def __init__(self) -> None:
        self.sent: list[tuple[str, object]] = []
        self.inbox: queue.Queue = queue.Queue()
        self.closed = False

    # Connection protocol
    def send_text(self, text: str) -> None:
        self.sent.append(("text", json.loads(text)))

    def send_bytes(self, data: bytes) -> None:
        self.sent.append(("bin", data))

    def recv(self):
        return self.inbox.get(timeout=3.0)

    def close(self) -> None:
        self.closed = True
        self.inbox.put((0, b""))  # unblock a waiting recv

    # test helpers
    def push_audio(self, turn_id: int, pcm: bytes, final: bool = False) -> None:
        self.inbox.put((2, encode_frame(FRAME_TTS, turn_id, pcm, final=final)))

    def push_event(self, event: dict) -> None:
        self.inbox.put((1, json.dumps(event)))

    def events(self, kind: str | None = None) -> list[dict]:
        texts = [payload for how, payload in self.sent if how == "text"]
        return [e for e in texts if kind is None or e.get("type") == kind]

    def binary(self) -> list[bytes]:
        return [payload for how, payload in self.sent if how == "bin"]


def make_session(
    **kwargs,
) -> tuple[DuplexVoiceSession, FakeConnection, list[bytes], list[bool]]:
    connection = FakeConnection()
    played: list[bytes] = []
    speaking: list[bool] = []
    events: list[dict] = []
    session = DuplexVoiceSession(
        url="ws://relay.test/v1/pib/stream",
        sink=played.append,
        connect=lambda url, token, timeout: connection,
        on_speaking=speaking.append,
        on_event=events.append,
        progress_interval_s=kwargs.pop("progress_interval_s", 0.02),
        **kwargs,
    )
    session._test_events = events  # type: ignore[attr-defined]
    return session, connection, played, speaking


def wait_for(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def start(session: DuplexVoiceSession, connection: FakeConnection) -> None:
    session.start()
    assert wait_for(lambda: session.connected), "session did not connect"


# --------------------------------------------------------------------------- turn


def test_a_turn_speaks_only_after_the_prebuffer_is_full() -> None:
    session, connection, played, speaking = make_session()
    start(session, connection)
    turn = session.say("Wie spät ist es?")

    # Two frames are less than the 60 ms prebuffer: nothing may reach the device.
    connection.push_audio(turn, tone())
    connection.push_audio(turn, tone())
    assert wait_for(lambda: len(connection.sent) >= 0)
    time.sleep(0.15)
    assert played == []

    connection.push_audio(turn, tone())
    connection.push_audio(turn, tone())
    assert wait_for(lambda: played), "audio never reached the sink"
    assert speaking[0] is True
    session.close()


def test_the_user_text_travels_and_never_the_answer() -> None:
    session, connection, _, _ = make_session()
    start(session, connection)
    session.say("Wie spät ist es?")
    opened = connection.events("turn_start")[0]
    assert opened["text"] == "Wie spät ist es?"
    assert "messages" not in opened
    session.close()


def test_another_turns_audio_never_reaches_the_speaker() -> None:
    session, connection, played, _ = make_session()
    start(session, connection)
    turn = session.say("Hallo")
    connection.push_audio(turn + 1, tone())
    connection.push_audio(turn + 1, tone())
    connection.push_audio(turn + 1, tone())
    time.sleep(0.2)
    assert played == []
    session.close()


def test_the_acknowledged_turn_id_wins_over_the_local_counter() -> None:
    """The server resets its count per connection; the client must follow or go silent."""
    session, connection, played, _ = make_session()
    start(session, connection)
    local = session.say("Hallo")
    assert local == 1
    connection.push_event({"type": "turn_ack", "turn_id": 4, "provider": "elevenlabs"})
    time.sleep(0.1)
    connection.push_audio(4, tone())
    connection.push_audio(4, tone())
    connection.push_audio(4, tone())
    connection.push_audio(4, tone())
    assert wait_for(lambda: played), "audio of the acknowledged turn was dropped"
    session.close()


# --------------------------------------------------------------------------- barge-in


def test_vad_ducks_at_once_and_asks_the_cloud_once() -> None:
    session, connection, played, _ = make_session()
    start(session, connection)
    turn = session.say("Erzähl mir etwas")
    for _ in range(4):
        connection.push_audio(turn, tone())
    assert wait_for(lambda: played)

    session.on_vad(True)
    session.on_vad(True)  # held VAD must not send a second event
    session.on_vad(False)
    session.on_vad(True)

    suspects = connection.events("vad_suspect_barge_in")
    assert len(suspects) == 1
    assert suspects[0]["played_ms"] >= 0
    assert session._gate.gain < 1.0 or session._gate.state == "ducking"
    session.close()


def test_a_rejected_barge_in_resumes_the_same_turn() -> None:
    session, connection, played, _ = make_session()
    start(session, connection)
    turn = session.say("Erzähl mir etwas")
    for _ in range(4):
        connection.push_audio(turn, tone())
    assert wait_for(lambda: played)
    session.on_vad(True)
    connection.push_event({"type": "barge_in_rejected", "turn_id": turn})

    assert wait_for(lambda: session._gate.gain > 0.99), "gain did not come back"
    assert connection.events("turn_end") == []
    assert session.is_speaking is True
    session.close()


def test_a_confirmed_barge_in_ends_the_turn_and_silences_the_speaker() -> None:
    session, connection, played, speaking = make_session()
    start(session, connection)
    turn = session.say("Erzähl mir etwas")
    for _ in range(4):
        connection.push_audio(turn, tone())
    assert wait_for(lambda: played)
    session.on_vad(True)
    connection.push_event({"type": "barge_in_confirmed", "turn_id": turn})

    assert wait_for(lambda: session.is_speaking is False), "turn kept speaking"
    assert speaking == [True, False]
    assert len(connection.events("turn_end")) == 1
    assert session._gate.gain == 0.0

    played_before = len(played)
    connection.push_audio(turn, tone())
    connection.push_audio(turn, tone())
    connection.push_audio(turn, tone())
    time.sleep(0.15)
    assert len(played) == played_before, "audio after the barge-in still played"
    session.close()


def test_turn_complete_ends_the_turn() -> None:
    session, connection, played, _ = make_session()
    start(session, connection)
    turn = session.say("Hallo")
    for _ in range(4):
        connection.push_audio(turn, tone())
    assert wait_for(lambda: played)
    connection.push_event({"type": "turn_complete", "turn_id": turn})
    assert wait_for(lambda: session.is_speaking is False)
    session.close()


def test_a_relay_error_ends_the_turn_and_is_reported() -> None:
    session, connection, _, _ = make_session()
    start(session, connection)
    session.say("Hallo")
    connection.push_event(
        {"type": "error", "code": "tts_unpriced", "message": "no price"}
    )
    assert wait_for(lambda: session.is_speaking is False)
    seen = [e for e in session._test_events if e.get("type") == "error"]  # type: ignore[attr-defined]
    assert seen and seen[0]["code"] == "tts_unpriced"
    session.close()


# --------------------------------------------------------------------------- microphone


def test_the_array_is_sent_as_channel_zero_mono_frames() -> None:
    session, connection, _, _ = make_session(input_channels=6)
    start(session, connection)
    session.say("Hallo")

    # 6 interleaved channels, one frame's worth of samples: channel 0 is a constant 1234.
    channels = 6
    block = np.zeros(FRAME_SAMPLES * channels, dtype="<i2")
    block[0::channels] = 1234
    block[1::channels] = -4321
    sent = session.on_microphone(block.tobytes())

    assert sent == 1
    frame = connection.binary()[0]
    assert frame[0] == FRAME_MIC
    payload = frame[4:]
    assert len(payload) == FRAME_BYTES
    assert np.frombuffer(payload, dtype="<i2").tolist() == [1234] * FRAME_SAMPLES
    session.close()


def test_a_mono_microphone_is_sent_unchanged() -> None:
    session, connection, _, _ = make_session(input_channels=1)
    start(session, connection)
    session.say("Hallo")
    pcm = tone()
    session.on_microphone(pcm)
    assert connection.binary()[0][4:] == pcm
    session.close()


# --------------------------------------------------------------------------- progress


def test_playback_position_is_reported_while_speaking() -> None:
    session, connection, played, _ = make_session(progress_interval_s=0.02)
    start(session, connection)
    turn = session.say("Hallo")
    # The protocol reports every 250 ms of played audio, so cross that line.
    for _ in range(20):
        connection.push_audio(turn, tone())
    assert wait_for(lambda: played)
    assert wait_for(lambda: connection.events("playback_progress"), timeout=1.0)
    progress = connection.events("playback_progress")[-1]
    assert progress["played_ms"] >= 60
    assert progress["turn_id"] == turn
    session.close()


def test_no_progress_is_reported_when_nothing_is_speaking() -> None:
    session, connection, _, _ = make_session(progress_interval_s=0.02)
    start(session, connection)
    time.sleep(0.2)
    assert connection.events("playback_progress") == []
    session.close()


# ------------------------------------------------------------------ echo window


def test_the_microphone_is_held_back_right_after_playback() -> None:
    """The speaker is still audible for a moment after the last frame: pib must not hear itself."""
    session, connection, played, _ = make_session()
    start(session, connection)
    turn = session.say("Hallo")
    for _ in range(4):
        connection.push_audio(turn, tone())
    assert wait_for(lambda: played)

    connection.push_event({"type": "turn_complete", "turn_id": turn})
    assert wait_for(lambda: session.is_speaking is False)

    sent_before = len(connection.binary())
    assert session.on_microphone(tone()) == 0
    assert len(connection.binary()) == sent_before

    time.sleep(0.25)  # the tail window is 200 ms
    assert session.on_microphone(tone()) == 1
    session.close()


def test_a_paused_uplink_sends_nothing_until_it_resumes() -> None:
    session, connection, _, _ = make_session()
    start(session, connection)
    session.say("Hallo")
    session.pause_uplink()
    assert session.on_microphone(tone()) == 0
    session.resume_uplink()
    assert session.on_microphone(tone()) == 1
    session.close()
