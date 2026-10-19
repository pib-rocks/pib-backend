"""PC-85: the duplex client - framing, turn filtering, reconnect - without a network.

The socket is injected, so every property here is a property of the client's own
logic: which frames it forwards, which it drops, and what it does when the
connection dies. A real server would add TCP noise to all three.
"""

from __future__ import annotations

import json
import queue
import threading
import time

import pytest

from voice_assistant.pib_duplex import (
    FRAME_MIC,
    FRAME_TTS,
    DuplexClient,
    ProtocolError,
    TurnFilter,
    decode_frame,
    encode_frame,
    pcm_frames,
)


class ConnectionClosed(Exception):
    """What a real socket raises when it is gone."""


class FakeConnection:
    def __init__(self, name: str = "conn") -> None:
        self.name = name
        self.sent: list[tuple[str, object]] = []
        self.inbound: queue.Queue = queue.Queue()
        self.closed = threading.Event()

    # the Connection protocol
    def send_text(self, text: str) -> None:
        self.sent.append(("text", text))

    def send_bytes(self, data: bytes) -> None:
        self.sent.append(("bytes", data))

    def recv(self):
        while True:
            if self.closed.is_set():
                raise ConnectionClosed()
            try:
                return self.inbound.get(timeout=0.02)
            except queue.Empty:
                continue

    def close(self) -> None:
        self.closed.set()

    # test helpers
    def deliver_event(self, event: dict) -> None:
        self.inbound.put((1, json.dumps(event)))

    def deliver_bytes(self, data: bytes) -> None:
        self.inbound.put((2, data))


def wait_until(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


# ----------------------------------------------------------------- frame format


def test_frame_roundtrip() -> None:
    frame = decode_frame(encode_frame(FRAME_TTS, 9, b"\x01\x02" * 4, final=True))
    assert frame.kind == FRAME_TTS
    assert frame.turn_id == 9
    assert frame.final is True
    assert frame.payload == b"\x01\x02" * 4


@pytest.mark.parametrize(
    "data",
    [b"", b"\x01", b"\x07\x00\x01\x00\x00\x00", b"\x01\x00\x01\x00\x00"],
)
def test_malformed_frames_are_refused(data: bytes) -> None:
    with pytest.raises(ProtocolError):
        decode_frame(data)


def test_pcm_is_split_into_full_frames_and_the_last_is_padded() -> None:
    pcm = b"\x11" * (640 * 2 + 100)
    frames = [decode_frame(f) for f in pcm_frames(pcm, FRAME_MIC, 4)]
    assert len(frames) == 3
    assert [f.final for f in frames] == [False, False, True]
    assert all(len(f.payload) == 640 for f in frames)
    assert frames[-1].payload[100:] == b"\x00" * (640 - 100)
    assert {f.turn_id for f in frames} == {4}


def test_turn_filter_drops_everything_but_the_current_turn() -> None:
    turns = TurnFilter()
    assert turns.accepts(decode_frame(encode_frame(FRAME_TTS, 0, b"\x00\x00"))) is False
    assert turns.advance() == 1
    assert turns.accepts(decode_frame(encode_frame(FRAME_TTS, 1, b"\x00\x00"))) is True
    assert turns.accepts(decode_frame(encode_frame(FRAME_TTS, 2, b"\x00\x00"))) is False
    # Microphone audio from the cloud makes no sense and must not be played.
    assert turns.accepts(decode_frame(encode_frame(FRAME_MIC, 1, b"\x00\x00"))) is False


# ----------------------------------------------------------------- client


def make_client(connection: FakeConnection, **kwargs):
    audio: list[tuple[bytes, int, bool]] = []
    events: list[dict] = []
    states: list[str] = []
    client = DuplexClient(
        "ws://test/v1/pib/stream",
        on_audio=lambda payload, turn, final: audio.append((payload, turn, final)),
        on_event=events.append,
        on_state=states.append,
        connect=lambda url, token, timeout: connection,
        ping_interval_s=10.0,
        backoff_initial_s=0.01,
        backoff_max_s=0.02,
        **kwargs,
    )
    return client, audio, events, states


def test_a_turn_opens_with_the_next_id_and_speaks_under_it() -> None:
    connection = FakeConnection()
    client, audio, _, states = make_client(connection)
    client.start()
    try:
        assert wait_until(lambda: client.connected)
        turn = client.send_turn_start("Sag hallo")
        assert turn == 1
        kind, payload = connection.sent[-1]
        assert kind == "text"
        assert json.loads(payload)["turn_id"] == turn

        connection.deliver_bytes(encode_frame(FRAME_TTS, turn, b"\x10" * 640))
        assert wait_until(lambda: audio)
        assert audio[0] == (b"\x10" * 640, turn, False)
        assert client.state == "connected"
        assert "connecting" in states
    finally:
        client.stop()


def test_audio_of_a_finished_turn_never_reaches_the_speaker() -> None:
    connection = FakeConnection()
    client, audio, _, _ = make_client(connection)
    client.start()
    try:
        assert wait_until(lambda: client.connected)
        client.send_turn_start("erster")
        connection.deliver_bytes(encode_frame(FRAME_TTS, 1, b"\x10" * 640))
        assert wait_until(lambda: len(audio) == 1)

        # A barge-in moves the turn on; the old turn's audio is now ghost audio.
        client.send_turn_start("zweiter")
        connection.deliver_bytes(encode_frame(FRAME_TTS, 1, b"\x20" * 640))
        time.sleep(0.1)
        assert len(audio) == 1
        connection.deliver_bytes(encode_frame(FRAME_TTS, 2, b"\x30" * 640))
        assert wait_until(lambda: len(audio) == 2)
        assert audio[1][1] == 2
    finally:
        client.stop()


def test_control_events_reach_the_consumer_and_garbage_does_not() -> None:
    connection = FakeConnection()
    client, _, events, _ = make_client(connection)
    client.start()
    try:
        assert wait_until(lambda: client.connected)
        connection.deliver_event({"type": "barge_in_confirmed", "turn_id": 1})
        assert wait_until(lambda: events)
        assert events[0]["type"] == "barge_in_confirmed"
        connection.inbound.put((1, "not json"))
        connection.inbound.put((2, b"\x01\x00"))  # shorter than a header
        time.sleep(0.1)
        assert len(events) == 1
    finally:
        client.stop()


def test_microphone_audio_is_framed_before_it_is_sent() -> None:
    connection = FakeConnection()
    client, _, _, _ = make_client(connection)
    client.start()
    try:
        assert wait_until(lambda: client.connected)
        client.send_turn_start("hallo")
        sent = client.send_audio(b"\x00\x00" * 500)  # 1000 bytes = 1.5 frames
        assert sent == 2
        binary = [payload for kind, payload in connection.sent if kind == "bytes"]
        assert len(binary) == 2
        assert all(len(frame) == 644 for frame in binary)
    finally:
        client.stop()


def test_a_dropped_connection_is_reconnected() -> None:
    first = FakeConnection("first")
    second = FakeConnection("second")
    connections = [first, second]
    attempts = {"count": 0}

    def connect(url, token, timeout):
        attempts["count"] += 1
        if attempts["count"] == 1:
            raise OSError("host unreachable")
        return connections.pop(0)

    states: list[str] = []
    client = DuplexClient(
        "ws://test/v1/pib/stream",
        on_audio=lambda *_: None,
        on_state=states.append,
        connect=connect,
        ping_interval_s=10.0,
        backoff_initial_s=0.01,
        backoff_max_s=0.02,
    )
    client.start()
    try:
        assert wait_until(lambda: client.connected)
        assert attempts["count"] == 2
        assert "reconnecting" in states or "connecting" in states
        first_now_connected = client.connected
        assert first_now_connected
    finally:
        client.stop()


def test_stop_ends_the_threads_and_the_state() -> None:
    connection = FakeConnection()
    client, _, _, states = make_client(connection)
    client.start()
    assert wait_until(lambda: client.connected)
    client.stop()
    assert client.connected is False
    assert client.state == "stopped"
    assert connection.closed.is_set()
    assert states[-1] == "stopped"


def test_sending_without_a_connection_is_a_no_op_not_an_exception() -> None:
    client = DuplexClient(
        "ws://test",
        on_audio=lambda *_: None,
        connect=lambda *args: FakeConnection(),
        backoff_initial_s=0.01,
        backoff_max_s=0.02,
    )
    assert client.send_turn_start("nichts") == 1
    assert client.send_audio(b"\x00\x00") == 0
    assert client.send_progress(0) is False


def test_the_websocket_adapter_skips_control_frames(monkeypatch) -> None:
    """A pong is two bytes; the frame decoder must never see it."""
    import sys
    import types

    class Abnf:
        OPCODE_TEXT = 1
        OPCODE_BINARY = 2
        OPCODE_CLOSE = 8
        OPCODE_PING = 9
        OPCODE_PONG = 10

    frames = [
        (Abnf.OPCODE_PING, b"keepalive"),
        (Abnf.OPCODE_PONG, b"\x00\x00"),
        (Abnf.OPCODE_TEXT, b'{"type":"pong"}'),
    ]

    class Socket:
        def recv_data(self):
            return frames.pop(0)

        def send(self, *args):
            pass

        def send_binary(self, *args):
            pass

        def close(self):
            pass

    fake = types.SimpleNamespace(ABNF=Abnf, create_connection=lambda *a, **k: Socket())
    monkeypatch.setitem(sys.modules, "websocket", fake)

    from voice_assistant.pib_duplex import websocket_connection

    connection = websocket_connection("wss://relay.test/v1/pib/stream", None, 5.0)
    assert connection.recv() == (Abnf.OPCODE_TEXT, '{"type":"pong"}')
    assert frames == []


def test_a_closed_socket_ends_the_receive_loop(monkeypatch) -> None:
    import sys
    import types

    class Abnf:
        OPCODE_TEXT = 1
        OPCODE_BINARY = 2
        OPCODE_CLOSE = 8
        OPCODE_PING = 9
        OPCODE_PONG = 10

    class Socket:
        def recv_data(self):
            return Abnf.OPCODE_CLOSE, b""

        def close(self):
            pass

    fake = types.SimpleNamespace(ABNF=Abnf, create_connection=lambda *a, **k: Socket())
    monkeypatch.setitem(sys.modules, "websocket", fake)

    from voice_assistant.pib_duplex import websocket_connection

    connection = websocket_connection("wss://relay.test/v1/pib/stream", None, 5.0)
    with pytest.raises(ConnectionError):
        connection.recv()
