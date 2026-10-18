"""Persistent duplex client for the pib.Cloud voice relay (PC-85).

One connection per conversation, audio in both directions, control events on the
same socket. The relay (PC-80) fixes the format: binary frames are
``kind (1) | turn_id (2, big endian) | flags (1) | pcm_s16le`` at 16 kHz mono in
20 ms frames, text frames are JSON control events.

Two decisions shape this module:

* **The turn id is the ghost-audio defence.** Every downlink frame names the turn
  it belongs to. :class:`TurnFilter` compares it with the turn the robot is in and
  drops everything else before it can reach the speaker. After a barge-in the
  robot advances the turn and the old audio becomes unreachable with one integer
  comparison - no queue drain, no per-frame bookkeeping.
* **The socket is a thread, not a task.** The voice stack is synchronous: pyaudio
  callbacks and ROS callbacks own their threads. A client that needs an event loop
  would force the audio path to hop threads for every frame, so the receive loop
  and the ping live in ordinary threads and hand frames over through callbacks.

The network is taken as an injected factory (:class:`ConnectionFactory`) instead
of being created inside the client. That is what makes reconnect and frame
handling testable without a server, and it keeps the websocket dependency at one
adapter (:func:`websocket_connection`) instead of spreading it through the class.
"""

from __future__ import annotations

import json
import logging
import random
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Protocol

logger = logging.getLogger(__name__)

FRAME_MIC = 0x01
"""Robot to cloud: microphone audio."""
FRAME_TTS = 0x02
"""Cloud to robot: synthesised speech."""

SAMPLE_RATE = 16_000
SAMPLE_BYTES = 2
FRAME_MS = 20
FRAME_BYTES = 640  # 320 samples at 16 kHz mono 16-bit
HEADER_BYTES = 4
TURN_MODULUS = 0xFFFF

DEFAULT_PING_INTERVAL_S = 5.0
DEFAULT_BACKOFF_INITIAL_S = 0.5
DEFAULT_BACKOFF_MAX_S = 10.0

STATE_CONNECTING = "connecting"
STATE_CONNECTED = "connected"
STATE_RECONNECTING = "reconnecting"
STATE_STOPPED = "stopped"


class ProtocolError(ValueError):
    """A frame this protocol cannot accept."""


@dataclass(frozen=True)
class Frame:
    kind: int
    turn_id: int
    final: bool
    payload: bytes


def encode_frame(
    kind: int, turn_id: int, payload: bytes, *, final: bool = False
) -> bytes:
    """One binary frame: ``kind | turn_id | flags | pcm``."""
    if kind not in (FRAME_MIC, FRAME_TTS):
        raise ProtocolError(f"unknown frame type {kind!r}")
    if len(payload) > 0xFFFF:
        raise ProtocolError("audio payload does not fit one frame")
    turn = turn_id % TURN_MODULUS if turn_id > 0 else 0
    return (
        bytes((kind, (turn >> 8) & 0xFF, turn & 0xFF, 0x01 if final else 0x00))
        + payload
    )


def decode_frame(data: bytes) -> Frame:
    """Parse one binary frame, refusing anything that is not header plus whole samples."""
    if len(data) < HEADER_BYTES:
        raise ProtocolError(f"frame is {len(data)} bytes, shorter than the header")
    kind, high, low, flags = data[0], data[1], data[2], data[3]
    if kind not in (FRAME_MIC, FRAME_TTS):
        raise ProtocolError(f"unknown frame type {kind!r}")
    payload = data[HEADER_BYTES:]
    if len(payload) % SAMPLE_BYTES:
        raise ProtocolError("payload is not a whole number of 16-bit samples")
    return Frame(
        kind=kind, turn_id=(high << 8) | low, final=bool(flags & 0x01), payload=payload
    )


def pcm_frames(
    pcm: bytes, kind: int, turn_id: int, *, final: bool = True
) -> Iterable[bytes]:
    """Split PCM into 640-byte frames, padding the last one with silence.

    A short final frame would be a special case in the audio callback, which is the
    last place that should have one; the robot's ring buffer always sees 20 ms.
    """
    if not pcm:
        return []
    frames = []
    for offset in range(0, len(pcm), FRAME_BYTES):
        chunk = pcm[offset : offset + FRAME_BYTES]
        is_last = offset + FRAME_BYTES >= len(pcm)
        if len(chunk) < FRAME_BYTES:
            chunk = chunk + b"\x00" * (FRAME_BYTES - len(chunk))
        frames.append(encode_frame(kind, turn_id, chunk, final=final and is_last))
    return frames


class TurnFilter:
    """Only audio of the turn the robot is currently in may reach the speaker."""

    def __init__(self, turn_id: int = 0) -> None:
        self._turn_id = turn_id
        self._open = False

    @property
    def turn_id(self) -> int:
        return self._turn_id

    def advance(self) -> int:
        """Move to the next turn and return it - the one call a barge-in needs."""
        self._turn_id = (self._turn_id + 1) % TURN_MODULUS or 1
        self._open = True
        return self._turn_id

    def accepts(self, frame: Frame) -> bool:
        """Nothing plays before the first turn: no open turn means no audio is wanted."""
        return self._open and frame.kind == FRAME_TTS and frame.turn_id == self._turn_id


class Connection(Protocol):
    """The slice of a websocket this client needs."""

    def send_text(self, text: str) -> None: ...

    def send_bytes(self, data: bytes) -> None: ...

    def recv(self) -> tuple[int, bytes | str]: ...

    def close(self) -> None: ...


ConnectionFactory = Callable[[str, Optional[str], float], Connection]


def websocket_connection(url: str, token: Optional[str], timeout: float) -> Connection:
    """Production adapter over ``websocket-client``.

    Imported lazily so the module stays importable (and testable) without the
    dependency, and so a missing wheel fails at connect time with a clear message
    instead of at import.
    """
    import websocket  # type: ignore[import-not-found]

    headers = {"Authorization": f"Bearer {token}"} if token else None
    socket = websocket.create_connection(url, timeout=timeout, header=headers)

    class _Adapter:
        def send_text(self, text: str) -> None:
            socket.send(text)

        def send_bytes(self, data: bytes) -> None:
            socket.send_binary(data)

        def recv(self):  # type: ignore[no-untyped-def]
            opcode, payload = socket.recv_data()
            if opcode == websocket.ABNF.OPCODE_TEXT:
                return opcode, payload.decode("utf-8", "replace")
            return opcode, payload

        def close(self) -> None:
            try:
                socket.close()
            except Exception:  # noqa: BLE001 - closing a dead socket is not an error
                pass

    return _Adapter()


class DuplexClient:
    """The robot's half of the relay: send frames, receive frames, survive a drop.

    ``on_audio(payload, turn_id, final)`` receives playable 20 ms PCM of the
    current turn only. ``on_event(dict)`` receives every control event. Both are
    called from the receive thread and must not block: hand off to the audio queue.
    """

    def __init__(
        self,
        url: str,
        *,
        on_audio: Callable[[bytes, int, bool], None],
        on_event: Optional[Callable[[dict], None]] = None,
        on_state: Optional[Callable[[str], None]] = None,
        connect: ConnectionFactory = websocket_connection,
        token: Optional[str] = None,
        connect_timeout: float = 10.0,
        ping_interval_s: float = DEFAULT_PING_INTERVAL_S,
        backoff_initial_s: float = DEFAULT_BACKOFF_INITIAL_S,
        backoff_max_s: float = DEFAULT_BACKOFF_MAX_S,
    ) -> None:
        self._url = url
        self._on_audio = on_audio
        self._on_event = on_event or (lambda event: None)
        self._on_state = on_state or (lambda state: None)
        self._connect = connect
        self._token = token
        self._connect_timeout = connect_timeout
        self._ping_interval_s = ping_interval_s
        self._backoff_initial_s = backoff_initial_s
        self._backoff_max_s = backoff_max_s
        self._connection: Optional[Connection] = None
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._ping_thread: Optional[threading.Thread] = None
        self._turns = TurnFilter()
        self.state = STATE_STOPPED

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> None:
        """Connect in the background and keep reconnecting until :meth:`stop`."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="pib-duplex", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        with self._send_lock:
            connection, self._connection = self._connection, None
        if connection is not None:
            connection.close()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None
        if self._ping_thread is not None:
            self._ping_thread.join(timeout)
            self._ping_thread = None
        self._set_state(STATE_STOPPED)

    @property
    def connected(self) -> bool:
        return self._connection is not None

    @property
    def turn_id(self) -> int:
        return self._turns.turn_id

    def _set_state(self, state: str) -> None:
        self.state = state
        try:
            self._on_state(state)
        except Exception:  # noqa: BLE001 - an observer must not kill the loop
            logger.exception("state observer failed")

    def _run(self) -> None:
        backoff = self._backoff_initial_s
        while not self._stop.is_set():
            self._set_state(
                STATE_CONNECTING
                if backoff == self._backoff_initial_s
                else STATE_RECONNECTING
            )
            try:
                connection = self._connect(
                    self._url, self._token, self._connect_timeout
                )
            except Exception as exc:  # noqa: BLE001 - any failure is a retryable one
                logger.warning("duplex connect failed: %s", exc)
                self._sleep_backoff(backoff)
                backoff = min(backoff * 2, self._backoff_max_s)
                continue
            with self._send_lock:
                self._connection = connection
            backoff = self._backoff_initial_s
            self._set_state(STATE_CONNECTED)
            self._start_ping()
            try:
                self._receive_loop(connection)
            except Exception as exc:  # noqa: BLE001 - a dropped socket is normal
                if not self._stop.is_set():
                    logger.warning("duplex receive ended: %s", exc)
            finally:
                with self._send_lock:
                    if self._connection is connection:
                        self._connection = None
                connection.close()
                self._stop_ping()
            if self._stop.is_set():
                break
            self._set_state(STATE_RECONNECTING)
            self._sleep_backoff(backoff)
            backoff = min(backoff * 2, self._backoff_max_s)

    def _sleep_backoff(self, seconds: float) -> None:
        # Jitter keeps a fleet of robots from reconnecting in lockstep after an outage.
        self._stop.wait(seconds * random.uniform(0.5, 1.5))

    # ------------------------------------------------------------------ receive

    def _receive_loop(self, connection: Connection) -> None:
        while not self._stop.is_set():
            opcode, payload = connection.recv()
            if isinstance(payload, str):
                self._handle_event(payload)
                continue
            self._handle_audio(payload)

    def _handle_audio(self, payload: bytes) -> None:
        try:
            frame = decode_frame(payload)
        except ProtocolError as exc:
            logger.warning("dropping malformed audio frame: %s", exc)
            return
        if not self._turns.accepts(frame):
            return
        try:
            self._on_audio(frame.payload, frame.turn_id, frame.final)
        except Exception:  # noqa: BLE001 - a consumer bug must not drop the socket
            logger.exception("audio consumer failed")

    def _handle_event(self, text: str) -> None:
        try:
            event = json.loads(text)
        except json.JSONDecodeError:
            logger.warning("dropping non-JSON control frame")
            return
        if not isinstance(event, dict):
            return
        try:
            self._on_event(event)
        except Exception:  # noqa: BLE001
            logger.exception("event consumer failed")

    # ------------------------------------------------------------------ send

    def _send(self, data: bytes) -> bool:
        with self._send_lock:
            connection = self._connection
            if connection is None:
                return False
            try:
                connection.send_bytes(data)
                return True
            except Exception as exc:  # noqa: BLE001 - the receive loop will notice
                logger.warning("duplex send failed: %s", exc)
                return False

    def _send_event(self, event: dict) -> bool:
        with self._send_lock:
            connection = self._connection
            if connection is None:
                return False
            try:
                connection.send_text(json.dumps(event, separators=(",", ":")))
                return True
            except Exception as exc:  # noqa: BLE001
                logger.warning("duplex event send failed: %s", exc)
                return False

    def send_turn_start(self, text: str) -> int:
        """Open a turn and return the id the robot will speak under."""
        turn = self._turns.advance()
        self._send_event({"type": "turn_start", "turn_id": turn, "text": text})
        return turn

    def send_turn_end(self) -> bool:
        return self._send_event({"type": "turn_end", "turn_id": self._turns.turn_id})

    def send_ack(self) -> bool:
        return self._send_event(
            {"type": "barge_in_ack", "turn_id": self._turns.turn_id}
        )

    def send_progress(self, played_ms: float) -> bool:
        """Report how much of the turn the robot has played."""
        return self._send_event(
            {
                "type": "playback_progress",
                "turn_id": self._turns.turn_id,
                "played_ms": played_ms,
            }
        )

    def send_suspect_barge_in(
        self, played_ms: float, doa_angle: Optional[int] = None
    ) -> bool:
        event = {
            "type": "vad_suspect_barge_in",
            "turn_id": self._turns.turn_id,
            "played_ms": played_ms,
        }
        if doa_angle is not None:
            event["doa_angle"] = int(doa_angle) % 360
        return self._send_event(event)

    def send_audio(self, pcm: bytes) -> int:
        """Send one chunk of microphone PCM as 20 ms frames; returns the frame count."""
        frames = pcm_frames(pcm, FRAME_MIC, self._turns.turn_id)
        sent = 0
        for frame in frames:
            if not self._send(frame):
                break
            sent += 1
        return sent

    # ------------------------------------------------------------------ ping

    def _start_ping(self) -> None:
        self._stop_ping()
        self._ping_thread = threading.Thread(
            target=self._ping_loop, name="pib-duplex-ping", daemon=True
        )
        self._ping_thread.start()

    def _stop_ping(self) -> None:
        thread, self._ping_thread = self._ping_thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(self._ping_interval_s + 1.0)

    def _ping_loop(self) -> None:
        connection = self._connection
        while not self._stop.is_set() and connection is not None:
            if self._stop.wait(self._ping_interval_s):
                return
            with self._send_lock:
                if self._connection is not connection:
                    return
                try:
                    connection.send_text('{"type":"ping"}')
                except Exception:  # noqa: BLE001
                    return
