"""The robot side of the duplex voice relay, assembled from its four parts.

The pieces exist separately and are tested separately: the client (PC-85) moves
frames, the playback clock (PC-87) decides what may be written, the speech gate
(PC-86) decides how loud, the device layer (PC-88) decides which channel and which
HID switches. None of them talks to the others by itself. This module is that
conversation, and nothing else:

    mic bytes ──► DuplexClient ──► cloud
    cloud ──► DuplexClient ──► PlaybackClock ──► SpeechGate ──► sink (the speaker)

**Barge-in is two stages at two different owners.** The local stage must be
instant and must not need the network: the array's VAD flips, the gate ducks to
15 % over 5 ms, one control event leaves. The second stage belongs to the cloud,
because only the cloud can tell a cough from a word: ``barge_in_rejected`` resumes
the same sample position, ``barge_in_confirmed`` ends the turn. The robot never
decides on its own to stop playing speech that the user did not interrupt - that
is the difference between a duck and a stutter.

**The turn id is re-synced, not counted.** Both sides count turns, and the server
resets its counter per connection while the robot's client outlives reconnects. A
robot that kept its own count would silently drop every frame after the first
reconnect, so the acknowledgement is authoritative: ``adopt(turn_id)``.
"""

from __future__ import annotations

import logging
import threading
from typing import Callable, Optional

import numpy as np

from voice_assistant.audio_devices import PROCESSED_CHANNEL, extract_channel
from voice_assistant.pib_duplex import (
    ConnectionFactory,
    DuplexClient,
    websocket_connection,
)
from voice_assistant.playback_clock import PlaybackClock
from voice_assistant.speech_gate import STATE_SPEAKING, SpeechGate

logger = logging.getLogger(__name__)

DEFAULT_PROGRESS_INTERVAL_S = 0.1


class DuplexVoiceSession:
    """One relay connection, one microphone, one speaker, and the rules between them.

    ``sink`` receives the PCM that may be written to the device *now*, already
    gated and ramped: normally a blocking ``stream.write``. It is called from the
    client's receive thread, so it must not block for longer than the audio - which
    a 20 ms write does not.

    ``on_speaking(bool)`` fires when the robot starts and stops producing speech;
    the personality and the LED ring hang off that.
    """

    def __init__(
        self,
        *,
        url: str,
        sink: Callable[[bytes], None],
        token: Optional[str] = None,
        connect: ConnectionFactory = websocket_connection,
        clock: Optional[PlaybackClock] = None,
        gate: Optional[SpeechGate] = None,
        on_event: Optional[Callable[[dict], None]] = None,
        on_speaking: Optional[Callable[[bool], None]] = None,
        on_turn_end: Optional[Callable[[str], None]] = None,
        on_state: Optional[Callable[[str], None]] = None,
        input_channels: int = 1,
        progress_interval_s: float = DEFAULT_PROGRESS_INTERVAL_S,
    ) -> None:
        self._sink = sink
        self._input_channels = input_channels
        self._clock = clock or PlaybackClock()
        self._gate = gate or SpeechGate()
        self._on_event = on_event
        self._on_speaking = on_speaking
        self._on_turn_end = on_turn_end
        self._speaking = False
        self._progress_stop = threading.Event()
        self._progress_thread: Optional[threading.Thread] = None
        self._progress_interval_s = progress_interval_s
        self._suspected = False
        self._accepting = False
        self._dropped_turns = 0
        self.client = DuplexClient(
            url,
            on_audio=self._on_audio,
            on_event=self._handle_event,
            on_state=on_state,
            connect=connect,
            token=token,
        )

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> None:
        self.client.start()
        self._start_progress()

    def stop(self) -> None:
        self._stop_progress()
        self.client.stop()

    @property
    def connected(self) -> bool:
        return self.client.connected

    @property
    def is_speaking(self) -> bool:
        return self._speaking

    def close(self) -> None:
        """End any turn that is still playing, then hang up."""
        self._end_turn(reason="closed")
        self.stop()

    # ------------------------------------------------------------------ speaking

    def say(self, text: str) -> int:
        """Speak ``text`` through the cloud and return the turn it belongs to.

        The text is what the *user* said: the relay owns the answer, the model
        choice and the synthesis, which is why the robot no longer needs a local
        TTS engine on this path.
        """
        self._gate.begin_turn()
        self._suspected = False
        self._accepting = True
        turn = self.client.send_turn_start(text)
        self._clock.begin_turn(turn)
        self._set_speaking(True)
        return turn

    def _on_audio(self, payload: bytes, turn_id: int, final: bool) -> None:
        # After a confirmed barge-in the rest of that turn may still arrive: the
        # socket is ahead of the decision. Nothing of it belongs to the user any
        # more, and writing its gated-to-silence frames would keep the device busy.
        if not self._accepting:
            return
        try:
            for rendered in self._clock.feed(payload):
                gated = self._gate.process(rendered)
                if gated:
                    self._sink(gated)
        except Exception:  # noqa: BLE001 - a bad frame must not kill the socket
            logger.exception("playback failed for turn %s", turn_id)

    def _set_speaking(self, speaking: bool) -> None:
        if speaking == self._speaking:
            return
        self._speaking = speaking
        listener = self._on_speaking
        if listener is not None:
            try:
                listener(speaking)
            except Exception:  # noqa: BLE001 - an observer must not break audio
                logger.exception("speaking listener failed")

    def _end_turn(self, *, reason: str) -> None:
        was_accepting = self._accepting
        self._accepting = False
        self._gate.cancel()
        self._suspected = False
        self._set_speaking(False)
        logger.debug("turn ended: %s", reason)
        listener = self._on_turn_end
        if listener is not None and was_accepting:
            try:
                listener(reason)
            except Exception:  # noqa: BLE001 - the loop's problem, not the socket's
                logger.exception("turn-end listener failed")

    # ------------------------------------------------------------------ barge-in

    def on_vad(self, speech: bool) -> None:
        """The array said something while pib was talking - the local, instant stage.

        Called from the microphone thread on every VAD edge. Only the rising edge
        acts: a held VAD must not send an event every frame.
        """
        if speech:
            if not self._speaking or self._suspected:
                return
            self._suspected = True
            self._gate.duck()
            self.client.send_suspect_barge_in(self._clock.position_ms)
            return
        # Silence before the cloud answered: the suspicion stands, the cloud's
        # window decides. Nothing to undo locally.

    # ------------------------------------------------------------------ events

    def _handle_event(self, event: dict) -> None:
        kind = event.get("type")
        if kind == "turn_ack":
            self._adopt_turn(event.get("turn_id"))
        elif kind == "barge_in_rejected":
            self._suspected = False
            self._gate.resume()
        elif kind == "barge_in_confirmed":
            self._send_turn_end()
            self._end_turn(reason="barge-in confirmed")
        elif kind == "turn_complete":
            self._end_turn(reason="complete")
        elif kind == "error":
            logger.warning(
                "relay error %s: %s", event.get("code"), event.get("message")
            )
            self._end_turn(reason=f"error {event.get('code')}")
        elif kind == "pong":
            pass
        observer = self._on_event
        if observer is not None:
            try:
                observer(event)
            except Exception:  # noqa: BLE001
                logger.exception("event observer failed")

    def _adopt_turn(self, turn_id: object) -> None:
        """The server's numbering is authoritative - see the module docstring."""
        if not isinstance(turn_id, int):
            return
        if turn_id != self.client.turn_id:
            self._dropped_turns += 1
            self.client.adopt_turn(turn_id)
        self._clock.begin_turn(turn_id)

    def _send_turn_end(self) -> None:
        try:
            self.client.send_turn_end()
        except Exception:  # noqa: BLE001 - the turn is over either way
            logger.debug("turn_end not delivered", exc_info=True)

    # ------------------------------------------------------------------ microphone

    def on_microphone(self, pcm: bytes) -> int:
        """Send microphone PCM upstream, taking channel 0 when the array is multichannel."""
        samples = np.frombuffer(pcm, dtype="<i2")
        if self._input_channels > 1:
            samples = extract_channel(samples, self._input_channels, PROCESSED_CHANNEL)
        if samples.size == 0:
            return 0
        return self.client.send_audio(samples.astype("<i2", copy=False).tobytes())

    def set_input_channels(self, channels: int) -> None:
        if channels < 1:
            raise ValueError("channels must be >= 1")
        self._input_channels = channels

    # ------------------------------------------------------------------ progress

    def _start_progress(self) -> None:
        if self._progress_thread is not None:
            return
        self._progress_stop.clear()
        self._progress_thread = threading.Thread(
            target=self._progress_loop, name="pib-voice-progress", daemon=True
        )
        self._progress_thread.start()

    def _stop_progress(self) -> None:
        self._progress_stop.set()
        thread, self._progress_thread = self._progress_thread, None
        if thread is not None:
            thread.join(self._progress_interval_s * 5)

    def _progress_loop(self) -> None:
        """Tell the relay where playback is, so it can cut exactly and pace its buffer.

        Reported from the clock's frame count, not from wall time: the clock counts
        what reached the device, which is the only number the truncation may use.
        """
        while not self._progress_stop.wait(self._progress_interval_s):
            if not self._speaking:
                continue
            due = self._clock.progress_due()
            if due is not None:
                self.client.send_progress(float(due))
