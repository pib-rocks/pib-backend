"""The live voice path on the relay: listen, transcribe here, let the cloud answer.

Today's live path asks Gemini for speech-to-speech. The relay works differently and
better suits the hardware: it takes the *user's words* and returns the answer as
audio, which means

* the turn starts from a transcription - the robot's own whisper, already on the
  Pi and already pinned to German, so no audio of the room leaves the house for
  the purpose of being transcribed, and
* everything after the transcription - prompt, model, speech engine, pricing -
  is one place in the cloud instead of two.

The cycle is therefore: an utterance ends, the recorder transcribes it, the text
opens a turn on the socket, the audio comes back and is played by
:class:`DuplexVoiceSession`. When the turn is over the robot listens again - the
loop has no idle state, which is what makes a conversation feel like one.

ROS is not imported here. The node hands in its own subscriptions, action calls and
audio-device writes, so the cycle is testable and so a pib without an array fails
at the wiring, not inside the loop.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from voice_assistant.duplex_session import DuplexVoiceSession

logger = logging.getLogger(__name__)

# How long a silence ends the spoken turn. The relay path is not the button path:
# there is no second press, so the answer has to start as soon as the sentence is
# over. 450 ms is what the measured local gate used (PC-30); whisper's endpointing
# needs a little more than a provider VAD, so the default sits at 700 ms and the
# personality's own pause threshold is honoured up to 800 ms.
DEFAULT_SILENCE_AFTER_S = 0.7
MIN_SILENCE_AFTER_S = 0.3
MAX_SILENCE_AFTER_S = 0.8
DEFAULT_SILENCE_BEFORE_S = 8.0


def silence_after_s(
    pause_threshold: Optional[float] = None, override: Optional[float] = None
) -> float:
    """Seconds of silence that end the turn, clamped to what still sounds natural."""
    if override is not None:
        return max(MIN_SILENCE_AFTER_S, float(override))
    value = (
        DEFAULT_SILENCE_AFTER_S if pause_threshold is None else float(pause_threshold)
    )
    return max(MIN_SILENCE_AFTER_S, min(value, MAX_SILENCE_AFTER_S))


STATE_IDLE = "idle"
STATE_LISTENING = "listening"
STATE_SPEAKING = "speaking"


class DuplexVoiceLoop:
    """Owns one :class:`DuplexVoiceSession` and the turn cycle around it.

    ``request_transcription(on_text)`` asks the recorder node for one utterance and
    calls ``on_text`` with what was said (empty string when nothing was). That
    indirection is deliberate: the loop must not know whether transcription happens
    in this process, in the recorder's action, or in a mock.
    """

    def __init__(
        self,
        *,
        session: DuplexVoiceSession,
        request_transcription: Callable[[Callable[[str], None]], None],
        on_speaking: Optional[Callable[[bool], None]] = None,
        on_close: Optional[Callable[[], None]] = None,
        idle_timeout_s: Optional[float] = None,
    ) -> None:
        self._session = session
        self._request_transcription = request_transcription
        self._on_speaking = on_speaking
        self._on_close = on_close
        self._idle_timeout_s = idle_timeout_s
        self._lock = threading.Lock()
        self._running = False
        self._transcribing = False
        self._state = STATE_IDLE
        self._turns = 0
        self._empty_turns = 0
        self._last_activity_s = time.monotonic()
        self.chat_id: Optional[str] = None

    # ------------------------------------------------------------------ lifecycle

    @property
    def is_listening(self) -> bool:
        return self._running

    @property
    def state(self) -> str:
        return self._state

    def start(
        self,
        chat_id: Optional[str] = None,
        model: Optional[str] = None,
        idle_timeout_s: Optional[float] = None,
    ) -> None:
        if self._running:
            return
        self.chat_id = chat_id
        if idle_timeout_s is not None:
            self._idle_timeout_s = idle_timeout_s
        self._running = True
        self._session.start()
        logger.info("duplex voice loop started (chat=%s)", chat_id)
        self._listen()

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        self._set_state(STATE_IDLE)
        self._session.close()
        closer = self._on_close
        if closer is not None:
            try:
                closer()
            except (
                Exception
            ):  # noqa: BLE001 - a device that fails to close is not fatal
                logger.exception("closing the voice devices failed")

    def set_speaking_listener(self, listener: Optional[Callable[[bool], None]]) -> None:
        self._on_speaking = listener

    def set_action_announcer(self, announcer) -> None:  # parity with the live loop
        self._announce = announcer

    # ------------------------------------------------------------------ turn cycle

    def submit_typed_text(self, text: str) -> bool:
        """A typed message takes the same path as a spoken one: the cloud answers."""
        text = (text or "").strip()
        if not text or not self._running:
            return False
        self._speak(text)
        return True

    def _listen(self) -> None:
        with self._lock:
            if not self._running or self._transcribing:
                return
            self._transcribing = True
        self._set_state(STATE_LISTENING)
        try:
            self._request_transcription(self._on_transcribed)
        except Exception:  # noqa: BLE001 - a failing recorder must not end the loop
            logger.exception("transcription request failed")
            with self._lock:
                self._transcribing = False

    def _on_transcribed(self, text: str) -> None:
        with self._lock:
            self._transcribing = False
        if not self._running:
            return
        text = (text or "").strip()
        if not text:
            self._empty_turns += 1
            # Silence is not the end of a conversation: the recorder waits for a
            # greeting, a question, anything. Only the personality's idle timeout
            # closes the session, and it is measured from the last real utterance.
            idle_for = time.monotonic() - self._last_activity_s
            if self._idle_timeout_s is not None and idle_for >= self._idle_timeout_s:
                logger.info("idle for %.0f s, closing the voice session", idle_for)
                self.stop()
                return
            self._listen()
            return
        self._empty_turns = 0
        self._last_activity_s = time.monotonic()
        self._speak(text)

    def _speak(self, text: str) -> None:
        self._last_activity_s = time.monotonic()
        self._turns += 1
        self._set_state(STATE_SPEAKING)
        self._session.say(text)

    # ------------------------------------------------------------------ events from the session

    def on_turn_finished(self) -> None:
        """The session finished a turn - listen again unless the user stopped us."""
        if not self._running:
            return
        self._set_state(STATE_IDLE)
        self._listen()

    def on_barge_in(self) -> None:
        """The user interrupted: the turn is gone, the next utterance is the question."""
        if not self._running:
            return
        self._set_state(STATE_LISTENING)
        self._listen()

    # ------------------------------------------------------------------ state

    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        was_speaking = self._state == STATE_SPEAKING
        self._state = state
        is_speaking = state == STATE_SPEAKING
        # Only speaking is announced. Listening and idle are internal states; a
        # listener that had to filter them would be listening to the wrong signal.
        if was_speaking == is_speaking:
            return
        listener = self._on_speaking
        if listener is not None:
            try:
                listener(is_speaking)
            except Exception:  # noqa: BLE001
                logger.exception("speaking listener failed")

    @property
    def stats(self) -> dict:
        return {"turns": self._turns, "empty_utterances": self._empty_turns}
