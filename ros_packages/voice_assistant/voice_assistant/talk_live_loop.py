"""Low-latency Gemini Live audio loop with hardware AEC/VAD barge-in, memory, and Hermes delegation (PR-2252).

Same surface as ``GeminiAudioLoop`` — ``start(chat_id, model, idle_timeout_s)``,
``stop()``, ``is_listening``, ``submit_typed_text`` and the two listeners — so
``assistant.py`` and the Cerebra UI keep working unchanged.

Architecture:
- Zero-resampling 16 kHz mono PCM uplink from ``/audio_stream`` -> Gemini Live.
- Smart Uplink Silence Shaper with a 320 ms (5-chunk) pre-roll ring buffer:
  streams real PCM during user speech (plus pre-roll and 250 ms hangover) and
  pure digital zero PCM (``\\x00``) during silence and non-interrupted playback.
  This prevents XVF3000 AGC room noise (+30 dB) from holding server VAD open
  for 9-16 s, cutting end-of-turn latency to ~600 ms.
- 20-ms sliced playback sink (960 bytes @ 24 kHz) with accurate acoustic
  ``is_playing`` tracking (including a 200 ms hardware tail) and sub-20 ms flush.
- Two-stage barge-in (XVF3000 ``/voice_activity`` + ``/speech_detected`` via
  ``BargeInDetector`` plus acoustic RMS fallback) that flushes local playback
  in <=20 ms, fences in-flight server audio from the old turn, and streams the
  320 ms pre-roll + user speech to trigger server ``START_OF_ACTIVITY_INTERRUPTS``.
- Full ``SOUL.md`` + ``MEMORY.md`` + ``USER.md`` + ``robot_context`` injection,
  cached MCP tool declarations, and ``ask_hermes_agent`` delegation to the warm
  Hermes daemon.
"""

from __future__ import annotations

import asyncio
from collections import deque
import json
import logging
import os
import queue
import subprocess
import threading
import time
from typing import Any, Callable
import uuid

import numpy as np
import pyaudio

from datatypes.srv import CreateOrUpdateChatMessage
from pib_hermes_config import profile_dir_for, strip_mcp_tools_soul_section
from pib_hermes_config.barge_in import BargeInDetector
from pib_hermes_config.channel import direct_system_prompt
from pib_hermes_config.live_interaction import (
    iter_playback_slices,
    playback_slice_bytes,
)
from pib_hermes_config.memory import read_memory
from voice_assistant.audio_loop import (
    CHANNELS,
    FORMAT,
    RECEIVE_SAMPLE_RATE,
    ROS_AUDIO_TOPIC,
    SEND_SAMPLE_RATE,
    RosAudioBridge,
)
from voice_assistant.cloud_session import (
    CascadeResult,
    CloudSession,
    CloudSessionError,
    mint_session,
    report_usage,
)
from voice_assistant.robot_context import get_robot_context
from voice_assistant.talk_provider import talk_gemini_realtime as gemini
from voice_assistant.talk_provider import talk_realtime as rt
from voice_assistant.turn_latency import TURN_TRACE_TOPIC

logger = logging.getLogger("TalkLiveLoop")

#: Voice name from the Gemini prebuilt set.
DEFAULT_VOICE = "Puck"

#: How long a silent session may live before it is closed and settled.
DEFAULT_IDLE_TIMEOUT_S = 180.0

#: 20 ms playback slice at 24 kHz mono s16le (960 bytes).
PLAYBACK_SLICE_BYTES = playback_slice_bytes(RECEIVE_SAMPLE_RATE, 2, CHANNELS)

#: Acoustic tail after the last slice is written to ALSA before the speaker is quiet.
PLAYBACK_TAIL_S = 0.35

#: Pre-roll chunks kept before speech onset (5 * 64 ms = 320 ms).
PREROLL_CHUNKS = 5

#: Hangover chunks kept after speech drops below threshold (4 * 64 ms = 256 ms).
HANGOVER_CHUNKS = 4

#: Mute window at session start so START_SIGNAL_FILE earcon is not sent as user speech.
START_EARCON_MUTE_S = 0.45

#: Minimum RMS for speech when hardware VAD is active vs. acoustic-only.
VAD_ASSISTED_RMS_FLOOR = 220.0
ACOUSTIC_SPEECH_RMS_FLOOR = 380.0
ACOUSTIC_BARGE_IN_RMS = 1600.0
HARDWARE_BARGE_IN_RMS = 1200.0

SEARCH_FILLER_TEXT = "Moment, ich recherchiere."

LIVE_VOICE_DIRECTIVE = (
    "Du führst ein direktes Echtzeit-Sprachgespräch auf Deutsch. "
    "Antworte natürlich, präzise und ohne Markdown-Formatierung in 1 bis 3 "
    "gesprochenen Sätzen, sofern der Nutzer keine ausführliche Erklärung "
    "wünscht. Nutze für Fragen zur Kameraszene sofort pib_describe_scene oder "
    "capture_image, für Erinnerungen pib_recall bzw. pib_remember, für ALLE "
    "aktuellen Informationen aus dem Internet (Wetter, Standorte, Fahrpläne, "
    "Veranstaltungen, Nachrichten, allgemeine Fakten) IMMER pib_web_search "
    "(niemals ask_hermes_agent!) und sage dabei kurz „Moment, ich recherchiere.“, "
    "und zum Beenden/Schlafengehen auf Wunsch des Nutzers ('Geh schlafen', "
    "'Pause', 'Tschüss') go_to_sleep."
)

ASK_HERMES_TOOL = rt.ToolDefinition(
    name="ask_hermes_agent",
    description=(
        "Delegiert ausschließlich interne mehrstufige System-Workflows an den "
        "lokalen Hermes-Agenten. Für Internet-Suche, Wetter, Fahrpläne, "
        "Veranstaltungen, Nachrichten oder Wissensfragen NIEMALS dieses Tool "
        "nutzen, sondern IMMER pib_web_search verwenden."
    ),
    parameters={
        "type": "object",
        "properties": {
            "task": {
                "type": "string",
                "description": "Die genaue System-Aufgabe für den Hermes-Agenten.",
            }
        },
        "required": ["task"],
    },
)

GO_TO_SLEEP_TOOL = rt.ToolDefinition(
    name="go_to_sleep",
    description=(
        "Beendet das aktive Live-Sprachgespräch nach einer kurzen Verabschiedung "
        "und versetzt pib in den Ruhezustand (wartend auf das Wake-Word 'Hey, pib!'), "
        "wenn der Nutzer sagt 'Geh schlafen', 'Pause', 'Stopp' oder sich verabschiedet."
    ),
    parameters={
        "type": "object",
        "properties": {},
    },
)

_CACHED_MCP_TOOLS: tuple[rt.ToolDefinition, ...] | None = None
_CACHED_MCP_TOOLS_LOCK = threading.Lock()

RESPEAKER_CARD_NAME = "alsa_card.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00"
RESPEAKER_PROFILE_3 = "output:analog-stereo+input:analog-surround-51"
RESPEAKER_SINK_NAME = (
    "alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo"
)
RESPEAKER_SOURCE_NAME = (
    "alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-surround-51"
)


def ensure_respeaker_profile_3() -> bool:
    """Ensure PipeWire has the ReSpeaker 4-Mic Array on Profile 3 (6ch AEC) before live audio starts."""
    if not os.getenv("PULSE_SERVER", "").strip():
        return False
    run_kwargs: dict[str, Any] = {
        "capture_output": True,
        "text": True,
        "timeout": 3,
    }
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        run_kwargs["user"] = 1000
        run_kwargs["group"] = 1000
    try:
        sources = subprocess.run(
            ["pactl", "list", "sources", "short"], **run_kwargs
        ).stdout
        if RESPEAKER_SOURCE_NAME not in sources:
            subprocess.run(
                [
                    "pactl",
                    "set-card-profile",
                    RESPEAKER_CARD_NAME,
                    RESPEAKER_PROFILE_3,
                ],
                **run_kwargs,
            )
            subprocess.run(
                ["pactl", "set-default-sink", RESPEAKER_SINK_NAME],
                **run_kwargs,
            )
            subprocess.run(
                ["pactl", "set-default-source", RESPEAKER_SOURCE_NAME],
                **run_kwargs,
            )
            logger.info(
                "Switched ReSpeaker PipeWire profile to %s (6-channel AEC)",
                RESPEAKER_PROFILE_3,
            )
            return True
    except Exception:
        logger.debug("Could not enforce ReSpeaker profile via pactl", exc_info=True)
    return False


def pcm_rms(pcm: bytes) -> float:
    """Root-mean-square amplitude of a s16le mono PCM chunk."""
    if not pcm:
        return 0.0
    samples = np.frombuffer(pcm, dtype="<i2")
    if samples.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(samples.astype(np.float32) ** 2)))


def upsample_16k_to_24k(pcm: bytes) -> bytes:
    """Kept for unit-test compatibility; the live uplink now runs natively at 16 kHz."""
    samples = np.frombuffer(pcm, dtype="<i2")
    if samples.size == 0:
        return pcm
    positions = np.arange(int(round(samples.size * 1.5))) / 1.5
    values = np.interp(positions, np.arange(samples.size), samples)
    return values.astype("<i2").tobytes()


class _PlaybackSink:
    """Writes model audio in 20 ms slices on a dedicated thread for <=20 ms barge-in."""

    def __init__(
        self,
        *,
        on_playing_change: Callable[[bool], None] | None = None,
        on_first_write: Callable[[int], None] | None = None,
    ) -> None:
        self._queue: "queue.Queue[bytes]" = queue.Queue(maxsize=400)
        self._stream = None
        self._pyaudio: pyaudio.PyAudio | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._stream_lock = threading.Lock()
        self._cancel_epoch = 0
        self._queued_at: float | None = None
        self._writing = False
        self._playhead = 0.0
        self._last_write_end = 0.0
        self._playing = False
        self._on_playing_change = on_playing_change
        self._on_first_write = on_first_write

    @property
    def is_playing(self) -> bool:
        if self._writing or not self._queue.empty():
            return True
        if self._last_write_end > 0.0 and time.monotonic() < (
            self._last_write_end + PLAYBACK_TAIL_S
        ):
            return True
        return False

    def _set_playing(self, playing: bool) -> None:
        if playing == self._playing:
            return
        self._playing = playing
        if self._on_playing_change is not None:
            try:
                self._on_playing_change(playing)
            except Exception:
                logger.exception("on_playing_change callback failed")

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        ensure_respeaker_profile_3()
        with self._stream_lock:
            if self._pyaudio is None:
                self._pyaudio = pyaudio.PyAudio()
            open_kwargs = dict(
                format=FORMAT,
                channels=CHANNELS,
                rate=RECEIVE_SAMPLE_RATE,
                output=True,
                frames_per_buffer=PLAYBACK_SLICE_BYTES // 2,
            )
            pulse_idx = self._pulse_output_index(self._pyaudio)
            stream = None
            if pulse_idx is not None:
                try:
                    stream = self._pyaudio.open(
                        output_device_index=pulse_idx, **open_kwargs
                    )
                except Exception:
                    stream = None
            if stream is None:
                stream = self._pyaudio.open(**open_kwargs)
            self._stream = stream
        self._thread = threading.Thread(
            target=self._run, name="TalkPlayback", daemon=True
        )
        self._thread.start()

    @staticmethod
    def _pulse_output_index(pya) -> int | None:
        if not os.getenv("PULSE_SERVER", "").strip():
            return None
        try:
            count = int(pya.get_device_count())
        except Exception:
            return None
        for idx in range(count):
            try:
                info = pya.get_device_info_by_index(idx)
            except Exception:
                continue
            if int(info.get("maxOutputChannels", 0)) < 1:
                continue
            if str(info.get("name", "")).strip().lower() == "pulse":
                return idx
        return None

    def write(self, data: bytes) -> None:
        if not data:
            return
        if self._queue.empty() and not self._writing:
            self._queued_at = time.monotonic()
        self._set_playing(True)
        try:
            self._queue.put_nowait(data)
        except queue.Full:
            pass

    def flush(self) -> int:
        """Abort current 20 ms slice, drain queued audio, and reset ALSA buffer.

        Returns the elapsed milliseconds taken to flush.
        """
        t0 = time.monotonic()
        self._cancel_epoch += 1
        try:
            while True:
                self._queue.get_nowait()
        except queue.Empty:
            pass
        self._queued_at = None
        self._writing = False
        self._playhead = 0.0
        self._last_write_end = 0.0
        with self._stream_lock:
            stream = self._stream
            if stream is not None:
                try:
                    stream.stop_stream()
                    stream.start_stream()
                except Exception:
                    pass
        self._set_playing(False)
        return int((time.monotonic() - t0) * 1000)

    def close(self) -> None:
        self._stop.set()
        self._cancel_epoch += 1
        try:
            while True:
                self._queue.get_nowait()
        except queue.Empty:
            pass
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            self._thread = None
        with self._stream_lock:
            if self._stream is not None:
                try:
                    self._stream.stop_stream()
                except Exception:
                    pass
                try:
                    self._stream.close()
                except Exception:
                    pass
                self._stream = None
        self._set_playing(False)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._queue.get(timeout=0.05)
            except queue.Empty:
                if self._playing and not self.is_playing:
                    self._set_playing(False)
                continue
            if self._stop.is_set():
                return
            epoch = self._cancel_epoch
            queued_at = self._queued_at
            self._queued_at = None
            self._writing = True
            self._set_playing(True)
            first_slice = True
            for slice_pcm in iter_playback_slices(data, PLAYBACK_SLICE_BYTES):
                if self._stop.is_set() or self._cancel_epoch != epoch:
                    break
                with self._stream_lock:
                    stream = self._stream
                    if stream is None or self._cancel_epoch != epoch:
                        break
                    try:
                        stream.write(slice_pcm)
                    except Exception:
                        logger.exception("Playback write failed")
                        break
                if first_slice:
                    first_slice = False
                    if queued_at is not None:
                        delay_ms = int((time.monotonic() - queued_at) * 1000)
                        logger.info(
                            "playback wrote first chunk %d ms after it was queued",
                            delay_ms,
                        )
                        if self._on_first_write is not None:
                            try:
                                self._on_first_write(delay_ms)
                            except Exception:
                                pass
                if self._cancel_epoch == epoch:
                    now_w = time.monotonic()
                    if self._playhead < now_w:
                        self._playhead = now_w
                    self._playhead += len(slice_pcm) / (2.0 * RECEIVE_SAMPLE_RATE)
                    self._last_write_end = self._playhead
                    while (
                        not self._stop.is_set()
                        and self._cancel_epoch == epoch
                        and (self._playhead - time.monotonic()) > 0.040
                    ):
                        self._stop.wait(0.010)
            self._writing = False


class TalkLiveLoop:
    """One live session, driven by the native 16 kHz Gemini Live transport."""

    def __init__(self, node, *, on_session_end=None) -> None:
        self._node = node
        self._on_session_end = on_session_end
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._session: gemini.GeminiRealtimeSession | None = None
        self._session_id: str | None = None
        self._chat_id: str = ""
        self._personality_id: str = ""
        self._listening = False
        self._speaking = False
        self._idle_timeout_s = DEFAULT_IDLE_TIMEOUT_S
        self._last_activity = time.monotonic()
        self._speaking_listener = None
        self._action_announcer = None

        self._barge_in_detector = BargeInDetector(
            confirm_ms=150.0,
            fallback_confirm_ms=200.0,
            earcon_mute_ms=350.0,
            cooldown_ms=350.0,
        )
        self._hw_vad_active = False
        self._hw_vad_at = 0.0
        self._hw_speech_at = 0.0

        self._vad_sub = None
        self._speech_sub = None
        self._audio_sub = None
        try:
            from std_msgs.msg import String

            self._trace_pub = node.create_publisher(String, TURN_TRACE_TOPIC, 10)
        except Exception:
            self._trace_pub = None

        self._chat_client = node.create_client(
            CreateOrUpdateChatMessage, "create_or_update_chat_message"
        )
        #: Recreated fresh per _session_main so it always binds to the active event loop.
        self._audio_in_queue: asyncio.Queue | None = None
        self._player = _PlaybackSink(
            on_playing_change=self._set_speaking,
            on_first_write=self._on_first_speaker_write,
        )
        self._ready = asyncio.Event()
        self._pending_text: dict[bool, str] = {True: "", False: ""}
        self._message_ids: dict[bool, str] = {True: "", False: ""}
        self._pending_live_images: deque[tuple[bytes, str]] = deque()
        self._pending_chat_images: list[str] = []

        # Uplink speech/silence shaping & barge-in state
        self._preroll: deque[bytes] = deque(maxlen=PREROLL_CHUNKS)
        self._in_user_speech = False
        self._hangover_left = 0
        self._noise_rms = 80.0
        self._loud_barge_streak = 0
        self._mute_uplink_until = 0.0
        self._session_paused = False
        self._server_generating = False
        self._barge_in_fenced = False

        # Telemetry state
        self._start_called_at = 0.0
        self._session_ready_ms: int | None = None
        self._speech_onset_at: float | None = None
        self._speech_stopped_at: float | None = None
        self._first_audio_at: float | None = None
        self._first_speaker_write_ms: int | None = None
        self._last_barge_in_stop_ms: int | None = None
        self._turn_audio_bytes = 0
        self._uplink_chunks = 0
        self._uplink_speech_chunks = 0
        self._uplink_silence_chunks = 0
        self._max_uplink_depth = 0
        self._uplink_drops = 0
        self._event_count = 0
        self._last_event_at = 0.0
        self._tools_called: list[str] = []

    # ---------------------------------------------------------------- surface

    @property
    def is_listening(self) -> bool:
        return self._listening

    def set_speaking_listener(self, listener) -> None:
        self._speaking_listener = listener

    def set_action_announcer(self, announcer) -> None:
        self._action_announcer = announcer

    def submit_typed_text(self, text: str) -> bool:
        """Interrupt playback, write user text to chat, and trigger live response."""
        cleaned = (text or "").strip()
        if (
            not cleaned
            or not self._listening
            or self._loop is None
            or self._session is None
        ):
            return False
        self._player.flush()
        if self._server_generating:
            self._barge_in_fenced = True
        self._write_transcript(cleaned, is_user=True)
        self._speech_onset_at = time.monotonic()
        self._speech_stopped_at = time.monotonic()
        command = rt.AddInputText(
            item_id=f"typed-{uuid.uuid4().hex[:12]}", text=cleaned
        )
        try:
            asyncio.run_coroutine_threadsafe(
                self._session.send([command, rt.StartResponse()]), self._loop
            )
        except Exception:
            logger.exception("Could not hand typed text to the live session")
            return False
        self._note_activity()
        return True

    def start(
        self, chat_id: str, model: str | None = None, idle_timeout_s=None
    ) -> None:
        if self._listening:
            return
        now = time.monotonic()
        self._start_called_at = now
        self._session_ready_ms = None
        self._chat_id = chat_id or ""
        self._idle_timeout_s = float(idle_timeout_s or DEFAULT_IDLE_TIMEOUT_S)
        self._stop_event.clear()
        self._sleep_requested = False
        self._listening = True
        self._speaking = False
        self._server_generating = False
        self._barge_in_fenced = False
        self._in_user_speech = False
        self._hangover_left = 0
        self._preroll.clear()
        self._barge_in_detector.reset()
        self._barge_in_detector.mute_for_earcon(
            now * 1000.0, duration_ms=START_EARCON_MUTE_S * 1000.0
        )
        self._mute_uplink_until = now + START_EARCON_MUTE_S
        self._note_activity()
        self._ensure_live_subscriptions()
        self._thread = threading.Thread(
            target=self._thread_main, args=(model,), name="TalkLiveLoop", daemon=True
        )
        self._thread.start()

    def _ensure_live_subscriptions(self) -> None:
        """Subscribe to high-rate audio/VAD topics only while a live session is active."""
        try:
            from std_msgs.msg import Bool, Int16MultiArray

            if self._vad_sub is None:
                self._vad_sub = self._node.create_subscription(
                    Bool, "/voice_activity", self._on_voice_activity, 10
                )
            if self._speech_sub is None:
                self._speech_sub = self._node.create_subscription(
                    Bool, "/speech_detected", self._on_speech_detected, 10
                )
            if self._audio_sub is None:
                self._audio_sub = self._node.create_subscription(
                    Int16MultiArray, ROS_AUDIO_TOPIC, self._on_audio_stream, 10
                )
        except Exception:
            pass

    def _teardown_live_subscriptions(self) -> None:
        destroy = getattr(self._node, "destroy_subscription", None)
        for attr in ("_audio_sub", "_vad_sub", "_speech_sub"):
            sub = getattr(self, attr, None)
            setattr(self, attr, None)
            if sub is not None and callable(destroy):
                try:
                    destroy(sub)
                except Exception:
                    pass

    def stop(self, join_timeout: float = 5.0) -> None:
        self._stop_event.set()
        self._teardown_live_subscriptions()
        loop = self._loop
        session = self._session
        if loop is not None and session is not None:
            try:
                asyncio.run_coroutine_threadsafe(session.close(), loop)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=join_timeout)
            self._thread = None
        self._set_speaking(False)
        self._listening = False
        self._session = None
        self._loop = None

    # -------------------------------------------------------------- internals

    def _note_activity(self) -> None:
        self._last_activity = time.monotonic()

    def _on_first_speaker_write(self, delay_ms: int) -> None:
        self._first_speaker_write_ms = delay_ms

    def _on_voice_activity(self, message) -> None:
        active = bool(getattr(message, "data", False))
        now = time.monotonic()
        self._hw_vad_active = active
        self._hw_vad_at = now
        self._barge_in_detector.observe_voice_activity(
            active,
            now_ms=now * 1000.0,
            can_interrupt=self._player.is_playing,
        )

    def _on_speech_detected(self, message) -> None:
        detected = bool(getattr(message, "data", False))
        now = time.monotonic()
        if detected:
            self._hw_speech_at = now
        self._barge_in_detector.observe_speech_detected(detected, now_ms=now * 1000.0)

    def _on_audio_stream(self, message) -> None:
        if not self._listening:
            return
        loop = self._loop
        audio_queue = self._audio_in_queue
        if loop is None or audio_queue is None:
            return
        try:
            if isinstance(message.data, (bytes, bytearray, memoryview)):
                pcm = bytes(message.data)
            elif hasattr(message.data, "tobytes"):
                pcm = message.data.tobytes()
            else:
                pcm = np.asarray(message.data, dtype=np.int16).tobytes()
        except Exception:
            return
        if not pcm:
            return
        try:
            loop.call_soon_threadsafe(self._enqueue_audio_on_loop, audio_queue, pcm)
        except RuntimeError:
            pass

    def _enqueue_audio_on_loop(self, audio_queue: asyncio.Queue, pcm: bytes) -> None:
        if audio_queue.full():
            try:
                audio_queue.get_nowait()
                self._uplink_drops += 1
            except asyncio.QueueEmpty:
                pass
        try:
            audio_queue.put_nowait({"data": pcm, "mime_type": "audio/pcm"})
        except asyncio.QueueFull:
            pass

    def _trigger_barge_in(self, reason: str, rms: float) -> None:
        """Stop local speaker in <=20 ms and fence old server audio."""
        if self._server_generating:
            self._barge_in_fenced = True
        flush_ms = self._player.flush()
        self._last_barge_in_stop_ms = flush_ms
        self._barge_in_detector.reset()
        self._loud_barge_streak = 0
        logger.info(
            "BARGE_IN_TRIGGERED reason=%s rms=%.0f flush_ms=%d fenced=%s",
            reason,
            rms,
            flush_ms,
            self._barge_in_fenced,
        )

    def _shape_uplink_chunk(self, pcm: bytes, now: float) -> bytes:
        """Return real PCM during user speech (with 320 ms pre-roll) or digital silence."""
        if now < self._mute_uplink_until or self._session_paused:
            self._preroll.clear()
            self._in_user_speech = False
            self._hangover_left = 0
            return b"\x00" * len(pcm)

        rms = pcm_rms(pcm)
        hw_vad_online = self._hw_vad_at > 0.0 and (now - self._hw_vad_at) < 5.0
        hw_fresh = (now - self._hw_vad_at) <= 0.35 and self._hw_vad_active
        hw_speech_fresh = (now - self._hw_speech_at) <= 0.35

        if self._player.is_playing:
            self._in_user_speech = False
            self._hangover_left = 0
            # While pib is speaking, require hardware AEC/VAD confirmation when
            # /voice_activity is online so AGC-boosted speaker echo (where the
            # XVF3000 DSP reports VOICEACTIVITY=0 / hw_fresh=False) never
            # self-interrupts or pollutes the pre-roll buffer.
            hw_confirmed = hw_fresh and hw_speech_fresh and rms >= HARDWARE_BARGE_IN_RMS
            if hw_vad_online:
                if hw_fresh and rms >= HARDWARE_BARGE_IN_RMS:
                    self._loud_barge_streak += 1
                else:
                    self._loud_barge_streak = 0
                    self._preroll.clear()
            else:
                if rms >= ACOUSTIC_BARGE_IN_RMS:
                    self._loud_barge_streak += 1
                else:
                    self._loud_barge_streak = 0

            if hw_confirmed or self._loud_barge_streak >= 2:
                reason = "hw_aec_vad" if hw_fresh else "acoustic_rms"
                self._trigger_barge_in(reason, rms)
                # Fall through to open user speech and flush pre-roll!
            else:
                if hw_fresh:
                    self._preroll.append(pcm)
                self._uplink_silence_chunks += 1
                return b"\x00" * len(pcm)

        speech_threshold = max(ACOUSTIC_SPEECH_RMS_FLOOR, self._noise_rms * 2.5)
        vad_threshold = max(VAD_ASSISTED_RMS_FLOOR, self._noise_rms * 1.5)
        if not self._in_user_speech and hw_vad_online:
            # Speech onset requires hardware VAD when the ReSpeaker VAD topic is active.
            is_speech = hw_fresh and rms >= vad_threshold
        else:
            is_speech = rms >= speech_threshold or (hw_fresh and rms >= vad_threshold)

        if is_speech:
            self._note_activity()
            if not self._in_user_speech:
                self._in_user_speech = True
                self._speech_onset_at = now
                self._speech_stopped_at = None
                self._first_audio_at = None
                self._first_speaker_write_ms = None
                self._turn_audio_bytes = 0
                self._tools_called.clear()
                preroll_bytes = b"".join(self._preroll)
                self._preroll.clear()
                self._hangover_left = HANGOVER_CHUNKS
                self._uplink_speech_chunks += 1
                logger.info(
                    "user speech onset: rms=%.0f noise_floor=%.0f hw_vad=%s preroll_bytes=%d",
                    rms,
                    self._noise_rms,
                    hw_fresh,
                    len(preroll_bytes),
                )
                return preroll_bytes + pcm
            self._hangover_left = HANGOVER_CHUNKS
            self._uplink_speech_chunks += 1
            return pcm

        # Non-speech chunk
        if not self._in_user_speech:
            # Update ambient noise floor slowly during silence
            if 40.0 <= rms <= 450.0:
                self._noise_rms = 0.95 * self._noise_rms + 0.05 * rms
            self._preroll.append(pcm)
            self._uplink_silence_chunks += 1
            return b"\x00" * len(pcm)

        if self._hangover_left > 0:
            self._hangover_left -= 1
            self._uplink_speech_chunks += 1
            if self._hangover_left == 0:
                self._in_user_speech = False
                self._speech_stopped_at = now
                logger.info(
                    "user speech end (hangover complete): speech_ms=%d",
                    int((now - (self._speech_onset_at or now)) * 1000),
                )
            return pcm

        self._in_user_speech = False
        self._preroll.append(pcm)
        self._uplink_silence_chunks += 1
        return b"\x00" * len(pcm)

    def _set_speaking(self, speaking: bool) -> None:
        if speaking == self._speaking:
            return
        self._speaking = speaking
        listener = self._speaking_listener
        if listener is not None:
            try:
                listener(speaking)
            except Exception:
                logger.exception("speaking listener failed")

    def _tool_definitions(self) -> tuple[rt.ToolDefinition, ...]:
        """The MCP catalogue (cached) plus ask_hermes_agent."""
        global _CACHED_MCP_TOOLS
        with _CACHED_MCP_TOOLS_LOCK:
            if _CACHED_MCP_TOOLS is not None:
                return _CACHED_MCP_TOOLS
            try:
                from pib_hermes_config.live_interaction import (
                    gemini_function_declarations,
                )
                from voice_assistant.direct_tool_loop import mcp_tool_declarations

                decls = gemini_function_declarations(mcp_tool_declarations())
                tools = tuple(
                    rt.ToolDefinition(
                        name=entry["name"],
                        description=entry.get("description", ""),
                        parameters=entry.get(
                            "parameters", {"type": "object", "properties": {}}
                        ),
                    )
                    for entry in decls
                ) + (ASK_HERMES_TOOL, GO_TO_SLEEP_TOOL)
                _CACHED_MCP_TOOLS = tools
                return tools
            except Exception:
                logger.exception("Could not read the MCP tool declarations")
                return (ASK_HERMES_TOOL, GO_TO_SLEEP_TOOL)

    def _emit_search_filler(self, tool_name: str) -> None:
        logger.info("Emitting spoken filler cue for %s", tool_name)
        speak = getattr(self._node, "speak_filler_notice", None)
        if not callable(speak):
            return
        try:
            speak(SEARCH_FILLER_TEXT)
        except Exception:
            pass

    def _run_tool(self, name: str, arguments: str) -> str:
        self._tools_called.append(name)
        try:
            parsed = json.loads(arguments) if arguments else {}
        except Exception:
            parsed = {}

        if (
            name in ("pib_web_search", "ask_hermes_agent")
            and self._turn_audio_bytes == 0
            and not self._player.is_playing
        ):
            self._emit_search_filler(name)

        if name == "go_to_sleep":
            self._sleep_requested = True
            return json.dumps(
                {"ok": True, "status": "going_to_sleep_after_farewell"},
                ensure_ascii=False,
            )

        if name == "ask_hermes_agent":
            task = str(parsed.get("task") or "").strip()
            if not task:
                return json.dumps(
                    {"ok": False, "error": "task_empty"}, ensure_ascii=False
                )
            try:
                from public_api_client.hermes_agent_client import _try_daemon_turn

                reply, err = _try_daemon_turn(
                    None,
                    task,
                    self._chat_id,
                    personality_id=self._personality_id or None,
                    timeout=45,
                )
                if reply:
                    return json.dumps({"ok": True, "reply": reply}, ensure_ascii=False)
            except Exception:
                logger.info(
                    "ask_hermes_agent daemon unavailable; falling back to pib_web_search"
                )
            try:
                from pib_hermes_config.cloud_search import search_cloud

                search_res = search_cloud(task)
                if search_res.get("ok"):
                    return json.dumps(search_res, ensure_ascii=False)
            except Exception:
                pass
            return json.dumps(
                {"ok": False, "error": "daemon_no_reply"},
                ensure_ascii=False,
            )

        from pib_hermes_config.visible_state import (
            ANSWER_GESTURE_TOOLS,
            issue_answer_gesture,
        )
        from voice_assistant.direct_tool_loop import execute_mcp_tool

        if name in ANSWER_GESTURE_TOOLS:
            try:
                outcome = issue_answer_gesture(name, parsed, execute_mcp_tool)
            except Exception as exc:
                outcome = {"ok": False, "error": str(exc)}
            return json.dumps(outcome, ensure_ascii=False)

        self._session_paused = True
        try:
            call_args = dict(parsed)
            if name == "capture_image":
                call_args["detail"] = True
            outcome = execute_mcp_tool(name, call_args)
            if name == "capture_image" and isinstance(outcome, dict):
                outcome = self._prepare_live_capture_image(outcome)
            if (
                name == "pib_remember"
                and isinstance(outcome, dict)
                and outcome.get("ok") is True
                and self._personality_id
            ):
                note = str(parsed.get("text") or "").strip()
                if note:
                    self._append_profile_memory(self._personality_id, note)
        except Exception as exc:
            logger.exception("Tool %s failed", name)
            outcome = {
                "ok": False,
                "error": {"code": "tool_failed", "message": str(exc)},
            }
        finally:
            self._mute_uplink_until = time.monotonic() + PLAYBACK_TAIL_S
            self._session_paused = False
        return json.dumps(outcome, ensure_ascii=False)

    def _prepare_live_capture_image(self, outcome: dict[str, Any]) -> dict[str, Any]:
        """Extract base64 JPEG from capture_image, queue it for Gemini Live mediaChunks and chat attachment, and return compact metadata."""
        import base64 as _b64

        payload = (
            outcome.get("result")
            if isinstance(outcome.get("result"), dict)
            else outcome
        )
        raw_b64 = payload.get("imageBase64") if isinstance(payload, dict) else None
        mime_type = (
            str(payload.get("mimeType") or "image/jpeg")
            if isinstance(payload, dict)
            else "image/jpeg"
        )
        if not isinstance(raw_b64, str) or not raw_b64.strip():
            return outcome

        scaled_b64 = raw_b64.strip()
        try:
            from voice_assistant.chat import prepare_tool_image

            prepared = prepare_tool_image(
                scaled_b64, mime_type, max_edge=768, quality=78
            )
            if isinstance(prepared, dict) and prepared.get("imageBase64"):
                scaled_b64 = str(prepared["imageBase64"])
                mime_type = str(prepared.get("mimeType") or "image/jpeg")
        except Exception:
            pass

        try:
            image_bytes = _b64.b64decode(scaled_b64, validate=False)
            if image_bytes:
                self._pending_live_images.append((image_bytes, mime_type))
                self._pending_chat_images.append(scaled_b64)
        except Exception:
            pass

        return {
            "ok": True,
            "result": {
                "imageCaptured": True,
                "imageDeliveredToLiveSession": True,
                "mimeType": mime_type,
                "note": "Das Kamerabild wurde soeben als Live-Bildframe übertragen. Beschreibe nun direkt, was du darauf siehst.",
            },
        }

    @staticmethod
    def _append_profile_memory(personality_id: str, note: str) -> None:
        """Mirror a pib_remember note into the Hermes profile's MEMORY.md."""
        try:
            from pib_hermes_config.memory import (
                ENTRY_DELIMITER,
                parse_entries,
                read_memory,
                write_memory,
            )

            existing = read_memory(personality_id)
            entries = parse_entries(existing)
            if note not in entries:
                entries.append(note)
                write_memory(personality_id, ENTRY_DELIMITER.join(entries))
        except Exception:
            logger.debug(
                "Could not mirror pib_remember into MEMORY.md for %s",
                personality_id,
                exc_info=True,
            )

    def _personality_text(self) -> str:
        """Assemble SOUL.md + robot_context + MEMORY.md + USER.md + live voice rules."""
        personality_name = "pib"
        custom_desc = ""
        personality_id = ""
        try:
            from pib_api_client import voice_assistant_client

            ok, personality = voice_assistant_client.get_personality_from_chat(
                self._chat_id
            )
            if ok and personality is not None:
                personality_id = getattr(personality, "personality_id", "") or ""
                self._personality_id = personality_id
                personality_name = getattr(personality, "name", "") or "pib"
                custom_desc = getattr(personality, "description", "") or ""
        except Exception:
            logger.exception("Could not read the personality; using a default")

        soul_text = ""
        user_text = ""
        memory_text = ""
        if personality_id:
            try:
                prof_dir = profile_dir_for(personality_id)
                soul_path = os.path.join(prof_dir, "SOUL.md")
                if os.path.isfile(soul_path):
                    with open(soul_path, encoding="utf-8") as handle:
                        soul_text = strip_mcp_tools_soul_section(handle.read()).strip()
                user_path = os.path.join(prof_dir, "memories", "USER.md")
                if os.path.isfile(user_path):
                    with open(user_path, encoding="utf-8") as handle:
                        user_text = handle.read().strip()
                memory_text = read_memory(personality_id).strip()
            except Exception:
                logger.debug(
                    "Could not read Hermes profile files for %s", personality_id
                )

        if not soul_text:
            cleaned_desc = strip_mcp_tools_soul_section(custom_desc).strip()
            soul_text = f"Du bist der humanoide Roboter {personality_name}."
            if cleaned_desc and cleaned_desc != soul_text:
                soul_text = f"{soul_text}\n\n{cleaned_desc}"

        base_prompt = direct_system_prompt(
            soul_text,
            images_shown=True,
            robot_context=get_robot_context(),
        )
        sections = [base_prompt, LIVE_VOICE_DIRECTIVE]
        if user_text:
            sections.append(f"## Nutzerprofil (USER.md)\n{user_text[:2000]}")
        if memory_text:
            sections.append(
                f"## Dauerhaftes Gedächtnis (MEMORY.md)\n{memory_text[:4000]}"
            )
        return "\n\n".join(part for part in sections if part.strip())

    def _write_transcript(
        self, text: str, *, is_user: bool, final: bool = True
    ) -> None:
        cleaned = text.strip()
        if not cleaned or not self._chat_id:
            return
        request = CreateOrUpdateChatMessage.Request()
        request.chat_id = self._chat_id
        request.text = cleaned
        request.is_user = is_user
        request.update_database = True
        request.message_id = self._message_ids.get(is_user, "")
        if final:
            self._message_ids[is_user] = ""
        if not self._chat_client.service_is_ready():
            if not self._chat_client.wait_for_service(timeout_sec=0.5):
                logger.warning("Chat service unavailable; transcript dropped")
                return
        future = self._chat_client.call_async(request)
        pending_images = (
            list(self._pending_chat_images)
            if (not is_user and final and self._pending_chat_images)
            else []
        )
        if not is_user and final and self._pending_chat_images:
            self._pending_chat_images.clear()
        if not final or pending_images:
            chat_id = self._chat_id

            def _capture_id(fut):
                try:
                    res = fut.result()
                    if res is not None and getattr(res, "successful", False):
                        mid = getattr(res, "message_id", "")
                        if mid and not final:
                            self._message_ids[is_user] = mid
                        if mid and pending_images:
                            threading.Thread(
                                target=self._attach_live_chat_images,
                                args=(chat_id, mid, cleaned, pending_images),
                                daemon=True,
                            ).start()
                except Exception:
                    pass

            future.add_done_callback(_capture_id)

    @staticmethod
    def _attach_live_chat_images(
        chat_id: str, message_id: str, content: str, images_b64: list[str]
    ) -> None:
        try:
            from pib_api_client import voice_assistant_client

            refs = []
            for img_b64 in images_b64:
                ref = voice_assistant_client.add_chat_message_image(
                    chat_id, message_id, img_b64
                )
                if ref:
                    refs.append(ref)
            if refs:
                voice_assistant_client.update_chat_message(
                    chat_id,
                    content,
                    False,
                    message_id,
                    meta={"source": "live", "images": refs},
                )
        except Exception:
            logger.debug("Could not attach live captured image to chat", exc_info=True)

    def _log_turn_trace(self, *, interrupted: bool = False) -> None:
        onset = self._speech_onset_at
        stop = self._speech_stopped_at or onset
        first_audio = self._first_audio_at
        trace = {
            "chat_id": self._chat_id,
            "session_ready_ms": self._session_ready_ms,
            "speech_ms": (
                int((stop - onset) * 1000)
                if onset is not None and stop is not None and stop >= onset
                else None
            ),
            "stop_to_first_audio_ms": (
                int((first_audio - stop) * 1000)
                if stop is not None and first_audio is not None
                else None
            ),
            "first_audio_to_speaker_ms": self._first_speaker_write_ms,
            "barge_in_stop_ms": self._last_barge_in_stop_ms,
            "interrupted": interrupted,
            "audio_out_s": round(self._turn_audio_bytes / 48000.0, 2),
            "uplink_speech_chunks": self._uplink_speech_chunks,
            "uplink_silence_chunks": self._uplink_silence_chunks,
            "max_uplink_depth": self._max_uplink_depth,
            "tools_called": list(self._tools_called),
            "noise_rms": round(self._noise_rms, 1),
        }
        line = json.dumps(trace, sort_keys=True)
        logger.info("LIVE_TURN_TRACE %s", line)
        if self._trace_pub is not None:
            try:
                from std_msgs.msg import String

                msg = String()
                msg.data = line
                self._trace_pub.publish(msg)
            except Exception:
                pass
        self._last_barge_in_stop_ms = None

    def _thread_main(self, model: str | None) -> None:
        reason = "closed"
        try:
            reason = asyncio.run(self._session_main(model)) or "closed"
        except Exception:
            reason = "connection_failed"
            logger.exception("Talk live loop ended with an error")
        finally:
            self._listening = False
            self._teardown_live_subscriptions()
            self._set_speaking(False)
            callback = self._on_session_end
            if callback is not None:
                try:
                    callback(reason)
                except Exception:
                    logger.exception("on_session_end callback failed")

    async def _session_main(self, model: str | None) -> str:
        self._loop = asyncio.get_running_loop()
        # A fresh Queue per event loop prevents Python's _LoopBoundMixin from
        # raising RuntimeError("is bound to a different event loop") on the
        # second session start.
        self._audio_in_queue = asyncio.Queue(maxsize=64)
        self._uplink_chunks = 0
        self._uplink_speech_chunks = 0
        self._uplink_silence_chunks = 0
        self._max_uplink_depth = 0
        self._uplink_drops = 0
        self._last_event_at = 0.0
        wire_model = model or "gemini-3.8-live"

        player_future = asyncio.to_thread(self._player.start)
        tools_future = asyncio.to_thread(self._tool_definitions)
        instructions_future = asyncio.to_thread(self._personality_text)
        mint_future = asyncio.to_thread(mint_session, wire_model, None)

        resumption_handle: str | None = None
        previous_sid: str | None = None
        current_usage: dict[str, int] = {}

        try:
            try:
                minted = await mint_future
            except CloudSessionError as exc:
                logger.warning("pib.Cloud live session mint failed: %s", exc.code)
                return exc.code
            if isinstance(minted, CascadeResult):
                logger.warning("pib.Cloud declined the live session: %s", minted.reason)
                return minted.reason
            if not isinstance(minted, CloudSession):
                logger.error("pib.Cloud returned no usable session")
                return "mint_failed"

            tools, instructions, _ = await asyncio.gather(
                tools_future, instructions_future, player_future
            )
            logger.info("live setup: %d tools, model=%s", len(tools), wire_model)
            setup = rt.SessionSetup(
                model=wire_model,
                voice=DEFAULT_VOICE,
                instructions=instructions,
                tools=tools,
            )

            while not self._stop_event.is_set():
                if minted is None:
                    try:
                        minted = await asyncio.to_thread(
                            mint_session, wire_model, previous_sid
                        )
                    except CloudSessionError as exc:
                        logger.warning(
                            "pib.Cloud live session renewal failed: %s", exc.code
                        )
                        return exc.code
                    if isinstance(minted, CascadeResult):
                        logger.warning(
                            "pib.Cloud ended the live session on renewal: %s",
                            minted.reason,
                        )
                        return minted.reason
                    if not isinstance(minted, CloudSession):
                        logger.error("pib.Cloud returned no usable renewal session")
                        return "mint_failed"

                self._session_id = minted.session_id
                ttl_seconds = float(minted.ttl_seconds or 300)
                session = gemini.GeminiRealtimeSession(
                    auth_token=minted.ephemeral_token,
                    auth_source="pib-cloud",
                    input_sample_rate=SEND_SAMPLE_RATE,
                    resumption_handle=resumption_handle,
                )
                self._session = session
                try:
                    await session.connect(setup)
                except Exception:
                    logger.exception("Could not open the Talk session")
                    return "connection_failed"

                renew_event = asyncio.Event()
                tasks = [
                    asyncio.create_task(self._uplink(session, renew_event)),
                    asyncio.create_task(self._downlink(session, renew_event)),
                    asyncio.create_task(self._idle_watchdog()),
                    asyncio.create_task(self._ttl_watchdog(ttl_seconds, renew_event)),
                ]
                try:
                    await self._wait_for_stop_or_renew(renew_event)
                finally:
                    for task in tasks:
                        task.cancel()
                    await asyncio.gather(*tasks, return_exceptions=True)
                    resumption_handle = session.resumption_handle or resumption_handle
                    current_usage = dict(session.usage_metadata)
                    try:
                        await session.close()
                    except Exception:
                        pass

                if self._stop_event.is_set():
                    break

                # Settle the expiring segment before minting the replacement token
                if self._session_id:
                    previous_sid = self._session_id
                    self._session_id = None
                    try:
                        await asyncio.to_thread(
                            report_usage, previous_sid, current_usage or None
                        )
                        current_usage = {}
                    except Exception:
                        logger.exception(
                            "Could not settle pib.Cloud session before renewal"
                        )
                minted = None
                logger.info(
                    "Renewing pib.Cloud live session (previous=%s, resumable=%s)",
                    previous_sid,
                    bool(resumption_handle),
                )
        finally:
            self._audio_in_queue = None
            await asyncio.to_thread(self._player.close)
            if self._session_id:
                sid = self._session_id
                self._session_id = None
                try:
                    await asyncio.to_thread(report_usage, sid, current_usage or None)
                except Exception:
                    logger.exception("Could not settle the pib.Cloud session")
        return "closed"

    async def _wait_for_stop_or_renew(self, renew_event: asyncio.Event) -> None:
        while not self._stop_event.is_set() and not renew_event.is_set():
            await asyncio.sleep(0.1)

    async def _ttl_watchdog(
        self, ttl_seconds: float, renew_event: asyncio.Event
    ) -> None:
        if ttl_seconds <= 0:
            return
        renew_at = time.monotonic() + (ttl_seconds * 0.8)
        hard_deadline = time.monotonic() + max(1.0, ttl_seconds - 5.0)
        while not self._stop_event.is_set() and not renew_event.is_set():
            await asyncio.sleep(0.25)
            now = time.monotonic()
            if now >= hard_deadline or (
                now >= renew_at
                and not self._in_user_speech
                and not self._player.is_playing
                and not self._server_generating
            ):
                renew_event.set()
                return

    async def _uplink(self, session, renew_event: asyncio.Event | None = None) -> None:
        deadline = time.monotonic() + 10.0
        while (
            session.state is not rt.SessionState.CONNECTED
            and not self._stop_event.is_set()
        ):
            if time.monotonic() > deadline:
                logger.error("Live session did not become ready; uplink not started")
                self._stop_event.set()
                return
            await asyncio.sleep(0.02)
        logger.info("uplink started (16 kHz native, zero-resampling)")
        audio_queue = self._audio_in_queue
        if audio_queue is None:
            return
        sent = 0
        while not self._stop_event.is_set() and (
            renew_event is None or not renew_event.is_set()
        ):
            message = await audio_queue.get()
            data = message.get("data", b"")
            if not data:
                continue
            now = time.monotonic()
            shaped = self._shape_uplink_chunk(data, now)
            if not shaped:
                continue
            sent += 1
            self._uplink_chunks = sent
            depth = audio_queue.qsize()
            if depth > self._max_uplink_depth:
                self._max_uplink_depth = depth
            try:
                await session.send([rt.AppendInputAudio(data=shaped)])
            except Exception:
                if not self._stop_event.is_set():
                    logger.warning(
                        "Uplink send interrupted; requesting session renewal"
                    )
                    if renew_event is not None:
                        renew_event.set()
                    else:
                        self._stop_event.set()
                return
            if sent % 200 == 0:
                logger.info(
                    "uplink sent %d chunks (speech=%d silence=%d max_depth=%d noise_rms=%.0f)",
                    sent,
                    self._uplink_speech_chunks,
                    self._uplink_silence_chunks,
                    self._max_uplink_depth,
                    self._noise_rms,
                )

    async def _downlink(
        self, session, renew_event: asyncio.Event | None = None
    ) -> None:
        try:
            async for event in session:
                now = time.monotonic()
                self._event_count += 1
                gap = (
                    int((now - self._last_event_at) * 1000)
                    if self._last_event_at
                    else 0
                )
                self._last_event_at = now
                if not isinstance(event, rt.OutputAudio):
                    logger.info(
                        "live event: %s (+%dms) uplink=%d speech=%d silence=%d",
                        type(event).__name__,
                        gap,
                        self._uplink_chunks,
                        self._uplink_speech_chunks,
                        self._uplink_silence_chunks,
                    )

                if isinstance(event, rt.SessionReady):
                    self._session_ready_ms = int((now - self._start_called_at) * 1000)
                    logger.info("live session ready in %d ms", self._session_ready_ms)

                elif isinstance(event, rt.ResponseStarted):
                    self._note_activity()
                    self._server_generating = True
                    # A new response starting clears any prior turn's fence.
                    self._barge_in_fenced = False

                elif isinstance(event, rt.OutputAudio):
                    self._note_activity()
                    if self._barge_in_fenced:
                        # Drop in-flight audio of the turn the user just interrupted.
                        continue
                    if self._first_audio_at is None:
                        self._first_audio_at = now
                        ref = self._speech_stopped_at or self._speech_onset_at
                        if ref is not None:
                            logger.info(
                                "first audio after end of speech: %d ms",
                                int((now - ref) * 1000),
                            )
                    self._turn_audio_bytes += len(event.data)
                    self._player.write(event.data)

                elif isinstance(event, rt.Transcript):
                    self._note_activity()
                    is_user = event.role is rt.TranscriptRole.USER
                    piece = event.text.strip()
                    if piece:
                        self._pending_text[is_user] = (
                            self._pending_text[is_user] + " " + piece
                        ).strip()
                    if event.final:
                        text = self._pending_text[is_user]
                        self._pending_text[is_user] = ""
                        if text:
                            self._write_transcript(text, is_user=is_user, final=True)
                            logger.info(
                                "transcript [%s]: %s",
                                "USER" if is_user else "PIB",
                                text[:160],
                            )

                elif isinstance(event, rt.FunctionCall):
                    self._note_activity()
                    logger.info("live tool call: %s(%s)", event.name, event.arguments)
                    output = await asyncio.to_thread(
                        self._run_tool, event.name, event.arguments
                    )
                    commands: list[rt.RealtimeCommand] = []
                    while self._pending_live_images:
                        img_bytes, img_mime = self._pending_live_images.popleft()
                        commands.append(
                            rt.AppendInputImage(data=img_bytes, mime_type=img_mime)
                        )
                    commands.append(
                        rt.SubmitToolResult(call_id=event.call_id, output=output)
                    )
                    await session.send(commands)

                elif isinstance(event, (rt.SpeechStarted, rt.OutputInterrupted)):
                    # Server confirmed interruption (serverContent.interrupted = true)
                    self._server_generating = False
                    self._barge_in_fenced = False
                    flush_ms = await asyncio.to_thread(self._player.flush)
                    if self._last_barge_in_stop_ms is None:
                        self._last_barge_in_stop_ms = flush_ms
                    self._pending_text[False] = ""
                    self._log_turn_trace(interrupted=True)

                elif isinstance(event, rt.SpeechStopped):
                    self._speech_stopped_at = now

                elif isinstance(event, (rt.ResponseFinished, rt.OutputTurnCompleted)):
                    self._server_generating = False
                    self._barge_in_fenced = False
                    if self._turn_audio_bytes > 0 or self._tools_called:
                        self._log_turn_trace(interrupted=False)

                elif isinstance(event, rt.SessionTerminated):
                    logger.info(
                        "live session terminated: state=%s detail=%s",
                        event.state,
                        event.detail,
                    )
                    if not self._stop_event.is_set() and renew_event is not None:
                        renew_event.set()
                    else:
                        self._stop_event.set()
                    return
        finally:
            self._server_generating = False
            self._barge_in_fenced = False

    async def _idle_watchdog(self) -> None:
        while not self._stop_event.is_set():
            await asyncio.sleep(0.5)
            if self._player.is_playing or self._server_generating:
                self._note_activity()
                continue
            if getattr(self, "_sleep_requested", False):
                logger.info(
                    "go_to_sleep tool completed farewell; stopping live session."
                )
                self._stop_event.set()
                return
            if time.monotonic() - self._last_activity > self._idle_timeout_s:
                logger.info(
                    "Live session idle for %.0fs; stopping so it does not keep billing.",
                    self._idle_timeout_s,
                )
                self._stop_event.set()
                return
