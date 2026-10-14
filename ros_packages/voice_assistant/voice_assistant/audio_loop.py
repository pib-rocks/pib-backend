# audio_loop.py
#
# Gemini live audio loop that:
# - Subscribes to a ROS audio topic and forwards gated PCM chunks to a Gemini live session.
# - Receives audio (TTS) and transcript events back from Gemini.
# - Streams user/assistant transcripts into ChatNode via the CreateOrUpdateChatMessage srv
#   so the chat UI updates live while Gemini speaks.
# - Handles Stop/Start robustly by closing the live session and joining the worker thread.

import asyncio
import os
import logging
import threading
from typing import Any, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int16MultiArray
from pib_api_client import voice_assistant_client
from pib_hermes_config.live_interaction import (
    activity_end_message,
    activity_start_message,
    admit_live_uplink,
    gemini_function_declarations,
    interruption_detected,
    iter_function_calls,
    live_turn_detection_config,
    on_interruption,
    perform_robot_action,
    playback_exclusion_deadline,
    playback_slice_bytes,
    iter_playback_slices,
)
from voice_assistant.live_gate import ActivityGate
from voice_assistant.live_registers import PRODUCT_ID, VENDOR_ID, open_register_port
from pib_hermes_config.live_session import (
    context_window_compression,
    gemini_live_connect_model,
    idle_expired,
)

import pyaudio
from google import genai

import wave
from pathlib import Path
from datetime import datetime
from asyncio import QueueEmpty

# service API to ChatNode (to avoid duplicating DB/publish logic here)
from datatypes.srv import CreateOrUpdateChatMessage

import queue
import time

# ——— Logging ———
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("GeminiAudioLoop")

# ——— Model + audio IO constants ———
pya = pyaudio.PyAudio()
FORMAT = pyaudio.paInt16
CHANNELS = 1  # send mono to Gemini
SEND_SAMPLE_RATE = 16000  # recorder publishes 16 kHz mono
RECEIVE_SAMPLE_RATE = 24000  # model replies at 24 kHz
# Measured on the robot 2026-10-14: /audio_stream messages are 1024 samples
# at 16 kHz (64 ms, about 12 messages per second). The gate's chunk counts
# are derived from this length and the target durations, not from 10 and 14.
CHUNK_SIZE = 1024
PRE_ROLL_MS = 320
TURN_END_MS = 450
# Raised on GAMMAVAD_SR while a motor tool runs. The value read at the start
# of the motion is what gets written back. 8.5 is the default, never 3.5.
DEFAULT_MOTION_VAD_DB = 8.5
_MOTION_VAD_ENV = "GAMMAVAD_SR_MOTION_DB"

# The live model id is a field of the provider row. This loop opens whatever
# pinned Gemini model the caller passes in.
CONFIG = {
    "response_modalities": ["AUDIO"],  # request synthesized speech back
    "input_audio_transcription": {},  # get user (input) transcript stream
    "output_audio_transcription": {},  # get assistant (output) transcript stream
}
ROS_AUDIO_TOPIC = os.getenv("ROS_AUDIO_TOPIC", "audio_stream")


# ——— Live session lifetime management ———
# Live API limits:
# - Without compression, audio-only sessions are limited to ~15 minutes.
# - A single WebSocket connection is limited to ~10 minutes (GoAway warning before termination).
# Context window compression stays on: it is what removes the duration cap.
# Session resumption + reconnect survives the connection cap.
ENABLE_SESSION_RESUMPTION = os.getenv("ENABLE_SESSION_RESUMPTION", "1") == "1"

# Reasonable defaults for native-audio 128k context models (tune if needed)
CWC_TRIGGER_TOKENS = int(os.getenv("CWC_TRIGGER_TOKENS", "100000"))
CWC_TARGET_TOKENS = int(os.getenv("CWC_TARGET_TOKENS", "80000"))

# Small delay before reconnecting (prevents tight loops on persistent failures)
LIVE_RECONNECT_BACKOFF_S = float(os.getenv("LIVE_RECONNECT_BACKOFF_S", "0.5"))


class ReconnectRequested(RuntimeError):
    """Raised by tasks to request a clean Live session reconnect."""

    pass


class LiveIdleTimeout(RuntimeError):
    """The personality's idle timeout elapsed with nobody speaking."""

    pass


def gate_counts_for_message(
    samples_per_message: int, sample_rate: int = SEND_SAMPLE_RATE
) -> tuple[int, int]:
    """Pre-roll and silence-end counts for one microphone message length.

    320 ms of audio is kept before the trigger, and about 450 ms of silence
    ends the turn. ``samples_per_message`` is the length actually observed.
    """
    if samples_per_message <= 0 or sample_rate <= 0:
        samples_per_message = CHUNK_SIZE
        sample_rate = SEND_SAMPLE_RATE
    chunk_ms = (samples_per_message / float(sample_rate)) * 1000.0
    pre_roll = max(1, int(round(PRE_ROLL_MS / chunk_ms)))
    silence_end = max(1, int(round(TURN_END_MS / chunk_ms)))
    return pre_roll, silence_end


def motion_vad_threshold_db() -> float:
    """Configured GAMMAVAD_SR while a motor moves. Default 8.5 dB."""
    raw = os.getenv(_MOTION_VAD_ENV, "").strip()
    if not raw:
        return DEFAULT_MOTION_VAD_DB
    try:
        return float(raw)
    except ValueError:
        return DEFAULT_MOTION_VAD_DB


def _session_root(endpoint_base: str) -> str:
    """Origin for ``POST /v1/session``.

    ``pib_cloud_base_url`` is the OpenAI-compatible root and already ends in
    ``/v1``. The session client appends ``/v1/session`` itself.
    """
    base = endpoint_base.strip().rstrip("/")
    if base.endswith("/v1"):
        return base[: -len("/v1")]
    return base


def _pcm_from_message(msg: object) -> bytes:
    if isinstance(msg, dict):
        data = msg.get("data") or b""
    else:
        data = msg
    if isinstance(data, bytes):
        return data
    try:
        return bytes(data)
    except Exception:
        return b""


def _mime_from_message(msg: object) -> str:
    if isinstance(msg, dict) and isinstance(msg.get("mime_type"), str):
        return msg["mime_type"]
    return "audio/pcm"


def _json_ready(value: object) -> object:
    """A JSON value. Usage metadata must not need the token to be rendered."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    enum_value = getattr(value, "value", None)
    if enum_value is not None and not callable(enum_value):
        return _json_ready(enum_value)
    return str(value)


def _usage_dict(usage: object) -> dict:
    if usage is None:
        return {}
    if isinstance(usage, dict):
        data = usage
    else:
        dump = getattr(usage, "model_dump", None)
        data = dump(exclude_none=True) if callable(dump) else None
    if not isinstance(data, dict):
        return {}
    ready = _json_ready(data)
    return ready if isinstance(ready, dict) else {}


def _int_field(data: dict, *names: str) -> Optional[int]:
    for name in names:
        value = data.get(name)
        if isinstance(value, bool) or not isinstance(value, int):
            continue
        return value
    return None


def _modality_tokens(data: dict, detail_keys: tuple[str, ...], modality: str) -> int:
    wanted = modality.upper()
    for key in detail_keys:
        details = data.get(key)
        if not isinstance(details, list):
            continue
        total = 0
        found = False
        for item in details:
            if not isinstance(item, dict):
                continue
            name = item.get("modality")
            count = item.get("token_count", item.get("tokenCount"))
            if not isinstance(name, str) or str(name).upper() != wanted:
                continue
            if isinstance(count, bool) or not isinstance(count, int):
                continue
            total += count
            found = True
        if found:
            return total
    return 0


class _RedactingFilter(logging.Filter):
    """Drops the pib token and the ephemeral session token from every line."""

    def __init__(self) -> None:
        super().__init__()
        self._secrets: list[str] = []

    def add(self, secret: object) -> None:
        if not isinstance(secret, str) or len(secret) < 8:
            return
        if secret not in self._secrets:
            self._secrets.append(secret)

    def filter(self, record: logging.LogRecord) -> bool:
        try:
            secrets = tuple(self._secrets)
            if not secrets:
                return True
            if record.args:
                record.msg = record.getMessage()
                record.args = ()
            message = record.msg
            if isinstance(message, str):
                for secret in secrets:
                    if secret in message:
                        message = message.replace(secret, "<redacted>")
                record.msg = message
            if record.exc_info:
                text = record.exc_text
                if text is None:
                    text = logging.Formatter().formatException(record.exc_info)
                for secret in secrets:
                    if secret in text:
                        text = text.replace(secret, "<redacted>")
                record.exc_text = text
        except Exception:
            return True
        return True


class _StatusRing:
    """LED ring states. A missing ring, or a missing pyusb, does nothing.

    The existing ``PixelRing`` helper is used when that package is importable.
    Otherwise the same vendor writes (off / listen / think / speak) go out
    directly, which is what the voice image can do once python3-usb is installed.
    """

    def __init__(self) -> None:
        self._ring = None
        self._opened = False

    def show(self, state: str) -> None:
        ring = self._ensure()
        if ring is None:
            return
        method = {
            "idle": "off",
            "listening": "listen",
            "thinking": "think",
            "speaking": "speak",
        }.get(state)
        if method is None:
            return
        try:
            getattr(ring, method)()
        except Exception:
            return

    def _ensure(self):
        if self._opened:
            return self._ring
        self._opened = True
        try:
            import usb.core

            device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        except Exception:
            return None
        if not device:
            return None
        try:
            from ros_audio_io.pixel_ring import PixelRing

            self._ring = PixelRing(device)
            return self._ring
        except Exception:
            pass
        try:
            self._ring = _DirectStatusRing(device)
        except Exception:
            self._ring = None
        return self._ring


class _DirectStatusRing:
    """The PixelRing write protocol, used when that module is not installed."""

    def __init__(self, device) -> None:
        self._device = device

    def _write(self, command: int, data) -> None:
        import usb.util

        self._device.ctrl_transfer(
            usb.util.CTRL_OUT
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            command,
            0x1C,
            [0] if data is None else data,
            8000,
        )

    def off(self) -> None:
        self._write(1, [0, 0, 0, 0])

    def listen(self) -> None:
        self._write(2, None)

    def speak(self) -> None:
        self._write(3, None)

    def think(self) -> None:
        self._write(4, None)


def _capture_bound_attention() -> None:
    """Remember the handle ``bind_attention`` already returns.

    The voice node binds that handle after this module is imported. The gate's
    start edge calls ``command_toward_speaker`` on it. No second head
    controller is built.
    """
    try:
        from voice_assistant import attention as attention_mod
    except Exception:
        return
    original = getattr(attention_mod, "bind_attention", None)
    if original is None or getattr(original, "_captures_handle", False):
        return

    def bind_attention(*args, **kwargs):
        handle = original(*args, **kwargs)
        attention_mod.bound_attention_handle = handle
        return handle

    bind_attention._captures_handle = True
    attention_mod.bind_attention = bind_attention


# ——————————————————————————————————————————
#         ROS subscriber bridge (thread)
# ——————————————————————————————————————————
class RosAudioBridge:
    """
    Subscribes to a ROS topic carrying PCM16 mono @16k (Int16MultiArray or AudioData)
    and forwards chunks into an asyncio.Queue that send_realtime() consumes.
    Runs in its own thread with a SingleThreadedExecutor.
    """

    def __init__(
        self, topic: str, loop: asyncio.AbstractEventLoop, out_queue: asyncio.Queue
    ):
        self._topic = topic
        self._loop = loop
        self._out_queue = out_queue
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._started_evt = threading.Event()

    def start(self):
        """Start ROS subscriber thread and wait until it’s actually listening."""
        self._thread = threading.Thread(
            target=self._run, name="RosAudioBridge", daemon=True
        )
        self._thread.start()
        self._started_evt.wait(timeout=3.0)

    def stop(self):
        """Signal shutdown and join quickly (best-effort)."""
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None

    def _run(self):
        try:
            # Optional support for audio_common_msgs/AudioData (bytes)
            AudioData = None
            try:
                from audio_common_msgs.msg import AudioData as _AudioData  # type: ignore

                AudioData = _AudioData
            except Exception:
                pass

            # Ensure a ROS context exists in this thread
            if not rclpy.ok():
                rclpy.init()

            class _Node(Node):
                """Tiny ROS node that receives PCM and forwards to asyncio queue."""

                def __init__(self, topic, loop, queue, stop_evt):
                    super().__init__("ros_audio_bridge")
                    self._loop = loop
                    self._queue = queue
                    self._stop_evt = stop_evt
                    self._subs = []
                    self._subs.append(
                        self.create_subscription(
                            Int16MultiArray, topic, self._cb_int16, 10
                        )
                    )
                    if AudioData is not None:
                        self._subs.append(
                            self.create_subscription(
                                AudioData, topic, self._cb_bytes, 10
                            )
                        )
                    self.get_logger().info(
                        f"Subscribed to '{topic}' for PCM16 mono @16k"
                    )

                def _enqueue(self, payload: bytes):
                    """Push a PCM payload into the asyncio queue (thread-safe)."""
                    if self._stop_evt.is_set():
                        return

                    async def _put():
                        await self._queue.put(
                            {"data": payload, "mime_type": "audio/pcm"}
                        )

                    try:
                        fut = asyncio.run_coroutine_threadsafe(_put(), self._loop)
                        fut.result(timeout=0.25)
                    except Exception:
                        # If the loop is busy, drop late chunks rather than blocking.
                        pass

                def _cb_int16(self, msg: Int16MultiArray):
                    try:
                        arr = np.asarray(msg.data, dtype=np.int16)
                        self._enqueue(arr.tobytes())
                    except Exception as e:
                        self.get_logger().error(f"Int16MultiArray convert error: {e}")

                def _cb_bytes(self, msg):  # AudioData
                    try:
                        self._enqueue(bytes(msg.data))
                    except Exception as e:
                        self.get_logger().error(f"AudioData forward error: {e}")

            node = _Node(self._topic, self._loop, self._out_queue, self._stop_evt)
            self._started_evt.set()

            executor = SingleThreadedExecutor(context=rclpy.get_default_context())
            executor.add_node(node)
            while not self._stop_evt.is_set():
                executor.spin_once(timeout_sec=0.1)
            executor.shutdown()
            node.destroy_node()

        except Exception:
            logger.exception("RosAudioBridge crashed")


# ——————————————————————————————————————————
#                Main audio loop
# ——————————————————————————————————————————
class GeminiAudioLoop:
    """
    Background worker that:
      - Maintains a Gemini live audio session.
      - Sends ROS PCM input upstream; receives PCM out + transcripts downstream.
      - For each transcript slice (user or assistant), calls ChatNode service to
        create/update the message so the UI reflects text while audio is speaking.
      - Supports Stop/Start: Stop sets an event, attempts to close the live session,
        cancels all tasks, and joins the thread so a new run can start cleanly.
    """

    def __init__(self, api_key: str = "") -> None:
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._is_listening = False
        self._chat_id: Optional[str] = None

        # Initialized in run()
        self.audio_in_queue: asyncio.Queue[tuple[bytes, Optional[str]]]
        self.out_queue: asyncio.Queue[bytes]
        self.session = None  # gemini live session (aio client)
        self.playback_stream = None  # PyAudio output stream
        self.api_key = api_key

        # ROS bridge handle
        self._ros_bridge: Optional[RosAudioBridge] = None

        # Logging / turns (optional: write input/output wavs)
        self._turn_id = 0
        self._in_wav: Optional[wave.Wave_write] = None
        self._out_wav: Optional[wave.Wave_write] = None
        self._log_lock: Optional[asyncio.Lock] = None
        self._log_input_dir = Path(os.getenv("AUDIO_LOG_INPUT_DIR", "input"))
        self._log_output_dir = Path(os.getenv("AUDIO_LOG_OUTPUT_DIR", "output"))

        # Service client state (for streaming ChatMessages to ChatNode)
        self._srv_node: Optional[Node] = None
        self._srv_client = None
        self._accum_text: str = ""
        self._last_pib_message_id: str = ""
        self._current_role: Optional[str] = None  # "user" | "assistant" | None

        # Chat update worker (so DB/UI updates don't block audio)
        self._chat_queue: "queue.Queue[CreateOrUpdateChatMessage.Request]" = (
            queue.Queue(maxsize=32)
        )
        self._chat_worker: Optional[threading.Thread] = None
        self._last_srv_call_time: float = 0.0
        self._srv_call_min_interval: float = 0.15  # seconds, 150 ms default

        # Event loop reference (used by stop() to close session)
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        # Live API session resumption handle (updated from SessionResumptionUpdate)
        self._session_handle: Optional[str] = None
        # Pinned model and idle timeout for the session start() was asked to open.
        self._model: Optional[str] = None
        self._idle_timeout_s: Optional[float] = None
        self._last_activity: float = 0.0
        # The local activity gate owns the uplink. Playback tail and a pause
        # while a robot action runs still hold an open turn back.
        self._exclude_until: float = 0.0
        self._session_paused: bool = False
        self._playback_cancel = threading.Event()
        self._hold_speaker = threading.Event()
        self._speaker_ready = threading.Event()
        self._interruption_reported = False
        self._offer_tools = True
        self._action_announcer: Optional[Any] = None
        self._speaking_listener: Optional[Any] = None
        self._speaking = False
        self._send_lock: Optional[asyncio.Lock] = None
        self._pending_typed: list[str] = []
        self._typed_lock = threading.Lock()
        self._typed_drain: Optional[asyncio.Task] = None
        self._playback_slice_bytes = playback_slice_bytes(
            RECEIVE_SAMPLE_RATE, 2, CHANNELS
        )
        self._activity_gate: Optional[ActivityGate] = None
        self._gate_samples: Optional[int] = None
        self._turn_announced = False
        self._register_port = None
        self._usb_lock = threading.Lock()
        self._motion_vad_depth = 0
        self._vad_before_motion: Optional[float] = None
        self._motion_vad_db = motion_vad_threshold_db()
        self._ring = _StatusRing()
        self._usage_metadata: object = None
        self._cloud_client = None
        self._cloud_session_id: Optional[str] = None
        self._live_was_open = False
        self._redactor = _RedactingFilter()
        logging.getLogger().addFilter(self._redactor)

    def submit_typed_text(self, text: str) -> bool:
        """Interrupt playback and send this line into the open live session.

        The session stays open. Speech barge-in is the client's activity
        gate. The line is also written into the chat, so the text is
        visible while the model answers it.
        """
        cleaned = text.strip() if isinstance(text, str) else ""
        loop = self._loop
        if (
            not self._is_listening
            or not cleaned
            or loop is None
            or not loop.is_running()
        ):
            return False
        with self._typed_lock:
            self._pending_typed.append(cleaned)
        loop.call_soon_threadsafe(self._schedule_typed_drain)
        return True

    def _schedule_typed_drain(self) -> None:
        # A new task per wake-up. The lock keeps two of them from sending
        # the same line, and a line that arrives as one task exits is not lost.
        self._typed_drain = asyncio.create_task(self._drain_typed_text())

    async def _drain_typed_text(self) -> None:
        while self.session is not None:
            with self._typed_lock:
                if not self._pending_typed:
                    return
                text = self._pending_typed.pop(0)
            await self._deliver_typed_text(text)

    async def _deliver_typed_text(self, text: str) -> None:
        """Stop the spoken turn and send the typed line. Do not close the session."""
        session = self.session
        if session is None:
            with self._typed_lock:
                self._pending_typed.insert(0, text)
            return
        self._note_activity()
        decision = on_interruption(time.monotonic())
        self._exclude_until = decision["exclude_until"]
        self._playback_cancel.set()
        audio_in = getattr(self, "audio_in_queue", None)
        if audio_in is not None:
            self._drain_queue(audio_in)
            try:
                audio_in.put_nowait((b"", None))
            except asyncio.QueueFull:
                self._playback_cancel.clear()
        self._start_new_stream("user")
        self._send_chat_piece(text, is_user=True, update_db=True, force_flush=True)
        try:
            lock = self._send_lock
            if lock is None:
                await session.send_realtime_input(text=text)
            else:
                async with lock:
                    await session.send_realtime_input(text=text)
        except Exception:
            logger.exception("Could not send typed text into the live session")

    def set_action_announcer(self, announcer) -> None:
        """Speak a robot action on the existing speech player before it runs."""
        self._action_announcer = announcer

    def set_speaking_listener(self, listener) -> None:
        """Called with True while live playback has audio, then False."""
        self._speaking_listener = listener

    def _notify_speaking(self, speaking: bool) -> None:
        speaking = bool(speaking)
        if speaking == self._speaking:
            return
        self._speaking = speaking
        self._show_ring("speaking" if speaking else self._idle_or_listening())
        listener = self._speaking_listener
        if listener is None:
            return
        try:
            listener(speaking)
        except Exception:
            logger.exception("speaking listener failed")

    def _idle_or_listening(self) -> str:
        if self.session is not None:
            return "listening"
        return "idle"

    def _show_ring(self, state: str) -> None:
        try:
            self._ring.show(state)
        except Exception:
            return

    def _install_redactor(self) -> _RedactingFilter:
        redactor = self._redactor
        root = logging.getLogger()
        if redactor not in root.filters:
            root.addFilter(redactor)
        return redactor

    def _read_voice_activity(self) -> bool:
        port = self._register_port
        if port is None:
            return False
        with self._usb_lock:
            return bool(port.read_voice_activity())

    def _gate_for_samples(self, samples: int) -> ActivityGate:
        if samples <= 0:
            samples = CHUNK_SIZE
        gate = self._activity_gate
        if gate is not None and (gate.streaming or samples == self._gate_samples):
            return gate
        pre_roll, silence_end = gate_counts_for_message(samples)
        self._gate_samples = samples
        self._activity_gate = ActivityGate(
            pre_roll_chunks=pre_roll,
            silence_end_chunks=silence_end,
        )
        return self._activity_gate

    def _begin_motion_vad(self) -> None:
        """Raise GAMMAVAD_SR for a motor tool. The read-back is restored later."""
        port = self._register_port
        if port is None:
            return
        with self._usb_lock:
            if self._motion_vad_depth == 0:
                self._vad_before_motion = port.read_vad_threshold_db()
                port.write_vad_threshold_db(self._motion_vad_db)
            self._motion_vad_depth += 1

    def _end_motion_vad(self) -> None:
        port = self._register_port
        if port is None:
            return
        with self._usb_lock:
            if self._motion_vad_depth <= 0:
                return
            self._motion_vad_depth -= 1
            if self._motion_vad_depth > 0:
                return
            previous = self._vad_before_motion
            self._vad_before_motion = None
            if previous is not None:
                port.write_vad_threshold_db(previous)

    def _execute_named_tool(self, tool_name: str, args: dict):
        from pib_hermes_config.visible_state import ANSWER_GESTURE_TOOLS
        from voice_assistant.direct_tool_loop import execute_mcp_tool

        motor = tool_name in ANSWER_GESTURE_TOOLS
        if motor:
            self._begin_motion_vad()
        try:
            return execute_mcp_tool(tool_name, args)
        finally:
            if motor:
                self._end_motion_vad()

    def _fire_head_turn(self, started_at: float) -> None:
        """Turn toward the speaker without waiting on the cloud."""

        def _run() -> None:
            try:
                from voice_assistant import attention as attention_mod

                handle = getattr(attention_mod, "bound_attention_handle", None)
                command = getattr(handle, "command_toward_speaker", None)
                if command is None:
                    return
                port = self._register_port
                angle = None
                if port is not None:
                    with self._usb_lock:
                        angle = port.read_doa_angle()
                if isinstance(angle, int):
                    buffer = getattr(handle, "buffer", None)
                    note = getattr(buffer, "note_doa", None)
                    if note is not None:
                        note(angle)
                command()
            except Exception:
                logger.exception("head turn on activity start failed")
                return
            delay_ms = (time.monotonic() - started_at) * 1000.0
            logger.info("head command delay_ms=%.1f", delay_ms)

        threading.Thread(target=_run, name="activity-head-turn", daemon=True).start()

    def _note_usage(self, message: object) -> None:
        usage = getattr(message, "usage_metadata", None)
        if usage is None:
            usage = getattr(message, "usageMetadata", None)
        if usage is not None:
            self._usage_metadata = usage

    @property
    def is_listening(self) -> bool:
        return self._is_listening

    # ---------- lifecycle ----------

    def start(self, chat_id, model: Optional[str] = None, idle_timeout_s=None) -> None:
        """
        Start the Gemini audio loop in a background thread.
        If already running, this is a no-op (defensive).
        ``model`` is the provider row's pinned live model. The retired preview
        id is not accepted.
        """
        if self._is_listening:
            logger.info("GeminiAudioLoop is already running.")
            return
        if gemini_live_connect_model(model) is None:
            logger.error("Refusing live session without a pinned Gemini model.")
            return

        self._chat_id = chat_id
        self._model = str(model).strip()
        self._idle_timeout_s = None if idle_timeout_s is None else float(idle_timeout_s)
        self._stop_event.clear()

        # Start chat worker first so it can consume requests
        self._start_chat_worker()

        self._thread = threading.Thread(target=self._run_thread, daemon=True)
        self._thread.start()
        self._is_listening = True
        logger.info("GeminiAudioLoop: started")

    def stop(self, join_timeout: float = 5.0) -> None:
        """
        Signal the loop to stop and join the thread.
        - Sets the stop flag.
        - Tries to close the live session (from this thread) via run_coroutine_threadsafe.
        - Joins the worker thread (best-effort) so Start can run cleanly after.
        """
        if not self._is_listening:
            logger.info("GeminiAudioLoop is not running.")
            return

        self._stop_event.set()
        with self._typed_lock:
            self._pending_typed.clear()
        # Wake chat worker so it can exit
        try:
            self._chat_queue.put_nowait(None)  # sentinel
        except Exception:
            pass

        if self._chat_worker:
            self._chat_worker.join(timeout=join_timeout)
            if self._chat_worker.is_alive():
                logger.warning("Chat worker thread did not stop cleanly.")
            else:
                self._chat_worker = None

        # Reset per-stream accumulators to avoid accidental reuse next run
        self._accum_text = ""
        self._last_pib_message_id = ""
        self._current_role = None

        # Try to close the live session from here (async -> thread-safe)
        if self._loop and self.session is not None:
            try:

                async def _close():
                    try:
                        await self.session.close()
                    except Exception:
                        pass

                asyncio.run_coroutine_threadsafe(_close(), self._loop)
            except Exception:
                pass

        if self._thread:
            self._thread.join(timeout=join_timeout)
            if self._thread.is_alive():
                logger.warning("GeminiAudioLoop thread did not stop cleanly.")
            else:
                self._thread = None

        self._is_listening = False
        logger.info("GeminiAudioLoop: stopped")

    def _run_thread(self):
        """
        Thread entrypoint: create an isolated asyncio loop and run self.run().
        This allows the main ROS process to remain responsive while we do async IO.
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            loop.run_until_complete(self.run())
        finally:
            self._loop = None
            loop.close()

    # ---------- small queue helpers ----------

    def _drain_queue(self, q: asyncio.Queue):
        """Best-effort drain to keep queues bounded during teardown."""
        drained = 0
        try:
            while True:
                q.get_nowait()
                drained += 1
        except QueueEmpty:
            if drained:
                logger.debug("Drained %d items from queue.", drained)

    async def _flush_queues(self, where: str):
        """Drain known queues on shutdown to free memory quickly."""
        logger.debug("Queues flushed (%s).", where)
        try:
            self._drain_queue(self.audio_in_queue)
        except Exception:
            pass
        try:
            self._drain_queue(self.out_queue)
        except Exception:
            pass
        logger.debug("Queues flushed (%s).", where)

    # ---------- optional simple WAV logging ----------

    async def _ensure_log_dirs(self):
        for d in (self._log_input_dir, self._log_output_dir):
            d.mkdir(parents=True, exist_ok=True)

    async def _open_turn_logs(self):
        """Open input/output WAV files for the current turn (lazy)."""
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            if self._in_wav or self._out_wav:
                return
            await self._ensure_log_dirs()
            self._turn_id += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            in_path = self._log_input_dir / f"turn{self._turn_id:04d}_{ts}_in.wav"
            out_path = self._log_output_dir / f"turn{self._turn_id:04d}_{ts}_out.wav"

            # Input (16k mono, PCM16)
            self._in_wav = wave.open(str(in_path), "wb")
            self._in_wav.setnchannels(1)
            self._in_wav.setsampwidth(2)  # 16-bit
            self._in_wav.setframerate(SEND_SAMPLE_RATE)

            # Output (24k mono, PCM16)
            self._out_wav = wave.open(str(out_path), "wb")
            self._out_wav.setnchannels(1)
            self._out_wav.setsampwidth(2)
            self._out_wav.setframerate(RECEIVE_SAMPLE_RATE)

            logger.debug(
                "Turn %04d: logging to %s and %s", self._turn_id, in_path, out_path
            )

    async def _close_turn_logs(self):
        """Close any open WAVs gracefully."""
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            try:
                if self._in_wav:
                    self._in_wav.close()
            except Exception:
                logger.exception("Error closing input WAV")
            finally:
                self._in_wav = None

            try:
                if self._out_wav:
                    self._out_wav.close()
            except Exception:
                logger.exception("Error closing output WAV")
            finally:
                self._out_wav = None

    async def _log_input_bytes(self, pcm: bytes):
        """Append input PCM to the 'in' log (optional)."""
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            if self._in_wav is None:
                await self._open_turn_logs()
            try:
                self._in_wav.writeframes(pcm)
            except Exception:
                logger.exception("Failed to write input audio log")

    async def _log_output_bytes(self, pcm: bytes):
        """Append output PCM to the 'out' log (optional)."""
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            if self._out_wav is None:
                await self._open_turn_logs()
            try:
                self._out_wav.writeframes(pcm)
            except Exception:
                logger.exception("Failed to write output audio log")

    # ---------- ChatNode service helpers ----------

    def _ensure_srv_client(self):
        """
        Lazily create a ROS node + service client for CreateOrUpdateChatMessage.
        We keep this very small so AudioLoop stays decoupled from ChatNode internals.
        """
        if self._srv_node is None:
            if not rclpy.ok():
                rclpy.init()
            self._srv_node = Node("gemini_audio_loop_client")
            self._srv_client = self._srv_node.create_client(
                CreateOrUpdateChatMessage, "create_or_update_chat_message"
            )
            if not self._srv_client.wait_for_service(timeout_sec=2.0):
                logger.warning(
                    "Service 'create_or_update_chat_message' not available yet."
                )

    def _start_chat_worker(self):
        """Start background thread that handles CreateOrUpdateChatMessage calls."""
        if self._chat_worker and self._chat_worker.is_alive():
            return

        def _worker():
            logger.debug("Chat worker thread started.")
            while not self._stop_event.is_set():
                try:
                    req = self._chat_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                if req is None:
                    # Sentinel for shutdown
                    break

                try:
                    self._ensure_srv_client()
                    if self._srv_client is None:
                        continue

                    future = self._srv_client.call_async(req)
                    rclpy.spin_until_future_complete(
                        self._srv_node, future, timeout_sec=1.0
                    )

                    if (
                        future.done()
                        and future.result() is not None
                        and future.result().successful
                    ):
                        # keep message_id for subsequent UPDATEs
                        self._last_pib_message_id = future.result().message_id
                except Exception:
                    logger.error(
                        "Chat worker: service call failed or timed out.", exc_info=True
                    )

            logger.debug("Chat worker thread exiting.")

        self._chat_worker = threading.Thread(target=_worker, daemon=True)
        self._chat_worker.start()

    def _start_new_stream(self, role: str):
        """
        Reset accumulation for a new logical message (role = 'user' or 'assistant').
        This ensures we create separate PIB messages for each side of the dialog.
        """
        if self._current_role is not None and self._accum_text:
            prev_is_user = self._current_role == "user"
            self._send_chat_piece(
                text_piece="",  # don’t change accumulator
                is_user=prev_is_user,
                update_db=True,
                force_flush=True,  # ignore throttle
            )

        self._current_role = role
        self._accum_text = ""
        self._last_pib_message_id = ""

    def _send_chat_piece(
        self,
        text_piece: str,
        is_user: bool,
        update_db: bool = True,
        force_flush: bool = False,
    ):
        """
        Send accumulated text to ChatNode service.

        - For user: usually throttled.
        - For assistant: typically force_flush=True -> realtime.
        """
        if self._stop_event.is_set() or not self._is_listening:
            return

        # 1) Update accumulator
        if text_piece:
            self._accum_text = (self._accum_text + " " + text_piece).strip()

        if not self._accum_text:
            return

        # 2) Build request
        req = CreateOrUpdateChatMessage.Request()
        req.chat_id = self._chat_id or ""
        req.text = self._accum_text
        req.is_user = is_user
        req.update_database = update_db
        req.message_id = self._last_pib_message_id  # "" -> CREATE on first call

        # 3) Queue for chat worker
        try:
            self._chat_queue.put_nowait(req)
        except queue.Full:
            logger.debug("Chat worker queue full, dropping chat update.")

    # ---------- tasks ----------

    async def _listen_from_ros(self):
        """Pulls PCM16 mono @16k from ROS topic into out_queue for the live session."""
        logger.debug(
            f"Listening from ROS topic '{ROS_AUDIO_TOPIC}' (expect PCM16 mono @16k)."
        )
        try:
            while not self._stop_event.is_set():
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            logger.debug("_listen_from_ros: cancelled")
            raise

    async def send_realtime(self):
        """Feed every microphone message through the local activity gate.

        Only ``decision.uplink`` is admitted. Activity start and activity end
        marks go out on the gate's edges. Playback tail and a paused robot
        action still hold the uplink, and excluded audio is not kept.
        """
        try:
            while not self._stop_event.is_set():
                msg = await self.out_queue.get()
                pcm = _pcm_from_message(msg)
                if not pcm:
                    continue
                now = time.monotonic()
                held = not admit_live_uplink(
                    now=now,
                    exclude_until=self._exclude_until,
                    session_paused=self._session_paused,
                    streaming=True,
                )
                gate = self._gate_for_samples(len(pcm) // 2)
                was_streaming = gate.streaming
                voice = await asyncio.to_thread(self._read_voice_activity)
                decision = gate.observe(bool(voice), pcm)
                started_at = time.monotonic() if decision.start else 0.0
                if held:
                    await self._drop_held_turn(was_streaming)
                    continue
                in_turn = bool(decision.uplink) or decision.start or decision.end
                if not admit_live_uplink(
                    now=now,
                    exclude_until=self._exclude_until,
                    session_paused=self._session_paused,
                    streaming=in_turn,
                ):
                    continue
                await self._send_gate_decision(
                    decision, _mime_from_message(msg), started_at
                )
        except asyncio.CancelledError:
            logger.debug("send_realtime: cancelled")
            raise
        except Exception as e:
            if self._stop_event.is_set():
                return
            logger.warning(
                "send_realtime: upstream send failed; requesting reconnect.",
                exc_info=True,
            )
            raise ReconnectRequested("send_realtime failed") from e

    async def _drop_held_turn(self, was_streaming: bool) -> None:
        """Exclusion and pause drop the chunk. An announced turn is closed."""
        gate = self._activity_gate
        if gate is not None:
            gate.reset()
        announced = was_streaming and self._turn_announced
        self._turn_announced = False
        if not announced or self.session is None:
            return
        try:
            async with self._send_lock:
                await self.session.send_realtime_input(**activity_end_message())
        except Exception:
            logger.exception("Could not end the turn while the uplink is held")

    async def _send_gate_decision(self, decision, mime: str, started_at: float) -> None:
        session = self.session
        if session is None:
            return
        if decision.start:
            self._fire_head_turn(started_at)
            self._show_ring("listening")
        async with self._send_lock:
            if decision.start:
                await session.send_realtime_input(**activity_start_message())
                self._turn_announced = True
            for chunk in decision.uplink:
                await session.send_realtime_input(
                    audio={"data": chunk, "mime_type": mime}
                )
            if decision.end:
                await session.send_realtime_input(**activity_end_message())
                self._turn_announced = False
        if decision.end and not self._speaking:
            self._show_ring("thinking")

    def _log_user_transcriptions(self, sc):
        """
        Called for each server_content event from Gemini:
        - input_transcription  -> what the user said (streaming text)
        """
        if self._stop_event.is_set() or not self._is_listening:
            return

        # User stream
        input_transcription = getattr(sc, "input_transcription", None)
        if input_transcription and getattr(input_transcription, "text", None):
            text_piece = input_transcription.text.strip()
            if text_piece:
                self._note_activity()
            logger.debug(f"User: {text_piece}")
            if self._current_role != "user":
                self._start_new_stream("user")
            self._send_chat_piece(
                text_piece,
                is_user=True,
                update_db=True,
                force_flush=False,
            )

    def _extract_assistant_text(self, sc) -> Optional[str]:
        """
        Extract assistant (Gemini) text from server_content without sending it.
        Return the text string or None.
        """
        output_transcription = getattr(sc, "output_transcription", None)
        if not output_transcription:
            return None

        txt = getattr(output_transcription, "text", None)
        if txt:
            text_piece = txt.strip()
            if text_piece:
                self._note_activity()
            logger.debug(
                f"Gemini (buffered in _extract_assistant_text) as single object: {text_piece}"
            )
            return text_piece

        return None

    async def receive_audio(self):
        """
        Receives downstream events from Gemini:
        - resp.data -> PCM audio bytes (playback)
        - resp.text -> occasional text events
        - resp.server_content -> transcript slices:
            * user -> handled immediately in _log_user_transcriptions
            * assistant -> extracted and paired with PCM for synchronized display
        """
        while not self._stop_event.is_set():
            try:
                # New turn -> reset stream roles/accumulators so messages are separate
                self._current_role = None
                self._accum_text = ""
                self._last_pib_message_id = ""
                await self._open_turn_logs()
            except Exception as e:
                logger.error(f"Failed to open turn logs: {e}")
                continue

            turn = None
            try:
                turn = self.session.receive()
                assistant_text_piece: Optional[str] = None
                turn_sent_audio = False
                async for resp in turn:
                    self._note_usage(resp)
                    # ----- Session management signals -----
                    # Session resumption checkpoints (store handle for reconnect)
                    session_resumption_update = getattr(
                        resp, "session_resumption_update", None
                    ) or getattr(resp, "sessionResumptionUpdate", None)
                    if session_resumption_update is not None:
                        new_handle = getattr(
                            session_resumption_update, "new_handle", None
                        ) or getattr(session_resumption_update, "newHandle", None)
                        resumable = getattr(
                            session_resumption_update, "resumable", None
                        )
                        if resumable and new_handle:
                            self._session_handle = new_handle

                    # GoAway warning (connection will be terminated soon)
                    go_away_signal = getattr(resp, "go_away", None) or getattr(
                        resp, "goAway", None
                    )
                    if go_away_signal is not None:
                        time_left = getattr(
                            go_away_signal, "time_left", None
                        ) or getattr(go_away_signal, "timeLeft", None)
                        logger.warning(
                            "GoAway received (time_left=%s). Reconnecting...", time_left
                        )
                        raise ReconnectRequested("go_away")
                    # Handle transcripts (user now, assistant buffered)
                    sc = getattr(resp, "server_content", None)
                    if sc is not None:
                        # User text is sent immediately
                        self._log_user_transcriptions(sc)
                        # Assistant text is only extracted and passed along
                        new_piece = self._extract_assistant_text(sc)
                        if new_piece:
                            assistant_text_piece = new_piece
                            logger.debug(
                                f"Buffered for upcoming audio: {assistant_text_piece}"
                            )
                        if interruption_detected(sc):
                            await self._on_interrupted()
                        if getattr(sc, "generation_complete", None) or getattr(
                            sc, "turn_complete", None
                        ):
                            try:
                                self.audio_in_queue.put_nowait((b"", None))
                            except asyncio.QueueFull:
                                pass
                            if getattr(sc, "turn_complete", None):
                                self._interruption_reported = False

                    tool_call = getattr(resp, "tool_call", None)
                    if tool_call is not None:
                        await self._handle_tool_call(tool_call)

                    if data := getattr(resp, "data", None):
                        # PCM audio from Gemini (24k mono). Speech resets the
                        # idle clock. Uplink silence does not: the mic stays open.
                        self._note_activity()
                        try:
                            # Put a pair: (pcm_bytes, assistant_text_piece or None)
                            queued_text = assistant_text_piece
                            logger.debug(f"Queueing audio with text: {queued_text}")

                            self.audio_in_queue.put_nowait((data, assistant_text_piece))
                            turn_sent_audio = True
                            if assistant_text_piece is not None:
                                assistant_text_piece = None
                            # If desired, uncomment to log output audio:
                            # await self._log_output_bytes(data)
                        except asyncio.QueueFull:
                            logger.warning(
                                "Audio input queue is full; dropping data chunk."
                            )
                    elif text := getattr(resp, "text", None):
                        # Occasionally text responses arrive without audio; print for debugging
                        print(text, end="")

                    if self._stop_event.is_set():
                        # Stop requested -> break out and let run() tear down
                        break

            except ReconnectRequested:
                raise

            except Exception as e:
                if self._stop_event.is_set():
                    return
                logger.warning(
                    "receive_audio: downstream receive failed; requesting reconnect.",
                    exc_info=True,
                )
                raise ReconnectRequested("receive_audio failed") from e

            # Clear any leftover audio between turns. Whatever was just playing
            # leaves a short tail in the microphone, so the uplink skips that.
            while not self.audio_in_queue.empty():
                self.audio_in_queue.get_nowait()
            if turn_sent_audio:
                self._exclude_until = max(
                    self._exclude_until,
                    playback_exclusion_deadline(time.monotonic()),
                )

    async def _finish_playback(self, *, cancelled: bool) -> None:
        """A playback end. An interruption drops the queue and the device buffer.

        A robot-action announcement holds the speaker until its own playback
        has finished, then this stream starts again.
        """
        if cancelled:
            self._playback_cancel.clear()
            self._drain_queue(self.audio_in_queue)
            try:
                await asyncio.to_thread(self.playback_stream.stop_stream)
            except Exception:
                logger.exception("Could not stop playback")
            self._speaker_ready.set()
            while self._hold_speaker.is_set() and not self._stop_event.is_set():
                await asyncio.sleep(0.05)
            try:
                await asyncio.to_thread(self.playback_stream.start_stream)
            except Exception:
                logger.exception("Could not restart playback")
        self._exclude_until = playback_exclusion_deadline(time.monotonic())
        self._notify_speaking(False)

    async def _on_interrupted(self) -> None:
        """Stop playback now, exclude the tail, and tell the model the turn was cut."""
        decision = on_interruption(time.monotonic())
        self._exclude_until = decision["exclude_until"]
        self._playback_cancel.set()
        self._drain_queue(self.audio_in_queue)
        if self._interruption_reported or self.session is None:
            return
        self._interruption_reported = True
        try:
            async with self._send_lock:
                await self.session.send_realtime_input(text=decision["report"])
        except Exception:
            logger.exception("Could not report the interruption to the model")

    def _run_one_tool(self, name: str, arguments: dict) -> dict:
        """Announce a robot action, pause the uplink, then run it.

        A gesture during the answer is only the MCP call. Playback is not
        stopped for it, and nothing is announced in its place.
        """
        from pib_hermes_config.visible_state import (
            ANSWER_GESTURE_TOOLS,
            issue_answer_gesture,
        )
        if name in ANSWER_GESTURE_TOOLS:
            try:
                result = issue_answer_gesture(name, arguments, self._execute_named_tool)
            except Exception:
                logger.exception("Answer gesture %s failed", name)
                result = {
                    "ok": False,
                    "error": {"code": "action_failed", "message": f"{name} failed"},
                }
            return {
                "offered": True,
                "executed": True,
                "announcement": None,
                "result": result,
                "reason": "gesture",
            }

        def speak(text: str) -> None:
            announcer = self._action_announcer
            self._speaker_ready.clear()
            self._hold_speaker.set()
            self._playback_cancel.set()
            loop = self._loop
            playback_queue = getattr(self, "audio_in_queue", None)
            if loop is not None and playback_queue is not None:

                def _wake() -> None:
                    try:
                        playback_queue.put_nowait((b"", None))
                    except Exception:
                        pass

                loop.call_soon_threadsafe(_wake)
                self._speaker_ready.wait(timeout=1.0)
            try:
                if announcer is not None:
                    announcer(text)
                else:
                    logger.info("Live action announcement: %s", text)
            finally:
                self._hold_speaker.clear()
                self._exclude_until = playback_exclusion_deadline(time.monotonic())

        def execute(tool_name: str, args: dict):
            return self_loop._execute_named_tool(tool_name, args)

        class _Pause:
            def begin(self) -> None:
                self_loop._session_paused = True

            def end(self) -> None:
                self_loop._session_paused = False

        self_loop = self
        return perform_robot_action(
            name, arguments, speak=speak, execute=execute, pause=_Pause()
        )

    def _live_tool_declarations(self) -> list:
        from voice_assistant.direct_tool_loop import mcp_tool_declarations

        return gemini_function_declarations(mcp_tool_declarations())

    async def _handle_tool_call(self, tool_call) -> None:
        calls = iter_function_calls(tool_call)
        responses = []
        for call in calls:
            try:
                outcome = await asyncio.to_thread(
                    self._run_one_tool, call["name"], call["arguments"]
                )
            except Exception:
                logger.exception("Live robot action %s failed", call["name"])
                self._session_paused = False
                outcome = {
                    "offered": True,
                    "result": {
                        "ok": False,
                        "error": {
                            "code": "action_failed",
                            "message": f"{call['name']} failed",
                        },
                    },
                }
            if not outcome.get("offered", True):
                body = {
                    "ok": False,
                    "error": {
                        "code": "not_offered_in_live_turn",
                        "message": (
                            f"{call['name']} is not offered inside a live turn"
                        ),
                    },
                }
            else:
                body = outcome.get("result")
                if not isinstance(body, dict):
                    body = {"result": body}
            responses.append({"id": call["id"], "name": call["name"], "response": body})
        if not responses or self.session is None:
            return
        try:
            async with self._send_lock:
                await self.session.send_tool_response(function_responses=responses)
        except Exception:
            logger.exception("Could not return the tool result to the live session")

    async def play_audio(self):
        """PyAudio playback consumer for PCM @24k mono + synchronized Gemini text."""
        self.playback_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        try:
            logger.info(
                "Playback stream opened: rate=%s, channels=%s",
                RECEIVE_SAMPLE_RATE,
                CHANNELS,
            )
        except Exception:
            pass

        try:
            while not self._stop_event.is_set():
                item = await self.audio_in_queue.get()
                # Expecting a pair: (pcm_bytes, assistant_text_piece or None)
                pcm, assistant_text_piece = item
                if not pcm or self._playback_cancel.is_set():
                    await self._finish_playback(
                        cancelled=self._playback_cancel.is_set()
                    )
                    continue

                self._notify_speaking(True)
                # Play in short slices so an interruption stops on the next one.
                cut = False
                for chunk in iter_playback_slices(pcm, self._playback_slice_bytes):
                    if self._playback_cancel.is_set():
                        cut = True
                        break
                    await asyncio.to_thread(self.playback_stream.write, chunk)
                if cut or self._playback_cancel.is_set():
                    await self._finish_playback(cancelled=True)
                    continue

                # If there is Gemini text attached to this PCM chunk, send it now.
                logger.debug(f"writing {assistant_text_piece} as gemini in UI")

                if assistant_text_piece:
                    # Ensure we are in assistant stream
                    if self._current_role != "assistant":
                        self._start_new_stream("assistant")
                    logger.debug(f"sending {assistant_text_piece} to the worker")

                    self._send_chat_piece(
                        assistant_text_piece,
                        is_user=False,
                        update_db=True,
                        force_flush=True,
                    )

        except asyncio.CancelledError:
            logger.debug("play_audio: cancelled, closing playback stream")
            try:
                self.playback_stream.close()
            finally:
                raise
        except Exception:
            logger.exception("play_audio error")

    def _note_activity(self) -> None:
        """Speech happened. Silence on the uplink does not count."""
        self._last_activity = time.monotonic()

    async def _watch_idle(self) -> None:
        """End the session when the personality's idle timeout elapses."""
        timeout = self._idle_timeout_s
        if not timeout or timeout <= 0:
            while not self._stop_event.is_set():
                await asyncio.sleep(0.5)
            return
        while not self._stop_event.is_set():
            await asyncio.sleep(0.5)
            if idle_expired(self._last_activity, time.monotonic(), float(timeout)):
                logger.info(
                    "Live session idle for %ss; stopping so it does not keep billing.",
                    timeout,
                )
                raise LiveIdleTimeout()

    async def run(self):
        """
        Main async entry:
        - Maintains a Gemini live session, reconnecting automatically on GoAway / disconnect.
        - Keeps context window compression on, and session resumption when enabled.
        - Stops on the personality idle timeout instead of reconnecting.
        - Starts ROS bridge + tasks (send/receive/play).
        """
        live_model = gemini_live_connect_model(self._model)
        if live_model is None:
            logger.error("Refusing live session without a pinned Gemini model.")
            return

        grant = self._request_live_grant(live_model)
        if grant is None:
            return
        if grant.use_cascade:
            logger.info(
                "pib.Cloud live session not opened; cascade reason=%s",
                grant.reason or "unspecified",
            )
            return
        if not grant.token:
            logger.error("pib.Cloud live grant has no ephemeral token")
            return

        client = genai.Client(
            api_key=grant.token,
            http_options={"api_version": "v1alpha"},
        )
        if grant.model:
            live_model = grant.model

        # Read chat personality/description from PIB to seed the model (once)
        description = "You are pib, a humanoid robot."
        personality = None
        try:
            successful, personality = voice_assistant_client.get_personality_from_chat(
                self._chat_id
            )
            if not successful:
                logger.error(f"no personality found for id {self._chat_id}")
                personality = None
            elif getattr(personality, "description", None):
                description = personality.description
        except Exception:
            logger.exception(
                "Failed to fetch personality; using default system instruction."
            )
            personality = None

        base_config = dict(CONFIG)
        base_config.update(live_turn_detection_config())
        # The SDK reads activity handling on the realtime input config. The
        # client-owned gate still publishes it at the top level as well.
        handling = base_config.get("activity_handling")
        realtime = base_config.get("realtime_input_config")
        if (
            handling
            and isinstance(realtime, dict)
            and "activity_handling" not in realtime
        ):
            realtime = dict(realtime)
            realtime["activity_handling"] = handling
            base_config["realtime_input_config"] = realtime
        base_config["system_instruction"] = description
        self._offer_tools = bool(getattr(personality, "tool_calling", True))

        async def _connect_config():
            connection_config = dict(base_config)
            connection_config["context_window_compression"] = (
                context_window_compression(CWC_TRIGGER_TOKENS, CWC_TARGET_TOKENS)
            )

            if ENABLE_SESSION_RESUMPTION:
                # Gemini Developer API supports sessi:contentReference[oaicite:3]{index=3}dle,
                # but NOT the 'transparent' parameter.
                sr = {"handle": self._session_handle if self._session_handle else None}
                connection_config["session_resumption"] = sr

            return connection_config

        self._send_lock = asyncio.Lock()
        self._register_port = open_register_port()
        pre_roll, silence_end = gate_counts_for_message(CHUNK_SIZE)
        logger.info(
            "activity gate samples_per_message=%s pre_roll_chunks=%s silence_end_chunks=%s",
            CHUNK_SIZE,
            pre_roll,
            silence_end,
        )

        # Outer loop: reconnect as needed until stopped
        try:
            await self._run_live_sessions(client, live_model, _connect_config)
        finally:
            self._show_ring("idle")
            if self._live_was_open:
                self._settle_live_usage(live_model)
            port = self._register_port
            self._register_port = None
            close = getattr(port, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    logger.exception("Could not close the microphone register port")

        logger.info("GeminiAudioLoop.run: terminated")

    def _request_live_grant(self, model: str):
        """Ephemeral live grant. The pib token and the grant token are not logged."""
        from voice_assistant.cloud_session import CloudSessionClient, CloudSessionError
        from voice_assistant.cloud_token import CloudTokenError, resolve_cloud_token
        from voice_assistant.pib_cloud_stream import pib_cloud_base_url

        redactor = self._install_redactor()
        redactor.add(self.api_key)
        try:
            pib_token = resolve_cloud_token()
        except CloudTokenError as exc:
            logger.error("pib.Cloud token unavailable: %s", exc)
            return None
        redactor.add(pib_token)
        try:
            cloud = CloudSessionClient(
                base_url=_session_root(pib_cloud_base_url()),
                token=pib_token,
            )
            grant = cloud.open_session(model=model)
        except CloudSessionError as exc:
            logger.error(
                "pib.Cloud did not grant a live session code=%s status=%s",
                exc.code,
                exc.status,
            )
            return None
        except Exception:
            logger.exception("pib.Cloud session request failed")
            return None
        redactor.add(grant.token)
        self._cloud_client = cloud
        self._cloud_session_id = grant.session_id
        return grant

    def _settle_live_usage(self, model: str) -> None:
        """Report the live session's own usage when the session is finished."""
        client = self._cloud_client
        session_id = self._cloud_session_id
        if client is None or not session_id:
            return
        usage = self._usage_metadata
        if usage is None and self.session is not None:
            usage = getattr(self.session, "usage_metadata", None)
            if usage is None:
                usage = getattr(self.session, "usageMetadata", None)
        data = _usage_dict(usage)
        audio_in = _int_field(data, "audio_input_tokens", "audioInputTokens")
        if audio_in is None:
            audio_in = _modality_tokens(
                data, ("prompt_tokens_details", "promptTokensDetails"), "AUDIO"
            )
        audio_out = _int_field(data, "audio_output_tokens", "audioOutputTokens")
        if audio_out is None:
            audio_out = _modality_tokens(
                data, ("response_tokens_details", "responseTokensDetails"), "AUDIO"
            )
        text_tokens = _int_field(data, "text_tokens", "textTokens")
        if text_tokens is None:
            text_tokens = _modality_tokens(
                data, ("prompt_tokens_details", "promptTokensDetails"), "TEXT"
            ) + _modality_tokens(
                data, ("response_tokens_details", "responseTokensDetails"), "TEXT"
            )
        video_in = _int_field(data, "video_input_tokens", "videoInputTokens")
        if video_in is None:
            video_in = _modality_tokens(
                data, ("prompt_tokens_details", "promptTokensDetails"), "VIDEO"
            )
        try:
            client.report_usage(
                session_id,
                audio_input_tokens=audio_in or 0,
                audio_output_tokens=audio_out or 0,
                text_tokens=text_tokens or 0,
                video_input_tokens=video_in or 0,
                usage_metadata=data or None,
                model=model,
            )
        except Exception as exc:
            logger.warning(
                "live session usage was not reported code=%s",
                getattr(exc, "code", None),
            )

    async def _run_live_sessions(self, client, live_model: str, connect_config) -> None:
        while not self._stop_event.is_set():
            ros_started = False
            tasks = []
            try:
                gemini_config = await connect_config()
                if self._offer_tools:
                    try:
                        declarations = await asyncio.to_thread(
                            self._live_tool_declarations
                        )
                        if declarations:
                            gemini_config["tools"] = [
                                {"function_declarations": declarations}
                            ]
                    except Exception:
                        logger.exception("Live session will not offer robot tools")

                async with client.aio.live.connect(
                    model=live_model, config=gemini_config
                ) as session:
                    self.session = session
                    self._live_was_open = True
                    self._turn_announced = False
                    self._activity_gate = None
                    self._gate_samples = None
                    self._show_ring("listening")
                    self._schedule_typed_drain()
                    self._note_activity()
                    self._exclude_until = 0.0
                    self._session_paused = False
                    self._playback_cancel.clear()
                    self._hold_speaker.clear()
                    self._interruption_reported = False
                    self.audio_in_queue = asyncio.Queue[tuple[bytes, Optional[str]]]()
                    self.out_queue = asyncio.Queue(maxsize=5)
                    self._log_lock = asyncio.Lock()

                    # Optional folders for WAV logs
                    await self._ensure_log_dirs()

                    # Start ROS subscriber thread that feeds PCM into out_queue
                    topic = os.getenv("ROS_AUDIO_TOPIC", "audio_stream")
                    self._ros_bridge = RosAudioBridge(
                        topic, asyncio.get_running_loop(), self.out_queue
                    )
                    self._ros_bridge.start()
                    ros_started = True
                    logger.info("RosAudioBridge started.")

                    # Start async tasks
                    tasks = [
                        asyncio.create_task(self._listen_from_ros()),
                        asyncio.create_task(self.send_realtime()),
                        asyncio.create_task(self.receive_audio()),
                        asyncio.create_task(self.play_audio()),
                        asyncio.create_task(self._watch_idle()),
                    ]

                    # Wait until one task errors (or requests reconnect)
                    done, pending = await asyncio.wait(
                        tasks, return_when=asyncio.FIRST_EXCEPTION
                    )
                    for t in done:
                        exc = t.exception()
                        if exc:
                            raise exc

                    # No exception, session ended normally -> stop loop
                    logger.info("Live session ended normally; stopping.")
                    break

            except LiveIdleTimeout:
                self._stop_event.set()
                break

            except ReconnectRequested as e:
                if self._stop_event.is_set():
                    break
                logger.info(
                    "Reconnect requested (%s). session_handle=%s",
                    str(e),
                    "present" if self._session_handle else "none",
                )

            except asyncio.CancelledError:
                logger.debug("GeminiAudioLoop.run: cancelled")
                break

            except Exception:
                logger.exception("GeminiAudioLoop.run: unexpected error")
                break

            finally:
                # Cancel all tasks and wait; ignore exceptions during teardown
                for t in tasks:
                    try:
                        t.cancel()
                    except Exception:
                        pass
                if tasks:
                    await asyncio.gather(*tasks, return_exceptions=True)

                # Stop ROS bridge if it was started
                if ros_started and self._ros_bridge:
                    try:
                        self._ros_bridge.stop()
                    except Exception:
                        pass
                    self._ros_bridge = None

                # Close playback stream if still open
                if self.playback_stream:
                    try:
                        self.playback_stream.close()
                    except Exception:
                        pass
                    self.playback_stream = None

                # Drain queues and close WAVs
                try:
                    await self._flush_queues("shutdown")
                except Exception:
                    pass
                try:
                    await self._close_turn_logs()
                except Exception:
                    pass

                if self.session is not None:
                    self._note_usage(self.session)
                self.session = None

            # Backoff before reconnecting
            if not self._stop_event.is_set():
                await asyncio.sleep(LIVE_RECONNECT_BACKOFF_S)


_capture_bound_attention()


def main():
    # Optional local entry for quick manual test (not used in ROS launch)
    loop = GeminiAudioLoop(api_key=os.getenv("GOOGLE_API_KEY", ""))
    loop.start()


if __name__ == "__main__":
    main()
