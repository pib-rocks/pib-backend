"""Wire the duplex voice path into a running ROS node.

Everything that needs the hardware lives here and nowhere else: the speaker stream
at the relay's format (16 kHz mono s16le), the microphone subscription, the array's
VAD topic, the HID switches and the recorder's transcription action. The loop and
the session stay importable and testable without ROS or a sound device, which is
why this file is thin and boring on purpose.

Two hardware facts decide the shape:

* The array delivers channel 0 already echo-cancelled and beamformed - that is the
  channel the relay hears, and it is why the speaker can play while the mic is open.
* The echo canceller only works when its HID switches are in the right state
  (``ECHOONOFF=1``, ``NLATTENONOFF=1``, ``AGCONOFF=0``). They are applied once at
  start-up and logged, because a silently absent switch turns every barge-in into
  a false positive.
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from typing import Callable, Optional

from voice_assistant.array_tuning_adapter import (
    open_array_tuning,
    tuning_is_voice,
    tuning_read,
    tuning_write,
)
from voice_assistant.audio_devices import choose_input
from voice_assistant.duplex_loop import DuplexVoiceLoop, silence_after_s
from voice_assistant.duplex_session import DuplexVoiceSession
from voice_assistant.fast_vad import (
    DEFAULT_SAMPLE_INTERVAL_S,
    FastVadSampler,
    MotionVadThresholdGuard,
)
from voice_assistant.hid_tuning import apply_tuning, open_hid
from voice_assistant.pib_duplex import websocket_connection
from voice_assistant.uplink_gate import UplinkGate

logger = logging.getLogger(__name__)

MIC_TOPIC = "audio_stream"
VAD_TOPIC = "/voice_activity"
OUTPUT_RATE = 16_000
OUTPUT_CHANNELS = 1

TuningOpener = Callable[[], Optional[object]]


@dataclass(frozen=True)
class DuplexVadWiring:
    """How this node sources voice activity for the duplex session."""

    fast_sampler: Optional[FastVadSampler]
    use_topic_fallback: bool


def duplex_silence_after_s(
    pause_threshold: Optional[float] = None,
    override: Optional[float] = None,
) -> float:
    if override is None:
        raw = os.getenv("PIB_DUPLEX_SILENCE_AFTER", "").strip()
        if raw:
            override = float(raw)
    return silence_after_s(pause_threshold, override)


def build_duplex_vad_wiring(
    *,
    opener: TuningOpener = open_array_tuning,
    tuning: Optional[object] = None,
    clock: Callable[[], float] = time.monotonic,
    log: Optional[logging.Logger] = None,
) -> DuplexVadWiring:
    """Prefer direct ``VOICEACTIVITY`` reads; fall back to ``/voice_activity`` once."""
    log = log or logger
    if tuning is None:
        tuning = opener()
    if tuning is not None:
        sampler = FastVadSampler(
            tuning_is_voice(tuning),
            interval_s=DEFAULT_SAMPLE_INTERVAL_S,
            clock=clock,
        )
        return DuplexVadWiring(fast_sampler=sampler, use_topic_fallback=False)
    log.warning(
        "XVF-3000 tuning port unavailable; using %s for voice activity",
        VAD_TOPIC,
    )
    return DuplexVadWiring(fast_sampler=None, use_topic_fallback=True)


def build_motion_vad_guard(
    *,
    opener: TuningOpener = open_array_tuning,
    tuning: Optional[object] = None,
    motion_db: Optional[float] = None,
) -> Optional[MotionVadThresholdGuard]:
    if tuning is None:
        tuning = opener()
    if tuning is None:
        return None
    if motion_db is None:
        raw = os.getenv("PIB_MOTION_VAD_THRESHOLD_DB", "8.5").strip()
        motion_db = float(raw)
    return MotionVadThresholdGuard(
        tuning_read(tuning),
        tuning_write(tuning),
        motion_db=motion_db,
    )


def open_speaker(rate: int = OUTPUT_RATE, channels: int = OUTPUT_CHANNELS):
    """Open the playback stream in the relay's format.

    No resampling, no conversion: the cloud already speaks 16 kHz mono s16le, and a
    resample in the middle would only add latency and a second clock.
    """
    import pyaudio  # type: ignore[import-not-found]

    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=pyaudio.paInt16,
        channels=channels,
        rate=rate,
        output=True,
        frames_per_buffer=320,
    )
    return audio, stream


def describe_input() -> object:
    """Which input this robot records from, for the start-up log."""
    import pyaudio  # type: ignore[import-not-found]

    audio = pyaudio.PyAudio()
    try:
        names = [
            audio.get_device_info_by_index(index).get("name", "")
            for index in range(audio.get_device_count())
        ]
    finally:
        audio.terminate()
    return choose_input([str(name) for name in names])


def tune_array() -> None:
    """Put the echo canceller's switches into their known-good state, once."""
    device = open_hid()
    if device is None:
        logger.warning(
            "no ReSpeaker HID device found; echo cancellation tuning skipped"
        )
        return
    outcome = apply_tuning(device)
    if outcome.applied:
        logger.info("HID switches applied: %s", ", ".join(outcome.applied))
    for name, reason in outcome.refused:
        logger.warning("HID switch %s refused: %s", name, reason)


def build_duplex_voice_loop(
    node,
    *,
    url: str,
    token: Optional[str] = None,
    request_transcription: Callable[[Callable[[str], None]], None],
    on_speaking: Optional[Callable[[bool], None]] = None,
    sink=None,
    mic_channels: int = 1,
    silence_after_s: Optional[float] = None,
    vad_wiring: Optional[DuplexVadWiring] = None,
    motion_vad_guard: Optional[MotionVadThresholdGuard] = None,
    tuning_opener: TuningOpener = open_array_tuning,
) -> DuplexVoiceLoop:
    """Create the session against this robot's hardware and return the running loop.

    ``sink`` is injectable so the same wiring can be exercised on a machine without
    a speaker; production leaves it ``None`` and gets the PyAudio stream.
    """
    from std_msgs.msg import Bool, Int16MultiArray  # type: ignore[import-not-found]

    audio = stream = None
    if sink is None:
        audio, stream = open_speaker()
        sink = stream.write

    holder: dict = {}

    def on_turn_end(reason: str) -> None:
        loop = holder.get("loop")
        if loop is None:
            return
        if reason.startswith("barge-in"):
            loop.on_barge_in()
        elif reason == "complete":
            loop.on_turn_finished()

    if silence_after_s is None:
        silence_after_s = duplex_silence_after_s()
    # One open handle for both: two find() calls would claim the USB interface twice.
    tuning = (
        tuning_opener() if (vad_wiring is None or motion_vad_guard is None) else None
    )
    if vad_wiring is None:
        vad_wiring = build_duplex_vad_wiring(tuning=tuning)
    if motion_vad_guard is None:
        motion_vad_guard = build_motion_vad_guard(tuning=tuning)

    uplink_gate = UplinkGate(silence_after_s=silence_after_s, clock=time.monotonic)

    session = DuplexVoiceSession(
        url=url,
        token=token,
        sink=sink,
        connect=websocket_connection,
        on_speaking=on_speaking,
        on_turn_end=on_turn_end,
        input_channels=mic_channels,
        uplink_gate=uplink_gate,
        motion_vad_guard=motion_vad_guard,
    )

    fast_sampler = vad_wiring.fast_sampler

    def on_microphone(msg) -> None:
        if fast_sampler is not None:
            speech = fast_sampler.poll(time.monotonic())
            if speech is not None:
                session.on_vad(speech)
        session.on_microphone(bytes(memoryview(msg.data)))

    def on_vad(msg) -> None:
        session.on_vad(bool(msg.data))

    node.create_subscription(Int16MultiArray, MIC_TOPIC, on_microphone, 10)
    if vad_wiring.use_topic_fallback:
        node.create_subscription(Bool, VAD_TOPIC, on_vad, 10)

    loop = DuplexVoiceLoop(
        session=session,
        request_transcription=request_transcription,
        on_close=_closer(audio, stream),
    )
    holder["loop"] = loop
    return loop


def _closer(audio, stream):
    def close() -> None:
        if stream is not None:
            try:
                stream.stop_stream()
                stream.close()
            except Exception:  # noqa: BLE001
                pass
        if audio is not None:
            audio.terminate()

    return close
