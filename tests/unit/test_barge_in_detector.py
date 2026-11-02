"""Unit tests for hardware AEC/VAD BargeInDetector and VoiceAssistantNode wiring (PR-2188)."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
HERMES_CFG = ROOT / "pib_hermes_config"
VOICE_PKG = ROOT / "ros_packages" / "voice_assistant"

for p in (HERMES_CFG, VOICE_PKG):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from pib_hermes_config.barge_in import BargeInDetector
from ros_stubs import install_ros_stubs

install_ros_stubs()


def test_barge_in_triggers_at_200ms_when_speech_detected_is_true():
    """Decision D1=1A: 4 consecutive 50 ms /voice_activity=True samples (200 ms)
    plus /speech_detected=True triggers barge-in immediately."""
    detector = BargeInDetector(confirm_ms=200.0, fallback_confirm_ms=250.0)
    detector.observe_speech_detected(True, now_ms=1000.0)

    # Tick 1 (t=1000 -> 50 ms sustained)
    d1 = detector.observe_voice_activity(True, now_ms=1000.0, can_interrupt=True)
    assert not d1.triggered and d1.sustained_ms == 50.0

    # Tick 2 (t=1050 -> 100 ms sustained)
    d2 = detector.observe_voice_activity(True, now_ms=1050.0, can_interrupt=True)
    assert not d2.triggered and d2.sustained_ms == 100.0

    # Tick 3 (t=1100 -> 150 ms sustained)
    d3 = detector.observe_voice_activity(True, now_ms=1100.0, can_interrupt=True)
    assert not d3.triggered and d3.sustained_ms == 150.0

    # Tick 4 (t=1150 -> 200 ms sustained + speech_detected=True -> FIRES)
    d4 = detector.observe_voice_activity(True, now_ms=1150.0, can_interrupt=True)
    assert d4.triggered is True
    assert d4.sustained_ms == 200.0
    assert d4.speech_detected is True
    assert d4.reason == "vad_and_speech_detected"


def test_barge_in_fallback_triggers_at_250ms_when_speech_detected_lags():
    """Decision D1=1A fallback: if /speech_detected (10 Hz) has not arrived yet,
    sustained /voice_activity for 250 ms triggers as a fallback."""
    detector = BargeInDetector(confirm_ms=200.0, fallback_confirm_ms=250.0)

    for offset in (0.0, 50.0, 100.0, 150.0):
        d = detector.observe_voice_activity(
            True, now_ms=2000.0 + offset, can_interrupt=True
        )
        assert not d.triggered

    # At +200 ms offset (250 ms sustained), fallback fires even without speech_detected
    d_fallback = detector.observe_voice_activity(
        True, now_ms=2200.0, can_interrupt=True
    )
    assert d_fallback.triggered is True
    assert d_fallback.sustained_ms == 250.0
    assert d_fallback.reason == "vad_fallback_window"


def test_transient_cough_under_200ms_resets_on_silence_and_never_triggers():
    """A 150 ms transient click or cough resets when /voice_activity drops to False."""
    detector = BargeInDetector(confirm_ms=200.0, fallback_confirm_ms=250.0)
    detector.observe_speech_detected(True, now_ms=3000.0)

    detector.observe_voice_activity(True, now_ms=3000.0, can_interrupt=True)
    detector.observe_voice_activity(True, now_ms=3050.0, can_interrupt=True)
    d_reset = detector.observe_voice_activity(False, now_ms=3100.0, can_interrupt=True)
    assert not d_reset.triggered

    # Next isolated tick starts fresh at 50 ms instead of continuing from 100 ms
    d_fresh = detector.observe_voice_activity(True, now_ms=3150.0, can_interrupt=True)
    assert not d_fresh.triggered
    assert d_fresh.sustained_ms == 50.0


def test_earcon_mute_and_inactive_state_suppress_barge_in():
    """While START_SIGNAL_FILE earcon is playing or can_interrupt is False, VAD never fires."""
    detector = BargeInDetector(
        confirm_ms=200.0, fallback_confirm_ms=250.0, earcon_mute_ms=350.0
    )
    detector.mute_for_earcon(now_ms=5000.0)
    detector.observe_speech_detected(True, now_ms=5050.0)

    for t in (5050.0, 5100.0, 5150.0, 5200.0, 5250.0):
        d = detector.observe_voice_activity(True, now_ms=t, can_interrupt=True)
        assert not d.triggered
        assert d.reason == "inactive_or_muted"

    # Also suppressed when can_interrupt=False (e.g. robot already listening)
    for t in (6000.0, 6050.0, 6100.0, 6150.0):
        d = detector.observe_voice_activity(True, now_ms=t, can_interrupt=False)
        assert not d.triggered


def test_voice_assistant_hardware_vad_triggers_barge_in_with_preroll_and_no_start_beep(
    monkeypatch,
):
    """Decision D2=2A & D4=4A: when hardware VAD confirms barge-in during robot
    speech, VoiceAssistantNode cancels the active chat turn, clears the playback
    queue, skips START_SIGNAL_FILE, and starts record_audio with silent_before=0.0
    so audio_recorder prepends the 500 ms pre-roll buffer."""
    import types
    from voice_assistant.assistant_turn_callbacks_mixin import (
        AssistantTurnCallbacksMixin,
    )

    clock = {"ms": 10000.0}
    assistant_stub = types.ModuleType("voice_assistant.assistant")
    assistant_stub.MAX_SILENT_SECONDS_BEFORE = 8.0
    assistant_stub.START_SIGNAL_FILE = "/start.wav"
    assistant_stub.STOP_SIGNAL_FILE = "/stop.wav"
    assistant_stub.FALLBACK_REPLY = "fallback"
    assistant_stub.now_ms = lambda: clock["ms"]
    monkeypatch.setitem(sys.modules, "voice_assistant.assistant", assistant_stub)

    class DummyNode(AssistantTurnCallbacksMixin):
        pass

    node = DummyNode()
    node.cycle = 1
    node._playback_epoch = 1
    node.turning_off = False
    node.waiting_for_transcribed_text = False
    node._answer_audio_pending = True
    node._engine_fallback = False
    node._turn_has_playback = True
    node.state = SimpleNamespace(
        turned_on=True,
        chat_id="chat-barge-1",
        speaking=True,
        live_session=False,
        personality_id="p1",
        personality_name="pib",
        using_fallback=False,
    )
    node.personality = SimpleNamespace(
        personality_id="p1",
        name="pib",
        gender="Male",
        language="German",
        pause_threshold=1.0,
        stt_engine="local_whisper",
    )
    node.gemini_loop = SimpleNamespace(is_listening=False)
    node.chat_id_to_is_listening = {"chat-barge-1": False}
    node._barge_in_detector = BargeInDetector(
        confirm_ms=200.0, fallback_confirm_ms=250.0
    )

    events = []
    node.get_logger = lambda: SimpleNamespace(
        info=lambda msg: events.append(("log", msg)),
        warning=lambda msg: events.append(("warn", msg)),
        debug=lambda msg: None,
        error=lambda msg: events.append(("err", msg)),
    )
    node._invalidate_voice_async_work = lambda: setattr(
        node, "_answer_audio_pending", False
    )
    node._close_latency_turn = lambda *, incomplete: events.append(
        ("close_trace", incomplete)
    )
    node.stop_chat = lambda cid: events.append(("stop_chat", cid))
    node._publish_voice_state = lambda: events.append(
        ("publish_state", node.state.speaking)
    )
    node._refresh_personality_turn_taking = lambda: None
    node.set_is_listening = lambda cid, val: events.append(("listening", cid, val))
    node.play_audio_from_file = lambda fp, cb=None: events.append(("play_file", fp))
    node.record_audio = lambda before, after, on_stop, on_text: events.append(
        ("record_audio", before, after)
    )
    node.clear_playback_queue = lambda on_cleared: (
        events.append(("clear_queue",)),
        on_cleared(),
    )

    # Feed /speech_detected=True and 4 ticks of /voice_activity=True
    node._on_hardware_speech_detected(SimpleNamespace(data=True))
    for step in (0.0, 50.0, 100.0, 150.0):
        clock["ms"] = 10000.0 + step
        node._on_hardware_voice_activity(SimpleNamespace(data=True))

    # Verify stop_chat, clear_queue, NO play_file (no start beep!), and record_audio(0.0, 1.0)
    assert ("stop_chat", "chat-barge-1") in events
    assert ("clear_queue",) in events
    assert not any(e[0] == "play_file" for e in events)
    assert ("record_audio", 0.0, 1.0) in events
    assert ("listening", "chat-barge-1", True) in events
