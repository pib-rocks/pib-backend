"""Loud audio with a voice-activity signal that reports no speech.

Measured on the test robot 192.168.1.172 (2026-10-15): counts for the same
sentence differed between intervals (VOICEACTIVITY 0 of 156 in one, 17 of 188 and
SPEECHDETECTED 50 of 188 in another) with room acoustics, human speech and
playback alignment uncontrolled, so no cause follows from that comparison. What
is established is a real communication failure: the array's control reads
returned '[Errno 32] Pipe error' and the device then left the USB bus.

A run like that can make the turn-based recorder count the window as silence and
end on max_silent_seconds_before. Loud audio alone is NOT a fault: the array's
echo canceller may be keeping the robot's own playback out of the detectors on
purpose. This diagnostic reports the observation and says so. It does not end
the turn and it claims no repair.
"""

from __future__ import annotations

from pathlib import Path

from pib_hermes_config.live_interaction import (
    ARRAY_VAD_SILENT_CHUNKS,
    ARRAY_VAD_SILENT_CLEARED,
    ARRAY_VAD_SILENT_WARNING,
    ArrayVadWarningState,
    array_vad_silent_warning,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
AUDIO_RECORDER = (
    REPO_ROOT
    / "ros_packages"
    / "voice_assistant"
    / "voice_assistant"
    / "audio_recorder.py"
)

NOT_A_FAULT_CANDIDATE = "This is not a fault by itself"
OWN_PLAYBACK_CANDIDATE = "the robot's own playback"
TELEMETRY_CANDIDATE = "telemetry is unavailable"
EVIDENCE_CANDIDATE = "Pipe error"


def _chunks(state, voice_activity, amplitude_silent, count):
    messages = []
    for _ in range(count):
        message, state = array_vad_silent_warning(
            voice_activity, amplitude_silent, state
        )
        messages.append(message)
    return messages, state


def test_loud_and_vad_false_for_n_chunks_warns_once():
    messages, state = _chunks(
        ArrayVadWarningState(), False, False, ARRAY_VAD_SILENT_CHUNKS
    )
    assert messages[:-1] == [None] * (ARRAY_VAD_SILENT_CHUNKS - 1)
    assert messages[-1] == ARRAY_VAD_SILENT_WARNING
    assert state.active is True
    assert NOT_A_FAULT_CANDIDATE in ARRAY_VAD_SILENT_WARNING
    assert OWN_PLAYBACK_CANDIDATE in ARRAY_VAD_SILENT_WARNING
    assert TELEMETRY_CANDIDATE in ARRAY_VAD_SILENT_WARNING
    assert EVIDENCE_CANDIDATE in ARRAY_VAD_SILENT_WARNING
    assert "recorder counts that as silence" in ARRAY_VAD_SILENT_WARNING
    # The text must not claim a repair that a reboot or a re-enumeration performs.
    for claim in ("re-enumerating", "working again after", "clears it"):
        assert claim not in ARRAY_VAD_SILENT_WARNING, claim


def test_further_loud_false_chunks_do_not_repeat():
    state = ArrayVadWarningState(active=True, loud_false_chunks=ARRAY_VAD_SILENT_CHUNKS)
    messages, state = _chunks(state, False, False, ARRAY_VAD_SILENT_CHUNKS)
    assert messages == [None] * ARRAY_VAD_SILENT_CHUNKS
    assert state.active is True


def test_voice_activity_returning_clears_the_warning():
    state = ArrayVadWarningState(active=True, loud_false_chunks=ARRAY_VAD_SILENT_CHUNKS)
    message, state = array_vad_silent_warning(True, False, state)
    assert message == ARRAY_VAD_SILENT_CLEARED
    assert state == ArrayVadWarningState()
    again, state = array_vad_silent_warning(True, False, state)
    assert again is None
    assert state.active is False


def test_a_quiet_room_does_not_warn():
    count = ARRAY_VAD_SILENT_CHUNKS * 2
    messages, state = _chunks(ArrayVadWarningState(), False, True, count)
    assert messages == [None] * count
    assert state.active is False
    assert state.loud_false_chunks == 0


def test_a_quiet_chunk_breaks_the_loud_run_and_does_not_clear():
    messages, state = _chunks(
        ArrayVadWarningState(), False, False, ARRAY_VAD_SILENT_CHUNKS - 1
    )
    assert messages == [None] * (ARRAY_VAD_SILENT_CHUNKS - 1)
    message, state = array_vad_silent_warning(False, True, state)
    assert message is None
    assert state.active is False
    assert state.loud_false_chunks == 0
    message, state = array_vad_silent_warning(False, False, state)
    assert message is None

    active = ArrayVadWarningState(
        active=True, loud_false_chunks=ARRAY_VAD_SILENT_CHUNKS
    )
    message, active = array_vad_silent_warning(False, True, active)
    assert message is None
    assert active.active is True


def test_amplitude_fallback_before_a_vad_sample_does_not_warn():
    count = ARRAY_VAD_SILENT_CHUNKS * 2
    for amplitude_silent in (False, True):
        messages, state = _chunks(ArrayVadWarningState(), None, amplitude_silent, count)
        assert messages == [None] * count
        assert state == ArrayVadWarningState()


def test_a_new_run_warns_again_after_voice_activity_returns():
    state = ArrayVadWarningState(active=True, loud_false_chunks=ARRAY_VAD_SILENT_CHUNKS)
    _, state = array_vad_silent_warning(True, False, state)
    messages, state = _chunks(state, False, False, ARRAY_VAD_SILENT_CHUNKS)
    assert messages[:-1] == [None] * (ARRAY_VAD_SILENT_CHUNKS - 1)
    assert messages[-1] == ARRAY_VAD_SILENT_WARNING
    assert state.active is True


def test_the_recorder_calls_it_from_its_silence_path():
    source = AUDIO_RECORDER.read_text(encoding="utf-8")
    silence = source.split("def is_silent(", 1)[1].split("\n    def ", 1)[0]
    assert "array_vad_silent_warning(" in silence
    assert "return not turn_based_is_speech(" in silence
    record = source.split("def record_audio(", 1)[1].split("\n    def ", 1)[0]
    assert "if self.is_silent(data_chunk):" in record
    assert "max_silent_seconds_before" in record
