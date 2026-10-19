"""PC-86: ducking is a gain envelope, and the tests measure it on the sample timeline.

The promise to the user is "pib gets quiet for a moment and then either continues
mid-sentence or stops". Both halves are sample arithmetic, so both are asserted
sample by sample rather than by listening.
"""

from __future__ import annotations

import numpy as np
import pytest

from voice_assistant.speech_gate import (
    DUCK_GAIN,
    RAMP_MS,
    SAMPLE_RATE,
    STATE_CANCELLED,
    STATE_DUCKING,
    STATE_SPEAKING,
    SpeechGate,
)

RAMP_SAMPLES = int(SAMPLE_RATE * RAMP_MS / 1000)


def tone(ms: int, value: int = 8000) -> bytes:
    return np.full(int(SAMPLE_RATE * ms / 1000), value, dtype="<i2").tobytes()


def samples(data: bytes) -> np.ndarray:
    return np.frombuffer(data, dtype="<i2")


def test_an_unducked_turn_is_sample_identical() -> None:
    gate = SpeechGate()
    gate.begin_turn()
    pcm = tone(20, 1000)
    assert gate.process(pcm) == pcm
    assert gate.gain == pytest.approx(1.0)


def test_duck_ramps_down_within_the_ramp_and_holds_the_level() -> None:
    gate = SpeechGate()
    gate.begin_turn()
    gate.duck()
    assert gate.state == STATE_DUCKING
    out = samples(gate.process(tone(RAMP_MS * 3)))
    # Monotonic decrease over the ramp, no step.
    ramp = out[:RAMP_SAMPLES]
    assert np.all(np.diff(ramp) <= 0)
    assert ramp[0] < 8000 and ramp[0] > 8000 * DUCK_GAIN
    # After the ramp the level is the ducked one.
    assert out[-1] == pytest.approx(8000 * DUCK_GAIN, abs=2)
    assert gate.gain == pytest.approx(DUCK_GAIN, abs=1e-3)


def test_duck_is_audible_in_the_first_chunk_after_the_flag() -> None:
    """The 10 ms promise: a 20 ms chunk handed over after duck() is already quieter."""

    gate = SpeechGate()
    gate.begin_turn()
    gate.duck()
    first = samples(gate.process(tone(20)))
    # Everything after the 5 ms ramp is at 15 %; the chunk average is far below half.
    assert np.mean(np.abs(first)) < 8000 * 0.5


def test_resume_returns_to_full_without_losing_a_sample() -> None:
    gate = SpeechGate()
    gate.begin_turn()
    reference = tone(40, 4000)
    gate.duck()
    ducked = gate.process(reference)
    gate.resume()
    assert gate.state == STATE_SPEAKING
    out = samples(gate.process(tone(RAMP_MS * 2 + 10)))
    # It starts just above the ducked level and is back at full within the ramp.
    assert out[0] > 8000 * DUCK_GAIN
    assert out[0] < 8000 * 0.5
    assert out[-1] == 8000
    assert gate.gain == pytest.approx(1.0)
    assert len(ducked) == len(reference), "ducking must not drop or add samples"


def test_resume_continues_the_same_audio_not_a_restart() -> None:
    """The buffer is untouched: the samples after a resumed duck are the next ones."""

    gate = SpeechGate()
    gate.begin_turn()
    first = samples(gate.process(tone(20, 1000)))
    gate.duck()
    gate.process(tone(20, 1000))
    gate.resume()
    gate.process(tone(RAMP_MS, 1000))
    after = samples(gate.process(tone(20, 1000)))
    assert len(first) == len(after)
    assert np.all(after == 1000), "back at full level, same waveform"


def test_cancel_renders_nothing_more() -> None:
    gate = SpeechGate()
    gate.begin_turn()
    gate.cancel()
    assert gate.state == STATE_CANCELLED
    assert gate.process(tone(20)) == b""
    assert gate.gain == 0.0


def test_the_envelope_does_not_depend_on_chunk_boundaries() -> None:
    """7 ms + 3 ms must render exactly like one 10 ms chunk."""

    whole = SpeechGate()
    whole.begin_turn()
    whole.duck()
    expected = whole.process(tone(10))

    split = SpeechGate()
    split.begin_turn()
    split.duck()
    actual = split.process(tone(7)) + split.process(tone(3))
    assert actual == expected


def test_a_partial_sample_is_refused() -> None:
    gate = SpeechGate()
    gate.begin_turn()
    with pytest.raises(ValueError):
        gate.process(b"\x00\x00\x00")
