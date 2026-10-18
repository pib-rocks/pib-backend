"""PC-87: the playback clock - position, prebuffer and the silence of an underrun.

The relay's pacing window and its history truncation both trust `played_ms`. These
tests pin that number to the samples actually handed to the device, and pin the
underrun behaviour to a continuous signal rather than a click.
"""

from __future__ import annotations

import numpy as np
import pytest

from voice_assistant.playback_clock import (
    FRAME_BYTES,
    FRAME_MS,
    SAMPLE_RATE,
    PlaybackClock,
)


def pcm(ms: int, value: int = 1000) -> bytes:
    return np.full(int(SAMPLE_RATE * ms / 1000), value, dtype="<i2").tobytes()


def samples(data: bytes) -> np.ndarray:
    return np.frombuffer(data, dtype="<i2")


def test_nothing_plays_before_the_prebuffer_is_full() -> None:
    clock = PlaybackClock(prebuffer_ms=60)
    clock.begin_turn(1)
    assert clock.feed(pcm(40)) == []
    assert clock.started is False
    assert clock.position_ms == 0.0
    ready = clock.feed(pcm(20))
    assert len(ready) == 1
    assert clock.started is True
    assert len(ready[0]) == len(pcm(60))


def test_position_counts_frames_not_wall_time() -> None:
    clock = PlaybackClock(prebuffer_ms=20)
    clock.begin_turn(1)
    clock.feed(pcm(100))
    assert clock.position_ms == pytest.approx(100.0, abs=1e-6)
    assert clock.bytes_played == len(pcm(100))


def test_only_whole_frames_are_released() -> None:
    clock = PlaybackClock(prebuffer_ms=20)
    clock.begin_turn(1)
    released = clock.feed(pcm(30))  # 1.5 frames
    assert len(released[0]) == FRAME_BYTES
    assert clock.buffered_ms == pytest.approx(10.0)
    rest = clock.feed(pcm(10))
    assert len(rest[0]) == FRAME_BYTES
    assert clock.buffered_ms == pytest.approx(0.0)


def test_progress_is_reported_every_interval_and_not_sooner() -> None:
    clock = PlaybackClock(prebuffer_ms=20, progress_interval_ms=250)
    clock.begin_turn(1)
    clock.feed(pcm(200))
    assert clock.progress_due() is None
    clock.feed(pcm(100))
    assert clock.progress_due() == 300
    assert clock.progress_due() is None
    clock.feed(pcm(300))
    assert clock.progress_due() == 600


def test_a_starved_buffer_decays_from_the_last_sample_instead_of_clicking() -> None:
    clock = PlaybackClock(prebuffer_ms=20)
    clock.begin_turn(1)
    clock.feed(pcm(20, value=1000))
    silence = samples(clock.starved(FRAME_MS))
    assert len(silence) == SAMPLE_RATE * FRAME_MS // 1000
    # It starts at the level that was playing and ends at zero - no step to silence.
    assert silence[0] == pytest.approx(1000, abs=2)
    assert silence[-1] == 0
    assert np.all(np.diff(silence.astype(int)) <= 0)
    assert clock.underruns == 1


def test_the_next_chunk_after_an_underrun_is_ramped_back_in() -> None:
    clock = PlaybackClock(prebuffer_ms=20)
    clock.begin_turn(1)
    clock.feed(pcm(20, value=1000))
    clock.starved(FRAME_MS)
    resumed = samples(clock.feed(pcm(20, value=1000))[0])
    assert resumed[0] < 200  # starts near zero
    assert resumed[-1] == pytest.approx(1000, abs=2)  # and is back at level


def test_a_new_turn_resets_position_and_counters() -> None:
    clock = PlaybackClock(prebuffer_ms=20, progress_interval_ms=250)
    clock.begin_turn(1)
    clock.feed(pcm(300))
    clock.starved()
    assert clock.position_ms > 0 and clock.underruns == 1
    clock.begin_turn(2)
    assert clock.turn_id == 2
    assert clock.position_ms == 0.0
    assert clock.underruns == 0
    assert clock.started is False


def test_a_partial_sample_is_refused() -> None:
    clock = PlaybackClock()
    clock.begin_turn(1)
    with pytest.raises(ValueError):
        clock.feed(b"\x00\x00\x00")
