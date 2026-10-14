"""The local gate keeps room silence on the robot."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from voice_assistant.live_gate import (  # noqa: E402
    CHUNK_MS,
    PRE_ROLL_CHUNKS,
    SAMPLES_PER_CHUNK,
    SILENCE_END_CHUNKS,
    VAD_POLL_INTERVAL_S,
    ActivityGate,
    PreRollBuffer,
)


def test_timing_constants_describe_a_320_ms_pre_roll():
    """512 samples at 16 kHz is one 32 ms chunk, and ten of them are 320 ms."""
    assert CHUNK_MS == 32
    assert SAMPLES_PER_CHUNK == 512
    assert PRE_ROLL_CHUNKS == 10
    assert SILENCE_END_CHUNKS == 14
    assert VAD_POLL_INTERVAL_S == 0.03
    assert PRE_ROLL_CHUNKS * CHUNK_MS == 320
    assert SAMPLES_PER_CHUNK * 1000 == CHUNK_MS * 16_000


def test_pre_roll_keeps_the_newest_chunks_oldest_first():
    buffer = PreRollBuffer(capacity=3)
    buffer.push(b"a")
    buffer.push(b"b")
    buffer.push(b"c")
    buffer.push(b"d")
    assert len(buffer) == 3
    assert buffer.drain() == (b"b", b"c", b"d")
    assert len(buffer) == 0
    assert buffer.drain() == ()


def test_start_flushes_the_ring_and_the_trigger_once():
    """The trigger is the last byte of the ring, not a second copy of it."""
    gate = ActivityGate(pre_roll_chunks=3, silence_end_chunks=14)
    assert gate.observe(False, b"a").uplink == ()
    assert gate.observe(False, b"b").uplink == ()
    assert gate.observe(False, b"c").uplink == ()
    decision = gate.observe(True, b"d")
    assert decision.start is True
    assert decision.end is False
    assert decision.uplink == (b"b", b"c", b"d")
    assert decision.uplink.count(b"d") == 1
    assert gate.streaming is True


def test_silence_outside_the_window_uplinks_nothing():
    gate = ActivityGate()
    decision = gate.observe(False, b"room")
    assert decision == type(decision)()
    assert decision.uplink == ()
    assert gate.streaming is False


def test_one_thousand_silent_observations_never_leave_the_robot():
    """Token-saving claim, by construction.

    A thousand silent microphone chunks produce an empty uplink. The
    provider is not offered the room.
    """
    gate = ActivityGate()
    sent: list[bytes] = []
    for index in range(1000):
        decision = gate.observe(False, f"silence-{index}".encode())
        sent.extend(decision.uplink)
        assert decision.start is False
        assert decision.end is False
    assert sent == []
    assert gate.streaming is False


def test_pre_roll_capacity_bounds_what_a_start_sends():
    gate = ActivityGate(pre_roll_chunks=PRE_ROLL_CHUNKS, silence_end_chunks=14)
    for index in range(50):
        assert gate.observe(False, bytes([index])).uplink == ()
    decision = gate.observe(True, b"trigger")
    expected = tuple(bytes([index]) for index in range(41, 50)) + (b"trigger",)
    assert decision.start is True
    assert decision.uplink == expected
    assert len(decision.uplink) == PRE_ROLL_CHUNKS


def test_turn_ends_after_exactly_silence_end_chunks():
    silence_end = 4
    gate = ActivityGate(pre_roll_chunks=2, silence_end_chunks=silence_end)
    started = gate.observe(True, b"speech")
    assert started.start is True
    assert gate.streaming is True
    for index in range(silence_end - 1):
        tail = f"tail-{index}".encode()
        decision = gate.observe(False, tail)
        assert decision.end is False
        assert decision.start is False
        assert decision.uplink == (tail,)
        assert gate.streaming is True
    last = gate.observe(False, b"tail-last")
    assert last.end is True
    assert last.start is False
    assert last.uplink == (b"tail-last",)
    assert gate.streaming is False


def test_speech_inside_the_window_resets_the_silence_counter():
    gate = ActivityGate(pre_roll_chunks=2, silence_end_chunks=3)
    gate.observe(True, b"start")
    gate.observe(False, b"s1")
    gate.observe(False, b"s2")
    continued = gate.observe(True, b"more")
    assert continued.start is False
    assert continued.end is False
    assert continued.uplink == (b"more",)
    assert gate.streaming is True
    gate.observe(False, b"t1")
    gate.observe(False, b"t2")
    assert gate.streaming is True
    ended = gate.observe(False, b"t3")
    assert ended.end is True
    assert gate.streaming is False


def test_second_start_after_reset_drains_only_the_new_ring():
    gate = ActivityGate(pre_roll_chunks=3, silence_end_chunks=1)
    gate.observe(False, b"old-a")
    gate.observe(False, b"old-b")
    first = gate.observe(True, b"old-trigger")
    assert b"old-trigger" in first.uplink
    gate.reset()
    assert gate.streaming is False
    gate.observe(False, b"new-a")
    gate.observe(False, b"new-b")
    second = gate.observe(True, b"new-trigger")
    assert second.start is True
    assert second.uplink == (b"new-a", b"new-b", b"new-trigger")
    assert b"old-a" not in second.uplink
    assert b"old-b" not in second.uplink
    assert b"old-trigger" not in second.uplink


def test_streaming_speech_uplinks_only_the_new_chunk():
    gate = ActivityGate(pre_roll_chunks=4, silence_end_chunks=2)
    gate.observe(False, b"pre")
    gate.observe(True, b"trig")
    nxt = gate.observe(True, b"next")
    assert nxt.start is False
    assert nxt.end is False
    assert nxt.uplink == (b"next",)
    assert gate.streaming is True
