"""PC-102 fast VAD sampler and motion-time threshold guard."""

from __future__ import annotations

import logging

from voice_assistant.fast_vad import (
    DEFAULT_SAMPLE_INTERVAL_S,
    GAMMAVAD_SR,
    MotionVadThresholdGuard,
    FastVadSampler,
)


class FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.t = start

    def now(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        self.t += seconds


def test_sampler_interval_is_at_most_64_ms_and_counts_gaps() -> None:
    assert DEFAULT_SAMPLE_INTERVAL_S <= 0.064

    clock = FakeClock()
    reads = iter([False, True, False])
    sampler = FastVadSampler(lambda: next(reads), clock=clock.now)

    assert sampler.poll() is False
    assert sampler.sample_count == 1

    clock.advance(0.02)
    assert sampler.poll() is None

    clock.advance(0.05)
    assert sampler.poll() is True
    assert sampler.sample_count == 2

    clock.advance(0.064)
    assert sampler.poll() is False
    assert sampler.sample_count == 3
    assert sampler.p95_gap_s >= 0.064


def test_motion_guard_raises_and_restores_threshold() -> None:
    state = {GAMMAVAD_SR: 3.5}
    writes: list[tuple[str, float]] = []

    def read(name: str) -> float:
        return float(state[name])

    def write(name: str, value: float) -> None:
        writes.append((name, value))
        state[name] = value

    guard = MotionVadThresholdGuard(
        read, write, motion_db=8.5, log=logging.getLogger("test")
    )
    guard.begin_motion()
    assert writes[-1] == (GAMMAVAD_SR, 8.5)
    guard.end_motion()
    assert writes[-1] == (GAMMAVAD_SR, 3.5)
    assert state[GAMMAVAD_SR] == 3.5


def test_motion_guard_restores_even_when_raise_write_fails() -> None:
    state = {GAMMAVAD_SR: 3.5}
    writes: list[tuple[str, float]] = []

    def read(name: str) -> float:
        return float(state[name])

    def write(name: str, value: float) -> None:
        writes.append((name, value))
        if value == 8.5:
            raise OSError("register busy")
        state[name] = value

    guard = MotionVadThresholdGuard(
        read, write, motion_db=8.5, log=logging.getLogger("test")
    )
    guard.begin_motion()
    guard.end_motion()
    assert writes[-1] == (GAMMAVAD_SR, 3.5)
    assert state[GAMMAVAD_SR] == 3.5


def test_motion_scope_restores_in_finally() -> None:
    state = {GAMMAVAD_SR: 3.5}

    def read(name: str) -> float:
        return float(state[name])

    def write(name: str, value: float) -> None:
        state[name] = value

    guard = MotionVadThresholdGuard(
        read, write, motion_db=8.5, log=logging.getLogger("test")
    )
    try:
        with guard.motion_scope():
            assert state[GAMMAVAD_SR] == 8.5
            raise RuntimeError("motor fault")
    except RuntimeError:
        pass
    assert state[GAMMAVAD_SR] == 3.5


def test_an_unreadable_threshold_leaves_the_device_alone_and_does_not_raise() -> None:
    """A missing USB device must not break pause_uplink() in the audio path."""

    writes: list[tuple[str, float]] = []

    def read(name: str) -> float:
        raise OSError("device gone")

    guard = MotionVadThresholdGuard(
        read, lambda name, value: writes.append((name, value))
    )

    guard.begin_motion()  # must not raise
    guard.end_motion()
    assert writes == []
    assert guard.active is False
