"""PC-102 uplink gate: pre-roll ring and turn open/close."""

from __future__ import annotations

from voice_assistant.uplink_gate import UplinkGate

SAMPLE_RATE = 16_000
BYTES_PER_SAMPLE = 2
CHUNK_MS = 20
CHUNK_BYTES = int(SAMPLE_RATE * CHUNK_MS / 1000) * BYTES_PER_SAMPLE


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        self.t += seconds


def _chunk(tag: int) -> bytes:
    return bytes([tag & 0xFF, 0]) * (CHUNK_BYTES // 2)


def test_nothing_is_uploaded_outside_an_open_turn() -> None:
    clock = FakeClock()
    gate = UplinkGate(clock=clock.now, silence_after_s=0.7)

    for tag in range(40):
        gate.set_speech(False)
        assert gate.feed(_chunk(tag)) == b""

    assert gate.bytes_uploaded == 0
    assert gate.bytes_uploaded_outside_turn == 0


def test_first_upload_of_a_turn_includes_at_least_300_ms_preroll() -> None:
    clock = FakeClock()
    gate = UplinkGate(clock=clock.now, preroll_s=0.32, silence_after_s=0.7)

    # Fifteen 20 ms chunks = 300 ms in the ring before speech.
    for tag in range(15):
        gate.set_speech(False)
        assert gate.feed(_chunk(tag)) == b""

    gate.set_speech(True)
    trigger = _chunk(99)
    first = gate.feed(trigger)

    min_preroll_bytes = int(0.3 * SAMPLE_RATE * BYTES_PER_SAMPLE)
    assert len(first) >= min_preroll_bytes + len(trigger)
    assert gate.is_open
    # The counter must name exactly what left the robot before the turn opened:
    # everything in `first` except the trigger chunk itself.
    assert gate.bytes_uploaded_outside_turn == len(first) - len(trigger)
    assert gate.bytes_uploaded_outside_turn >= min_preroll_bytes


def test_turn_closes_after_trailing_silence() -> None:
    clock = FakeClock()
    gate = UplinkGate(clock=clock.now, silence_after_s=0.1)

    gate.set_speech(True)
    gate.feed(_chunk(1))
    assert gate.is_open

    gate.set_speech(False)
    for _ in range(6):
        gate.feed(_chunk(2))

    assert not gate.is_open
    assert gate.feed(_chunk(3)) == b""
