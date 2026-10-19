"""Microphone uplink gating: pre-roll ring and turn open/close (PC-102).

Pure Python — no ROS. While no user turn is open, audio is retained in a ring
but not uploaded. A turn opens on voice activity, flushes the ring plus the
trigger chunk, stays open until trailing silence exceeds the configured limit.
"""

from __future__ import annotations

from collections import deque
from typing import Callable, Deque, Optional

DEFAULT_PREROLL_S = 0.32
DEFAULT_SILENCE_AFTER_S = 0.7
SAMPLE_RATE = 16_000
SAMPLE_BYTES = 2


class UplinkGate:
    """Ring buffer plus open/close state machine for cloud microphone uplink."""

    def __init__(
        self,
        *,
        preroll_s: float = DEFAULT_PREROLL_S,
        silence_after_s: float = DEFAULT_SILENCE_AFTER_S,
        sample_rate: int = SAMPLE_RATE,
        bytes_per_sample: int = SAMPLE_BYTES,
        clock: Callable[[], float] = None,
    ) -> None:
        if preroll_s <= 0:
            raise ValueError("preroll_s must be positive")
        if silence_after_s <= 0:
            raise ValueError("silence_after_s must be positive")
        self._preroll_capacity = int(preroll_s * sample_rate * bytes_per_sample)
        self._silence_after_s = float(silence_after_s)
        self._bytes_per_second = sample_rate * bytes_per_sample
        self._clock = clock or (lambda: 0.0)
        self._ring: Deque[bytes] = deque()
        self._ring_bytes = 0
        self._open = False
        self._speech = False
        self._silence_elapsed_s = 0.0
        self.bytes_uploaded = 0
        self.bytes_uploaded_outside_turn = 0

    @property
    def is_open(self) -> bool:
        return self._open

    @property
    def preroll_capacity_bytes(self) -> int:
        return self._preroll_capacity

    def set_speech(self, speech: bool, now: Optional[float] = None) -> None:
        """Update the latest voice-activity signal (may be called between chunks)."""
        self._speech = bool(speech)

    def feed(self, pcm: bytes, now: Optional[float] = None) -> bytes:
        """Return the PCM that may be sent upstream for this chunk (possibly empty)."""
        if not pcm:
            return b""
        duration_s = len(pcm) / self._bytes_per_second

        if not self._open:
            if self._speech:
                preroll = self._drain_ring()
                # This is the only audio that ever leaves the robot while no turn
                # was open: what the ring kept before the trigger. Counting it makes
                # the "nothing outside a turn" claim a number instead of a promise.
                self.bytes_uploaded_outside_turn += len(preroll)
                self._open = True
                self._silence_elapsed_s = 0.0
                out = preroll + pcm
            else:
                self._push_ring(pcm)
                return b""
        else:
            out = pcm
            if self._speech:
                self._silence_elapsed_s = 0.0
            else:
                self._silence_elapsed_s += duration_s
                if self._silence_elapsed_s >= self._silence_after_s:
                    self._open = False
                    self._silence_elapsed_s = 0.0
                    self._push_ring(pcm)

        if out:
            self.bytes_uploaded += len(out)
        return out

    def _push_ring(self, pcm: bytes) -> None:
        self._ring.append(pcm)
        self._ring_bytes += len(pcm)
        while self._ring_bytes > self._preroll_capacity and self._ring:
            dropped = self._ring.popleft()
            self._ring_bytes -= len(dropped)

    def _drain_ring(self) -> bytes:
        if not self._ring:
            return b""
        parts = list(self._ring)
        self._ring.clear()
        self._ring_bytes = 0
        return b"".join(parts)
