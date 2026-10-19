"""Frame-accurate playback position, prebuffer and underrun silence (PC-87).

Two features of the relay depend on the robot knowing exactly where it is in a
turn: the pacing window (the cloud stops generating ~4 s ahead of the robot's
position) and the history truncation after a barge-in (the turn is cut at the
position the robot reports). Neither can use wall-clock time - a stalled device or
a slow WLAN chunk makes wall time and audio position diverge, and the divergence is
exactly what a wrong truncation looks like.

So the position is counted in samples actually handed to the device, in 20 ms
frames. That is the only clock the audio path trusts.

The other half is what happens when the buffer runs dry mid-sentence. Letting the
device insert an XRUN produces a click; a click is a discontinuity in the echo
canceller's reference and an audible artefact. :meth:`PlaybackClock.starved`
therefore returns silence with a short cosine fade at both edges, so the signal
stays continuous.

This module is deliberately device-free: it is fed the same bytes the player would
write and it answers what to write.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

SAMPLE_RATE = 16_000
SAMPLE_BYTES = 2
FRAME_MS = 20
FRAME_BYTES = SAMPLE_RATE * FRAME_MS // 1000 * SAMPLE_BYTES  # 640

DEFAULT_PREBUFFER_MS = 60
DEFAULT_PROGRESS_INTERVAL_MS = 250
FADE_MS = 2.0


def _silence(samples: int) -> bytes:
    return b"\x00\x00" * samples


def _fade_shape(samples: int) -> np.ndarray:
    """A cosine from 0 to 1 (or 1 to 0) over ``samples`` points."""
    if samples <= 0:
        return np.empty(0)
    return 0.5 - 0.5 * np.cos(np.linspace(0.0, np.pi, samples))


class PlaybackClock:
    """One turn's position: what has been played, what is due, what to say when it starves."""

    def __init__(
        self,
        *,
        sample_rate: int = SAMPLE_RATE,
        prebuffer_ms: int = DEFAULT_PREBUFFER_MS,
        progress_interval_ms: int = DEFAULT_PROGRESS_INTERVAL_MS,
        fade_ms: float = FADE_MS,
    ) -> None:
        self.sample_rate = sample_rate
        self._prebuffer_bytes = int(sample_rate * prebuffer_ms / 1000) * SAMPLE_BYTES
        self._progress_interval_ms = progress_interval_ms
        self._fade_samples = max(1, int(sample_rate * fade_ms / 1000))
        self.turn_id = 0
        self._pending = bytearray()
        self._played_samples = 0
        self._reported_ms = 0
        self._started = False
        self.underruns = 0
        self.bytes_played = 0
        self._last_sample = 0
        self._needs_ramp_in = False

    # ------------------------------------------------------------------ turn

    def begin_turn(self, turn_id: int) -> None:
        self.turn_id = turn_id
        self._pending.clear()
        self._played_samples = 0
        self._reported_ms = 0
        self._started = False
        self.underruns = 0
        self.bytes_played = 0
        self._last_sample = 0
        self._needs_ramp_in = False

    @property
    def position_ms(self) -> float:
        """How much of the turn has reached the device, in milliseconds."""
        return self._played_samples * 1000.0 / self.sample_rate

    @property
    def started(self) -> bool:
        """False while the prebuffer is still filling - nothing has been written yet."""
        return self._started

    @property
    def buffered_ms(self) -> float:
        return len(self._pending) / SAMPLE_BYTES * 1000.0 / self.sample_rate

    # ------------------------------------------------------------------ feeding

    def feed(self, pcm: bytes) -> list[bytes]:
        """Accept audio and return the chunks that may be written to the device now.

        Nothing is returned until the prebuffer is full: starting on the first frame
        of a WLAN turn means starting on the first underrun.
        """
        if len(pcm) % SAMPLE_BYTES:
            raise ValueError("chunk is not a whole number of 16-bit samples")
        self._pending.extend(pcm)
        if not self._started:
            if len(self._pending) < self._prebuffer_bytes:
                return []
            self._started = True
        # Whole frames only: the device writes 20 ms at a time.
        ready_frames = len(self._pending) // FRAME_BYTES
        if ready_frames == 0:
            return []
        ready = bytes(self._pending[: ready_frames * FRAME_BYTES])
        del self._pending[: ready_frames * FRAME_BYTES]
        written = self._accept(ready)
        return [written]

    def _accept(self, rendered: bytes) -> bytes:
        """Count what was written, and ramp back in if the stream just came out of silence."""
        samples = np.frombuffer(rendered, dtype="<i2").astype(np.float64)
        if self._needs_ramp_in and samples.size:
            fade = min(self._fade_samples, samples.size)
            samples[:fade] *= _fade_shape(fade)
            self._needs_ramp_in = False
        rendered = np.clip(np.rint(samples), -32768, 32767).astype("<i2").tobytes()
        self.bytes_played += len(rendered)
        self._played_samples += len(rendered) // SAMPLE_BYTES
        if samples.size:
            self._last_sample = int(samples[-1])
        return rendered

    # ------------------------------------------------------------------ underrun

    def starved(self, ms: float = FRAME_MS) -> bytes:
        """The buffer ran dry: decay from the last sample to silence, then hold zeros.

        A jump from speech to zeros (and later back) is a click in the reference the
        echo canceller subtracts - and an audible artefact. Decaying over 2 ms keeps
        the signal continuous, and the next real chunk is ramped back in.
        """
        self.underruns += 1
        count = max(1, int(self.sample_rate * ms / 1000))
        out = np.zeros(count, dtype=np.float64)
        fade = min(self._fade_samples, count)
        if self._last_sample:
            out[:fade] = _fade_shape(fade)[::-1] * self._last_sample
        self._last_sample = 0
        self._needs_ramp_in = True
        return np.clip(np.rint(out), -32768, 32767).astype("<i2").tobytes()

    # ------------------------------------------------------------------ progress

    def progress_due(self) -> int | None:
        """The ``played_ms`` to report, or ``None`` while the interval has not passed."""
        position = int(self.position_ms)
        if position - self._reported_ms >= self._progress_interval_ms:
            self._reported_ms = position
            return position
        return None
