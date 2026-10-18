"""Ducking and the two-stage barge-in, as a gain envelope over the playback stream (PC-86).

Today an interruption ends playback: the queue is dropped and the model is told.
That is right for a confirmed interruption and wrong for a cough - every cough
currently silences pib. The cloud now decides within 250-400 ms (PC-80), so the
robot's job in that window is to get quiet *without losing its place*.

The whole mechanism is one floating gain applied to the samples on their way to
the device:

* ``duck()`` ramps to 15 % over 5 ms. The buffer is untouched, so the position in
  the turn is untouched.
* ``resume()`` ramps back to 100 % over 5 ms. The same samples continue where they
  stopped - the render is sample-identical to an unducked run apart from the gain.
* ``cancel()`` stops rendering this turn at all.

Why a gain and not a pause: the echo canceller in the microphone array is fed from
the same signal. A hard stop or a jump in level is a discontinuity, and a
discontinuity is a click - which the canceller then has to fight and the person
hears. A 5 ms ramp is inaudible and leaves the AEC reference continuous.

The class is deliberately free of audio hardware, ROS and threads: it takes bytes
and returns bytes, which is what makes the timing assertions below possible.
"""

from __future__ import annotations

import numpy as np

SAMPLE_RATE = 16_000
SAMPLE_BYTES = 2
DUCK_GAIN = 0.15
RAMP_MS = 5

STATE_IDLE = "idle"
STATE_SPEAKING = "speaking"
STATE_DUCKING = "ducking"
STATE_CANCELLED = "cancelled"


class SpeechGate:
    """One turn's output gain: full, ducked, or gone.

    ``process`` is called with the next chunk on its way to the device and returns
    the chunk with the envelope applied. Chunk size does not have to be a multiple
    of anything: the envelope is kept as a per-sample value and continues across
    calls, so a 7 ms chunk and a 20 ms chunk produce the same audio.
    """

    def __init__(
        self,
        *,
        sample_rate: int = SAMPLE_RATE,
        duck_gain: float = DUCK_GAIN,
        ramp_ms: float = RAMP_MS,
    ) -> None:
        if not 0.0 < duck_gain <= 1.0:
            raise ValueError("duck_gain must be in (0, 1]")
        if ramp_ms < 0:
            raise ValueError("ramp_ms must be >= 0")
        self.sample_rate = sample_rate
        self.duck_gain = duck_gain
        self._ramp_samples = max(1, int(sample_rate * ramp_ms / 1000))
        self._gain = 1.0
        self._target = 1.0
        self._step = 0.0
        self._samples_to_target = 0
        self.state = STATE_IDLE

    # ------------------------------------------------------------------ control

    @property
    def gain(self) -> float:
        """The gain that will be applied to the next sample."""
        return self._gain if self.state != STATE_CANCELLED else 0.0

    def begin_turn(self) -> None:
        self._gain = 1.0
        self._target = 1.0
        self._step = 0.0
        self._samples_to_target = 0
        self.state = STATE_SPEAKING

    def duck(self) -> None:
        """Asked to get quiet: ramp down, keep playing underneath."""
        if self.state == STATE_CANCELLED:
            return
        self._ramp_to(self.duck_gain)
        self.state = STATE_DUCKING

    def resume(self) -> None:
        """The noise was not speech: come back up at the same position."""
        if self.state == STATE_CANCELLED:
            return
        self._ramp_to(1.0)
        self.state = STATE_SPEAKING

    def cancel(self) -> None:
        """The turn is over: nothing more of it is rendered."""
        self.state = STATE_CANCELLED
        self._gain = 0.0
        self._target = 0.0
        self._samples_to_target = 0

    def _ramp_to(self, target: float) -> None:
        delta = target - self._gain
        if delta == 0:
            self._samples_to_target = 0
            return
        self._target = target
        self._samples_to_target = self._ramp_samples
        self._step = delta / self._ramp_samples

    # ------------------------------------------------------------------ rendering

    def process(self, pcm: bytes) -> bytes:
        """Apply the current envelope to ``pcm`` and advance it."""
        if self.state == STATE_CANCELLED:
            return b""
        if not pcm:
            return pcm
        if len(pcm) % SAMPLE_BYTES:
            raise ValueError("chunk is not a whole number of 16-bit samples")

        samples = np.frombuffer(pcm, dtype="<i2").astype(np.float64)
        gains = self._envelope(samples.size)
        rendered = np.clip(np.rint(samples * gains), -32768, 32767).astype("<i2")
        return rendered.tobytes()

    def _envelope(self, count: int) -> np.ndarray:
        gains = np.empty(count, dtype=np.float64)
        position = 0
        while position < count:
            if self._samples_to_target == 0:
                gains[position:] = self._target
                self._gain = self._target
                return gains
            run = min(self._samples_to_target, count - position)
            ramp = self._gain + self._step * np.arange(1, run + 1)
            gains[position : position + run] = ramp
            self._gain = float(ramp[-1])
            self._samples_to_target -= run
            position += run
            if self._samples_to_target == 0:
                self._gain = self._target
        return gains
