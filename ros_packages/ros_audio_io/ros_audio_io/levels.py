"""Pure helpers for levels from the mono PCM stream published by ros-audio-io."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

INT16_SCALE = 32768.0


def calculate_levels(samples: Iterable[int]) -> Tuple[float, float]:
    """Return normalized ``(RMS, peak)`` for signed 16-bit mono PCM samples.

    The result is the exact layout published on ``/microphone_levels``.  The
    audio streamer publishes one processed mono channel, so this function does
    not invent values for the four raw microphone channels.
    """

    values = tuple(float(sample) for sample in samples)
    if not values:
        return 0.0, 0.0

    rms = math.sqrt(sum(sample * sample for sample in values) / len(values))
    peak = max(abs(sample) for sample in values)
    return min(rms / INT16_SCALE, 1.0), min(peak / INT16_SCALE, 1.0)
