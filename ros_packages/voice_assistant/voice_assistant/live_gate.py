"""Local voice-activity gate for the live audio uplink.

The microphone array decides when a person is speaking. Room silence stays
on the robot: only the pre-roll and the open turn are admitted to the
provider, with an explicit start and an explicit end.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

CHUNK_MS = 32
SAMPLES_PER_CHUNK = 512  # 512 samples at 16 kHz = 32 ms
PRE_ROLL_CHUNKS = 10  # 320 ms of audio kept before the trigger
SILENCE_END_CHUNKS = 14  # about 450 ms of silence ends the turn
VAD_POLL_INTERVAL_S = 0.03  # a register read was measured at about 29 ms


@dataclass(frozen=True)
class GateDecision:
    """What the uplink should do with one observation."""

    start: bool = False
    end: bool = False
    uplink: tuple[bytes, ...] = ()


class PreRollBuffer:
    """Capacity-bounded FIFO that keeps the newest chunks.

    ``drain`` returns them oldest first and leaves the buffer empty.
    """

    def __init__(self, capacity: int = PRE_ROLL_CHUNKS) -> None:
        self._chunks: deque[bytes] = deque(maxlen=capacity)

    def push(self, chunk: bytes) -> None:
        self._chunks.append(chunk)

    def drain(self) -> tuple[bytes, ...]:
        items = tuple(self._chunks)
        self._chunks.clear()
        return items

    def __len__(self) -> int:
        return len(self._chunks)


class ActivityGate:
    """Turn the hardware voice-activity bit into start, audio, and end.

    ``observe`` applies the rules in this order:

    1. A provided chunk is pushed into the pre-roll ring first.
    2. Not streaming and voice present: start, and uplink the ring. The
       ring already ends with this chunk, so the pre-roll and the trigger
       are contiguous and the trigger is not sent twice.
    3. Not streaming and no voice: uplink nothing. Room silence outside
       the window never leaves the robot.
    4. Streaming and voice present: reset the silence counter and uplink
       the new chunk only.
    5. Streaming and no voice: count the silence. The chunk is still
       uplinked so the provider hears the tail. The turn ends when the
       counter reaches ``silence_end_chunks``.
    """

    def __init__(
        self,
        *,
        pre_roll_chunks: int = PRE_ROLL_CHUNKS,
        silence_end_chunks: int = SILENCE_END_CHUNKS,
    ) -> None:
        self._pre_roll = PreRollBuffer(pre_roll_chunks)
        self._silence_end_chunks = silence_end_chunks
        self._streaming = False
        self._silent = 0

    @property
    def streaming(self) -> bool:
        return self._streaming

    def observe(self, voice_activity: bool, chunk: bytes | None = None) -> GateDecision:
        if chunk is not None:
            self._pre_roll.push(chunk)

        if not self._streaming:
            if voice_activity:
                self._streaming = True
                self._silent = 0
                return GateDecision(start=True, uplink=self._pre_roll.drain())
            return GateDecision()

        if voice_activity:
            self._silent = 0
            uplink = (chunk,) if chunk is not None else ()
            return GateDecision(uplink=uplink)

        self._silent += 1
        ended = self._silent >= self._silence_end_chunks
        if ended:
            self._streaming = False
            self._silent = 0
        uplink = (chunk,) if chunk is not None else ()
        return GateDecision(end=ended, uplink=uplink)

    def reset(self) -> None:
        """Drop the open turn and the audio kept before the next one."""
        self._streaming = False
        self._silent = 0
        self._pre_roll.drain()
