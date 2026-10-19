"""Fast hardware VAD sampling and motion-time threshold guard (PC-102)."""

from __future__ import annotations

import logging
from contextlib import contextmanager
from typing import Callable, Iterator, List, Optional

logger = logging.getLogger(__name__)

DEFAULT_SAMPLE_INTERVAL_S = 0.064
MOTION_VAD_THRESHOLD_DB = 8.5
GAMMAVAD_SR = "GAMMAVAD_SR"


def percentile(values: List[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((pct / 100.0) * (len(ordered) - 1)))
    return ordered[index]


class FastVadSampler:
    """Poll ``is_voice`` at most every ``interval_s`` (default 64 ms)."""

    def __init__(
        self,
        reader: Callable[[], bool],
        *,
        interval_s: float = DEFAULT_SAMPLE_INTERVAL_S,
        clock: Optional[Callable[[], float]] = None,
    ) -> None:
        if interval_s <= 0:
            raise ValueError("interval_s must be positive")
        self.interval_s = float(interval_s)
        self._reader = reader
        self._clock = clock or (lambda: 0.0)
        self._last_sample_at: Optional[float] = None
        self.sample_count = 0
        self._gaps_s: List[float] = []

    @property
    def p95_gap_s(self) -> float:
        return percentile(self._gaps_s, 95.0)

    def poll(self, now: Optional[float] = None) -> Optional[bool]:
        """Sample when due; return voice activity or ``None`` if not yet time."""
        now = now if now is not None else self._clock()
        if self._last_sample_at is not None:
            elapsed = now - self._last_sample_at
            if elapsed < self.interval_s:
                return None
            self._gaps_s.append(elapsed)
        self._last_sample_at = now
        self.sample_count += 1
        return bool(self._reader())


class MotionVadThresholdGuard:
    """Raise ``GAMMAVAD_SR`` during motion and restore the prior value in ``finally``."""

    def __init__(
        self,
        read: Callable[[str], float],
        write: Callable[[str, float], None],
        *,
        motion_db: float = MOTION_VAD_THRESHOLD_DB,
        param: str = GAMMAVAD_SR,
        log: Optional[logging.Logger] = None,
    ) -> None:
        self._read = read
        self._write = write
        self._motion_db = float(motion_db)
        self._param = param
        self._log = log or logger
        self._saved_db: Optional[float] = None
        self._depth = 0

    @property
    def active(self) -> bool:
        return self._depth > 0

    def begin_motion(self) -> None:
        if self._depth == 0:
            try:
                self._saved_db = float(self._read(self._param))
            except Exception:
                # No readable device: leave the firmware value alone rather than
                # guessing one, and do not let the failure reach the audio path.
                self._saved_db = None
                self._log.warning(
                    "%s unreadable; motion threshold left unchanged", self._param
                )
                self._depth += 1
                return
            try:
                self._write(self._param, self._motion_db)
                self._log.info(
                    "%s raised for motion: %.2f dB -> %.2f dB",
                    self._param,
                    self._saved_db,
                    self._motion_db,
                )
            except Exception:
                self._log.exception(
                    "%s write to %.2f dB failed; will still restore %.2f dB",
                    self._param,
                    self._motion_db,
                    self._saved_db,
                )
        self._depth += 1

    def end_motion(self) -> None:
        if self._depth == 0:
            return
        self._depth -= 1
        if self._depth > 0:
            return
        saved = self._saved_db
        self._saved_db = None
        if saved is None:
            return
        try:
            self._write(self._param, saved)
            self._log.info("%s restored after motion: %.2f dB", self._param, saved)
        except Exception:
            self._log.exception("%s restore to %.2f dB failed", self._param, saved)

    @contextmanager
    def motion_scope(self) -> Iterator[None]:
        """Raise the motion threshold for the block and restore in ``finally``."""
        self.begin_motion()
        try:
            yield
        finally:
            self.end_motion()
