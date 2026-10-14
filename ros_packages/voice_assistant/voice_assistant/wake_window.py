"""Session window that a wake word opens and silence closes.

The wake-word engine itself is a separate ticket. This module only keeps
the window: open on wake, held open by speech, closed after idle time.
"""

from __future__ import annotations

from typing import Protocol

WAKE_IDLE_SECONDS = 30.0


class WakeWindow:
    """The half of the wake-word path that decides whether the session is open.

    ``tick`` returns True only on the open-to-closed transition, and not
    again until the window is opened once more.
    """

    def __init__(self, *, idle_seconds: float = WAKE_IDLE_SECONDS) -> None:
        self._idle_seconds = idle_seconds
        self._awake = False
        self._last_activity: float | None = None

    @property
    def awake(self) -> bool:
        return self._awake

    def on_wake(self, now: float) -> None:
        """Open the window and start the idle clock."""
        self._awake = True
        self._last_activity = now

    def on_speech(self, now: float) -> None:
        """Hold an open window open. Speech does not open a closed window."""
        if not self._awake:
            return
        self._last_activity = now

    def tick(self, now: float) -> bool:
        """True only on the transition from open to closed."""
        if not self._awake or self._last_activity is None:
            return False
        if now - self._last_activity < self._idle_seconds:
            return False
        self._awake = False
        self._last_activity = None
        return True

    def reset(self) -> None:
        """Close the window without reporting an idle transition."""
        self._awake = False
        self._last_activity = None


class WakeDetector(Protocol):
    """A wake-word engine. Detection itself is out of scope for this branch."""

    def detect(self, frame: bytes) -> bool:
        """True when this frame contains the wake word."""
