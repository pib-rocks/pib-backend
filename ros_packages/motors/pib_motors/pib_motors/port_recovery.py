"""Decide when a serial port that stopped answering should be closed and reopened.

A hung PortHandler stays "open" as far as the object is concerned, so the cache in
``feetech_actuator._get_or_open_port`` hands the same dead handle out forever and no amount of
retrying helps. Measured on the pib5advanced: three of four adapters stop answering during
operation, /joint_states shrinks from 26 names to 8, and only restarting the container brings
them back - for a few minutes.

This module holds the policy only: how many failures in a row justify a reopen, how often a
reopen may be attempted, and when to give up and say so. It is pure Python with no ROS and no
serial access, so the thresholds can be tested.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

_LOG = logging.getLogger(__name__)

#: Consecutive failed transactions on one port before the handle is thrown away.
FAILURE_THRESHOLD = 3
#: Minimum time between two reopen attempts on the same port.
REOPEN_INTERVAL_SEC = 10.0
#: Attempts without a single success before the port is declared gone.
MAX_REOPENS_WITHOUT_SUCCESS = 5


@dataclass
class _PortState:
    failures: int = 0
    reopens_without_success: int = 0
    last_reopen_at: float = 0.0
    gave_up: bool = False


@dataclass
class PortRecovery:
    """Failure bookkeeping per port, with the reopen decision."""

    failure_threshold: int = FAILURE_THRESHOLD
    reopen_interval_sec: float = REOPEN_INTERVAL_SEC
    max_reopens_without_success: int = MAX_REOPENS_WITHOUT_SUCCESS
    clock_fn: object = time.monotonic
    states: dict[str, _PortState] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def _state(self, device: str) -> _PortState:
        return self.states.setdefault(device, _PortState())

    def note_success(self, device: str) -> None:
        """A port that answers is healthy: clear everything."""
        with self.lock:
            state = self._state(device)
            state.failures = 0
            state.reopens_without_success = 0
            state.gave_up = False

    def note_failure(self, device: str) -> bool:
        """Record a failure. True means: close and reopen this port now."""
        with self.lock:
            state = self._state(device)
            state.failures += 1
            if state.failures < self.failure_threshold:
                return False
            now = self.clock_fn()  # type: ignore[operator]
            if now - state.last_reopen_at < self.reopen_interval_sec:
                return False
            if state.reopens_without_success >= self.max_reopens_without_success:
                if not state.gave_up:
                    state.gave_up = True
                    _LOG.error(
                        "[PERF_TRACE] PORT_GAVE_UP device=%s reopens=%d - the adapter does not "
                        "answer even after reopening; its joints stay out of the sweep",
                        device,
                        state.reopens_without_success,
                    )
                return False
            state.last_reopen_at = now
            # ERROR on purpose: an adapter that stops answering is a fault worth finding in the
            # log later, not a warning that scrolls away between the sweep messages.
            _LOG.error(
                "[PERF_TRACE] PORT_DROPOUT device=%s consecutive_failures=%d reopen=%d - "
                "closing the handle and reopening",
                device,
                state.failures,
                state.reopens_without_success + 1,
            )
            state.failures = 0
            state.reopens_without_success += 1
            return True

    def snapshot(self, device: str) -> dict[str, object]:
        with self.lock:
            state = self._state(device)
            return {
                "failures": state.failures,
                "reopens_without_success": state.reopens_without_success,
                "gave_up": state.gave_up,
            }

    def reset(self) -> None:
        """Tests only."""
        with self.lock:
            self.states.clear()


_POLICY = PortRecovery()


def note_success(device: str) -> None:
    _POLICY.note_success(device)


def note_failure(device: str) -> bool:
    return _POLICY.note_failure(device)


def policy() -> PortRecovery:
    return _POLICY


def reset_for_tests(clock_fn=None) -> None:
    global _POLICY
    _POLICY = PortRecovery(clock_fn=clock_fn or time.monotonic)
