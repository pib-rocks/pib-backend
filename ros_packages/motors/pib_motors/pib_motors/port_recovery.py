"""Decide when a serial port that stopped answering should be closed and reopened.

A hung PortHandler stays "open" as far as the object is concerned, so the cache in
``feetech_actuator._get_or_open_port`` hands the same dead handle out forever and no amount of
retrying helps. Measured on the pib5advanced: three of four adapters stop answering during
operation, /joint_states shrinks from 26 names to 8, and only restarting the container brings
them back - for a few minutes.

This module holds the policy only: how many failures in a row justify a reopen, how often a
reopen may be attempted, when to escalate to a USB re-enumeration, and when to give up. It is
pure Python with no ROS and no serial access, so the thresholds can be tested.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from dataclasses import dataclass, field
from enum import Enum

from pib_motors.port_usb_reset import should_attempt_usb_reset

_LOG = logging.getLogger(__name__)

#: Consecutive failed transactions on one port before the handle is thrown away.
FAILURE_THRESHOLD = 3
#: Minimum time between two recovery attempts on the same port.
REOPEN_INTERVAL_SEC = 10.0
#: Reopen attempts without a single success before the port enters slow-retry mode.
MAX_REOPENS_WITHOUT_SUCCESS = 5
#: Slow retry interval after giving up so a revived adapter is noticed without restarting.
GAVE_UP_RETRY_INTERVAL_SEC = 600.0

_BY_ID_SERIAL_RE = re.compile(r"usb-1a86_USB_Single_Serial_([^-/\s]+)-if\d+")


def normalize_port_key(device: str) -> str:
    """Map both a raw adapter serial ('5B91030562') and its /dev/serial/by-id path to the
    same canonical key so probe and sweep paths share one counter per physical adapter.
    """
    cleaned = str(device or "").strip()
    match = _BY_ID_SERIAL_RE.search(cleaned)
    if match:
        return match.group(1)
    return cleaned


class RecoveryAction(str, Enum):
    NONE = "none"
    REOPEN = "reopen"
    USB_RESET = "usb_reset"


@dataclass
class _PortState:
    failures: int = 0
    reopens_without_success: int = 0
    last_reopen_at: float = 0.0
    gave_up: bool = False
    usb_reset_attempted: bool = False


@dataclass
class PortRecovery:
    """Failure bookkeeping per port, with the reopen decision."""

    failure_threshold: int = FAILURE_THRESHOLD
    reopen_interval_sec: float = REOPEN_INTERVAL_SEC
    max_reopens_without_success: int = MAX_REOPENS_WITHOUT_SUCCESS
    gave_up_retry_interval_sec: float = GAVE_UP_RETRY_INTERVAL_SEC
    clock_fn: object = time.monotonic
    usb_reset_enabled_fn: object = should_attempt_usb_reset
    states: dict[str, _PortState] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def _state(self, device: str) -> _PortState:
        return self.states.setdefault(normalize_port_key(device), _PortState())

    def note_success(self, device: str) -> None:
        """A port that answers is healthy: clear everything."""
        with self.lock:
            state = self._state(device)
            state.failures = 0
            state.reopens_without_success = 0
            state.gave_up = False
            state.usb_reset_attempted = False

    def note_failure(self, device: str) -> RecoveryAction:
        """Record a failure and return the recovery stage to run, if any."""
        with self.lock:
            state = self._state(device)
            now = self.clock_fn()  # type: ignore[operator]
            if state.gave_up:
                state.failures += 1
                if (
                    self.gave_up_retry_interval_sec > 0
                    and now - state.last_reopen_at >= self.gave_up_retry_interval_sec
                    and state.failures >= self.failure_threshold
                ):
                    consecutive = state.failures
                    reopen_num = state.reopens_without_success + 1
                    _LOG.error(
                        "[PERF_TRACE] PORT_DROPOUT device=%s stage=SLOW_RETRY "
                        "consecutive_failures=%d reopen=%d",
                        device,
                        consecutive,
                        reopen_num,
                    )
                    state.failures = 0
                    state.last_reopen_at = now
                    state.reopens_without_success += 1
                    return RecoveryAction.REOPEN
                return RecoveryAction.NONE
            state.failures += 1
            if state.failures < self.failure_threshold:
                return RecoveryAction.NONE
            if now - state.last_reopen_at < self.reopen_interval_sec:
                _LOG.debug(
                    "[PERF_TRACE] PORT_RETRY_HELD device=%s failures=%d wait_ms=%.0f",
                    device,
                    state.failures,
                    (self.reopen_interval_sec - (now - state.last_reopen_at)) * 1000.0,
                )
                return RecoveryAction.NONE
            if state.reopens_without_success >= self.max_reopens_without_success:
                if not state.gave_up:
                    state.gave_up = True
                    state.last_reopen_at = now
                    _LOG.error(
                        "[PERF_TRACE] PORT_GAVE_UP device=%s stage=GAVE_UP reopens=%d "
                        "consecutive_failures=%d - the adapter does not answer even after "
                        "reopening; switching to slow retry",
                        device,
                        state.reopens_without_success,
                        state.failures,
                    )
                return RecoveryAction.NONE

            consecutive = state.failures
            reopen_num = state.reopens_without_success + 1
            _LOG.error(
                "[PERF_TRACE] PORT_DROPOUT device=%s stage=DROPOUT consecutive_failures=%d "
                "reopen=%d",
                device,
                consecutive,
                reopen_num,
            )
            state.failures = 0
            state.last_reopen_at = now

            if (
                not state.usb_reset_attempted
                and self.usb_reset_enabled_fn(  # type: ignore[operator]
                    state.reopens_without_success
                )
            ):
                state.usb_reset_attempted = True
                return RecoveryAction.USB_RESET

            state.reopens_without_success += 1
            return RecoveryAction.REOPEN

    def snapshot(self, device: str) -> dict[str, object]:
        with self.lock:
            state = self._state(device)
            return {
                "failures": state.failures,
                "reopens_without_success": state.reopens_without_success,
                "gave_up": state.gave_up,
                "usb_reset_attempted": state.usb_reset_attempted,
            }

    def reset(self) -> None:
        """Tests only."""
        with self.lock:
            self.states.clear()


_POLICY = PortRecovery()


def note_success(device: str) -> None:
    _POLICY.note_success(device)


def note_failure(device: str) -> RecoveryAction:
    return _POLICY.note_failure(device)


def policy() -> PortRecovery:
    return _POLICY


def reset_for_tests(clock_fn=None, usb_reset_enabled_fn=None) -> None:
    global _POLICY
    _POLICY = PortRecovery(
        clock_fn=clock_fn or time.monotonic,
        usb_reset_enabled_fn=usb_reset_enabled_fn or should_attempt_usb_reset,
    )
