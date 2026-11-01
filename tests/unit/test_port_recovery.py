"""Unit tests for the port recovery policy (PR-2154, PR-2157)."""

from __future__ import annotations

import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
if str(PIB_MOTORS_DIR) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_DIR))

from pib_motors.port_recovery import (  # noqa: E402
    PortRecovery,
    RecoveryAction,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _policy(
    clock: FakeClock | None = None,
    *,
    usb_reset_enabled_fn=None,
) -> tuple[PortRecovery, FakeClock]:
    fake = clock or FakeClock()
    kwargs = {"clock_fn": fake}
    if usb_reset_enabled_fn is not None:
        kwargs["usb_reset_enabled_fn"] = usb_reset_enabled_fn
    return PortRecovery(**kwargs), fake


def test_a_single_failure_never_reopens() -> None:
    policy, _clock = _policy()
    assert policy.note_failure("/dev/a") is RecoveryAction.NONE


def test_the_threshold_triggers_exactly_one_reopen() -> None:
    policy, _clock = _policy()
    results = [policy.note_failure("/dev/a") for _ in range(4)]
    assert results == [
        RecoveryAction.NONE,
        RecoveryAction.NONE,
        RecoveryAction.REOPEN,
        RecoveryAction.NONE,
    ], results


def test_the_reopen_is_rate_limited_per_port() -> None:
    policy, clock = _policy()
    assert [policy.note_failure("/dev/a") for _ in range(3)] == [
        RecoveryAction.NONE,
        RecoveryAction.NONE,
        RecoveryAction.REOPEN,
    ]
    # the counter was reset by the reopen, and the interval forbids another one right away
    assert [policy.note_failure("/dev/a") for _ in range(6)] == [
        RecoveryAction.NONE
    ] * 6
    # after the interval the next failure over the threshold reopens again
    clock.advance(11.0)
    assert policy.note_failure("/dev/a") is RecoveryAction.REOPEN


def test_four_reopen_decisions_when_every_sweep_fails() -> None:
    """Persistent faults must reach the policy on every cycle, not just the first dropout."""
    policy, clock = _policy()
    reopens = 0
    for cycle in range(4):
        if cycle:
            clock.advance(11.0)
        assert policy.note_failure("/dev/a") is RecoveryAction.NONE
        assert policy.note_failure("/dev/a") is RecoveryAction.NONE
        action = policy.note_failure("/dev/a")
        assert action is RecoveryAction.REOPEN, action
        reopens += 1
    assert reopens == 4


def test_ports_are_counted_independently() -> None:
    policy, _clock = _policy()
    policy.note_failure("/dev/a")
    policy.note_failure("/dev/a")
    policy.note_failure("/dev/b")
    assert policy.note_failure("/dev/a") is RecoveryAction.REOPEN
    assert policy.note_failure("/dev/b") is RecoveryAction.NONE


def test_a_success_resets_everything() -> None:
    policy, _clock = _policy()
    policy.note_failure("/dev/a")
    policy.note_failure("/dev/a")
    policy.note_success("/dev/a")
    assert policy.note_failure("/dev/a") is RecoveryAction.NONE
    assert policy.snapshot("/dev/a")["failures"] == 1


def test_it_gives_up_instead_of_looping_forever(caplog) -> None:
    policy, clock = _policy()
    reopens = 0
    with caplog.at_level(logging.ERROR):
        for _ in range(40):
            clock.advance(11.0)
            for _ in range(3):
                if policy.note_failure("/dev/a") is RecoveryAction.REOPEN:
                    reopens += 1
    assert reopens == policy.max_reopens_without_success, reopens
    assert "PORT_GAVE_UP" in caplog.text
    assert "stage=GAVE_UP" in caplog.text
    assert policy.snapshot("/dev/a")["gave_up"] is True


def test_the_dropout_is_logged_as_error(caplog) -> None:
    """The operator asked for this explicitly: a dropout has to be findable later."""
    policy, _clock = _policy()
    with caplog.at_level(logging.ERROR):
        policy.note_failure("/dev/a")
        policy.note_failure("/dev/a")
        policy.note_failure("/dev/a")
    errors = [record for record in caplog.records if record.levelno == logging.ERROR]
    assert errors, "no ERROR was logged"
    assert "PORT_DROPOUT" in caplog.text
    assert "stage=DROPOUT" in caplog.text
    assert "/dev/a" in caplog.text
    assert "consecutive_failures=3" in caplog.text


def test_a_healthy_port_logs_nothing(caplog) -> None:
    """Counter-check: without this the ERROR above could fire on every sweep."""
    policy, _clock = _policy()
    with caplog.at_level(logging.ERROR):
        for _ in range(10):
            policy.note_success("/dev/a")
    assert caplog.text == ""


def test_usb_reset_runs_after_a_reopen_when_enabled() -> None:
    always_usb = lambda reopens: reopens >= 1  # noqa: E731
    policy, clock = _policy(usb_reset_enabled_fn=always_usb)
    assert policy.note_failure("/dev/a") is RecoveryAction.NONE
    assert policy.note_failure("/dev/a") is RecoveryAction.NONE
    assert policy.note_failure("/dev/a") is RecoveryAction.REOPEN
    clock.advance(11.0)
    assert policy.note_failure("/dev/a") is RecoveryAction.NONE
    assert policy.note_failure("/dev/a") is RecoveryAction.NONE
    assert policy.note_failure("/dev/a") is RecoveryAction.USB_RESET


def test_usb_reset_stays_off_when_the_guard_is_false() -> None:
    never_usb = lambda reopens: False  # noqa: E731
    policy, clock = _policy(usb_reset_enabled_fn=never_usb)
    for cycle in range(3):
        if cycle:
            clock.advance(11.0)
        for _ in range(2):
            policy.note_failure("/dev/a")
        assert policy.note_failure("/dev/a") is RecoveryAction.REOPEN
    assert policy.snapshot("/dev/a")["usb_reset_attempted"] is False


def test_a_port_that_keeps_failing_is_retried_until_the_budget_is_spent() -> None:
    """The retry loop must really retry, not attempt once and fall silent.

    Measured on the robot in PR-2154: one PORT_REOPEN, then a silent log with ports_ok stuck at
    3/4, because an empty sweep counted as a success and reset the counter every tick.
    """
    from pib_motors.port_recovery import RecoveryAction

    fake = FakeClock()
    policy = PortRecovery(clock_fn=fake, usb_reset_enabled_fn=lambda reopens: False)

    actions = []
    for _ in range(policy.max_reopens_without_success):
        for _ in range(3):
            actions.append(policy.note_failure("/dev/a"))
        fake.advance(policy.reopen_interval_sec + 1.0)

    reopens = [a for a in actions if a is RecoveryAction.REOPEN]
    assert len(reopens) == policy.max_reopens_without_success, reopens


def test_the_give_up_comes_after_the_last_attempt_and_stops_the_loop() -> None:
    from pib_motors.port_recovery import RecoveryAction

    fake = FakeClock()
    policy = PortRecovery(clock_fn=fake, usb_reset_enabled_fn=lambda reopens: False)
    for _ in range(policy.max_reopens_without_success):
        for _ in range(3):
            policy.note_failure("/dev/a")
        fake.advance(policy.reopen_interval_sec + 1.0)

    later = []
    for _ in range(9):
        later.append(policy.note_failure("/dev/a"))
        fake.advance(policy.reopen_interval_sec + 1.0)
    assert set(later) == {RecoveryAction.NONE}, later
    assert policy.snapshot("/dev/a")["gave_up"] is True
