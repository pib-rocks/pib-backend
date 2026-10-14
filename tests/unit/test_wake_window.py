"""The wake window opens on the wake word and closes after idle time."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from voice_assistant.wake_window import (  # noqa: E402
    WAKE_IDLE_SECONDS,
    WakeDetector,
    WakeWindow,
)


def test_default_idle_is_thirty_seconds():
    assert WAKE_IDLE_SECONDS == 30.0


def test_opens_on_wake_and_speech_does_not_open_a_closed_window():
    window = WakeWindow()
    assert window.awake is False
    window.on_speech(0.0)
    assert window.awake is False
    assert window.tick(WAKE_IDLE_SECONDS) is False
    window.on_wake(100.0)
    assert window.awake is True
    assert window.tick(100.0) is False
    assert window.awake is True


def test_stays_open_while_speech_arrives():
    window = WakeWindow(idle_seconds=30.0)
    window.on_wake(0.0)
    assert window.tick(29.0) is False
    assert window.awake is True
    window.on_speech(29.0)
    # 29 seconds after the speech the window is still open.
    assert window.tick(58.0) is False
    assert window.awake is True
    # It closes on the sample that lands exactly idle_seconds after speech.
    assert window.tick(59.0) is True
    assert window.awake is False


def test_closes_exactly_after_idle_seconds_without_speech():
    window = WakeWindow()
    window.on_wake(10.0)
    assert window.tick(10.0 + WAKE_IDLE_SECONDS - 1.0) is False
    assert window.awake is True
    assert window.tick(10.0 + WAKE_IDLE_SECONDS) is True
    assert window.awake is False


def test_tick_reports_the_transition_once_and_not_again():
    window = WakeWindow(idle_seconds=5.0)
    window.on_wake(0.0)
    assert window.tick(5.0) is True
    assert window.tick(5.0) is False
    assert window.tick(100.0) is False
    assert window.awake is False


def test_reset_closes_without_reporting_a_transition():
    window = WakeWindow(idle_seconds=5.0)
    window.on_wake(0.0)
    window.reset()
    assert window.awake is False
    assert window.tick(100.0) is False


def test_wake_detector_protocol_is_a_detect_callable():
    class AlwaysAsleep:
        def detect(self, frame: bytes) -> bool:
            return False

    detector: WakeDetector = AlwaysAsleep()
    assert detector.detect(b"\x00\x00") is False
