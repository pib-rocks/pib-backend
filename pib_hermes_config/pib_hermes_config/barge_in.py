"""Deterministic hardware AEC/VAD barge-in detector (PR-2188, Decision D1=1A, D2=2A).

Evaluates the Seeed ReSpeaker 4-Mic Array (XVF3000) hardware signals:
- ``/voice_activity`` (``VOICEACTIVITY`` register, polled at 20 Hz)
- ``/speech_detected`` (``SPEECHDETECTED`` register, polled at 10 Hz)

Trigger contract (Decision D1=1A):
- When the robot is actively speaking or a chat turn is streaming/pending playback
  (and not muted by an earcon cooldown), sustained ``voice_activity == True`` for
  at least ``confirm_ms`` (default 200 ms) **with** ``speech_detected == True``
  observed inside the confirmation window triggers a barge-in.
- If ``speech_detected`` (10 Hz) has not arrived yet, sustained ``voice_activity``
  for at least ``fallback_confirm_ms`` (default 250 ms) triggers as a fallback.
- A brief transient click or cough (< 200 ms) resets as soon as ``voice_activity``
  drops back to ``False``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

DEFAULT_CONFIRM_MS: float = 200.0
DEFAULT_FALLBACK_CONFIRM_MS: float = 250.0
DEFAULT_EARCON_MUTE_MS: float = 350.0
DEFAULT_COOLDOWN_MS: float = 400.0
SPEECH_DETECTED_RECENT_WINDOW_MS: float = 250.0


@dataclass(frozen=True)
class BargeInDecision:
    """Result of one VAD / speech_detected observation."""

    triggered: bool
    sustained_ms: float
    speech_detected: bool
    reason: str = ""


class BargeInDetector:
    """Pure state machine tracking hardware VAD & speech detection during robot speech."""

    def __init__(
        self,
        *,
        confirm_ms: float = DEFAULT_CONFIRM_MS,
        fallback_confirm_ms: float = DEFAULT_FALLBACK_CONFIRM_MS,
        earcon_mute_ms: float = DEFAULT_EARCON_MUTE_MS,
        cooldown_ms: float = DEFAULT_COOLDOWN_MS,
    ) -> None:
        self.confirm_ms = max(50.0, float(confirm_ms))
        self.fallback_confirm_ms = max(self.confirm_ms, float(fallback_confirm_ms))
        self.earcon_mute_ms = max(0.0, float(earcon_mute_ms))
        self.cooldown_ms = max(0.0, float(cooldown_ms))

        self._vad_onset_ms: Optional[float] = None
        self._last_vad_true_ms: Optional[float] = None
        self._last_speech_detected_true_ms: Optional[float] = None
        self._speech_detected_in_streak: bool = False
        self._muted_until_ms: float = 0.0
        self._cooldown_until_ms: float = 0.0
        self._fired_in_current_streak: bool = False

    def reset(self) -> None:
        """Clear any active VAD streak (e.g. when a turn finishes or barge-in fires)."""
        self._vad_onset_ms = None
        self._last_vad_true_ms = None
        self._speech_detected_in_streak = False
        self._fired_in_current_streak = False

    def mute_for_earcon(
        self, now_ms: float, duration_ms: Optional[float] = None
    ) -> None:
        """Suppress barge-in while START_SIGNAL_FILE / STOP_SIGNAL_FILE is playing."""
        span = (
            self.earcon_mute_ms if duration_ms is None else max(0.0, float(duration_ms))
        )
        self._muted_until_ms = max(self._muted_until_ms, float(now_ms) + span)
        self.reset()

    def observe_speech_detected(self, detected: bool, *, now_ms: float) -> None:
        """Record the latest 10 Hz /speech_detected sample from the XVF3000."""
        if detected:
            self._last_speech_detected_true_ms = float(now_ms)
            if self._vad_onset_ms is not None:
                self._speech_detected_in_streak = True

    def observe_voice_activity(
        self,
        active: bool,
        *,
        now_ms: float,
        can_interrupt: bool,
    ) -> BargeInDecision:
        """Evaluate a 20 Hz /voice_activity sample and return whether barge-in fires."""
        ts = float(now_ms)
        if (
            not can_interrupt
            or ts < self._muted_until_ms
            or ts < self._cooldown_until_ms
        ):
            if not active:
                self.reset()
            return BargeInDecision(
                triggered=False,
                sustained_ms=0.0,
                speech_detected=False,
                reason="inactive_or_muted",
            )

        if not active:
            self.reset()
            return BargeInDecision(
                triggered=False,
                sustained_ms=0.0,
                speech_detected=False,
                reason="silence",
            )

        if self._vad_onset_ms is None:
            self._vad_onset_ms = ts
            self._last_vad_true_ms = ts
            recent_sd = (
                self._last_speech_detected_true_ms is not None
                and (ts - self._last_speech_detected_true_ms)
                <= SPEECH_DETECTED_RECENT_WINDOW_MS
            )
            self._speech_detected_in_streak = bool(recent_sd)
            # First tick opens the confirmation window (sustained_ms = 50 ms for 1st 20Hz sample)
            sustained_ms = 50.0
        else:
            self._last_vad_true_ms = ts
            sustained_ms = max(50.0, (ts - self._vad_onset_ms) + 50.0)

        if self._fired_in_current_streak:
            return BargeInDecision(
                triggered=False,
                sustained_ms=sustained_ms,
                speech_detected=self._speech_detected_in_streak,
                reason="already_fired",
            )

        has_sd = self._speech_detected_in_streak or (
            self._last_speech_detected_true_ms is not None
            and (ts - self._last_speech_detected_true_ms)
            <= SPEECH_DETECTED_RECENT_WINDOW_MS
        )

        if has_sd and sustained_ms >= self.confirm_ms:
            self._fired_in_current_streak = True
            self._cooldown_until_ms = ts + self.cooldown_ms
            return BargeInDecision(
                triggered=True,
                sustained_ms=sustained_ms,
                speech_detected=True,
                reason="vad_and_speech_detected",
            )

        if sustained_ms >= self.fallback_confirm_ms:
            self._fired_in_current_streak = True
            self._cooldown_until_ms = ts + self.cooldown_ms
            return BargeInDecision(
                triggered=True,
                sustained_ms=sustained_ms,
                speech_detected=has_sd,
                reason="vad_fallback_window",
            )

        return BargeInDecision(
            triggered=False,
            sustained_ms=sustained_ms,
            speech_detected=has_sd,
            reason="confirming",
        )
