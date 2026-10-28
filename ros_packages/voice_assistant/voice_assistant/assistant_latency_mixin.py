"""Latency tracing and spoken-turn timing for VoiceAssistantNode (PR-2120)."""

from __future__ import annotations

import os
import threading
from typing import Callable, Optional

from pib_hermes_config.turn_taking import (
    authored_filler,
    first_token_budget_ms,
    subprocess_fallback_phrase,
)
from voice_assistant.turn_latency import (
    append_record,
    audio_event_belongs_to_turn,
    default_log_path,
    format_perf_trace_tts_first_audio,
    now_ms,
    parse_event,
    render_line,
    tts_first_audio_elapsed_ms,
)


def _assistant():
    import voice_assistant.assistant as mod

    return mod


def _turn_trace_max_seconds() -> float:
    """How long an open turn may stay open before the watchdog closes it."""
    raw = os.environ.get("PIB_TURN_TRACE_MAX_SECONDS", "120")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return 120.0
    if value <= 0:
        return 120.0
    return value


def _answer_drain_max_seconds() -> float:
    """How long queued answer audio may wait for a drain callback (PC-49)."""
    raw = os.environ.get("PIB_ANSWER_DRAIN_MAX_SECONDS", "90")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return 90.0
    if value <= 0:
        return 90.0
    return value


class AssistantLatencyMixin:
    def _start_spoken_turn(self) -> None:
        """Arm clause playback and, when the personality wrote one, a filler."""
        # Every answer owns its callbacks, including consecutive turns in one
        # ON-cycle. An old drain must never release the next answer.
        self._invalidate_voice_async_work()
        self._cancel_filler_timer()
        self._streaming_speech = _assistant().StreamingSpeech()
        self._turn_has_playback = False
        self._answer_audio_pending = False
        with self._filler_lock:
            self._first_token_seen = False
        self._subprocess_fallback_spoken = False
        self._refresh_personality_turn_taking()
        self._arm_filler()
        self._open_latency_turn()

    def _arm_filler(self) -> None:
        personality = self.personality
        if personality is None:
            return
        filler = getattr(personality, "thinking_filler", None)
        if not isinstance(filler, str) or not filler.strip():
            return
        budget_ms = first_token_budget_ms()

        def _fire() -> None:
            if not self.state.turned_on or self.turning_off:
                return
            with self._filler_lock:
                phrase = authored_filler(
                    filler,
                    elapsed_ms=float(budget_ms),
                    budget_ms=budget_ms,
                    first_token_seen=self._first_token_seen,
                )
                if phrase is None:
                    return
            if self.personality is None:
                return
            self._turn_has_playback = True
            self.play_audio_from_speech(
                phrase,
                personality.gender,
                personality.language,
            )

        owned_fire = self.if_cycle_not_changed(_fire, require_voice_on=True)
        timer = _assistant().threading.Timer(
            budget_ms / 1000.0, lambda: self._schedule_voice_callback(owned_fire)
        )
        timer.daemon = True
        self._filler_timer = timer
        timer.start()

    def _speak_subprocess_fallback_notice(self) -> None:
        """Speak once when Hermes must use the oneshot subprocess (PR-2105)."""
        if not self.state.turned_on or self.turning_off:
            return
        if self._subprocess_fallback_spoken:
            return
        personality = self.personality
        if personality is None:
            return
        phrase = subprocess_fallback_phrase(
            getattr(personality, "thinking_filler", None),
            getattr(personality, "language", None),
        )
        if not phrase:
            return
        self._subprocess_fallback_spoken = True
        self._cancel_filler_timer()
        gender = personality.gender
        language = personality.language

        def _speak() -> None:
            if not self.state.turned_on or self.turning_off:
                return
            if self.personality is None:
                return
            self._turn_has_playback = True
            self.play_audio_from_speech(phrase, gender, language)

        self._schedule_voice_callback(
            self.if_cycle_not_changed(_speak, require_voice_on=True)
        )

    def _note_first_token(self) -> None:
        with self._filler_lock:
            self._first_token_seen = True
        self._cancel_filler_timer()

    def _cancel_filler_timer(self) -> None:
        timer = self._filler_timer
        self._filler_timer = None
        if timer is not None:
            timer.cancel()

    def _finish_spoken_turn(self) -> None:
        """Restart listening after audio that was queued earlier in the turn."""
        if not self.state.turned_on or self.turning_off:
            return
        if self.personality is None:
            return
        if self._turn_has_playback:
            self._answer_audio_pending = True
            self._arm_pending_drain_watchdog()
            self.play_audio_from_speech(
                "",
                self.personality.gender,
                self.personality.language,
                self.if_cycle_not_changed(
                    self.on_final_sentence_played, require_voice_on=True
                ),
            )
            return
        self.on_final_sentence_played()

    def _trace_source(self) -> str:
        """Model id when the personality has one, otherwise a generic label."""
        model = getattr(self.personality, "assistant_model", None)
        api_name = getattr(model, "api_name", None)
        if isinstance(api_name, str) and api_name.strip():
            return api_name.strip()
        return "voice"

    def _open_latency_turn(self) -> None:
        """Open the trace. An end_of_speech seen while idle becomes the origin."""
        try:
            pending = self._pending_end_of_speech
            self._pending_end_of_speech = None
            origin_ms = None
            if isinstance(pending, dict):
                try:
                    stamp = float(pending["ts_ms"])
                except (KeyError, TypeError, ValueError):
                    pending = None
                    self._orphan_dropped += 1
                else:
                    age_ms = now_ms() - stamp
                    limit_ms = self._turn_trace_max_seconds * 1000.0
                    if age_ms < 0 or age_ms > limit_ms:
                        pending = None
                        self._orphan_dropped += 1
                    else:
                        origin_ms = stamp
            if self._turn_trace.is_open:
                self._close_latency_turn(incomplete=True)
            self._sentence_flushed = False
            self._tts_first_audio_logged = False
            self._turn_trace.open(
                self.state.chat_id or "",
                self._trace_source(),
                origin_ms=origin_ms,
            )
            if self._orphan_dropped:
                self._turn_trace.dropped_events += self._orphan_dropped
                self._orphan_dropped = 0
            if pending is not None and origin_ms is not None:
                self._turn_trace.observe(pending)
        except Exception as exc:
            self.get_logger().warning(f"turn trace open failed: {exc}")

    def _mark_text_received(self) -> None:
        try:
            self._turn_trace.mark_local("text_received")
        except Exception as exc:
            self.get_logger().warning(f"turn trace text_received failed: {exc}")

    def _mark_first_sentence_flushed(self) -> None:
        """The turn's first answer clause handed to the synthesiser."""
        try:
            if not self._turn_trace.is_open or self._sentence_flushed:
                return
            self._sentence_flushed = True
            self._turn_trace.mark_local("first_sentence_flushed")
        except Exception as exc:
            self.get_logger().warning(
                f"turn trace first_sentence_flushed failed: {exc}"
            )

    def _close_latency_turn(self, *, incomplete: bool) -> None:
        """Log the one line and append the JSON record. A closed turn is a no-op."""
        try:
            trace = self._turn_trace
            if not trace.is_open:
                return
            if not incomplete:
                # The audio player stamps last_audio when the burst's queue
                # drains. Its event may still be in flight for a few
                # milliseconds when the turn closes, so the completion of the
                # last playback call - the same boundary, observed here - fills
                # the gap. Never invented: it only applies to a turn whose
                # first audio was already reported.
                if (
                    trace.elapsed("last_audio") is None
                    and trace.elapsed("first_audio") is not None
                ):
                    trace.mark_local("last_audio")
                trace.mark_local("turn_end")
            record = trace.close(incomplete=incomplete)
            if record is None:
                return
            self.get_logger().info(render_line(record))
            append_record(default_log_path(), record)
        except Exception as exc:
            self.get_logger().warning(f"turn trace close failed: {exc}")

    def _on_turn_trace(self, msg: String) -> None:
        """Subscription callback. Parse and store; never block and never raise."""
        try:
            event = parse_event(getattr(msg, "data", "") or "")
            if event is None:
                if self._turn_trace.is_open:
                    self._turn_trace.dropped_events += 1
                else:
                    self._orphan_dropped += 1
                return
            if self._turn_trace.is_open:
                stage = str(event.get("stage") or "")
                if stage == "hermes_subprocess_fallback":
                    self._speak_subprocess_fallback_notice()
                    return
                if not audio_event_belongs_to_turn(
                    str(event.get("stage") or ""), self._sentence_flushed
                ):
                    # A signal file or a filler is not the answer's audio.
                    self._turn_trace.dropped_events += 1
                    return
                self._turn_trace.observe(event)
                if event.get("stage") == "first_audio" and not getattr(
                    self, "_tts_first_audio_logged", False
                ):
                    elapsed = tts_first_audio_elapsed_ms(self._turn_trace)
                    if elapsed is not None:
                        self._tts_first_audio_logged = True
                        self.get_logger().info(
                            format_perf_trace_tts_first_audio(
                                self._turn_trace.chat_id, elapsed
                            )
                        )
                return
            if event.get("stage") == "end_of_speech":
                self._pending_end_of_speech = event
                return
            self._orphan_dropped += 1
        except Exception as exc:
            self.get_logger().warning(f"turn trace event dropped: {exc}")

    def _turn_trace_watchdog(self) -> None:
        """Close a turn that outlived PIB_TURN_TRACE_MAX_SECONDS so it cannot leak."""
        try:
            trace = self._turn_trace
            if not trace.is_open:
                return
            limit_ms = self._turn_trace_max_seconds * 1000.0
            if now_ms() - trace.origin_ms < limit_ms:
                return
            self._close_latency_turn(incomplete=True)
        except Exception as exc:
            self.get_logger().warning(f"turn trace watchdog failed: {exc}")

    def _schedule_voice_callback(self, callback: Callable) -> None:
        """Timers admit work on the single ROS executor, never across teardown."""
        executor = self.executor
        if executor is not None:
            executor.create_task(callback)

    def _invalidate_voice_async_work(self) -> None:
        """Drop stale playback/recording callbacks and release pending flags."""
        self._playback_epoch += 1
        self._answer_audio_pending = False
        self._turn_has_playback = False
        self._streaming_speech = _assistant().StreamingSpeech()
        self._cancel_filler_timer()
        self._cancel_pending_drain_watchdog()
        self.waiting_for_transcribed_text = False

    def _cancel_pending_drain_watchdog(self) -> None:
        timer = self._pending_drain_watchdog
        self._pending_drain_watchdog = None
        if timer is not None:
            timer.cancel()

    def _arm_pending_drain_watchdog(self) -> None:
        if not self._answer_audio_pending:
            return
        if self._pending_drain_watchdog is not None:
            return
        armed_epoch = self._playback_epoch
        timeout_s = self._answer_drain_max_seconds

        def _on_timeout() -> None:
            if not self._answer_audio_pending:
                return
            if self._playback_epoch != armed_epoch:
                return
            self._pending_drain_watchdog = None
            self.get_logger().warning(
                "answer audio drain timed out; forcing playback boundary (PC-49)"
            )
            # No drain acknowledgement: fail OFF and request cancellation.
            # Never fabricate completion or open another recording window.
            self.update_state(False)

        owned_timeout = self.if_cycle_not_changed(_on_timeout, require_voice_on=True)
        timer = _assistant().threading.Timer(
            timeout_s, lambda: self._schedule_voice_callback(owned_timeout)
        )
        timer.daemon = True
        self._pending_drain_watchdog = timer
        timer.start()
