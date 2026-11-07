"""Voice turn lifecycle callbacks for VoiceAssistantNode (PR-2120)."""

from __future__ import annotations

import threading
from typing import Any, Callable, Optional

from datatypes.msg import ChatIsListening
from pib_hermes_config.live_session import (
    VOICE_MODE_LIVE,
    channel_turn_on_allowed,
    live_session_ends_on_handover,
)
from pib_hermes_config.turn_taking import transcript_opens_turn
from pib_hermes_config.visible_state import answer_uses_fallback
from public_api_client.hermes_agent_client import FALLBACK_REPLY
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from std_msgs.msg import Bool, String
from voice_assistant.degraded_chat import allows_cloud_chat, fetch_operating_mode
from voice_assistant.stt_transcription import visible_failure_hint


def _assistant():
    import voice_assistant.assistant as mod

    return mod


class AssistantTurnCallbacksMixin:
    def on_start_signal_played(self, *, from_barge_in: bool = False) -> None:
        if self.gemini_loop.is_listening:
            return

        if self._answer_audio_pending:
            # PC-40: never open a recording window on top of answer audio.
            # PC-49: do not poll with audible start tones; a bounded watchdog
            # closes a lost drain instead of hanging or stacking turns.
            self.get_logger().info(
                "deferring recording until answer audio drains (pending playback)"
            )
            self._arm_pending_drain_watchdog()
            return

        self._refresh_personality_turn_taking()
        silent_before = 0.0 if from_barge_in else _assistant().MAX_SILENT_SECONDS_BEFORE
        self.record_audio(
            silent_before,
            self.personality.pause_threshold,
            self.if_cycle_not_changed(self.on_stopped_recording, require_voice_on=True),
            self.if_cycle_not_changed(
                self.on_transcribed_text_received, require_voice_on=True
            ),
        )

        self._answer_is_fallback = False
        self._stt_failure_cause = None
        self.state.speaking = False
        self.state.using_fallback = self._engine_fallback
        self.set_is_listening(self.state.chat_id, True)

    def _on_stop_signal_played(self) -> None:
        # signal all four sub-loops to exit
        self.get_logger().debug("_on_stop_signal_played")

    def on_stopped_recording(self) -> None:
        if self.gemini_loop.is_listening:
            return

        if not self.get_is_listening(self.state.chat_id):
            return

        self.play_audio_from_file(
            _assistant().STOP_SIGNAL_FILE,
        )
        self.set_is_listening(self.state.chat_id, False)
        self.waiting_for_transcribed_text = True

    def on_transcribed_text_received(self, transcribed_text: str) -> None:
        if not self.waiting_for_transcribed_text:
            return
        self.waiting_for_transcribed_text = False
        if self._stt_failure_cause:
            # The recording was aborted because the engine failed; the hint
            # naming the cause is already being spoken. An empty transcript
            # here is not a user who said nothing.
            return
        if not transcript_opens_turn(transcribed_text):
            # Nobody said anything in that window. Answering it makes the robot
            # talk to itself: the model replies to an empty message with a
            # filler sentence, that answer is spoken for 6.5-55 s, and the next
            # window follows - measured on the test robot as seven
            # self-triggered turns with empty transcripts in four minutes
            # (PC-40). So: no chat turn, no speech, just listen again.
            if bool(getattr(self.personality, "use_wake_word", False)):
                self.get_logger().info(
                    "nothing transcribed in active window; returning to wake-word standby"
                )
                if getattr(self, "_dialog_active", False):
                    self._dialog_active = False
                    runner = getattr(self, "run_program_by_number", None)
                    if callable(runner):
                        runner(getattr(self.personality, "go_sleep_program", None))
                self._enter_wake_word_standby(self.state.chat_id)
                return
            self.get_logger().info(
                "nothing transcribed in this window; no chat turn, listening again"
            )
            self.play_audio_from_file(
                _assistant().START_SIGNAL_FILE,
                self.if_cycle_not_changed(
                    self.on_start_signal_played, require_voice_on=True
                ),
            )
            return
        self._start_spoken_turn()
        self._turn_episode_task = transcribed_text.strip()
        self._mark_text_received()
        self.chat(
            transcribed_text,
            self.state.chat_id,
            True,
            self.if_cycle_not_changed(self.on_sentence_received, require_voice_on=True),
            self.if_cycle_not_changed(
                self.on_code_visual_received, require_voice_on=True
            ),
        )

    def on_sentence_received(self, sentence: str, is_final: bool) -> None:
        if not self.state.turned_on or self.turning_off:
            return
        if is_final:
            self.final_chat_response_received = True
        if self.gemini_loop.is_listening:
            return

        self._note_first_token()
        if not sentence:
            # Empty tail after streamed clauses: finalize playback, not voice OFF.
            if is_final and not self.is_executing_program:
                self._finish_spoken_turn()
            return
        clauses = self._streaming_speech.take(sentence, is_final)
        self.final_chat_response_received = is_final
        if clauses:
            # This is the handoff of the turn's first answer clause to the
            # synthesiser. Marked here rather than in play_audio_from_speech so
            # a thinking filler never becomes the answer's first sentence.
            if not getattr(self, "_turn_episode_outcome", ""):
                self._turn_episode_outcome = clauses[0].strip()
            self._mark_first_sentence_flushed()
        if not clauses:
            if is_final and not self.is_executing_program:
                self._finish_spoken_turn()
            return
        gender = self.personality.gender
        language = self.personality.language
        for index, clause in enumerate(clauses):
            last = is_final and index == len(clauses) - 1
            on_stopped_playing = (
                self.if_cycle_not_changed(
                    self.on_final_sentence_played, require_voice_on=True
                )
                if last and not self.is_executing_program
                else None
            )
            self._turn_has_playback = True
            self._answer_audio_pending = True
            if on_stopped_playing is not None:
                self._arm_pending_drain_watchdog()
            self.play_audio_from_speech(clause, gender, language, on_stopped_playing)

    def on_code_visual_received(self, code_visual: str, is_final: bool) -> None:
        if not self.state.turned_on or self.turning_off:
            return
        self.final_chat_response_received = is_final
        if self.is_executing_program:
            self.code_visual_queue.append(code_visual)
        else:
            self.run_program(
                code_visual,
                self.if_cycle_not_changed(self.on_stopped_executing_program),
            )
        self.is_executing_program = True

    def on_stopped_executing_program(self) -> None:
        if not self.state.turned_on or self.turning_off:
            return
        if self.code_visual_queue:
            code_visual = self.code_visual_queue.pop()
            self.run_program(
                code_visual,
                self.if_cycle_not_changed(self.on_stopped_executing_program),
            )
        else:
            self.is_executing_program = False
            if self.final_chat_response_received:
                self._finish_spoken_turn()

    def on_final_sentence_played(self) -> None:
        # The player has drained this turn's audio: a new recording window may
        # open again (PC-40).
        self._cancel_pending_drain_watchdog()
        self._answer_audio_pending = False
        # The answer is over; a later greeting or live announcement must not
        # inherit this id (and with it a cancellation).
        self._speech_turn_id = ""
        self.state.speaking = False
        detector = getattr(self, "_barge_in_detector", None)
        if detector is not None:
            detector.reset()
        self._playback_epoch += 1
        self._close_latency_turn(incomplete=False)
        if not self.state.turned_on or self.turning_off:
            return
        self.play_audio_from_file(
            _assistant().START_SIGNAL_FILE,
            self.if_cycle_not_changed(
                self.on_start_signal_played, require_voice_on=True
            ),
        )

    def if_cycle_not_changed(
        self, callback: Callable, *, require_voice_on: bool = False
    ) -> Callable:
        """Run callback only if cycle, playback epoch, and optional ON still match."""
        current_cycle = self.cycle
        playback_epoch = self._playback_epoch

        def decorated_callback(*args):
            if self.cycle != current_cycle:
                return
            if self._playback_epoch != playback_epoch:
                return
            if require_voice_on and not self.state.turned_on:
                return
            callback(*args)

        return decorated_callback

    def digest_goal_handle_future(
        self, goal_handle_future: Future, callback: Callable[[Any], None] = None
    ) -> Callable[[], None]:
        """adds a result callback to the goal and returns a function, that can be used to cancel the goal"""
        if callback is not None:

            def result_callback(result_future: Future):
                result = result_future.result().result
                callback(result)

            def done_callback(goal_handle_future: Future):
                goal_handle: ClientGoalHandle = goal_handle_future.result()
                result_future: Future = goal_handle.get_result_async()
                result_future.add_done_callback(result_callback)

            goal_handle_future.add_done_callback(done_callback)

        def cancel(future: Future) -> None:
            goal_handle: ClientGoalHandle = future.result()
            goal_handle.cancel_goal_async()

        return lambda: goal_handle_future.add_done_callback(cancel)

    def set_is_listening(self, chat_id: str, listening: bool) -> None:
        """updates and publishes the listening status of a chat"""
        self.chat_id_to_is_listening[chat_id] = listening
        chat_is_listening = ChatIsListening()
        chat_is_listening.listening = listening
        chat_is_listening.chat_id = chat_id

        self.chat_is_listening_publisher.publish(chat_is_listening)
        self._publish_voice_state()

    def _note_spoken_answer(self, speech: str) -> None:
        """Speaking starts. A fallback sentence is shown as fallback."""
        self._answer_is_fallback = answer_uses_fallback(
            speech, _assistant().FALLBACK_REPLY
        )
        self.state.speaking = True
        self.state.using_fallback = self._answer_is_fallback or self._engine_fallback
        self._publish_voice_state()

    def _on_live_speaking(self, speaking: bool) -> None:
        """Live playback. The session stays open, so listening is not cleared."""
        self.state.speaking = bool(speaking)
        if not speaking:
            self._answer_is_fallback = False
        self.state.using_fallback = self._answer_is_fallback or self._engine_fallback
        self._publish_voice_state()

    def _on_engine_fallback(self, msg: Bool) -> None:
        """STT or TTS left its primary engine. The face reads this from voice state."""
        self._engine_fallback = bool(msg.data)
        self.state.using_fallback = self._engine_fallback or self._answer_is_fallback
        self._publish_voice_state()

    def _on_stt_failure(self, msg: String) -> None:
        """The local engine could not transcribe. Name the cause out loud.

        The recorder aborts that goal, so nothing continues with an empty
        transcript; the technical cause is spoken instead of the canned
        "could not hear you" line, which would hide a broken engine behind a
        sentence about silence.
        """
        cause = (msg.data or "").strip()
        self._stt_failure_cause = cause
        self.waiting_for_transcribed_text = False
        self._engine_fallback = True
        self.state.using_fallback = True
        self._publish_voice_state()
        self.get_logger().error(f"STT engine failure: {cause}")
        personality = self.personality
        if personality is None:
            return
        self.play_audio_from_speech(
            visible_failure_hint(cause),
            getattr(personality, "gender", None) or "Female",
            getattr(personality, "language", None) or "German",
        )

    def _republish_visible_state(self) -> None:
        """Repeat the current holder so a display that joined late still sees it."""
        chat_id = self.state.chat_id
        if chat_id:
            listening = self.chat_id_to_is_listening.get(chat_id, False) or (
                self.gemini_loop.is_listening and chat_id == self.state.chat_id
            )
            message = ChatIsListening()
            message.listening = bool(listening)
            message.chat_id = chat_id
            self.chat_is_listening_publisher.publish(message)
        self._publish_voice_state()

    def get_is_listening(self, chat_id: str) -> bool:
        """find out, if a chat is currently listening for user input"""
        return self.chat_id_to_is_listening.get(chat_id, True)

    def stop_chat(self, chat_id: str) -> None:
        """if the chat of the provided chat-id is active, stop receiving messages from the chat"""
        stop_chat = self.chat_id_to_stop_chat.get(chat_id)
        if stop_chat is not None:
            stop_chat()

    def _can_barge_in_now(self) -> bool:
        """True when robot speech or an active spoken turn can be interrupted by the user."""
        if not getattr(self.state, "turned_on", False) or getattr(
            self, "turning_off", False
        ):
            return False
        if getattr(self.gemini_loop, "is_listening", False):
            return False
        chat_id = getattr(self.state, "chat_id", "") or ""
        if not chat_id:
            return False
        if self.chat_id_to_is_listening.get(chat_id, False):
            return False
        if getattr(self, "waiting_for_transcribed_text", False):
            return False
        return bool(
            getattr(self, "_answer_audio_pending", False)
            or getattr(self.state, "speaking", False)
            or getattr(self, "_turn_has_playback", False)
        )

    def _on_hardware_speech_detected(self, msg: Bool) -> None:
        """Receive 10 Hz /speech_detected from XVF3000 hardware AEC/ASR detector."""
        detector = getattr(self, "_barge_in_detector", None)
        if detector is None:
            return
        detector.observe_speech_detected(
            bool(getattr(msg, "data", False)), now_ms=_assistant().now_ms()
        )

    def _on_hardware_voice_activity(self, msg: Bool) -> None:
        """Receive 20 Hz /voice_activity from XVF3000 and trigger barge-in when confirmed."""
        detector = getattr(self, "_barge_in_detector", None)
        if detector is None:
            return
        decision = detector.observe_voice_activity(
            bool(getattr(msg, "data", False)),
            now_ms=_assistant().now_ms(),
            can_interrupt=self._can_barge_in_now(),
        )
        if not decision.triggered:
            return
        chat_id = getattr(self.state, "chat_id", "") or ""
        self.get_logger().info(
            f"[PERF_TRACE] BARGE_IN_TRIGGERED chat={chat_id} "
            f"vad_ms={decision.sustained_ms:.1f} "
            f"speech_detected={decision.speech_detected} reason={decision.reason}"
        )
        self.barge_in(chat_id, from_vad=True)

    def barge_in(
        self, chat_id: Optional[str] = None, *, from_vad: bool = False
    ) -> bool:
        """Interrupt active playback and stream cleanly so the next turn can start."""
        effective_chat_id = chat_id or self.state.chat_id
        if not effective_chat_id:
            return False
        # The interrupted answer keeps this id: clause requests it already
        # dispatched must never reach the speaker after the clear, otherwise
        # the old answer talks over the new one.
        from voice_assistant.util import speech_turn_id

        cancelled_turn_id = getattr(self, "_speech_turn_id", "") or speech_turn_id(
            effective_chat_id, self._playback_epoch
        )
        detector = getattr(self, "_barge_in_detector", None)
        if detector is not None:
            detector.reset()
        self.cycle += 1
        self._invalidate_voice_async_work()
        self._speech_turn_id = ""
        self._close_latency_turn(incomplete=True)
        self.stop_chat(effective_chat_id)
        self.final_chat_response_received = False
        self.state.speaking = False

        def on_cleared():
            if self.state.turned_on and effective_chat_id == self.state.chat_id:
                try:
                    self.on_start_signal_played(from_barge_in=from_vad)
                except TypeError:
                    self.on_start_signal_played()
            else:
                self.set_is_listening(effective_chat_id, True)

        self.clear_playback_queue(on_cleared, turn_id=cancelled_turn_id)
        self._publish_voice_state()
        return True

    def _sync_voice_state(self) -> None:
        """Fields both Cerebra windows read. The holder is backend state."""
        self.state.live_session = bool(self.gemini_loop.is_listening)
        if self.state.turned_on and self.personality is not None:
            self.state.personality_id = self.personality.personality_id or ""
            self.state.personality_name = getattr(self.personality, "name", "") or ""
        else:
            self.state.personality_id = ""
            self.state.personality_name = ""
            self.state.speaking = False
            self._answer_is_fallback = False
            self.state.using_fallback = self._engine_fallback

    def _publish_voice_state(self) -> None:
        self._sync_voice_state()
        self.voice_assistant_state_publisher.publish(self.state)

    def update_state(self, turned_on: bool, chat_id: str = "") -> bool:
        """Attempts to update the internal state, and returns whether this was successful."""
        import traceback

        # Use a stable chat id: when turning OFF, requests may pass "" → fall back to current state
        effective_chat_id = chat_id or self.state.chat_id

        # Debug
        stack = "".join(traceback.format_stack(limit=5))
        self.get_logger().debug(f"Call stack (most recent 5 frames):\n{stack}")
        self.get_logger().debug(
            f"update_state: requested chat_id={chat_id!r}, effective_chat_id={effective_chat_id!r}, "
            f"turned_on={turned_on}, gemini_loop.is_listening={self.gemini_loop.is_listening}"
        )

        # ---------- TURNING OFF ----------
        if not turned_on:
            was_dialog_active = bool(getattr(self, "_dialog_active", False))
            self._waiting_for_wake_word = False
            self._dialog_active = False
            if was_dialog_active and self.personality is not None:
                runner = getattr(self, "run_program_by_number", None)
                if callable(runner):
                    runner(getattr(self.personality, "go_sleep_program", None))
            # Releasing the channel ends a live session. A turn-based session
            # still uses the legacy cleanup below.
            if self.gemini_loop.is_listening:
                self.gemini_loop.stop()
                self._invalidate_voice_async_work()

                self.state.turned_on = False
                # keep the current chat id if none was provided
                self.state.chat_id = effective_chat_id
                self._publish_voice_state()
                return True

            # Legacy deactivation (unchanged)
            try:
                if self.turning_off:
                    raise Exception("voice assistant is currently turning off")
                if not self.state.turned_on:
                    # already off → no-op
                    self.state.turned_on = False
                    self.state.chat_id = effective_chat_id
                    self._publish_voice_state()
                    return True

                self.cycle += 1
                self._invalidate_voice_async_work()
                self.turning_off = True
                self.state.turned_on = False
                self.state.chat_id = effective_chat_id
                self.stop_recording()
                self.stop_chat(effective_chat_id)
                self.stop_program_execution()
                self.code_visual_queue.clear()
                self.final_chat_response_received = False
                self.is_executing_program = False

                current_chat_id = self.state.chat_id

                cleared_epoch = self._playback_epoch
                from voice_assistant.util import speech_turn_id

                interrupted_turn_id = getattr(self, "_speech_turn_id", "") or (
                    speech_turn_id(current_chat_id, cleared_epoch)
                )
                self._speech_turn_id = ""

                def on_playback_queue_cleared():
                    if self._playback_epoch != cleared_epoch:
                        return
                    self.turning_off = False
                    self.set_is_listening(current_chat_id, True)

                self.clear_playback_queue(
                    on_playback_queue_cleared, turn_id=interrupted_turn_id
                )

                self.state.turned_on = False
                self.state.chat_id = effective_chat_id
                self._publish_voice_state()
                return True

            except Exception as e:
                self.get_logger().error(
                    f"following error occured while trying to update state: {str(e)}."
                )
                return False

        # ---------- TURNING ON ----------
        # No mode may bypass an unacknowledged cancellation.
        if self.turning_off:
            self.get_logger().warning(
                "voice activation refused: playback clear not acknowledged"
            )
            return False
        # Turning ON requires a valid chat id
        if not effective_chat_id:
            self.get_logger().error("update_state: turning ON but no chat_id available")
            return False

        # Always resolve the target personality when turning ON (we may be switching chats/models)
        (
            is_success,
            pers,
        ) = _assistant().voice_assistant_client.get_personality_from_chat(
            effective_chat_id
        )
        if not is_success or pers is None:
            self.get_logger().error(f"no personality with chat id {effective_chat_id}")
            return False

        holder_id = (
            self.personality.personality_id if self.personality is not None else None
        )
        if not channel_turn_on_allowed(
            self.state.turned_on, holder_id, pers.personality_id
        ):
            self.get_logger().info(
                f"voice channel is held by personality {holder_id}; "
                f"refusing {pers.personality_id}"
            )
            return False

        # Moving the voice to another chat ends the open live session first.
        if live_session_ends_on_handover(
            self.state.turned_on,
            self.state.chat_id,
            effective_chat_id,
            self.gemini_loop.is_listening,
        ):
            self.gemini_loop.stop()

        self.personality = pers
        if bool(getattr(pers, "use_wake_word", False)):
            if (
                turned_on == self.state.turned_on
                and effective_chat_id == self.state.chat_id
                and (
                    getattr(self, "_waiting_for_wake_word", False)
                    or getattr(self, "_dialog_active", False)
                )
            ):
                return True
            self.stop_chat(effective_chat_id)
            self._invalidate_voice_async_work()
            runner = getattr(self, "run_program_by_number", None)
            if callable(runner):
                runner(getattr(pers, "go_sleep_program", None))
            self._enter_wake_word_standby(effective_chat_id)
            return True

        self._waiting_for_wake_word = False
        self._dialog_active = True
        runner = getattr(self, "run_program_by_number", None)
        if callable(runner):
            runner(getattr(pers, "wake_up_program", None))
        return self._start_active_dialog(effective_chat_id, pers, already_on_guard=True)

    def _enter_wake_word_standby(self, chat_id: str) -> None:
        """Wait locally for 'Hey, pib!' without opening a cloud session (PR-1153)."""
        self._waiting_for_wake_word = True
        self._dialog_active = False
        self.waiting_for_transcribed_text = False
        self.state.turned_on = True
        self.state.chat_id = chat_id
        self.state.speaking = False
        self.set_is_listening(chat_id, True)
        self._listen_for_wake_word()

    def _listen_for_wake_word(self) -> None:
        if (
            not getattr(self.state, "turned_on", False)
            or getattr(self, "turning_off", False)
            or not getattr(self, "_waiting_for_wake_word", False)
        ):
            return
        callback = self.if_cycle_not_changed(
            self._on_wake_word_transcript, require_voice_on=True
        )
        wake_stt_engine = (
            "cloud_whisper"
            if self._cloud_session_allowed()
            else (getattr(self.personality, "stt_engine", None) or "local_whisper")
        )
        try:
            self.record_audio(
                _assistant().MAX_SILENT_SECONDS_BEFORE,
                0.6,
                None,
                callback,
                stt_engine=wake_stt_engine,
            )
        except TypeError:
            self.record_audio(
                _assistant().MAX_SILENT_SECONDS_BEFORE,
                0.6,
                None,
                callback,
            )

    def _on_wake_word_transcript(self, transcribed_text: str) -> None:
        if (
            not getattr(self.state, "turned_on", False)
            or getattr(self, "turning_off", False)
            or not getattr(self, "_waiting_for_wake_word", False)
        ):
            return
        if transcribed_text and transcribed_text.strip():
            self.get_logger().info(f"wake-word standby heard: {transcribed_text!r}")
        from pib_hermes_config.wake_word import matches_wake_word

        if not matches_wake_word(transcribed_text):
            self._listen_for_wake_word()
            return
        self.get_logger().info(
            f"wake word detected ({transcribed_text!r}); starting active dialog"
        )
        self._waiting_for_wake_word = False
        self._dialog_active = True
        pers = self.personality
        runner = getattr(self, "run_program_by_number", None)
        if callable(runner) and pers is not None:
            runner(getattr(pers, "wake_up_program", None))
        self._start_active_dialog(self.state.chat_id, pers, already_on_guard=False)

    def _start_active_dialog(
        self, effective_chat_id: str, pers: Any, *, already_on_guard: bool = True
    ) -> bool:
        start_live = (
            getattr(pers, "voice_start_mode", None) == VOICE_MODE_LIVE
            and bool(getattr(pers, "live_model", None))
            and self._cloud_session_allowed()
        )
        self.get_logger().debug(
            "update_state: voice_start_mode="
            f"{getattr(pers, 'voice_start_mode', None)} "
            f"live_model={getattr(pers, 'live_model', None)}"
        )

        # Live path: the provider row's pinned model, gated by the live flag
        # (already folded into voice_start_mode). A locked key store falls
        # through to the local recorder instead.
        if start_live:
            if not self.gemini_loop.is_listening:
                self.gemini_loop.start(
                    chat_id=effective_chat_id,
                    model=pers.live_model,
                    idle_timeout_s=getattr(pers, "live_idle_timeout", None),
                )
                self.play_audio_from_file(_assistant().START_SIGNAL_FILE)
            else:
                self.get_logger().debug("Live session already running; no-op")

            self.state.turned_on = True
            self.state.chat_id = effective_chat_id
            self.state.speaking = False
            self.set_is_listening(effective_chat_id, True)
            return True

        # Legacy activation (unchanged)
        try:
            if self.turning_off:
                raise Exception("voice assistant is currently turning off")
            if (
                already_on_guard
                and True == self.state.turned_on
                and effective_chat_id == self.state.chat_id
            ):
                raise Exception("voice assistant is already turned on.")

            self.stop_chat(effective_chat_id)
            self.set_is_listening(effective_chat_id, False)
            self.state.turned_on = True
            self.state.chat_id = effective_chat_id
            self._invalidate_voice_async_work()
            self.play_audio_from_file(
                _assistant().START_SIGNAL_FILE,
                self.if_cycle_not_changed(
                    self.on_start_signal_played, require_voice_on=True
                ),
            )

        except Exception as e:
            self.get_logger().error(
                f"following error occured while trying to update state: {str(e)}."
            )
            return False

        self._publish_voice_state()
        return True

    def speak_filler_notice(self, text: str) -> None:
        if not text or not text.strip():
            return
        try:
            self.text_to_speech(text.strip(), self.get_TtsEngine(), lambda: None)
        except Exception as exc:
            self.get_logger().warning(f"speak_filler_notice failed: {exc}")
