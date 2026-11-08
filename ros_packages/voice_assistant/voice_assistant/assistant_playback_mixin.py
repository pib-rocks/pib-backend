"""Playback and program execution helpers for VoiceAssistantNode (PR-2260)."""

from __future__ import annotations

import threading
from typing import Callable, Optional

from datatypes.action import RunProgram
from datatypes.srv import PlayAudioFromFile, PlayAudioFromSpeech
from pib_api_client import voice_assistant_client
from pib_hermes_config.turn_taking import pause_threshold_now
from rclpy.task import Future
from voice_assistant.turn_latency import now_ms


class AssistantPlaybackMixin:
    def play_audio_from_file(
        self, filepath: str, on_stopped_playing: Callable[[], None] = None
    ) -> None:
        detector = getattr(self, "_barge_in_detector", None)
        if detector is not None:
            detector.mute_for_earcon(now_ms())
        request = PlayAudioFromFile.Request()
        request.filepath = filepath
        request.join = on_stopped_playing is not None
        future: Future = self.play_audio_from_file_client.call_async(request)
        if request.join:
            future.add_done_callback(lambda _: on_stopped_playing())

    def _consider_presence_greeting(self) -> None:
        """Greet a person who is looking, unless a turn is already underway."""
        if not self.state.turned_on or self.personality is None:
            return
        if self.gemini_loop.is_listening:
            return
        if (
            self.waiting_for_transcribed_text
            or self._turn_has_playback
            or self.is_executing_program
        ):
            return
        line = self._attention.take_opening_greeting(
            language=getattr(self.personality, "language", None)
        )
        if not line:
            return
        self.play_audio_from_speech(
            line,
            self.personality.gender,
            self.personality.language,
        )

    def _announce_live_action(self, text: str) -> None:
        """Speak a robot action on the turn-based player before it runs.

        The call waits until playback finishes so the action cannot start first.
        """
        done = threading.Event()
        gender = getattr(self.personality, "gender", None) or "Female"
        language = getattr(self.personality, "language", None) or "German"
        self.play_audio_from_speech(text, gender, language, done.set)
        if not done.wait(timeout=30.0):
            self.get_logger().warning(
                f"Live action announcement did not finish within 30s: {text}"
            )

    def play_audio_from_speech(
        self,
        speech: str,
        gender: str,
        language: str,
        on_stopped_playing: Callable[[], None] = None,
    ) -> None:
        if speech:
            self._note_spoken_answer(speech)
        request = PlayAudioFromSpeech.Request()
        request.speech = speech
        request.gender = gender
        request.language = language
        request.join = on_stopped_playing is not None
        request.tts_engine = getattr(self.personality, "tts_engine", None) or ""
        request.voice_id = getattr(self.personality, "voice_id", None) or ""
        request.turn_id = getattr(self, "_speech_turn_id", "") or ""
        if speech and str(speech).strip():
            self._note_spoken_answer(str(speech))
        future: Future = self.play_audio_from_speech_client.call_async(request)
        if request.join:
            future.add_done_callback(lambda _: on_stopped_playing())

    def _refresh_personality_turn_taking(self) -> None:
        """Read pause threshold, filler and voice again so an edit applies while running.

        The voice belongs here for the same reason as the threshold: the dialog writes it and
        the operator expects the next sentence to use it, not the one after a restart.
        """
        personality = self.personality
        if personality is None:
            return
        personality_id = getattr(personality, "personality_id", None)
        if not personality_id:
            return
        try:
            ok, fresh = voice_assistant_client.get_personality(personality_id)
        except Exception as exc:
            self.get_logger().warning(f"personality refresh failed: {exc}")
            return
        if not ok or fresh is None:
            return
        personality.pause_threshold = pause_threshold_now(
            personality.pause_threshold,
            getattr(fresh, "pause_threshold", None),
        )
        personality.thinking_filler = getattr(fresh, "thinking_filler", None)
        fresh_gender = getattr(fresh, "gender", None)
        if isinstance(fresh_gender, str) and fresh_gender.strip():
            personality.gender = fresh_gender.strip().title()

    def run_program(
        self, code_visual: str, on_stopped_executing_program: Callable[[None], None]
    ):

        goal = RunProgram.Goal()
        goal.source_type = RunProgram.Goal.SOURCE_CODE_VISUAL
        goal.source = code_visual
        result_callback = lambda _: on_stopped_executing_program()
        future: Future = self.run_program_client.send_goal_async(goal)
        self.stop_program_execution = self.digest_goal_handle_future(
            future, result_callback
        )

    def run_program_by_number(self, program_number: Optional[str]) -> None:
        """Fire-and-forget a saved Blockly program by its programNumber (PR-1153)."""
        cleaned = (
            program_number.strip()
            if isinstance(program_number, str) and program_number.strip()
            else ""
        )
        if not cleaned:
            return
        client = getattr(self, "run_program_client", None)
        if client is None:
            return
        goal = RunProgram.Goal()
        goal.source_type = getattr(RunProgram.Goal, "SOURCE_PROGRAM_NUMBER", 0)
        goal.source = cleaned
        client.send_goal_async(goal)
