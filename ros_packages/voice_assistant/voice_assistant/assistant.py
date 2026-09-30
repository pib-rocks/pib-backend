import threading
import time
from collections import deque
from typing import Any, Callable, Optional

import rclpy
from datatypes.action import Chat, RecordAudio, RunProgram
from datatypes.msg import VoiceAssistantState, ChatIsListening
from datatypes.srv import (
    SetVoiceAssistantState,
    GetVoiceAssistantState,
    ClearPlaybackQueue,
    PlayAudioFromFile,
    PlayAudioFromSpeech,
    GetChatIsListening,
    SendChatMessage,
)
from pib_api_client import voice_assistant_client
from pib_api_client.voice_assistant_client import Personality
from pib_hermes_config.live_session import (
    VOICE_MODE_LIVE,
    channel_turn_on_allowed,
    live_session_ends_on_handover,
)
from pib_hermes_config.turn_taking import (
    StreamingSpeech,
    authored_filler,
    first_token_budget_ms,
    pause_threshold_now,
)
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.task import Future
from voice_assistant import START_SIGNAL_FILE, STOP_SIGNAL_FILE
from voice_assistant.audio_loop import GeminiAudioLoop
from voice_assistant.degraded_chat import allows_cloud_chat, fetch_operating_mode

MAX_SILENT_SECONDS_BEFORE = 8.0


class VoiceAssistantNode(Node):

    def __init__(self):

        super().__init__("voice_assistant")

        self.gemini_loop = GeminiAudioLoop(api_key="")

        # state -------------------------------------------------------------------------

        # a counter for indicating the index of the current on-off-cycle
        self.cycle: int = 0
        # contains information on whether the va is turned on (and what the active chat is)
        self.state: VoiceAssistantState = VoiceAssistantState()
        # indicates if the va is turned on or off
        self.state.turned_on = False
        # id of the active chat (may be an arbitrary value, if the va is turned off)
        self.state.chat_id = ""
        self.state.live_session = False
        self.state.personality_id = ""
        # indicates if the voice_assistant is currently turning off
        self.turning_off = False
        # the personality associated with the active chat
        self.personality: Optional[Personality] = None
        # calling this function stops audio-recording
        self.stop_recording: Callable[[], None] = lambda: None
        # maps a chat-id to a function that can be used to stop receiving llm-responses
        self.chat_id_to_stop_chat: dict[str, Callable[[], None]] = {}
        # maps a chat-id to the listening status of the respective chat
        self.chat_id_to_is_listening: dict[str, bool] = {}
        # indicates, whether audio was recorded and va is currently awaitng the transcription
        self.waiting_for_transcribed_text = False
        # programs (in for of visual-code) received from the chat-server are buffered here until the current program finished executing
        self.code_visual_queue: deque[str] = deque()
        # indicates whether the final response (code or sentence) in the current request/response cycle of the active chat was already received
        self.final_chat_response_received = False
        # calling this function stops the current program
        self.stop_program_execution: Callable[[], None] = lambda: None
        # indicates if a program is currently executing
        self.is_executing_program = False
        # Clauses already handed to the synthesizer for the current answer.
        self._streaming_speech = StreamingSpeech()
        self._filler_timer: Optional[threading.Timer] = None
        self._filler_lock = threading.Lock()
        self._first_token_seen = False
        self._turn_has_playback = False

        # services ----------------------------------------------------------------------

        # Service for setting VoiceAssistantState
        self.set_voice_assistant_service: Service = self.create_service(
            SetVoiceAssistantState,
            "set_voice_assistant_state",
            self.set_voice_assistant_state,
        )

        # Service for getting current VoiceAssistantState
        self.get_voice_assistant_service: Service = self.create_service(
            GetVoiceAssistantState,
            "get_voice_assistant_state",
            self.get_voice_assistant_state,
        )

        # Service for getting the listening status of a chat
        self.get_chat_is_listening_service: Service = self.create_service(
            GetChatIsListening, "get_chat_is_listening", self.get_chat_is_listening
        )

        # Service that allows clients to send chat messages
        self.send_chat_message: Service = self.create_service(
            SendChatMessage, "send_chat_message", self.send_chat_message
        )

        # publishers --------------------------------------------------------------------

        # Publisher for VoiceAssistantState
        self.voice_assistant_state_publisher: Publisher = self.create_publisher(
            VoiceAssistantState, "voice_assistant_state", 10
        )

        # Publisher for ChatIsListening
        self.chat_is_listening_publisher: Publisher = self.create_publisher(
            ChatIsListening, "chat_is_listening", 10
        )

        # clients -----------------------------------------------------------------------

        self.chat_client: ActionClient = ActionClient(self, Chat, "chat")
        self.chat_client.wait_for_server()

        self.record_audio_client: ActionClient = ActionClient(
            self, RecordAudio, "record_audio"
        )
        self.record_audio_client.wait_for_server()

        self.play_audio_from_file_client: Client = self.create_client(
            PlayAudioFromFile, "play_audio_from_file"
        )
        self.play_audio_from_file_client.wait_for_service()

        self.play_audio_from_speech_client: Client = self.create_client(
            PlayAudioFromSpeech, "play_audio_from_speech"
        )
        self.play_audio_from_speech_client.wait_for_service()

        self.clear_playback_queue_client: Client = self.create_client(
            ClearPlaybackQueue, "clear_playback_queue"
        )
        self.clear_playback_queue_client.wait_for_service()

        self.run_program_client: ActionClient = ActionClient(
            self, RunProgram, "run_program"
        )
        self.run_program_client.wait_for_server()

        self.gemini_loop.set_action_announcer(self._announce_live_action)

        from voice_assistant.attention import GREETING_CHECK_SECONDS, bind_attention

        # Speech starting turns the head before a live answer. The timer greets
        # a recognised person who is looking, once per cooldown.
        self._attention = bind_attention(
            self,
            command_on_speech=True,
            speech_allowed=lambda: bool(self.state.turned_on),
        )
        self._presence_timer = self.create_timer(
            GREETING_CHECK_SECONDS, self._consider_presence_greeting
        )

        self.get_logger().info("Now running VA")

    # client accessors ------------------------------------------------------------------

    def clear_playback_queue(
        self, on_playback_queue_cleared: Callable[[], None] = None
    ):

        future = self.clear_playback_queue_client.call_async(
            ClearPlaybackQueue.Request()
        )
        future.add_done_callback(lambda _: on_playback_queue_cleared())

    def record_audio(
        self,
        max_silent_seconds_before: float,
        max_silent_seconds_after: float,
        on_stopped_recording: Callable[[], None] = None,
        on_transcribed_text_received: Callable[[str], None] = None,
    ) -> None:

        goal = RecordAudio.Goal()
        goal.max_silent_seconds_before = max_silent_seconds_before
        goal.max_silent_seconds_after = max_silent_seconds_after
        goal.stt_engine = (
            getattr(self.personality, "stt_engine", None) or "local_whisper"
        )
        feedback_callback = (
            None if on_stopped_recording is None else lambda _: on_stopped_recording()
        )
        result_callback = (
            None
            if on_transcribed_text_received is None
            else lambda res: on_transcribed_text_received(res.transcribed_text)
        )
        future: Future = self.record_audio_client.send_goal_async(
            goal, feedback_callback
        )
        self.stop_recording = self.digest_goal_handle_future(future, result_callback)

    def chat(
        self,
        text: str,
        chat_id: str,
        generate_code: bool,
        on_sentence_received: Callable[[str, bool], None] = lambda _1, _2: None,
        on_code_visual_received: Callable[[str, bool], None] = lambda _1, _2: None,
    ) -> None:

        goal = Chat.Goal()
        goal.text = text
        goal.chat_id = chat_id
        goal.generate_code = generate_code

        def feedback_callback(msg) -> None:
            feedback: Chat.Feedback = msg.feedback
            text = feedback.text
            if feedback.text_type == Chat.Goal.TEXT_TYPE_SENTENCE:
                on_sentence_received(text, False)
            elif feedback.text_type == Chat.Goal.TEXT_TYPE_CODE_VISUAL:
                on_code_visual_received(text, False)
            else:
                raise Exception(f"unsupported text-type: {feedback.text_type}")

        def result_callback(result: Chat.Result) -> None:
            text = result.text
            if result.text_type == Chat.Goal.TEXT_TYPE_SENTENCE:
                on_sentence_received(text, True)
            elif result.text_type == Chat.Goal.TEXT_TYPE_CODE_VISUAL:
                on_code_visual_received(text, True)
            else:
                raise Exception(f"unsupported text-type: {result.text_type}")

        future: Future = self.chat_client.send_goal_async(goal, feedback_callback)
        stop_chat = self.digest_goal_handle_future(future, result_callback)
        self.chat_id_to_stop_chat[chat_id] = stop_chat

    def play_audio_from_file(
        self, filepath: str, on_stopped_playing: Callable[[], None] = None
    ) -> None:
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
                "Live action announcement did not finish within 30s: %s", text
            )

    def play_audio_from_speech(
        self,
        speech: str,
        gender: str,
        language: str,
        on_stopped_playing: Callable[[], None] = None,
    ) -> None:
        request = PlayAudioFromSpeech.Request()
        request.speech = speech
        request.gender = gender
        request.language = language
        request.join = on_stopped_playing is not None
        request.tts_engine = (
            getattr(self.personality, "tts_engine", None) or "supertone"
        )
        future: Future = self.play_audio_from_speech_client.call_async(request)
        if request.join:
            future.add_done_callback(lambda _: on_stopped_playing())

    def _refresh_personality_turn_taking(self) -> None:
        """Read pause threshold and filler again so an edit applies while running."""
        personality = self.personality
        if personality is None:
            return
        personality_id = getattr(personality, "personality_id", None)
        if not personality_id:
            return
        try:
            ok, fresh = voice_assistant_client.get_personality(personality_id)
        except Exception as exc:
            self.get_logger().warning("personality refresh failed: %s", exc)
            return
        if not ok or fresh is None:
            return
        personality.pause_threshold = pause_threshold_now(
            personality.pause_threshold,
            getattr(fresh, "pause_threshold", None),
        )
        personality.thinking_filler = getattr(fresh, "thinking_filler", None)

    def _start_spoken_turn(self) -> None:
        """Arm clause playback and, when the personality wrote one, a filler."""
        self._cancel_filler_timer()
        self._streaming_speech = StreamingSpeech()
        self._turn_has_playback = False
        with self._filler_lock:
            self._first_token_seen = False
        self._refresh_personality_turn_taking()
        self._arm_filler()

    def _arm_filler(self) -> None:
        personality = self.personality
        if personality is None:
            return
        filler = getattr(personality, "thinking_filler", None)
        if not isinstance(filler, str) or not filler.strip():
            return
        budget_ms = first_token_budget_ms()

        def _fire() -> None:
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

        timer = threading.Timer(budget_ms / 1000.0, _fire)
        timer.daemon = True
        self._filler_timer = timer
        timer.start()

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
        if self.personality is None:
            return
        if self._turn_has_playback:
            self.play_audio_from_speech(
                "",
                self.personality.gender,
                self.personality.language,
                self.if_cycle_not_changed(self.on_final_sentence_played),
            )
            return
        self.on_final_sentence_played()

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

    # serivce callbacks -----------------------------------------------------------------

    def get_voice_assistant_state(
        self,
        _: GetVoiceAssistantState.Request,
        response: GetVoiceAssistantState.Response,
    ) -> GetVoiceAssistantState.Response:
        """callback function for 'get_voice_assistant_state' service"""
        self._sync_voice_state()
        response.voice_assistant_state = self.state
        return response

    def set_voice_assistant_state(
        self,
        request: SetVoiceAssistantState.Request,
        response: SetVoiceAssistantState.Response,
    ) -> SetVoiceAssistantState.Response:
        """callback function for 'set_voice_assistant_state' service"""
        request_state: VoiceAssistantState = request.voice_assistant_state
        successful = self.update_state(request_state.turned_on, request_state.chat_id)
        response.successful = successful
        return response

    def get_chat_is_listening(
        self, request: GetChatIsListening.Request, response: GetChatIsListening.Response
    ) -> GetChatIsListening.Response:
        """callback function for 'get_chat_is_listening' service"""
        live_for_this_chat = (
            self.gemini_loop.is_listening and request.chat_id == self.state.chat_id
        )
        response.listening = (
            self.get_is_listening(request.chat_id) or live_for_this_chat
        )
        return response

    def send_chat_message(
        self, request: SendChatMessage.Request, response: SendChatMessage.Response
    ) -> SendChatMessage.Response:
        """callback function for 'send_chat_message' service"""

        if self.gemini_loop.is_listening:
            return response

        # do not create a message, if chat is not listening
        if not self.get_is_listening(request.chat_id):
            return response

        # if chat is active, jump to next stage of the va-cycle
        elif request.chat_id == self.state.chat_id:
            self.set_is_listening(request.chat_id, False)
            self.play_audio_from_file(STOP_SIGNAL_FILE)
            self.stop_recording()
            self.set_is_listening(request.chat_id, False)
            self._start_spoken_turn()
            self.chat(
                request.content,
                self.state.chat_id,
                True,
                self.if_cycle_not_changed(self.on_sentence_received),
                self.if_cycle_not_changed(self.on_code_visual_received),
            )

        # if not active, create messages, without playing audio etc.
        else:
            self.set_is_listening(request.chat_id, False)

            def on_sentence_received(sentence: str, is_final: bool):
                if is_final:
                    self.set_is_listening(request.chat_id, True)

            self.chat(  # TODO : there is a race condition here, that could lead to the va falsely starting to listen, when activating this chat
                request.content, request.chat_id, False, on_sentence_received
            )

        response.successful = True
        return response

    # callback cycle --------------------------------------------------------------------

    def _cloud_session_allowed(self) -> bool:
        """Live and other cloud sessions need the unlocked key store."""
        return allows_cloud_chat(fetch_operating_mode())

    def on_start_signal_played(self) -> None:
        if self.gemini_loop.is_listening:
            return

        self._refresh_personality_turn_taking()
        self.record_audio(
            MAX_SILENT_SECONDS_BEFORE,
            self.personality.pause_threshold,
            self.if_cycle_not_changed(self.on_stopped_recording),
            self.if_cycle_not_changed(self.on_transcribed_text_received),
        )

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
            STOP_SIGNAL_FILE,
        )
        self.set_is_listening(self.state.chat_id, False)
        self.waiting_for_transcribed_text = True

    def on_transcribed_text_received(self, transcribed_text: str) -> None:
        if not self.waiting_for_transcribed_text:
            return
        self.waiting_for_transcribed_text = False
        self._start_spoken_turn()
        self.chat(
            transcribed_text,
            self.state.chat_id,
            True,
            self.if_cycle_not_changed(self.on_sentence_received),
            self.if_cycle_not_changed(self.on_code_visual_received),
        )

    def on_sentence_received(self, sentence: str, is_final: bool) -> None:
        if self.gemini_loop.is_listening:
            return

        self._note_first_token()
        if not sentence:
            self.update_state(False)
            return
        clauses = self._streaming_speech.take(sentence, is_final)
        self.final_chat_response_received = is_final
        if not clauses:
            if is_final and not self.is_executing_program:
                self._finish_spoken_turn()
            return
        gender = self.personality.gender
        language = self.personality.language
        for index, clause in enumerate(clauses):
            last = is_final and index == len(clauses) - 1
            on_stopped_playing = (
                self.if_cycle_not_changed(self.on_final_sentence_played)
                if last and not self.is_executing_program
                else None
            )
            self._turn_has_playback = True
            self.play_audio_from_speech(clause, gender, language, on_stopped_playing)

    def on_code_visual_received(self, code_visual: str, is_final: bool) -> None:
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
        if self.code_visual_queue:
            code_visual = self.code_visual_queue.pop()
            self.run_program(
                code_visual,
                self.if_cycle_not_changed(self.on_stopped_executing_program),
            )
        else:
            self.is_executing_program = False
            if self.final_chat_response_received:
                self.play_audio_from_file(
                    START_SIGNAL_FILE,
                    self.if_cycle_not_changed(self.on_start_signal_played),
                )

    def on_final_sentence_played(self) -> None:
        self.play_audio_from_file(
            START_SIGNAL_FILE, self.if_cycle_not_changed(self.on_start_signal_played)
        )

    # helper functions ------------------------------------------------------------------

    def if_cycle_not_changed(self, callback: Callable) -> Callable:
        """a decorator. the decorated callback only executes, if the cycle has not changed after its creation"""
        current_cycle = self.cycle

        def decorated_callback(*args):
            if self.cycle == current_cycle:
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

    def get_is_listening(self, chat_id: str) -> bool:
        """find out, if a chat is currently listening for user input"""
        return self.chat_id_to_is_listening.get(chat_id, True)

    def stop_chat(self, chat_id: str) -> None:
        """if the chat of the provided chat-id is active, stop receiving messages from the chat"""
        stop_chat = self.chat_id_to_stop_chat.get(chat_id)
        if stop_chat is not None:
            stop_chat()

    def _sync_voice_state(self) -> None:
        """Fields both Cerebra windows read. The holder is backend state."""
        self.state.live_session = bool(self.gemini_loop.is_listening)
        if self.state.turned_on and self.personality is not None:
            self.state.personality_id = self.personality.personality_id or ""
        else:
            self.state.personality_id = ""

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
            # Releasing the channel ends a live session. A turn-based session
            # still uses the legacy cleanup below.
            if self.gemini_loop.is_listening:
                self.gemini_loop.stop()
                self.play_audio_from_file(STOP_SIGNAL_FILE)

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
                self.turning_off = True
                self.waiting_for_transcribed_text = False
                self._cancel_filler_timer()
                self.stop_recording()
                self.stop_chat(effective_chat_id)
                self.stop_program_execution()
                self.code_visual_queue.clear()
                self.final_chat_response_received = False
                self.is_executing_program = False

                current_chat_id = self.state.chat_id

                def on_playback_queue_cleared():
                    self.turning_off = False
                    self.set_is_listening(current_chat_id, True)
                    self.play_audio_from_file(STOP_SIGNAL_FILE)

                self.clear_playback_queue(on_playback_queue_cleared)

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
        # Turning ON requires a valid chat id
        if not effective_chat_id:
            self.get_logger().error("update_state: turning ON but no chat_id available")
            return False

        # Always resolve the target personality when turning ON (we may be switching chats/models)
        is_success, pers = voice_assistant_client.get_personality_from_chat(
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
                "voice channel is held by personality %s; refusing %s",
                holder_id,
                pers.personality_id,
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
        start_live = (
            getattr(pers, "voice_start_mode", None) == VOICE_MODE_LIVE
            and bool(getattr(pers, "live_model", None))
            and self._cloud_session_allowed()
        )
        self.get_logger().debug(
            "update_state: voice_start_mode=%s live_model=%s",
            getattr(pers, "voice_start_mode", None),
            getattr(pers, "live_model", None),
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
                self.play_audio_from_file(START_SIGNAL_FILE)
            else:
                self.get_logger().debug("Live session already running; no-op")

            self.state.turned_on = True
            self.state.chat_id = effective_chat_id
            self._publish_voice_state()
            return True

        # Legacy activation (unchanged)
        try:
            if self.turning_off:
                raise Exception("voice assistant is currently turning off")
            if (
                turned_on == self.state.turned_on
                and effective_chat_id == self.state.chat_id
            ):
                raise Exception(
                    f"voice assistant is already turned {'on' if turned_on else 'off'}."
                )

            self.stop_chat(effective_chat_id)
            self.set_is_listening(effective_chat_id, False)
            self.play_audio_from_file(
                START_SIGNAL_FILE,
                self.if_cycle_not_changed(self.on_start_signal_played),
            )

            self.state.turned_on = True
            self.state.chat_id = effective_chat_id

        except Exception as e:
            self.get_logger().error(
                f"following error occured while trying to update state: {str(e)}."
            )
            return False

        self._publish_voice_state()
        return True


def main(args=None):
    rclpy.init()
    node = VoiceAssistantNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
