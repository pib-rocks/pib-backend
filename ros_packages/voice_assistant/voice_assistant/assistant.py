import os
import threading
import time
import traceback
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
    CancelChatTurn,
)
from voice_assistant.chat_turn import (
    STATE_CANCELLED,
    STATE_RUNNING,
    TextTurnRegistry,
    TurnBusy,
    turn_state_event,
)
from pib_api_client import voice_assistant_client
from pib_api_client.voice_assistant_client import Personality
from pib_hermes_config.live_interaction import typed_text_joins_live
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
    subprocess_fallback_phrase,
    transcript_opens_turn,
)
from pib_hermes_config.visible_state import (
    VOICE_STT_FAILURE_TOPIC,
    VOICE_USING_FALLBACK_TOPIC,
    answer_uses_fallback,
)
from public_api_client.hermes_agent_client import FALLBACK_REPLY
from std_msgs.msg import Bool, String
from voice_assistant.turn_latency import (
    format_perf_trace_tts_first_audio,
    tts_first_audio_elapsed_ms,
    TURN_TRACE_TOPIC,
    TurnTrace,
    append_record,
    audio_event_belongs_to_turn,
    default_log_path,
    now_ms,
    parse_event,
    render_line,
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
from voice_assistant.stt_transcription import visible_failure_hint

from voice_assistant.assistant_latency_mixin import (
    AssistantLatencyMixin,
    _answer_drain_max_seconds,
    _turn_trace_max_seconds,
)
from voice_assistant.assistant_turn_callbacks_mixin import AssistantTurnCallbacksMixin

MAX_SILENT_SECONDS_BEFORE = 8.0


_CLEAR_PLAYBACK_ACK_LOG_SECONDS = 5.0


class VoiceAssistantNode(AssistantTurnCallbacksMixin, AssistantLatencyMixin, Node):

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
        self.state.speaking = False
        self.state.using_fallback = False
        self.state.personality_name = ""
        self._answer_is_fallback = False
        self._engine_fallback = False
        # technical cause of a failed local transcription, None while healthy
        self._stt_failure_cause: Optional[str] = None
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
        self._subprocess_fallback_spoken = False
        self._turn_has_playback = False
        # True while this turn's answer audio is queued or playing. A recording
        # window must not open on top of it (PC-40).
        self._answer_audio_pending = False
        # Bumped on teardown/barge-in so stale drain callbacks cannot touch a new turn.
        self._playback_epoch: int = 0
        self._pending_drain_watchdog: Optional[threading.Timer] = None
        self._answer_drain_max_seconds = _answer_drain_max_seconds()
        self._turn_trace = TurnTrace()
        self._pending_end_of_speech: Optional[dict] = None
        self._orphan_dropped = 0
        self._sentence_flushed = False
        self._turn_trace_max_seconds = _turn_trace_max_seconds()
        self._text_turn_registry = TextTurnRegistry()

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
        self.cancel_chat_turn: Service = self.create_service(
            CancelChatTurn, "cancel_chat_turn", self.cancel_chat_turn
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
        self.create_subscription(
            Bool,
            VOICE_USING_FALLBACK_TOPIC,
            self._on_engine_fallback,
            10,
        )
        # A failed local transcription is spoken, never passed off as silence.
        self.create_subscription(
            String,
            VOICE_STT_FAILURE_TOPIC,
            self._on_stt_failure,
            10,
        )
        self._turn_trace_publisher = self.create_publisher(String, TURN_TRACE_TOPIC, 10)
        self.create_subscription(String, TURN_TRACE_TOPIC, self._on_turn_trace, 10)
        self.create_timer(1.0, self._turn_trace_watchdog)

        # clients -----------------------------------------------------------------------

        self.chat_client: ActionClient = ActionClient(self, Chat, "chat")
        self.chat_client.wait_for_server()

        self.record_audio_client: ActionClient = ActionClient(
            self, RecordAudio, "record_audio"
        )
        self.record_audio_client.wait_for_server()

        # Which live voice path speaks. "gemini" is the speech-to-speech session
        # that has always been here; "duplex" hands the answer and the synthesis to
        # the pib.Cloud relay and keeps transcription local.
        self.voice_backend = os.getenv("PIB_VOICE_BACKEND", "gemini").strip().lower()
        if self.voice_backend == "duplex":
            self.gemini_loop = self._build_duplex_loop()

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
        self.gemini_loop.set_speaking_listener(self._on_live_speaking)
        self.create_timer(1.0, self._republish_visible_state)

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
        self.get_logger().debug(
            "clear_playback_queue: submitting request (ACK is not proof the device stopped)"
        )
        ack_seen = threading.Event()
        future = self.clear_playback_queue_client.call_async(
            ClearPlaybackQueue.Request()
        )

        def on_service_done(_future: Future) -> None:
            try:
                if _future.cancelled() or _future.result() is None:
                    raise RuntimeError("clear request did not return a response")
            except Exception as exc:
                self.get_logger().warning(
                    f"clear_playback_queue: service failed ({type(exc).__name__}); no ACK"
                )
                return
            ack_seen.set()
            self.get_logger().debug("clear_playback_queue: service ACK received")
            if on_playback_queue_cleared is not None:
                on_playback_queue_cleared()

        future.add_done_callback(on_service_done)

        def log_missing_ack() -> None:
            if not ack_seen.is_set():
                self.get_logger().warning(
                    "clear_playback_queue: no service ACK within "
                    f"{_CLEAR_PLAYBACK_ACK_LOG_SECONDS:.0f}s"
                )

        timer = threading.Timer(_CLEAR_PLAYBACK_ACK_LOG_SECONDS, log_missing_ack)
        timer.daemon = True
        timer.start()

    def _build_duplex_loop(self):
        """Swap the Gemini live session for the relay, behind one env switch.

        The loop keeps the interface the rest of this node already calls
        (``is_listening``, ``start``, ``stop``, ``submit_typed_text``,
        ``set_speaking_listener``, ``set_action_announcer``), so nothing else in
        the node has to know which path is active.
        """
        from voice_assistant.ros_duplex import build_duplex_voice_loop

        url = os.getenv("PIB_CLOUD_VOICE_URL", "wss://api.pib.rocks/v1/pib/stream")
        try:
            loop = build_duplex_voice_loop(
                self,
                url=url,
                token=os.getenv("PIB_CLOUD_TOKEN") or None,
                request_transcription=self._request_duplex_transcription,
                mic_channels=int(os.getenv("PIB_MIC_CHANNELS", "1")),
            )
        except Exception as exc:
            # rclpy's logger rejects exc_info (TypeError: parameter "exc_info" is
            # not one of the recognized logging options), so the traceback text is
            # put into the message instead - otherwise the error path itself kills
            # the node and hides the audio failure (measured on .172, PR-2086).
            self.get_logger().error(
                "duplex voice backend unavailable; staying on the Gemini live path: "
                f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}"
            )
            return self.gemini_loop
        self.get_logger().info(f"voice backend: duplex relay at {url}")
        return loop

    def _request_duplex_transcription(self, on_text) -> None:
        """One utterance from the existing recorder, then hand the text to the relay.

        The pause threshold stays the personality's own, capped at 800 ms: 1.2 s of
        dead air after every sentence is what makes a conversation feel switched off
        rather than paused. The "before" window is how long pib waits for someone to
        start speaking at all, so it stays as generous as the button path.
        """
        from voice_assistant.duplex_loop import (
            DEFAULT_SILENCE_BEFORE_S,
            silence_after_s,
        )

        override = os.getenv("PIB_DUPLEX_SILENCE_AFTER", "").strip()
        self.record_audio(
            max_silent_seconds_before=float(
                os.getenv("PIB_DUPLEX_SILENCE_BEFORE", str(DEFAULT_SILENCE_BEFORE_S))
            ),
            max_silent_seconds_after=silence_after_s(
                getattr(self.personality, "pause_threshold", None),
                float(override) if override else None,
            ),
            on_transcribed_text_received=on_text,
        )

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
        turn_id: str = "",
    ) -> None:

        goal = Chat.Goal()
        goal.text = text
        goal.chat_id = chat_id
        goal.generate_code = generate_code
        goal.turn_id = turn_id

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
                f"Live action announcement did not finish within 30s: {text}"
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
        if speech and str(speech).strip():
            self._note_spoken_answer(str(speech))
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
            self.get_logger().warning(f"personality refresh failed: {exc}")
            return
        if not ok or fresh is None:
            return
        personality.pause_threshold = pause_threshold_now(
            personality.pause_threshold,
            getattr(fresh, "pause_threshold", None),
        )
        personality.thinking_filler = getattr(fresh, "thinking_filler", None)

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

    def _publish_text_turn_state(
        self, turn_id: str, state: str, chat_id: str, *, reason: str = ""
    ) -> None:
        if not turn_id:
            return
        try:
            message = String()
            message.data = turn_state_event(
                turn_id,
                state,
                chat_id=chat_id,
                reason=reason,
                source="voice_assistant",
            )
            self._turn_trace_publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"text turn state was not published: {exc}")

    def _stop_active_chat_goal(self, chat_id: str) -> None:
        stop = self.chat_id_to_stop_chat.get(chat_id)
        if stop is not None:
            stop()

    def cancel_chat_turn(
        self, request: CancelChatTurn.Request, response: CancelChatTurn.Response
    ) -> CancelChatTurn.Response:
        """Cancel one text turn started through ``send_chat_message``."""
        turn_id = (request.turn_id or "").strip()
        record = self._text_turn_registry.get(turn_id)
        chat_id = record.chat_id if record is not None else ""
        successful, reason = self._text_turn_registry.cancel(turn_id)
        response.successful = successful
        response.status_reason = reason
        if successful:
            self._publish_text_turn_state(turn_id, STATE_CANCELLED, chat_id)
        return response

    def send_chat_message(
        self, request: SendChatMessage.Request, response: SendChatMessage.Response
    ) -> SendChatMessage.Response:
        """callback function for 'send_chat_message' service"""
        response.turn_id = ""
        response.status_reason = ""

        # A typed line for the open live chat joins that session. It does not
        # start a second turn, and it does not close the session.
        decision = typed_text_joins_live(
            live_open=self.gemini_loop.is_listening,
            live_chat_id=self.state.chat_id,
            message_chat_id=request.chat_id,
            text=request.content,
        )
        if decision["join"]:
            response.successful = self.gemini_loop.submit_typed_text(decision["text"])
            return response
        if self.gemini_loop.is_listening and request.chat_id == self.state.chat_id:
            return response

        # do not create a message, if chat is not listening
        if not self.get_is_listening(request.chat_id):
            return response

        # Spoken barge-in only when voice is ON and this chat owns the channel.
        elif request.chat_id == self.state.chat_id and self.state.turned_on:
            self.set_is_listening(request.chat_id, False)
            self.play_audio_from_file(STOP_SIGNAL_FILE)
            self.stop_recording()
            self.set_is_listening(request.chat_id, False)
            self._start_spoken_turn()
            self.chat(
                request.content,
                self.state.chat_id,
                True,
                self.if_cycle_not_changed(
                    self.on_sentence_received, require_voice_on=True
                ),
                self.if_cycle_not_changed(
                    self.on_code_visual_received, require_voice_on=True
                ),
            )

        # Text-only: other chat, or same chat while voice is OFF.
        else:
            try:
                turn_id = self._text_turn_registry.start(
                    request.chat_id,
                    stop=lambda: self._stop_active_chat_goal(request.chat_id),
                )
            except TurnBusy as busy:
                response.successful = False
                response.status_reason = busy.reason
                return response

            self.set_is_listening(request.chat_id, False)
            self._publish_text_turn_state(turn_id, STATE_RUNNING, request.chat_id)

            def on_sentence_received(sentence: str, is_final: bool):
                if is_final:
                    self.set_is_listening(request.chat_id, True)
                    self._text_turn_registry.finish(turn_id)

            self.chat(  # TODO : there is a race condition here, that could lead to the va falsely starting to listen, when activating this chat
                request.content,
                request.chat_id,
                False,
                on_sentence_received,
                turn_id=turn_id,
            )
            response.turn_id = turn_id

        response.successful = True
        return response

    # callback cycle --------------------------------------------------------------------

    def _cloud_session_allowed(self) -> bool:
        """Live and other cloud sessions need the unlocked key store."""
        return allows_cloud_chat(fetch_operating_mode())

    # helper functions ------------------------------------------------------------------


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
