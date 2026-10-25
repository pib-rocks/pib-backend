import os
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

MAX_SILENT_SECONDS_BEFORE = 8.0


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


_CLEAR_PLAYBACK_ACK_LOG_SECONDS = 5.0


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
        except Exception:
            self.get_logger().error(
                "duplex voice backend unavailable; staying on the Gemini live path",
                exc_info=True,
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

    def _start_spoken_turn(self) -> None:
        """Arm clause playback and, when the personality wrote one, a filler."""
        # Every answer owns its callbacks, including consecutive turns in one
        # ON-cycle. An old drain must never release the next answer.
        self._invalidate_voice_async_work()
        self._cancel_filler_timer()
        self._streaming_speech = StreamingSpeech()
        self._turn_has_playback = False
        self._answer_audio_pending = False
        with self._filler_lock:
            self._first_token_seen = False
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
        timer = threading.Timer(
            budget_ms / 1000.0, lambda: self._schedule_voice_callback(owned_fire)
        )
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
        self.record_audio(
            MAX_SILENT_SECONDS_BEFORE,
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
            STOP_SIGNAL_FILE,
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
            self.get_logger().info(
                "nothing transcribed in this window; no chat turn, listening again"
            )
            self.play_audio_from_file(
                START_SIGNAL_FILE,
                self.if_cycle_not_changed(
                    self.on_start_signal_played, require_voice_on=True
                ),
            )
            return
        self._start_spoken_turn()
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
        self._playback_epoch += 1
        self._close_latency_turn(incomplete=False)
        if not self.state.turned_on or self.turning_off:
            return
        self.play_audio_from_file(
            START_SIGNAL_FILE,
            self.if_cycle_not_changed(
                self.on_start_signal_played, require_voice_on=True
            ),
        )

    # helper functions ------------------------------------------------------------------

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
        self._streaming_speech = StreamingSpeech()
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
        timer = threading.Timer(
            timeout_s, lambda: self._schedule_voice_callback(owned_timeout)
        )
        timer.daemon = True
        self._pending_drain_watchdog = timer
        timer.start()

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
        self._answer_is_fallback = answer_uses_fallback(speech, FALLBACK_REPLY)
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

    def barge_in(self, chat_id: Optional[str] = None) -> bool:
        """Interrupt active playback and stream cleanly so the next turn can start."""
        effective_chat_id = chat_id or self.state.chat_id
        if not effective_chat_id:
            return False
        self.cycle += 1
        self._invalidate_voice_async_work()
        self.stop_chat(effective_chat_id)
        self.final_chat_response_received = False
        self.state.speaking = False

        def on_cleared():
            if self.state.turned_on and effective_chat_id == self.state.chat_id:
                self.on_start_signal_played()
            else:
                self.set_is_listening(effective_chat_id, True)

        self.clear_playback_queue(on_cleared)
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

                def on_playback_queue_cleared():
                    if self._playback_epoch != cleared_epoch:
                        return
                    self.turning_off = False
                    self.set_is_listening(current_chat_id, True)

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
                self.play_audio_from_file(START_SIGNAL_FILE)
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
                turned_on == self.state.turned_on
                and effective_chat_id == self.state.chat_id
            ):
                raise Exception(
                    f"voice assistant is already turned {'on' if turned_on else 'off'}."
                )

            self.stop_chat(effective_chat_id)
            self.set_is_listening(effective_chat_id, False)
            self.state.turned_on = True
            self.state.chat_id = effective_chat_id
            self._invalidate_voice_async_work()
            self.play_audio_from_file(
                START_SIGNAL_FILE,
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
