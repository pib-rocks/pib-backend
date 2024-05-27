from typing import Any, Callable
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.service import Service
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.publisher import Publisher
from rclpy.client import Client

from datatypes.action import Chat, RecordAudio
from datatypes.srv import (
    SetVoiceAssistantState,
    GetVoiceAssistantState,
    ClearPlaybackQueue,
    PlayAudioFromFile,
    PlayAudioFromSpeech,
    GetChatIsListening,
    SendChatMessage,
)
from datatypes.msg import VoiceAssistantState, ChatIsListening

import os

from pib_api_client import voice_assistant_client
from pib_api_client.voice_assistant_client import Personality

VOICE_ASSISTANT_DIRECTORY = os.getenv(
    "VOICE_ASSISTANT_DIR", "/home/pib/ros_working_dir/src/voice_assistant"
)
START_SIGNAL_FILE = (
    VOICE_ASSISTANT_DIRECTORY + "/audiofiles/assistant_start_listening.wav"
)
STOP_SIGNAL_FILE = (
    VOICE_ASSISTANT_DIRECTORY + "/audiofiles/assistant_stop_listening.wav"
)
MAX_SILENT_SECONDS_BEFORE = 8.0


class VoiceAssistantNode(Node):

    def __init__(self):

        super().__init__("voice_assistant")

        # state -------------------------------------------------------------------------

        self.cycle: int = (
            0  # a counter for indicating the index of the current on-off-cycle
        )
        self.state: VoiceAssistantState = VoiceAssistantState()
        self.state.turned_on = False  # indicates if the va is turned on or off
        self.state.chat_id = ""  # id of the active chat
        self.turning_off = (
            False  # indicates if the voice_assistant is currently turning off
        )
        self.personality: Personality = (
            None  # the personality associated with the active chat
        )
        self.stop_recording: Callable[[], None] = (
            lambda: None
        )  # calling this function stops audio-recording
        self.chat_id_to_stop_chat: dict[str, Callable[[], None]] = (
            {}
        )  # maps a chat-id to a function that can be used to stop receiving llm-responses
        self.chat_id_to_is_listening: dict[str, bool] = (
            {}
        )  # maps a chat-id to the listening status of the respective chat
        self.waiting_for_transcribed_text = False  # indicates, whether audio was recorded and va is currently awaitng the transcription

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
        on_sentence_received: Callable[[str], None] = None,
        on_final_sentence_received: Callable[[str], None] = None,
    ) -> None:
        goal = Chat.Goal()
        goal.text = text
        goal.chat_id = chat_id
        feedback_callback = (
            self._deactivate_voice_assistant()
            if on_sentence_received is None
            else lambda msg: on_sentence_received(msg.feedback.sentence)
        )
        result_callback = (
            self._deactivate_voice_assistant()
            if on_final_sentence_received is None
            else lambda result: on_final_sentence_received(result.rest)
        )
        future: Future = self.chat_client.send_goal_async(goal, feedback_callback)
        stop_chat = self.digest_goal_handle_future(future, result_callback)
        self.chat_id_to_stop_chat[chat_id] = stop_chat

    def play_audio_from_file(
        self, filepath: str, on_stopped_playing: Callable[[], None] = None
    ):
        request = PlayAudioFromFile.Request()
        request.filepath = filepath
        request.join = on_stopped_playing is not None
        future: Future = self.play_audio_from_file_client.call_async(request)
        if request.join:
            future.add_done_callback(lambda _: on_stopped_playing())

    def play_audio_from_speech(
        self,
        speech: str,
        gender: str,
        language: str,
        on_stopped_playing: Callable[[], None] = None,
    ):
        request = PlayAudioFromSpeech.Request()
        request.speech = speech
        request.gender = gender
        request.language = language
        request.join = on_stopped_playing is not None
        future: Future = self.play_audio_from_speech_client.call_async(request)
        if request.join:
            future.add_done_callback(lambda _: on_stopped_playing())

    # serivce callbacks -----------------------------------------------------------------

    def get_voice_assistant_state(
        self,
        _: GetVoiceAssistantState.Request,
        response: GetVoiceAssistantState.Response,
    ) -> GetVoiceAssistantState.Response:
        response.voice_assistant_state = self.state
        return response

    def set_voice_assistant_state(
        self,
        request: SetVoiceAssistantState.Request,
        response: SetVoiceAssistantState.Response,
    ) -> SetVoiceAssistantState.Response:
        request_state: VoiceAssistantState = request.voice_assistant_state

        try:

            if self.turning_off:  # ignore if currently turning off
                raise Exception("voice assistant is currently turning off")

            elif (
                request_state.turned_on == self.state.turned_on
            ):  # ignore if activation stage not changed
                raise Exception(
                    f"voice assistant is already turned {'on' if request_state.turned_on else 'off'}."
                )

            elif not request_state.turned_on:  # deactivate voice assistant
                self.cycle += 1
                self.turning_off = True
                self.stop_recording()
                self.stop_chat(self.state.chat_id)
                current_chat_id = self.state.chat_id

                def on_playback_queue_cleared():
                    self.turning_off = False
                    self.set_is_listening(current_chat_id, True)
                    self.play_audio_from_file(STOP_SIGNAL_FILE)

                self.clear_playback_queue(on_playback_queue_cleared)

            else:  # activate voice assistant
                self.stop_chat(request_state.chat_id)
                successful, self.personality = (
                    voice_assistant_client.get_personality_from_chat(
                        request_state.chat_id
                    )
                )
                if not successful:
                    raise Exception(
                        f"no personality with chat of id {request_state.chat_id} found..."
                    )
                self.set_is_listening(request_state.chat_id, False)
                self.play_audio_from_file(
                    START_SIGNAL_FILE,
                    self.if_cycle_not_changed(self.on_start_signal_played),
                )

            self.state = request_state
            response.successful = True

        except Exception as e:
            self.get_logger().error(
                f"following error occured while trying to set voice assistant state: {str(e)}."
            )

        self.voice_assistant_state_publisher.publish(self.state)

        return response

    def get_chat_is_listening(
        self, request: GetChatIsListening.Request, response: GetChatIsListening.Response
    ) -> GetChatIsListening.Response:
        response.listening = self.get_is_listening(request.chat_id)
        return response

    def send_chat_message(
        self, request: SendChatMessage.Request, response: SendChatMessage.Response
    ) -> GetChatIsListening.Response:
        if not self.get_is_listening(
            request.chat_id
        ):  # do not create a message, if chat is not listening
            return response

        elif (
            request.chat_id == self.state.chat_id
        ):  # if chat is active, jump to next stage of the va-cycle
            self.set_is_listening(request.chat_id, False)
            self.play_audio_from_file(STOP_SIGNAL_FILE)
            self.stop_recording()
            self.set_is_listening(request.chat_id, False)
            self.chat(
                request.content,
                self.state.chat_id,
                self.if_cycle_not_changed(self.on_sentence_received),
                self.if_cycle_not_changed(self.on_final_sentence_received),
            )

        else:  # if not active, create messages, without playing audio etc.
            self.set_is_listening(request.chat_id, False)
            self.chat(
                request.content,
                request.chat_id,
                on_final_sentence_received=lambda _: self.set_is_listening(
                    request.chat_id, True
                ),
            )

        response.successful = True
        return response

    # callback cycle --------------------------------------------------------------------

    def on_start_signal_played(self) -> None:
        self.record_audio(
            MAX_SILENT_SECONDS_BEFORE,
            self.personality.pause_threshold,
            self.if_cycle_not_changed(self.on_stopped_recording),
            self.if_cycle_not_changed(self.on_transcribed_text_received),
        )

        self.set_is_listening(self.state.chat_id, True)

    def on_stopped_recording(self) -> None:
        if not self.get_is_listening(self.state.chat_id):
            return

        self.play_audio_from_file(STOP_SIGNAL_FILE)
        self.set_is_listening(self.state.chat_id, False)
        self.waiting_for_transcribed_text = True

    def on_transcribed_text_received(self, transcribed_text: str) -> None:
        if not self.waiting_for_transcribed_text:
            return
        self.waiting_for_transcribed_text = False

        self.chat(
            transcribed_text,
            self.state.chat_id,
            self.if_cycle_not_changed(self.on_sentence_received),
            self.if_cycle_not_changed(self.on_final_sentence_received),
        )

    def on_sentence_received(self, sentence: str) -> None:
        if not sentence:
            self._turn_off_voice_assistant()
            return
        self.play_audio_from_speech(
            sentence, self.personality.gender, self.personality.language
        )

    def on_final_sentence_received(self, sentence: str) -> None:
        if not sentence:
            self._turn_off_voice_assistant()
            return
        self.play_audio_from_speech(
            sentence,
            self.personality.gender,
            self.personality.language,
            self.if_cycle_not_changed(self.on_final_sentence_played),
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
        """if the chat of the provided is active, stop receiving messages from the chat"""
        stop_chat = self.chat_id_to_stop_chat.get(chat_id)
        if stop_chat is not None:
            stop_chat()

    def _turn_off_voice_assistant(self):
        request: SetVoiceAssistantState.Request = SetVoiceAssistantState.Request()
        request.voice_assistant_state = self.state
        request.voice_assistant_state.turned_on = False

        response = SetVoiceAssistantState.Response()
        self.set_voice_assistant_state(request, response)


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
