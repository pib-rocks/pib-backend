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

from datatypes.action import Chat, PlayAudioFromFile, PlayAudioFromSpeech, RecordAudio
from datatypes.srv import SetVoiceAssistantState, GetVoiceAssistantState, ClearPlaybackQueue
from datatypes.msg import ChatMessage, VoiceAssistantState

import os

from pib_api_client import chat_client, personality_client



VOICE_ASSISTANT_DIRECTORY = os.getenv("VOICE_ASSISTANT_DIR", "/home/pib/ros_working_dir/src/voice_assistant")
START_SIGNAL_FILE = VOICE_ASSISTANT_DIRECTORY + "/audiofiles/assistant_start_listening.wav"
STOP_SIGNAL_FILE = VOICE_ASSISTANT_DIRECTORY + "/audiofiles/assistant_stop_listening.wav"

MAX_SILENT_SECONDS_BEFORE = 8.0


class Personality:

    def __init__(self, gender: str, language: str, pause_threshold: float, description: str | None = None):
        self.gender = gender
        self.language = language
        self.pause_threshold = pause_threshold
        self.description = description if description is not None else 'Du bist pib, ein humanoider Roboter'



class VoiceAssistantNode(Node):

    def __init__(self):

        super().__init__('voice_assistant')

        self.phase: int = 0
        self.state: VoiceAssistantState = VoiceAssistantState()
        self.state.turned_on = False
        self.state.chat_id = ""
        self.turning_off = False # indicates if the voice_assistant is currently turning off
        self.personality: Personality = Personality("", "", 1.0)

        # the handle for the current 'record_audio' or 'chat' goal
        self.current_goal_handle: ClientGoalHandle | None = None

        # publishers + services -----------------------------------------------

        # Service for setting VoiceAssistantState
        self.set_voice_assistant_service: Service = self.create_service(
            SetVoiceAssistantState, 
            'set_voice_assistant_state',
            self.set_voice_assistant_state)

        # Service for getting current VoiceAssistantState
        self.get_voice_assistant_service: Service = self.create_service(
            GetVoiceAssistantState, 
            'get_voice_assistant_state',
            self.get_voice_assistant_state)

        # Publisher for VoiceAssistantState
        self.voice_assistant_state_publisher: Publisher = self.create_publisher(
            VoiceAssistantState, 
            "voice_assistant_state",
            10)

        # Publisher for ChatMessages
        self.chat_message_publisher: Publisher = self.create_publisher(
            ChatMessage, 
            "chat_messages",
            10)

        # clients -------------------------------------------------------------

        self.clear_playback_queue_client = self.create_client(ClearPlaybackQueue, 'clear_playback_queue')
        self.clear_playback_queue_client.wait_for_service()

        self.record_audio_client = ActionClient(self, RecordAudio, 'record_audio')
        self.record_audio_client.wait_for_server()

        self.chat_client = ActionClient(self, Chat, 'chat')
        self.chat_client.wait_for_server()

        self.play_audio_from_file_client = ActionClient(self, PlayAudioFromFile, 'play_audio_from_file')
        self.play_audio_from_file_client.wait_for_server()

        self.play_audio_from_speech_client = ActionClient(self, PlayAudioFromSpeech, 'play_audio_from_speech')
        self.play_audio_from_speech_client.wait_for_server()

        self.get_logger().info('Now running VA')


    
    def if_phase_not_changed(self, callback: Callable) -> Callable:
        """a decorator. the decorated callback only executes, if the phase has not changed after its creation"""

        current_phase = self.phase
        def decorated_callback(*args):
            if self.phase == current_phase: callback(*args)

        return decorated_callback
    


    def add_result_callback(self, future: Future, callback: Callable[[Any], None]) -> None:
        """executes the callback, with the result of the action as argument"""

        def result_callback(result_future: Future):
            result = result_future.result().result
            callback(result)

        def done_callback(goal_handle_future: Future):
            goal_handle: ClientGoalHandle = goal_handle_future.result()
            self.current_goal_handle = goal_handle
            result_future: Future = goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)

        future.add_done_callback(done_callback)



    def get_voice_assistant_state(self, _: GetVoiceAssistantState.Request, response: GetVoiceAssistantState.Response) -> GetVoiceAssistantState.Response:

        response.voice_assistant_state.chat_id = self.state.chat_id
        response.voice_assistant_state.turned_on = self.state.turned_on

        return response



    def set_voice_assistant_state(self, request: SetVoiceAssistantState.Request, response: SetVoiceAssistantState.Response) -> SetVoiceAssistantState.Response:
        
        request_state: VoiceAssistantState = request.voice_assistant_state

        try:
            if self.turning_off:
                raise Exception("voice assistant is currently turning off")
            if request_state.turned_on == self.state.turned_on:
                raise Exception(f"voice assistant is already turned {'on' if request_state.turned_on else 'off'}.")
            if request_state.turned_on: self.activate_voice_assistant(request_state.chat_id)
            else: self.deactivate_voice_assistant()
            self.state = request_state
            response.successful = True

        except Exception as e:
            self.get_logger().error(f"following error occured while trying to set voice assistant state: {str(e)}.")
            response.successful = False

        self.voice_assistant_state_publisher.publish(self.state)

        return response
    


    def activate_voice_assistant(self, chat_id: str) -> None:
        
        self.personality = self.get_personality_from_chat_id(chat_id)
        if self.personality is None:
            raise Exception(f"no personality with chat of id {chat_id} found...")

        goal = PlayAudioFromFile.Goal()
        goal.filepath = START_SIGNAL_FILE
        future: Future = self.play_audio_from_file_client.send_goal_async(goal)
        self.add_result_callback(future, self.if_phase_not_changed(self.on_start_signal_played))



    def deactivate_voice_assistant(self) -> None:
        
        self.phase += 1
        self.turning_off = True

        if self.current_goal_handle is not None: 
            self.current_goal_handle.cancel_goal_async()

        future = self.clear_playback_queue_client.call_async(ClearPlaybackQueue.Request())
        future.add_done_callback(self.if_phase_not_changed(self.on_playback_queue_cleared))

    

    def on_playback_queue_cleared(self, _: Future):

        self.turning_off = False

        goal = PlayAudioFromFile.Goal()
        goal.filepath = STOP_SIGNAL_FILE
        self.play_audio_from_file_client.send_goal_async(goal)



    def get_personality_from_chat_id(self, chat_id: str) -> Personality | None:

        successful, chat_dto = chat_client.get_chat(chat_id)
        if not successful: return None
        successful, personality_dto = personality_client.get_personality(chat_dto['personalityId'])
        if not successful: return None
        return Personality(
            personality_dto['gender'],
            "German", # TODO: language should be saved in db -> replace with 'personality_dto['language']'
            personality_dto['pauseThreshold'],
            personality_dto['description'])



    def on_start_signal_played(self, _: PlayAudioFromFile.Result) -> None:
        
        goal = RecordAudio.Goal()
        goal.max_silent_seconds_before = MAX_SILENT_SECONDS_BEFORE
        goal.max_silent_seconds_after = self.personality.pause_threshold
        future: Future = self.record_audio_client.send_goal_async(goal, self.if_phase_not_changed(self.on_stopped_recording))
        self.add_result_callback(future, self.if_phase_not_changed(self.on_transcribed_user_input_received))


    
    def on_stopped_recording(self, _: RecordAudio.Feedback) -> None:

        goal = PlayAudioFromFile.Goal()
        goal.filepath = STOP_SIGNAL_FILE
        self.play_audio_from_file_client.send_goal_async(goal)



    def on_transcribed_user_input_received(self, result: RecordAudio.Result) -> None:

        user_text = result.transcribed_text

        goal = Chat.Goal()
        goal.text = user_text
        goal.description = self.personality.description
        future: Future = self.chat_client.send_goal_async(goal, self.if_phase_not_changed(self.on_sentence_received))
        self.add_result_callback(future, self.if_phase_not_changed(self.on_final_sentence_received))

        self.create_chat_message(user_text, True)

    

    def on_sentence_received(self, feedback_message: Any) -> None:

        feedback: Chat.Feedback = feedback_message.feedback
        sentence = feedback.sentence

        goal = PlayAudioFromSpeech.Goal()
        goal.gender = self.personality.gender
        goal.language = self.personality.language
        goal.speech = feedback.sentence

        self.play_audio_from_speech_client.send_goal_async(goal)

        self.create_chat_message(sentence, False)



    def on_final_sentence_received(self, result: Chat.Result) -> None:

        sentence = result.rest

        goal = PlayAudioFromSpeech.Goal()
        goal.gender = self.personality.gender
        goal.language = self.personality.language
        goal.speech = sentence

        future: Future = self.play_audio_from_speech_client.send_goal_async(goal)
        self.add_result_callback(future, self.if_phase_not_changed(self.on_final_sentence_played))

        self.create_chat_message(sentence, False)


    
    def create_chat_message(self, text: str, is_user: bool) -> None:

        if text == "": return

        _, chat_message_dto = chat_client.create_chat_message(self.state.chat_id, text, is_user)

        chat_message_ros = ChatMessage(
            chat_id=self.state.chat_id,
            content=chat_message_dto['content'],
            is_user=chat_message_dto['isUser'],
            message_id=chat_message_dto['messageId'],
            timestamp=chat_message_dto['timestamp'])
            
        self.chat_message_publisher.publish(chat_message_ros)



    def on_final_sentence_played(self, _: PlayAudioFromSpeech.Result) -> None:

        goal = PlayAudioFromFile.Goal()
        goal.filepath = START_SIGNAL_FILE
        future: Future = self.play_audio_from_file_client.send_goal_async(goal)
        self.add_result_callback(future, self.if_phase_not_changed(self.on_start_signal_played)) 



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
