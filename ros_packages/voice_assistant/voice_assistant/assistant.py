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
from datatypes.srv import SetVoiceAssistantState, GetVoiceAssistantState, ClearPlaybackQueue, PlayAudioFromFile, PlayAudioFromSpeech, GetVoiceAssistantChatIsListening, SendChatMessage
from datatypes.msg import ChatMessage, VoiceAssistantState, VoiceAssistantChatIsListening

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

        # state -------------------------------------------------------------------------

        self.cycle: int = 0
        self.state: VoiceAssistantState = VoiceAssistantState()
        self.state.turned_on = False
        self.state.chat_id = ""
        self.turning_off = False # indicates if the voice_assistant is currently turning off
        self.personality: Personality = Personality("", "", 1.0)
        self.stop_recording: Callable[[], None] = lambda: None
        self.stop_chat: Callable[[], None] = lambda: None
        self.chat_id_to_is_listening: dict[str, bool] = {}

        # services ----------------------------------------------------------------------

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
        
        # Service for getting the listening status of a chat
        self.get_voice_assistant_chat_is_listening_service: Service = self.create_service(
            GetVoiceAssistantChatIsListening, 
            'get_voice_assistant_chat_is_listening',
            self.get_voice_assistant_chat_is_listening)
        
        # Service that allows clients to send chat messages
        self.send_chat_message: Service = self.create_service(
            SendChatMessage, 
            'send_chat_message',
            self.send_chat_message)
        
        # publishers --------------------------------------------------------------------

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

        # Publisher for VoiceAssistantChatIsListening
        self.voice_assistant_chat_is_listening_publisher: Publisher = self.create_publisher(
            VoiceAssistantChatIsListening, 
            "voice_assistant_chat_is_listening",
            10)

        # clients -----------------------------------------------------------------------

        self.chat_client: ActionClient = ActionClient(self, Chat, 'chat')
        self.chat_client.wait_for_server()

        self.record_audio_client: ActionClient = ActionClient(self, RecordAudio, 'record_audio')
        self.record_audio_client.wait_for_server()

        self.play_audio_from_file_client: Client = self.create_client(PlayAudioFromFile, 'play_audio_from_file')
        self.play_audio_from_file_client.wait_for_service()

        self.play_audio_from_speech_client: Client = self.create_client(PlayAudioFromSpeech, 'play_audio_from_speech')
        self.play_audio_from_speech_client.wait_for_service()

        self.clear_playback_queue_client: Client = self.create_client(ClearPlaybackQueue, 'clear_playback_queue')
        self.clear_playback_queue_client.wait_for_service()

        self.get_logger().info('Now running VA')

    
    # client accessors ------------------------------------------------------------------


    def clear_playback_queue(self, 
                             on_playback_queue_cleared: Callable[[], None] = None):
        
        future = self.clear_playback_queue_client.call_async(ClearPlaybackQueue.Request())
        future.add_done_callback(lambda _ : on_playback_queue_cleared())

    def record_audio(self, 
                     max_silent_seconds_before: float, 
                     max_silent_seconds_after: float, 
                     on_stopped_recording: Callable[[], None] = None, 
                     on_transcribed_text_received: Callable[[str], None] = None) -> Callable[[], None]:
        
        goal = RecordAudio.Goal()
        goal.max_silent_seconds_before = max_silent_seconds_before
        goal.max_silent_seconds_after = max_silent_seconds_after
        future: Future = self.record_audio_client.send_goal_async(goal, lambda _ : on_stopped_recording())
        def result_callback(result: RecordAudio.Result) -> None: on_transcribed_text_received(result.transcribed_text)
        return self.digest_goal_handle_future(future, result_callback if on_transcribed_text_received is not None else None)

    def chat(self, 
             text: str, 
             description: str, 
             on_sentence_received: Callable[[str], None] = None, 
             on_final_sentence_received: Callable[[str], None] = None) -> Callable[[], None]:
        
        goal = Chat.Goal()
        goal.text = text
        goal.description = description
        future: Future = self.chat_client.send_goal_async(goal, lambda msg: on_sentence_received(msg.feedback.sentence))
        result_callback = None
        def result_callback(result: Chat.Result) -> None: on_final_sentence_received(result.rest)
        return self.digest_goal_handle_future(future, result_callback if on_final_sentence_received is not None else None)

    def play_audio_from_file(self, 
                             filepath: str, 
                             on_stopped_playing: Callable[[], None] = None):
        
        request = PlayAudioFromFile.Request()
        request.filepath = filepath
        future: Future = self.play_audio_from_file_client.call_async(request)
        if on_stopped_playing is not None: future.add_done_callback(lambda _ : on_stopped_playing())

    def play_audio_from_speech(self, 
                               speech: str, 
                               gender: str, 
                               language: str, 
                               on_stopped_playing: Callable[[], None] = None):
        
        request = PlayAudioFromSpeech.Request()
        request.speech = speech
        request.gender = gender
        request.language = language
        future: Future = self.play_audio_from_speech_client.call_async(request)
        if on_stopped_playing is not None: future.add_done_callback(lambda _ : on_stopped_playing())


    # serivce callbacks -----------------------------------------------------------------


    def get_voice_assistant_state(self, _: GetVoiceAssistantState.Request, response: GetVoiceAssistantState.Response) -> GetVoiceAssistantState.Response:

        response.voice_assistant_state = self.state
        return response



    def set_voice_assistant_state(self, request: SetVoiceAssistantState.Request, response: SetVoiceAssistantState.Response) -> SetVoiceAssistantState.Response:
        
        request_state: VoiceAssistantState = request.voice_assistant_state

        try:

            if self.turning_off: # ignore if currently turning off
                raise Exception("voice assistant is currently turning off")
            
            elif request_state.turned_on == self.state.turned_on: # ignore if activation stage not changed
                raise Exception(f"voice assistant is already turned {'on' if request_state.turned_on else 'off'}.")
            
            elif not request_state.turned_on: # deactivate voice assistant
                self.cycle += 1
                self.turning_off = True
                self.stop_recording()
                self.stop_chat()
                def on_playback_queue_cleared():
                    self.turning_off = False
                    self.set_is_listening(request_state.chat_id, True)
                    self.play_audio_from_file(STOP_SIGNAL_FILE)
                self.clear_playback_queue(on_playback_queue_cleared)

            elif not self.get_is_listening(request_state.chat_id): # do not activate, if chat is not listening for input
                raise Exception(f"cannot activate, because chat with id {request_state.chat_id} is currently not listening for user input")
            
            else:  # activate voice assistant
                self.personality = self.get_personality_from_chat_id(request_state.chat_id)
                if self.personality is None: raise Exception(f"no personality with chat of id {request_state.chat_id} found...")
                self.set_is_listening(request_state.chat_id, False)
                self.play_audio_from_file(START_SIGNAL_FILE, self.if_cycle_not_changed(self.on_start_signal_played))

            self.state = request_state
            response.successful = True

        except Exception as e:
            self.get_logger().error(f"following error occured while trying to set voice assistant state: {str(e)}.")

        self.voice_assistant_state_publisher.publish(self.state)

        return response
    


    def get_voice_assistant_chat_is_listening(self, request: GetVoiceAssistantChatIsListening.Request, response: GetVoiceAssistantChatIsListening.Response) -> GetVoiceAssistantChatIsListening.Response:

        response.listening = self.get_is_listening(request.chat_id)
        return response



    def send_chat_message(self, request: SendChatMessage.Request, response: SendChatMessage.Response) -> GetVoiceAssistantChatIsListening.Response:

        if not self.get_is_listening(request.chat_id): # do not create a message, if chat is not listening
            return response
        
        elif request.chat_id == self.state.chat_id: # if chat is active, jump to next stage of the va-cycle
            self.set_is_listening(request.chat_id, False)
            self.play_audio_from_file(STOP_SIGNAL_FILE)
            self.stop_recording()
            self.on_user_input_text_received(request.content)

        else: # if not active, create messages, without playing audio etc.
            personality = self.get_personality_from_chat_id(request.chat_id)
            if personality is None: return response
            self.create_chat_message(request.chat_id, request.content, True)
            self.set_is_listening(request.chat_id, False)
            def on_sentence_received(sentence: str) -> None:
                self.create_chat_message(request.chat_id, sentence, False)
            def on_final_sentence_received(sentence: str) -> None:
                on_sentence_received(sentence)
                self.set_is_listening(request.chat_id, True)
            self.chat(
                request.content,
                personality.description,
                on_sentence_received,
                on_final_sentence_received)

        response.successful = True
        return response
    


    # callback cycle --------------------------------------------------------------------


    def on_start_signal_played(self) -> None:

        self.record_audio(
            MAX_SILENT_SECONDS_BEFORE, 
            self.personality.pause_threshold,
            self.if_cycle_not_changed(self.on_stopped_recording),
            self.if_cycle_not_changed(self.on_user_input_text_received))
        
        self.set_is_listening(self.state.chat_id, True)
        
        

    
    def on_stopped_recording(self) -> None:

        if not self.get_is_listening(self.state.chat_id): return

        self.play_audio_from_file(STOP_SIGNAL_FILE)
        self.set_is_listening(self.state.chat_id, False)



    def on_user_input_text_received(self, transcribed_text: str) -> None:

        self.chat(
            transcribed_text, 
            self.personality.description,
            self.if_cycle_not_changed(self.on_sentence_received),
            self.if_cycle_not_changed(self.on_final_sentence_received))

        self.create_chat_message(self.state.chat_id, transcribed_text, True)

    

    def on_sentence_received(self, sentence: str) -> None:

        self.play_audio_from_speech(
            sentence, 
            self.personality.gender, 
            self.personality.language)

        self.create_chat_message(self.state.chat_id, sentence, False)



    def on_final_sentence_received(self, sentence: str) -> None:

        self.play_audio_from_speech(
            sentence, 
            self.personality.gender, 
            self.personality.language,
            self.if_cycle_not_changed(self.on_final_sentence_played))
        
        self.create_chat_message(self.state.chat_id, sentence, False)



    def on_final_sentence_played(self) -> None:

        self.play_audio_from_file(
            START_SIGNAL_FILE, 
            self.if_cycle_not_changed(self.on_start_signal_played))
        

    # helper functions ------------------------------------------------------------------


    def get_personality_from_chat_id(self, chat_id: str) -> Personality | None:
        """get the personality associated with the chat_id from the db"""

        successful, chat_dto = chat_client.get_chat(chat_id)
        if not successful: return None
        successful, personality_dto = personality_client.get_personality(chat_dto['personalityId'])
        if not successful: return None
        return Personality(
            personality_dto['gender'],
            "German", # TODO: language should be saved in db -> replace with 'personality_dto['language']'
            personality_dto['pauseThreshold'],
            personality_dto['description'])
    


    def create_chat_message(self, chat_id: str, text: str, is_user: bool) -> None:
        """writes a new chat-message to the db, and publishes it to the 'chat_messages'-topic"""

        if text == "": return

        _, chat_message_dto = chat_client.create_chat_message(chat_id, text, is_user)

        chat_message_ros = ChatMessage()
        chat_message_ros.chat_id = chat_id
        chat_message_ros.content = chat_message_dto['content']
        chat_message_ros.is_user = chat_message_dto['isUser']
        chat_message_ros.message_id = chat_message_dto['messageId']
        chat_message_ros.timestamp = chat_message_dto['timestamp']

        self.chat_message_publisher.publish(chat_message_ros)

    

    def if_cycle_not_changed(self, callback: Callable) -> Callable:
        """a decorator. the decorated callback only executes, if the cycle has not changed after its creation"""

        current_cycle = self.cycle
        def decorated_callback(*args):
            if self.cycle == current_cycle: callback(*args)

        return decorated_callback
    


    def digest_goal_handle_future(self, goal_handle_future: Future, callback: Callable[[Any], None] = None) -> Callable[[], None]:
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

        voice_assistant_chat_is_listening = VoiceAssistantChatIsListening()
        voice_assistant_chat_is_listening.listening = listening
        voice_assistant_chat_is_listening.chat_id = chat_id

        self.voice_assistant_chat_is_listening_publisher.publish(voice_assistant_chat_is_listening)



    def get_is_listening(self, chat_id: str) -> bool:
        """find out, if a chat is currently listening for user input"""

        return self.chat_id_to_is_listening.get(chat_id, True)



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