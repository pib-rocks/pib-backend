from multiprocessing.connection import Connection
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from datatypes.srv import SetVoiceAssistantState, GetVoiceAssistantState, TextToSpeechPush, TextToSpeechClear
from datatypes.msg import ChatMessage, VoiceAssistantState

from multiprocessing import Process, Pipe
from threading import Lock

from pib_voice.voice import gpt_chat, play_audio, speech_to_text
from pib_api_client import chat_client, personality_client



RECEIVE_CHAT_MESSAGE_WAITING_PERIOD_SECONDS = 0.1

VOICE_ASSISTANT_PATH_PREFIX = "/home/pib/ros_working_dir/src/voice_assistant"
START_SIGNAL_FILE = VOICE_ASSISTANT_PATH_PREFIX + "/audiofiles/assistant_start_listening.wav"
STOP_SIGNAL_FILE = VOICE_ASSISTANT_PATH_PREFIX + "/audiofiles/assistant_stop_listening.wav"

pib_api_client_lock = Lock()



class Personality:

    def __init__(self, gender: str, pause_threshold: float, description: str | None = None):
        self.gender = gender
        self.pause_threshold = pause_threshold
        self.description = description if description is not None else 'Du bist pib, ein humanoider Roboter'



class TransientChatMessage:

    def __init__(self, content: str, is_user: bool, chat_id: float, gender: str | None):
        self.content = content
        self.is_user = is_user
        self.chat_id = chat_id
        self.gender = gender



class VoiceAssistantNode(Node):

    def __init__(self, ros_to_main: Connection):

        super().__init__('voice_assistant')
        self.get_logger().info('Now running VA')

        chat_message_callback_group = MutuallyExclusiveCallbackGroup()
        va_state_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

    	# stores the current state of the va, to allow the current
        # state to be retrieved via the 'get_voice_assistant_state'-service
        self.state = VoiceAssistantState(turned_on=False, chat_id='')
        self.state_lock: type[Lock] = Lock()

        # for communication bewtween ros and worker. Needs a lock, since
        # it is accessed by both the timer-callback, as well as the set-
        # voice-assistant-state-callback, which may run in parallel
        self.ros_to_worker: Connection = None
        self.worker_changed: bool = False
        self.ros_to_worker_lock = Lock()

        # for communication between ros and main. does not need a lock,
        # since it is only accessed by the 'set_voice_assistant_state'
        # callback which is in a mutex-callback-group
        self.ros_to_main: Connection = ros_to_main

        # Service for setting VoiceAssistantState
        self.set_voice_assistant_service = self.create_service(
            SetVoiceAssistantState, 
            'set_voice_assistant_state',
            self.set_voice_assistant_state,
            callback_group=va_state_callback_group
        )

        # Service for getting current VoiceAssistantState
        self.get_voice_assistant_service = self.create_service(
            GetVoiceAssistantState, 
            'get_voice_assistant_state',
            self.get_voice_assistant_state,
            callback_group=va_state_callback_group
        )

        # Publisher for VoiceAssistantState
        self.voice_assistant_state_publisher = self.create_publisher(
            VoiceAssistantState, 
            "voice_assistant_state", 
            10,
            callback_group=va_state_callback_group
        )

        # Publisher for ChatMessages
        self.chat_message_publisher = self.create_publisher(
            ChatMessage, 
            "chat_messages", 
            10,
            callback_group=chat_message_callback_group
        )

        # Check for Start Signal periodically
        self.timer = self.create_timer(
            RECEIVE_CHAT_MESSAGE_WAITING_PERIOD_SECONDS, 
            self.forward_messages,
            callback_group=timer_callback_group
        )

        # Client for sending requests to text-to-speech
        self.tts_push_client = self.create_client(TextToSpeechPush, 'tts_push')
        self.tts_clear_client = self.create_client(TextToSpeechClear, 'tts_clear')



    def get_voice_assistant_state(self, _, response: GetVoiceAssistantState.Response):

        with self.state_lock:
            response.voice_assistant_state.chat_id = self.state.chat_id
            response.voice_assistant_state.turned_on = self.state.turned_on

        return response
    


    def get_personality_from_chat_id(self, chat_id: str) -> Personality | None:

        with pib_api_client_lock:
            successful, chat_dto = chat_client.get_chat(chat_id)
            if not successful: return None
            successful, personality_dto = personality_client.get_personality(chat_dto['personalityId'])
            if not successful: return None
            return Personality(
                personality_dto['gender'],
                personality_dto['pauseThreshold'],
                personality_dto['description'])
            
        

    def set_voice_assistant_state(self, request: SetVoiceAssistantState.Request, response: SetVoiceAssistantState.Response):

        request_state: VoiceAssistantState = request.voice_assistant_state

        with self.state_lock:
            
            try:
                if request_state.turned_on == self.state.turned_on:
                    raise Exception(f"voice assistant is already turned {'on' if request_state.turned_on else 'off'}.")
                
                elif request_state.turned_on:
                    personality = self.get_personality_from_chat_id(request_state.chat_id)
                    if personality is None: 
                        raise Exception(f"no personality for chat with id {request_state.chat_id} found.")
                    self.ros_to_main.send((personality, request_state.chat_id))
                    with self.ros_to_worker_lock: self.ros_to_worker = self.ros_to_main.recv()
                
                else:
                    self.tts_clear_client.call(TextToSpeechClear.Request())
                    with self.ros_to_worker_lock: 
                        self.worker_changed = True # TODO: worker_changed -> worker_stale
                        self.ros_to_worker = None
                    self.ros_to_main.send(None)
                
                self.state = request_state
                response.successful = True

            except Exception as e:
                self.get_logger().error(f"following error occured while trying to set voice assistant state: {str(e)}.")
                response.successful = False

            self.voice_assistant_state_publisher.publish(self.state)
            
        return response
            


    def forward_messages(self):

        while True:

            with self.ros_to_worker_lock:
                if self.ros_to_worker is None or not self.ros_to_worker.poll(): return
                chat_message: TransientChatMessage = self.ros_to_worker.recv()
                self.worker_changed = False

            if not chat_message.is_user:

                request = TextToSpeechPush.Request()
                request.text = chat_message.content
                request.voice = "Vicki" if chat_message.gender == "Female" else "Daniel"
                request.join = True
                
                future: Future = self.tts_push_client.call_async(request)

                def when_done(_: Future):
                    with self.ros_to_worker_lock: 
                        if not self.worker_changed:
                            self.ros_to_worker.send(chat_message.chat_id) 

                future.add_done_callback(when_done)

            with pib_api_client_lock: 
                _, chat_message_dto = chat_client.create_chat_message(
                    chat_message.chat_id, 
                    chat_message.content, 
                    chat_message.is_user)

            chat_message_ros = ChatMessage(
                chat_id=chat_message.chat_id,
                content=chat_message_dto['content'],
                is_user=chat_message_dto['isUser'],
                message_id=chat_message_dto['messageId'],
                timestamp=chat_message_dto['timestamp'])
                
            self.chat_message_publisher.publish(chat_message_ros)
            
            



def worker_target(chat_id: str, personality: Personality, worker_to_ros: Connection):

    while True:
        play_audio(START_SIGNAL_FILE)
        user_input = speech_to_text(personality.pause_threshold)
        play_audio(STOP_SIGNAL_FILE)
        if user_input != '': worker_to_ros.send(TransientChatMessage(user_input, True, chat_id, None))
        va_response = gpt_chat(user_input, personality.description)
        worker_to_ros.send(TransientChatMessage(va_response, False, chat_id, personality.gender))
        worker_to_ros.recv()



def ros_target(ros_to_main: Connection):
    
    rclpy.init()
    node = VoiceAssistantNode(ros_to_main)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
        


def main(args=None):

    main_to_ros, ros_to_main = Pipe()

    ros_process = Process(target=ros_target, args=(ros_to_main,))
    ros_process.start()

    while True:

        personality, chat_id = main_to_ros.recv()

        ros_to_worker, worker_to_ros = Pipe()
        main_to_ros.send(ros_to_worker)
        worker_process = Process(target=worker_target, args=(chat_id, personality, worker_to_ros))    
        worker_process.start()

        print('ON')
        main_to_ros.recv()
        print('OFF')
        worker_process.terminate()



if __name__ == "__main__":
    main()
