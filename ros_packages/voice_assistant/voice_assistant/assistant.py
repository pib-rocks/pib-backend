from multiprocessing.connection import Connection
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from datatypes.srv import SetVoiceAssistantState, GetVoiceAssistantState
from datatypes.msg import ChatMessage, VoiceAssistantState

from openai import OpenAI
from google.cloud import texttospeech
from google.cloud import speech_v1p1beta1 as speech
import pyaudio
import wave
import os
import speech_recognition as sr
from speech_recognition import WaitTimeoutError
from multiprocessing import Process, Pipe, Lock
import time

from pib_api_client import chat_client, personality_client


RECEIVE_CHAT_MESSAGE_WAITING_PERIOD_SECONDS = 0.1
MAIN_LOOP_RECEIVE_SIGNAL_WAITING_PERIOD_SECONDS = 0.05


VOICE_ASSISTANT_PATH_PREFIX = "/home/pib/ros_working_dir/src/voice_assistant"
AUDIO_OUTPUT_FILE = VOICE_ASSISTANT_PATH_PREFIX + "/audiofiles/assistant_output.wav"
START_SIGNAL_FILE = VOICE_ASSISTANT_PATH_PREFIX + "/audiofiles/assistant_start_listening.wav"
STOP_SIGNAL_FILE = VOICE_ASSISTANT_PATH_PREFIX + "/audiofiles/assistant_stop_listening.wav"
OPENAI_KEY_PATH = VOICE_ASSISTANT_PATH_PREFIX + "/credentials/openai-key"
GOOGLE_KEY_PATH = VOICE_ASSISTANT_PATH_PREFIX + "/credentials/google-key"

pib_api_client_lock = Lock()


# Set up OpenAI GPT-3 API
with open(OPENAI_KEY_PATH) as f:
    openai_api_key = f.read().strip()
openai_client = OpenAI(api_key=openai_api_key)

# Google Cloud API
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = GOOGLE_KEY_PATH
client = speech.SpeechClient()



class Personality:

    def __init__(self, gender: str, pause_threshold: float, description: str | None = None):
        self.gender = gender
        self.pause_threshold = pause_threshold
        self.description = description if description is not None else 'Du bist pib, ein humanoider Roboter'



class TransientChatMessage:

    def __init__(self, content: str, is_user: bool, chat_id: float):
        self.content = content
        self.is_user = is_user
        self.chat_id = chat_id



class VoiceAssistantNode(Node):

    def __init__(self, chat_message_from_main: Connection, state_to_main: Connection):

        super().__init__('voice_assistant')
        self.get_logger().info('Now running VA')

        chat_message_callback_group = MutuallyExclusiveCallbackGroup()
        va_state_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.state = VoiceAssistantState(turned_on=False, chat_id='')
        self.state_lock: type[Lock] = Lock()

        self.chat_message_from_main: Connection = chat_message_from_main
        self.state_to_main: Connection = state_to_main

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
            self.timer_callback,
            callback_group=timer_callback_group
        )



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
                    self.get_logger().warn(f"voice assistant is already turned {'on' if request_state.turned_on else 'off'}.")
                    response.successful = False
                else:
                    if request_state.turned_on:
                        personality = self.get_personality_from_chat_id(request_state.chat_id)
                        if personality is not None: 
                            self.state_to_main.send((personality, request_state.chat_id))
                            self.state = VoiceAssistantState(turned_on=True, chat_id=request_state.chat_id)
                        response.successful = personality is not None
                    else:
                        self.state_to_main.send(None)
                        response.successful = True
                        self.state = VoiceAssistantState(turned_on=False, chat_id=request_state.chat_id)

            except Exception as e:
                self.get_logger().error(f"following error occured while trying to set voice assistant state: {str(e)}.")
                response.successful = False

            self.voice_assistant_state_publisher.publish(VoiceAssistantState(
                turned_on=self.state.turned_on, 
                chat_id=self.state.chat_id))
            
        return response
            


    def timer_callback(self):

        if not self.chat_message_from_main.poll(): return

        chat_message: TransientChatMessage = self.chat_message_from_main.recv()

        with pib_api_client_lock: 
            _, chat_message_dto = chat_client.create_chat_message(chat_message.chat_id, chat_message.content, chat_message.is_user)

        chat_message_ros = ChatMessage(
            chat_id=chat_message.chat_id,
            content=chat_message_dto['content'],
            is_user=chat_message_dto['isUser'],
            message_id=chat_message_dto['messageId'],
            timestamp=chat_message_dto['timestamp'])
        
        self.chat_message_publisher.publish(chat_message_ros)



def speech_to_text(pause_threshold: float) -> str:

    r = sr.Recognizer()
    r.pause_threshold = max(pause_threshold, r.non_speaking_duration)
    print('----------------------------------------------ALSA')
    with sr.Microphone() as source:
        print('----------------------------------------------')
        #r.adjust_for_ambient_noise(source) # this should not be done here
        print('Say something!')
        try: audio = r.listen(source, timeout=8)
        except WaitTimeoutError: return ''
    # Speech recognition using Google's Speech Recognition
    data = ''
    try:
        data = r.recognize_google(audio, language="de-DE")
        print('You said: ' + data)
    except sr.UnknownValueError:
        print('Google Speech Recognition could not understand')
    except sr.RequestError as e:
        print('Request error from Google Speech Recognition')
    return data



def gpt_chat(input_text: str, personality_description: str) -> str:

    response = openai_client.chat.completions.create(
        model="gpt-4-0314",
        messages=[
            {
                "role": "system",
                "content": personality_description,
            },
            {
                "role": "user",
                "content": input_text,
            },
        ]
    )

    return response.choices[0].message.content



def text_to_speech(text_input: str, gender: str) -> None:

    client = texttospeech.TextToSpeechClient()
    synthesis_input = texttospeech.SynthesisInput(text=text_input)
    voice = texttospeech.VoiceSelectionParams(
        language_code="de-DE",
        name=f"de-DE-Standard-{'A' if gender == 'Female' else 'B'}"
    )
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.LINEAR16
    )
    response = client.synthesize_speech(
        input=synthesis_input, voice=voice, audio_config=audio_config
    )
    with open(AUDIO_OUTPUT_FILE, "wb") as out:
        out.write(response.audio_content)
    os.chmod(AUDIO_OUTPUT_FILE, 0o777)
    


def play_audio(file_path: str) -> None:

    CHUNK = 1024
    wf = wave.open(file_path, 'rb')
    print('++++++++++++++++++++++++++++++++++++++ALSA')
    p = pyaudio.PyAudio()
    print('++++++++++++++++++++++++++++++++++++++')

    stream = p.open(
        format=p.get_format_from_width(wf.getsampwidth()),
        channels=wf.getnchannels(),
        rate=wf.getframerate(),
        output=True
    )

    data = wf.readframes(CHUNK)
    while data:
        stream.write(data)
        data = wf.readframes(CHUNK)

    stream.stop_stream()
    stream.close()
    p.terminate()



def worker_target(chat_id: str, personality: Personality, chat_message_to_main: Connection):

    while True:
        play_audio(START_SIGNAL_FILE)
        user_input = speech_to_text(personality.pause_threshold)
        play_audio(STOP_SIGNAL_FILE)
        if user_input != '': chat_message_to_main.send(TransientChatMessage(user_input, True, chat_id))
        va_response = gpt_chat(user_input, personality.description)
        chat_message_to_main.send(TransientChatMessage(va_response, False, chat_id))
        text_to_speech(va_response, personality.gender)
        play_audio(AUDIO_OUTPUT_FILE,)
        



def ros_target(chat_message_from_main: Connection, state_to_main: Connection):
    
    rclpy.init()
    node = VoiceAssistantNode(chat_message_from_main, state_to_main)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
        


def main(args=None):

    chat_message_to_ros, chat_message_from_main = Pipe()
    state_to_main, state_from_ros = Pipe()

    ros_process = Process(target=ros_target, args=(chat_message_from_main, state_to_main))
    ros_process.start()

    while True:

        personality, chat_id = state_from_ros.recv()

        chat_message_to_main, chat_message_from_worker = Pipe()
        worker_process = Process(target=worker_target, args=(chat_id, personality, chat_message_to_main))    
        worker_process.start()

        print('ON')

        while not state_from_ros.poll():
            while (chat_message_from_worker.poll()):
                chat_message_to_ros.send(chat_message_from_worker.recv())
            time.sleep(MAIN_LOOP_RECEIVE_SIGNAL_WAITING_PERIOD_SECONDS)
        
        print('OFF')
        
        worker_process.terminate()
        state_from_ros.recv()



if __name__ == "__main__":
    main()
