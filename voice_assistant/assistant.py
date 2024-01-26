from multiprocessing.connection import Connection
from typing import Any, Callable
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from datatypes.srv import SetVoiceAssistantState, GetVoiceAssistantState
from datatypes.msg import ChatMessage, VoiceAssistantState

import openai
from google.cloud import texttospeech
from google.cloud import speech_v1p1beta1 as speech
import pyaudio
import wave
import os
import speech_recognition as sr
from speech_recognition import WaitTimeoutError
from multiprocessing import Process, Value, Pipe, Manager, Lock
import time

import ctypes

from pib_api_client import chat_client, personality_client



TURN_ON_WAITING_PERIOD_SECONDS = 0.5
WORKER_SIGNAL_WAITING_PERIOD_SECONDS = 0.1
WORKER_PROCESS_RESPONSE_WAITING_PERIOD_SECONDS = 0.1

AUDIO_OUTPUT_FILE = "/home/pib/ros_working_dir/src/voice-assistant/output.wav"
OPENAI_KEY_PATH = "/home/pib/ros_working_dir/src/voice-assistant/credentials/openai-key"
GOOGLE_KEY_PATH = "/home/pib/ros_working_dir/src/voice-assistant/credentials/google-key"

pib_api_client_lock = Lock()


# Set up OpenAI GPT-3 API
with open(OPENAI_KEY_PATH) as f:
    openai.api_key = f.read().strip()

# Google Cloud API
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = GOOGLE_KEY_PATH
client = speech.SpeechClient()



class InternalState:
    
    def __init__(self, turned_on, chat_id = ''):
        self.turned_on = turned_on
        self.chat_id = chat_id
        if turned_on:
            with pib_api_client_lock:
                successful, chat_dto = chat_client.get_chat(chat_id)
                if not successful: raise Exception("failed to retrieve chat from pib-api")
                successful, personality_dto = personality_client.get_personality(chat_dto['personalityId'])
                if not successful: raise Exception("failed to retrieve personlaity from pib-api")
            self.description = personality_dto['description']
            if self.description is None: self.description = 'Du bist pib, ein humanoider Roboter'
            self.pause_threshold = personality_dto['pauseThreshold']
            self.gender = personality_dto['gender']



class VoiceAssistantNode(Node):

    def __init__(self, worker_connection):

        super().__init__('voice_assistant')
        self.get_logger().info('Now running VA')

        chat_message_callback_group = MutuallyExclusiveCallbackGroup()
        va_state_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.internal_state = InternalState(False)
        self.internal_state_lock = Lock()

        self.worker_connection = worker_connection

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
            TURN_ON_WAITING_PERIOD_SECONDS, 
            self.timer_callback,
            callback_group=timer_callback_group
        )



    def get_voice_assistant_state(self, _, response: GetVoiceAssistantState.Response):

        with self.internal_state_lock:
            response.voice_assistant_state.chat_id = self.internal_state.chat_id
            response.voice_assistant_state.turned_on = self.internal_state.turned_on

        return response
    


    def set_voice_assistant_state(self, request: SetVoiceAssistantState.Request, response: SetVoiceAssistantState.Response):

        request_state = request.voice_assistant_state
        with self.internal_state_lock:
            
            try:
                if request_state.turned_on == self.internal_state.turned_on:
                    self.get_logger().warn(f"voice assistant is already turned {'on' if request_state.turned_on else 'off'}.")
                    response.successful = False
                else:
                    self.internal_state = InternalState(request_state.turned_on, request_state.chat_id)
                    response.successful = True

            except Exception as e:
                self.get_logger().error(f"following error occured while trying to set voice assistant state: {str(e)}.")
                response.successful = False

            self.voice_assistant_state_publisher.publish(VoiceAssistantState(
                turned_on=self.internal_state.turned_on, 
                chat_id=self.internal_state.chat_id))

        return response

        

    def run_on_worker(self, callback: Callable, input: tuple[Any] = ()) -> (bool, str):

        self.worker_connection.send(callback)
        self.worker_connection.send(input)

        interrupted = False
        while not self.worker_connection.poll():
            with self.internal_state_lock: interrupted = not self.internal_state.turned_on
            if interrupted: break
            time.sleep(WORKER_SIGNAL_WAITING_PERIOD_SECONDS)
        
        self.worker_connection.send(-1)
        return interrupted, self.worker_connection.recv()
            


    def timer_callback(self):

        self.get_logger().info("Waiting to start...")
        with self.internal_state_lock: 
            if not self.internal_state.turned_on: return 0

        self.get_logger().info("ON")

        while True:

            with self.internal_state_lock: 
                current_chat_id = self.internal_state.chat_id
                current_description = self.internal_state.description
                current_pause_threshold = self.internal_state.pause_threshold
                gender = self.internal_state.gender

            interrupted, user_input = self.run_on_worker(speech_to_text, (current_pause_threshold,))
            if interrupted: break
            if user_input != '': self.persist_and_publish_message(user_input, True, current_chat_id)
            interrupted, va_response = self.run_on_worker(gpt_chat, (user_input, current_description))
            if interrupted: break
            self.persist_and_publish_message(va_response, False, current_chat_id)
            interrupted, _ = self.run_on_worker(play_audio, (va_response, gender))
            if interrupted: break

        self.get_logger().info("OFF")



    def persist_and_publish_message(self, message_content: str, is_user: bool, chat_id: str) -> bool:

        with pib_api_client_lock: 
            successful, chat_message_dto = chat_client.create_chat_message(chat_id, message_content, is_user)

        if not successful: return False

        chat_message_ros = ChatMessage(
            chat_id=chat_id,
            content=message_content,
            is_user=is_user,
            message_id=chat_message_dto['messageId'],
            timestamp=chat_message_dto['timestamp'])
        
        self.chat_message_publisher.publish(chat_message_ros)
        return True



def speech_to_text(pause_threshold: float, output: type[Value]) -> None:

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
    output.value = data



def gpt_chat(input_text: str, personality_description: str, output: type[Value]) -> None:

    response = openai.ChatCompletion.create(
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
    output.value = response['choices'][0]['message']['content']



def play_audio(text_input: str, gender: str, output: type[Value]) -> None:

    # generate audio file from text

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
    

    # play audio from generated file

    CHUNK = 1024
    wf = wave.open(AUDIO_OUTPUT_FILE, 'rb')
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
        


def main(args=None):

    def worker_target(connection: Connection):
        while True:
            callback: Callable = connection.recv()
            input_arguments: tuple[Any] = connection.recv()
            with Manager() as manager:
                result = manager.Value(ctypes.c_wchar_p, "")
                arguments = input_arguments + (result,)
                process = Process(target=callback, args=arguments)
                process.start()
                while process.is_alive() and not connection.poll(): 
                    time.sleep(WORKER_SIGNAL_WAITING_PERIOD_SECONDS)
                process.terminate()
                process.join()
                connection.send(result.value)
                connection.recv()

    parent_connection, child_connection = Pipe()
    worker = Process(target=worker_target, args=(child_connection,))
    worker.start()
    
    rclpy.init()
    node = VoiceAssistantNode(parent_connection)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
