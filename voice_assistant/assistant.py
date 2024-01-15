from typing import Any
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
from multiprocessing import Process, Value, Pipe, Manager, Lock
import time

import json
import ctypes
from urllib import request as urllib_request, error as urllib_error


# Set up OpenAI GPT-3 API
with open("/home/pib/ros_working_dir/src/voice-assistant/openapi-key") as f:
    openai.api_key = f.read()

# Google Cloud API
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/pib/ros_working_dir/src/voice-assistant/voice_assistant/pibVoice.json"
client = speech.SpeechClient()

TURN_ON_WAITING_PERIOD_SECONDS = 0.5
WORKER_SIGNAL_WAITING_PERIOD_SECONDS = 0.5
WORKER_PROCESS_RESPONSE_WAITING_PERIOD_SECONDS = 0.1
AUDIO_OUTPUT_FILE = "output.wav"
CHAT_MESSAGE_ROUTE = "http://localhost:5000/voice-assistant/chat/%s/messages"



class VoiceAssistantNode(Node):

    def __init__(self, worker_connection):

        super().__init__('voice_assistant')
        self.get_logger().info('Now running VA')

        chat_message_callback_group = MutuallyExclusiveCallbackGroup()
        va_state_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.current_state = VoiceAssistantState()
        self.current_state.turned_on = False
        self.current_state_lock = Lock()

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



    def get_voice_assistant_state(self, request, response):

        with self.current_state_lock:
            response.voice_assistant_state.chat_id = self.current_state.chat_id
            response.voice_assistant_state.turned_on = self.current_state.turned_on
            return response
    


    def set_voice_assistant_state(self, request, response):

        request_state = request.voice_assistant_state
        
        with self.current_state_lock:
            try:
                if request_state.turned_on == self.current_state.turned_on:
                    self.get_logger().warn(f"voice assistant is already turned {'on' if request_state.turned_on else 'off'}.")
                    response.successful = False
                else:
                    self.current_state = VoiceAssistantState(turned_on=request_state.turned_on, chat_id=request_state.chat_id)
                    response.successful = True
            except Exception as e:
                self.get_logger().info(f"following error occured while trying to set voice assistant state: {str(e)}.")
                response.successful = False

            self.voice_assistant_state_publisher.publish(VoiceAssistantState(
                turned_on=self.current_state.turned_on, 
                chat_id=self.current_state.chat_id
            ))

        return response

        

    def run_on_worker(self, callback, input = "") -> (bool, str):

        self.worker_connection.send(callback)
        self.worker_connection.send(input)

        interrupted = False
        while not self.worker_connection.poll():
            with self.current_state_lock: interrupted = not self.current_state.turned_on
            if interrupted: break
            time.sleep(WORKER_SIGNAL_WAITING_PERIOD_SECONDS)
        
        self.worker_connection.send(-1)
        return interrupted, self.worker_connection.recv()
            


    def timer_callback(self):

        self.get_logger().info("Waiting to start...")
        with self.current_state_lock: 
            if not self.current_state.turned_on: return 0

        self.get_logger().info("ON")

        while True:
            current_chat_id = ""
            with self.current_state_lock: current_chat_id = self.current_state.chat_id
            interrupted, user_input = self.run_on_worker(speech_to_text)
            if interrupted: break
            if user_input != '': self.persist_and_publish_message(user_input, True, current_chat_id)
            interrupted, va_response = self.run_on_worker(gpt_chat, user_input)
            if interrupted: break
            self.persist_and_publish_message(va_response, False, current_chat_id)
            interrupted, _ = self.run_on_worker(play_audio, va_response)
            if interrupted: break

        self.get_logger().info("OFF")



    def persist_and_publish_message(self, message_content: str, is_user: bool, chat_id: str):

        try:
                # compile motor settings to UTF-8 encoded JSON string
                request_body = json.dumps({
                    'content': message_content,
                    'isUser': is_user
                }).encode('UTF-8'),
                
                # create 'PUT' request to '/motor-settings'
                request = urllib_request.Request(
                        CHAT_MESSAGE_ROUTE % chat_id,
                        method='POST',
                        data=request_body,
                        headers={ "Content-Type": "application/json" }                           
                )

                # send request to pib-api and publish response to topic
                with urllib_request.urlopen(request) as response:
                        
                        response_json = json.loads(response.read().decode('utf-8'))
                        chat_message = ChatMessage()
                        chat_message.chat_id = chat_id
                        chat_message.content = message_content
                        chat_message.is_user = is_user
                        chat_message.message_id = response_json['messageId']
                        chat_message.timestamp = response_json['timestamp']
                        self.chat_message_publisher.publish(chat_message)
                        
                return True
                
        except urllib_error.HTTPError as e: # if server response has 4XX or 5XX status code   
                self.get_logger().error(
                        f"Error sending HTTP-Request, received following response from server: \n" +
                        f"\tstatus: {e.code},\n" +
                        f"\treason: {e.reason},\n" +
                        f"\tresponse-body: {e.fp.read().decode()}" +
                        f"Request that caused the error:\n" +
                        f"\turl: {request.full_url},\n" +
                        f"\tmethod: {request.method},\n" +
                        f"\tbody: {request.data},\n" +
                        f"\theaders: {request.header_items()}"
                )
        
        except Exception as e: # if something else fails
                self.get_logger().warn(f"Error while sending HTTP-Request: {str(e)}")

        return False



def speech_to_text(input, output):

    print(f"value before: {output.value}")

    r = sr.Recognizer()
    print('----------------------------------------------ALSA')
    with sr.Microphone() as source:
        print('----------------------------------------------')
        r.adjust_for_ambient_noise(source)
        print('Say something!')
        audio = r.listen(source, phrase_time_limit=20)
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
    print(f"output value: '{output.value}'")



def gpt_chat(input, output):

    response = openai.ChatCompletion.create(
        model="gpt-4-0314",
        messages=[
            {
                "role": "system",
                "content": "Du bist pib, ein humanoider Roboter.",
            },
            {
                "role": "user",
                "content": input,
            },
        ]
    )
    output.value = response['choices'][0]['message']['content']



def play_audio(input, output):

    # generate audio file from text

    client = texttospeech.TextToSpeechClient()
    synthesis_input = texttospeech.SynthesisInput(text=input)
    voice = texttospeech.VoiceSelectionParams(
        language_code="de-DE",
        name="de-DE-Standard-A"
    )
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.LINEAR16
    )
    response = client.synthesize_speech(
        input=synthesis_input, voice=voice, audio_config=audio_config
    )
    with open(AUDIO_OUTPUT_FILE, "wb") as out:
        out.write(response.audio_content)

    # play audio from generated file

    CHUNK = 1024
    wf = wave.open(AUDIO_OUTPUT_FILE, 'rb')
    print(AUDIO_OUTPUT_FILE)
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

    def worker_target(connection):
        while True:
            callback = connection.recv()
            argument = connection.recv()
            with Manager() as manager:
                result = manager.Value(ctypes.c_wchar_p, "")
                process = Process(target=callback, args=(argument, result))
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
