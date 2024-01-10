from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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
from multiprocessing import Process, Value
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

WAITING_PERIOD_SECONDS = 0.5
AUDIO_OUTPUT_FILE = "output.wav"
CHAT_MESSAGE_ROUTE = "http://localhost:5000/voice-assistant/chat/%s/messages"



class AssistantSubscriber(Node):

    def __init__(self, start_bool, done_bool, chat_id):

        super().__init__('assistant_subscriber')
        self.get_logger().info('Now running VA')
        self.start_bool = start_bool
        self.done_bool = done_bool
        self.chat_id = chat_id

        chat_message_callback_group = MutuallyExclusiveCallbackGroup()
        va_state_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

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
            WAITING_PERIOD_SECONDS, 
            self.timer_callback,
            callback_group=timer_callback_group
        )



    def get_voice_assistant_state(self, request, response):

        response.voice_assistant_state.chat_id = self.chat_id.value
        response.voice_assistant_state.turned_on = self.start_bool.value
        return response
    


    def set_voice_assistant_state(self, request, response):

        response.successful = True
        try:
            if request.voice_assistant_state.turned_on: self.start_bool.value = True
            else: self.done_bool.value = True 
            self.chat_id.value = request.voice_assistant_state.chat_id
            self.voice_assistant_state_publisher.publish(request.voice_assistant_state)
        except Exception as e:
            self.get_logger().info(f"following error occured while trying to set voice assistant state: {str(e)}.")
            response.successful = False

        return response



    def timer_callback(self):

        self.get_logger().info("Waiting to start...")
        if not self.start_bool.value: return 0

        self.get_logger().info("on")
        while not self.done_bool.value:
            current_chat_id = self.chat_id.value
            input_received, user_input = self.get_user_input()
            if not input_received: continue
            self.persist_and_publish_message(user_input, True, current_chat_id)
            response_received, va_response = self.get_va_response(user_input)
            if not response_received: continue
            self.persist_and_publish_message(va_response, False, current_chat_id)
            self.play_audio(va_response)

        self.get_logger().info("turning off voice-assistant")
        self.start_bool.value = False
        self.done_bool.value = False


    
    def get_user_input(self) -> (bool, str):

        return True, speech_to_text()



    def get_va_response(self, user_input) -> (bool, str):

        return True, gpt_chat(user_input)



    def play_audio(self, text):

        text_to_speech(text, AUDIO_OUTPUT_FILE)
        play_audio(AUDIO_OUTPUT_FILE)
    


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



def speech_to_text():

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
    return data




def gpt_chat(prompt):

    response = openai.ChatCompletion.create(
        model="gpt-4-0314",
        messages=[
            {
                "role": "system",
                "content": "Du bist pib, ein humanoider Roboter.",
            },
            {
                "role": "user",
                "content": prompt,
            },
        ]
    )
    return response['choices'][0]['message']['content']




def text_to_speech(text, output_file):

    client = texttospeech.TextToSpeechClient()
    synthesis_input = texttospeech.SynthesisInput(text=text)
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
    with open(output_file, "wb") as out:
        out.write(response.audio_content)



def play_audio(file_path):

    CHUNK = 1024
    wf = wave.open(file_path, 'rb')
    print(file_path)
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

    start_bool = Value(ctypes.c_bool, False)
    done_bool = Value(ctypes.c_bool, False)
    chat_id = Value(ctypes.c_wchar_p, "")
    
    rclpy.init()
    node = AssistantSubscriber(start_bool, done_bool, chat_id)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
