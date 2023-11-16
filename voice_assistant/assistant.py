import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import openai
from google.cloud import texttospeech
from google.cloud import speech_v1p1beta1 as speech
import pyaudio
import wave
import os
import speech_recognition as sr
from multiprocessing import Process, Value
import time

# Set up OpenAI GPT-3 API
openai.api_key = ""

# Google Cloud API
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/pib/ros_working_dir/src/voice-assistant/voice_assistant/pibVoice.json"
client = speech.SpeechClient()

class AssistantSubscriber(Node):

    def __init__(self, start_bool, done_bool):
        super().__init__('assistant_subscriber')
        self.get_logger().info('Now running VA')
        self.subscription = self.create_subscription(String, '/cerebra_voice_settings', self.listener_callback, 10)
        self.start_bool = start_bool
        self.done_bool = done_bool

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == '{"activationFlag":true}':
            print('Assistant activate')
            self.start_bool.value = True
        elif msg.data == '{"activationFlag":false}':
            print('Assistant deactivate')
            self.done_bool.value = True
        else:
            self.get_logger().warn('Strange message from cerebra')

def start_listening(done_bool):
    count = 1
    while not done_bool.value:
        time.sleep(0.2)
        print("Say Something:")
        user_input = speech_to_text()
        gpt_response = gpt_chat(user_input)
        print("Pib:", gpt_response)
        output_file = "output.wav"
        text_to_speech(gpt_response, output_file)
        play_audio(output_file)
        print("Pib is done speaking.")
        os.remove(output_file)
        print(str(count) + '-te Aussage seit Start.')
        count += 1

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

def toggler(start_bool, done_bool):
    rclpy.init()
    subscriber_node = AssistantSubscriber(start_bool, done_bool)
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    start_bool = Value('b', False)
    done_bool = Value('b', False)

    print('toggler started')
    toggle_process = Process(target=toggler, args=(start_bool, done_bool))
    toggle_process.start()
    while True:
        start_bool.value = False
        print('Waiting to start.')
        while not start_bool.value:
            time.sleep(0.5)

        done_bool.value = False
        print('on')
        assistant_process = Process(target=start_listening, args=(done_bool,))
        assistant_process.start()

        print('Press off when done.')
        while not done_bool.value:
            time.sleep(0.5)

        assistant_process.terminate()

if __name__ == "__main__":
    main()
