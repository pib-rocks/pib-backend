import json
import os
import tempfile
from google.cloud import texttospeech
from google.cloud import speech_v1p1beta1 as speech
from typing import Any
import pyaudio
import wave
import os
import speech_recognition as sr
from speech_recognition import WaitTimeoutError
from openai import OpenAI
import boto3



CREDENTIALS_PATH_PREFIX = "/home/pib/ros_working_dir/src/voice_assistant/credentials"
OPENAI_KEY_PATH = CREDENTIALS_PATH_PREFIX + "/openai-key"
GOOGLE_KEY_PATH = CREDENTIALS_PATH_PREFIX + "/google-key"
AWS_KEY_PATH = CREDENTIALS_PATH_PREFIX + "/aws-key"



# OpenAI
with open(OPENAI_KEY_PATH) as f:
    openai_api_key = f.read().strip()
openai_client = OpenAI(api_key=openai_api_key)

# Google Cloud API
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = GOOGLE_KEY_PATH
client = speech.SpeechClient()

# AWS
with open(AWS_KEY_PATH) as f:
    aws_key = json.loads(f.read().strip())
session = boto3.Session(
    aws_access_key_id=aws_key['access_key_id'],
    aws_secret_access_key=aws_key['secret_access_key'],
    region_name=aws_key['region_name'])



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



def play_audio_from_text(text: str, voice: str) -> None:

    pya = pyaudio.PyAudio()
    stream = pya.open(format=pya.get_format_from_width(width=2), channels=1, rate=16000, output=True)
    polly_client = session.client('polly')

    response = polly_client.synthesize_speech(
        VoiceId=voice,
        OutputFormat='pcm', 
        Text=text,
        Engine='neural')
    
    stream.write(response['AudioStream'].read())

    stream.stop_stream()
    stream.close()
    pya.terminate()



def play_audio_from_file(file_path: str) -> None:

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