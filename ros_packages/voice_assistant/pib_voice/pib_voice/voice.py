import json
import os
import time
from google.cloud import speech_v1p1beta1 as speech
from typing import Any, Tuple
import pyaudio
import wave
import os
import re
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
    # wait for a brief period of time before closing the stream. Otherwise end of audio is cut off,
    # since polly does not insert any silence at the end of the returned audio. Alternatively,
    # ssml could be used to instruct polly to insert silence at the end of the audio
    # f.e.: '<speak>some-text<break time=100ms/></speak>'
    time.sleep(0.1)

    stream.stop_stream()
    stream.close()
    pya.terminate()



def play_audio_from_file(file_path: str) -> None:

    CHUNK = 1024
    wf = wave.open(file_path, 'rb')
    print('++++++++++++++++++++++++++++++++++++++ALSA')
    pya = pyaudio.PyAudio()
    print('++++++++++++++++++++++++++++++++++++++')

    stream = pya.open(
        format=pya.get_format_from_width(wf.getsampwidth()),
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
    pya.terminate()



def gpt_chat(input_text: str, personality_description: str) -> Tuple[str, bool]:
    """
    @return: str - sentence of streaming response
    @return: bool - whether current sentence is the final sentence of streaming response
    """
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
        ],
        stream=True
    )
    # Regex pattern matches for:
    # Any string that contains a lower-case character followed by !?.:
    # Numbers (e.g., enumerations 1. [...] or dates) and upper-case characters (e.g., abbreviations S.P.Q.R) get ignored
    sentence_boundary = re.compile(r"[^\d | ^A-Z][\.|!|\?|:]")
    cur_sentence = ""
    prev_sentence = ""
    
    # Always previous sentence is returned, to be able to mark the final sentence
    for stream in response:
        current_token = stream.choices[0].delta.content
        if current_token is None:
            break

        current_token = current_token.replace("\n", " ")
        cur_sentence += current_token
        if sentence_boundary.search(cur_sentence):
            if prev_sentence != "":
                yield prev_sentence, False
            prev_sentence = cur_sentence.strip()
            cur_sentence = ""
    yield prev_sentence, True