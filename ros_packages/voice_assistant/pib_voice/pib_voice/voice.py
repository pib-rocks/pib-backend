import json
import logging
import os
import time

import requests
from google.cloud import speech_v1p1beta1 as speech
from typing import Tuple
import pyaudio
import wave
import os
import re
import speech_recognition as sr
from speech_recognition import WaitTimeoutError
from openai import OpenAI
import boto3
import numpy as np

logging.basicConfig(level=logging.INFO,
                    format="[%(levelname)s] [%(asctime)s] [%(process)d] [%(filename)s:%(lineno)s]: %(message)s")

ROS_WORKING_DIR = os.getenv("ROS_WORKING_DIR", "/home/pib/ros_working_dir")
VOICE_ASSISTANT_DIRECTORY = os.getenv("VOICE_ASSISTANT_DIR", "/home/pib/ros_working_dir/src/voice_assistant")

USER_AUDIO_INPUT_FILENAME = "UserInput.wav"
AUDIO_INPUT_FILE = VOICE_ASSISTANT_DIRECTORY + "/audiofiles/" + USER_AUDIO_INPUT_FILENAME

CREDENTIALS_DIRECTORY = VOICE_ASSISTANT_DIRECTORY + "/credentials"
OPENAI_KEY_PATH = CREDENTIALS_DIRECTORY + "/openai-key"
GOOGLE_KEY_PATH = CREDENTIALS_DIRECTORY + "/google-key"
AWS_KEY_PATH = CREDENTIALS_DIRECTORY + "/aws-key"

# Record audio settings
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 1024

# Docker containers require enviromnent variables instead of hard-coded file paths
try:
    # Set up OpenAI GPT-3 API
    with open(OPENAI_KEY_PATH) as f:
        openai_api_key = f.read().strip()
    openai_client = OpenAI(api_key=openai_api_key)

    # Google Cloud API
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = GOOGLE_KEY_PATH
    client = speech.SpeechClient()

    # AWS
    with open(AWS_KEY_PATH) as f:
        aws_key = json.loads(f.read().strip())

except Exception:
    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = os.getenv("GOOGLE_APPLICATION_CREDENTIALS")
    client = speech.SpeechClient()
    aws_key = os.getenv("AWS_KEY_CREDENTIALS")

# AWS
with open(AWS_KEY_PATH) as f:
    aws_key = json.loads(f.read().strip())

session = boto3.Session(
    aws_access_key_id=aws_key['access_key_id'],
    aws_secret_access_key=aws_key['secret_access_key'],
    region_name=aws_key['region_name'])


def speech_to_text(pause_threshold: float, silence_threshold: int) -> str:
    logging.info("start recording")
    start_recording(pause_threshold, silence_threshold)
    data = ''
    try:
        logging.info('convert audio file into text')
        audio_file = open(AUDIO_INPUT_FILE, "rb")
        data = openai_client.audio.transcriptions.create(
            model="whisper-1",
            file=audio_file
        )
        logging.info("you sad: " + data.text)
    except Exception as e:
        logging.error(f"OpenAIError: {e}")
    return data.text


def start_recording(max_silence_seconds, silence_threshold):
    # Audiosettings for record
    def is_silent(data_chunk, threshold):
        """Check whether a frame is below the minimum volume threshold"""
        as_ints = np.frombuffer(data_chunk, dtype=np.int16)
        return np.abs(as_ints).mean() < threshold

    audio = pyaudio.PyAudio()
    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    frames = []
    silent_frames = 0
    while True:
        data = stream.read(CHUNK, exception_on_overflow=False)
        frames.append(data)
        if is_silent(data, silence_threshold):
            silent_frames += 1
            if (silent_frames > 800):
                break
        else:
            silent_frames = 0
        if silent_frames >= max_silence_seconds * RATE / CHUNK:
            logging.info("silence recgonized, stopping recording")
            break
    # Beenden der Aufnahme
    stream.stop_stream()
    stream.close()
    audio.terminate()
    # Speichern der Aufnahme in einer WAV-Datei
    wave_file = wave.open(AUDIO_INPUT_FILE, 'wb')
    wave_file.setnchannels(CHANNELS)
    wave_file.setsampwidth(audio.get_sample_size(FORMAT))
    wave_file.setframerate(RATE)
    wave_file.writeframes(b''.join(frames))
    wave_file.close()
    logging.info("saving audio recording")


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


def llava_chat(input_text: str, personality_description: str, image_base64: str) -> Tuple[str, bool]:
    LLAVA_URL = os.getenv("LLAVA_URL")
    request = {
            "model": "llava",
            "messages": [
                {
                    "role": "system",
                    "content": personality_description
                },
                {
                    "role": "user",
                    "content": input_text,
                    "images": [image_base64]
                }
            ]
        }
    s = requests.Session()
    sentence_boundary = re.compile(r"[^\d | ^A-Z][\.|!|\?|:]")
    cur_sentence = ""
    prev_sentence = ""

    # Always previous sentence is returned, to be able to mark the final sentence
    with s.post(LLAVA_URL, json=request, headers=None, stream=True) as resp:
        for line in resp.iter_lines():
            if not line:
                break
            current_token = json.loads(line)["message"]["content"]
            current_token = current_token.replace("\n", " ")
            cur_sentence += current_token
            if sentence_boundary.search(cur_sentence):
                if prev_sentence != "":
                    yield prev_sentence, False
                prev_sentence = cur_sentence.strip()
                cur_sentence = ""
        yield prev_sentence, True


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
