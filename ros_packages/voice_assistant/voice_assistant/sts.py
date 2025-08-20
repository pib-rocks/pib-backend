import asyncio

import os
import sys
import traceback

import pyaudio


sys.path.append("/ros2_ws")

import time
from speech_to_text import send_audio


path_to_model_audio = "audio/model/"
path_to_user_audio = "audio/user/"

model_audio_filename="model_audio_"
user_audio_filename="user_audio_"
audio_filename=".wav"

from google import genai

if sys.version_info < (3, 11, 0):
    import taskgroup, exceptiongroup

    asyncio.TaskGroup = taskgroup.TaskGroup
    asyncio.ExceptionGroup = exceptiongroup.ExceptionGroup

FORMAT = pyaudio.get_format_from_width(2)
CHANNELS = 1
SEND_SAMPLE_RATE = 16000
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024

USER_MODEL_TIME_OVERLAP=1.0

MODEL = "models/gemini-2.5-flash"

MODE = "None"

client = genai.Client(api_key='AIzaSyD4O5Us-w410pF8UVszWzyUs2pYyeDb7AM')

CONFIG = {"response_modalities": ["AUDIO"]}

pya = pyaudio.PyAudio()
class AudioLoop:
    def __init__(self):
        self.audio_in_queue = None
        self.tryb_audio_in_queue = None
        self.audio_out_queue = b""
        self.video_out_queue = None

        self.session = None

        self.send_text_task = None
        self.receive_audio_task = None
        self.play_audio_task = None
        self.model_turn = False
        self.first_model_turn = True

    async def send_text(self):
        while True:
            text = await asyncio.to_thread(
                input,
                "message > ",
            )
            if text.lower() == "q":
                break
            await self.session.send(input=text or ".", end_of_turn=True)

    async def send_data_tryb(self):
            while True:
                msg = await self.tryb_audio_in_queue.get()
                # print(msg)
                await send_audio(msg["data"], msg["sample_rate"])

    async def send_realtime(self):
        while True:
            msg = await self.out_queue.get()
            await self.session.send(input=msg)

    async def listen_audio(self):
        mic_info = pya.get_default_input_device_info()
        self.audio_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=SEND_SAMPLE_RATE,
            input=True,
            input_device_index=mic_info["index"],
            frames_per_buffer=CHUNK_SIZE,
        )
        if __debug__:
            kwargs = {"exception_on_overflow": False}
        else:
            kwargs = {}

        while True:
            data = await asyncio.to_thread(self.audio_stream.read, CHUNK_SIZE, **kwargs)
            await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})

            self.audio_out_queue += data

    async def receive_audio(self):
        """Background task to read from the websocket, play audio, and write PCM chunks to the output queue."""
        all_audio_data = b""  # Buffer to store all received audio data
        start_time = None  # To track when audio generation starts
        audio_duration = 0  # Total duration of received audio in seconds
        playback_time = 0
        while True:
            turn = self.session.receive()

            async for response in turn:        
                if model_turn := response.server_content.model_turn:   
                    self.model_turn = True
                    # Write microphone audio
                    if self.first_model_turn:                        
                        send_to_tryb = self.audio_out_queue
                        
                        if start_time is not None:
                            target_bytes = int((playback_time-USER_MODEL_TIME_OVERLAP) * CHANNELS * 2 * SEND_SAMPLE_RATE)
                            target_bytes -= target_bytes % (CHANNELS * 2)
                            send_to_tryb = send_to_tryb[target_bytes:]
                            start_time = None
                        
                        self.tryb_audio_in_queue.put_nowait({"data":send_to_tryb,"sample_rate":SEND_SAMPLE_RATE})
                        self.audio_out_queue = b""
                        self.first_model_turn = False

                if data := response.data:
                    if start_time is None:
                        # Start the timer when the first audio chunk is received
                        start_time = time.time()

                    # Append data to the buffer
                    self.audio_in_queue.put_nowait(data)
                    all_audio_data += data

                    # Estimate the duration of this chunk (CHUNK_SIZE / SAMPLE_RATE)
                    chunk_duration = len(data) / (CHANNELS * 2 * RECEIVE_SAMPLE_RATE)
                    audio_duration += chunk_duration
                    continue

                if turn_complete := response.server_content.turn_complete:
                    self.first_model_turn=True

                    if start_time is not None:
                        # Calculate total playback duration
                        end_time = time.time()
                        playback_time = end_time - start_time
                        
                        # Determine the number of bytes corresponding to playback time
                        target_bytes = int(playback_time * CHANNELS * 2 * RECEIVE_SAMPLE_RATE)
                        trimmed_audio_data = all_audio_data[:target_bytes]

                        # Send trimmed audio data TRYB queue
                        self.tryb_audio_in_queue.put_nowait({"data":trimmed_audio_data, "sample_rate":RECEIVE_SAMPLE_RATE})

                        # Reset for the next turn
                        all_audio_data = b""
                        self.model_turn = False
                        audio_duration = 0
                    continue            
                        
                if text := response.text:
                    print(text, end="")

            # If interrupted, empty the queue
            while not self.audio_in_queue.empty():
                self.audio_in_queue.get_nowait()
               
    async def play_audio(self):
        stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        while True:
            bytestream = await self.audio_in_queue.get()
            await asyncio.to_thread(stream.write, bytestream)

    async def run(self):
        try:
            async with (
                client.aio.live.connect(model=MODEL, config=CONFIG) as session,
                asyncio.TaskGroup() as tg,
            ):
                self.session = session

                self.audio_in_queue = asyncio.Queue()
                self.tryb_audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)

                send_text_task = tg.create_task(self.send_text())
                tg.create_task(self.send_realtime())
                tg.create_task(self.listen_audio())
                tg.create_task(self.send_data_tryb())
                tg.create_task(self.receive_audio())
                tg.create_task(self.play_audio())

                await send_text_task
                raise asyncio.CancelledError("User requested exit")

        except asyncio.CancelledError:
            pass
        except ExceptionGroup as EG:
            self.audio_stream.close()
            traceback.print_exception(EG)

async def main():
    main = AudioLoop()
    asyncio.run(main.run())

# We have to save temp wav file for tryb
temp_folder_path = "temp"

if __name__ == "__main__":
    if not os.path.exists(temp_folder_path):
        os.makedirs(temp_folder_path)
        print(f"Folder '{temp_folder_path}' created.")
    else:
        print(f"Folder '{temp_folder_path}' already exists.")

    main = AudioLoop()
    asyncio.run(main.run())