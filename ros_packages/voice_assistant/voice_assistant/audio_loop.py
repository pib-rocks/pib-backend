import asyncio
import traceback
import logging
import pyaudio
import threading
from google import genai
from google.genai import types
# ——— Configure logging ———
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("GeminiAudioLoop")

pya = pyaudio.PyAudio()

FORMAT = pyaudio.paInt16
CHANNELS = 1
SEND_SAMPLE_RATE = 16000
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024
MODEL = "gemini-2.5-flash-preview-native-audio-dialog"
CONFIG = {"response_modalities": ["AUDIO"]}
class GeminiAudioLoop:
    def __init__(self, stop_event: threading.Event):
        self.audio_in_queue = None
        self.out_queue = None
        self.session = None
        self.audio_stream = None
        self.stop_event = stop_event

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
        kwargs = {"exception_on_overflow": False}
        try:
            while not self.stop_event.is_set():
                data = await asyncio.to_thread(self.audio_stream.read, CHUNK_SIZE, **kwargs)
                await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})
        except asyncio.CancelledError:
            logger.info("listen_audio: cancelled, closing stream")
            self.audio_stream.close()
            raise
        except Exception as e:
            logger.exception(f"listen_audio error: {e}")

    async def send_realtime(self):
        try:
            while not self.stop_event.is_set():
                msg = await self.out_queue.get()
                await self.session.send_realtime_input(audio=msg)
        except asyncio.CancelledError:
            logger.info("send_realtime: cancelled")
            raise
        except Exception as e:
            logger.exception(f"send_realtime error: {e}")

    async def receive_audio(self):
        try:
            while not self.stop_event.is_set():
                turn = self.session.receive()
                async for resp in turn:
                    if data := resp.data:
                        self.audio_in_queue.put_nowait(data)
                    elif text := resp.text:
                        print(text, end="")
                # clear queue on interruptions
                while not self.audio_in_queue.empty():
                    self.audio_in_queue.get_nowait()
        except asyncio.CancelledError:
            logger.info("receive_audio: cancelled")
            raise
        except Exception as e:
            logger.exception(f"receive_audio error: {e}")

    async def play_audio(self):
        stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        try:
            while not self.stop_event.is_set():
                pcm = await self.audio_in_queue.get()
                await asyncio.to_thread(stream.write, pcm)
        except asyncio.CancelledError:
            logger.info("play_audio: cancelled, closing playback stream")
            stream.close()
            raise
        except Exception as e:
            logger.exception(f"play_audio error: {e}")

    async def close(self):
        self.audio_stream.close()

    async def run(self):
        client = genai.Client(api_key="")  # or let it pick up ADC
        try:
            async with client.aio.live.connect(model=MODEL, config=CONFIG) as session:
                self.session = session
                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)

                # start all four subtasks and wait for any to error/cancel
                async with asyncio.TaskGroup() as tg:
                    tg.create_task(self.listen_audio())
                    tg.create_task(self.send_realtime())
                    tg.create_task(self.receive_audio())
                    tg.create_task(self.play_audio())
        except asyncio.CancelledError:
            logger.info("GeminiAudioLoop.run: cancelled, shutting down")
            pass
        except Exception as eg:
            logger.exception("GeminiAudioLoop.run: unexpected error")
            traceback.print_exception(eg)
        finally:
            if self.audio_stream:
                self.audio_stream.close()
            logger.info("GeminiAudioLoop.run: terminated")
