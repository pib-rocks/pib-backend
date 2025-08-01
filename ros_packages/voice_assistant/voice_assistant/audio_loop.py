import asyncio
import traceback
import logging
import pyaudio
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
SEND_RATE = 16000
RECV_RATE = 24000
CHUNK = 1024
MODEL = "gemini-2.5-flash-preview-native-audio-dialog"
CONFIG = types.LiveConnectConfig(response_modalities=["AUDIO"])

class GeminiAudioLoop:
    def __init__(self):
        self.in_queue = None
        self.out_queue = None
        self.session = None
        self.audio_stream = None

    async def listen_audio(self):
        logger.info("listen_audio: starting microphone capture")
        mic_info = pya.get_default_input_device_info()
        self.audio_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=SEND_RATE,
            input=True,
            input_device_index=mic_info["index"],
            frames_per_buffer=CHUNK,
        )
        kwargs = {"exception_on_overflow": False}
        try:
            while True:
                data = await asyncio.to_thread(self.audio_stream.read, CHUNK, **kwargs)
                await self.out_queue.put(data)
                logger.debug(f"listen_audio: queued {len(data)} bytes")
        except asyncio.CancelledError:
            logger.info("listen_audio: cancelled, closing stream")
            self.audio_stream.close()
            raise
        except Exception as e:
            logger.exception(f"listen_audio: error: {e}")

    async def send_realtime(self):
        logger.info("send_realtime: starting to send audio to Gemini")
        try:
            while True:
                msg = await self.out_queue.get()
                await self.session.send_realtime_input(audio=msg)
                logger.debug(f"send_realtime: sent {len(msg)} bytes as Blob")
        except asyncio.CancelledError:
            logger.info("send_realtime: cancelled")
            raise
        except Exception as e:
            logger.exception(f"send_realtime: error: {e}")

    async def receive_audio(self):
        logger.info("receive_audio: awaiting responses")
        try:
            async for turn in self.session.receive():
                for resp in turn:
                    if resp.audio and resp.audio.data:
                        self.in_queue.put_nowait(resp.audio.data)
                        logger.debug(f"receive_audio: received {len(resp.audio.data)} bytes")
                # flush any leftovers on interruption
                while not self.in_queue.empty():
                    _ = self.in_queue.get_nowait()
            logger.info("receive_audio: stream closed by server")
        except asyncio.CancelledError:
            logger.info("receive_audio: cancelled")
            raise
        except Exception as e:
            logger.exception(f"receive_audio: error: {e}")

    async def play_audio(self):
        logger.info("play_audio: starting playback")
        stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT, channels=CHANNELS, rate=RECV_RATE, output=True
        )
        try:
            while True:
                pcm = await self.in_queue.get()
                await asyncio.to_thread(stream.write, pcm)
                logger.debug(f"play_audio: played {len(pcm)} bytes")
        except asyncio.CancelledError:
            logger.info("play_audio: cancelled, closing playback stream")
            stream.close()
            raise
        except Exception as e:
            logger.exception(f"play_audio: error: {e}")

    async def run(self):
        logger.info(f"run: connecting to Gemini model {MODEL}")
        client = genai.Client(api_key="")  # uses/env GOOGLE_API_KEY or ADC
        try:
            async with client.aio.live.connect(model=MODEL, config=CONFIG) as session:
                self.session = session
                self.in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)
                logger.info("run: session established, launching tasks")
                await asyncio.gather(
                    self.listen_audio(),
                    self.send_realtime(),
                    self.receive_audio(),
                    self.play_audio(),
                )
        except asyncio.CancelledError:
            logger.info("run: cancelled, shutting down audio loop")
            raise
        except Exception as e:
            logger.exception(f"run: unexpected error: {e}")
        finally:
            if self.audio_stream:
                self.audio_stream.close()
            logger.info("run: terminated")
