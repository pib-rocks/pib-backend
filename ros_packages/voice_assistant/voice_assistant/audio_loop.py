import asyncio
import traceback
import logging
import pyaudio
import threading
from typing import Any, Optional
from google import genai

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
    def __init__(self, api_key: str = "") -> None:
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._is_listening = False

        # These get initialized in run()
        self.audio_in_queue: asyncio.Queue[bytes]
        self.out_queue: asyncio.Queue[dict[str, Any]]
        self.session = None
        self.audio_stream = None
        self.playback_stream = None
        self.api_key = api_key

    @property
    def is_listening(self) -> bool:
        return self._is_listening

    def start(self) -> None:
        """Start the Gemini audio loop in a background thread."""
        if self._is_listening:
            logger.info("GeminiAudioLoop is already running.")
            return

        # clear any previous stop flag and spawn the thread
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_wrapper, daemon=True)
        self._thread.start()
        self._is_listening = True
        logger.info("GeminiAudioLoop: started")

    def stop(self, join_timeout: float = 1.0) -> None:
        """Signal the loop to stop and join the thread."""
        if not self._is_listening:
            logger.info("GeminiAudioLoop is not running.")
            return

        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=join_timeout)
        self._is_listening = False
        self._thread = None
        logger.info("GeminiAudioLoop: stopped")

    def _run_wrapper(self):
        try:
            asyncio.run(self.run())
        except Exception:
            logger.exception("GeminiAudioLoop error in thread")

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
            while not self._stop_event.is_set():
                data = await asyncio.to_thread(self.audio_stream.read, CHUNK_SIZE, **kwargs)
                await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})
        except asyncio.CancelledError:
            logger.info("listen_audio: cancelled, closing stream")
            self.audio_stream.close()
            raise
        except Exception:
            logger.exception("listen_audio error")

    async def send_realtime(self):
        try:
            while not self._stop_event.is_set():
                msg = await self.out_queue.get()
                await self.session.send_realtime_input(audio=msg)
        except asyncio.CancelledError:
            logger.info("send_realtime: cancelled")
            raise
        except Exception:
            logger.exception("send_realtime error")

    async def receive_audio(self):
        try:
            while not self._stop_event.is_set():
                turn = self.session.receive()
                async for resp in turn:
                    if data := getattr(resp, "data", None):
                        self.audio_in_queue.put_nowait(data)
                    elif text := getattr(resp, "text", None):
                        print(text, end="")
                # flush on interruptions
                while not self.audio_in_queue.empty():
                    self.audio_in_queue.get_nowait()
        except asyncio.CancelledError:
            logger.info("receive_audio: cancelled")
            raise
        except Exception:
            logger.exception("receive_audio error")

    async def play_audio(self):
        self.playback_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        try:
            while not self._stop_event.is_set():
                pcm = await self.audio_in_queue.get()
                await asyncio.to_thread(self.playback_stream.write, pcm)
        except asyncio.CancelledError:
            logger.info("play_audio: cancelled, closing playback stream")
            self.playback_stream.close()
            raise
        except Exception:
            logger.exception("play_audio error")

    async def run(self):
        client = genai.Client(api_key=self.api_key)
        try:
            async with client.aio.live.connect(model=MODEL, config=CONFIG) as session:
                self.session = session
                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)

                # spawn tasks
                tasks = [
                    asyncio.create_task(self.listen_audio()),
                    asyncio.create_task(self.send_realtime()),
                    asyncio.create_task(self.receive_audio()),
                    asyncio.create_task(self.play_audio()),
                ]

                # stop when any task errors
                done, pending = await asyncio.wait(
                    tasks, return_when=asyncio.FIRST_EXCEPTION
                )
                for t in done:
                    if t.exception():
                        raise t.exception()

        except asyncio.CancelledError:
            logger.info("GeminiAudioLoop.run: cancelled")
        except Exception:
            logger.exception("GeminiAudioLoop.run: unexpected error")
        finally:
            # cancel and clean up
            for t in locals().get("tasks", []):
                t.cancel()
            await asyncio.gather(*locals().get("tasks", []), return_exceptions=True)

            if self.audio_stream:
                self.audio_stream.close()
            if self.playback_stream:
                self.playback_stream.close()
            logger.info("GeminiAudioLoop.run: terminated")
