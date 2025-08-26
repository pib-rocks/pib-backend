import asyncio
import os
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

# ——— Audio constants (forced) ———
FORMAT = pyaudio.paInt16
CHANNELS = 1                # force mono
SEND_SAMPLE_RATE = 16000    # force 16 kHz capture
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024

MODEL = "gemini-2.5-flash-preview-native-audio-dialog"
CONFIG = {"response_modalities": ["AUDIO"]}

PREFERRED_SUBSTR = os.getenv("MIC_DEVICE", "respeaker").lower().strip()


def _list_input_devices() -> list[dict]:
    """Return a list of input-capable device info dicts."""
    devices = []
    try:
        for i in range(pya.get_device_count()):
            info = pya.get_device_info_by_index(i)
            if info.get("maxInputChannels", 0) > 0:
                devices.append(info)
    except Exception:
        logger.exception("Failed to enumerate input devices")
    return devices


def _log_device_inventory():
    """Log a compact table of input-capable devices."""
    devs = _list_input_devices()
    if not devs:
        logger.warning("No input-capable audio devices found.")
        return
    logger.info("Input device inventory (index | name | maxInCh | defaultSR):")
    for d in devs:
        logger.info(
            "  %3s | %s | %s | %s",
            d.get("index"),
            d.get("name"),
            d.get("maxInputChannels"),
            d.get("defaultSampleRate"),
        )


def _find_respeaker_device_index(preferred_substring: str) -> Optional[int]:
    """
    Try to find an input device whose name contains the preferred substring.
    Returns the device index or None.
    """
    preferred_substring = preferred_substring.lower()
    for i in range(pya.get_device_count()):
        info = pya.get_device_info_by_index(i)
        if info.get("maxInputChannels", 0) > 0:
            name = str(info.get("name", "")).lower()
            if preferred_substring in name:
                return info["index"]
    return None


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

    async def _open_input_stream(self):
        """
        Open PyAudio input stream with forced mono/16kHz on a device whose
        name contains 'respeaker' (or MIC_DEVICE env). Falls back to default.
        """
        _log_device_inventory()

        # First try preferred (ReSpeaker) device
        preferred_idx = _find_respeaker_device_index(PREFERRED_SUBSTR)
        use_idx = preferred_idx
        use_label = "preferred (match)"
        if preferred_idx is None:
            # Fall back to default input device
            try:
                default_info = pya.get_default_input_device_info()
                use_idx = default_info["index"]
                use_label = "fallback (default)"
                logger.warning(
                    "Preferred input device with '%s' not found. Falling back to default input: index=%s name=%s",
                    PREFERRED_SUBSTR,
                    default_info.get("index"),
                    default_info.get("name"),
                )
            except Exception:
                logger.exception("No default input device available")
                raise

        # Open forced mono/16kHz stream
        logger.info(
            "Opening input stream (%s): index=%s, rate=%s, channels=%s, format=paInt16",
            use_label, use_idx, SEND_SAMPLE_RATE, CHANNELS
        )
        stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,           # forced 1
            rate=SEND_SAMPLE_RATE,       # forced 16000
            input=True,
            input_device_index=use_idx,
            frames_per_buffer=CHUNK_SIZE,
        )

        # Log what we actually got (some backends coerce settings)
        try:
            actual_rate = stream._rate if hasattr(stream, "_rate") else SEND_SAMPLE_RATE
            actual_ch = stream._channels if hasattr(stream, "_channels") else CHANNELS
            logger.info("Input stream opened: actual_rate=%s, actual_channels=%s", actual_rate, actual_ch)
        except Exception:
            pass

        return stream

    async def listen_audio(self):
        self.audio_stream = await self._open_input_stream()
        kwargs = {"exception_on_overflow": False}

        try:
            while not self._stop_event.is_set():
                data = await asyncio.to_thread(self.audio_stream.read, CHUNK_SIZE, **kwargs)
                # Always PCM 16-bit mono 16kHz
                await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})
        except asyncio.CancelledError:
            logger.info("listen_audio: cancelled, closing stream")
            try:
                self.audio_stream.close()
            finally:
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
        # Playback stays at 24k mono unless you need something else
        self.playback_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        try:
            logger.info("Playback stream opened: rate=%s, channels=%s", RECEIVE_SAMPLE_RATE, CHANNELS)
        except Exception:
            pass

        try:
            while not self._stop_event.is_set():
                pcm = await self.audio_in_queue.get()
                await asyncio.to_thread(self.playback_stream.write, pcm)
        except asyncio.CancelledError:
            logger.info("play_audio: cancelled, closing playback stream")
            try:
                self.playback_stream.close()
            finally:
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
                try:
                    self.audio_stream.close()
                except Exception:
                    pass
            if self.playback_stream:
                try:
                    self.playback_stream.close()
                except Exception:
                    pass
            logger.info("GeminiAudioLoop.run: terminated")
