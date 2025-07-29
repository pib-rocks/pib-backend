import asyncio, traceback
import pyaudio
from google import genai
from google.genai import types

FORMAT = pyaudio.paInt16
CHANNELS = 1
SEND_RATE = 16000
RECV_RATE = 24000
CHUNK = 1024
MODEL = "gemini-2.5-flash-preview-native-audio-dialog"
CONFIG = types.LiveConnectConfig(response_modalities=["AUDIO"])

class GeminiAudioLoop:
    def __init__(self):
        self.in_queue: asyncio.Queue[bytes] = None
        self.out_queue: asyncio.Queue[bytes] = None
        self.session = None
        self.audio_stream = None

    async def listen_audio(self):
        mic = pyaudio.PyAudio().get_default_input_device_info()
        self.audio_stream = await asyncio.to_thread(
            pyaudio.PyAudio().open,
            format=FORMAT, channels=CHANNELS, rate=SEND_RATE,
            input=True, input_device_index=mic["index"],
            frames_per_buffer=CHUNK,
        )
        kwargs = {"exception_on_overflow": False}
        while True:
            data = await asyncio.to_thread(self.audio_stream.read, CHUNK, **kwargs)
            await self.out_queue.put(data)

    async def send_realtime(self):
        while True:
            chunk = await self.out_queue.get()
            await self.session.send_realtime_input(audio=chunk)

    async def receive_audio(self):
        async for turn in self.session.receive():
            for resp in turn:
                if resp.audio:
                    self.in_queue.put_nowait(resp.audio.data)
            # flush on interruption
            while not self.in_queue.empty():
                self.in_queue.get_nowait()

    async def play_audio(self):
        stream = await asyncio.to_thread(
            pyaudio.PyAudio().open,
            format=FORMAT, channels=CHANNELS, rate=RECV_RATE, output=True
        )
        while True:
            pcm = await self.in_queue.get()
            await asyncio.to_thread(stream.write, pcm)

    async def run(self):
        client = genai.Client(api_key=None)  # reads GOOGLE_API_KEY or ADC
        try:
            async with client.aio.live.connect(model=MODEL, config=CONFIG) as session:
                self.session = session
                self.in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)
                await asyncio.gather(
                    self.listen_audio(),
                    self.send_realtime(),
                    self.receive_audio(),
                    self.play_audio(),
                )
        except asyncio.CancelledError:
            pass
        except Exception:
            if self.audio_stream:
                self.audio_stream.close()
            traceback.print_exc()
