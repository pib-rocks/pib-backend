# # voice_assistant/gemini_chat_async.py
# from __future__ import annotations
# import os
# import asyncio
# from typing import AsyncIterable, Iterable, List, Optional
# import pyaudio
# import traceback

# import google.genai as genai
# from google.genai import types
# from public_api_client.public_voice_client import PublicApiChatMessage

# pya = pyaudio.PyAudio()

# GOOGLE_API_KEY_ENV = "GOOGLE_API_KEY"

# FORMAT = pyaudio.paInt16
# CHANNELS = 1
# SEND_SAMPLE_RATE = 16000
# RECEIVE_SAMPLE_RATE = 24000
# CHUNK_SIZE = 1024

# client = genai.Client()  # GOOGLE_API_KEY must be set as env variable

# MODEL = "gemini-2.5-flash-preview-native-audio-dialog"
# CONFIG = {"response_modalities": ["AUDIO"]}

# class AudioLoop:
#     def __init__(self):
#         self.audio_in_queue = None
#         self.out_queue = None

#         self.session = None

#         self.audio_stream = None

#         self.receive_audio_task = None
#         self.play_audio_task = None


#     async def listen_audio(self):
#         mic_info = pya.get_default_input_device_info()
#         self.audio_stream = await asyncio.to_thread(
#             pya.open,
#             format=FORMAT,
#             channels=CHANNELS,
#             rate=SEND_SAMPLE_RATE,
#             input=True,
#             input_device_index=mic_info["index"],
#             frames_per_buffer=CHUNK_SIZE,
#         )
#         if __debug__:
#             kwargs = {"exception_on_overflow": False}
#         else:
#             kwargs = {}
#         while True:
#             data = await asyncio.to_thread(self.audio_stream.read, CHUNK_SIZE, **kwargs)
#             await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})

#     async def send_realtime(self):
#         while True:
#             msg = await self.out_queue.get()
#             await self.session.send_realtime_input(audio=msg)

#     async def receive_audio(self):
#         "Background task to reads from the websocket and write pcm chunks to the output queue"
#         while True:
#             turn = self.session.receive()
#             async for response in turn:
#                 if data := response.data:
#                     self.audio_in_queue.put_nowait(data)
#                     continue
#                 if text := response.text:
#                     print(text, end="")

#             # If you interrupt the model, it sends a turn_complete.
#             # For interruptions to work, we need to stop playback.
#             # So empty out the audio queue because it may have loaded
#             # much more audio than has played yet.
#             while not self.audio_in_queue.empty():
#                 self.audio_in_queue.get_nowait()

#     async def play_audio(self):
#         stream = await asyncio.to_thread(
#             pya.open,
#             format=FORMAT,
#             channels=CHANNELS,
#             rate=RECEIVE_SAMPLE_RATE,
#             output=True,
#         )
#         while True:
#             bytestream = await self.audio_in_queue.get()
#             await asyncio.to_thread(stream.write, bytestream)

#     async def run(self):
#         try:
#             async with (
#                 client.aio.live.connect(model=MODEL, config=CONFIG) as session,
#                 asyncio.TaskGroup() as tg,
#             ):
#                 self.session = session

#                 self.audio_in_queue = asyncio.Queue()
#                 self.out_queue = asyncio.Queue(maxsize=5)

#                 tg.create_task(self.send_realtime())
#                 tg.create_task(self.listen_audio())
#                 tg.create_task(self.receive_audio())
#                 tg.create_task(self.play_audio())
#         except asyncio.CancelledError:
#             pass
#         except ExceptionGroup as EG:
#             if self.audio_stream:
#                 self.audio_stream.close()
#             traceback.print_exception(EG)

# async def gemini_chat_completion(
#     *,
#     text: str,
#     description: str,
#     message_history: List[PublicApiChatMessage],
#     image_base64: Optional[str],
#     model: str,
#     public_api_token: str,     # still required by PIB but we read key from env
#     audio_stream: Optional[Iterable[bytes]] = None,
# ) -> AsyncIterable[str]:
#     """
#     Live, interruptible Gemini chat. Reuses one session per call.
#     If you call this again on the same session, history/context is preserved.
#     """
#     # We'll create a new session for each call here; for multi-turn you can
#     # hoist session creation out and reuse the same GeminiLiveSession.
#     loop = AudioLoop()
#     asyncio.run(loop.run())
