import asyncio
from google import genai
from google.genai import types

LIVE_MODEL = "gemini-2.5-flash-preview-native-audio-dialog"
LIVE_CONFIG = types.LiveConnectConfig(response_modalities=["AUDIO"])

async def live_chat_stream(in_queue: asyncio.Queue, out_queue: asyncio.Queue, text_queue: asyncio.Queue):
    # Initialize GenAI client
    client = genai.Client()  # GOOGLE_API_KEY env var must be set
    # Open a Live API session
    async with client.aio.live.connect(model=LIVE_MODEL, config=LIVE_CONFIG) as session:
        # Set up queues
        audio_in_q = in_queue
        audio_out_q = out_queue
        text_q = text_queue

        async def send_audio():
            while True:
                chunk = await audio_out_q.get()
                # Pack into realtime input
                await session.send_realtime_input(audio=chunk)

        async def receive_stream():
            # Receives both audio and text turns
            async for turn in session.receive():
                for resp in turn:
                    if resp.audio:
                        await audio_in_q.put(resp.audio.data)
                    if resp.text:
                        await text_q.put(resp.text.content)

        # Run both tasks concurrently
        await asyncio.gather(send_audio(), receive_stream())
