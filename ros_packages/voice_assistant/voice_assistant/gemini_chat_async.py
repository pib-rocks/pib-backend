# voice_assistant/gemini_chat_async.py
from __future__ import annotations
import os
import asyncio
from typing import AsyncIterable, Iterable, List, Optional

import google.genai as genai
from google.genai import types
from public_api_client.public_voice_client import PublicApiChatMessage

GOOGLE_API_KEY_ENV = "GOOGLE_API_KEY"

def _configure_api_key() -> None:
    key = "AIzaSyDCcTrpz2KoNOf4Y3bGPGiuLupnthweVYA"
    if not key:
        raise RuntimeError(f"{GOOGLE_API_KEY_ENV} not set")
    genai.configure(api_key=key)

def _build_history(
    messages: List[PublicApiChatMessage],
) -> list[types.ChatMessage]:
    return [
        types.ChatMessage(
            role=types.ChatRole.USER if m.is_user else types.ChatRole.ASSISTANT,
            content=m.content
        )
        for m in messages
    ]

class GeminiLiveSession:
    """Persistent Gemini Live API session with interrupt support."""
    def __init__(
        self,
        *,
        description: str,
        message_history: List[PublicApiChatMessage],
        model: str,
    ) -> None:
        self.description = description
        self.history = message_history
        self.model = model
        self._session_cm: Optional[types.LiveConnect] = None
        self._session: Optional[types.LiveSession] = None
        self._lock = asyncio.Lock()

    async def _ensure_session(self) -> None:
        if self._session is not None:
            return
        _configure_api_key()
        self._session_cm = genai.Client().aio.live.connect(
            model=self.model,
            config={
                "system_instruction": self.description,
                "response_modalities": ["AUDIO", "TEXT"]
            },
        )
        self._session = await self._session_cm.__aenter__()
        # send system prompt + history
        await self._session.send(
            types.ChatMessage(role=types.ChatRole.SYSTEM, content=self.description)
        )
        for msg in _build_history(self.history):
            await self._session.send(msg)

    async def ask(
        self,
        text: Optional[str] = None,
        *,
        audio_stream: Optional[Iterable[bytes]] = None
    ) -> AsyncIterable[str]:
        """
        Send a new user turn (text and/or audio) and yield back text tokens live.
        Stops yielding if the model is interrupted by new input.
        """
        # ensure only one turn at a time
        async with self._lock:
            await self._ensure_session()

            # send text if provided
            if text is not None:
                await self._session.send(
                    types.ChatMessage(role=types.ChatRole.USER, content=text)
                )

            # stream audio if provided
            if audio_stream is not None:
                for chunk in audio_stream:
                    await self._session.send_realtime_input(
                        audio=types.Blob(data=chunk, mime_type="audio/pcm;rate=16000")
                    )
                await self._session.send_realtime_input(audio_stream_end=True)

            # stream back the response until complete or interrupted
            async for event in self._session.receive():
                if getattr(event, "interrupted", False):
                    # model was cut off by user—end this turn
                    break
                if event.text:
                    yield event.text

    async def close(self) -> None:
        """Terminate the WebSocket and clear session state."""
        if self._session_cm is not None:
            await self._session_cm.__aexit__(None, None, None)
            self._session_cm = None
            self._session = None

async def gemini_chat_completion(
    *,
    text: str,
    description: str,
    message_history: List[PublicApiChatMessage],
    image_base64: Optional[str],
    model: str,
    public_api_token: str,     # still required by PIB but we read key from env
    audio_stream: Optional[Iterable[bytes]] = None,
) -> AsyncIterable[str]:
    """
    Live, interruptible Gemini chat. Reuses one session per call.
    If you call this again on the same session, history/context is preserved.
    """
    # We'll create a new session for each call here; for multi-turn you can
    # hoist session creation out and reuse the same GeminiLiveSession.
    session = GeminiLiveSession(
        description=description,
        message_history=message_history,
        model=model,
    )
    try:
        async for token in session.ask(text=text, audio_stream=audio_stream):
            yield token
    finally:
        await session.close()
