"""Async interface to Google's Gemini Live API."""
from __future__ import annotations

import os
from typing import AsyncIterable, Iterable, List, Optional

from public_api_client.public_voice_client import PublicApiChatMessage

import google.generativeai as genai
from google.generativeai import types


GOOGLE_API_KEY_ENV = "GOOGLE_API_KEY"


def _configure_api_key() -> None:
    """Configure the API key for ``google-generativeai`` from the environment."""
    api_key = os.getenv(GOOGLE_API_KEY_ENV)
    if api_key:
        genai.configure(api_key=api_key)


def _build_history(messages: List[PublicApiChatMessage]) -> list[dict[str, object]]:
    """Convert ``PublicApiChatMessage`` objects to Gemini chat history entries."""
    return [
        {"role": "user" if m.is_user else "model", "parts": [m.content]}
        for m in messages
    ]


class GeminiLiveSession:
    """Manage a persistent Gemini Live API session."""

    def __init__(
        self,
        *,
        description: str,
        message_history: List[PublicApiChatMessage],
        image_base64: Optional[str],
        model: str,
    ) -> None:
        self.description = description
        self.message_history = message_history
        self.image_base64 = image_base64
        self.model = model
        self._session_cm: Optional[object] = None
        self._session: Optional[object] = None

    async def _ensure_session(self) -> None:
        if self._session is not None:
            return

        _configure_api_key()

        config = {
            "system_instruction": self.description,
        }

        self._session_cm = genai.Client().aio.live.connect(model=self.model, config=config)
        self._session = await self._session_cm.__aenter__()
        for msg in self.message_history:
            await self._session.send(input=msg.content, end_of_turn=True)

    async def ask(
        self, text: str, *, audio_stream: Optional[Iterable[bytes]] = None
    ) -> AsyncIterable[str]:
        await self._ensure_session()

        await self._session.send(input=text, end_of_turn=audio_stream is None)

        if audio_stream is not None:
            for chunk in audio_stream:
                await self._session.send_realtime_input(
                    audio=types.Blob(data=chunk, mime_type="audio/pcm;rate=16000")
                )
            await self._session.end_turn()

        async for response in self._session.receive():
            if response.text:
                yield response.text

    async def close(self) -> None:
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
    public_api_token: str,
    audio_stream: Optional[Iterable[bytes]] = None,
) -> AsyncIterable[str]:
    """Yield text tokens from Gemini asynchronously.

    If ``audio_stream`` is provided, its PCM chunks (16kHz, mono, 16-bit) are
    streamed to the model using the Gemini Live API. Audio responses from the
    model are ignored; only text is yielded to the caller.
    """

    _configure_api_key()

    config = {
        "system_instruction": description,
    }

    async with genai.Client().aio.live.connect(model=model, config=config) as session:
        for msg in message_history:
            await session.send(input=msg.content, end_of_turn=True)

        await session.send(input=text, end_of_turn=audio_stream is None)

        if audio_stream is not None:
            for chunk in audio_stream:
                await session.send_realtime_input(
                    audio=types.Blob(data=chunk, mime_type="audio/pcm;rate=16000")
                )
            await session.end_turn()

        async for response in session.receive():
            if response.text:
                yield response.text

