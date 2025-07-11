"""Async interface to Google's Gemini Live API."""
from __future__ import annotations

import base64
import os
from io import BytesIO
from typing import AsyncIterable, Iterable, List, Optional

from PIL import Image

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
        "response_modalities": ["AUDIO"],
        "system_instruction": description,
    }

    async with genai.Client().aio.live.connect(model=model, config=config) as session:
        for msg in message_history:
            await session.send(input=msg.content, end_of_turn=True)

        parts: list[object] = [text]
        if image_base64:
            try:
                image_data = base64.b64decode(image_base64)
                parts.append(Image.open(BytesIO(image_data)))
            except Exception:
                pass

        await session.send(input=parts if len(parts) > 1 else parts[0], end_of_turn=audio_stream is None)

        if audio_stream is not None:
            for chunk in audio_stream:
                await session.send_realtime_input(
                    audio=types.Blob(data=chunk, mime_type="audio/pcm;rate=16000")
                )
            await session.end_turn()

        async for response in session.receive():
            if response.text:
                yield response.text

