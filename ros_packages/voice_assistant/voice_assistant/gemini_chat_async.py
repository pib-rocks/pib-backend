"""Async streaming interface to Google's Gemini models."""
from __future__ import annotations

import base64
import os
from io import BytesIO
from typing import AsyncIterable, List, Optional

from PIL import Image

from public_api_client.public_voice_client import PublicApiChatMessage

import google.generativeai as genai


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
) -> AsyncIterable[str]:
    """Yield text tokens from Gemini asynchronously."""

    _configure_api_key()

    history = _build_history(message_history)
    model_obj = genai.GenerativeModel(model, system_instruction=description)
    chat = model_obj.start_chat(history=history)

    parts: list[object] = [text]
    if image_base64:
        try:
            image_data = base64.b64decode(image_base64)
            parts.append(Image.open(BytesIO(image_data)))
        except Exception:
            # If image decoding fails, fall back to text only
            pass

    response = await chat.send_message_async(parts, stream=True)
    async for chunk in response:
        if chunk.text:
            yield chunk.text
