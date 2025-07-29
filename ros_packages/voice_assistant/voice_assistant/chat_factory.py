"""Factory to route chat requests to the correct provider."""
from __future__ import annotations

from typing import AsyncIterable, List, Optional

from public_api_client import public_voice_client

from .gemini_chat_async import GeminiLiveSession

_gemini_sessions: dict[str, GeminiLiveSession] = {}


async def chat_completion(
    *,
    text: str,
    description: str,
    message_history: List[public_voice_client.PublicApiChatMessage],
    public_api_token: str,
    image_base64: Optional[str] = None,
    model: str,
    chat_id: str,
) -> AsyncIterable[str]:
    """Return tokens from the selected LLM model."""
    if "gemini" in model.lower():
        session = _gemini_sessions.get(chat_id)
        if session is None:
            session = GeminiLiveSession(
                description=description,
                message_history=message_history,
                model=model,
            )
            _gemini_sessions[chat_id] = session

        async for token in session.ask(text):
            yield token
    else:
        # wrap synchronous generator in async iterator
        tokens = public_voice_client.chat_completion(
            text=text,
            description=description,
            message_history=message_history,
            image_base64=image_base64,
            model=model,
            public_api_token=public_api_token,
        )
        for token in tokens:
            yield token
