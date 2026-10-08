"""The on-device Ollama model, shared by the API and the voice assistant.

Ollama answers on the host at ``127.0.0.1:11434``. The API and the voice
assistant run in containers. Neither container has a host route today (the
motors and programs services do, via ``host.docker.internal``). Set
``PIB_OLLAMA_BASE_URL`` to that route once it exists. Until then the default
below reaches Ollama only from a process on the host network.
"""

from __future__ import annotations

import json
import os
from typing import Iterable, Iterator, Optional
from urllib.request import Request, urlopen

API_NAME = "qwen-fast"
VISUAL_NAME = "Local (qwen-fast)"
PROVIDER_NAME = "Local"
#: Capability flag so the interface can show that this model runs on the
#: device and works without a provider key or the internet.
OFFLINE_CAPABILITY = "offline"

_DEFAULT_ORIGIN = "http://127.0.0.1:11434"
_EMPTY_USER = "echo 'I could not hear you, please repeat your message.'"


def model_capabilities() -> dict[str, bool]:
    """Flags stored on the local model row.

    The five cloud flags stay off. ``offline`` is the extra marker and is
    not one of the shared provider flags.
    """
    return {
        "tools": False,
        "images": False,
        "live": False,
        "stt": False,
        "tts": False,
        OFFLINE_CAPABILITY: True,
    }


def provider_capabilities() -> dict[str, bool]:
    """Flags stored on the local provider. It has no shared cloud capability."""
    flags = model_capabilities()
    flags.pop(OFFLINE_CAPABILITY)
    return flags


def ollama_origin() -> str:
    """Base URL of the Ollama daemon, without a path."""
    configured = os.environ.get("PIB_OLLAMA_BASE_URL", "").strip()
    if configured:
        return configured.rstrip("/")
    return _DEFAULT_ORIGIN


def tags_url() -> str:
    return ollama_origin() + "/api/tags"


def openai_base_url() -> str:
    """OpenAI-compatible root, ``/v1`` on the Ollama origin."""
    return ollama_origin() + "/v1"


def chat_completions_url() -> str:
    return openai_base_url() + "/chat/completions"


def _bare_name(name: str) -> str:
    return name.split(":", 1)[0].strip()


def tags_include_local_model(payload: object) -> bool:
    """True when the tags document lists qwen-fast itself.

    The parent weights (``qwen2.5:1.5b``) do not count. A tag such as
    ``qwen-fast:latest`` does.
    """
    if not isinstance(payload, dict):
        return False
    models = payload.get("models")
    if not isinstance(models, list):
        return False
    for item in models:
        names: list[str] = []
        if isinstance(item, str):
            names.append(item)
        elif isinstance(item, dict):
            for key in ("name", "model"):
                value = item.get(key)
                if isinstance(value, str):
                    names.append(value)
        for name in names:
            if _bare_name(name) == API_NAME:
                return True
    return False


def _chat_timeout() -> float:
    raw = os.environ.get("PIB_OLLAMA_TIMEOUT", "").strip()
    if not raw:
        return 120.0
    try:
        return float(raw)
    except ValueError:
        return 120.0


def _chat_messages(
    system: str, history: Iterable[tuple[str, bool]], user_text: str
) -> list[dict[str, str]]:
    messages = [{"role": "system", "content": system}]
    for content, is_user in history:
        text = content if isinstance(content, str) else ""
        if not text.strip():
            continue
        messages.append({"role": "user" if is_user else "assistant", "content": text})
    spoken = user_text.strip() if isinstance(user_text, str) else ""
    if not spoken:
        spoken = _EMPTY_USER
    if (
        not messages
        or messages[-1]["role"] != "user"
        or messages[-1]["content"] != spoken
    ):
        messages.append({"role": "user", "content": spoken})
    return messages


def _reply_text(payload: object) -> str:
    if not isinstance(payload, dict):
        raise RuntimeError("local model returned a response that is not an object")
    choices = payload.get("choices")
    if not isinstance(choices, list) or not choices:
        raise RuntimeError("local model returned no choices")
    first = choices[0]
    if not isinstance(first, dict):
        raise RuntimeError("local model returned a choice that is not an object")
    message = first.get("message")
    content = message.get("content") if isinstance(message, dict) else None
    if not isinstance(content, str) or not content.strip():
        raise RuntimeError("local model returned an empty reply")
    return content


def iter_reply(
    system: str,
    history: Iterable[tuple[str, bool]],
    user_text: str,
    opener=None,
    timeout: Optional[float] = None,
) -> Iterator[str]:
    """One completion from the OpenAI-compatible endpoint. No provider key.

    The whole reply is one chunk. The caller still splits it into sentences.
    """
    body = json.dumps(
        {
            "model": API_NAME,
            "messages": _chat_messages(system, history, user_text),
            "stream": False,
        }
    ).encode("utf-8")
    request = Request(
        chat_completions_url(),
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    open_fn = urlopen if opener is None else opener
    limit = _chat_timeout() if timeout is None else timeout
    with open_fn(request, timeout=limit) as response:
        payload = json.loads(response.read().decode("utf-8"))
    yield _reply_text(payload)
