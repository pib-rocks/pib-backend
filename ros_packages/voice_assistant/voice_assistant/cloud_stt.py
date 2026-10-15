"""Robot client for the sold pib.Cloud STT endpoint (PC-44).

One WAV POST, no provider credential or bearer token, and a total async deadline
covering connect, upload and response reading. No retry or redirect can send
recordings to a second destination. Error messages contain metadata only.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import time
import uuid
from dataclasses import dataclass
from urllib.parse import urlsplit

import httpx

DEFAULT_BASE = "http://192.168.1.252:8090/v1"
MAX_RESPONSE_BYTES = 65536


@dataclass(frozen=True)
class CloudSTTResult:
    text: str
    request_id: str
    duration_ms: float
    http_status: int


class CloudSTTError(RuntimeError):
    def __init__(self, cause, request_id, duration_ms, http_status=None):
        super().__init__(cause)
        self.cause = cause
        self.request_id = request_id
        self.duration_ms = duration_ms
        self.http_status = http_status


def speech_to_text_url() -> str:
    base = os.getenv("PIB_CLOUD_BASE_URL", DEFAULT_BASE).strip().rstrip("/")
    try:
        parsed = urlsplit(base)
        valid = (
            parsed.scheme in ("http", "https")
            and bool(parsed.hostname)
            and not parsed.username
            and not parsed.password
            and not parsed.query
            and not parsed.fragment
            and parsed.path in ("", "/v1")
        )
        if os.getenv("PIB_CLOUD_MODE", "test") == "production":
            valid = valid and parsed.scheme == "https"
        _ = parsed.port
    except ValueError:
        valid = False
    if not valid:
        raise ValueError("invalid_cloud_stt_url")
    return (
        base
        + ("" if parsed.path == "/v1" else "/v1")
        + "/voice-assistant/speech-to-text"
    )


def request_timeout() -> float:
    try:
        value = float(os.getenv("PIB_CLOUD_STT_TIMEOUT_SECONDS", "3"))
    except ValueError:
        raise ValueError("invalid_cloud_stt_timeout") from None
    if not math.isfinite(value) or value <= 0:
        raise ValueError("invalid_cloud_stt_timeout")
    return value


def transcribe(wav_data: bytes, *, language=None) -> CloudSTTResult:
    started = time.monotonic()
    request_id = str(uuid.uuid4())
    status = None

    def error(cause):
        return CloudSTTError(
            cause, request_id, (time.monotonic() - started) * 1000, status
        )

    try:
        endpoint, timeout = speech_to_text_url(), request_timeout()
    except ValueError as exc:
        raise error(str(exc)) from None

    async def post():
        nonlocal status
        # Per-call client deliberately avoids a cross-thread loop and orphaned
        # requests. The cloud service owns the persistent provider HTTP/2 pool.
        async with httpx.AsyncClient(
            trust_env=False, follow_redirects=False, timeout=None
        ) as client:
            async with client.stream(
                "POST",
                endpoint,
                files={"file": ("speech.wav", wav_data, "audio/wav")},
                data={"language": language} if language else {},
                headers={"X-Request-ID": request_id},
            ) as response:
                status = response.status_code
                if status != 200:
                    raise error("cloud_http_error")
                payload = bytearray()
                async for chunk in response.aiter_bytes():
                    if len(payload) + len(chunk) > MAX_RESPONSE_BYTES:
                        raise error("cloud_response_too_large")
                    payload.extend(chunk)
                try:
                    body = json.loads(payload)
                except (ValueError, UnicodeError):
                    raise error("cloud_invalid_json") from None
                if not isinstance(body, dict) or not isinstance(body.get("text"), str):
                    raise error("cloud_invalid_text")
                text = body["text"].strip()
                if not text:
                    raise error("cloud_empty_text")
                return text

    async def bounded():
        return await asyncio.wait_for(post(), timeout)

    try:
        text = asyncio.run(bounded())
    except CloudSTTError:
        raise
    except (TimeoutError, httpx.TimeoutException):
        raise error("cloud_timeout") from None
    except Exception:
        # A transport exception may include credentials, URLs or response data.
        raise error("cloud_connection_error") from None
    return CloudSTTResult(text, request_id, (time.monotonic() - started) * 1000, status)
