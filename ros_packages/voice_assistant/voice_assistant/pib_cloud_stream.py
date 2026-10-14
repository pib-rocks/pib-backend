"""pib.Cloud Stage 1 SSE streaming client (Confluence 1171390467, PC-12).

Consumes ``POST /v1/chat/completions`` with ``stream: true`` over a persistent
HTTP keep-alive session, yields visible ``delta.content`` tokens incrementally
as frames arrive, ignores ``delta.reasoning`` / control frames, authenticates
with the existing key-store row 4 (``pib.Cloud``) bearer token, and applies
the deterministic error/budget fallbacks defined in the pib-side guide.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import threading
import time
from typing import Any, Callable, Iterable, Iterator, Mapping, Optional, Sequence

import requests

from voice_assistant.cloud_token import CloudTokenError, resolve_cloud_token
from voice_assistant.turn_latency import format_event

logger = logging.getLogger(__name__)

# ChatNode installs this. Until then emissions are a no-op and cannot delay a turn.
_trace_emitter: Optional[Callable[[str], None]] = None

_TRACE_HEADER_FIELDS = (
    ("X-Upstream-Ttft-Ms", "upstream_ttft_ms"),
    ("X-Usage-Prompt-Tokens", "usage_prompt_tokens"),
    ("X-Usage-Completion-Tokens", "usage_completion_tokens"),
    ("X-Budget-Remaining-Tokens", "budget_remaining_tokens"),
)


def set_trace_emitter(fn: Optional[Callable[[str], None]]) -> None:
    """Register the chat node's ``/voice_turn_trace`` publisher. None clears it."""
    global _trace_emitter
    _trace_emitter = fn


def trace_emit(
    stage: str,
    *,
    chat_id: str = "",
    detail: Optional[Mapping[str, Any]] = None,
) -> None:
    """Publish one stage. A failure is a warning and never leaves this function."""
    emitter = _trace_emitter
    if emitter is None:
        return
    try:
        emitter(
            format_event(
                stage,
                source="pib-cloud",
                chat_id=chat_id,
                detail=detail,
            )
        )
    except Exception:
        logger.warning("turn trace emit failed for stage %s", stage)


def _header_value(headers: Any, name: str) -> Any:
    if headers is None:
        return None
    try:
        getter = getattr(headers, "get", None)
        if getter is not None:
            value = getter(name)
            if value not in (None, ""):
                return value
    except Exception:
        return None
    try:
        for key, value in headers.items():
            if str(key).lower() == name.lower() and value not in (None, ""):
                return value
    except Exception:
        return None
    return None


def _header_number(value: Any) -> Any:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value
    text = str(value).strip()
    if not text:
        return None
    try:
        if "." in text:
            return float(text)
        return int(text)
    except ValueError:
        return None


def _response_header_detail(response: Any) -> Optional[dict[str, Any]]:
    """API timing and usage headers. Missing headers are left out, never zeroed."""
    headers = getattr(response, "headers", None)
    detail: dict[str, Any] = {}
    for header_name, field in _TRACE_HEADER_FIELDS:
        parsed = _header_number(_header_value(headers, header_name))
        if parsed is not None:
            detail[field] = parsed
    return detail or None


DEFAULT_PIB_CLOUD_BASE_URL = "http://192.168.1.252:8090/v1"
PIB_CLOUD_BASE_URL_ENV = "PIB_CLOUD_BASE_URL"
DEFAULT_TIMEOUT_SECONDS = 60.0
DEFAULT_RETRY_DELAY_SECONDS = 0.5

BUDGET_EXHAUSTED_NOTICE = (
    "Dein pib.Cloud-Budget ist aufgebraucht. Ich antworte dir lokal weiter."
)
STREAM_CUTOFF_NOTICE = "Leider wurde die Verbindung unterbrochen."
CLOUD_AUTH_FAILED_NOTICE = (
    "Der pib.Cloud-Zugangsschlüssel ist ungültig oder widerrufen. "
    "Bitte prüfe die Einstellungen."
)
CLOUD_UNAVAILABLE_NOTICE = (
    "pib.Cloud ist gerade nicht erreichbar. Bitte versuche es gleich noch einmal."
)
EMPTY_CONTENT_NOTICE = "pib.Cloud hat keine sprechbare Antwort geliefert."

_EMPTY_USER_FALLBACK = "echo 'I could not hear you, please repeat your message.'"
_REASONING_CHANNELS = frozenset({"reasoning", "analysis", "thinking"})
_RETRYABLE_STATUS_CODES = frozenset({429, 502, 504})
_AUTH_ERROR_STATUS_CODES = frozenset({401, 403})

_http_session: Optional[requests.Session] = None
_http_session_lock = threading.Lock()
_revoked_token_hashes: set[str] = set()
_revoked_lock = threading.Lock()


class _SSEStreamError(Exception):
    """Raised when the SSE stream carries an error frame or terminates early."""

    def __init__(
        self,
        message: str,
        *,
        code: Optional[str] = None,
        status: Optional[int] = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.status = status


def pib_cloud_base_url(endpoint_base: Optional[str] = None) -> str:
    """OpenAI-compatible base URL for pib.Cloud (``.../v1``)."""
    if isinstance(endpoint_base, str) and endpoint_base.strip():
        return endpoint_base.strip().rstrip("/")
    configured = os.environ.get(PIB_CLOUD_BASE_URL_ENV, "").strip()
    if configured:
        return configured.rstrip("/")
    return DEFAULT_PIB_CLOUD_BASE_URL


def chat_completions_url(endpoint_base: Optional[str] = None) -> str:
    """Full URL for ``POST /v1/chat/completions``."""
    base = pib_cloud_base_url(endpoint_base)
    if base.endswith("/chat/completions"):
        return base
    if not base.endswith("/v1"):
        base = base + "/v1"
    return base + "/chat/completions"


def _get_http_session() -> requests.Session:
    """Singleton requests.Session so turns reuse the keep-alive connection."""
    global _http_session
    with _http_session_lock:
        if _http_session is None:
            session = requests.Session()
            session.headers.update({"Connection": "keep-alive"})
            _http_session = session
        return _http_session


def reset_cloud_auth_state() -> None:
    """Clear cached revoked-token markers (used in tests or after key rotation)."""
    with _revoked_lock:
        _revoked_token_hashes.clear()


def _token_fingerprint(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _is_token_revoked(token: str) -> bool:
    fp = _token_fingerprint(token)
    with _revoked_lock:
        return fp in _revoked_token_hashes


def _mark_token_revoked(token: str) -> None:
    fp = _token_fingerprint(token)
    with _revoked_lock:
        _revoked_token_hashes.add(fp)


def build_chat_messages(
    system_prompt: str,
    history: Sequence[tuple[str, bool]],
    user_text: str,
) -> list[dict[str, str]]:
    """Build OpenAI-format messages without sending a model name."""
    messages: list[dict[str, str]] = []
    if isinstance(system_prompt, str) and system_prompt.strip():
        messages.append({"role": "system", "content": system_prompt.strip()})

    items = list(history)
    if items and items[-1][1] is True and items[-1][0] == user_text:
        items = items[:-1]

    for content, is_user in items:
        text = content if isinstance(content, str) else ""
        if not text.strip():
            continue
        messages.append({"role": "user" if is_user else "assistant", "content": text})

    spoken = user_text.strip() if isinstance(user_text, str) else ""
    if not spoken:
        spoken = _EMPTY_USER_FALLBACK
    messages.append({"role": "user", "content": spoken})
    return messages


def extract_visible_delta(frame: Mapping[str, Any]) -> Optional[str]:
    """Return visible ``delta.content`` from an SSE JSON frame, or None.

    Frames that carry only ``delta.reasoning`` / ``delta.reasoning_content``,
    a reasoning channel marker, usage metadata, or an empty delta return None
    so reasoning text is never spoken by TTS.
    """
    choices = frame.get("choices")
    if not isinstance(choices, list) or not choices:
        return None
    first = choices[0]
    if not isinstance(first, Mapping):
        return None
    delta = first.get("delta")
    if not isinstance(delta, Mapping):
        return None
    channel = delta.get("channel")
    if isinstance(channel, str) and channel.strip().lower() in _REASONING_CHANNELS:
        return None
    content = delta.get("content")
    if not isinstance(content, str) or content == "":
        return None
    return content


def _extract_error_details(payload: object) -> tuple[str, Optional[str], Optional[int]]:
    """Extract (message, code, status) from an HTTP or SSE error payload."""
    if not isinstance(payload, Mapping):
        return ("pib.Cloud stream error", None, None)
    err = payload.get("error")
    if isinstance(err, Mapping):
        msg = str(err.get("message") or err.get("code") or "pib.Cloud error")
        code = str(err["code"]) if err.get("code") is not None else None
        status = err.get("status")
        return (msg, code, int(status) if isinstance(status, int) else None)
    if isinstance(err, str) and err.strip():
        code = payload.get("code")
        return (err.strip(), str(code) if code is not None else None, None)
    return ("pib.Cloud stream error", None, None)


def _iter_sse_lines(response: Any) -> Iterator[str]:
    """Yield decoded lines from an HTTP response without 512-byte chunk buffering."""
    try:
        lines = response.iter_lines(chunk_size=1, decode_unicode=True)
    except TypeError:
        lines = response.iter_lines(decode_unicode=True)
    for raw in lines:
        if raw is None:
            continue
        if isinstance(raw, bytes):
            yield raw.decode("utf-8", errors="replace")
        else:
            yield str(raw)


def _yield_local_or_notice(
    local_fallback: Optional[Callable[[], Iterable[str]]],
    notice: str,
) -> Iterator[str]:
    """Run the local model fallback when available; otherwise yield ``notice``."""
    if local_fallback is not None:
        try:
            produced = False
            for token in local_fallback():
                if isinstance(token, str) and token:
                    produced = True
                    yield token
            if produced:
                return
        except Exception as exc:
            logger.warning("local model fallback failed: %s", exc)
    yield notice


def stream_pib_cloud_chat(
    *,
    system_prompt: str,
    user_text: str,
    history: Sequence[tuple[str, bool]] = (),
    chat_id: str = "",
    endpoint_base: Optional[str] = None,
    token_fetcher: Optional[Callable[[], Mapping[str, Any]]] = None,
    local_fallback: Optional[Callable[[], Iterable[str]]] = None,
    is_cancelled: Optional[Callable[[], bool]] = None,
    http_post: Optional[Callable[..., Any]] = None,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
    retry_delay_s: float = DEFAULT_RETRY_DELAY_SECONDS,
    sleep_fn: Callable[[float], None] = time.sleep,
    max_tokens: Optional[int] = None,
) -> Iterator[str]:
    """Stream visible content tokens from ``POST /v1/chat/completions``.

    - Reads the key-store row 4 (``pib.Cloud``) bearer token via
      ``resolve_cloud_token`` and sends ``Authorization: Bearer <token>``.
    - Yields each visible ``delta.content`` string as soon as its SSE frame
      arrives, ignoring ``delta.reasoning`` and empty/usage frames.
    - Closes the stream immediately on cancellation (barge-in).
    - Handles 401/403, 402 budget_exhausted, 429, 502/504, and mid-stream
      errors deterministically per Confluence guide 1171390467.
    """
    cancelled = is_cancelled or (lambda: False)
    if cancelled():
        return

    try:
        token = resolve_cloud_token(token_fetcher)
    except CloudTokenError as exc:
        logger.error("pib.Cloud token unavailable (chat=%s): %s", chat_id, exc)
        yield from _yield_local_or_notice(local_fallback, CLOUD_AUTH_FAILED_NOTICE)
        return

    if _is_token_revoked(token):
        logger.error(
            "pib.Cloud token was previously rejected (401/403); "
            "skipping cloud and answering locally (chat=%s)",
            chat_id,
        )
        yield from _yield_local_or_notice(local_fallback, CLOUD_AUTH_FAILED_NOTICE)
        return

    url = chat_completions_url(endpoint_base)
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
        "Accept": "text/event-stream",
        "User-Agent": "pib-voice-assistant/1.0",
    }
    body: dict[str, Any] = {
        "messages": build_chat_messages(system_prompt, history, user_text),
        "stream": True,
    }
    if max_tokens is not None:
        body["max_tokens"] = int(max_tokens)

    post = http_post or _get_http_session().post
    emitted_content = False
    stream_outcome_sent = False

    def _trace_stream_end(outcome: str) -> None:
        nonlocal stream_outcome_sent
        if stream_outcome_sent:
            return
        stream_outcome_sent = True
        trace_emit(
            "stream_done",
            chat_id=chat_id,
            detail={"stream_end": outcome},
        )

    for attempt in range(2):
        if cancelled():
            return

        req_t0 = time.monotonic()
        logger.info(
            "[PERF_TRACE] PIB_CLOUD_REQ_SENT chat=%s attempt=%d elapsed_ms=0.00",
            chat_id,
            attempt + 1,
        )
        trace_emit("request_sent", chat_id=chat_id)
        response = None
        try:
            response = post(
                url,
                headers=headers,
                json=body,
                stream=True,
                timeout=timeout,
            )
            status = getattr(response, "status_code", 200)
            trace_emit(
                "response_headers",
                chat_id=chat_id,
                detail=_response_header_detail(response),
            )

            if status in _AUTH_ERROR_STATUS_CODES:
                _mark_token_revoked(token)
                logger.error(
                    "pib.Cloud rejected bearer token with HTTP %s (chat=%s); "
                    "disabling cloud for this token and falling back locally",
                    status,
                    chat_id,
                )
                _trace_stream_end("error")
                yield from _yield_local_or_notice(
                    local_fallback, CLOUD_AUTH_FAILED_NOTICE
                )
                return

            if status == 402:
                logger.warning(
                    "pib.Cloud budget exhausted (HTTP 402, chat=%s); "
                    "switching turn to local model",
                    chat_id,
                )
                _trace_stream_end("error")
                yield from _yield_local_or_notice(
                    local_fallback, BUDGET_EXHAUSTED_NOTICE
                )
                return

            if status in _RETRYABLE_STATUS_CODES:
                if attempt == 0 and not emitted_content:
                    logger.warning(
                        "pib.Cloud returned HTTP %s (chat=%s); retrying once after %.2fs",
                        status,
                        chat_id,
                        retry_delay_s,
                    )
                    if retry_delay_s > 0:
                        sleep_fn(retry_delay_s)
                    continue
                logger.error(
                    "pib.Cloud HTTP %s persisted after retry (chat=%s); "
                    "falling back locally",
                    status,
                    chat_id,
                )
                _trace_stream_end("error")
                yield from _yield_local_or_notice(
                    local_fallback, CLOUD_UNAVAILABLE_NOTICE
                )
                return

            if status >= 400:
                logger.error(
                    "pib.Cloud returned unexpected HTTP %s (chat=%s); "
                    "falling back locally",
                    status,
                    chat_id,
                )
                _trace_stream_end("error")
                yield from _yield_local_or_notice(
                    local_fallback, CLOUD_UNAVAILABLE_NOTICE
                )
                return

            saw_done = False
            first_token_logged = False
            event_type: Optional[str] = None

            for raw_line in _iter_sse_lines(response):
                if cancelled():
                    logger.info(
                        "pib.Cloud stream cancelled by barge-in (chat=%s)", chat_id
                    )
                    return

                line = raw_line.strip()
                if not line:
                    event_type = None
                    continue
                if line.startswith(":"):
                    continue
                if line.startswith("event:"):
                    event_type = line[6:].strip().lower()
                    continue
                if not line.startswith("data:"):
                    continue

                data_str = line[5:].strip()
                if not data_str:
                    continue
                if data_str == "[DONE]":
                    saw_done = True
                    logger.info(
                        "[PERF_TRACE] PIB_CLOUD_STREAM_DONE chat=%s elapsed_ms=%.2f",
                        chat_id,
                        (time.monotonic() - req_t0) * 1000.0,
                    )
                    _trace_stream_end("done")
                    break

                try:
                    frame = json.loads(data_str)
                except json.JSONDecodeError as exc:
                    raise _SSEStreamError(f"malformed SSE JSON frame: {exc}") from exc

                if not isinstance(frame, Mapping):
                    raise _SSEStreamError("SSE frame is not a JSON object")

                if event_type == "error" or frame.get("error"):
                    msg, code, err_status = _extract_error_details(frame)
                    raise _SSEStreamError(msg, code=code, status=err_status)

                event_type = None
                piece = extract_visible_delta(frame)
                if piece is not None:
                    if not first_token_logged:
                        first_token_logged = True
                        logger.info(
                            "[PERF_TRACE] PIB_CLOUD_FIRST_TOKEN chat=%s elapsed_ms=%.2f",
                            chat_id,
                            (time.monotonic() - req_t0) * 1000.0,
                        )
                        trace_emit("first_token", chat_id=chat_id)
                    emitted_content = True
                    yield piece

            if not saw_done:
                raise _SSEStreamError("pib.Cloud SSE stream closed before [DONE]")

            if not emitted_content:
                logger.warning(
                    "pib.Cloud stream finished with [DONE] but yielded no visible "
                    "content (reasoning-only response, chat=%s)",
                    chat_id,
                )
                yield EMPTY_CONTENT_NOTICE
            return

        except _SSEStreamError as exc:
            if emitted_content:
                logger.warning(
                    "pib.Cloud stream interrupted mid-answer (chat=%s): %s; "
                    "appending cutoff notice",
                    chat_id,
                    exc,
                )
                _trace_stream_end("error")
                yield f" {STREAM_CUTOFF_NOTICE}"
                return
            if exc.code == "budget_exhausted" or exc.status == 402:
                logger.warning(
                    "pib.Cloud SSE budget_exhausted before content (chat=%s); "
                    "switching turn to local model",
                    chat_id,
                )
                _trace_stream_end("error")
                yield from _yield_local_or_notice(
                    local_fallback, BUDGET_EXHAUSTED_NOTICE
                )
                return
            if exc.status in _AUTH_ERROR_STATUS_CODES or exc.code in (
                "invalid_token",
                "token_revoked",
            ):
                _mark_token_revoked(token)
                logger.error(
                    "pib.Cloud SSE auth error (chat=%s): %s; falling back locally",
                    chat_id,
                    exc,
                )
                _trace_stream_end("error")
                yield from _yield_local_or_notice(
                    local_fallback, CLOUD_AUTH_FAILED_NOTICE
                )
                return
            logger.warning(
                "pib.Cloud SSE failed before content (chat=%s): %s; "
                "falling back locally",
                chat_id,
                exc,
            )
            _trace_stream_end("error")
            yield from _yield_local_or_notice(local_fallback, CLOUD_UNAVAILABLE_NOTICE)
            return

        except requests.RequestException as exc:
            if emitted_content:
                logger.warning(
                    "pib.Cloud transport dropped mid-answer (chat=%s): %s; "
                    "appending cutoff notice",
                    chat_id,
                    exc,
                )
                _trace_stream_end("error")
                yield f" {STREAM_CUTOFF_NOTICE}"
                return
            if attempt == 0:
                logger.warning(
                    "pib.Cloud connection error on attempt 1 (chat=%s): %s; "
                    "retrying once after %.2fs",
                    chat_id,
                    exc,
                    retry_delay_s,
                )
                if retry_delay_s > 0:
                    sleep_fn(retry_delay_s)
                continue
            logger.error(
                "pib.Cloud unreachable after retry (chat=%s): %s; "
                "falling back locally",
                chat_id,
                exc,
            )
            _trace_stream_end("error")
            yield from _yield_local_or_notice(local_fallback, CLOUD_UNAVAILABLE_NOTICE)
            return

        finally:
            if response is not None:
                try:
                    response.close()
                except Exception:
                    pass
