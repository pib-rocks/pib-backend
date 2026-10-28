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

from pib_hermes_config.local_model import API_NAME as LOCAL_FALLBACK_MODEL_NAME
from pib_hermes_config.visible_state import (
    ANSWER_GESTURE_TOOLS,
    issue_answer_gesture,
)

from voice_assistant import direct_tool_loop
from voice_assistant.cloud_token import CloudTokenError, resolve_cloud_token
from voice_assistant.turn_latency import format_event

logger = logging.getLogger(__name__)

LOCAL_FALLBACK_LOG_PREFIX = "[PIB_CLOUD_LOCAL_FALLBACK]"

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


def _cloud_failure_detail(
    response: Any,
    *,
    reason: str,
    http_status: Optional[int] = None,
) -> dict[str, Any]:
    """Sanitized cloud failure context for a local fallback (no bodies or secrets)."""
    detail: dict[str, Any] = {"fallback_reason": reason}
    if http_status is not None:
        detail["cloud_http_status"] = int(http_status)
    headers = getattr(response, "headers", None) if response is not None else None
    request_id = _header_value(headers, "X-Request-ID")
    attempt_id = _header_value(headers, "X-Attempt-ID")
    if request_id is not None:
        detail["cloud_request_id"] = str(request_id)
    if attempt_id is not None:
        detail["cloud_attempt_id"] = str(attempt_id)
    return detail


def format_local_fallback_line(
    *,
    chat_id: str,
    cloud_failure: Mapping[str, Any],
    local_model: str,
    outcome: str,
    local_total_ms: float,
    local_first_token_ms: Optional[float] = None,
) -> str:
    """One machine-readable stderr/log line for a cloud-to-local fallback turn."""
    payload: dict[str, Any] = {
        "event": "pib_cloud_local_fallback",
        "chat_id": chat_id or "",
        "local_model": local_model,
        "outcome": outcome,
        "local_total_ms": round(float(local_total_ms), 2),
    }
    for key, value in cloud_failure.items():
        if value is not None:
            payload[str(key)] = value
    if local_first_token_ms is not None:
        payload["local_first_token_ms"] = round(float(local_first_token_ms), 2)
    return f"{LOCAL_FALLBACK_LOG_PREFIX} " + json.dumps(
        payload, separators=(",", ":"), sort_keys=True
    )


def parse_local_fallback_line(line: str) -> Optional[dict[str, Any]]:
    """Parse a ``[PIB_CLOUD_LOCAL_FALLBACK]`` line, or return None."""
    prefix = LOCAL_FALLBACK_LOG_PREFIX + " "
    if not isinstance(line, str) or not line.startswith(prefix):
        return None
    try:
        data = json.loads(line[len(prefix) :])
    except (json.JSONDecodeError, TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _log_local_fallback_event(
    *,
    chat_id: str,
    cloud_failure: Mapping[str, Any],
    outcome: str,
    local_total_ms: float,
    local_first_token_ms: Optional[float] = None,
) -> None:
    logger.info(
        "%s",
        format_local_fallback_line(
            chat_id=chat_id,
            cloud_failure=cloud_failure,
            local_model=LOCAL_FALLBACK_MODEL_NAME,
            outcome=outcome,
            local_total_ms=local_total_ms,
            local_first_token_ms=local_first_token_ms,
        ),
    )


DEFAULT_PIB_CLOUD_BASE_URL = "https://api.pib.rocks/v1"
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
) -> list[dict[str, Any]]:
    """Build OpenAI-format messages without sending a model name."""
    messages: list[dict[str, Any]] = []
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


def build_openai_tools(
    tools: Optional[Sequence[Mapping[str, Any]]],
) -> list[dict[str, Any]]:
    """Convert MCP tool declarations into OpenAI chat-completions tool schemas."""
    if not tools:
        return []
    openai_tools: list[dict[str, Any]] = []
    for item in tools:
        if not isinstance(item, Mapping):
            continue
        if item.get("type") == "function" and isinstance(item.get("function"), Mapping):
            openai_tools.append(dict(item))
            continue
        name = item.get("name")
        if not isinstance(name, str) or not name.strip():
            continue
        params = item.get("parameters")
        if not (isinstance(params, dict) and params.get("type") == "object"):
            params = {"type": "object", "properties": {}}
        openai_tools.append(
            {
                "type": "function",
                "function": {
                    "name": name.strip(),
                    "description": str(item.get("description") or ""),
                    "parameters": params,
                },
            }
        )
    return openai_tools


def _accumulate_tool_call_deltas(
    frame: Mapping[str, Any],
    pending: dict[int, dict[str, str]],
) -> None:
    """Accumulate streamed ``choices[0].delta.tool_calls`` chunks by index."""
    choices = frame.get("choices")
    if not isinstance(choices, list) or not choices:
        return
    first = choices[0]
    if not isinstance(first, Mapping):
        return
    delta = first.get("delta")
    if not isinstance(delta, Mapping):
        return
    raw_calls = delta.get("tool_calls")
    if not isinstance(raw_calls, list):
        return
    for raw_idx, tc in enumerate(raw_calls):
        if not isinstance(tc, Mapping):
            continue
        idx_val = tc.get("index")
        idx = (
            idx_val
            if isinstance(idx_val, int) and not isinstance(idx_val, bool)
            else raw_idx
        )
        entry = pending.setdefault(
            idx,
            {"id": "", "name": "", "arguments": ""},
        )
        tc_id = tc.get("id")
        if isinstance(tc_id, str) and tc_id:
            entry["id"] = tc_id
        fn = tc.get("function")
        if isinstance(fn, Mapping):
            fn_name = fn.get("name")
            if isinstance(fn_name, str) and fn_name:
                if not entry["name"]:
                    entry["name"] = fn_name
            fn_args = fn.get("arguments")
            if isinstance(fn_args, str) and fn_args:
                entry["arguments"] += fn_args
        elif isinstance(tc.get("name"), str) and tc.get("name"):
            if not entry["name"]:
                entry["name"] = str(tc["name"])


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


def _stream_usage_detail(frame: Mapping[str, Any]) -> Optional[dict[str, Any]]:
    """The provider's usage report as the API forwards it on the closing frame.

    A streaming turn does not carry ``X-Usage-*`` headers (they exist only at
    stream end, see the interface description); the API forwards the provider's
    report on the last SSE frame instead. Only counts that are actually present
    are reported - a missing count is left out, never zeroed.
    """
    usage = frame.get("usage")
    if not isinstance(usage, Mapping):
        return None
    detail: dict[str, Any] = {}
    prompt = _header_number(usage.get("prompt_tokens"))
    completion = _header_number(usage.get("completion_tokens"))
    if prompt is not None:
        detail["usage_prompt_tokens"] = prompt
    if completion is not None:
        detail["usage_completion_tokens"] = completion
    return detail or None


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
    *,
    chat_id: str = "",
    cloud_failure: Optional[Mapping[str, Any]] = None,
) -> Iterator[str]:
    """Run the local model fallback when available; otherwise yield ``notice``."""
    if local_fallback is not None:
        t0 = time.monotonic()
        first_token_ms: Optional[float] = None
        try:
            produced = False
            for token in local_fallback():
                if isinstance(token, str) and token:
                    if first_token_ms is None:
                        first_token_ms = (time.monotonic() - t0) * 1000.0
                    produced = True
                    yield token
            total_ms = (time.monotonic() - t0) * 1000.0
            if produced:
                if cloud_failure is not None:
                    _log_local_fallback_event(
                        chat_id=chat_id,
                        cloud_failure=cloud_failure,
                        outcome="success",
                        local_total_ms=total_ms,
                        local_first_token_ms=first_token_ms,
                    )
                return
            if cloud_failure is not None:
                _log_local_fallback_event(
                    chat_id=chat_id,
                    cloud_failure=cloud_failure,
                    outcome="empty",
                    local_total_ms=total_ms,
                )
        except Exception as exc:
            total_ms = (time.monotonic() - t0) * 1000.0
            logger.warning("local model fallback failed: %s", exc)
            if cloud_failure is not None:
                _log_local_fallback_event(
                    chat_id=chat_id,
                    cloud_failure=cloud_failure,
                    outcome="error",
                    local_total_ms=total_ms,
                    local_first_token_ms=first_token_ms,
                )
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
    tools: Optional[Sequence[Mapping[str, Any]]] = None,
    execute_tool: Optional[Callable[[str, dict[str, Any]], Any]] = None,
    on_tool_call: Optional[Callable[[dict], None]] = None,
    on_tool_start: Optional[Callable[[dict], None]] = None,
    allow_image: bool = False,
    max_tool_rounds: int = direct_tool_loop.MAX_TOOL_ROUNDS,
) -> Iterator[str]:
    """Stream visible content tokens from ``POST /v1/chat/completions``.

    - Reads the key-store row 4 (``pib.Cloud``) bearer token via
      ``resolve_cloud_token`` and sends ``Authorization: Bearer ***
    - Yields each visible ``delta.content`` string as soon as its SSE frame
      arrives, ignoring ``delta.reasoning`` and empty/usage frames.
    - When ``tools`` are provided, sends OpenAI-format ``tools`` in the request,
      accumulates streamed ``delta.tool_calls``, executes MCP tools locally,
      and continues the conversation until a final answer is streamed.
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
        yield from _yield_local_or_notice(
            local_fallback,
            CLOUD_AUTH_FAILED_NOTICE,
            chat_id=chat_id,
            cloud_failure={"fallback_reason": "cloud_token_unavailable"},
        )
        return

    if _is_token_revoked(token):
        logger.error(
            "pib.Cloud token was previously rejected (401/403); "
            "skipping cloud and answering locally (chat=%s)",
            chat_id,
        )
        yield from _yield_local_or_notice(
            local_fallback,
            CLOUD_AUTH_FAILED_NOTICE,
            chat_id=chat_id,
            cloud_failure={"fallback_reason": "cloud_token_revoked"},
        )
        return

    url = chat_completions_url(endpoint_base)
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
        "Accept": "text/event-stream",
        "User-Agent": "pib-voice-assistant/1.0",
    }
    messages: list[dict[str, Any]] = build_chat_messages(
        system_prompt, history, user_text
    )
    openai_tools = build_openai_tools(tools)
    runner = execute_tool or direct_tool_loop.execute_mcp_tool

    post = http_post or _get_http_session().post
    emitted_content = False
    first_token_logged = False
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

    rounds = max(1, int(max_tool_rounds)) if openai_tools else 1
    tool_call_counter = 0
    for _tool_round in range(rounds):
        body: dict[str, Any] = {
            "messages": list(messages),
            "stream": True,
        }
        if max_tokens is not None:
            body["max_tokens"] = int(max_tokens)
        if openai_tools:
            body["tools"] = openai_tools

        round_tool_calls: list[dict[str, Any]] = []
        round_content_parts: list[str] = []

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
                        local_fallback,
                        CLOUD_AUTH_FAILED_NOTICE,
                        chat_id=chat_id,
                        cloud_failure=_cloud_failure_detail(
                            response,
                            reason="cloud_auth_rejected",
                            http_status=status,
                        ),
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
                        local_fallback,
                        BUDGET_EXHAUSTED_NOTICE,
                        chat_id=chat_id,
                        cloud_failure=_cloud_failure_detail(
                            response,
                            reason="cloud_budget_exhausted",
                            http_status=status,
                        ),
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
                        local_fallback,
                        CLOUD_UNAVAILABLE_NOTICE,
                        chat_id=chat_id,
                        cloud_failure=_cloud_failure_detail(
                            response,
                            reason="cloud_http_retry_exhausted",
                            http_status=status,
                        ),
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
                        local_fallback,
                        CLOUD_UNAVAILABLE_NOTICE,
                        chat_id=chat_id,
                        cloud_failure=_cloud_failure_detail(
                            response,
                            reason="cloud_http_error",
                            http_status=status,
                        ),
                    )
                    return

                saw_done = False
                event_type: Optional[str] = None
                pending_tool_calls: dict[int, dict[str, str]] = {}
                round_content_parts = []

                for raw_line in _iter_sse_lines(response):
                    if cancelled():
                        logger.info(
                            "pib.Cloud stream cancelled by barge-in (chat=%s)",
                            chat_id,
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
                        break

                    try:
                        frame = json.loads(data_str)
                    except json.JSONDecodeError as exc:
                        raise _SSEStreamError(
                            f"malformed SSE JSON frame: {exc}"
                        ) from exc

                    if not isinstance(frame, Mapping):
                        raise _SSEStreamError("SSE frame is not a JSON object")

                    if event_type == "error" or frame.get("error"):
                        msg, code, err_status = _extract_error_details(frame)
                        raise _SSEStreamError(msg, code=code, status=err_status)

                    usage_detail = _stream_usage_detail(frame)
                    if usage_detail is not None:
                        trace_emit("usage", chat_id=chat_id, detail=usage_detail)

                    event_type = None
                    if openai_tools:
                        _accumulate_tool_call_deltas(frame, pending_tool_calls)

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
                        round_content_parts.append(piece)
                        yield piece

                if not saw_done:
                    raise _SSEStreamError("pib.Cloud SSE stream closed before [DONE]")

                round_tool_calls = [
                    {
                        "id": entry["id"] or f"{entry['name']}-{idx}",
                        "type": "function",
                        "function": {
                            "name": entry["name"],
                            "arguments": entry["arguments"] or "{}",
                        },
                    }
                    for idx, entry in sorted(pending_tool_calls.items())
                    if entry.get("name")
                ]

                if round_tool_calls and openai_tools:
                    break

                _trace_stream_end("done")
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
                        local_fallback,
                        BUDGET_EXHAUSTED_NOTICE,
                        chat_id=chat_id,
                        cloud_failure={
                            "fallback_reason": "cloud_sse_budget_exhausted",
                            "cloud_http_status": 402,
                        },
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
                        local_fallback,
                        CLOUD_AUTH_FAILED_NOTICE,
                        chat_id=chat_id,
                        cloud_failure={
                            "fallback_reason": "cloud_sse_auth_rejected",
                            "cloud_http_status": exc.status,
                        },
                    )
                    return
                logger.warning(
                    "pib.Cloud SSE failed before content (chat=%s): %s; "
                    "falling back locally",
                    chat_id,
                    exc,
                )
                _trace_stream_end("error")
                yield from _yield_local_or_notice(
                    local_fallback,
                    CLOUD_UNAVAILABLE_NOTICE,
                    chat_id=chat_id,
                    cloud_failure={"fallback_reason": "cloud_sse_pre_content"},
                )
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
                transport_reason = (
                    "cloud_transport_timeout"
                    if isinstance(exc, requests.Timeout)
                    else "cloud_transport_retry_exhausted"
                )
                logger.error(
                    "pib.Cloud unreachable after retry (chat=%s): %s; "
                    "falling back locally",
                    chat_id,
                    exc,
                )
                _trace_stream_end("error")
                yield from _yield_local_or_notice(
                    local_fallback,
                    CLOUD_UNAVAILABLE_NOTICE,
                    chat_id=chat_id,
                    cloud_failure={"fallback_reason": transport_reason},
                )
                return

            finally:
                if response is not None:
                    try:
                        response.close()
                    except Exception:
                        pass

        if not round_tool_calls or not openai_tools:
            break

        messages.append(
            {
                "role": "assistant",
                "content": (
                    "".join(round_content_parts) if round_content_parts else None
                ),
                "tool_calls": round_tool_calls,
            }
        )
        for tc in round_tool_calls:
            fn = tc["function"]
            name = fn["name"]
            raw_args = fn.get("arguments") or "{}"
            try:
                parsed_args = (
                    json.loads(raw_args)
                    if isinstance(raw_args, str) and raw_args.strip()
                    else {}
                )
            except json.JSONDecodeError:
                parsed_args = {}
            if not isinstance(parsed_args, dict):
                parsed_args = {}

            if name == direct_tool_loop.IMAGE_TOOL and not allow_image:
                raise direct_tool_loop.DirectToolLoopError(
                    "image tool is not available on this turn"
                )

            call_id = direct_tool_loop._tool_call_id(tc, name, tool_call_counter)
            tool_call_counter += 1
            logger.info("pib.Cloud direct tool call name=%s chat=%s", name, chat_id)
            direct_tool_loop._report_tool_start(
                on_tool_start,
                name=name,
                call_id=call_id,
                arguments=parsed_args,
            )
            started = time.monotonic()
            try:
                if name in ANSWER_GESTURE_TOOLS:
                    output = issue_answer_gesture(name, parsed_args, runner)
                else:
                    output = runner(name, parsed_args)
            except Exception as exc:
                direct_tool_loop._report_tool_call(
                    on_tool_call,
                    name=name,
                    call_id=call_id,
                    arguments=parsed_args,
                    started=started,
                    ok=False,
                    error=type(exc).__name__,
                )
                raise
            direct_tool_loop._report_tool_call(
                on_tool_call,
                name=name,
                call_id=call_id,
                arguments=parsed_args,
                started=started,
                ok=True,
                error="",
            )
            tool_payload = output if isinstance(output, dict) else {"result": output}
            messages.append(
                {
                    "role": "tool",
                    "tool_call_id": tc["id"],
                    "content": json.dumps(tool_payload),
                }
            )

    _trace_stream_end("done")
    if not emitted_content:
        yield EMPTY_CONTENT_NOTICE
