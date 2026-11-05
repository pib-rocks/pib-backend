"""Mint and settle pib.Cloud live sessions.

The SmartConnect bearer comes from the key store. Ephemeral tokens and raw
HTTP bodies are never logged and never appear in error text.
"""

from __future__ import annotations

import json
import os
import socket
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from voice_assistant.cloud_token import CloudTokenError, read_cloud_token
from voice_assistant.pib_cloud_stream import pib_cloud_base_url

_SESSION_PATH = "/session"
_USAGE_PATH = "/session/usage"
_TIMEOUT_ENV = "PIB_CLOUD_SESSION_TIMEOUT_SECONDS"
_DEFAULT_TIMEOUT_SECONDS = 5.0

_SESSION_STRING_FIELDS = (
    "session_id",
    "ephemeral_token",
    "issued_at",
    "expires_at",
    "model",
    "transport",
    "endpoint",
)
_SESSION_INT_FIELDS = (
    "ttl_seconds",
    "daily_slots_used",
    "daily_slots_limit",
    "daily_slots_remaining",
)
# The service returns the pre-hold as a number (a float), never as an int.
_SESSION_NUMBER_FIELDS = ("pre_hold_pib_tokens",)


@dataclass(frozen=True)
class CloudSession:
    session_id: str
    ephemeral_token: str
    issued_at: str
    expires_at: str
    ttl_seconds: int
    model: str
    transport: str
    endpoint: str
    pre_hold_pib_tokens: float
    daily_slots_used: int
    daily_slots_limit: int
    daily_slots_remaining: int


@dataclass(frozen=True)
class CascadeResult:
    reason: str


@dataclass(frozen=True)
class CloudUsageResult:
    settled_pib_tokens: float | None = None
    status: str | None = None


class CloudSessionError(Exception):
    """Live session HTTP or payload failure. ``code`` is stable; the message has no secret."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def session_url(base_url: Optional[str] = None) -> str:
    """``POST`` target for minting a live session."""
    return pib_cloud_base_url(base_url) + _SESSION_PATH


def session_usage_url(base_url: Optional[str] = None) -> str:
    """``POST`` target for usage settlement."""
    return pib_cloud_base_url(base_url) + _USAGE_PATH


def session_timeout_seconds() -> float:
    raw = os.environ.get(_TIMEOUT_ENV, str(_DEFAULT_TIMEOUT_SECONDS))
    try:
        value = float(raw)
    except (TypeError, ValueError):
        value = _DEFAULT_TIMEOUT_SECONDS
    if value <= 0:
        value = _DEFAULT_TIMEOUT_SECONDS
    return value


def mint_session(
    model: str,
    previous_session_id: str | None = None,
    *,
    fetch: Optional[Callable[[], Mapping[str, Any]]] = None,
    opener=None,
    base_url: Optional[str] = None,
    timeout: Optional[float] = None,
) -> CloudSession | CascadeResult:
    """Request an ephemeral live session from pib.Cloud, or a cascade hint."""
    token = read_cloud_token(fetch)
    payload: dict[str, object] = {"model": model}
    if previous_session_id is not None:
        payload["previous_session_id"] = previous_session_id
    raw, status = _post_json(
        session_url(base_url),
        token,
        payload,
        opener,
        timeout if timeout is not None else session_timeout_seconds(),
    )
    if status != 200:
        raise _http_error(status, _json_object(raw), (token,))
    body = _json_object(raw)
    if not isinstance(body, dict):
        raise CloudSessionError("bad_response", _generic_message("bad_response"))
    if body.get("use_cascade") is True:
        reason = body.get("reason")
        if not isinstance(reason, str) or not reason.strip():
            reason = "use_cascade"
        return CascadeResult(reason=reason)
    if body.get("status") != "ok":
        raise CloudSessionError("bad_response", _generic_message("bad_response"))
    try:
        return _session_from_body(body)
    except CloudSessionError:
        raise
    except Exception:
        raise CloudSessionError(
            "bad_response", _generic_message("bad_response")
        ) from None


def report_usage(
    session_id: str,
    usage_metadata: dict | None = None,
    *,
    fetch: Optional[Callable[[], Mapping[str, Any]]] = None,
    opener=None,
    base_url: Optional[str] = None,
    timeout: Optional[float] = None,
) -> CloudUsageResult:
    """Post provider usage metadata for settlement."""
    token = read_cloud_token(fetch)
    payload: dict[str, object] = {"session_id": session_id}
    if usage_metadata is not None:
        payload["usage_metadata"] = usage_metadata
    raw, status = _post_json(
        session_usage_url(base_url),
        token,
        payload,
        opener,
        timeout if timeout is not None else session_timeout_seconds(),
    )
    if status != 200:
        raise _http_error(status, _json_object(raw), (token,))
    body = _json_object(raw)
    if not isinstance(body, dict):
        raise CloudSessionError("bad_response", _generic_message("bad_response"))
    # The service bills with "billed_pib_tokens"; "settled_pib_tokens" is kept
    # as a fallback for older answers. Both are numbers, never ints.
    settled = body.get("billed_pib_tokens")
    if settled is None:
        settled = body.get("settled_pib_tokens")
    if settled is not None and not isinstance(settled, (int, float)):
        try:
            settled = float(settled)
        except (TypeError, ValueError):
            settled = None
    status_text = body.get("status")
    if status_text is not None and not isinstance(status_text, str):
        status_text = None
    return CloudUsageResult(
        settled_pib_tokens=(
            float(settled) if isinstance(settled, (int, float)) else None
        ),
        status=status_text if isinstance(status_text, str) else None,
    )


def _post_json(
    url: str,
    bearer: str,
    payload: dict[str, object],
    opener,
    timeout: float,
) -> tuple[bytes, int]:
    data = json.dumps(payload).encode("utf-8")
    request = Request(url, data=data, method="POST")
    request.add_header("Content-Type", "application/json")
    request.add_header("Authorization", f"Bearer {bearer}")
    return _exchange(request, opener, timeout)


def _exchange(request: Request, opener, timeout: float) -> tuple[bytes, int]:
    open_ = opener or urlopen
    try:
        with open_(request, timeout=timeout) as response:
            status = getattr(response, "status", 200)
            return response.read(), status
    except HTTPError as error:
        try:
            raw = error.read()
        except Exception:
            raw = b""
        return raw, error.code
    except (TimeoutError, socket.timeout):
        raise CloudSessionError("timeout", _generic_message("timeout")) from None
    except URLError as error:
        reason = getattr(error, "reason", None)
        if isinstance(reason, (TimeoutError, socket.timeout)):
            raise CloudSessionError("timeout", _generic_message("timeout")) from None
        raise CloudSessionError(
            "upstream_unavailable", _generic_message("upstream_unavailable")
        ) from None
    except Exception:
        raise CloudSessionError(
            "upstream_unavailable", _generic_message("upstream_unavailable")
        ) from None


def _http_error(
    status: int, body: object, secrets: tuple[str, ...]
) -> CloudSessionError:
    if status == 401:
        code = "unauthorized"
    elif status == 402:
        code = "budget_exhausted"
    elif status == 400:
        code = "invalid_request"
    elif status >= 500:
        code = "upstream_unavailable"
    else:
        code = "bad_response"
    return CloudSessionError(code, _safe_message(body, secrets, code))


def _session_from_body(body: dict[str, object]) -> CloudSession:
    values: dict[str, object] = {}
    for name in _SESSION_STRING_FIELDS:
        value = body.get(name)
        if not isinstance(value, str) or value.strip() == "":
            raise CloudSessionError("bad_response", _generic_message("bad_response"))
        values[name] = value
    for name in _SESSION_INT_FIELDS:
        value = body.get(name)
        if isinstance(value, bool) or not isinstance(value, int):
            raise CloudSessionError("bad_response", _generic_message("bad_response"))
        values[name] = value
    for name in _SESSION_NUMBER_FIELDS:
        value = body.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CloudSessionError("bad_response", _generic_message("bad_response"))
        values[name] = float(value)
    return CloudSession(**values)  # type: ignore[arg-type]


def _json_object(raw: bytes) -> object:
    if not raw:
        return None
    try:
        return json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError):
        return None


def _generic_message(code: str) -> str:
    return {
        "unauthorized": "pib.Cloud rejected the SmartConnect credential.",
        "budget_exhausted": "pib.Cloud live session budget is exhausted.",
        "invalid_request": "pib.Cloud rejected the live session request.",
        "upstream_unavailable": "pib.Cloud live session service is unavailable.",
        "timeout": "pib.Cloud live session request timed out.",
        "bad_response": "pib.Cloud live session response was invalid.",
    }.get(code, "pib.Cloud live session request failed.")


def _safe_message(body: object, secrets: tuple[str, ...], code: str) -> str:
    if isinstance(body, dict):
        for key in ("error", "message"):
            candidate = body.get(key)
            if isinstance(candidate, str) and candidate.strip():
                if all(secret not in candidate for secret in secrets if secret):
                    return candidate
            nested = body.get(key)
            if isinstance(nested, dict):
                nested_text = nested.get("message") or nested.get("code")
                if isinstance(nested_text, str) and nested_text.strip():
                    if all(secret not in nested_text for secret in secrets if secret):
                        return nested_text
    return _generic_message(code)
