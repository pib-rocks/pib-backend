"""pib.Cloud live-session client.

``POST {base}/v1/session`` asks for a short-lived live grant.
``POST {base}/v1/session/usage`` settles one session. HTTP is urllib only.
The bearer token and the ephemeral token are never logged.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError
from urllib.request import Request, urlopen

logger = logging.getLogger(__name__)

_SESSION_PATH = "/v1/session"
_USAGE_PATH = "/v1/session/usage"

# Stable codes from the pib.Cloud error envelope, keyed by HTTP status.
_CODE_BY_STATUS = {
    400: "invalid_request",
    401: "invalid_token",
    402: "budget_exhausted",
    403: "token_revoked",
    502: "upstream_unavailable",
    504: "upstream_timeout",
}


class CloudSessionError(Exception):
    """The session call failed. The message never contains a token.

    ``code`` and ``status`` are set when the API answered.
    """

    def __init__(
        self,
        message: str,
        *,
        code: str | None = None,
        status: int | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.status = status


@dataclass(frozen=True)
class LiveSessionGrant:
    """One answer from ``POST /v1/session``.

    ``token`` is the ephemeral token. A daily-quota fallback has
    ``use_cascade`` set and no token; that answer is not an error.
    """

    use_cascade: bool
    session_id: str | None = None
    token: str | None = None
    model: str | None = None
    endpoint: str | None = None
    expires_at: str | None = None
    ttl_seconds: int | None = None
    reason: str | None = None
    cascade_endpoint: str | None = None
    daily_slots_remaining: int | None = None

    def __repr__(self) -> str:
        secret = self.token
        parts = []
        for name, value in (
            ("use_cascade", self.use_cascade),
            ("session_id", self.session_id),
            ("token", None),
            ("model", self.model),
            ("endpoint", self.endpoint),
            ("expires_at", self.expires_at),
            ("ttl_seconds", self.ttl_seconds),
            ("reason", self.reason),
            ("cascade_endpoint", self.cascade_endpoint),
            ("daily_slots_remaining", self.daily_slots_remaining),
        ):
            if name == "token":
                rendered = "'<redacted>'" if secret is not None else "None"
            else:
                rendered = repr(value)
                if secret:
                    rendered = rendered.replace(secret, "<redacted>")
            parts.append(f"{name}={rendered}")
        return "LiveSessionGrant(" + ", ".join(parts) + ")"


class CloudSessionClient:
    """Opens a live session and reports its usage. ``opener`` stands in for urlopen."""

    def __init__(
        self,
        *,
        base_url: str,
        token: str,
        timeout_s: float = 15.0,
        opener=None,
    ) -> None:
        self._base_url = base_url.rstrip("/")
        self._token = token
        self._timeout_s = timeout_s
        self._opener = opener

    def open_session(
        self,
        *,
        model: str | None = None,
        previous_session_id: str | None = None,
    ) -> LiveSessionGrant:
        """Ask for a live grant.

        A 200 body with ``use_cascade`` true is returned, not raised.
        401, 403, 400, 402 and 5xx raise ``CloudSessionError`` with the
        typed code from the error envelope.
        """
        payload: dict[str, Any] = {}
        if model is not None:
            payload["model"] = model
        if previous_session_id is not None:
            payload["previous_session_id"] = previous_session_id
        raw, status = self._post(_SESSION_PATH, payload)
        if status != 200:
            raise self._failure(status, raw)
        grant = _parse_grant(raw, self._token)
        logger.info("pib.Cloud session grant %s", grant)
        return grant

    def report_usage(
        self,
        session_id: str,
        *,
        audio_input_tokens: int = 0,
        audio_output_tokens: int = 0,
        text_tokens: int = 0,
        video_input_tokens: int = 0,
        usage_metadata: dict | None = None,
        model: str | None = None,
    ) -> dict:
        """Settle ``session_id`` with the canonical usage field names.

        The route also accepts the camelCase aliases a Live client has in
        hand. This client sends the canonical names.
        """
        if not isinstance(session_id, str) or session_id.strip() == "":
            raise CloudSessionError(
                "session_id is required",
                code="invalid_request",
            )
        payload: dict[str, Any] = {
            "session_id": session_id,
            "audio_input_tokens": audio_input_tokens,
            "audio_output_tokens": audio_output_tokens,
            "video_input_tokens": video_input_tokens,
            "text_tokens": text_tokens,
        }
        if usage_metadata is not None:
            payload["usage_metadata"] = usage_metadata
        if model is not None:
            payload["model"] = model
        raw, status = self._post(_USAGE_PATH, payload)
        if status != 200:
            raise self._failure(status, raw)
        body = _json_object(raw)
        if not isinstance(body, dict):
            raise CloudSessionError(
                "pib.Cloud usage response is not a JSON object",
                status=status,
            )
        logger.info("pib.Cloud session usage reported session_id=%s", session_id)
        return body

    def _post(self, path: str, payload: dict[str, Any]) -> tuple[bytes, int]:
        request = Request(
            self._base_url + path,
            data=json.dumps(payload).encode("utf-8"),
            method="POST",
        )
        request.add_header("Content-Type", "application/json")
        request.add_header("Authorization", f"Bearer {self._token}")
        return _exchange(request, self._opener, self._timeout_s)

    def _failure(self, status: int, raw: bytes) -> CloudSessionError:
        code, message = _error_fields(_json_object(raw), status)
        message = _redact(message, self._token)
        error = CloudSessionError(message, code=code, status=status)
        logger.warning(
            "pib.Cloud session failed status=%s code=%s",
            error.status,
            error.code,
        )
        return error


def _exchange(request: Request, opener, timeout: float) -> tuple[bytes, int]:
    open_ = opener or urlopen
    try:
        with open_(request, timeout=timeout) as response:
            status = getattr(response, "status", 200)
            return response.read(), int(status)
    except HTTPError as error:
        try:
            raw = error.read()
        except Exception:
            raw = b""
        return raw, int(error.code)
    except Exception:
        raise CloudSessionError("pib.Cloud session request failed") from None


def _parse_grant(raw: bytes, secret: str) -> LiveSessionGrant:
    body = _json_object(raw)
    if not isinstance(body, dict):
        raise CloudSessionError(
            _redact("pib.Cloud session response is not a JSON object", secret)
        )
    use_cascade = body.get("use_cascade")
    if not isinstance(use_cascade, bool):
        raise CloudSessionError(
            _redact("pib.Cloud session response has no use_cascade flag", secret)
        )
    return LiveSessionGrant(
        use_cascade=use_cascade,
        session_id=_as_str(body.get("session_id")),
        token=_as_str(body.get("ephemeral_token")),
        model=_as_str(body.get("model")),
        endpoint=_as_str(body.get("endpoint")),
        expires_at=_as_str(body.get("expires_at")),
        ttl_seconds=_as_int(body.get("ttl_seconds")),
        reason=_as_str(body.get("reason")),
        cascade_endpoint=_as_str(body.get("cascade_endpoint")),
        daily_slots_remaining=_as_int(body.get("daily_slots_remaining")),
    )


def _error_fields(body: object, status: int) -> tuple[str, str]:
    """``(code, message)`` from the OpenAI-shaped error envelope."""
    code = _CODE_BY_STATUS.get(status)
    message = "pib.Cloud rejected the session request"
    if isinstance(body, dict):
        error = body.get("error")
        if isinstance(error, dict):
            raw_code = error.get("code")
            if isinstance(raw_code, str) and raw_code.strip():
                code = raw_code
            raw_message = error.get("message")
            if isinstance(raw_message, str) and raw_message.strip():
                message = raw_message
        elif isinstance(error, str) and error.strip():
            message = error
            raw_code = body.get("code")
            if isinstance(raw_code, str) and raw_code.strip():
                code = raw_code
    if code is None:
        code = f"http_{status}"
    return code, message


def _json_object(raw: bytes) -> object:
    if not raw:
        return None
    try:
        return json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError):
        return None


def _as_str(value: object) -> str | None:
    if isinstance(value, str):
        return value
    return None


def _as_int(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _redact(text: str, secret: str) -> str:
    if secret and secret in text:
        return text.replace(secret, "<redacted>")
    return text
