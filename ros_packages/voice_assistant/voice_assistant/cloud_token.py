"""SmartConnect token, read from the one key store.

The voice node uses the same credential route as a provider key. The route
is addressed by the pib.Cloud catalogue id. A locked store fails with the
clear message. Nothing here falls back to a file, to cleartext, or to
another provider. The secret is not logged.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, Callable, Mapping, Optional
from urllib.error import HTTPError
from urllib.request import Request, urlopen

logger = logging.getLogger(__name__)

CLOUD_PROVIDER = "pib-cloud"
KEY_SOURCE_STORE = "key-store"
_SMART_CONNECT_PATH = "/system/smart-connect"
_PROVIDERS_PATH = "/provider"


class CloudTokenError(Exception):
    """The cloud token is not available. The message never contains a secret."""


def unavailable_message() -> str:
    """Same wording as a provider key the store cannot hand out."""
    return f"No keys are available for provider {CLOUD_PROVIDER}."


def log_cloud_token_source() -> None:
    """Name the route. The line does not carry the token."""
    logger.info(
        "cloud token source=%s provider=%s",
        KEY_SOURCE_STORE,
        CLOUD_PROVIDER,
    )


def read_cloud_token(
    fetch: Optional[Callable[[], Mapping[str, Any]]] = None,
) -> str:
    """The unlocked SmartConnect token, or CloudTokenError.

    ``mode`` other than unlocked is a failure even when a secret field is
    present. An empty secret is a failure. The environment is not consulted.
    """
    reader = fetch or _fetch_cloud_token
    try:
        state = reader()
    except Exception:
        state = None
    if not isinstance(state, dict) or state.get("mode") != "unlocked":
        raise CloudTokenError(unavailable_message()) from None
    secret = state.get("secret")
    if not isinstance(secret, str) or secret.strip() == "":
        raise CloudTokenError(unavailable_message()) from None
    return secret


def resolve_cloud_token(
    fetch: Optional[Callable[[], Mapping[str, Any]]] = None,
) -> str:
    """Read the token and record that the key store supplied it."""
    token = read_cloud_token(fetch)
    log_cloud_token_source()
    return token


def submit_cloud_token(
    token: str,
    opener=None,
    base_url: Optional[str] = None,
    timeout: float = 2.0,
) -> str:
    """Store the token. The request body is the token and nothing else.

    Returns the credential_ref. A refusal raises CloudTokenError. The token
    is not logged.
    """
    if not isinstance(token, str) or token.strip() == "":
        raise CloudTokenError(unavailable_message()) from None
    url = _base(base_url) + _SMART_CONNECT_PATH
    payload = json.dumps({"token": token}).encode("utf-8")
    request = Request(url, data=payload, method="POST")
    request.add_header("Content-Type", "application/json")
    raw, status = _exchange(request, opener, timeout)
    body = _json_object(raw)
    if status >= 400 or not isinstance(body, dict):
        raise CloudTokenError(_safe_error(body, token)) from None
    ref = body.get("credentialRef")
    if not isinstance(ref, str) or ref.strip() == "" or token in ref:
        raise CloudTokenError(unavailable_message()) from None
    return ref


def delete_stored_cloud_token(
    opener=None,
    base_url: Optional[str] = None,
    timeout: float = 2.0,
) -> None:
    """Remove the one stored token. Sends no password and no secret."""
    url = _base(base_url) + _SMART_CONNECT_PATH
    request = Request(url, method="DELETE")
    raw, status = _exchange(request, opener, timeout)
    if status in (200, 204):
        return
    raise CloudTokenError(_safe_error(_json_object(raw), "")) from None


def cloud_token_is_stored(
    opener=None,
    base_url: Optional[str] = None,
    timeout: float = 2.0,
) -> bool:
    """True when the pib.Cloud row has a credential_ref. The secret is absent.

    Another provider's credential does not count.
    """
    url = _base(base_url) + _PROVIDERS_PATH
    try:
        raw, status = _exchange(Request(url, method="GET"), opener, timeout)
    except CloudTokenError:
        return False
    if status >= 400:
        return False
    return _cloud_credential_present(_json_object(raw))


def _fetch_cloud_token() -> dict:
    """The credential route for this provider only."""
    try:
        from pib_api_client.key_store_client import read_provider_key
    except Exception:
        return {"mode": "unavailable", "secret": None}
    return read_provider_key(CLOUD_PROVIDER)


def _base(base_url: Optional[str]) -> str:
    if base_url is not None:
        return base_url.rstrip("/")
    return os.environ.get("FLASK_API_BASE_URL", "http://127.0.0.1:5000").rstrip("/")


def _exchange(request, opener, timeout: float) -> tuple[bytes, int]:
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
    except Exception:
        raise CloudTokenError(unavailable_message()) from None


def _json_object(raw: bytes) -> object:
    if not raw:
        return None
    try:
        return json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError):
        return None


def _safe_error(body: object, token: str) -> str:
    """An error string that does not repeat the token."""
    if isinstance(body, dict):
        error = body.get("error")
        if isinstance(error, str) and error.strip() and token not in error:
            return error
    return unavailable_message()


def _cloud_credential_present(payload: object) -> bool:
    if not isinstance(payload, dict):
        return False
    providers = payload.get("providers")
    if not isinstance(providers, list):
        return False
    for provider in providers:
        if not isinstance(provider, dict):
            continue
        models = provider.get("models")
        if not isinstance(models, list):
            continue
        for model in models:
            if not isinstance(model, dict):
                continue
            if model.get("apiName") != CLOUD_PROVIDER:
                continue
            ref = model.get("credentialRef")
            return isinstance(ref, str) and ref.strip() != ""
    return False
