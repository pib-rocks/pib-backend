"""Keep the live suite from overwriting the robot's real SmartConnect token.

The suite used to POST a placeholder token to ``/system/smart-connect`` on the
real robot. That endpoint stores the value in the robot's key store, so a test
run replaced the operator's real pib.Cloud token with a placeholder - and every
cloud turn afterwards answered ``401`` and silently fell back to the slow local
model until someone re-entered the real token. Measured once on 192.168.1.172:
a turn took 58.7 s with the placeholder instead of 0.84 s with the real token.

The rule here: a test may only *create* the token when the store is empty, and
must delete exactly the token it created. A stored token is never touched, so a
robot with a real key still runs the suite (the Smart channel works) and comes
out of it unchanged.
"""

from __future__ import annotations

import contextlib
from collections.abc import Iterator

import requests

PLACEHOLDER_TOKEN = "12345678"
CLOUD_PROVIDER_NAMES = frozenset({"pib.cloud", "pib-cloud", "pib cloud"})
DEFAULT_TIMEOUT_S = 15


def _providers(base_url: str, timeout: int = DEFAULT_TIMEOUT_S) -> list[dict]:
    response = requests.get(f"{base_url}/provider", timeout=timeout)
    response.raise_for_status()
    payload = response.json()
    providers = payload.get("providers") if isinstance(payload, dict) else None
    return providers if isinstance(providers, list) else []


def cloud_credential_ref(base_url: str, timeout: int = DEFAULT_TIMEOUT_S) -> str | None:
    """The pib.Cloud credential ref, or ``None`` when no token is stored.

    The provider document is the robot's own view of its key store, so this
    needs no secret and works with encryption on and off.
    """
    for provider in _providers(base_url, timeout=timeout):
        if not isinstance(provider, dict):
            continue
        name = (
            str(provider.get("api_name") or provider.get("name") or "").strip().lower()
        )
        if name in CLOUD_PROVIDER_NAMES:
            ref = provider.get("credentialRef")
            return ref if isinstance(ref, str) and ref else None
    return None


def token_is_stored(base_url: str, timeout: int = DEFAULT_TIMEOUT_S) -> bool:
    """True when the robot already holds a SmartConnect token."""
    return cloud_credential_ref(base_url, timeout=timeout) is not None


def store_cloud_token(
    base_url: str, token: str = PLACEHOLDER_TOKEN, timeout: int = DEFAULT_TIMEOUT_S
) -> None:
    """Store a SmartConnect token. Raises for a failed request."""
    response = requests.post(
        f"{base_url}/system/smart-connect", json={"token": token}, timeout=timeout
    )
    response.raise_for_status()


def delete_cloud_token(base_url: str, timeout: int = DEFAULT_TIMEOUT_S) -> None:
    """Remove the stored SmartConnect token."""
    response = requests.delete(f"{base_url}/system/smart-connect", timeout=timeout)
    response.raise_for_status()


def ensure_smart_connect_token(
    base_url: str, token: str = PLACEHOLDER_TOKEN, timeout: int = DEFAULT_TIMEOUT_S
) -> bool:
    """Make sure a token is stored without ever replacing one.

    Returns ``True`` only when this call created the placeholder - the caller
    then owns the cleanup and must call :func:`delete_cloud_token`.
    """
    if token_is_stored(base_url, timeout=timeout):
        return False
    store_cloud_token(base_url, token=token, timeout=timeout)
    return True


@contextlib.contextmanager
def smart_connect_available(
    base_url: str, token: str = PLACEHOLDER_TOKEN, timeout: int = DEFAULT_TIMEOUT_S
) -> Iterator[bool]:
    """Yield whether this block created the placeholder, and restore the state.

    A stored token (the operator's real one) stays untouched: the block yields
    ``False`` and cleans nothing up.
    """
    created = ensure_smart_connect_token(base_url, token=token, timeout=timeout)
    try:
        yield created
    finally:
        if created:
            with contextlib.suppress(requests.RequestException):
                delete_cloud_token(base_url, timeout=timeout)
