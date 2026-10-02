"""Read one provider key from the flask key store.

Same HTTP path the voice node already uses for the store's mode. The
response body is the secret, so this module never logs it. A locked or
unreachable store is reported as such and carries no secret.
"""

from __future__ import annotations

import json
import os
from typing import Optional
from urllib.parse import quote
from urllib.request import urlopen

_CREDENTIAL_PATH = "/system/key-store/credential/"


def read_provider_key(
    api_name: str,
    opener=None,
    base_url: Optional[str] = None,
    timeout: float = 2.0,
) -> dict:
    """``mode`` is ``unlocked``, ``degraded``, or ``unavailable``.

    ``secret`` is set only when the store is unlocked and this provider has
    a key. Anything else is None, including a failed read.
    """
    unavailable = {"mode": "unavailable", "secret": None}
    if not isinstance(api_name, str) or api_name.strip() == "":
        return unavailable
    if opener is None:
        opener = urlopen
    base = (
        base_url
        if base_url is not None
        else os.environ.get("FLASK_API_BASE_URL", "http://127.0.0.1:5000")
    ).rstrip("/")
    url = base + _CREDENTIAL_PATH + quote(api_name, safe="")
    try:
        with opener(url, timeout=timeout) as response:
            payload = response.read().decode("utf-8")
        body = json.loads(payload)
    except Exception:
        return unavailable
    if not isinstance(body, dict):
        return unavailable
    mode = body.get("mode")
    if mode not in ("unlocked", "degraded"):
        return unavailable
    secret = None
    if mode == "unlocked" and body.get("available") is True:
        value = body.get("secret")
        if isinstance(value, str) and value.strip():
            secret = value
    return {"mode": mode, "secret": secret}
