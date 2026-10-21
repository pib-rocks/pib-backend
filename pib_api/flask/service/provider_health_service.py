"""Reachability probe for the provider a personality is pointed at (PC-115).

The voice-assistant status strip names the provider that will answer a turn.
Before the first turn nothing in the browser knows whether that provider is
usable, so Cerebra asks this service once when a personality is opened.

Three rules decide the shape of this module:

* The probe runs here, in pib-backend, never in the browser. The provider
  credentials live in the robot key store and nowhere else, and the cloud
  endpoint answers a browser preflight with 405.
* **Green means "this personality can talk to its provider now".** For
  pib.Cloud that is three facts together: the service answers, the stored
  SmartConnect token is valid, and the token still has budget left. A live
  ``/health`` alone would paint a chip green for a robot whose token was
  revoked or whose budget is spent.
* A probe that cannot run is not a failure. No stored token, a locked key
  store, and a provider whose catalogue row carries no endpoint all report
  ``unknown``; only a request that was attempted and failed reports
  ``unreachable``. The UI paints ``unknown`` neutral, so nothing merely
  unknown can look like an outage.

No provider host is hardcoded here: the endpoint comes from the catalogue row
(``Provider.endpoint_base``), because a production module may not name a model
or an account the shipped catalogue does not contain
(``test_production_models_are_in_the_catalogue``). A third-party row without a
configured endpoint is therefore ``unknown``/``no_endpoint``, not red; a probe
descriptor in the catalogue can extend that later.
"""

from __future__ import annotations

import json
import logging
import os
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, Tuple

from model.provider_model import RegistryModel
from service import key_store_service, provider_service

LOGGER = logging.getLogger(__name__)

STATE_HEALTHY = "healthy"
STATE_UNREACHABLE = "unreachable"
STATE_UNKNOWN = "unknown"

# A status strip is not worth a long wait; a provider that answers slower than
# this is reported unreachable rather than blocking the page.
DEFAULT_TIMEOUT_SECONDS = 3.0
TIMEOUT_ENV = "PIB_PROVIDER_HEALTH_TIMEOUT"

# The cloud base the voice runtime already uses. ``PIB_CLOUD_BASE_URL``
# overrides it, and a catalogue endpoint wins over both.
DEFAULT_CLOUD_BASE = "https://api.pib.rocks/v1"
CLOUD_BASE_ENV = "PIB_CLOUD_BASE_URL"

# What the probe actually did, so a caller can tell the checks apart.
PROBE_CLOUD_USAGE = "cloud_usage"
PROBE_HEALTH = "health"
PROBE_MODELS = "models"

# Reason codes carried in the payload, so the UI can explain a chip.
REASON_NO_KEY = "no_key"
REASON_KEY_STORE_LOCKED = "key_store_locked"
REASON_NO_ENDPOINT = "no_endpoint"
REASON_UNSUPPORTED = "unsupported"
REASON_UNREACHABLE = "unreachable"
REASON_HTTP_STATUS = "http_status"
REASON_KEY_REJECTED = "key_rejected"
REASON_TOKEN_INVALID = "token_invalid"
REASON_BUDGET_EXHAUSTED = "budget_exhausted"


def _timeout_seconds() -> float:
    configured = os.environ.get(TIMEOUT_ENV, "").strip()
    if not configured:
        return DEFAULT_TIMEOUT_SECONDS
    try:
        value = float(configured)
    except ValueError:
        LOGGER.warning("%s is not a number: %r", TIMEOUT_ENV, configured)
        return DEFAULT_TIMEOUT_SECONDS
    return value if value > 0 else DEFAULT_TIMEOUT_SECONDS


def _origin(url: str) -> str:
    """Scheme and host of ``url``, without a path, a query, or a trailing slash."""
    without_scheme = url.split("://", 1)
    if len(without_scheme) != 2:
        return url.rstrip("/")
    scheme, rest = without_scheme
    host = rest.split("/", 1)[0]
    return f"{scheme}://{host}"


def _family(provider_name: Optional[str], api_name: str) -> str:
    """Which probe a row belongs to.

    Only the rows whose endpoint the robot itself defines are probed in their
    own way: the cloud (a liveness route plus the token's usage) and the
    on-device model (Ollama's tag listing, which needs no credential).
    Everything else is a generic models listing on the row's endpoint.
    """
    name = (provider_name or "").strip().lower()
    api = (api_name or "").strip().lower()
    if name == "pib.cloud" or api == "pib-cloud":
        return "cloud"
    if name == "local" or api == "qwen-fast":
        return "local"
    return "generic"


def _cloud_base(endpoint_base: Optional[str]) -> str:
    if endpoint_base:
        return endpoint_base.rstrip("/")
    configured = os.environ.get(CLOUD_BASE_ENV, "").strip()
    if configured:
        return configured.rstrip("/")
    return DEFAULT_CLOUD_BASE


def _models_probe(
    provider_name: Optional[str],
    endpoint_base: Optional[str],
    secret: Optional[str],
) -> Optional[Tuple[str, Dict[str, str]]]:
    """A models listing on the row's own endpoint, or None when it has none.

    A Google account takes its key as a query parameter, every other account as
    a bearer token, which is what its listing accepts.
    """
    base = (endpoint_base or "").rstrip("/")
    if not base:
        return None
    name = (provider_name or "").strip().lower()
    if name == "google":
        separator = "&" if "?" in base else "?"
        suffix = f"{separator}key={secret}" if secret else ""
        return f"{base}/models{suffix}", {}
    headers = {"Authorization": f"Bearer {secret}"} if secret else {}
    return f"{base}/models", headers


def _payload(
    model: RegistryModel,
    state: str,
    reason: Optional[str] = None,
    detail: str = "",
    status: Optional[int] = None,
    latency_ms: Optional[int] = None,
    probe: Optional[str] = None,
    budget_remaining: Optional[float] = None,
    budget_known: bool = False,
) -> dict:
    provider = model.provider
    return {
        "state": state,
        "provider": provider.name if provider is not None else None,
        "model": model.visual_name,
        "apiName": model.api_name,
        "probe": probe,
        "reason": reason,
        "detail": detail,
        "status": status,
        "latencyMs": latency_ms,
        "budgetRemaining": budget_remaining,
        "budgetKnown": budget_known,
    }


def _missing_key_reason() -> str:
    """Why a key could not be read: a locked store or simply no key."""
    if key_store_service.encryption_enabled() and (
        key_store_service.operating_mode() != key_store_service.MODE_UNLOCKED
    ):
        return REASON_KEY_STORE_LOCKED
    return REASON_NO_KEY


def _http_get(url: str, headers: Dict[str, str]) -> Tuple[Optional[int], Any, str, int]:
    """One GET, without raising: ``(status, body, error text, latency ms)``."""
    request = urllib.request.Request(url, headers=headers, method="GET")
    started = time.monotonic()
    try:
        with urllib.request.urlopen(request, timeout=_timeout_seconds()) as response:
            raw = response.read()
            status = response.status
    except urllib.error.HTTPError as error:
        latency_ms = int((time.monotonic() - started) * 1000)
        try:
            body = json.loads(error.read().decode("utf-8", "replace"))
        except Exception:  # noqa: BLE001 - an error body is optional
            body = None
        return error.code, body, "", latency_ms
    except Exception as error:  # noqa: BLE001 - any transport failure is unreachable
        latency_ms = int((time.monotonic() - started) * 1000)
        return None, None, f"{type(error).__name__}: {error}", latency_ms

    latency_ms = int((time.monotonic() - started) * 1000)
    try:
        payload = json.loads(raw.decode("utf-8", "replace")) if raw else None
    except ValueError:
        payload = None
    return status, payload, "", latency_ms


def _budget_state(payload: Any) -> Tuple[Optional[float], bool]:
    """Remaining budget in pib.Tokens and whether the figure is known.

    ``None`` with ``known`` False means the payload said nothing about a
    budget; ``None`` with ``known`` True means the token has no ceiling.
    """
    if not isinstance(payload, dict):
        return None, False
    budget = payload.get("budget")
    if not isinstance(budget, dict):
        return None, False
    if "remaining" not in budget:
        return None, False
    remaining = budget.get("remaining")
    if remaining is None:
        # A token without a budget ceiling: valid, nothing to spend out.
        return None, True
    if isinstance(remaining, bool) or not isinstance(remaining, (int, float)):
        return None, False
    return float(remaining), True


def _probe_cloud(model: RegistryModel, secret: Optional[str]) -> dict:
    """Reachable, token valid and budget left - all three, or not green.

    Two requests: the service's own liveness route (no credential, so a dead
    service is told apart from a rejected token) and the token's usage route,
    which refuses an invalid or revoked token and carries the remaining budget
    in its body.
    """
    provider = model.provider
    endpoint_base = provider.endpoint_base if provider is not None else None
    base = _cloud_base(endpoint_base)
    origin = _origin(base)

    status, _body, error, latency_ms = _http_get(f"{origin}/health", {})
    if status is None:
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_UNREACHABLE,
            f"The cloud did not answer: {error}",
            latency_ms=latency_ms,
            probe=PROBE_HEALTH,
        )
    if not 200 <= status < 300:
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_HTTP_STATUS,
            f"The cloud answered {status} on /health.",
            status=status,
            latency_ms=latency_ms,
            probe=PROBE_HEALTH,
        )

    if not secret:
        return _payload(
            model,
            STATE_UNKNOWN,
            _missing_key_reason(),
            "The cloud answers, but no SmartConnect token is stored.",
            status=status,
            latency_ms=latency_ms,
            probe=PROBE_HEALTH,
        )

    usage_status, usage_body, usage_error, usage_latency = _http_get(
        f"{base}/usage", {"Authorization": f"Bearer {secret}"}
    )
    if usage_status is None:
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_UNREACHABLE,
            f"The usage request failed: {usage_error}",
            status=status,
            latency_ms=usage_latency,
            probe=PROBE_CLOUD_USAGE,
        )
    if usage_status == 402:
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_BUDGET_EXHAUSTED,
            "The token has no budget left.",
            status=usage_status,
            latency_ms=usage_latency,
            probe=PROBE_CLOUD_USAGE,
        )
    if usage_status in (401, 403):
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_TOKEN_INVALID,
            "The stored SmartConnect token is not valid.",
            status=usage_status,
            latency_ms=usage_latency,
            probe=PROBE_CLOUD_USAGE,
        )
    if not 200 <= usage_status < 300:
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_HTTP_STATUS,
            f"The usage request answered {usage_status}.",
            status=usage_status,
            latency_ms=usage_latency,
            probe=PROBE_CLOUD_USAGE,
        )

    revoked = False
    if isinstance(usage_body, dict):
        token_block = usage_body.get("token")
        if isinstance(token_block, dict):
            revoked = token_block.get("revoked") is True
    if revoked:
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_TOKEN_INVALID,
            "The stored SmartConnect token has been revoked.",
            status=usage_status,
            latency_ms=usage_latency,
            probe=PROBE_CLOUD_USAGE,
        )

    remaining, known = _budget_state(usage_body)
    if known and remaining is not None and remaining <= 0:
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_BUDGET_EXHAUSTED,
            "The token has no budget left.",
            status=usage_status,
            latency_ms=usage_latency,
            probe=PROBE_CLOUD_USAGE,
            budget_remaining=remaining,
            budget_known=True,
        )

    return _payload(
        model,
        STATE_HEALTHY,
        status=usage_status,
        latency_ms=usage_latency,
        probe=PROBE_CLOUD_USAGE,
        budget_remaining=remaining,
        budget_known=known,
    )


def _probe_local(model: RegistryModel, endpoint_base: Optional[str]) -> dict:
    """Ollama's tag listing. The stored endpoint is the OpenAI-compatible base."""
    base = (endpoint_base or "").rstrip("/")
    if not base:
        return _payload(
            model,
            STATE_UNKNOWN,
            REASON_NO_ENDPOINT,
            "The catalogue carries no endpoint for the on-device model.",
        )
    if base.endswith("/v1"):
        base = base[: -len("/v1")]
    status, _body, error, latency_ms = _http_get(f"{base}/api/tags", {})
    if status is None:
        LOGGER.info("Provider probe for %s failed: %s", model.api_name, error)
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_UNREACHABLE,
            error,
            latency_ms=latency_ms,
            probe=PROBE_MODELS,
        )
    if 200 <= status < 300:
        return _payload(
            model,
            STATE_HEALTHY,
            status=status,
            latency_ms=latency_ms,
            probe=PROBE_MODELS,
        )
    return _payload(
        model,
        STATE_UNREACHABLE,
        REASON_HTTP_STATUS,
        f"The on-device model answered {status}.",
        status=status,
        latency_ms=latency_ms,
        probe=PROBE_MODELS,
    )


def probe_model(model_id: int) -> Optional[dict]:
    """Probe the provider of one catalogue row.

    ``None`` means the row does not exist. A refused probe is a payload with
    ``state`` ``unknown``, never an exception: the status strip is not a place
    where a provider error may break the page.
    """
    model = provider_service.get_model_by_id(model_id)
    if model is None:
        return None

    provider = model.provider
    provider_name = provider.name if provider is not None else None
    endpoint_base = provider.endpoint_base if provider is not None else None
    family = _family(provider_name, model.api_name)

    if family == "cloud":
        return _probe_cloud(
            model, key_store_service.unlocked_secret_for_api_name(model.api_name)
        )
    if family == "local":
        return _probe_local(model, endpoint_base)

    secret = key_store_service.unlocked_secret_for_api_name(model.api_name)
    target = _models_probe(provider_name, endpoint_base, secret)
    if target is None:
        return _payload(
            model,
            STATE_UNKNOWN,
            REASON_NO_ENDPOINT,
            "The catalogue carries no endpoint for this provider.",
        )
    if not secret:
        return _payload(model, STATE_UNKNOWN, _missing_key_reason())

    url, headers = target
    status, _body, error, latency_ms = _http_get(url, headers)
    if status is None:
        LOGGER.info("Provider probe for %s failed: %s", model.api_name, error)
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_UNREACHABLE,
            error,
            latency_ms=latency_ms,
            probe=PROBE_MODELS,
        )
    # A keyed listing answers 401/403 when the stored key is wrong: the
    # provider is reachable, but this personality cannot use it, so the chip
    # is red with a reason of its own.
    if status in (401, 403):
        return _payload(
            model,
            STATE_UNREACHABLE,
            REASON_KEY_REJECTED,
            f"The provider answered {status}; the stored key was rejected.",
            status=status,
            latency_ms=latency_ms,
            probe=PROBE_MODELS,
        )
    if 200 <= status < 300:
        return _payload(
            model,
            STATE_HEALTHY,
            status=status,
            latency_ms=latency_ms,
            probe=PROBE_MODELS,
        )
    return _payload(
        model,
        STATE_UNREACHABLE,
        REASON_HTTP_STATUS,
        f"The provider answered {status}.",
        status=status,
        latency_ms=latency_ms,
        probe=PROBE_MODELS,
    )
