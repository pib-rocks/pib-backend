"""Cold/warm state of a personality's warm Hermes agent (PR-2130).

A personality whose agent the ``hermes-daemon`` has not built yet pays the whole
cold start - profile provisioning, module imports, tool registration and MCP
discovery - before its first token. Warming builds exactly that agent and runs
no model request, so it costs **no tokens**.

This module is the thin HTTP bridge: the daemon owns the agent cache, and only
the daemon process can answer whether an agent exists. Flask never imports the
Hermes runtime; ``personality_service`` already talks to the daemon for the same
reason.

Two rules shape the payload:

* ``state`` speaks the UI contract: ``warm``, ``cold``, ``not_supported`` (a
  direct-channel personality never uses a Hermes agent) or ``unknown`` (the
  daemon cannot be asked). ``warming`` belongs to the caller's own request in
  flight and is never produced here.
* An unreachable daemon is not a cold personality. It answers ``unknown`` with
  200, so the status chip stays neutral instead of claiming a verdict nobody
  measured.
"""

from __future__ import annotations

import logging
import os
from typing import Any, Dict, Optional

from pib_hermes_config.channel import CHANNEL_DIRECT, effective_channel
from service import personality_service

LOGGER = logging.getLogger(__name__)

DEFAULT_DAEMON_URL = "http://hermes-daemon:8088"
DAEMON_WARM_PATH = "/warm"

STATE_WARM = "warm"
STATE_COLD = "cold"
STATE_NOT_SUPPORTED = "not_supported"
STATE_UNKNOWN = "unknown"

REASON_DAEMON_UNREACHABLE = "daemon_unreachable"
REASON_INVALID_RESPONSE = "invalid_response"
REASON_DIRECT_CHANNEL = "direct_channel"

# The warm-up itself is local work on the robot (~2 s once the profile exists,
# up to ~8.5 s for a profile that is provisioned on the spot).
WARM_TIMEOUT_SECONDS = 120.0
STATE_TIMEOUT_SECONDS = 5.0


def _daemon_base_url() -> str:
    """Daemon base URL, the same environment the profile provisioning uses."""
    return os.environ.get("PIB_HERMES_DAEMON_URL") or DEFAULT_DAEMON_URL


def _daemon_warm_url() -> str:
    return _daemon_base_url().rstrip("/") + DAEMON_WARM_PATH


def _payload(
    personality_id: str,
    *,
    state: str,
    warm: bool = False,
    reason: Optional[str] = None,
    detail: str = "",
    elapsed_ms: Optional[float] = None,
) -> Dict[str, Any]:
    return {
        "personalityId": personality_id,
        "state": state,
        "warm": warm,
        "reason": reason,
        "detail": detail,
        "elapsedMs": elapsed_ms,
    }


def _unknown(personality_id: str, reason: str, detail: str = "") -> Dict[str, Any]:
    return _payload(personality_id, state=STATE_UNKNOWN, reason=reason, detail=detail)


def _read_state(personality_id: str, body: Any) -> Dict[str, Any]:
    """Map the daemon's answer onto this module's contract."""
    if not isinstance(body, dict):
        return _unknown(
            personality_id,
            REASON_INVALID_RESPONSE,
            "The Hermes daemon returned an unexpected body.",
        )
    state = body.get("state")
    if state == STATE_WARM or body.get("warm") is True:
        return _payload(personality_id, state=STATE_WARM, warm=True)
    if state == STATE_COLD:
        return _payload(
            personality_id,
            state=STATE_COLD,
            reason=body.get("reason") if isinstance(body.get("reason"), str) else None,
            elapsed_ms=_as_float(body.get("elapsedMs")),
        )
    return _unknown(
        personality_id,
        REASON_INVALID_RESPONSE,
        "The Hermes daemon did not report a warm or cold state.",
    )


def _as_float(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _requires_hermes(personality_id: str) -> bool:
    """True when this personality runs through the Hermes agent at all."""
    personality = personality_service.get_personality(personality_id)
    return effective_channel(getattr(personality, "channel", None)) != CHANNEL_DIRECT


def personality_warmth(personality_id: str) -> Dict[str, Any]:
    """Read the current state without warming. Never raises."""
    if not _requires_hermes(personality_id):
        return _payload(
            personality_id,
            state=STATE_NOT_SUPPORTED,
            reason=REASON_DIRECT_CHANNEL,
            detail="Direct-channel personalities do not use a Hermes agent.",
        )

    import requests

    try:
        response = requests.get(
            _daemon_warm_url(),
            params={"personality_id": personality_id},
            timeout=STATE_TIMEOUT_SECONDS,
        )
    except Exception as exc:  # noqa: BLE001 - unreachable is a state
        LOGGER.info("Hermes daemon not reachable for warmth: %s", exc)
        return _unknown(
            personality_id,
            REASON_DAEMON_UNREACHABLE,
            "The Hermes daemon did not answer.",
        )

    if response.status_code != 200:
        return _unknown(
            personality_id,
            REASON_DAEMON_UNREACHABLE,
            f"The Hermes daemon answered {response.status_code}.",
        )
    try:
        body = response.json()
    except ValueError:
        return _unknown(
            personality_id,
            REASON_INVALID_RESPONSE,
            "The Hermes daemon returned invalid JSON.",
        )
    return _read_state(personality_id, body)


def warm_personality(
    personality_id: str, timeout: float = WARM_TIMEOUT_SECONDS
) -> Dict[str, Any]:
    """Build the personality's warm agent. Costs no tokens. Never raises."""
    if not _requires_hermes(personality_id):
        return _payload(
            personality_id,
            state=STATE_NOT_SUPPORTED,
            reason=REASON_DIRECT_CHANNEL,
            detail="Direct-channel personalities do not use a Hermes agent.",
        )

    import requests

    try:
        response = requests.post(
            _daemon_warm_url(),
            json={"personality_id": personality_id},
            timeout=timeout,
        )
    except Exception as exc:  # noqa: BLE001 - unreachable is a state
        LOGGER.info("Hermes daemon not reachable for warm-up: %s", exc)
        return _unknown(
            personality_id,
            REASON_DAEMON_UNREACHABLE,
            "The Hermes daemon did not answer.",
        )

    if response.status_code != 200:
        return _unknown(
            personality_id,
            REASON_DAEMON_UNREACHABLE,
            f"The Hermes daemon answered {response.status_code}.",
        )
    try:
        body = response.json()
    except ValueError:
        return _unknown(
            personality_id,
            REASON_INVALID_RESPONSE,
            "The Hermes daemon returned invalid JSON.",
        )
    return _read_state(personality_id, body)
