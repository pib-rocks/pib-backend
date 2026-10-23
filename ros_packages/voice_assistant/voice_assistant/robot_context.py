"""Cached body description for Direct LLM system prompts."""

from __future__ import annotations

import logging
import time

from pib_api_client import motion_model_client

ROBOT_CONTEXT_TTL_SEC = 30

_logger = logging.getLogger(__name__)
_cached_prompt: str | None = None
_cached_at: float = 0.0
_fetch_failed_logged = False


def invalidate() -> None:
    global _cached_prompt, _cached_at
    _cached_prompt = None
    _cached_at = 0.0


def get_robot_context() -> str:
    global _cached_prompt, _cached_at, _fetch_failed_logged
    now = time.monotonic()
    if _cached_prompt is not None and (now - _cached_at) < ROBOT_CONTEXT_TTL_SEC:
        return _cached_prompt

    try:
        successful, payload = motion_model_client.get_motion_model()
    except Exception as exc:
        if not _fetch_failed_logged:
            _logger.warning("Robot motion model could not be loaded: %s", exc)
            _fetch_failed_logged = True
        return ""

    if not successful or not isinstance(payload, dict):
        if not _fetch_failed_logged:
            _logger.warning("Robot motion model could not be loaded from the API")
            _fetch_failed_logged = True
        return ""

    prompt = payload.get("prompt")
    if not isinstance(prompt, str) or not prompt.strip():
        _cached_prompt = ""
    else:
        _cached_prompt = prompt
    _cached_at = now
    return _cached_prompt
