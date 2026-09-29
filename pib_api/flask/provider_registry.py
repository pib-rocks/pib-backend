"""Rules for the provider registry.

Selection and control state read capability flags. They do not branch on
provider names. The names below only describe how an existing assistant_model
row is copied into the registry.
"""

from __future__ import annotations

import json
from typing import Any, Mapping

#: Stored on a personality that follows the current default provider.
#: Resolving it is a lookup, so changing which row is default needs no rewrite.
DEFAULT_PROVIDER_REF = "default"

#: Smart is the default channel. hermes-agent is that channel's existing row.
DEFAULT_PROVIDER_API_NAME = "hermes-agent"

#: The voice node already opens a live session for this assistant model only.
LIVE_API_NAME = "gemini-3.5-flash"

CAPABILITY_KEYS = ("tools", "images", "live", "stt", "tts")


def capabilities_for(api_name: str, has_image_support: bool) -> dict[str, bool]:
    """Flags copied onto one registry row.

    images follows the row's existing has_image_support value. The two gpt-4o
    rows already differ on that column. tools is on because the
    OpenAI-compatible cloud endpoint supports tool calling. live is on only
    for the model the voice node already treats as live. stt and tts stay off:
    these rows are chat models, and the local speech engines are not rows.
    """
    return {
        "tools": True,
        "images": bool(has_image_support),
        "live": api_name == LIVE_API_NAME,
        "stt": False,
        "tts": False,
    }


def is_registry_default(api_name: str) -> bool:
    return api_name == DEFAULT_PROVIDER_API_NAME


def has_images_capability(capabilities: Any) -> bool:
    """True when this row may be offered for selection."""
    parsed = _as_mapping(capabilities)
    return bool(parsed.get("images"))


def _as_mapping(capabilities: Any) -> Mapping[str, Any]:
    if capabilities is None:
        return {}
    if isinstance(capabilities, str):
        loaded = json.loads(capabilities)
        if isinstance(loaded, dict):
            return loaded
        return {}
    if isinstance(capabilities, Mapping):
        return capabilities
    return {}
