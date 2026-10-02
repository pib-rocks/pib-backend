"""The catalogue of models Cerebra supports.

This list is the one place those models are maintained. It is updated as
Cerebra evolves, and that is the whole point of it: models are added here
and models are removed here. A removed model keeps no entry and no row. A
personality that still points at a removed row is reported as needing a new
model, and starting a chat on it is refused. It is never rewritten onto a
different model. The settings screen is where it gets a current one.

Each entry is one line so the list stays easy to edit. Where an identifier
is not confirmed, the line carries ``# TODO(confirm id)`` and no invented id.
"""

from __future__ import annotations

import json
from typing import Any, Mapping, NamedTuple

#: Stored on a personality that follows the current default provider.
#: Resolving it is a lookup, so changing which row is default needs no rewrite.
DEFAULT_PROVIDER_REF = "default"

STATUS_ACTIVE = "active"
STATUS_UNCONFIRMED = "unconfirmed"

CAPABILITY_KEYS = ("tools", "images", "live", "stt", "tts")

#: Shown when a chat is started on a personality whose model row is gone.
#: The row carries the name, so there is none to show.
MISSING_MODEL_CHAT_MESSAGE = (
    "This personality's model is no longer available. "
    "Choose a current model in settings before starting a chat."
)

#: pib.Cloud's own model identifier cannot be verified from this repository.
#: This value is provisional and has not been confirmed against the service.
PIB_CLOUD_API_NAME = "pib-cloud"  # TODO(confirm id)


class CatalogueEntry(NamedTuple):
    """One model line in the catalogue.

    ``api_name`` is None until the operator confirms an identifier.
    ``is_default`` marks the default route, pib.Cloud.
    """

    provider: str
    api_name: str | None
    visual_name: str
    tools: bool
    images: bool
    live: bool
    stt: bool
    tts: bool
    status: str
    is_default: bool

    def capabilities(self) -> dict[str, bool]:
        return {
            "tools": self.tools,
            "images": self.images,
            "live": self.live,
            "stt": self.stt,
            "tts": self.tts,
        }


# Columns:
# provider, api_name, visual_name, tools, images, live, stt, tts, status, is_default
# fmt: off
_CATALOGUE_ROWS = (
    ("Google", "gemini-3.8-flash", "Gemini 3.8 Flash", True, True, True, False, False, STATUS_ACTIVE, False),
    ("OpenAI", "gpt-6", "GPT-6", True, True, False, False, False, STATUS_ACTIVE, False),
    ("Anthropic", "claude-sonnet-5-5", "Claude Sonnet 5.5", True, True, False, False, False, STATUS_ACTIVE, False),
    ("pib.Cloud", PIB_CLOUD_API_NAME, "pib.Cloud", True, True, False, False, False, STATUS_ACTIVE, True),  # TODO(confirm id)
    ("OpenAI", "gpt-realtime", "GPT Realtime", False, False, True, False, False, STATUS_UNCONFIRMED, False),  # TODO(confirm id)
    ("Mistral", None, "Mistral", False, False, False, False, False, STATUS_UNCONFIRMED, False),  # TODO(confirm id)
)
# fmt: on

CATALOGUE: tuple[CatalogueEntry, ...] = tuple(
    CatalogueEntry(*row) for row in _CATALOGUE_ROWS
)

_BY_API_NAME = {entry.api_name: entry for entry in CATALOGUE if entry.api_name}

#: The default route. Every personality that stores 'default' resolves to it.
DEFAULT_PROVIDER_API_NAME = next(
    entry.api_name for entry in CATALOGUE if entry.is_default and entry.api_name
)


def active_entries() -> tuple[CatalogueEntry, ...]:
    """The lines that get a row. Unconfirmed lines have none until confirmed."""
    return tuple(
        entry for entry in CATALOGUE if entry.status == STATUS_ACTIVE and entry.api_name
    )


def active_api_names() -> frozenset[str]:
    """Chat ids a row may carry. A row with any other id is removed."""
    return frozenset(entry.api_name for entry in active_entries() if entry.api_name)


def catalogue_entry(api_name: str) -> CatalogueEntry | None:
    """The catalogue line for this chat id, or None when it is not listed."""
    return _BY_API_NAME.get(api_name)


def model_status(api_name: str) -> str:
    """Catalogue status for a chat id. An id with no line is unlisted."""
    entry = catalogue_entry(api_name)
    if entry is None:
        return "unlisted"
    return entry.status


def pins_gemini_live_model(api_name: str) -> bool:
    """True when this chat id's row stores the pinned Gemini live model."""
    entry = catalogue_entry(api_name)
    if entry is None or not entry.live or not entry.api_name:
        return False
    return "gemini" in entry.api_name.lower()


def gemini_live_chat_api_names() -> tuple[str, ...]:
    """Chat ids whose registry row is pinned to the Gemini live model."""
    return tuple(
        entry.api_name
        for entry in CATALOGUE
        if entry.api_name and pins_gemini_live_model(entry.api_name)
    )


def capabilities_for(api_name: str, has_image_support: bool) -> dict[str, bool]:
    """Flags stored on one registry row.

    tools, live, stt and tts come from the catalogue. images follows the row's
    own has_image_support value. A chat id that is not in the catalogue keeps
    tools on and live, stt and tts off. stt and tts stay off for these rows:
    they are chat models, and the local speech engines are not rows.
    """
    entry = catalogue_entry(api_name)
    if entry is None:
        flags = {
            "tools": True,
            "images": False,
            "live": False,
            "stt": False,
            "tts": False,
        }
    else:
        flags = entry.capabilities()
    flags["images"] = bool(has_image_support)
    return flags


def is_registry_default(api_name: str) -> bool:
    return api_name == DEFAULT_PROVIDER_API_NAME


def has_images_capability(capabilities: Any) -> bool:
    """True when this row may be offered for selection."""
    return has_capability(capabilities, "images")


def has_capability(capabilities: Any, key: str) -> bool:
    """True when this row carries the named capability flag."""
    parsed = _as_mapping(capabilities)
    return bool(parsed.get(key))


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
