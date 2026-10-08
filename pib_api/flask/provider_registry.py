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

from pib_hermes_config.local_model import API_NAME as LOCAL_DEVICE_API_NAME

#: Stored on a personality that follows the current default provider.
#: Resolving it is a lookup, so changing which row is default needs no rewrite.
DEFAULT_PROVIDER_REF = "default"

STATUS_ACTIVE = "active"
STATUS_UNCONFIRMED = "unconfirmed"

CAPABILITY_KEYS = ("tools", "images", "live", "stt", "tts")

#: Namespace of a typed, unambiguous model reference: ``model:<id>``.
#: The prefix carries the kind, so a provider-account id can never be read as
#: a model-row id: the numbers of the two sequences overlap, which is what
#: made the bare ``providerRef`` string ambiguous (PR-1928 / PR-1930).
MODEL_REF_PREFIX = "model:"

#: Shown when a typed reference is not ``"default"`` or ``"model:<id>"``.
MODEL_REF_FORM_ERROR = "modelRef must be 'default' or a model reference like 'model:6'."
#: Shown when a typed reference names a row that does not exist.
MODEL_REF_UNKNOWN_ERROR = "modelRef names no model row."
#: Shown when the referenced model row has lost its provider account.
MODEL_REF_PROVIDER_GONE_ERROR = "modelRef names a model whose provider is gone."
#: Shown when the typed reference and a deprecated alias disagree.
MODEL_REF_CONFLICT_ERROR = (
    "modelRef disagrees with a deprecated reference field; send one reference."
)


def format_model_ref(model_id: int) -> str:
    """The typed spelling of a concrete model row, for the wire."""
    return f"{MODEL_REF_PREFIX}{model_id}"


def typed_model_id(value: object) -> int:
    """The row id inside a typed reference, or ValueError when it is not one.

    Only ``model:<positive integer>`` passes. A bare number, an unknown
    namespace such as ``account:5``, and the catalogue ``api_name`` all fail,
    which is the point: a reference with no explicit kind is refused instead
    of guessed.
    """
    text = "" if value is None else str(value).strip()
    if not text.startswith(MODEL_REF_PREFIX):
        raise ValueError(f"not a typed model reference: {value!r}")
    digits = text[len(MODEL_REF_PREFIX) :]
    if not digits.isdigit() or int(digits) < 1:
        raise ValueError(f"not a typed model reference: {value!r}")
    return int(digits)


def model_ref_for_stored(provider_ref: object) -> str:
    """The typed spelling of a stored reference (``"default"`` or a bare id).

    The column keeps the bare row id, which was always unambiguous because it
    is written only after the row was validated; this turns it back into the
    typed wire form. An already-typed value is passed through unchanged.
    """
    if provider_ref is None or str(provider_ref).strip() in ("", DEFAULT_PROVIDER_REF):
        return DEFAULT_PROVIDER_REF
    text = str(provider_ref).strip()
    if text.startswith(MODEL_REF_PREFIX):
        return text
    return format_model_ref(int(text))


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
# A live model is its own line. It is not a flag pinned onto the chat model.
# fmt: off
_CATALOGUE_ROWS = (
    ("Google", "gemini-3.8-flash", "Gemini 3.8 Flash", True, True, False, False, False, STATUS_ACTIVE, False),
    ("Google", "gemini-3.8-live", "Gemini 3.8 Live", True, False, True, False, False, STATUS_ACTIVE, False),
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
    """Catalogue status for a chat id. An id with no line is unlisted.

    The on-device model is not a cloud catalogue line. While its row is
    offered it is active, so the interface can show it with the others.
    """
    if api_name == LOCAL_DEVICE_API_NAME:
        return STATUS_ACTIVE
    entry = catalogue_entry(api_name)
    if entry is None:
        return "unlisted"
    return entry.status


def pins_gemini_live_model(api_name: str) -> bool:
    """Live speech is its own catalogue line, never a pin on a chat row.

    Older migrations still pass the chat id. It does not select a row.
    """
    del api_name
    return False


def gemini_live_chat_api_names() -> tuple[str, ...]:
    """Chat ids that used to carry a hidden live pin. There are none."""
    return tuple(
        entry.api_name
        for entry in CATALOGUE
        if entry.api_name and pins_gemini_live_model(entry.api_name)
    )


def is_listed_model(row: Any) -> bool:
    """A personality may choose an image model, a live model, or the on-device model.

    ``offline`` is set only on the local row. It means the model runs on the
    device and works without a provider key. Cloud rows do not carry it, so
    their place in the list is unchanged.
    """
    capabilities = getattr(row, "capabilities", row)
    return (
        has_images_capability(capabilities)
        or has_capability(capabilities, "live")
        or has_capability(capabilities, "offline")
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


def provider_name_for(api_name: str, visual_name: str | None = None) -> str:
    """Catalogue provider of a chat id. A row with no line keeps its own name."""
    entry = catalogue_entry(api_name)
    if entry is not None:
        return entry.provider
    if visual_name:
        return visual_name
    return api_name


def capabilities_held_by_all(rows: Any) -> dict[str, bool]:
    """Flags that are true on every model of one provider.

    One model means its own flags: they hold for all of that provider's
    models. A provider with none stores every flag off.
    """
    shared = {key: True for key in CAPABILITY_KEYS}
    found = False
    for row in rows:
        found = True
        parsed = _as_mapping(row)
        for key in CAPABILITY_KEYS:
            shared[key] = bool(shared[key] and parsed.get(key))
    if not found:
        return {key: False for key in CAPABILITY_KEYS}
    return shared


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
