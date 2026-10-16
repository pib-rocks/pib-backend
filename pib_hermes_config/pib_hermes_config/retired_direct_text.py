"""The legacy tryb Direct text transport is retired on new software.

That transport was ``POST /bff/tt/be/pub/api/voice-assistant/text`` on
``platform.tryb.ai``. New software must not call it. What stays: Smart/Hermes,
native Live, pib.Cloud, the on-device model, and Direct on a model that still
has a direct generateContent transport - with or without tool calling. The
tool-calling flag chooses whether the turn offers tools; it no longer selects a
transport, so ``tool_calling`` false is a plain chat, not a retirement. Stored
channel, model and tool-calling values are not rewritten. Legacy
speech-to-text and text-to-speech are separate routes and are not retired
here.

The sentence below is what a turn says. The validation string is the stable
API message for a selection that would dispatch to the retired route.
"""

from __future__ import annotations

from pib_hermes_config.channel import CHANNEL_SMART
from pib_hermes_config.live_session import VOICE_MODE_LIVE
from pib_hermes_config.local_model import API_NAME as LOCAL_MODEL_API_NAME

#: Catalogue api_name of pib.Cloud. Same value as
#: ``provider_registry.PIB_CLOUD_API_NAME``; this package does not import Flask.
PIB_CLOUD_MODEL_API_NAME = "pib-cloud"

#: The one Direct model whose ``:generateContent`` transport stays. It serves a
#: plain chat and a tool turn on the same endpoint, so this constant is also the
#: pinned model of the Direct tool loop - the voice side imports it instead of
#: repeating the string, so the two can not drift apart.
DIRECT_TEXT_MODEL_API_NAME = "gemini-3.8-flash"

#: Spoken on a turn that would have used the retired text endpoint.
RETIRED_DIRECT_TEXT_SENTENCE = (
    "Bitte wähle Smart, pib.Cloud, das lokale Modell oder Direct mit dem "
    "unterstützten Modell, denn der Direct-Textweg ist nicht mehr verfügbar."
)

#: Stable validation message. Attached to the field that entered the retired
#: combination: channel or the model reference.
RETIRED_DIRECT_TEXT_VALIDATION = (
    "Direct text chat is retired for this model. "
    "Choose Smart, pib.Cloud, the on-device model, or Direct with a supported "
    "model."
)


def legacy_direct_text_selected(
    *,
    channel: str,
    api_name: str | None,
    voice_mode: str | None = None,
) -> bool:
    """True when a turn would have called the retired tryb text endpoint.

    ``channel`` is the channel the turn would actually use. pib.Cloud, the
    on-device model and the Direct model with a live transport are routed
    before that endpoint, and Smart uses Hermes. What remains - Direct on a
    model without a transport of its own - has no answer any more. The
    tool-calling flag deliberately does not appear here: Direct without tools
    is a plain chat on the supported model.
    """
    if api_name in (
        LOCAL_MODEL_API_NAME,
        PIB_CLOUD_MODEL_API_NAME,
        DIRECT_TEXT_MODEL_API_NAME,
    ):
        return False
    if channel == CHANNEL_SMART or voice_mode == VOICE_MODE_LIVE:
        return False
    return True
