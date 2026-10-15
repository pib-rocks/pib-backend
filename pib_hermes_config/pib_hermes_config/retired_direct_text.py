"""The legacy tryb Direct text transport is retired on new software.

That transport was ``POST /bff/tt/be/pub/api/voice-assistant/text`` on
``platform.tryb.ai``. New software must not call it. Smart/Hermes, native
Live, pib.Cloud, the on-device model, and Direct with tool calling stay.
Stored channel, model and tool-calling values are not rewritten. Legacy
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

#: Spoken on a turn that would have used the retired text endpoint.
RETIRED_DIRECT_TEXT_SENTENCE = (
    "Bitte wähle Smart, pib.Cloud, das lokale Modell oder Direct mit "
    "Werkzeugen, denn der Direct-Textweg ist nicht mehr verfügbar."
)

#: Stable validation message. Attached to the field that entered the retired
#: combination: toolCalling, channel, or the model reference.
RETIRED_DIRECT_TEXT_VALIDATION = (
    "Direct text chat without tool calling is retired. "
    "Choose Smart, pib.Cloud, the on-device model, or Direct with tool calling."
)


def legacy_direct_text_selected(
    *,
    channel: str,
    api_name: str | None,
    tool_calling: bool,
    voice_mode: str | None = None,
) -> bool:
    """True when a turn would have called the retired tryb text endpoint.

    ``channel`` is the channel the turn would actually use. pib.Cloud and the
    on-device model are routed before that endpoint. Smart uses Hermes. Direct
    with tool calling uses the modern tool loop. ``tool_calling`` false on
    Smart, pib.Cloud or the on-device model does not select this transport.
    """
    if tool_calling:
        return False
    if api_name in (LOCAL_MODEL_API_NAME, PIB_CLOUD_MODEL_API_NAME):
        return False
    if channel == CHANNEL_SMART or voice_mode == VOICE_MODE_LIVE:
        return False
    return True
