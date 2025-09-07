import json
import os

try:
    with open("/home/pib/public_api/config.json") as f:
        _data = f.read()
    _config = json.loads(_data)
    tryb_url_prefix = _config["trybUrlPrefix"]
except FileNotFoundError:
    tryb_url_prefix = os.getenv("TRYB_URL_PREFIX")

if not tryb_url_prefix:
    raise RuntimeError(f"no tryb configuration found")

SPEECH_TO_TEXT_URL = f"{tryb_url_prefix}/bff/tt/be/pub/api/conversions/speech-to-text"
TEXT_TO_SPEECH_URL = f"{tryb_url_prefix}/bff/tt/be/pub/api/conversions/text-to-speech"
VOICE_ASSISTANT_TEXT_URL = f"{tryb_url_prefix}/bff/tt/be/pub/api/voice-assistant/text"
LANGCHAIN_PROXY_URL = "http://localhost:9393/voice/text"
