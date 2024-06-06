"""provides methods for accessing TRYB's public-voice-api"""

from public_api_client import configuration
import requests
from typing import Any, Iterable
import base64
import json
import logging

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] [%(asctime)s] [%(process)d] [%(filename)s:%(lineno)s]: %(message)s",
)

SPEECH_TO_TEXT_URL = (
    configuration.tryb_url_prefix + "/public-api/conversions/speech-to-text"
)
TEXT_TO_SPEECH_URL = (
    configuration.tryb_url_prefix + "/public-api/conversions/text-to-speech"
)
VOICE_ASSISTANT_TEXT_URL = (
    configuration.tryb_url_prefix
    + "/public-api/voice-assistant/text?include-audio=false"
)


def _send_request(
    method: str, url: str, headers: dict[str, str], body: dict[str, Any], stream: bool
):
    try:
        headers["Authorization"] = "Bearer " + configuration.public_api_token
        response = requests.request(
            method=method, url=url, stream=stream, headers=headers, json=body
        )
        response.raise_for_status()
        return response

    except requests.HTTPError as error:
        response: requests.Response = error.response
        headers_without_auth = {
            k: v for k, v in headers.items() if k != "Authorization"
        }
        logging.info(
            f"An Error occured while sending request:\n"
            + f"-----------------------------------------------------\n"
            + f"url: {url}\n"
            + f"method: {method}\n"
            + f"body: {body}\n"
            + f"headers: {headers_without_auth}"
            f"-----------------------------------------------------\n"
            + f"Received following response from the public-api:\n"
            + f"-----------------------------------------------------\n"
            + f"status: {response.status_code}\n"
            f"headers: {response.headers}\n"
            f"content: {response.content}\n"
            + f":::::::::::::::::::::::::::::::::::::::::::::::::::::\n"
        )

        raise Exception("error while sending request to public-api")


def speech_to_text(audio: bytes) -> str:
    """converts binary audio data to text using tryb-public-api"""

    headers = {"Accept": "application/json", "Content-Type": "application/json"}
    body = {"data": base64.encodebytes(audio).decode("ascii")}
    response = _send_request("POST", SPEECH_TO_TEXT_URL, headers, body, False)

    return response.json()["responseText"]


def text_to_speech(text: str, gender: str, language: str) -> Iterable[bytes]:
    """converts text to speech using tryb-public-api"""

    headers = {"Accept": "audio/pcm", "Content-Type": "application/json"}
    body = {"data": text, "personality": {"gender": gender, "language": language}}
    response = _send_request("POST", TEXT_TO_SPEECH_URL, headers, body, True)

    return response.iter_content(chunk_size=None)


def chat_completion(
    text: str,
    description: str,
    image_base64: str | None = None,
    model: str = "gpt-3.5-turbo",
) -> Iterable[str]:
    """
    receive a textual llm response, that takes into account the provided description
    """
    headers = {"Accept": "text/event-stream", "Content-Type": "application/json"}

    # Claude does not accept empty strings as input
    if (text is None) or (text == ""):
        text = "empty"

    body = {"data": text, "personality": {"description": description, "model": model}}

    if image_base64 is not None:
        body["imageBase64"] = image_base64

    response = _send_request("POST", VOICE_ASSISTANT_TEXT_URL, headers, body, True)

    line_bin: bytes
    for line_bin in response.iter_lines():
        line_str = line_bin.decode("utf-8")
        if not line_str.startswith("data:"):
            continue
        yield json.loads(line_str[21:-1])
