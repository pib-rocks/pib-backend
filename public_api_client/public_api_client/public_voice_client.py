"""provides methods for accessing TRYB's public-voice-api"""

import base64
import logging
from typing import Any, Iterable, List, Optional

import requests

from pib_hermes_config.retired_direct_text import RETIRED_DIRECT_TEXT_SENTENCE
from provider_registry import DEFAULT_PROVIDER_API_NAME
from public_api_client import (
    SPEECH_TO_TEXT_URL,
    TEXT_TO_SPEECH_URL,
)

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] [%(asctime)s] [%(process)d] [%(filename)s:%(lineno)s]: %(message)s",
)


class PublicApiChatMessage:
    def __init__(self, content: str, is_user: bool):
        self.content = content
        self.is_user = is_user


def _send_request(
    method: str,
    url: str,
    headers: dict[str, str],
    body: dict[str, Any],
    stream: bool,
    public_api_token: str,
):
    try:
        headers["Authorization"] = "Bearer " + public_api_token
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
            f":::::::::::::::::::::::::::::::::::::::::::::::::::::\n"
            + f"An Error occured while sending request:\n"
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


def speech_to_text(audio: bytes, public_api_token: str) -> str:
    """converts binary audio data to text using tryb-public-api"""

    headers = {"Accept": "application/json", "Content-Type": "application/json"}
    body = {"data": base64.encodebytes(audio).decode("ascii")}
    response = _send_request(
        "POST", SPEECH_TO_TEXT_URL, headers, body, False, public_api_token
    )

    return response.json()["responseText"]


def text_to_speech(
    text: str, gender: str, language: str, public_api_token: str
) -> Iterable[bytes]:
    """converts text to speech using tryb-public-api"""

    headers = {"Accept": "audio/pcm", "Content-Type": "application/json"}
    body = {"data": text, "personality": {"gender": gender, "language": language}}
    response = _send_request(
        "POST", TEXT_TO_SPEECH_URL, headers, body, True, public_api_token
    )

    return response.iter_content(chunk_size=None)


def chat_completion(
    text: str,
    description: str,
    message_history: List[PublicApiChatMessage],
    public_api_token: str,
    image_base64: Optional[str] = None,
    # Catalogue default. The public API's id is not confirmed in this repository.
    model: str = DEFAULT_PROVIDER_API_NAME,  # TODO(confirm id)
) -> Iterable[str]:
    """Retired text transport. The signature stays; no request is sent.

    Callers receive the explicit retirement sentence. A missing token is not
    consulted, so it cannot hide that reason. Speech routes in this module are
    unchanged.
    """
    del text, description, message_history, public_api_token, image_base64, model
    return (RETIRED_DIRECT_TEXT_SENTENCE,)
