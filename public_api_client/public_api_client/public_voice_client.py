"""provides methods for accessing TRYB's public-voice-api"""

import base64
import json
import logging
from typing import Any, Iterable, List, Optional

import requests

from public_api_client import (
    SPEECH_TO_TEXT_URL,
    TEXT_TO_SPEECH_URL,
    VOICE_ASSISTANT_TEXT_URL,
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
    model: str = "gpt-3.5-turbo",
) -> Iterable[str]:
    """
    Liest SSE vom Server und yieldet die Delta-Tokens.
    """
    import logging

    headers = {
        "Accept": "text/event-stream",
        "Content-Type": "application/json",
    }
    if public_api_token:  # optional
        headers["Authorization"] = f"Bearer {public_api_token}"

    if not text:
        text = "echo 'I could not hear you, please repeat your message.'"

    body = {
        "data": text,
        "messageHistory": [
            {"content": m.content, "isUser": m.is_user} for m in message_history
        ],
        "personality": {"description": description, "model": model},
    }
    if image_base64 is not None:
        body["imageBase64"] = image_base64

    # WICHTIG: keine JSON-URL! Der Server streamt SSE unter /voice/text
    response = _send_request(
        "POST",
        "http://andi-desktop:9393/voice/text",
        headers,
        body,
        True,                  # stream=True
        public_api_token,
    )

    # Optional: zur Diagnose einmal den Content-Type loggen
    logging.info("public-api CT: %s", getattr(response, "headers", {}).get("content-type"))

    # SSE lesen: nur data:-Zeilen parsen
    for line_bin in response.iter_lines():
        if not line_bin:
            continue
        line_str = line_bin.decode("utf-8", "replace").strip()
        # Kommentare ": keep-alive" ignorieren
        if not line_str.startswith("data: "):
            continue

        try:
            evt = json.loads(line_str[6:])  # korrekt: alles nach "data: "
        except json.JSONDecodeError:
            # Vorschau loggen und weiter
            logging.warning("Non-JSON SSE line: %r", line_str[:120])
            continue

        typ = evt.get("type")
        if typ == "answer":
            yield evt.get("delta", "")
        elif typ == "error":
            logging.error("SSE error: %s", evt.get("error"))
            break
        elif typ == "done":
            break

