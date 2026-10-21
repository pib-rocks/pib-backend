"""Storing and serving the images a tool captured during a chat turn (PC-119).

The bytes live in a file, the message carries a reference. See
``model/chat_message_image_model`` for why they are not inlined.

What this module refuses, on purpose:

* **Anything that is not a picture the browser paints.** Only JPEG and PNG are
  accepted; a tool that returns something else is reported as an unsupported
  format rather than stored and served as a download.
* **Anything too large.** A camera frame is stored scaled down (the runtime does
  that before uploading, since it already carries Pillow), and a payload beyond
  the hard ceiling is refused instead of silently filling the robot's disk.
* **A client-supplied path.** The file name is generated here; a request can
  never choose where something is written.

A stored image is deleted with its message and with its chat. A missing file is
not an error to the caller of a delete: the row goes, and the reader answers 404.
"""

from __future__ import annotations

import base64
import binascii
import logging
import os
from pathlib import Path
from typing import Any, Optional, Tuple

from model.chat_message_image_model import ChatMessageImage
from model.util import generate_uuid

LOGGER = logging.getLogger(__name__)

#: Directories are configured for the container; the default sits beside the DB.
IMAGE_DIR_ENV = "PIB_CHAT_IMAGE_DIR"
DEFAULT_IMAGE_DIR = "/app/chat_images"

#: The hard ceiling for the bytes a single upload may carry. The runtime scales
#: a frame down to a fraction of this; the ceiling is what stops a caller that
#: does not.
MAX_IMAGE_BYTES = 4 * 1024 * 1024

#: Header bytes decide the format, not the declared MIME type.
_FORMAT_BY_HEADER = (
    (b"\xff\xd8\xff", "image/jpeg", ".jpg"),
    (b"\x89PNG\r\n\x1a\n", "image/png", ".png"),
)


class UnsupportedImage(ValueError):
    """The payload is not a picture this API stores."""


def image_dir() -> Path:
    """The configured image directory, created on first use."""
    directory = Path(os.environ.get(IMAGE_DIR_ENV, DEFAULT_IMAGE_DIR))
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _decode(image_base64: Any) -> bytes:
    if not isinstance(image_base64, str) or not image_base64.strip():
        raise UnsupportedImage("image data is missing")
    payload = image_base64.strip()
    # A data URI is a common way to hand an image over; accept it unwrapped.
    if payload.startswith("data:") and "," in payload:
        payload = payload.split(",", 1)[1]
    try:
        raw = base64.b64decode(payload, validate=False)
    except (binascii.Error, ValueError) as error:
        raise UnsupportedImage(f"image data is not base64: {error}") from error
    if not raw:
        raise UnsupportedImage("image data is empty")
    if len(raw) > MAX_IMAGE_BYTES:
        raise UnsupportedImage(
            f"image is {len(raw)} bytes, more than the {MAX_IMAGE_BYTES} allowed"
        )
    return raw


def _format_of(raw: bytes) -> Tuple[str, str]:
    for header, mime_type, suffix in _FORMAT_BY_HEADER:
        if raw.startswith(header):
            return mime_type, suffix
    raise UnsupportedImage("image is neither JPEG nor PNG")


def _dimensions(raw: bytes, mime_type: str) -> Tuple[Optional[int], Optional[int]]:
    """Width and height from the file itself, without a decoder dependency.

    A wrong pair is worse than none: the chat only uses them as a hint while the
    browser lays out the picture, so a payload this cannot read returns ``(None,
    None)``.
    """
    try:
        if mime_type == "image/png" and len(raw) >= 24:
            return (
                int.from_bytes(raw[16:20], "big"),
                int.from_bytes(raw[20:24], "big"),
            )
        if mime_type == "image/jpeg":
            index = 2
            while index < len(raw) - 9:
                if raw[index] != 0xFF:
                    index += 1
                    continue
                marker = raw[index + 1]
                if marker in (
                    0xC0,
                    0xC1,
                    0xC2,
                    0xC3,
                    0xC5,
                    0xC6,
                    0xC7,
                    0xC9,
                    0xCA,
                    0xCB,
                ):
                    height = int.from_bytes(raw[index + 5 : index + 7], "big")
                    width = int.from_bytes(raw[index + 7 : index + 9], "big")
                    return width, height
                segment = int.from_bytes(raw[index + 2 : index + 4], "big")
                index += 2 + segment
    except (IndexError, ValueError):  # noqa: BLE001 - a hint, not a fact
        return None, None
    return None, None


def store(message_id: str, image_base64: Any) -> ChatMessageImage:
    """Keep one image for one message and answer its row. Raises UnsupportedImage."""
    from app.app import db

    raw = _decode(image_base64)
    mime_type, suffix = _format_of(raw)
    width, height = _dimensions(raw, mime_type)
    file_name = f"{generate_uuid()}{suffix}"
    directory = image_dir()
    path = directory / file_name
    path.write_bytes(raw)

    sequence = ChatMessageImage.query.filter_by(message_id=message_id).count()
    image = ChatMessageImage(
        message_id=message_id,
        sequence=sequence,
        mime_type=mime_type,
        byte_size=len(raw),
        width=width,
        height=height,
        file_name=file_name,
    )
    db.session.add(image)
    db.session.flush()
    LOGGER.info(
        "stored chat image message=%s bytes=%d mime=%s", message_id, len(raw), mime_type
    )
    return image


def image_of(message_id: str, index: int) -> Optional[ChatMessageImage]:
    """The ``index``-th stored image of one message, oldest first."""
    if index < 0:
        return None
    return ChatMessageImage.query.filter_by(
        message_id=message_id, sequence=index
    ).one_or_none()


def path_of(image: ChatMessageImage) -> Path:
    return image_dir() / image.file_name


def read_bytes(image: ChatMessageImage) -> Optional[bytes]:
    """The stored bytes, or None when the file is gone (a restored backup)."""
    try:
        return path_of(image).read_bytes()
    except OSError as error:
        LOGGER.warning("chat image %s cannot be read: %s", image.id, error)
        return None


def public_reference(image: ChatMessageImage, index: int) -> dict:
    """What the message meta and the chat client see: a reference, never bytes."""
    return {
        "id": image.id,
        "mimeType": image.mime_type,
        "width": image.width,
        "height": image.height,
        "byteSize": image.byte_size,
        "index": index,
    }


def _unlink(image: ChatMessageImage) -> None:
    try:
        path_of(image).unlink()
    except FileNotFoundError:
        pass
    except OSError as error:
        LOGGER.warning("chat image %s left on disk: %s", image.id, error)


def delete_for_message(message_id: str) -> int:
    """Drop every image of one message, with its file. Answers how many."""
    from app.app import db

    images = ChatMessageImage.query.filter_by(message_id=message_id).all()
    for image in images:
        _unlink(image)
        db.session.delete(image)
    if images:
        db.session.flush()
    return len(images)


def delete_for_chat(chat_id: str) -> int:
    """Drop every image of a chat, with its files. Answers how many."""
    from app.app import db
    from model.chat_message_model import ChatMessage

    message_ids = [
        row.message_id for row in ChatMessage.query.filter_by(chat_id=chat_id).all()
    ]
    if not message_ids:
        return 0
    images = ChatMessageImage.query.filter(
        ChatMessageImage.message_id.in_(message_ids)
    ).all()
    for image in images:
        _unlink(image)
        db.session.delete(image)
    if images:
        db.session.flush()
    return len(images)
