"""PC-119: an image a tool captured is stored beside its message, not inside it.

The reason for the split is the read path: ``GET /chat/<id>/messages`` answers
with every message of a chat and no paging, so bytes in the message would ride
along on every load of the history and grow the database by 4/3 of each picture,
forever. These tests hold both halves of that promise - the bytes are served, and
they are nowhere in the payload the chat loads.

The image is deleted with its message and with its chat, and a payload that is
not a picture the browser paints is refused rather than stored.
"""

from __future__ import annotations

import base64

import pytest

from app.app import db
from model.chat_message_image_model import ChatMessageImage
from service import chat_image_service, chat_service


def _jpeg(width: int = 64, height: int = 48) -> bytes:
    """A minimal JPEG: SOI, an SOF0 carrying the dimensions, EOI.

    Enough for the two things this API does with the file itself - it recognises
    the format by the header and reads the size out of the frame header. The
    browser is never asked to decode this fixture.
    """
    sof0 = (
        b"\xff\xc0"
        + (17).to_bytes(2, "big")
        + b"\x08"
        + height.to_bytes(2, "big")
        + width.to_bytes(2, "big")
        + b"\x03\x01\x11\x00\x02\x11\x01\x03\x11\x01"
    )
    return (
        b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
        + sof0
        + b"\xff\xd9"
    )


def _png(width: int = 32, height: int = 16) -> bytes:
    ihdr = (
        b"\x00\x00\x00\x0dIHDR"
        + width.to_bytes(4, "big")
        + height.to_bytes(4, "big")
        + b"\x08\x06\x00\x00\x00"
        + b"\x00\x00\x00\x00"
    )
    return b"\x89PNG\r\n\x1a\n" + ihdr + b"\x00\x00\x00\x00IEND\xaeB`\x82"


def _as_payload(raw: bytes) -> dict:
    return {"imageBase64": base64.b64encode(raw).decode("ascii")}


@pytest.fixture()
def chat_with_message(app_ctx, make_personality):
    """One chat with one stored assistant message, and its ids."""
    personality = make_personality(name="Bildtest")
    chat = chat_service.create_chat(
        {"personality_id": personality.personality_id, "topic": "Bilder"}
    )
    message = chat_service.create_chat_message(
        chat.chat_id, {"is_user": False, "content": "Hier ist ein Bild."}
    )
    db.session.commit()
    return chat.chat_id, message.message_id


# --- storing and serving ---------------------------------------------------


def test_a_jpeg_is_stored_and_served_unchanged(
    app, chat_with_message, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message
    raw = _jpeg()
    client = app.test_client()

    stored = client.post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(raw),
    )
    served = client.get(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images/0"
    )

    assert stored.status_code == 201
    assert served.status_code == 200
    assert served.data == raw
    assert served.mimetype == "image/jpeg"


def test_the_reference_carries_no_bytes(app, chat_with_message, tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message

    response = app.test_client().post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(_jpeg()),
    )
    body = response.get_json()

    assert set(body) == {
        "id",
        "mimeType",
        "width",
        "height",
        "byteSize",
        "index",
        "url",
    }
    assert body["mimeType"] == "image/jpeg"
    assert body["width"] == 64 and body["height"] == 48
    assert body["url"].endswith(f"/messages/{message_id}/images/0")


def test_the_bytes_are_not_in_the_message_payload(
    app, chat_with_message, tmp_path, monkeypatch
):
    """What the chat loads stays small: the history carries the message, not the picture."""
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message
    raw = _jpeg()
    client = app.test_client()
    client.post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(raw),
    )
    db.session.commit()

    payload = client.get(f"/voice-assistant/chat/{chat_id}/messages").get_data(
        as_text=True
    )

    assert base64.b64encode(raw).decode("ascii") not in payload
    assert message_id in payload


def test_a_served_image_can_be_cached_forever(
    app, chat_with_message, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message
    client = app.test_client()
    client.post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(_jpeg()),
    )

    response = client.get(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images/0"
    )

    assert "immutable" in response.headers["Cache-Control"]
    assert response.headers["ETag"]


def test_the_second_image_of_one_message_is_at_index_one(
    app, chat_with_message, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message
    client = app.test_client()
    first, second = _jpeg(64, 48), _jpeg(40, 30)
    client.post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(first),
    )
    client.post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(second),
    )
    db.session.commit()

    one = client.get(f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images/0")
    two = client.get(f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images/1")
    beyond = client.get(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images/2"
    )

    assert one.data == first
    assert two.data == second
    assert beyond.status_code == 404


def test_a_png_is_accepted_with_its_own_size(
    app, chat_with_message, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message

    body = (
        app.test_client()
        .post(
            f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
            json=_as_payload(_png()),
        )
        .get_json()
    )

    assert body["mimeType"] == "image/png"
    assert body["width"] == 32 and body["height"] == 16


# --- refusals --------------------------------------------------------------


def test_something_that_is_not_a_picture_is_refused(
    app, chat_with_message, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message

    response = app.test_client().post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(b"just some text, not an image"),
    )

    assert response.status_code == 400
    assert "JPEG" in response.get_json()["error"]


def test_an_oversized_payload_is_refused(app, chat_with_message, tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    monkeypatch.setattr(chat_image_service, "MAX_IMAGE_BYTES", 1024)
    chat_id, message_id = chat_with_message

    oversized = b"\xff\xd8\xff" + b"\x00" * 2048
    response = app.test_client().post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(oversized),
    )

    assert response.status_code == 400
    assert "more than" in response.get_json()["error"]


def test_a_message_of_another_chat_is_refused(
    app, chat_with_message, make_personality, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    _, message_id = chat_with_message
    other = chat_service.create_chat(
        {
            "personality_id": make_personality(name="Andere").personality_id,
            "topic": "Anders",
        }
    )
    db.session.commit()

    response = app.test_client().post(
        f"/voice-assistant/chat/{other.chat_id}/messages/{message_id}/images",
        json=_as_payload(_jpeg()),
    )

    assert response.status_code == 404


def test_a_missing_image_field_is_refused(
    app, chat_with_message, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message

    response = app.test_client().post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images", json={}
    )

    assert response.status_code == 400


# --- deletion --------------------------------------------------------------


def test_deleting_the_message_deletes_its_images(
    app, chat_with_message, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message
    client = app.test_client()
    client.post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(_jpeg()),
    )
    stored = ChatMessageImage.query.filter_by(message_id=message_id).one()
    path = chat_image_service.path_of(stored)
    assert path.exists()

    client.delete(f"/voice-assistant/chat/{chat_id}/messages/{message_id}")

    assert not path.exists()
    assert ChatMessageImage.query.filter_by(message_id=message_id).count() == 0


def test_deleting_the_chat_deletes_its_images(
    app, chat_with_message, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message
    client = app.test_client()
    client.post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(_jpeg()),
    )
    stored = ChatMessageImage.query.filter_by(message_id=message_id).one()
    path = chat_image_service.path_of(stored)

    client.delete(f"/voice-assistant/chat/{chat_id}")

    assert not path.exists()
    assert ChatMessageImage.query.count() == 0


def test_a_lost_file_answers_404_instead_of_breaking_the_message(
    app, chat_with_message, tmp_path, monkeypatch
):
    """A restored database can point at a file that is gone; the chat stays readable."""
    monkeypatch.setenv("PIB_CHAT_IMAGE_DIR", str(tmp_path / "images"))
    chat_id, message_id = chat_with_message
    client = app.test_client()
    client.post(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images",
        json=_as_payload(_jpeg()),
    )
    stored = ChatMessageImage.query.filter_by(message_id=message_id).one()
    chat_image_service.path_of(stored).unlink()

    image = client.get(
        f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images/0"
    )
    message = client.get(f"/voice-assistant/chat/{chat_id}/messages")

    assert image.status_code == 404
    assert message.status_code == 200
