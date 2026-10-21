"""One image a tool captured during a turn, stored beside its chat message (PC-119).

A tool that takes a picture - the camera tool returns ``imageBase64``/``mimeType``
- produces a payload of a few hundred kilobytes. The chat message itself keeps
only a reference: ``meta["images"]`` carries ``id``, ``mimeType``, ``width`` and
``height``, and the bytes live in a file next to it.

The message row is the wrong place for them. ``GET /chat/<id>/messages`` answers
with every message of a chat and no paging, so inline base64 would ride along on
every load of the history, and the SQLite row (and every database backup) would
grow by 4/3 of each picture, forever.

An image without its message is useless, so the row is deleted with the message
(and with the chat); ``chat_image_service`` removes the file in the same step.
"""

from app.app import db
from model.util import generate_uuid
from sqlalchemy.sql import func


class ChatMessageImage(db.Model):
    __tablename__ = "chatMessageImage"

    id = db.Column(db.String(255), primary_key=True, default=generate_uuid)
    #: Position among the images of this message, 0-based. The URL a chat client
    #: uses ends in this number, so it has to be stable: a timestamp alone ties
    #: two images stored in the same second, and the order would come from a UUID.
    sequence = db.Column(db.Integer, nullable=False, default=0)
    #: The message this image belongs to. ``chatMessage.message_id`` is the
    #: string id the API speaks, and it is what a client references.
    message_id = db.Column(db.String(255), nullable=False, index=True)
    #: Only formats the browser paints as a picture without a plugin.
    mime_type = db.Column(db.String(64), nullable=False)
    #: Size of the stored bytes, so a reader can refuse a file that changed.
    byte_size = db.Column(db.Integer, nullable=False)
    width = db.Column(db.Integer, nullable=True)
    height = db.Column(db.Integer, nullable=True)
    #: File name inside the image directory; the path is never client-supplied.
    file_name = db.Column(db.String(255), nullable=False)
    created_at = db.Column(db.DateTime, nullable=False, default=func.now())
