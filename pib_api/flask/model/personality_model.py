from app.app import db
from model.util import generate_uuid
from pib_hermes_config.channel import CHANNEL_SMART


class Personality(db.Model):

    __tablename__ = "personality"

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False)
    personality_id = db.Column(
        db.String(255), nullable=False, default=generate_uuid, unique=True
    )
    gender = db.Column(db.String(255), nullable=False)
    description = db.Column(db.String(38000), nullable=True)
    pause_threshold = db.Column(db.Float, nullable=False)
    message_history = db.Column(db.Integer, nullable=False)
    stt_engine = db.Column(
        db.String(255),
        nullable=False,
        default="local_whisper",
        server_default="local_whisper",
    )
    chats = db.relationship(
        "Chat", backref="personality", lazy=True, cascade="all,delete"
    )
    assistant_model_id = db.Column(
        db.Integer, db.ForeignKey("assistant_model.id"), nullable=True
    )
    # 'default' or the decimal id of a provider row. 'default' is a pointer.
    provider_ref = db.Column(db.String(255), nullable=False)
    # Independent of the provider. Smart is the Hermes agent; Direct is the
    # backend's own completion. The installer flag can force Direct at runtime
    # without rewriting this column.
    channel = db.Column(
        db.String(255),
        nullable=False,
        default=CHANNEL_SMART,
        server_default=CHANNEL_SMART,
    )
    # On by default. Off removes every tool, including capture_image, so a
    # camera frame cannot be attached to the turn.
    tool_calling = db.Column(
        db.Boolean,
        nullable=False,
        default=True,
        server_default="1",
    )
