from app.app import db
from model.util import generate_uuid
from pib_hermes_config.channel import CHANNEL_SMART
from pib_hermes_config.live_session import (
    DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS,
    VOICE_MODE_LIVE,
)

# A new personality needs a name only. Everything else starts from these
# values and is changed afterwards in the Advanced dialog.
DEFAULT_GENDER = "Female"
#: Seconds of silence that end the user's turn.
DEFAULT_PAUSE_THRESHOLD = 0.8
#: Number of earlier messages sent along with a turn.
DEFAULT_MESSAGE_HISTORY = 5


class Personality(db.Model):

    __tablename__ = "personality"

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False)
    personality_id = db.Column(
        db.String(255), nullable=False, default=generate_uuid, unique=True
    )
    gender = db.Column(db.String(255), nullable=False, default=DEFAULT_GENDER)
    description = db.Column(db.String(38000), nullable=True)
    pause_threshold = db.Column(
        db.Float, nullable=False, default=DEFAULT_PAUSE_THRESHOLD
    )
    # Spoken only when the first token is later than the budget. Empty means
    # silence. The assistant never substitutes a phrase of its own.
    thinking_filler = db.Column(db.String(255), nullable=True)
    message_history = db.Column(
        db.Integer, nullable=False, default=DEFAULT_MESSAGE_HISTORY
    )
    stt_engine = db.Column(
        db.String(255),
        nullable=False,
        default="local_whisper",
        server_default="local_whisper",
    )
    # Local Supertone, or the id of a provider row with the tts capability.
    tts_engine = db.Column(
        db.String(255),
        nullable=False,
        default="supertone",
        server_default="supertone",
    )
    chats = db.relationship(
        "Chat", backref="personality", lazy=True, cascade="all,delete"
    )
    assistant_model_id = db.Column(
        db.Integer, db.ForeignKey("assistant_model.id"), nullable=True
    )
    # 'default' or the decimal id of a model row. 'default' is a pointer.
    # The provider follows from that model.
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
    # live or turn_based. The provider's live flag and pinned model decide
    # whether live is actually what the voice button starts.
    voice_mode = db.Column(
        db.String(255),
        nullable=False,
        default=VOICE_MODE_LIVE,
        server_default=VOICE_MODE_LIVE,
    )
    # Seconds of silence after which a live session stops, so it does not
    # keep billing. Applied only to live chats.
    live_idle_timeout = db.Column(
        db.Integer,
        nullable=False,
        default=DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS,
        server_default=str(DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS),
    )
