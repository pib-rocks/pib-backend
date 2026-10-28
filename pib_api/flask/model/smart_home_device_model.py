import json

from app.app import db


class SmartHomeDevice(db.Model):
    __tablename__ = "smart_home_device"

    id = db.Column(db.Integer, primary_key=True)
    entity_id = db.Column(db.String(255), nullable=False, unique=True)
    domain = db.Column(db.String(64), nullable=False)
    friendly_name = db.Column(db.String(255), nullable=False)
    room = db.Column(db.String(128), nullable=False, default="General")
    expose_to_voice = db.Column(db.Boolean, nullable=False, default=True)
    expose_to_programs = db.Column(db.Boolean, nullable=False, default=True)
    cached_state = db.Column(db.String(255), nullable=False, default="unknown")
    attributes_json = db.Column(db.Text, nullable=False, default="{}")

    @property
    def attributes(self) -> dict:
        try:
            return json.loads(self.attributes_json or "{}")
        except json.JSONDecodeError:
            return {}

    @attributes.setter
    def attributes(self, value: dict) -> None:
        self.attributes_json = json.dumps(value or {})
