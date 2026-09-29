from datetime import datetime

from sqlalchemy.orm import validates

from app.app import db

TINKERFORGE_BRICKLET = "tinkerforge_bricklet"
FEETECH_ST_SERIAL = "feetech_st_serial"
ROBSTRIDE_CAN = "robstride_can"

SERVO_BRICKLET = "Servo Bricklet"
SOLID_STATE_RELAY_BRICKLET = "Solid State Relay Bricklet"
RGB_LED_BUTTON_BRICKLET = "RGB LED Button Bricklet"
TINKERFORGE_DEVICE_TYPES = (
    SERVO_BRICKLET,
    SOLID_STATE_RELAY_BRICKLET,
    RGB_LED_BUTTON_BRICKLET,
)

SUPPORTED_CONTROLLER_KINDS = frozenset(
    {
        TINKERFORGE_BRICKLET,
        FEETECH_ST_SERIAL,
        ROBSTRIDE_CAN,
    }
)


class Controller(db.Model):
    __tablename__ = "controller"

    id = db.Column(db.Integer, primary_key=True)
    kind = db.Column(db.String(50), nullable=False)
    device_type = db.Column(db.String(50), nullable=True)
    address = db.Column(db.String(255), nullable=True)
    number = db.Column(db.Integer, nullable=False, unique=True)
    supply_voltage = db.Column(db.Float, nullable=True)
    created_at = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
    updated_at = db.Column(
        db.DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow
    )

    motors = db.relationship("Motor", back_populates="controller", lazy=True)

    @validates("kind")
    def validate_kind(self, _key, kind):
        if kind not in SUPPORTED_CONTROLLER_KINDS:
            raise ValueError(f"Unsupported controller kind: {kind!r}")
        if kind != TINKERFORGE_BRICKLET and self.device_type is not None:
            raise ValueError("device_type must be None for non-Tinkerforge controllers")
        return kind

    @validates("device_type")
    def validate_device_type(self, _key, device_type):
        if device_type is not None and device_type not in TINKERFORGE_DEVICE_TYPES:
            raise ValueError(f"Unsupported Tinkerforge device type: {device_type!r}")
        if (
            device_type is not None
            and self.kind is not None
            and self.kind != TINKERFORGE_BRICKLET
        ):
            raise ValueError("device_type must be None for non-Tinkerforge controllers")
        return device_type
