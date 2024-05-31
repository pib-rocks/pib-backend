import os
from dataclasses import dataclass


@dataclass
class MotorConfig:
    TINKERFORGE_HOST: str = os.getenv("TINKERFORGE_HOST", "localhost")
    TINKERFORGE_PORT: int = int(os.getenv("TINKERFORGE_PORT", 4223))
    FLASK_API: str = os.getenv("FLASK_API_BASE_URL", "http://127.0.0.1:5000")


cfg: MotorConfig = MotorConfig()
