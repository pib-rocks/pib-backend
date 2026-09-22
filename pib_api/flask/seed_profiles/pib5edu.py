"""Seed profile for pib 5 educational robots."""

from seed_profiles.edu_microphone_tuning import MICROPHONE_TUNING
from seed_profiles.edu_motor_parameters import (
    MOTOR_PARAMETER_DEFAULTS,
    MOTOR_PARAMETER_DEVIATIONS,
)
from seed_profiles.pib4edu import (
    MOTOR_MAPPING as PIB4EDU_MOTOR_MAPPING,
    RGB_LED_BUTTON_BRICKLET,
    SERVO_BRICKLET,
    SOLID_STATE_RELAY_BRICKLET,
    TINKERFORGE_BRICKLET,
)
from seed_profiles.profile import ControllerProfile, HardwareProfile

MOTOR_MAPPING = {
    **PIB4EDU_MOTOR_MAPPING,
    "shoulder_vertical_right": (4, 0),
    "shoulder_vertical_left": (4, 1),
    "elbow_left": (4, 2),
    "elbow_right": (4, 3),
}

PROFILE = HardwareProfile(
    variant="pib5edu",
    description="pib 5 educational hardware",
    controllers=(
        ControllerProfile(1, TINKERFORGE_BRICKLET, SERVO_BRICKLET, 7.5),
        ControllerProfile(2, TINKERFORGE_BRICKLET, SERVO_BRICKLET, 7.5),
        ControllerProfile(3, TINKERFORGE_BRICKLET, SERVO_BRICKLET, 7.5),
        ControllerProfile(4, TINKERFORGE_BRICKLET, SERVO_BRICKLET, 12.0),
        ControllerProfile(5, TINKERFORGE_BRICKLET, SOLID_STATE_RELAY_BRICKLET, None),
        ControllerProfile(6, TINKERFORGE_BRICKLET, RGB_LED_BUTTON_BRICKLET, None),
        ControllerProfile(7, TINKERFORGE_BRICKLET, RGB_LED_BUTTON_BRICKLET, None),
        ControllerProfile(8, TINKERFORGE_BRICKLET, RGB_LED_BUTTON_BRICKLET, None),
    ),
    motor_mapping=MOTOR_MAPPING,
    rgb_button_controller_ids=(6, 7, 8),
    motor_parameter_defaults=MOTOR_PARAMETER_DEFAULTS,
    motor_parameter_deviations=MOTOR_PARAMETER_DEVIATIONS,
    microphone_tuning=MICROPHONE_TUNING,
)
