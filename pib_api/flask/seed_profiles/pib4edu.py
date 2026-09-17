"""Seed profile for pib 4 educational robots."""

from seed_profiles.edu_motor_parameters import (
    MOTOR_PARAMETER_DEFAULTS,
    MOTOR_PARAMETER_DEVIATIONS,
)
from seed_profiles.profile import ControllerProfile, HardwareProfile

TINKERFORGE_BRICKLET = "tinkerforge_bricklet"
SERVO_BRICKLET = "Servo Bricklet"
SOLID_STATE_RELAY_BRICKLET = "Solid State Relay Bricklet"
RGB_LED_BUTTON_BRICKLET = "RGB LED Button Bricklet"

MOTOR_MAPPING = {
    "turn_head_motor": (2, 4),
    "tilt_forward_motor": (2, 5),
    "upper_arm_left_rotation": (3, 9),
    "elbow_left": (3, 8),
    "lower_arm_left_rotation": (3, 7),
    "shoulder_vertical_left": (2, 9),
    "shoulder_horizontal_left": (2, 8),
    "upper_arm_right_rotation": (1, 9),
    "elbow_right": (1, 8),
    "lower_arm_right_rotation": (1, 7),
    "shoulder_vertical_right": (2, 1),
    "shoulder_horizontal_right": (2, 0),
    "thumb_right_opposition": (1, 0),
    "thumb_right_stretch": (1, 1),
    "index_right_stretch": (1, 2),
    "middle_right_stretch": (1, 3),
    "ring_right_stretch": (1, 4),
    "pinky_right_stretch": (1, 5),
    "thumb_left_opposition": (3, 0),
    "thumb_left_stretch": (3, 1),
    "index_left_stretch": (3, 2),
    "middle_left_stretch": (3, 3),
    "ring_left_stretch": (3, 4),
    "pinky_left_stretch": (3, 5),
    "wrist_left": (3, 6),
    "wrist_right": (1, 6),
}

PROFILE = HardwareProfile(
    variant="pib4edu",
    description="pib 4 educational hardware",
    controllers=(
        ControllerProfile(1, TINKERFORGE_BRICKLET, SERVO_BRICKLET, 7.5),
        ControllerProfile(2, TINKERFORGE_BRICKLET, SERVO_BRICKLET, 7.5),
        ControllerProfile(3, TINKERFORGE_BRICKLET, SERVO_BRICKLET, 7.5),
        ControllerProfile(4, TINKERFORGE_BRICKLET, SOLID_STATE_RELAY_BRICKLET, None),
        ControllerProfile(5, TINKERFORGE_BRICKLET, RGB_LED_BUTTON_BRICKLET, None),
        ControllerProfile(6, TINKERFORGE_BRICKLET, RGB_LED_BUTTON_BRICKLET, None),
        ControllerProfile(7, TINKERFORGE_BRICKLET, RGB_LED_BUTTON_BRICKLET, None),
    ),
    motor_mapping=MOTOR_MAPPING,
    rgb_button_controller_ids=(5, 6, 7),
    motor_parameter_defaults=MOTOR_PARAMETER_DEFAULTS,
    motor_parameter_deviations=MOTOR_PARAMETER_DEVIATIONS,
)
