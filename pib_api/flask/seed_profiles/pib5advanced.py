"""Seed profile for a pib 5 advanced robot with serial motor controllers.

This variant is wired differently from the educational robots. The motors sit on
four serial controllers - Waveshare adapters on a Feetech ST bus - and the
Tinkerforge HAT carries only the solid-state relay and up to three RGB LED
buttons. There is no servo bricklet.

Every controller starts **without an address**. The identifiers of the serial
adapters are read separately and entered later, either through the hardware-IDs
page or through an exported hardware config (``controllers[].address`` accepts a
free-form string for a non-Tinkerforge controller). ``_upsert_controllers``
writes an address only when it creates a row, so nothing that was entered later
is overwritten by the next start.
"""

from seed_profiles.edu_microphone_tuning import MICROPHONE_TUNING
from seed_profiles.edu_motor_parameters import (
    FINGER_MOTOR_PARAMETERS,
    INVERTED_FINGER_MOTOR_PARAMETERS,
    MOTOR_PARAMETER_DEFAULTS,
)
from seed_profiles.pib4edu import (
    RGB_LED_BUTTON_BRICKLET,
    SOLID_STATE_RELAY_BRICKLET,
    TINKERFORGE_BRICKLET,
)
from seed_profiles.profile import ControllerProfile, HardwareProfile

#: The serial bus the four Waveshare adapters speak. The name has to match a kind
#: ``Controller`` accepts, otherwise the seed raises on the first controller.
FEETECH_ST_SERIAL = "feetech_st_serial"

CONTROLLER_NUMBERS_SERIAL = (1, 2, 3, 4)
CONTROLLER_NUMBER_RELAY = 5
CONTROLLER_NUMBERS_RGB_BUTTONS = (6, 7, 8)

#: Mapping of the 26 joints to (controller_number 1..4, Feetech STS servo ID),
#: measured on the reference pib5advanced robot (192.168.1.56):
#: - Controller 1: right hand + shoulder_vertical_right + turn_head_motor
#: - Controller 2: right arm
#: - Controller 3: left hand + shoulder_vertical_left + tilt_forward_motor
#: - Controller 4: left arm
MOTOR_MAPPING = {
    "turn_head_motor": (1, 51),
    "tilt_forward_motor": (3, 50),
    "upper_arm_left_rotation": (4, 39),
    "elbow_left": (4, 38),
    "lower_arm_left_rotation": (4, 37),
    "shoulder_vertical_left": (3, 41),
    "shoulder_horizontal_left": (4, 40),
    "upper_arm_right_rotation": (2, 19),
    "elbow_right": (2, 18),
    "lower_arm_right_rotation": (2, 17),
    "shoulder_vertical_right": (1, 21),
    "shoulder_horizontal_right": (2, 20),
    "thumb_right_opposition": (1, 10),
    "thumb_right_stretch": (1, 11),
    "index_right_stretch": (1, 12),
    "middle_right_stretch": (1, 13),
    "ring_right_stretch": (1, 14),
    "pinky_right_stretch": (1, 15),
    "thumb_left_opposition": (3, 30),
    "thumb_left_stretch": (3, 31),
    "index_left_stretch": (3, 32),
    "middle_left_stretch": (3, 33),
    "ring_left_stretch": (3, 34),
    "pinky_left_stretch": (3, 35),
    "wrist_left": (4, 36),
    "wrist_right": (2, 16),
}

#: The ``upper_endpoint`` values below mirror the measured configuration of the
#: reference pib5advanced robot (192.168.10.217, 2026-10-23): twelve motors drive
#: their positive end towards the rotation-range minimum. A fresh install of this
#: variant must therefore move in the same direction as that robot - keep the two
#: in sync, and change them only together with a measurement on the hardware.
MOTOR_PARAMETER_DEVIATIONS = {
    "tilt_forward_motor": {
        "rotation_range_min": -2000,
        "rotation_range_max": 4500,
        "upper_endpoint": "min",
    },
    "upper_arm_left_rotation": {"velocity": 10000, "upper_endpoint": "min"},
    "elbow_left": {
        "rotation_range_min": -4500,
        "rotation_range_max": 9000,
        "velocity": 10000,
        "upper_endpoint": "min",
    },
    "lower_arm_left_rotation": {"velocity": 10000, "upper_endpoint": "min"},
    "shoulder_horizontal_left": {"velocity": 10000, "upper_endpoint": "min"},
    "upper_arm_right_rotation": {"velocity": 10000},
    "elbow_right": {
        "rotation_range_min": -4500,
        "rotation_range_max": 9000,
        "velocity": 10000,
    },
    "lower_arm_right_rotation": {"velocity": 10000},
    "shoulder_vertical_right": {"velocity": 10000, "upper_endpoint": "min"},
    "shoulder_horizontal_right": {"velocity": 10000, "upper_endpoint": "min"},
    "wrist_left": {"velocity": 10000, "visible": False},
    "wrist_right": {"velocity": 10000},
    "thumb_right_opposition": FINGER_MOTOR_PARAMETERS,
    "thumb_right_stretch": INVERTED_FINGER_MOTOR_PARAMETERS,
    "index_right_stretch": FINGER_MOTOR_PARAMETERS,
    "middle_right_stretch": INVERTED_FINGER_MOTOR_PARAMETERS,
    "ring_right_stretch": INVERTED_FINGER_MOTOR_PARAMETERS,
    "pinky_right_stretch": FINGER_MOTOR_PARAMETERS,
    "thumb_left_opposition": INVERTED_FINGER_MOTOR_PARAMETERS,
    "thumb_left_stretch": FINGER_MOTOR_PARAMETERS,
    "index_left_stretch": FINGER_MOTOR_PARAMETERS,
    "middle_left_stretch": INVERTED_FINGER_MOTOR_PARAMETERS,
    "ring_left_stretch": FINGER_MOTOR_PARAMETERS,
    "pinky_left_stretch": FINGER_MOTOR_PARAMETERS,
}

PROFILE = HardwareProfile(
    variant="pib5advanced",
    description="pib 5 advanced hardware with serial motor controllers",
    controllers=tuple(
        [
            # The four serial adapters: address unknown until it is read and
            # entered, the motor supply is a property of the robot, not a fact
            # this profile can know.
            ControllerProfile(number, FEETECH_ST_SERIAL, None, None)
            for number in CONTROLLER_NUMBERS_SERIAL
        ]
        + [
            ControllerProfile(
                CONTROLLER_NUMBER_RELAY,
                TINKERFORGE_BRICKLET,
                SOLID_STATE_RELAY_BRICKLET,
                None,
            )
        ]
        + [
            ControllerProfile(
                number, TINKERFORGE_BRICKLET, RGB_LED_BUTTON_BRICKLET, None
            )
            for number in CONTROLLER_NUMBERS_RGB_BUTTONS
        ]
    ),
    motor_mapping=MOTOR_MAPPING,
    rgb_button_controller_ids=CONTROLLER_NUMBERS_RGB_BUTTONS,
    # The serial bus speaks Feetech ST, so the shared defaults apply unchanged.
    motor_parameter_defaults=MOTOR_PARAMETER_DEFAULTS,
    motor_parameter_deviations=MOTOR_PARAMETER_DEVIATIONS,
    microphone_tuning=MICROPHONE_TUNING,
)
