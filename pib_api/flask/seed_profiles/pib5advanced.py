"""Seed profile for a pib 5 advanced robot with serial motor controllers.

This variant is wired differently from the educational robots. The motors sit on
four serial controllers - Waveshare adapters on a Feetech ST bus - and the
Tinkerforge HAT carries only the solid-state relay and up to three RGB LED
buttons. There is no servo bricklet.

The profile exists so that an installation of this variant can seed, and a robot
whose seed fails does not start at all: the container runs
``flask db upgrade && flask seed_db && …`` and ``get_profile`` raises for a
variant without a profile.

Every controller starts **without an address**. The identifiers of the serial
adapters are read separately and entered later, either through the hardware-IDs
page or through an exported hardware config (``controllers[].address`` accepts a
free-form string for a non-Tinkerforge controller). ``_upsert_controllers``
writes an address only when it creates a row, so nothing that was entered later
is overwritten by the next start.
"""

from seed_profiles.edu_microphone_tuning import MICROPHONE_TUNING
from seed_profiles.edu_motor_parameters import MOTOR_PARAMETER_DEFAULTS
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
    # No motor rows yet: which motor sits on which bus channel follows the
    # identifiers that are entered later, and a guessed mapping would drive the
    # wrong joint. ``pib seed_hardware --force`` adds them once they are known;
    # it also leaves existing motors alone while this mapping is empty.
    motor_mapping={},
    rgb_button_controller_ids=CONTROLLER_NUMBERS_RGB_BUTTONS,
    # The serial bus speaks Feetech ST, so the shared defaults apply unchanged.
    motor_parameter_defaults=MOTOR_PARAMETER_DEFAULTS,
    motor_parameter_deviations={},
    microphone_tuning=MICROPHONE_TUNING,
)
