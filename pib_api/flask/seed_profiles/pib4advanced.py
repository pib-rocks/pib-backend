"""Seed profile for pib 4 advanced hardware with serial motor controllers."""

from seed_profiles.pib5advanced import PROFILE as PIB5ADVANCED_PROFILE
from seed_profiles.profile import HardwareProfile

PROFILE = HardwareProfile(
    variant="pib4advanced",
    description="pib 4 advanced hardware with serial motor controllers",
    controllers=PIB5ADVANCED_PROFILE.controllers,
    motor_mapping=PIB5ADVANCED_PROFILE.motor_mapping,
    rgb_button_controller_ids=PIB5ADVANCED_PROFILE.rgb_button_controller_ids,
    motor_parameter_defaults=PIB5ADVANCED_PROFILE.motor_parameter_defaults,
    motor_parameter_deviations=PIB5ADVANCED_PROFILE.motor_parameter_deviations,
    microphone_tuning=PIB5ADVANCED_PROFILE.microphone_tuning,
)
