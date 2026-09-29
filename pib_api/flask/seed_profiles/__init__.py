"""Hardware profile registry and installed-variant resolution."""

import os
from pathlib import Path

from seed_profiles.pib4edu import PROFILE as PIB4EDU_PROFILE
from seed_profiles.pib5edu import PROFILE as PIB5EDU_PROFILE
from seed_profiles.profile import ControllerProfile, HardwareProfile

DEFAULT_HARDWARE_VARIANT = "pib5edu"
HARDWARE_VARIANT_FILE = Path("/etc/pib_hardware_variant")

PROFILES = {
    PIB4EDU_PROFILE.variant: PIB4EDU_PROFILE,
    PIB5EDU_PROFILE.variant: PIB5EDU_PROFILE,
}


class UnknownHardwareVariantError(ValueError):
    """Raised when no seed profile is implemented for a selected variant."""


def get_profile(variant: str) -> HardwareProfile:
    try:
        return PROFILES[variant]
    except KeyError as error:
        implemented = ", ".join(sorted(PROFILES))
        raise UnknownHardwareVariantError(
            f"Hardware variant {variant!r} has no implemented seed profile. "
            f"Implemented profiles: {implemented}."
        ) from error


def resolve_variant_and_source() -> tuple[str, str]:
    environment_variant = os.environ.get("PIB_HARDWARE_VARIANT", "").strip()
    if environment_variant:
        return environment_variant, "environment"

    try:
        file_variant = HARDWARE_VARIANT_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        # a missing file is normal on a machine that never ran the setup script; docker
        # creates a directory when a bind-mounted host path does not exist, so do not
        # only guard against FileNotFoundError here
        file_variant = ""

    if file_variant:
        return file_variant, "file"
    return DEFAULT_HARDWARE_VARIANT, "default"


def resolve_variant_from_environment() -> str:
    return resolve_variant_and_source()[0]


__all__ = [
    "ControllerProfile",
    "HardwareProfile",
    "PROFILES",
    "UnknownHardwareVariantError",
    "get_profile",
    "resolve_variant_and_source",
    "resolve_variant_from_environment",
]
