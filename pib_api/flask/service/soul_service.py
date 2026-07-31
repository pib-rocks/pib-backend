"""Materializes a personality's SOUL text to its Hermes profile SOUL.md.

The Hermes Agent reads the SOUL.md of the profile it runs under (-p) to establish
pib's identity/persona. The authoritative copy lives in the personality.description
DB column; this module mirrors it into the profile directory.

Verified: ~/.hermes/profiles/<profile>/SOUL.md is injected when running
`hermes -p <profile> -z ...`.
"""
import os

HERMES_HOME = os.environ.get("HERMES_HOME", "/home/pib/.hermes")
PROFILE_PREFIX = "pib_"

DEFAULT_SOUL = "Du bist pib, ein humanoider Roboter."


def profile_name_for(personality_id: str) -> str:
    """Name of the Hermes profile that hosts this personality."""
    return PROFILE_PREFIX + personality_id


def soul_path_for(personality_id: str) -> str:
    """Absolute path of the SOUL.md belonging to one personality."""
    return os.path.join(
        HERMES_HOME, "profiles", profile_name_for(personality_id), "SOUL.md"
    )


def write_soul(personality_id: str, text: str) -> str:
    """Write the SOUL text to the profile, creating parent dirs. Returns the path."""
    path = soul_path_for(personality_id)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text or DEFAULT_SOUL)
    return path


def read_soul(personality_id: str) -> str:
    """Read the SOUL text, or '' when it does not exist yet."""
    path = soul_path_for(personality_id)
    if not os.path.isfile(path):
        return ""
    with open(path, encoding="utf-8") as fh:
        return fh.read()
