"""Materializes a personality's SOUL text to its Hermes profile SOUL.md.

The Hermes Agent reads the SOUL.md of the profile it runs under (-p) to establish
pib's identity/persona. The authoritative copy lives in the personality.description
DB column; this module mirrors it into the profile directory.

The profile location comes from pib_hermes_config so that this API and the ROS
voice assistant that runs the agent cannot drift apart; see that module and the
profiles bind mount in docker-compose.yaml.
"""
import os

from pib_hermes_config import (
    DEFAULT_SOUL,
    align_profile_ownership,
    profile_name_for,
    profiles_dir,
    soul_path_for,
)

__all__ = [
    "DEFAULT_SOUL",
    "profile_name_for",
    "profiles_dir",
    "soul_path_for",
    "write_soul",
    "read_soul",
]


def write_soul(personality_id: str, text: str) -> str:
    """Write the SOUL text to the profile, creating parent dirs. Returns the path."""
    path = soul_path_for(personality_id)
    profile_dir = os.path.dirname(path)
    os.makedirs(profile_dir, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text or DEFAULT_SOUL)
    # This API runs as root in its container; without this the profile it just
    # created belongs to root and the pib user cannot even list it.
    align_profile_ownership(profile_dir)
    return path


def read_soul(personality_id: str) -> str:
    """Read the SOUL text, or '' when it does not exist yet."""
    path = soul_path_for(personality_id)
    if not os.path.isfile(path):
        return ""
    with open(path, encoding="utf-8") as fh:
        return fh.read()
