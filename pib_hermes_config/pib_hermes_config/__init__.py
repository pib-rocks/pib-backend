"""Shared, dependency-free resolution of the Hermes profiles location.

Two separately deployed processes touch the same files: the Flask API writes a
personality's ``SOUL.md`` and the ROS voice assistant runs the agent that reads
it. They run in different containers, so the directory only lines up when both
resolve it identically and the host directory is bind-mounted into both at that
same path. If the two ever disagree, the API keeps reporting success while the
agent reads a file nobody writes.

``PIB_HERMES_PROFILES_DIR`` and ``DEFAULT_PROFILES_DIR`` must therefore stay in
sync with the bind mounts in ``docker-compose.yaml``.
"""

import os
import re

DEFAULT_PROFILES_DIR = "/home/pib/.hermes/profiles"
PROFILES_DIR_ENV = "PIB_HERMES_PROFILES_DIR"
PROFILE_PREFIX = "pib_"
SOUL_FILENAME = "SOUL.md"
DEFAULT_SOUL = "Du bist pib, ein humanoider Roboter."

_UNSAFE = re.compile(r"[^A-Za-z0-9_-]")


def profiles_dir() -> str:
    """Directory holding one Hermes profile per pib personality."""
    return os.environ.get(PROFILES_DIR_ENV) or DEFAULT_PROFILES_DIR


def profile_name_for(personality_id: str) -> str:
    """Name of the Hermes profile that hosts this personality."""
    return PROFILE_PREFIX + _UNSAFE.sub("", (personality_id or "").replace(" ", "_"))


def profile_dir_for(personality_id: str) -> str:
    """Absolute path of one personality's Hermes profile directory."""
    return os.path.join(profiles_dir(), profile_name_for(personality_id))


def soul_path_for(personality_id: str) -> str:
    """Absolute path of the SOUL.md belonging to one personality."""
    return os.path.join(profile_dir_for(personality_id), SOUL_FILENAME)
