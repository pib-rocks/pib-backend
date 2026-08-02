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
    build_default_soul_text,
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


def write_soul(personality_id: str, text: str, personality_name: str = "pib") -> str:
    """Write the SOUL text to the profile, creating parent dirs.

    When ``text`` is blank, seeds the default SOUL template for ``personality_name``.
    Otherwise writes ``text`` as-is so the DB/editor content stays authoritative.

    Returns the full written soul content string.
    """
    path = soul_path_for(personality_id)
    profile_dir = os.path.dirname(path)
    os.makedirs(profile_dir, exist_ok=True)
    if text and text.strip():
        soul_content = text
    else:
        soul_content = build_default_soul_text(personality_name)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(soul_content)
    try:
        os.chmod(path, 0o664)
    except OSError:
        pass
    align_profile_ownership(profile_dir)
    # created belongs to root and the pib user cannot even list it.
    align_profile_ownership(profile_dir)
    return soul_content


def read_soul(personality_id: str) -> str:
    """Read the full SOUL.md content, or '' when it does not exist yet."""
    path = soul_path_for(personality_id)
    if not os.path.isfile(path):
        return ""
    with open(path, encoding="utf-8") as fh:
        return fh.read()
