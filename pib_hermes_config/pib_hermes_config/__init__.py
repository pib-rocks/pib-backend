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

import logging
import os
import re

DEFAULT_PROFILES_DIR = "/home/pib/.hermes/profiles"
PROFILES_DIR_ENV = "PIB_HERMES_PROFILES_DIR"
PROFILE_PREFIX = "pib_"
SOUL_FILENAME = "SOUL.md"
DEFAULT_SOUL = "Du bist pib, ein humanoider Roboter."
PROFILE_DIR_MODE = 0o700

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


def align_profile_ownership(profile_dir: str) -> None:
    """Hand a profile to whoever owns the profiles directory. Best effort.

    Both writers run as root inside their container, so everything they create is
    root-owned — unreadable for the ``pib`` user that owns the bind-mounted
    profiles directory on the host, and for any consumer running under a
    different uid, which locks an operator out of inspecting or repairing the
    profile. The intended owner is read off the parent directory rather than
    hardcoded to uid 1000, and every failure here is only logged: a personality
    update or a chat turn must never fail over file ownership.
    """
    parent = profiles_dir()
    try:
        intended = os.stat(parent)
    except OSError as exc:
        logging.debug("cannot stat %s to align profile ownership: %s", parent, exc)
        return

    paths = [profile_dir]
    for root, dirnames, filenames in os.walk(profile_dir):
        paths += [os.path.join(root, name) for name in dirnames + filenames]
    for path in paths:
        try:
            os.chown(path, intended.st_uid, intended.st_gid)
        except OSError as exc:
            # Typically: not running as root. Every remaining path would fail the
            # same way, so stop rather than repeat the same log line.
            logging.debug(
                "could not chown %s to %s:%s: %s",
                path, intended.st_uid, intended.st_gid, exc,
            )
            break
    else:
        logging.debug(
            "hermes profile %s now owned by %s:%s",
            profile_dir, intended.st_uid, intended.st_gid,
        )

    try:
        os.chmod(profile_dir, PROFILE_DIR_MODE)
    except OSError as exc:
        logging.debug("could not chmod %s to %o: %s", profile_dir, PROFILE_DIR_MODE, exc)
