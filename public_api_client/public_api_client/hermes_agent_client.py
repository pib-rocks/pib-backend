"""Runs the Hermes Agent as the conversation partner for a pib chat.

One pib chat_id maps to exactly one persistent Hermes session, so the agent
retains memory across turns and across robot restarts.
"""
import logging
import os
import re
import subprocess
from typing import Optional

HERMES_BIN = os.environ.get("PIB_HERMES_BIN", "/home/pib/.local/bin/hermes")
SESSION_PREFIX = "pib_chat_"
PROFILE_PREFIX = "pib_"
HERMES_API_NAME = "hermes-agent"
DEFAULT_TIMEOUT_SECONDS = int(os.environ.get("PIB_HERMES_TIMEOUT", "120"))

_UNSAFE = re.compile(r"[^A-Za-z0-9_-]")


def uses_hermes_backend(api_name: Optional[str]) -> bool:
    """True when the personality's assistant model should route to Hermes Agent."""
    return api_name == HERMES_API_NAME


def session_name_for(chat_id: str) -> str:
    """Deterministic Hermes session name for a pib chat id."""
    return SESSION_PREFIX + _UNSAFE.sub("", (chat_id or "").replace(" ", "_"))


def profile_name_for(personality_id: str) -> str:
    """Hermes profile that holds this personality's SOUL.md."""
    return PROFILE_PREFIX + _UNSAFE.sub("", (personality_id or "").replace(" ", "_"))


def build_command(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
) -> list[str]:
    """argv for one one-shot turn in this chat's persistent session.

    The personality's persona comes from the Hermes PROFILE
    (~/.hermes/profiles/pib_<personality_id>/SOUL.md), selected via -p.
    Conversation memory comes from the named SESSION, selected via -c.
    Verified: -p and -c compose correctly (persona + memory together).
    """
    cmd = [HERMES_BIN]
    if personality_id:
        cmd += ["-p", profile_name_for(personality_id)]
    cmd += ["-z", text, "-c", session_name_for(chat_id)]
    if toolsets:
        cmd += ["-t", toolsets]
    return cmd


HERMES_HOME = os.environ.get("HERMES_HOME", "/home/pib/.hermes")


def profile_dir_for(personality_id: str) -> str:
    return os.path.join(HERMES_HOME, "profiles", profile_name_for(personality_id))


def ensure_profile(personality_id: str, soul_text: str, timeout: int = 60) -> str:
    """Create the personality's Hermes profile if needed and write its SOUL.md.

    Uses --clone so config.yaml AND the provider credentials are inherited from
    the active profile; without this the profile has no LLM provider configured.
    Returns the profile directory.
    """
    pdir = profile_dir_for(personality_id)
    if not os.path.isdir(pdir):
        subprocess.run(
            [HERMES_BIN, "profile", "create", profile_name_for(personality_id),
             "--clone", "--no-alias",
             "--description", f"pib personality {personality_id}"],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
    os.makedirs(pdir, exist_ok=True)
    with open(os.path.join(pdir, "SOUL.md"), "w", encoding="utf-8") as fh:
        fh.write(soul_text or "Du bist pib, ein humanoider Roboter.")
    return pdir


def delete_profile(personality_id: str, timeout: int = 60) -> bool:
    """Remove a personality's Hermes profile (best-effort).

    NOTE: `hermes profile delete` prompts for confirmation — feed the name on stdin.
    """
    name = profile_name_for(personality_id)
    try:
        result = subprocess.run(
            [HERMES_BIN, "profile", "delete", name],
            input=name + "\n",
            capture_output=True, text=True, timeout=timeout, check=False,
        )
        return result.returncode == 0
    except Exception as exc:
        logging.warning("could not delete hermes profile %s: %s", name, exc)
        return False


FALLBACK_REPLY = (
    "Entschuldige, das hat gerade einen Moment zu lange gedauert. "
    "Frag mich bitte später noch einmal."
)


def run_turn(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
) -> str:
    """Run one conversational turn. Always returns speakable text."""
    cmd = build_command(text, chat_id, personality_id, toolsets)
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False
        )
    except subprocess.TimeoutExpired:
        logging.warning("hermes turn timed out after %ss (chat=%s)", timeout, chat_id)
        return FALLBACK_REPLY
    except Exception as exc:
        logging.error("hermes turn failed (chat=%s): %s", chat_id, exc)
        return FALLBACK_REPLY

    if result.returncode != 0:
        logging.error(
            "hermes exited %s (chat=%s): %s",
            result.returncode, chat_id, (result.stderr or "")[:500],
        )
        return FALLBACK_REPLY

    reply = (result.stdout or "").strip()
    return reply or FALLBACK_REPLY


def delete_session(chat_id: str, timeout: int = 30) -> bool:
    """Remove the Hermes session backing this pib chat. Best-effort."""
    try:
        result = subprocess.run(
            [HERMES_BIN, "sessions", "delete", session_name_for(chat_id)],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
        return result.returncode == 0
    except Exception as exc:
        logging.warning("could not delete hermes session for %s: %s", chat_id, exc)
        return False
