"""Runs the Hermes Agent as the conversation partner for a pib chat.

One pib chat_id maps to exactly one persistent Hermes session, so the agent
retains memory across turns and across robot restarts.

The profile location is imported from pib_hermes_config, which the Flask API uses
as well: the SOUL.md the API writes must be the very file this agent reads. Both
the binary and the profiles directory are bind-mounted into the voice-assistant
container; see docker-compose.yaml. That mount list also needs the uv-managed
Python directory, because the CLI is a wrapper that execs a venv interpreter
symlinked into it — probe_binary() is what catches a deployment that forgot it.
"""
import logging
import os
import re
import subprocess
from typing import Optional

from pib_hermes_config import (
    DEFAULT_SOUL,
    PROFILE_PREFIX,
    profile_dir_for,
    profile_name_for,
    profiles_dir,
    soul_path_for,
)

DEFAULT_HERMES_BIN = "/home/pib/.local/bin/hermes"
SESSION_PREFIX = "pib_chat_"
HERMES_API_NAME = "hermes-agent"
DEFAULT_TIMEOUT_SECONDS = int(os.environ.get("PIB_HERMES_TIMEOUT", "120"))

# Startup liveness probe only. Deliberately small: it runs before the chat node
# is up, so it must diagnose a broken install without delaying startup. A real
# `hermes --version` answers in well under a second.
PROBE_TIMEOUT_SECONDS = 5

_UNSAFE = re.compile(r"[^A-Za-z0-9_-]")


def hermes_bin() -> str:
    """Path of the Hermes CLI. One explicit location, never probed or guessed."""
    return os.environ.get("PIB_HERMES_BIN") or DEFAULT_HERMES_BIN


def hermes_binary_available() -> bool:
    """True when the configured Hermes CLI exists and may be executed."""
    path = hermes_bin()
    return os.path.isfile(path) and os.access(path, os.X_OK)


def probe_binary(timeout: int = PROBE_TIMEOUT_SECONDS) -> tuple[bool, str]:
    """Check that the configured CLI actually runs. Returns (ok, detail).

    An existence check is not sufficient and has already produced a false green
    on a live robot: the CLI is a small wrapper script that execs an interpreter
    inside the hermes venv, and that interpreter is a symlink into uv-managed
    Python outside HERMES_HOME. When that directory is not mounted, the wrapper
    is present and executable yet exits 127. Only running it reveals that.

    `--version` is used because it is cheap, offline and needs no LLM provider.
    `detail` carries the captured stderr on failure; that text is what pinpoints
    a broken install, so callers should log it verbatim.
    """
    path = hermes_bin()
    if not hermes_binary_available():
        return False, f"no executable file at '{path}'"
    try:
        result = subprocess.run(
            [path, "--version"],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
    except subprocess.TimeoutExpired:
        # A probe timeout condemns the probe, not the install: report it as a
        # failure but never let it hold up node startup any longer than this.
        return False, f"'{path} --version' did not answer within {timeout}s"
    except Exception as exc:
        return False, f"'{path} --version' could not be started: {exc}"

    if result.returncode != 0:
        stderr = (result.stderr or result.stdout or "").strip()[:500]
        return False, (
            f"'{path} --version' exited {result.returncode}: {stderr}"
        )
    banner = (result.stdout or "").strip().splitlines()
    return True, (banner[0][:200] if banner else "")


def uses_hermes_backend(api_name: Optional[str]) -> bool:
    """True when the personality's assistant model should route to Hermes Agent."""
    return api_name == HERMES_API_NAME


def session_name_for(chat_id: str) -> str:
    """Deterministic Hermes session name for a pib chat id."""
    return SESSION_PREFIX + _UNSAFE.sub("", (chat_id or "").replace(" ", "_"))


def build_command(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
) -> list[str]:
    """argv for one one-shot turn in this chat's persistent session.

    The personality's persona comes from the Hermes PROFILE
    (<profiles_dir>/pib_<personality_id>/SOUL.md), selected via -p.
    Conversation memory comes from the named SESSION, selected via -c.
    Verified: -p and -c compose correctly (persona + memory together).
    """
    cmd = [hermes_bin()]
    if personality_id:
        cmd += ["-p", profile_name_for(personality_id)]
    cmd += ["-z", text, "-c", session_name_for(chat_id)]
    if toolsets:
        cmd += ["-t", toolsets]
    return cmd


def ensure_profile(personality_id: str, soul_text: str, timeout: int = 60) -> str:
    """Create the personality's Hermes profile if needed and write its SOUL.md.

    Uses --clone so config.yaml AND the provider credentials are inherited from
    the active profile; without this the profile has no LLM provider configured.
    Returns the profile directory.
    """
    pdir = profile_dir_for(personality_id)
    if not os.path.isdir(pdir):
        if hermes_binary_available():
            subprocess.run(
                [hermes_bin(), "profile", "create", profile_name_for(personality_id),
                 "--clone", "--no-alias",
                 "--description", f"pib personality {personality_id}"],
                capture_output=True, text=True, timeout=timeout, check=False,
            )
        else:
            logging.error(
                "cannot create hermes profile %s: binary %s is missing",
                profile_name_for(personality_id), hermes_bin(),
            )
    os.makedirs(pdir, exist_ok=True)
    with open(soul_path_for(personality_id), "w", encoding="utf-8") as fh:
        fh.write(soul_text or DEFAULT_SOUL)
    return pdir


def delete_profile(personality_id: str, timeout: int = 60) -> bool:
    """Remove a personality's Hermes profile (best-effort).

    NOTE: `hermes profile delete` prompts for confirmation — feed the name on stdin.
    """
    name = profile_name_for(personality_id)
    try:
        result = subprocess.run(
            [hermes_bin(), "profile", "delete", name],
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
    if not hermes_binary_available():
        # Distinct from a timeout: the agent was never started at all.
        logging.error(
            "hermes binary %s is missing or not executable (chat=%s); "
            "answering with the fallback reply. Install the hermes CLI for the "
            "pib user and check the PIB_HERMES_BIN mount.",
            hermes_bin(), chat_id,
        )
        return FALLBACK_REPLY

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
            [hermes_bin(), "sessions", "delete", session_name_for(chat_id)],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
        return result.returncode == 0
    except Exception as exc:
        logging.warning("could not delete hermes session for %s: %s", chat_id, exc)
        return False
