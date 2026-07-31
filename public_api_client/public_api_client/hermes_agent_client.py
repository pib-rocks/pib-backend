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
import shutil
import subprocess
from typing import Optional

import yaml

from pib_hermes_config import (
    DEFAULT_SOUL,
    PROFILE_PREFIX,
    align_profile_ownership,
    profile_dir_for,
    profile_name_for,
    profiles_dir,
    soul_path_for,
)

DEFAULT_HERMES_BIN = "/home/pib/.local/bin/hermes"
DEFAULT_HERMES_HOME = "/home/pib/.hermes"
SESSION_PREFIX = "pib_chat_"
HERMES_API_NAME = "hermes-agent"
DEFAULT_TIMEOUT_SECONDS = int(os.environ.get("PIB_HERMES_TIMEOUT", "120"))

CONFIG_FILENAME = "config.yaml"
ENV_FILENAME = ".env"
ENV_FILE_MODE = 0o600

# Default MCP entry for pib robot tools. Kept in sync with setup/setup-pib.sh.
PIB_MCP_SERVER = {
    "command": "python3",
    "args": ["-m", "pib_mcp_server"],
}

# Startup liveness probe only. Deliberately small: it runs before the chat node
# is up, so it must diagnose a broken install without delaying startup. A real
# `hermes --version` answers in well under a second.
PROBE_TIMEOUT_SECONDS = 5

_UNSAFE = re.compile(r"[^A-Za-z0-9_-]")


def hermes_bin() -> str:
    """Path of the Hermes CLI. One explicit location, never probed or guessed."""
    return os.environ.get("PIB_HERMES_BIN") or DEFAULT_HERMES_BIN


def hermes_home() -> str:
    """Base Hermes install: the profile-independent config and credentials."""
    return os.environ.get("HERMES_HOME") or DEFAULT_HERMES_HOME


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


def _create_profile_with_cli(personality_id: str, timeout: int) -> bool:
    """Optional enhancement: let the CLI create the profile. True when it did.

    Not a precondition for a working profile: whichever container calls
    ensure_profile() may not have the CLI mounted at all (the Flask API does
    not), and this used to fail silently there and leave behind a profile with a
    SOUL.md but no credentials.
    """
    name = profile_name_for(personality_id)
    if not hermes_binary_available():
        logging.info(
            "hermes CLI %s is not available here; provisioning profile %s with "
            "filesystem operations only", hermes_bin(), name,
        )
        return False
    try:
        result = subprocess.run(
            [hermes_bin(), "profile", "create", name,
             "--clone", "--no-alias",
             "--description", f"pib personality {personality_id}"],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
    except Exception as exc:
        logging.warning("hermes profile create %s could not be run: %s", name, exc)
        return False
    if result.returncode != 0:
        logging.warning(
            "hermes profile create %s exited %s: %s",
            name, result.returncode, (result.stderr or "")[:500],
        )
        return False
    logging.info("created hermes profile %s with the CLI (--clone)", name)
    return True


def _ensure_mcp_servers_pib(pdir: str) -> None:
    """Merge mcp_servers.pib into the profile config.yaml if it is missing.

    Runs even when config.yaml already exists, so profiles created before
    auto-seeding still get pib_mcp_server on the next ensure_profile call.
    """
    target = os.path.join(pdir, CONFIG_FILENAME)
    cfg = {}
    if os.path.isfile(target):
        try:
            with open(target, encoding="utf-8") as fh:
                loaded = yaml.safe_load(fh) or {}
            if not isinstance(loaded, dict):
                logging.warning(
                    "hermes profile %s is not a mapping; rewriting mcp_servers.pib",
                    target,
                )
                loaded = {}
            cfg = loaded
        except (OSError, yaml.YAMLError) as exc:
            logging.warning("could not read %s for mcp seeding: %s", target, exc)
            return

    servers = cfg.get("mcp_servers")
    if not isinstance(servers, dict):
        servers = {}
        cfg["mcp_servers"] = servers
    if "pib" in servers:
        return

    servers["pib"] = dict(PIB_MCP_SERVER)
    try:
        with open(target, "w", encoding="utf-8") as fh:
            yaml.safe_dump(cfg, fh, default_flow_style=False, sort_keys=False)
    except OSError as exc:
        logging.warning("could not write mcp_servers.pib into %s: %s", target, exc)
        return
    logging.info("seeded mcp_servers.pib into hermes profile %s", pdir)


def _inherit_base_config(pdir: str) -> None:
    """Materialize the provider config a profile needs, from HERMES_HOME.

    `hermes -p <profile>` resolves its LLM provider from the profile, so a
    profile holding only a SOUL.md fails every turn with "No LLM provider
    configured". Copies rather than symlinks, so that a later `hermes profile
    delete` cannot damage the base install, and never replaces a file the profile
    already has, which may have been customized on purpose.

    Always ensures mcp_servers.pib is present afterwards, including when the
    profile already had its own config.yaml.
    """
    base = hermes_home()
    for name, mode in ((CONFIG_FILENAME, None), (ENV_FILENAME, ENV_FILE_MODE)):
        target = os.path.join(pdir, name)
        if os.path.exists(target):
            logging.debug("hermes profile keeps its own %s (%s)", name, target)
            continue
        source = os.path.join(base, name)
        if not os.path.isfile(source):
            logging.warning(
                "hermes base install has no %s at %s: hermes-agent personalities "
                "will answer with the fallback reply until credentials are "
                "configured there (`sudo -u pib -H hermes setup`)",
                name, source,
            )
            continue
        try:
            shutil.copyfile(source, target)
            if mode is not None:
                os.chmod(target, mode)
        except OSError as exc:
            logging.warning(
                "could not copy %s from %s into %s: %s", name, base, pdir, exc
            )
            continue
        logging.info("copied %s from %s into hermes profile %s", name, base, pdir)

    _ensure_mcp_servers_pib(pdir)


def ensure_profile(personality_id: str, soul_text: str, timeout: int = 60) -> str:
    """Create the personality's Hermes profile if needed and write its SOUL.md.

    Provisioning is done with filesystem operations alone, because the CLI is not
    installed in every container that calls this. config.yaml and .env are copied
    in from the base install (HERMES_HOME) so that `hermes -p <profile>` finds a
    provider; the base install must therefore hold working credentials.

    Returns the profile directory.
    """
    pdir = profile_dir_for(personality_id)
    if not os.path.isdir(pdir):
        _create_profile_with_cli(personality_id, timeout)
    os.makedirs(pdir, exist_ok=True)
    with open(soul_path_for(personality_id), "w", encoding="utf-8") as fh:
        fh.write(soul_text or DEFAULT_SOUL)
    # Also repairs a profile that an earlier deployment left without credentials.
    _inherit_base_config(pdir)
    align_profile_ownership(pdir)
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
