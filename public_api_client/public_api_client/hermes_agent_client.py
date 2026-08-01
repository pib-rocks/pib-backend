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
    build_default_soul_text,
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
    "env": {
        "FLASK_API_BASE_URL": os.getenv("FLASK_API_BASE_URL", "http://flask-app:5000"),
        "PIB_MCP_API_BASE_URL": os.getenv("FLASK_API_BASE_URL", "http://flask-app:5000"),
        "PIB_MCP_ROSBRIDGE_URL": os.getenv("PIB_MCP_ROSBRIDGE_URL", "ws://rosbridge-ws:9090"),
    },
}

# Permanent Hermes LLM pin. Kept in sync with setup/setup-pib.sh.
DEFAULT_HERMES_MODEL = "gemini-3.6-flash"
DEFAULT_HERMES_PROVIDER = "gemini"

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
    cmd += ["--continue", session_name_for(chat_id)]
    toolsets = toolsets if toolsets is not None else "pib"
    if toolsets:
        cmd += ["-t", toolsets]
    cmd += ["-z", text]
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
    """Pin the Hermes model/provider and merge mcp_servers.pib if missing.

    Always sets model/provider to the permanent Gemini defaults. Runs even when
    config.yaml already exists, so profiles created before auto-seeding still get
    pib_mcp_server (and the pinned model) on the next ensure_profile call.
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

    changed = False
    if (
        cfg.get("model") != DEFAULT_HERMES_MODEL
        or cfg.get("provider") != DEFAULT_HERMES_PROVIDER
    ):
        cfg["model"] = DEFAULT_HERMES_MODEL
        cfg["provider"] = DEFAULT_HERMES_PROVIDER
        changed = True

    servers = cfg.get("mcp_servers")
    if not isinstance(servers, dict):
        servers = {}
        cfg["mcp_servers"] = servers
        changed = True
    if "pib" not in servers:
        servers["pib"] = dict(PIB_MCP_SERVER)
        changed = True

    if not changed:
        return

    try:
        with open(target, "w", encoding="utf-8") as fh:
            yaml.safe_dump(cfg, fh, default_flow_style=False, sort_keys=False)
    except OSError as exc:
        logging.warning("could not write profile config into %s: %s", target, exc)
        return
    logging.info("ensured hermes model/provider and mcp_servers.pib in %s", pdir)


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


def ensure_profile(
    personality_id: str,
    soul_text: str = "",
    timeout: int = 60,
    personality_name: Optional[str] = None,
) -> str:
    """Create the personality's Hermes profile if needed and write its SOUL.md.

    Provisioning is done with filesystem operations alone, because the CLI is not
    installed in every container that calls this. config.yaml and .env are copied
    in from the base install (HERMES_HOME) so that `hermes -p <profile>` finds a
    provider; the base install must therefore hold working credentials.

    The SOUL.md is always seeded from ``build_default_soul_text`` so every profile
    knows its robot identity and the available pib MCP tools. ``soul_text`` is
    treated as an optional custom description appended after the identity line.

    Returns the profile directory.
    """
    pdir = profile_dir_for(personality_id)
    if not os.path.isdir(pdir):
        _create_profile_with_cli(personality_id, timeout)
    os.makedirs(pdir, exist_ok=True)
    text = build_default_soul_text(
        personality_name or "pib",
        custom_description=soul_text or None,
    )
    with open(soul_path_for(personality_id), "w", encoding="utf-8") as fh:
        fh.write(text)
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

# Local warm daemon (see hermes_daemon.py). Overridable for tests / custom binds.
DEFAULT_DAEMON_TURN_URL = "http://127.0.0.1:8088/turn"


def daemon_turn_url() -> str:
    """POST target for a warm-daemon turn. Trailing path is always /turn."""
    override = os.environ.get("PIB_HERMES_DAEMON_URL")
    if override:
        base = override.rstrip("/")
        return base if base.endswith("/turn") else base + "/turn"
    return DEFAULT_DAEMON_TURN_URL


def _try_daemon_turn(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
) -> Optional[str]:
    """POST /turn to the warm daemon. None means unreachable or non-200."""
    try:
        import requests
    except ImportError:
        return None

    payload = {"text": text, "chat_id": chat_id, "timeout": timeout}
    if personality_id is not None:
        payload["personality_id"] = personality_id
    if toolsets is not None:
        payload["toolsets"] = toolsets

    try:
        response = requests.post(
            daemon_turn_url(),
            json=payload,
            timeout=timeout,
        )
    except requests.exceptions.RequestException as exc:
        logging.debug(
            "hermes daemon unreachable (chat=%s): %s; falling back to subprocess",
            chat_id, exc,
        )
        return None

    if response.status_code != 200:
        logging.warning(
            "hermes daemon returned %s (chat=%s); falling back to subprocess",
            response.status_code, chat_id,
        )
        return None

    try:
        data = response.json()
    except ValueError:
        logging.warning(
            "hermes daemon returned non-json body (chat=%s); falling back",
            chat_id,
        )
        return None

    reply = data.get("reply") if isinstance(data, dict) else None
    if not isinstance(reply, str):
        logging.warning(
            "hermes daemon response missing reply string (chat=%s); falling back",
            chat_id,
        )
        return None

    return reply.strip() or FALLBACK_REPLY


def run_turn_subprocess(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
) -> str:
    """Run one turn via a oneshot Hermes CLI subprocess. Always returns text."""
    if not hermes_binary_available():
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


def run_turn(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
) -> str:
    """Run one conversational turn. Always returns speakable text.

    Prefers the warm localhost daemon (POST /turn). If the daemon is
    unreachable or fails, falls back to a oneshot ``subprocess.run`` of the
    Hermes CLI.
    """
    if not hermes_binary_available():
        # Distinct from a timeout: the agent was never started at all.
        logging.error(
            "hermes binary %s is missing or not executable (chat=%s); "
            "answering with the fallback reply. Install the hermes CLI for the "
            "pib user and check the PIB_HERMES_BIN mount.",
            hermes_bin(), chat_id,
        )
        return FALLBACK_REPLY

    daemon_reply = _try_daemon_turn(
        text, chat_id, personality_id, toolsets, timeout=timeout,
    )
    if daemon_reply is not None:
        return daemon_reply

    return run_turn_subprocess(
        text, chat_id, personality_id, toolsets, timeout=timeout,
    )


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
