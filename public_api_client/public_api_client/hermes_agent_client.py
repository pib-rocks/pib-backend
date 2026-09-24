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

import copy
import json
import logging
import os
import re
import subprocess
import time
from typing import Iterator, Optional

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
# Voice turns use a narrow allowlist, with the existing blacklist retained as a
# second isolation layer. Operators may tune both values without a rebuild.
DEFAULT_ENABLED_TOOLSETS = os.environ.get(
    "PIB_HERMES_ENABLED_TOOLSETS", "mcp-pib,vision"
)
DEFAULT_DISABLED_TOOLSETS = os.environ.get(
    "PIB_HERMES_DISABLED_TOOLSETS",
    "terminal,code_execution,file,memory,session_search",
)
DEFAULT_MAX_TURNS = int(os.environ.get("PIB_HERMES_MAX_TURNS", "4"))

CONFIG_FILENAME = "config.yaml"
ENV_FILENAME = ".env"
ENV_FILE_MODE = 0o600

# Default MCP entry for pib robot tools, including the env the server needs:
# Hermes spawns it as a subprocess without forwarding this process' environment,
# so without `env` pib_mcp_server resolves the REST base URL to its own
# http://localhost:5000 default and every robot tool call fails in the container.
# The single definition of that entry — setup/setup-pib.sh seeds the same one and
# tests/unit/test_setup_hermes_model_pin.py fails if the two ever drift.
PIB_MCP_SERVER = {
    "command": "python3",
    "args": ["-m", "pib_mcp_server"],
    "env": {
        "FLASK_API_BASE_URL": os.getenv("FLASK_API_BASE_URL", "http://flask-app:5000"),
        "PIB_MCP_API_BASE_URL": os.getenv(
            "FLASK_API_BASE_URL", "http://flask-app:5000"
        ),
        "PIB_MCP_ROSBRIDGE_URL": os.getenv(
            "PIB_MCP_ROSBRIDGE_URL", "ws://rosbridge-ws:9090"
        ),
    },
}

# Permanent Hermes LLM pin. Kept in sync with setup/setup-pib.sh
# and pib_hermes_config.DEFAULT_HERMES_MODEL.
DEFAULT_HERMES_MODEL = "gemini-3.5-flash"
DEFAULT_HERMES_LITE_MODEL = "gemini-3.5-flash-lite"
DEFAULT_HERMES_PROVIDER = "gemini"
# High-speed defaults for Gemini Flash / Flash-Lite (PR-1524).
DEFAULT_REASONING_EFFORT = "low"
DEFAULT_MAX_TOKENS = 1024
DEFAULT_TEMPERATURE = 0.3

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
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        # A probe timeout condemns the probe, not the install: report it as a
        # failure but never let it hold up node startup any longer than this.
        return False, f"'{path} --version' did not answer within {timeout}s"
    except Exception as exc:
        return False, f"'{path} --version' could not be started: {exc}"

    if result.returncode != 0:
        stderr = (result.stderr or result.stdout or "").strip()[:500]
        return False, (f"'{path} --version' exited {result.returncode}: {stderr}")
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
    if toolsets:
        cmd += ["-t", toolsets]
    cmd += ["-z", text]
    return cmd


def _merge_missing_mcp_env(entry: dict) -> bool:
    """Fill the env keys PIB_MCP_SERVER needs into an existing entry. Returns changed.

    Migration path for the profiles that were seeded before the entry carried an
    `env` block. Only MISSING keys are added: an operator's own `command`, `args`
    and any env value they set themselves stay exactly as they wrote them.
    """
    env = entry.get("env")
    changed = False
    if not isinstance(env, dict):
        env = {}
        entry["env"] = env
        changed = True
    for key, value in PIB_MCP_SERVER["env"].items():
        if key not in env:
            env[key] = value
            changed = True
    return changed


def _ensure_mcp_servers_pib(pdir: str) -> None:
    """Pin Hermes model/provider/speed defaults and repair mcp_servers.pib.

    Always sets model/provider and high-speed defaults (reasoning_effort, max_tokens,
    temperature) to the permanent Gemini Flash values. Runs even when config.yaml
    already exists, so profiles created before auto-seeding still get pib_mcp_server
    (and the pinned model/speed settings) on the next ensure_profile call. An
    mcp_servers.pib entry that is already there keeps its command/args and only
    gets the env keys it is missing.
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

    if cfg.get("reasoning_effort") != DEFAULT_REASONING_EFFORT:
        cfg["reasoning_effort"] = DEFAULT_REASONING_EFFORT
        changed = True
    if cfg.get("max_tokens") != DEFAULT_MAX_TOKENS:
        cfg["max_tokens"] = DEFAULT_MAX_TOKENS
        changed = True
    if cfg.get("temperature") != DEFAULT_TEMPERATURE:
        cfg["temperature"] = DEFAULT_TEMPERATURE
        changed = True

    servers = cfg.get("mcp_servers")
    if not isinstance(servers, dict):
        servers = {}
        cfg["mcp_servers"] = servers
        changed = True
    entry = servers.get("pib")
    if not isinstance(entry, dict):
        servers["pib"] = copy.deepcopy(PIB_MCP_SERVER)
        changed = True
    elif _merge_missing_mcp_env(entry):
        changed = True

    if not changed:
        return

    try:
        with open(target, "w", encoding="utf-8") as fh:
            yaml.safe_dump(cfg, fh, default_flow_style=False, sort_keys=False)
    except OSError as exc:
        logging.warning("could not write profile config into %s: %s", target, exc)
        return
    logging.info(
        "ensured hermes model/provider/speed defaults and mcp_servers.pib in %s",
        pdir,
    )


def ensure_profile(
    personality_id: str,
    soul_text: str = "",
    timeout: int = 60,
    personality_name: Optional[str] = None,
) -> str:
    """Loud local repair path, executed only where Hermes is importable."""
    del timeout
    from public_api_client.hermes_daemon import ensure_profile_home

    result = ensure_profile_home(
        personality_id,
        personality_name=personality_name,
        soul_text=soul_text,
    )
    return result["profile_dir"]


def delete_profile(personality_id: str, timeout: int = 60) -> bool:
    """Remove a personality's Hermes profile (best-effort).

    NOTE: `hermes profile delete` prompts for confirmation — feed the name on stdin.
    """
    name = profile_name_for(personality_id)
    try:
        result = subprocess.run(
            [hermes_bin(), "profile", "delete", name],
            input=name + "\n",
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
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

# Persistent HTTP session for warm-daemon turns (connection pooling / keep-alive).
_daemon_http_session = None
_daemon_http_session_lock = None


def _perf_ms(start: float) -> float:
    """Elapsed milliseconds since ``start`` (from time.monotonic())."""
    return (time.monotonic() - start) * 1000.0


def daemon_turn_url() -> str:
    """POST target for a warm-daemon turn. Trailing path is always /turn."""
    override = os.environ.get("PIB_HERMES_DAEMON_URL")
    if override:
        base = override.rstrip("/")
        return base if base.endswith("/turn") else base + "/turn"
    return DEFAULT_DAEMON_TURN_URL


def daemon_profile_url() -> str:
    """POST target for canonical profile provisioning."""
    return daemon_turn_url().removesuffix("/turn") + "/profile"


def provision_profile(
    personality_id: str,
    personality_name: Optional[str] = None,
    soul_text: Optional[str] = None,
    timeout: int = 60,
) -> dict:
    """Ask the Hermes daemon to create or repair a complete profile."""
    try:
        import requests
    except ImportError as exc:
        raise RuntimeError(
            "requests is required for Hermes profile provisioning"
        ) from exc

    payload = {"personality_id": personality_id}
    if personality_name is not None:
        payload["personality_name"] = personality_name
    if soul_text is not None:
        payload["soul_text"] = soul_text

    session = _get_daemon_session()
    post = session.post if session is not None else requests.post
    try:
        response = post(daemon_profile_url(), json=payload, timeout=timeout)
    except requests.exceptions.RequestException as exc:
        raise RuntimeError(f"Hermes profile daemon is unreachable: {exc}") from exc

    try:
        result = response.json()
    except ValueError as exc:
        raise RuntimeError("Hermes profile daemon returned invalid JSON") from exc
    if (
        response.status_code >= 300
        or not isinstance(result, dict)
        or not result.get("ok")
    ):
        error = result.get("error") if isinstance(result, dict) else None
        raise RuntimeError(
            error or f"Hermes profile daemon returned {response.status_code}"
        )
    return result


def _get_daemon_session():
    """Lazy singleton ``requests.Session`` for pooled daemon HTTP calls."""
    global _daemon_http_session, _daemon_http_session_lock
    try:
        import requests
        import threading
    except ImportError:
        return None

    if _daemon_http_session_lock is None:
        _daemon_http_session_lock = threading.Lock()

    with _daemon_http_session_lock:
        if _daemon_http_session is None:
            _daemon_http_session = requests.Session()
        return _daemon_http_session


def is_warm_daemon_active(timeout: float = 0.15) -> bool:
    """True when the warm Hermes daemon answers GET /health quickly.

    Used to skip expensive filesystem profile re-validation on the hot path
    when turns can be served by the already-warm process at 127.0.0.1:8088.
    """
    try:
        from public_api_client import hermes_daemon
    except ImportError:
        return False
    try:
        return hermes_daemon.is_daemon_reachable(timeout=timeout)
    except Exception:
        return False


def _try_daemon_turn(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = DEFAULT_DISABLED_TOOLSETS,
    max_turns: int = DEFAULT_MAX_TURNS,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
    enabled_toolsets: Optional[str] = DEFAULT_ENABLED_TOOLSETS,
) -> Optional[str]:
    """POST /turn to the warm daemon. None means unreachable or non-200."""
    try:
        import requests
    except ImportError:
        return None

    payload = {
        "text": text,
        "chat_id": chat_id,
        "timeout": timeout,
        "max_turns": max_turns,
    }
    if personality_id is not None:
        payload["personality_id"] = personality_id
    if toolsets is not None:
        payload["toolsets"] = toolsets
    if enabled_toolsets is not None:
        payload["enabled_toolsets"] = enabled_toolsets

    http_start = time.monotonic()
    logging.info(
        "[PERF_TRACE] DAEMON_HTTP_START chat=%s elapsed_ms=0.00",
        chat_id,
    )

    session = _get_daemon_session()
    post = session.post if session is not None else requests.post

    try:
        response = post(
            daemon_turn_url(),
            json=payload,
            timeout=timeout,
        )
    except requests.exceptions.RequestException as exc:
        logging.debug(
            "hermes daemon unreachable (chat=%s): %s; falling back to subprocess",
            chat_id,
            exc,
        )
        return None

    ttft_ms = _perf_ms(http_start)
    logging.info(
        "[PERF_TRACE] DAEMON_TTFT_MS chat=%s elapsed_ms=%.2f status=%s",
        chat_id,
        ttft_ms,
        response.status_code,
    )

    if response.status_code != 200:
        logging.warning(
            "hermes daemon returned %s (chat=%s); falling back to subprocess",
            response.status_code,
            chat_id,
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


def stream_turn(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = DEFAULT_DISABLED_TOOLSETS,
    max_turns: int = DEFAULT_MAX_TURNS,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
    enabled_toolsets: Optional[str] = DEFAULT_ENABLED_TOOLSETS,
) -> Iterator[str]:
    """Yield daemon text deltas.

    Streaming is deliberately daemon-only. A transport or protocol failure
    raises so the voice node can retry through the established non-streaming
    path, including its subprocess fallback.
    """
    try:
        import requests
    except ImportError as exc:
        raise RuntimeError("requests is required for Hermes streaming") from exc

    payload = {
        "text": text,
        "chat_id": chat_id,
        "timeout": timeout,
        "max_turns": max_turns,
        "stream": True,
    }
    if personality_id is not None:
        payload["personality_id"] = personality_id
    if toolsets is not None:
        payload["toolsets"] = toolsets
    if enabled_toolsets is not None:
        payload["enabled_toolsets"] = enabled_toolsets

    session = _get_daemon_session()
    post = session.post if session is not None else requests.post
    try:
        response = post(
            daemon_turn_url(),
            json=payload,
            timeout=timeout,
            stream=True,
        )
        response.raise_for_status()
        saw_final = False
        assembled = ""
        for raw_line in response.iter_lines(decode_unicode=True):
            if not raw_line:
                continue
            data = json.loads(raw_line)
            if not isinstance(data, dict):
                raise ValueError("Hermes stream chunk must be a JSON object")
            if "error" in data:
                raise RuntimeError(str(data["error"]))
            delta = data.get("delta")
            if delta is not None:
                if not isinstance(delta, str):
                    raise ValueError("Hermes stream delta must be a string")
                if delta:
                    assembled += delta
                    yield delta
            if "reply" in data:
                final_reply = data["reply"]
                if not isinstance(final_reply, str):
                    raise ValueError("Hermes final stream reply must be a string")
                if final_reply.startswith(assembled):
                    remainder = final_reply[len(assembled) :]
                    if remainder:
                        yield remainder
                elif final_reply != assembled:
                    yield final_reply
                saw_final = True
        if not saw_final:
            raise RuntimeError("Hermes stream ended without a final reply")
    except requests.exceptions.RequestException as exc:
        raise RuntimeError(f"Hermes streaming request failed: {exc}") from exc
    finally:
        if "response" in locals():
            response.close()


def run_turn_subprocess(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = DEFAULT_DISABLED_TOOLSETS,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
    enabled_toolsets: Optional[str] = DEFAULT_ENABLED_TOOLSETS,
) -> str:
    """Run one turn via a oneshot Hermes CLI subprocess. Always returns text."""
    if not hermes_binary_available():
        logging.error(
            "hermes binary %s is missing or not executable (chat=%s); "
            "answering with the fallback reply. Install the hermes CLI for the "
            "pib user and check the PIB_HERMES_BIN mount.",
            hermes_bin(),
            chat_id,
        )
        return FALLBACK_REPLY

    # Hermes CLI -t is an enabled-toolset selector, not a blacklist. Prefer the
    # voice allowlist; explicit legacy non-voice selections retain their existing
    # CLI plumbing.
    cli_toolsets = enabled_toolsets
    if cli_toolsets is None and toolsets != DEFAULT_DISABLED_TOOLSETS:
        cli_toolsets = toolsets
    cmd = build_command(text, chat_id, personality_id, cli_toolsets)
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
            result.returncode,
            chat_id,
            (result.stderr or "")[:500],
        )
        return FALLBACK_REPLY

    reply = (result.stdout or "").strip()
    return reply or FALLBACK_REPLY


def run_turn(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = DEFAULT_DISABLED_TOOLSETS,
    max_turns: int = DEFAULT_MAX_TURNS,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
    enabled_toolsets: Optional[str] = DEFAULT_ENABLED_TOOLSETS,
) -> str:
    """Run one conversational turn. Always returns speakable text.

    Prefers the warm localhost daemon (POST /turn) without requiring a local
    Hermes binary check first. If the daemon is unreachable or fails, falls
    back to a oneshot ``subprocess.run`` of the Hermes CLI (which does check
    the binary).
    """
    t0 = time.monotonic()
    logging.info(
        "[PERF_TRACE] HERMES_CLIENT_START chat=%s elapsed_ms=0.00",
        chat_id,
    )

    # Try the warm daemon first — no filesystem binary/profile checks on this path.
    daemon_reply = _try_daemon_turn(
        text,
        chat_id,
        personality_id,
        toolsets,
        max_turns,
        timeout=timeout,
        enabled_toolsets=enabled_toolsets,
    )
    if daemon_reply is not None:
        logging.info(
            "[PERF_TRACE] HERMES_CLIENT_DONE chat=%s via=daemon elapsed_ms=%.2f",
            chat_id,
            _perf_ms(t0),
        )
        return daemon_reply

    if not hermes_binary_available():
        # Distinct from a timeout: the agent was never started at all.
        logging.error(
            "hermes binary %s is missing or not executable (chat=%s); "
            "answering with the fallback reply. Install the hermes CLI for the "
            "pib user and check the PIB_HERMES_BIN mount.",
            hermes_bin(),
            chat_id,
        )
        logging.info(
            "[PERF_TRACE] HERMES_CLIENT_DONE chat=%s via=fallback elapsed_ms=%.2f",
            chat_id,
            _perf_ms(t0),
        )
        return FALLBACK_REPLY

    reply = run_turn_subprocess(
        text,
        chat_id,
        personality_id,
        toolsets,
        timeout=timeout,
        enabled_toolsets=enabled_toolsets,
    )
    logging.info(
        "[PERF_TRACE] HERMES_CLIENT_DONE chat=%s via=subprocess elapsed_ms=%.2f",
        chat_id,
        _perf_ms(t0),
    )
    return reply


def delete_session(chat_id: str, timeout: int = 30) -> bool:
    """Remove the Hermes session backing this pib chat. Best-effort."""
    try:
        result = subprocess.run(
            [hermes_bin(), "sessions", "delete", session_name_for(chat_id)],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return result.returncode == 0
    except Exception as exc:
        logging.warning("could not delete hermes session for %s: %s", chat_id, exc)
        return False
