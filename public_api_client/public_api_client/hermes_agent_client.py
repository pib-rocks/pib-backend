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
    env_vars_for_provider,
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
# The MCP server is named by its bare `mcp_servers` key (`pib`): the `mcp-<server>`
# alias is only registered after MCP discovery has run, so passing it here makes
# Hermes print "Warning: Unknown toolsets: mcp-pib" on stdout, which lands in the
# chat reply we hand back.
DEFAULT_ENABLED_TOOLSETS = os.environ.get("PIB_HERMES_ENABLED_TOOLSETS", "pib,vision")
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
DEFAULT_HERMES_MODEL = "gemini-3.8-flash"
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

#: Top-level config.yaml key holding user-defined provider routes.
PROVIDERS_CONFIG_KEY = "providers"


def _current_hermes_home() -> str:
    """The active Hermes home: the context-local override first, then env.

    ``hermes_home_scope`` installs the personality's profile directory as a
    context-local override while a turn runs, so reading it back gives the turn
    the profile it belongs to. The override needs Hermes' own constants module;
    without it (a plain voice container) this falls back to HERMES_HOME.
    """
    try:
        from hermes_constants import get_hermes_home

        return str(get_hermes_home())
    except Exception:
        return hermes_home()


def _load_config_mapping(path: str) -> dict:
    """Read a config.yaml into a mapping; an unreadable file is an empty one."""
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, encoding="utf-8") as fh:
            loaded = yaml.safe_load(fh) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return loaded if isinstance(loaded, dict) else {}


def profile_hermes_settings(
    profile_dir: Optional[str] = None,
) -> tuple[str, str, Optional[str], tuple[str, ...]]:
    """The Hermes model/provider settings a profile's config.yaml declares.

    Returns ``(model, provider, base_url, env_vars)``. The backend writes these
    at provision time (see ``_ensure_mcp_servers_pib``); a turn reads them back
    so it runs the personality's model instead of a pinned default. Missing
    values fall back to the pinned Gemini defaults.
    """
    directory = profile_dir or _current_hermes_home()
    cfg = _load_config_mapping(os.path.join(directory, CONFIG_FILENAME))
    model = cfg.get("model") or DEFAULT_HERMES_MODEL
    provider = cfg.get("provider") or DEFAULT_HERMES_PROVIDER
    providers = cfg.get(PROVIDERS_CONFIG_KEY)
    entry = providers.get(provider) if isinstance(providers, dict) else None
    if not isinstance(entry, dict):
        entry = {}
    base_url = entry.get("base_url")
    if not (isinstance(base_url, str) and base_url.strip()):
        base_url = None
    env_vars = env_vars_for_provider(provider, entry.get("key_env"))
    return model, provider, base_url, env_vars


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
    """True when this model name is the historical hermes-agent row.

    Chat routing does not use this. A turn follows the personality's channel
    setting (smart or direct), which is independent of the provider.
    """
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

    The `chat` subcommand carries the turn rather than the top-level -z oneshot
    because only `chat` accepts `--create-if-missing`: a chat's FIRST turn has no
    session yet, and `--continue <name>` alone exits 1 with "No session found
    matching '<name>'". `-Q` keeps the stdout contract to the final response.
    """
    cmd = [hermes_bin()]
    if personality_id:
        cmd += ["-p", profile_name_for(personality_id)]
    cmd += [
        "chat",
        "-Q",
        "--oneshot",
        "-q",
        text,
        "--continue",
        session_name_for(chat_id),
        "--create-if-missing",
    ]
    if toolsets:
        cmd += ["-t", toolsets]
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


def _ensure_mcp_servers_pib(
    pdir: str,
    model: Optional[str] = None,
    provider: Optional[str] = None,
    base_url: Optional[str] = None,
    env_vars: tuple[str, ...] = (),
) -> None:
    """Seed the personality's model/provider and repair mcp_servers.pib.

    The model and provider are the personality's own: the backend passes them
    when it provisions or re-provisions a profile, instead of pinning every
    profile to Gemini Flash (PR-1930b). When a caller does not pass them, an
    existing value in config.yaml is kept and only a profile that has none gets
    the pinned default — the chat-time repair path must not clobber what the
    backend wrote.

    A provider that is a user-defined route (Local, pib.Cloud, anything with a
    ``base_url``) additionally gets a ``providers.<name>`` entry carrying its
    ``base_url`` and, when it needs a key, its ``key_env``.

    Speed defaults (reasoning_effort, max_tokens, temperature) are written only
    when the key is absent, so an operator's or a model's own value survives.

    Runs even when config.yaml already exists. An mcp_servers.pib entry that is
    already there keeps its command/args and only gets the env keys it is
    missing.
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

    # The personality's model/provider win. A caller that did not supply one
    # keeps the profile's existing pair; only a profile that has neither gets
    # the pinned default. The chat-time repair path must not clobber what the
    # backend already wrote.
    instructed = bool(model or provider)
    desired_model = model or cfg.get("model") or DEFAULT_HERMES_MODEL
    desired_provider = provider or cfg.get("provider") or DEFAULT_HERMES_PROVIDER
    if instructed or (not cfg.get("model") and not cfg.get("provider")):
        if cfg.get("model") != desired_model:
            cfg["model"] = desired_model
            changed = True
        if cfg.get("provider") != desired_provider:
            cfg["provider"] = desired_provider
            changed = True

    # Decision 5: speed defaults are a starting point, not a per-model override.
    if "reasoning_effort" not in cfg:
        cfg["reasoning_effort"] = DEFAULT_REASONING_EFFORT
        changed = True
    if "max_tokens" not in cfg:
        cfg["max_tokens"] = DEFAULT_MAX_TOKENS
        changed = True
    if "temperature" not in cfg:
        cfg["temperature"] = DEFAULT_TEMPERATURE
        changed = True

    # A user-defined route is declared under providers.<name>. Built-in
    # providers (gemini/openai/anthropic) need no entry.
    if base_url:
        providers = cfg.get(PROVIDERS_CONFIG_KEY)
        if not isinstance(providers, dict):
            providers = {}
            cfg[PROVIDERS_CONFIG_KEY] = providers
            changed = True
        entry = providers.get(desired_provider)
        if not isinstance(entry, dict):
            entry = {}
            providers[desired_provider] = entry
            changed = True
        if entry.get("name") != desired_provider:
            entry["name"] = desired_provider
            changed = True
        if entry.get("base_url") != base_url:
            entry["base_url"] = base_url
            changed = True
        key_env = env_vars[0] if env_vars else None
        if key_env and entry.get("key_env") != key_env:
            entry["key_env"] = key_env
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
        "ensured hermes model=%s provider=%s and mcp_servers.pib in %s",
        desired_model,
        desired_provider,
        pdir,
    )


def ensure_profile(
    personality_id: str,
    soul_text: str = "",
    timeout: int = 60,
    personality_name: Optional[str] = None,
    model: Optional[str] = None,
    provider: Optional[str] = None,
    base_url: Optional[str] = None,
) -> str:
    """Loud local repair path, executed only where Hermes is importable.

    ``model``/``provider``/``base_url`` are the personality's registry values.
    The backend passes them; a chat-time repair leaves them None so it keeps the
    model the backend already wrote.
    """
    del timeout
    from public_api_client.hermes_daemon import ensure_profile_home

    result = ensure_profile_home(
        personality_id,
        personality_name=personality_name,
        soul_text=soul_text,
        model=model,
        provider=provider,
        base_url=base_url,
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


def provider_key_for_turn() -> Optional[str]:
    """The key for the current profile's provider, from the Flask key store.

    The model and provider come from the profile's config.yaml, which the
    backend writes at provision time. A provider that needs no key (the
    on-device model) returns None instead of a key. A locked, empty or
    mismatched store raises; the environment is not a fallback, and the value
    is not logged.
    """
    model, provider, _base_url, env_vars = profile_hermes_settings()
    if not env_vars:
        return None
    from voice_assistant.direct_tool_loop import resolve_hermes_provider_key

    key, _source = resolve_hermes_provider_key(provider, api_name=model)
    return key


def _provider_key_was_refused(payload: object) -> bool:
    """True when a daemon error is the store's missing-key message.

    That message names the provider and carries no secret. A transport
    failure is a different case and may still use the subprocess path.
    """
    if not isinstance(payload, dict):
        return False
    error = payload.get("error")
    if not isinstance(error, str):
        return False
    # The daemon names whichever provider it could not key, not a fixed one.
    return error.startswith("No keys are available for provider")


def _child_environment(
    provider_key: Optional[str], env_vars: tuple[str, ...] = ()
) -> Optional[dict]:
    """Environment for one Hermes subprocess.

    None leaves the child with this process's environment. A key is placed
    only in the child, under the names Hermes reads for the provider, and is
    not written to the profile.
    """
    if not provider_key or not env_vars:
        return None
    env = os.environ.copy()
    for name in env_vars:
        env[name] = provider_key
    return env


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
        try:
            payload = response.json()
        except ValueError:
            payload = None
        error_text = payload.get("error") if isinstance(payload, dict) else None
        if _provider_key_was_refused(payload):
            from voice_assistant.direct_tool_loop import DirectToolLoopError

            raise DirectToolLoopError(str(error_text))
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
    provider_key_for_turn()
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
                message = str(data["error"])
                if _provider_key_was_refused(data):
                    from voice_assistant.direct_tool_loop import DirectToolLoopError

                    raise DirectToolLoopError(message)
                raise RuntimeError(message)
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
    provider_key: Optional[str] = None,
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
    _model, _provider, _base_url, env_vars = profile_hermes_settings()
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
            env=_child_environment(provider_key, env_vars),
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
    """Run one conversational turn. Returns speakable text.

    A missing provider key raises instead of selecting another provider.
    The warm daemon is preferred. When it is unreachable, the oneshot
    subprocess receives the store key in its own environment.
    """
    t0 = time.monotonic()
    logging.info(
        "[PERF_TRACE] HERMES_CLIENT_START chat=%s elapsed_ms=0.00",
        chat_id,
    )
    provider_key = provider_key_for_turn()

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
        provider_key=provider_key,
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
