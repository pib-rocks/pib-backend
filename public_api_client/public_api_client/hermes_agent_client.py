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
import threading
import time
from typing import Iterator, Optional

import yaml
from pib_hermes_config import (
    DEFAULT_SOUL,
    PROFILE_CONFIG_VERSION,
    PROFILE_CONFIG_VERSION_KEY,
    PROFILE_PREFIX,
    PROFILE_TOOL_SEARCH_CONFIG,
    REASONING_EFFORT_ERROR,
    REASONING_EFFORTS,
    align_profile_ownership,
    build_default_soul_text,
    env_vars_for_provider,
    provider_needs_key,
    profile_dir_for,
    profile_name_for,
    profiles_dir,
    soul_path_for,
)


def motion_model_prompt_for_profile() -> str:
    """Compact motion-model text for profile prompts, or '' when unavailable."""
    try:
        from pib_api_client import motion_model_client

        successful, payload = motion_model_client.get_motion_model()
    except Exception:
        return ""
    if not successful or not isinstance(payload, dict):
        return ""
    prompt = payload.get("prompt")
    if not isinstance(prompt, str) or not prompt.strip():
        return ""
    return prompt.strip()


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
DEFAULT_ENABLED_TOOLSETS_WITHOUT_MCP = os.environ.get(
    "PIB_HERMES_ENABLED_TOOLSETS_WITHOUT_MCP", "vision"
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
# PIB_MCP_ENABLE_ACTUATION is written explicitly for the same reason: an omitted
# value leaves the actuator gate closed. The shipped value is true, matching
# docker-compose.yaml. The single definition of that entry — setup/setup-pib.sh
# seeds the same one and tests/unit/test_setup_hermes_model_pin.py fails if the
# two ever drift.
# Interpreter that runs the MCP server. It must be the one the installers put pib_mcp_server AND its
# runtime dependencies into (Dockerfile and setup-pib.sh both install into the system Python with
# --break-system-packages). A bare "python3" resolves to the Hermes install venv when the CLI spawns
# the server, and that venv carries pib_mcp_server without websocket-client - every tool that goes
# through rosbridge then failed with an ImportError while the read tools kept working (PC-71).
MCP_SERVER_PYTHON = os.getenv("PIB_MCP_PYTHON", "/usr/bin/python3")

PIB_MCP_SERVER = {
    "command": MCP_SERVER_PYTHON,
    "args": ["-m", "pib_mcp_server"],
    "env": {
        "FLASK_API_BASE_URL": os.getenv("FLASK_API_BASE_URL", "http://flask-app:5000"),
        "PIB_MCP_API_BASE_URL": os.getenv(
            "FLASK_API_BASE_URL", "http://flask-app:5000"
        ),
        "PIB_MCP_ROSBRIDGE_URL": os.getenv(
            "PIB_MCP_ROSBRIDGE_URL", "ws://rosbridge-ws:9090"
        ),
        "PIB_MCP_ENABLE_ACTUATION": "true",
    },
}

# Permanent Hermes LLM pin. Kept in sync with setup/setup-pib.sh
# and pib_hermes_config.DEFAULT_HERMES_MODEL.
DEFAULT_HERMES_MODEL = "gemini-3.8-flash"
DEFAULT_HERMES_LITE_MODEL = "gemini-3.5-flash-lite"
DEFAULT_HERMES_PROVIDER = "gemini"
# High-speed defaults for Gemini Flash / Flash-Lite (PR-1524).
# Written into the agent block. Hermes reads agent.reasoning_effort and
# ignores the same keys at the config root.
# Thinking off by default (PR-2039), see pib_hermes_config.DEFAULT_REASONING_EFFORT.
DEFAULT_REASONING_EFFORT = "none"

# Provider name of the pib.Cloud route. The cloud decides the reasoning effort of its own upstream
# call (Settings.reasoning_effort), so a value in the profile only travels with every request and
# its API refuses the field - measured as a 400 on `reasoning_effort` (PC-69).
PIB_CLOUD_PROVIDER = "pib-cloud"

# Request fields a provider may refuse on the wire (measured on pib.Cloud, PC-69).
REFUSED_REQUEST_FIELD_REASONING_EFFORT = "reasoning_effort"
REFUSED_REQUEST_FIELD_STREAM_OPTIONS = "stream_options"
REFUSED_REQUEST_FIELD_RESPONSE_FORMAT = "response_format"
REFUSED_REQUEST_FIELDS = frozenset(
    {
        REFUSED_REQUEST_FIELD_REASONING_EFFORT,
        REFUSED_REQUEST_FIELD_STREAM_OPTIONS,
        REFUSED_REQUEST_FIELD_RESPONSE_FORMAT,
    }
)

_provider_refused_fields: dict[tuple[str, str, str], set[str]] = {}
_provider_refused_fields_lock = threading.Lock()


def provider_capability_key(
    provider: str,
    base_url: Optional[str],
    model: str,
) -> tuple[str, str, str]:
    """Process-local cache key for what a route accepts."""
    normalized_provider = (provider or "").strip()
    normalized_model = (model or "").strip()
    normalized_base = (base_url or "").strip().rstrip("/")
    return (normalized_provider, normalized_base, normalized_model)


def clear_provider_refused_fields_cache() -> None:
    """Drop cached refusals (tests and daemon shutdown)."""
    with _provider_refused_fields_lock:
        _provider_refused_fields.clear()


def record_provider_refused_fields(
    provider: str,
    base_url: Optional[str],
    model: str,
    *fields: str,
) -> None:
    """Remember fields this route already refused once in this process."""
    if not fields:
        return
    key = provider_capability_key(provider, base_url, model)
    with _provider_refused_fields_lock:
        bucket = _provider_refused_fields.setdefault(key, set())
        for field in fields:
            if field in REFUSED_REQUEST_FIELDS:
                bucket.add(field)
    if REFUSED_REQUEST_FIELD_RESPONSE_FORMAT in fields:
        prime_auxiliary_response_format_rejection(provider, base_url, model)


#: Fields a route refuses by contract, known before the first request. pib.Cloud
#: rejects all three (PC-69), and probing them costs a 400 plus a retry on the
#: first turn of every daemon start (measured 2026-10-25: 8.2 s first token
#: versus 815 ms on the next chat).
KNOWN_PROVIDER_REFUSALS = {
    PIB_CLOUD_PROVIDER: frozenset(
        {
            REFUSED_REQUEST_FIELD_REASONING_EFFORT,
            REFUSED_REQUEST_FIELD_STREAM_OPTIONS,
            REFUSED_REQUEST_FIELD_RESPONSE_FORMAT,
        }
    )
}


def provider_refuses_field(
    provider: str,
    base_url: Optional[str],
    model: str,
    field: str,
) -> bool:
    known = KNOWN_PROVIDER_REFUSALS.get((provider or "").strip(), frozenset())
    if field in known:
        return True
    key = provider_capability_key(provider, base_url, model)
    with _provider_refused_fields_lock:
        return field in _provider_refused_fields.get(key, set())


CLOUD_ROUTE_DEFAULT_BASE_URLS = {
    "pib-cloud": "https://api.pib.rocks/v1",
}
"""Cloud routes may not live on the LAN (PR-2096).

A profile provisioned once against a local dev proxy kept that URL forever,
because the provider entry is only rewritten when a caller passes a base_url.
On .172 that made every smart turn answer "pib-cloud rejected your API key"
while the key store and the provider health were fine.
"""


def _is_private_base_url(value: str) -> bool:
    """True for loopback, link-local or RFC1918 hosts (a LAN address)."""

    import ipaddress
    from urllib.parse import urlparse

    host = (urlparse(value).hostname or "").strip().lower()
    if not host:
        return False
    if host in {"localhost", "localhost.localdomain"}:
        return True
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        return False
    return bool(address.is_private or address.is_loopback or address.is_link_local)


def _authoritative_cloud_base_url(provider: str) -> Optional[str]:
    """Where a cloud route must point, from the environment or the documented default."""

    default = CLOUD_ROUTE_DEFAULT_BASE_URLS.get(provider)
    if default is None:
        return None
    env_value = os.getenv(
        f"PIB_{provider.replace('-', '_').upper()}_BASE_URL", ""
    ).strip()
    return env_value.rstrip("/") or default


def _guard_cloud_route_base_url(
    provider: str, base_url: Optional[str]
) -> Optional[str]:
    """Replace a LAN address on a cloud route with the authoritative URL."""

    if not base_url:
        return base_url
    authoritative = _authoritative_cloud_base_url(provider)
    if authoritative is None:
        return base_url
    if not _is_private_base_url(base_url):
        return base_url
    if base_url.rstrip("/") == authoritative:
        return base_url
    logging.warning(
        "provider %s pointed at a LAN address (%s); using %s instead "
        "(a device must not talk to a dev proxy)",
        provider,
        base_url,
        authoritative,
    )
    return authoritative


def _route_base_url_from_cfg(cfg: dict, provider: str) -> Optional[str]:
    providers = cfg.get(PROVIDERS_CONFIG_KEY)
    if not isinstance(providers, dict):
        return None
    entry = providers.get(provider)
    if not isinstance(entry, dict):
        return None
    base_url = entry.get("base_url")
    if isinstance(base_url, str) and base_url.strip():
        return base_url.strip().rstrip("/")
    return None


def apply_provider_capability_cache_to_cfg(
    cfg: dict,
    *,
    provider: Optional[str] = None,
    base_url: Optional[str] = None,
    model: Optional[str] = None,
) -> bool:
    """Strip refused fields from the profile mapping. Returns whether cfg changed."""
    resolved_provider = provider or cfg.get("provider") or ""
    resolved_model = model or cfg.get("model") or ""
    resolved_base = base_url or _route_base_url_from_cfg(cfg, resolved_provider)
    agent = cfg.get("agent")
    if not isinstance(agent, dict):
        return False
    changed = False
    if provider_refuses_field(
        resolved_provider,
        resolved_base,
        resolved_model,
        REFUSED_REQUEST_FIELD_REASONING_EFFORT,
    ):
        if REFUSED_REQUEST_FIELD_REASONING_EFFORT in agent:
            del agent[REFUSED_REQUEST_FIELD_REASONING_EFFORT]
            changed = True
    return changed


def prime_auxiliary_response_format_rejection(
    provider: str,
    base_url: Optional[str],
    model: str,
    format_type: str = "json_schema",
) -> None:
    """Tell Hermes' auxiliary client not to send this response_format type again."""
    try:
        from agent.auxiliary_structured_output import _REJECTED_ROUTES, _route_key
    except ImportError:
        return
    _REJECTED_ROUTES.add(
        (_route_key(provider, base_url), str(model or ""), format_type)
    )


def title_generation_extra_body(
    provider: str,
    base_url: Optional[str],
    model: str,
    extra_body: dict,
) -> dict:
    """Extra body for a title call, minus response_format when the route refused it."""
    if not provider_refuses_field(
        provider, base_url, model, REFUSED_REQUEST_FIELD_RESPONSE_FORMAT
    ):
        return extra_body
    if REFUSED_REQUEST_FIELD_RESPONSE_FORMAT not in extra_body:
        return extra_body
    return {
        key: value
        for key, value in extra_body.items()
        if key != REFUSED_REQUEST_FIELD_RESPONSE_FORMAT
    }


# The tools a robot personality may see (decision 2026-10-16, extended the same day): the MCP
# server's tools, memory, and the Hermes skills, so a personality can read its own skills. Before this
# the profile was cloned with the whole Hermes tool zoo, so the cloud model picked vision_analyze or
# the generic tool dispatcher and narrated an answer instead of calling the robot tool (PC-70).
ROBOT_TOOLSETS = ("pib", "memory", "skills")
ROBOT_TOOLSETS_WITHOUT_MCP = ("memory", "skills")

# The Hermes platform the robot's agent runs as (measured: the daemon's turn logs platform=cli), and
# the config key that maps a platform to its toolsets.
ROBOT_PLATFORM = "cli"
PLATFORM_TOOLSETS_KEY = "platform_toolsets"
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


def _coerce_profile_config_version(value: object) -> Optional[int]:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str) and value.isdigit():
        return int(value)
    return None


def read_profile_config_version(profile_dir: str) -> Optional[int]:
    """The ``config_version`` a profile's config.yaml declares, if any."""
    cfg = _load_config_mapping(os.path.join(profile_dir, CONFIG_FILENAME))
    return _coerce_profile_config_version(cfg.get(PROFILE_CONFIG_VERSION_KEY))


def profile_config_version_is_current(profile_dir: str) -> bool:
    """True when the on-disk version matches what the profile writer would emit."""
    version = read_profile_config_version(profile_dir)
    return version == PROFILE_CONFIG_VERSION


def profile_config_drift_messages(cfg: dict) -> list[str]:
    """Human-readable drift lines for ``--check`` without rewriting the file."""
    messages: list[str] = []
    version = _coerce_profile_config_version(cfg.get(PROFILE_CONFIG_VERSION_KEY))
    if version != PROFILE_CONFIG_VERSION:
        messages.append(
            f"{PROFILE_CONFIG_VERSION_KEY}: {version!r} (expected {PROFILE_CONFIG_VERSION})"
        )

    tools = cfg.get("tools")
    tool_search = tools.get("tool_search") if isinstance(tools, dict) else None
    if not isinstance(tool_search, dict):
        tool_search = {}
    desired_enabled = PROFILE_TOOL_SEARCH_CONFIG["tool_search"]["enabled"]
    if tool_search.get("enabled") != desired_enabled:
        messages.append(
            "tools.tool_search.enabled: "
            f"{tool_search.get('enabled')!r} (expected {desired_enabled!r})"
        )

    auxiliary = cfg.get("auxiliary")
    title_generation = (
        auxiliary.get("title_generation") if isinstance(auxiliary, dict) else None
    )
    if not isinstance(title_generation, dict):
        title_generation = {}
    if title_generation.get("model_upgrade_enabled") is not False:
        messages.append(
            "auxiliary.title_generation.model_upgrade_enabled: "
            f"{title_generation.get('model_upgrade_enabled')!r} (expected False)"
        )
    return messages


def profile_config_drift(profile_dir: str) -> list[str]:
    """Drift messages for one profile directory."""
    cfg = _load_config_mapping(os.path.join(profile_dir, CONFIG_FILENAME))
    return profile_config_drift_messages(cfg)


def iter_personality_profile_dirs(
    profiles_root: Optional[str] = None,
) -> list[tuple[str, str]]:
    """``(personality_id, profile_dir)`` for each ``pib_*`` profile on disk."""
    root = profiles_root or profiles_dir()
    if not os.path.isdir(root):
        return []
    discovered: list[tuple[str, str]] = []
    prefix = PROFILE_PREFIX
    for entry in sorted(os.listdir(root)):
        if not entry.startswith(prefix):
            continue
        profile_dir = os.path.join(root, entry)
        if not os.path.isdir(profile_dir):
            continue
        personality_id = entry[len(prefix) :]
        if not personality_id:
            continue
        discovered.append((personality_id, profile_dir))
    return discovered


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


def profile_tool_calling_enabled(profile_dir: Optional[str] = None) -> bool:
    """Whether MCP robot tools (``pib``) are enabled in the profile's config.yaml."""
    directory = profile_dir or _current_hermes_home()
    cfg = _load_config_mapping(os.path.join(directory, CONFIG_FILENAME))
    servers = cfg.get("mcp_servers")
    if isinstance(servers, dict):
        pib_entry = servers.get("pib")
        if isinstance(pib_entry, dict) and pib_entry.get("enabled") is False:
            return False
    toolsets = cfg.get(PLATFORM_TOOLSETS_KEY)
    if isinstance(toolsets, dict):
        cli_tools = toolsets.get(ROBOT_PLATFORM)
        if isinstance(cli_tools, list) and "pib" not in cli_tools:
            return False
    return True


def enabled_toolsets_for_turn(
    tool_calling: bool = True,
    enabled_toolsets: Optional[str] = None,
) -> str:
    """Resolve the comma-separated enabled toolsets for a voice turn."""
    raw = enabled_toolsets or DEFAULT_ENABLED_TOOLSETS
    if tool_calling:
        return raw
    items = [
        item.strip()
        for item in raw.split(",")
        if item.strip() and item.strip() not in ("pib", "mcp-pib")
    ]
    return ",".join(items) or DEFAULT_ENABLED_TOOLSETS_WITHOUT_MCP


def disabled_toolsets_for_turn(
    tool_calling: bool = True,
    toolsets: Optional[str] = None,
) -> str:
    """Resolve the comma-separated disabled toolsets for a voice turn."""
    raw = DEFAULT_DISABLED_TOOLSETS if toolsets is None else toolsets
    if tool_calling:
        return raw
    items = [item.strip() for item in raw.split(",") if item.strip()]
    if "pib" not in items:
        items.append("pib")
    return ",".join(items)


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
    # The interpreter is repaired too, but only where the shipped default is still in place: a bare
    # "python3" resolves to the Hermes venv when the CLI spawns the server, and that venv carries
    # pib_mcp_server without websocket-client (PC-71). A deliberate absolute path stays untouched.
    shipped_args = entry.get("args") in (None, PIB_MCP_SERVER["args"])
    if shipped_args and entry.get("command") in (None, "", "python3"):
        if entry.get("command") != MCP_SERVER_PYTHON:
            entry["command"] = MCP_SERVER_PYTHON
            changed = True
    return changed


def _agent_block(cfg: dict) -> tuple[dict, bool]:
    """The config's agent mapping, created when it is missing or not a mapping."""
    agent = cfg.get("agent")
    if isinstance(agent, dict):
        return agent, False
    agent = {}
    cfg["agent"] = agent
    return agent, True


def _apply_agent_speed_defaults(
    cfg: dict,
    reasoning_effort: Optional[str] = None,
    *,
    personality_reasoning: bool = False,
    provider: Optional[str] = None,
) -> bool:
    """Write speed settings into the agent block. Returns whether cfg changed.

    Hermes reads ``agent.reasoning_effort`` (and the speed keys beside it).
    The same keys at the config root are never read, so they are not written.

    ``personality_reasoning`` means the personality column is the authority.
    A string overwrites ``agent.reasoning_effort``. ``None`` is unmanaged and
    leaves whatever the profile already has. Without that flag, a missing
    agent key is seeded with the speed default and an existing value is kept.
    ``max_tokens`` and ``temperature`` are seeded only when absent.
    """
    agent, changed = _agent_block(cfg)
    if provider == PIB_CLOUD_PROVIDER:
        # The cloud sets the effort itself; every value here is one more field the cloud refuses.
        # Purging rather than skipping also repairs profiles that already carry it (PC-69).
        if "reasoning_effort" in agent:
            del agent["reasoning_effort"]
            changed = True
        if "max_tokens" not in agent:
            agent["max_tokens"] = DEFAULT_MAX_TOKENS
            changed = True
        if "temperature" not in agent:
            agent["temperature"] = DEFAULT_TEMPERATURE
            changed = True
        return changed
    if personality_reasoning:
        if reasoning_effort is not None:
            if reasoning_effort not in REASONING_EFFORTS:
                raise ValueError(REASONING_EFFORT_ERROR)
            if agent.get("reasoning_effort") != reasoning_effort:
                agent["reasoning_effort"] = reasoning_effort
                changed = True
    elif "reasoning_effort" not in agent:
        agent["reasoning_effort"] = DEFAULT_REASONING_EFFORT
        changed = True
    if "max_tokens" not in agent:
        agent["max_tokens"] = DEFAULT_MAX_TOKENS
        changed = True
    if "temperature" not in agent:
        agent["temperature"] = DEFAULT_TEMPERATURE
        changed = True
    return changed


def _apply_profile_tool_search_defaults(cfg: dict) -> bool:
    """Keep robot MCP tools eager (PR-2053): pin ``tools.tool_search.enabled`` off.

    Hermes defaults defer MCP/plugin tools behind the tool_search bridge; each
    deferred tool costs an extra model round trip. The backend repairs this on
    every provisioning pass so rewrites cannot drift.
    """
    desired = PROFILE_TOOL_SEARCH_CONFIG
    tools = cfg.get("tools")
    if not isinstance(tools, dict):
        tools = {}
        cfg["tools"] = tools
    tool_search = tools.get("tool_search")
    if not isinstance(tool_search, dict):
        tool_search = {}
        tools["tool_search"] = tool_search
    changed = False
    for key, value in desired["tool_search"].items():
        if tool_search.get(key) != value:
            tool_search[key] = value
            changed = True
    return changed


def _apply_profile_auxiliary_defaults(cfg: dict) -> bool:
    """Keep the derived chat title instead of paying a model call for it (PR-2058).

    Hermes upgrades the instantly derived title with an auxiliary model call. On a
    route that refuses ``reasoning_effort`` and ``response_format`` (pib.Cloud, PC-69)
    that call is rejected twice per new chat - measured 2026-10-25 18:11 in the
    daemon log - before the title is given up. The derived title is enough for the
    chat list, so the upgrade is pinned off in every personality profile.
    """
    auxiliary = cfg.get("auxiliary")
    if not isinstance(auxiliary, dict):
        auxiliary = {}
        cfg["auxiliary"] = auxiliary
    title_generation = auxiliary.get("title_generation")
    if not isinstance(title_generation, dict):
        title_generation = {}
        auxiliary["title_generation"] = title_generation
    if title_generation.get("model_upgrade_enabled") is False:
        return False
    title_generation["model_upgrade_enabled"] = False
    return True


def _ensure_mcp_servers_pib(
    pdir: str,
    model: Optional[str] = None,
    provider: Optional[str] = None,
    base_url: Optional[str] = None,
    env_vars: tuple[str, ...] = (),
    reasoning_effort: Optional[str] = None,
    *,
    personality_reasoning: bool = False,
    tool_calling: Optional[bool] = None,
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

    Speed defaults (reasoning_effort, max_tokens, temperature) are written into
    the agent block, and only when that key is absent, so an operator's or a
    model's own value survives. A personality reasoning level, when given,
    overwrites agent.reasoning_effort. NULL (personality_reasoning with no
    level) leaves the profile's existing level alone. Root-level copies of
    these keys are not written: Hermes does not read them.

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

    existing_servers = cfg.get("mcp_servers")
    existing_pib = (
        existing_servers.get("pib") if isinstance(existing_servers, dict) else None
    )
    existing_enabled = (
        existing_pib.get("enabled") if isinstance(existing_pib, dict) else None
    )
    existing_toolsets = cfg.get(PLATFORM_TOOLSETS_KEY)
    existing_cli_tools = (
        existing_toolsets.get(ROBOT_PLATFORM)
        if isinstance(existing_toolsets, dict)
        else None
    )
    if tool_calling is not None:
        effective_tool_calling = bool(tool_calling)
    elif existing_enabled is False or existing_cli_tools == list(
        ROBOT_TOOLSETS_WITHOUT_MCP
    ):
        effective_tool_calling = False
    else:
        effective_tool_calling = True

    # The robot tool surface: the MCP server's tools, memory, and skills when
    # tool_calling is on; memory and skills (without pib) when tool_calling is off.
    desired_toolsets = (
        list(ROBOT_TOOLSETS)
        if effective_tool_calling
        else list(ROBOT_TOOLSETS_WITHOUT_MCP)
    )
    toolsets = cfg.get(PLATFORM_TOOLSETS_KEY)
    if not isinstance(toolsets, dict):
        toolsets = {}
        cfg[PLATFORM_TOOLSETS_KEY] = toolsets
        changed = True
    if toolsets.get(ROBOT_PLATFORM) != desired_toolsets:
        toolsets[ROBOT_PLATFORM] = desired_toolsets
        changed = True

    # Decision 5, corrected: speed defaults live in the agent block.
    if _apply_agent_speed_defaults(
        cfg,
        reasoning_effort,
        personality_reasoning=personality_reasoning,
        provider=desired_provider,
    ):
        changed = True

    if _apply_profile_tool_search_defaults(cfg):
        changed = True

    if _apply_profile_auxiliary_defaults(cfg):
        changed = True

    guarded_base_url = _guard_cloud_route_base_url(
        desired_provider, base_url or _route_base_url_from_cfg(cfg, desired_provider)
    )
    if (
        guarded_base_url
        and base_url is None
        and _route_base_url_from_cfg(cfg, desired_provider) != guarded_base_url
    ):
        base_url = guarded_base_url
    route_base_url = guarded_base_url
    if apply_provider_capability_cache_to_cfg(
        cfg,
        provider=desired_provider,
        base_url=route_base_url,
        model=desired_model,
    ):
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
        entry = copy.deepcopy(PIB_MCP_SERVER)
        servers["pib"] = entry
        changed = True
    elif _merge_missing_mcp_env(entry):
        changed = True

    if tool_calling is not None:
        desired_enabled = bool(tool_calling)
        if entry.get("enabled") is not desired_enabled:
            entry["enabled"] = desired_enabled
            changed = True
    elif not effective_tool_calling and entry.get("enabled") is not False:
        entry["enabled"] = False
        changed = True

    if cfg.get(PROFILE_CONFIG_VERSION_KEY) != PROFILE_CONFIG_VERSION:
        cfg[PROFILE_CONFIG_VERSION_KEY] = PROFILE_CONFIG_VERSION
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
    reasoning_effort: Optional[str] = None,
    *,
    personality_reasoning: bool = False,
    tool_calling: Optional[bool] = None,
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
        reasoning_effort=reasoning_effort,
        personality_reasoning=personality_reasoning,
        tool_calling=tool_calling,
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
    reasoning_effort: Optional[str] = None,
    *,
    personality_reasoning: bool = False,
    tool_calling: Optional[bool] = None,
) -> dict:
    """Ask the Hermes daemon to create or repair a complete profile."""
    try:
        import requests
    except ImportError as exc:
        raise RuntimeError(
            "requests is required for Hermes profile provisioning"
        ) from exc

    payload: dict = {"personality_id": personality_id}
    if personality_name is not None:
        payload["personality_name"] = personality_name
    if soul_text is not None:
        payload["soul_text"] = soul_text
    if personality_reasoning:
        payload["reasoning_effort"] = reasoning_effort
    if tool_calling is not None:
        payload["tool_calling"] = bool(tool_calling)

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
    if not env_vars and not provider_needs_key(provider):
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
    tool_sink: Optional[list],
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = DEFAULT_DISABLED_TOOLSETS,
    max_turns: int = DEFAULT_MAX_TURNS,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
    enabled_toolsets: Optional[str] = DEFAULT_ENABLED_TOOLSETS,
    message_history: Optional[int] = None,
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
    if message_history is not None:
        payload["message_history"] = message_history

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

    if isinstance(tool_sink, list) and isinstance(data.get("toolsUsed"), list):
        tool_sink.extend(data["toolsUsed"])
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
    tool_sink: Optional[list] = None,
    message_history: Optional[int] = None,
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
    if message_history is not None:
        payload["message_history"] = message_history

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
            if isinstance(tool_sink, list) and isinstance(data.get("toolsUsed"), list):
                # The daemon reports the tools of this turn next to the text
                # (decision PC-84); only names and outcomes travel.
                tool_sink.extend(data["toolsUsed"])
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
    tool_sink: Optional[list] = None,
    message_history: Optional[int] = None,
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
        tool_sink,
        text,
        chat_id,
        personality_id,
        toolsets,
        max_turns,
        timeout=timeout,
        enabled_toolsets=enabled_toolsets,
        message_history=message_history,
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

    from public_api_client.hermes_daemon import seed_flask_conversation_for_chat

    seed_flask_conversation_for_chat(
        chat_id, message_history, personality_id=personality_id
    )
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
