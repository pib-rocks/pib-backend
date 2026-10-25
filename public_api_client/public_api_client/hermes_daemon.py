"""Long-lived Hermes Agent daemon for low-latency voice assistant turns.

Exposes a localhost HTTP API so ``hermes_agent_client.run_turn`` can dispatch
to a warm process instead of cold-starting the Hermes CLI on every chat turn.

Endpoints:
  GET  /health  → 200 {"status": "ok"}
  POST /turn    → JSON {text, chat_id, personality_id?, toolsets?, max_turns?}
                  → {"reply": "..."}
                  With stream=true, returns newline-delimited delta/final JSON.
"""

from __future__ import annotations

import contextlib
import glob
import importlib
import inspect
import io
import json
import logging
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path
from typing import Optional

# Force IPv4 preference in socket.getaddrinfo to prevent 10s IPv6 timeouts on Pi networks
_orig_getaddrinfo = socket.getaddrinfo


def _ipv4_preferred_getaddrinfo(*args, **kwargs):
    res = _orig_getaddrinfo(*args, **kwargs)
    ipv4 = [r for r in res if r[0] == socket.AF_INET]
    return ipv4 if ipv4 else res


socket.getaddrinfo = _ipv4_preferred_getaddrinfo
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional
from urllib.error import URLError
from urllib.parse import parse_qs, urlparse
from urllib.request import Request, urlopen

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8088
DEFAULT_BASE_URL = f"http://{DEFAULT_HOST}:{DEFAULT_PORT}"

# How long ensure_daemon_running waits for /health after spawning.
_STARTUP_WAIT_SECONDS = 5.0
_STARTUP_POLL_SECONDS = 0.05

# Process-local server handle started by start_daemon / ensure_daemon_running.
_server: Optional[ThreadingHTTPServer] = None
_server_thread: Optional[threading.Thread] = None
_server_lock = threading.Lock()

TurnRunner = Callable[..., str]

# Model used for the in-process ``run_agent.main`` entry point.
IN_PROCESS_MODEL = "gemini-3.8-flash"
DEFAULT_AGENT_CACHE_SIZE = 32
DEFAULT_PROFILE_FACTORY_MODE = "require"
PROFILE_FACTORY_ENV = "PIB_HERMES_PROFILE_FACTORY"
PROFILE_DIRS = (
    "memories",
    "sessions",
    "skills",
    "skins",
    "logs",
    "plans",
    "workspace",
    "cron",
    "home",
)
CLONE_CONFIG_FILES = ("config.yaml", ".env", "SOUL.md")
CLONE_SUBDIR_FILES = ("memories/MEMORY.md", "memories/USER.md")
_filesystem_factory_warned = False

# Every chat gets its own Hermes session, so history never crosses chats.
SESSION_ID_PREFIX = "pib_chat_"

# Hermes install layout used to import the agent in-process.
DEFAULT_HERMES_HOME = "/home/pib/.hermes"
HERMES_AGENT_DIRNAME = "hermes-agent"
# Cached answer of agent_source_imports_here(); None = not probed yet.
_AGENT_IMPORT_PROBE: Optional[bool] = None
AGENT_IMPORT_PROBE_TIMEOUT = 20.0
_VENV_SITE_PACKAGES_GLOB = os.path.join("venv", "lib", "python3.*", "site-packages")
_PYTHON_VERSION = re.compile(r"python(\d+)\.(\d+)")

# ``run_agent.main`` has no return value: it prints the answer to stdout below a
# "FINAL RESPONSE:" banner, followed by decorative dashes and closing status
# lines. The daemon captures that stdout and extracts the answer from it.
_FINAL_RESPONSE_MARKER = "FINAL RESPONSE:"
_DECORATION_LINE = re.compile(r"^[-=\s]*$")
_TRAILING_MARKERS = (
    "Agent execution completed",
    "Sample trajectory saved to",
    "Failed to save sample",
)

# redirect_stdout swaps a process-global, so parallel /turn requests would read
# each other's output. Serialize the captured runs.
_stdout_capture_lock = threading.Lock()


class _CachedAgent:
    def __init__(
        self,
        agent,
        settings: tuple,
        key_token: str,
    ):
        self.agent = agent
        self.settings = settings
        # The store key this agent was built with. Compared, never logged.
        # A later turn whose store key differs builds a new agent.
        self.key_token = key_token
        self.lock = threading.Lock()


_GEMINI_KEY_NAMES = ("GOOGLE_API_KEY", "GEMINI_API_KEY")


@contextlib.contextmanager
def _provider_key_in_environment(env_names: tuple[str, ...], key: str):
    """Expose the store key under this provider's names while Hermes builds.

    Hermes reads these names from the process environment when it constructs
    its client and keeps the value on the client. Restoring the previous
    environment means a Direct turn does not observe this assignment. The
    secret is not logged.
    """
    if not env_names:
        yield
        return
    previous = {name: os.environ.get(name) for name in env_names}
    try:
        for name in env_names:
            os.environ[name] = key
        yield
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


@contextlib.contextmanager
def _gemini_key_in_environment(key: str):
    """Back-compat wrapper: the Gemini key under its two names."""
    with _provider_key_in_environment(_GEMINI_KEY_NAMES, key):
        yield


def _missing_provider_key(exc: BaseException) -> bool:
    """The store refused a key. The type's name avoids importing the voice package here."""
    return type(exc).__name__ == "DirectToolLoopError"


_agent_cache: OrderedDict[tuple[str, str], _CachedAgent] = OrderedDict()
_agent_cache_lock = threading.Lock()
_mcp_discovery_attempted: set[str] = set()
_mcp_discovery_lock = threading.Lock()

# Last planned provider wire fields per turn (tests assert on this, not timing).
_provider_field_attempts: list[frozenset[str]] = []
_AUXILIARY_CALL_LLM_PATCHED = False

# One SQLite session store per personality home.
_session_dbs: dict[str, object] = {}
_session_db_attempted: set[str] = set()
_session_db_lock = threading.Lock()


def session_id_for_chat(chat_id: str) -> str:
    """Hermes session id backing one pib chat."""
    return f"{SESSION_ID_PREFIX}{chat_id}"


def _profile_layout(personality_id: str) -> dict:
    from pib_hermes_config import profile_dir_for

    profile_dir = profile_dir_for(personality_id)
    return {
        "profile_dir": profile_dir,
        "has_config": os.path.isfile(os.path.join(profile_dir, "config.yaml")),
        "has_memories": os.path.isdir(os.path.join(profile_dir, "memories")),
        "has_sessions": os.path.isdir(os.path.join(profile_dir, "sessions")),
        "has_soul": os.path.isfile(os.path.join(profile_dir, "SOUL.md")),
    }


def _profile_is_complete(profile_dir: str) -> bool:
    return all(
        os.path.isfile(os.path.join(profile_dir, filename))
        for filename in ("config.yaml", ".env")
    ) and all(
        os.path.isdir(os.path.join(profile_dir, dirname)) for dirname in PROFILE_DIRS
    )


_HERMES_SOURCE_PREPARED = False


def _drop_foreign_hermes_modules(agent_dir: str) -> list:
    """Forget ``hermes*`` modules that were imported from outside ``agent_dir``.

    A ``sys.path`` entry only decides what the *next* import finds: a module that was
    imported before the path was prepared keeps its foreign version in ``sys.modules``.
    Live, that produced a mixed pair - the mounted ``hermes_cli`` next to the container's
    older ``hermes_constants`` - and the import died with ``cannot import name
    'LOCAL_RUNTIME_ROOT_DIRS' from 'hermes_constants'``, so no profile was provisioned and
    every new personality answered with the fallback sentence.

    Dropping the foreign copies is cheap: the following import re-reads them from the
    mounted source. Modules without a ``__file__`` (frozen or namespace builds) are kept,
    because their origin cannot be judged.
    """
    root = os.path.abspath(agent_dir)
    dropped = []
    for name, module in list(sys.modules.items()):
        if not (name == "hermes" or name.startswith("hermes")):
            continue
        origin = getattr(module, "__file__", None)
        if not origin:
            continue
        origin_abs = os.path.abspath(origin)
        if origin_abs == root or origin_abs.startswith(root + os.sep):
            continue
        sys.modules.pop(name, None)
        dropped.append(name)
    if dropped:
        logging.info("dropped foreign hermes modules: %s", dropped)
    return dropped


def ensure_hermes_source_on_path() -> str:
    """Put the mounted Hermes source in front of ``sys.path`` and drop foreign Hermes modules.

    Provisioning and turns must resolve the SAME Hermes. Live, provisioning imported
    ``hermes_cli`` from the container's site-packages first; that foreign version then
    stayed in ``sys.modules`` and the turn's ``run_agent`` import failed with an
    ImportError, so every turn silently fell back to the (broken) CLI wrapper and
    answered with the fallback sentence. Returns the source directory.
    """
    global _HERMES_SOURCE_PREPARED
    agent_dir = hermes_agent_dir()
    entries = [agent_dir, *venv_site_packages(agent_dir)]
    for path_entry in entries:
        if path_entry in sys.path:
            sys.path.remove(path_entry)
    for path_entry in reversed(entries):
        if os.path.exists(path_entry):
            sys.path.insert(0, path_entry)
    _drop_foreign_hermes_modules(agent_dir)
    if not _HERMES_SOURCE_PREPARED:
        _HERMES_SOURCE_PREPARED = True
        logging.info(
            "hermes source on sys.path: %s",
            [entry for entry in entries if os.path.exists(entry)],
        )
    _install_auxiliary_capability_guard()
    return agent_dir


def _install_auxiliary_capability_guard() -> None:
    """Strip refused auxiliary fields (e.g. title response_format) before the wire."""
    global _AUXILIARY_CALL_LLM_PATCHED
    if _AUXILIARY_CALL_LLM_PATCHED:
        return
    try:
        import agent.auxiliary_client as auxiliary_client
    except ImportError:
        return
    original = auxiliary_client.call_llm

    def call_llm_with_provider_capabilities(*args, **kwargs):
        from public_api_client.hermes_agent_client import title_generation_extra_body

        task = kwargs.get("task")
        if task == "title_generation":
            extra_body = kwargs.get("extra_body")
            if isinstance(extra_body, dict):
                runtime = kwargs.get("main_runtime")
                if isinstance(runtime, dict):
                    kwargs["extra_body"] = title_generation_extra_body(
                        str(runtime.get("provider") or ""),
                        runtime.get("base_url"),
                        str(runtime.get("model") or ""),
                        extra_body,
                    )
        return original(*args, **kwargs)

    auxiliary_client.call_llm = call_llm_with_provider_capabilities
    _AUXILIARY_CALL_LLM_PATCHED = True


def _would_send_stream_options(agent, base_url: Optional[str]) -> bool:
    try:
        from agent.chat_completion_helpers import is_native_gemini_base_url
    except ImportError:
        is_native_gemini_base_url = lambda _url: False  # noqa: E731
    effective_base = (
        base_url
        or getattr(agent, "base_url", None)
        or getattr(agent, "_base_url", None)
    )
    if is_native_gemini_base_url(effective_base):
        return False
    return not getattr(agent, "_stream_options_unsupported", False)


def _profile_carries_reasoning_effort() -> bool:
    from public_api_client.hermes_agent_client import (
        CONFIG_FILENAME,
        _load_config_mapping,
    )

    cfg = _load_config_mapping(os.path.join(_current_home_key(), CONFIG_FILENAME))
    agent = cfg.get("agent")
    return isinstance(agent, dict) and "reasoning_effort" in agent


def planned_provider_request_fields(
    agent,
    provider: str,
    base_url: Optional[str],
    model: str,
) -> frozenset[str]:
    """Fields the next main chat request is expected to carry."""
    from public_api_client.hermes_agent_client import (
        REFUSED_REQUEST_FIELD_REASONING_EFFORT,
        REFUSED_REQUEST_FIELD_STREAM_OPTIONS,
        provider_refuses_field,
    )

    fields: set[str] = set()
    if (
        _profile_carries_reasoning_effort()
        and not provider_refuses_field(
            provider, base_url, model, REFUSED_REQUEST_FIELD_REASONING_EFFORT
        )
        and not getattr(agent, "_reasoning_effort_rejected", False)
    ):
        fields.add(REFUSED_REQUEST_FIELD_REASONING_EFFORT)
    if _would_send_stream_options(agent, base_url) and not provider_refuses_field(
        provider, base_url, model, REFUSED_REQUEST_FIELD_STREAM_OPTIONS
    ):
        fields.add(REFUSED_REQUEST_FIELD_STREAM_OPTIONS)
    return frozenset(fields)


def _sync_provider_capabilities_to_agent(
    agent,
    provider: str,
    base_url: Optional[str],
    model: str,
) -> None:
    from public_api_client.hermes_agent_client import (
        REFUSED_REQUEST_FIELD_REASONING_EFFORT,
        REFUSED_REQUEST_FIELD_RESPONSE_FORMAT,
        REFUSED_REQUEST_FIELD_STREAM_OPTIONS,
        prime_auxiliary_response_format_rejection,
        provider_refuses_field,
    )

    if provider_refuses_field(
        provider, base_url, model, REFUSED_REQUEST_FIELD_REASONING_EFFORT
    ):
        agent._reasoning_effort_rejected = True
    if provider_refuses_field(
        provider, base_url, model, REFUSED_REQUEST_FIELD_STREAM_OPTIONS
    ):
        agent._stream_options_unsupported = True
    if provider_refuses_field(
        provider, base_url, model, REFUSED_REQUEST_FIELD_RESPONSE_FORMAT
    ):
        prime_auxiliary_response_format_rejection(provider, base_url, model)


def _harvest_provider_refusals_from_agent(
    agent,
    provider: str,
    base_url: Optional[str],
    model: str,
) -> None:
    from public_api_client.hermes_agent_client import (
        REFUSED_REQUEST_FIELD_REASONING_EFFORT,
        REFUSED_REQUEST_FIELD_STREAM_OPTIONS,
        record_provider_refused_fields,
    )

    refused: list[str] = []
    if getattr(agent, "_reasoning_effort_rejected", False):
        refused.append(REFUSED_REQUEST_FIELD_REASONING_EFFORT)
    if getattr(agent, "_stream_options_unsupported", False):
        refused.append(REFUSED_REQUEST_FIELD_STREAM_OPTIONS)
    if not refused:
        return
    record_provider_refused_fields(provider, base_url, model, *refused)
    _repair_profile_for_provider_capabilities()


def _repair_profile_for_provider_capabilities() -> None:
    from public_api_client.hermes_agent_client import (
        _ensure_mcp_servers_pib,
        profile_hermes_settings,
    )

    model, provider, base_url, _env = profile_hermes_settings()
    _ensure_mcp_servers_pib(
        _current_home_key(),
        model=model,
        provider=provider,
        base_url=base_url,
    )


def _profile_dirs_are_complete(profile_dir: str) -> bool:
    return all(
        os.path.isdir(os.path.join(profile_dir, dirname)) for dirname in PROFILE_DIRS
    )


def _filesystem_profile_factory(personality_id: str, profile_dir: str) -> None:
    """Explicit emergency opt-out when the canonical Hermes API is unavailable."""
    global _filesystem_factory_warned
    if not _filesystem_factory_warned:
        logging.warning(
            "%s=filesystem: bypassing the canonical Hermes profile factory",
            PROFILE_FACTORY_ENV,
        )
        _filesystem_factory_warned = True

    from public_api_client.hermes_agent_client import hermes_home

    os.makedirs(profile_dir, exist_ok=True)
    for dirname in PROFILE_DIRS:
        os.makedirs(os.path.join(profile_dir, dirname), exist_ok=True)
    for relative in CLONE_CONFIG_FILES + CLONE_SUBDIR_FILES:
        source = os.path.join(hermes_home(), relative)
        target = os.path.join(profile_dir, relative)
        if os.path.isfile(source) and not os.path.exists(target):
            os.makedirs(os.path.dirname(target), exist_ok=True)
            shutil.copyfile(source, target)
    env_path = os.path.join(profile_dir, ".env")
    if not os.path.exists(env_path):
        Path(env_path).write_text(
            "# Per-profile secrets for this Hermes profile.\n", encoding="utf-8"
        )


def ensure_profile_home(
    personality_id: str,
    personality_name: Optional[str] = None,
    soul_text: Optional[str] = None,
    model: Optional[str] = None,
    provider: Optional[str] = None,
    base_url: Optional[str] = None,
    reasoning_effort: Optional[str] = None,
    *,
    personality_reasoning: bool = False,
    tool_calling: Optional[bool] = None,
) -> dict:
    """Create or repair one complete Hermes home with the canonical factory.

    ``model``/``provider``/``base_url`` are the personality's registry values
    (``provider`` is the registry name, ``base_url`` its endpoint). They are
    written into the profile's config.yaml so the smart chat runs the model the
    personality is set to. Passing none keeps whatever the profile already has.

    ``personality_reasoning`` writes ``reasoning_effort`` into the agent block.
    ``None`` leaves the profile's existing agent.reasoning_effort alone.
    """
    from pib_hermes_config import (
        align_profile_ownership,
        build_default_soul_text,
        profile_dir_for,
        profile_name_for,
        provider_for_profile,
    )
    from public_api_client.hermes_agent_client import _ensure_mcp_servers_pib

    if not isinstance(personality_id, str) or not personality_id:
        raise ValueError("personality_id must be a non-empty string")

    mapping = (
        provider_for_profile(provider, base_url)
        if isinstance(provider, str) and provider.strip()
        else None
    )

    mode = os.environ.get(PROFILE_FACTORY_ENV, DEFAULT_PROFILE_FACTORY_MODE).lower()
    if mode not in {"require", "filesystem"}:
        raise ValueError(f"{PROFILE_FACTORY_ENV} must be 'require' or 'filesystem'")

    profile_dir = profile_dir_for(personality_id)
    complete = _profile_is_complete(profile_dir)
    created = False
    factory = "filesystem" if mode == "filesystem" else "hermes-cli-api"

    if not complete:
        if os.path.exists(profile_dir):
            logging.warning("repairing incomplete Hermes profile at %s", profile_dir)
        if mode == "filesystem":
            _filesystem_profile_factory(personality_id, profile_dir)
            created = True
        else:
            try:
                ensure_hermes_source_on_path()
                from hermes_cli.profiles import create_profile, profile_exists
            except Exception as exc:
                logging.error("Hermes profile factory is unavailable: %s", exc)
                raise RuntimeError("Hermes profile factory is unavailable") from exc
            logging.info(
                "hermes profile factory source: %s",
                inspect.getsourcefile(create_profile) or create_profile.__module__,
            )

            profile_name = profile_name_for(personality_id)
            if profile_exists(profile_name) and not os.path.exists(profile_dir):
                raise RuntimeError(
                    f"Hermes profile {profile_name} resolves outside {profile_dir}"
                )
            backup = None
            if os.path.exists(profile_dir):
                backup = profile_dir + ".incomplete"
                if os.path.exists(backup):
                    shutil.rmtree(backup)
                os.replace(profile_dir, backup)
            try:
                # Pass only the keyword arguments the INSTALLED Hermes factory
                # accepts: the container ships its own hermes_cli (dist-packages)
                # whose create_profile has no `clone_channels`, and passing it
                # made every provisioning attempt fail with a TypeError. A factory
                # that forwards **kwargs (or a test double) gets everything.
                factory_kwargs = {
                    "name": profile_name,
                    "clone_from": None,
                    "clone_all": False,
                    "clone_config": True,
                    "no_alias": True,
                    "no_skills": False,
                    "description": f"pib personality {personality_id}",
                    "clone_channels": False,
                }
                parameters = inspect.signature(create_profile).parameters
                forwards_kwargs = any(
                    parameter.kind is inspect.Parameter.VAR_KEYWORD
                    for parameter in parameters.values()
                )
                if not forwards_kwargs:
                    factory_kwargs = {
                        key: value
                        for key, value in factory_kwargs.items()
                        if key in parameters
                    }
                result = create_profile(**factory_kwargs)
                logging.info(
                    "hermes profile factory created %s (kwargs: %s)",
                    profile_name,
                    ",".join(sorted(factory_kwargs)),
                )
                if os.path.abspath(str(result)) != os.path.abspath(profile_dir):
                    raise RuntimeError(
                        f"Hermes factory created {result}, expected {profile_dir}"
                    )
                if not _profile_dirs_are_complete(profile_dir):
                    raise RuntimeError("Hermes factory returned an incomplete profile")
            except Exception as exc:
                logging.error(
                    "Hermes profile factory failed for %s: %s", personality_id, exc
                )
                if os.path.exists(profile_dir):
                    shutil.rmtree(profile_dir)
                if backup is not None:
                    os.replace(backup, profile_dir)
                raise RuntimeError(f"Hermes profile factory failed: {exc}") from exc
            if backup is not None:
                for filename in ("config.yaml", ".env"):
                    old_file = os.path.join(backup, filename)
                    if os.path.isfile(old_file):
                        shutil.copyfile(old_file, os.path.join(profile_dir, filename))
                shutil.rmtree(backup)
            created = True

    soul = build_default_soul_text(
        personality_name or "pib", custom_description=soul_text or None
    )
    soul_path = os.path.join(profile_dir, "SOUL.md")
    with open(soul_path, "w", encoding="utf-8") as fh:
        fh.write(soul)
    _ensure_mcp_servers_pib(
        profile_dir,
        model=model,
        provider=mapping.provider if mapping else None,
        base_url=mapping.base_url if mapping else None,
        env_vars=mapping.env_vars if mapping else (),
        reasoning_effort=reasoning_effort,
        personality_reasoning=personality_reasoning,
        tool_calling=tool_calling,
    )
    align_profile_ownership(profile_dir)
    os.chmod(profile_dir, 0o700)
    os.chmod(soul_path, 0o644)
    env_path = os.path.join(profile_dir, ".env")
    if os.path.exists(env_path):
        try:
            os.chmod(env_path, 0o600)
        except OSError as exc:
            # PC-45: the voice container sees a filtered view of the Hermes home
            # in which credential files are replaced by read-only, blank covers.
            # Such a placeholder carries no secret, so failing to restrict it is
            # not fatal - aborting here made every Hermes turn answer with the
            # fallback reply inside the recreated voice container. The runtime
            # provider key comes from the key store, never from this file.
            logging.warning(
                "could not restrict %s to 0600 (%s); continuing without it",
                env_path,
                exc,
            )

    return {
        "ok": True,
        "profile_dir": profile_dir,
        "created": created,
        "factory": factory,
    }


@contextlib.contextmanager
def hermes_home_scope(profile_dir: str):
    """Context-local Hermes home override; never mutates process HERMES_HOME."""
    try:
        from hermes_constants import (
            reset_hermes_home_override,
            set_hermes_home_override,
        )
    except ImportError as exc:
        raise RuntimeError("Hermes home override API is unavailable") from exc

    token = set_hermes_home_override(Path(profile_dir))
    try:
        yield
    finally:
        reset_hermes_home_override(token)


def _current_home_key() -> str:
    try:
        from hermes_constants import get_hermes_home

        return str(Path(get_hermes_home()).resolve())
    except ImportError:
        return str(Path(os.environ.get("HERMES_HOME") or DEFAULT_HERMES_HOME).resolve())


def _create_session_db():
    """Best-effort SQLite session store, as ``hermes_cli.oneshot`` builds it.

    Returns None when Hermes has no usable store; turns then run without history.
    """
    try:
        from hermes_state import SessionDB

        return SessionDB()
    except Exception as exc:
        logging.warning(
            "Hermes session store unavailable; voice turns run without "
            "conversation history: %s",
            exc,
            exc_info=True,
        )
        return None


def _shared_session_db():
    """Return the session store for the current context-local Hermes home."""
    home = _current_home_key()
    with _session_db_lock:
        if home not in _session_db_attempted:
            _session_db_attempted.add(home)
            _session_dbs[home] = _create_session_db()
        return _session_dbs.get(home)


def _load_conversation_history(session_db, session_id: str) -> list:
    """Stored turns of one chat session; empty for a chat that has none yet.

    ``AIAgent`` never loads history itself, the caller owns it. The reopen clears the
    ``ended_at`` a previous turn stamped, because ``end_session`` only writes rows whose
    ``ended_at`` is null and a closed row would stop recording this chat.
    """
    if session_db is None:
        return []

    try:
        restored, _display = session_db.get_resume_conversations(session_id)
    except Exception:
        logging.debug(
            "could not load Hermes history for session %s", session_id, exc_info=True
        )
        return []

    try:
        session_db.reopen_session(session_id)
    except Exception:
        logging.debug("could not reopen session %s", session_id, exc_info=True)

    return [
        message
        for message in restored or []
        if not isinstance(message, dict) or message.get("role") != "session_meta"
    ]


def _toolset_list(toolsets: Optional[str]) -> Optional[list[str]]:
    if not toolsets:
        return None
    values = [value.strip() for value in toolsets.split(",") if value.strip()]
    return values or None


def _close_agent(agent) -> None:
    close = getattr(agent, "close", None)
    if callable(close):
        try:
            close()
        except Exception:
            logging.debug("could not close evicted Hermes agent", exc_info=True)


def clear_agent_cache() -> None:
    """Close and remove all process-local chat agents (primarily for shutdown/tests)."""
    with _agent_cache_lock:
        entries = list(_agent_cache.values())
        _agent_cache.clear()
    for entry in entries:
        _close_agent(entry.agent)

    with _session_db_lock:
        stores = list(_session_dbs.values())
        _session_dbs.clear()
        _session_db_attempted.clear()
    for store in stores:
        # The agents are gone, so nothing can write anymore: close to checkpoint the WAL.
        try:
            store.close()
        except Exception:
            logging.debug("could not close the Hermes session store", exc_info=True)


def _discover_mcp_tools() -> None:
    """Run Hermes' bounded, non-interactive MCP startup path."""
    from hermes_cli.mcp_startup import (
        ensure_mcp_discovery_before_agent_build,
        mcp_discovery_in_flight,
    )

    ensure_mcp_discovery_before_agent_build(
        logger=logging.getLogger(__name__),
        thread_name="pib-hermes-daemon-mcp",
    )
    if mcp_discovery_in_flight():
        logging.warning(
            "Hermes MCP discovery timed out before agent construction; "
            "continuing without waiting"
        )


def _ensure_mcp_tools_discovered() -> None:
    """Attempt MCP discovery once for each context-local Hermes home."""
    home = _current_home_key()
    with _mcp_discovery_lock:
        if home in _mcp_discovery_attempted:
            return
        _mcp_discovery_attempted.add(home)
        try:
            _discover_mcp_tools()
        except Exception as exc:
            logging.warning(
                "Hermes MCP discovery failed; continuing without MCP tools: %s",
                exc,
                exc_info=True,
            )


def _registered_mcp_tool_count() -> int:
    """Return the current process-wide MCP tool count, or zero if unavailable."""
    try:
        from tools.registry import registry

        return sum(
            1 for entry in registry.get_all_entries() if entry.name.startswith("mcp__")
        )
    except Exception:
        logging.debug("could not inspect registered Hermes MCP tools", exc_info=True)
        return 0


def _agent_for_chat(
    chat_id: str,
    enabled_toolsets: Optional[str],
    toolsets: Optional[str],
    max_turns: int,
    agent_cls,
    provider_key: Optional[str],
):
    """Return the sole cached agent for a chat, evicting least-recently-used chats.

    The model, provider and base URL come from the personality's config.yaml
    (written by the backend at provision time), so a turn runs the model the
    personality is set to. They are part of the cache key, so changing the
    model rebuilds the agent instead of reusing the pinned one.
    """
    from public_api_client.hermes_agent_client import (
        disabled_toolsets_for_turn,
        enabled_toolsets_for_turn,
        profile_hermes_settings,
        profile_tool_calling_enabled,
    )

    model, provider, base_url, env_vars = profile_hermes_settings()
    tool_calling = profile_tool_calling_enabled()
    if not tool_calling:
        enabled_toolsets = enabled_toolsets_for_turn(False, enabled_toolsets)
        toolsets = disabled_toolsets_for_turn(False, toolsets)
    settings = (
        enabled_toolsets,
        toolsets,
        max_turns,
        model,
        provider,
        base_url,
        tool_calling,
    )
    key = (_current_home_key(), chat_id)
    with _agent_cache_lock:
        cached = _agent_cache.get(key)
        if (
            cached is not None
            and cached.settings == settings
            and cached.key_token == (provider_key or "")
        ):
            _agent_cache.move_to_end(key)
            return cached
        if cached is not None:
            _agent_cache.pop(key)
            _close_agent(cached.agent)

        construction_started = time.monotonic()
        # The key is in the environment only for this constructor: that is
        # when Hermes copies it onto the client. api_key is the same value,
        # for a constructor that accepts it directly. A key-free route (the
        # on-device model) is built without either.
        key_scope = (
            _provider_key_in_environment(env_vars, provider_key)
            if provider_key and env_vars
            else contextlib.nullcontext()
        )
        with key_scope:
            agent = agent_cls(
                model=model,
                provider=provider,
                base_url=base_url,
                api_key=provider_key,
                session_db=_shared_session_db(),
                session_id=session_id_for_chat(chat_id),
                enabled_toolsets=_toolset_list(enabled_toolsets),
                disabled_toolsets=_toolset_list(toolsets),
                max_iterations=max_turns,
                platform="cli",
                skip_memory=True,
            )
        logging.info(
            "[PERF_TRACE] HERMES_AGENT_CONSTRUCTED chat=%s "
            "elapsed_ms=%.2f mcp_tools=%d",
            chat_id,
            (time.monotonic() - construction_started) * 1000.0,
            _registered_mcp_tool_count(),
        )
        cached = _CachedAgent(agent, settings, provider_key or "")
        _agent_cache[key] = cached

        while len(_agent_cache) > DEFAULT_AGENT_CACHE_SIZE:
            _evicted_key, evicted = _agent_cache.popitem(last=False)
            _close_agent(evicted.agent)
        return cached


def extract_final_response(stdout_text: str) -> str:
    """Pull the answer text out of ``run_agent.main`` stdout.

    Returns an empty string when the banner is absent or carries no text.
    """
    if not stdout_text:
        return ""

    marker_at = stdout_text.rfind(_FINAL_RESPONSE_MARKER)
    if marker_at < 0:
        return ""

    lines = stdout_text[marker_at + len(_FINAL_RESPONSE_MARKER) :].splitlines()
    while lines and _DECORATION_LINE.match(lines[0]):
        lines.pop(0)

    body: list[str] = []
    for line in lines:
        if any(marker in line for marker in _TRAILING_MARKERS):
            break
        body.append(line)

    while body and _DECORATION_LINE.match(body[-1]):
        body.pop()

    return "\n".join(body).strip()


def _runner_reports_tool_calls(runner: object) -> bool:
    """True when this runner accepts the ``on_tool_call`` collector.

    A runner injected by a caller (tests, embedders) may predate the parameter, so
    the keyword is only passed when the signature accepts it.
    """
    try:
        signature = inspect.signature(runner)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    if "on_tool_call" in signature.parameters:
        return True
    return any(
        parameter.kind is inspect.Parameter.VAR_KEYWORD
        for parameter in signature.parameters.values()
    )


def _tools_used(result: object) -> list:
    """Tool names of one agent turn, read from the conversation it returns.

    An assistant message lists the tool calls it asked for, a ``tool`` message
    names the tool that answered. Only names and outcomes travel - argument values
    stay in the agent's own transcript, so nothing secret can leak into a chat
    message or a trace event (PC-84).
    """
    if not isinstance(result, dict):
        return []
    messages = result.get("messages")
    if not isinstance(messages, list):
        return []
    answered = [
        {"name": str(message["name"]), "ok": True}
        for message in messages
        if isinstance(message, dict)
        and message.get("role") == "tool"
        and message.get("name")
    ]
    if answered:
        return answered
    requested: list = []
    for message in messages:
        if not isinstance(message, dict):
            continue
        for call in message.get("tool_calls") or []:
            if not isinstance(call, dict):
                continue
            function = call.get("function") or {}
            name = function.get("name") or call.get("name")
            if name:
                requested.append({"name": str(name), "ok": True})
    return requested


def _coerce_reply(reply: object) -> str:
    """Normalise the various shapes ``run_agent`` implementations return."""
    if reply is None:
        return ""
    if hasattr(reply, "content"):
        reply = reply.content
    elif isinstance(reply, dict):
        # run_conversation returns the turn result; run_agent variants use "response".
        reply = reply.get("final_response", reply.get("response", reply))
    if reply is None:
        return ""
    return (reply if isinstance(reply, str) else str(reply)).strip()


def hermes_agent_dir() -> str:
    """Directory of the bundled hermes-agent sources inside HERMES_HOME."""
    home = os.environ.get("HERMES_HOME") or DEFAULT_HERMES_HOME
    return os.path.join(home, HERMES_AGENT_DIRNAME)


def _running_python_version() -> tuple[int, int]:
    return sys.version_info.major, sys.version_info.minor


def _python_version_from_site_packages(path: str) -> Optional[tuple[int, int]]:
    found = _PYTHON_VERSION.search(path)
    if found is None:
        return None
    return int(found.group(1)), int(found.group(2))


def venv_site_packages(agent_dir: Optional[str] = None) -> list[str]:
    """Hermes venv site-packages whose pythonX.Y matches this interpreter.

    Native wheels in a 3.13 venv cannot load on 3.12 (and vice versa). Search
    stays version-agnostic among matching trees; several hits are returned in
    a deterministic path order. Empty when nothing matches — the in-process
    turn then uses the interpreter's own packages.
    """
    base = agent_dir or hermes_agent_dir()
    running = _running_python_version()
    try:
        matches = [
            path
            for path in glob.glob(os.path.join(base, _VENV_SITE_PACKAGES_GLOB))
            if os.path.isdir(path)
        ]
    except OSError as exc:
        logging.warning("could not scan %s for a hermes venv: %s", base, exc)
        return []

    compatible = [
        path for path in matches if _python_version_from_site_packages(path) == running
    ]
    compatible.sort()
    if compatible:
        logging.info(
            "prepending hermes venv site-packages matching Python %s.%s: %s",
            running[0],
            running[1],
            compatible,
        )
    else:
        logging.info(
            "no hermes venv site-packages matching Python %s.%s; inserting none",
            running[0],
            running[1],
        )
    return compatible


def daemon_host() -> str:
    return os.environ.get("PIB_HERMES_DAEMON_HOST") or DEFAULT_HOST


def daemon_port() -> int:
    raw = os.environ.get("PIB_HERMES_DAEMON_PORT")
    if raw:
        return int(raw)
    return DEFAULT_PORT


def daemon_base_url() -> str:
    """Base URL of the daemon (no trailing slash). Overridable via env."""
    return (
        os.environ.get("PIB_HERMES_DAEMON_URL")
        or f"http://{daemon_host()}:{daemon_port()}"
    ).rstrip("/")


def daemon_turn_url() -> str:
    return daemon_base_url() + "/turn"


def daemon_health_url() -> str:
    return daemon_base_url() + "/health"


def agent_source_imports_here(agent_dir: Optional[str] = None) -> bool:
    """True when the agent source imports in THIS interpreter.

    A PIB install keeps the agent's dependencies in a PM-managed environment
    (``~/.hermes/installs/<id>/environments/<id>/venv/lib/pythonX.Y``), which
    ``venv_site_packages()`` does not scan -- it only looks for ``venv/lib``
    inside the agent directory. Probing the import answers the actual question
    and covers both layouts.

    Runs in a subprocess on purpose: importing the agent here would leave its
    modules in ``sys.modules`` and could let the bootstrap re-exec this process.
    The answer is cached per process because a probe costs an interpreter start.
    """
    global _AGENT_IMPORT_PROBE
    if _AGENT_IMPORT_PROBE is None:
        base = agent_dir or hermes_agent_dir()
        # hermes_constants is stdlib-only. hermes_bootstrap is deliberately NOT
        # imported here: importing it can make the bootstrap re-exec the probe
        # interpreter, which fails in an isolated subprocess and would report the
        # agent as unavailable although it is importable.
        code = "import sys; sys.path.insert(0, %r); import hermes_constants" % (base,)
        try:
            done = subprocess.run(
                [sys.executable, "-c", code],
                capture_output=True,
                timeout=AGENT_IMPORT_PROBE_TIMEOUT,
                stdin=subprocess.DEVNULL,
            )
            _AGENT_IMPORT_PROBE = done.returncode == 0
        except (OSError, subprocess.SubprocessError) as exc:
            logging.info("agent import probe failed: %s", exc)
            _AGENT_IMPORT_PROBE = False
    return _AGENT_IMPORT_PROBE


def in_process_available() -> bool:
    """True when the Hermes agent can run inside THIS interpreter.

    The in-process API needs Hermes' packages importable for the running Python.
    Two layouts provide them: a ``venv/lib/pythonX.Y`` inside the agent directory
    (a matching one is prepended to ``sys.path``), or a PM-managed environment
    that the interpreter already carries. When neither holds, importing the agent
    source would make Hermes' bootstrap re-exec this process under its managed
    Python -- fatal here, because this process is the ROS ``chat`` node and its
    setuptools launcher cannot resolve the ``voice-assistant`` distribution
    there. Callers fall back to the CLI subprocess in that case.
    """
    return bool(venv_site_packages()) or agent_source_imports_here()


def _run_turn_in_home(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    max_turns: Optional[int] = None,
    timeout: Optional[int] = None,
    stream_callback: Optional[Callable[[str], None]] = None,
    enabled_toolsets: Optional[str] = None,
    provider_key: str = "",
    on_tool_call: Optional[Callable[[dict], None]] = None,
) -> str:
    """Execute one turn after its Hermes home scope has been installed.

    The turn replays this chat's stored conversation, so what was said earlier is in
    context instead of the agent searching for it with its whole iteration budget.
    """
    from public_api_client.hermes_agent_client import (
        DEFAULT_DISABLED_TOOLSETS,
        DEFAULT_ENABLED_TOOLSETS,
        DEFAULT_MAX_TURNS,
        FALLBACK_REPLY,
        disabled_toolsets_for_turn,
        enabled_toolsets_for_turn,
        profile_tool_calling_enabled,
        run_turn_subprocess,
    )

    tool_calling = profile_tool_calling_enabled()
    effective_toolsets = disabled_toolsets_for_turn(tool_calling, toolsets)
    effective_enabled_toolsets = enabled_toolsets_for_turn(
        tool_calling, enabled_toolsets
    )
    effective_max_turns = DEFAULT_MAX_TURNS if max_turns is None else max_turns

    def _subprocess_reply() -> str:
        kwargs = {
            "text": text,
            "chat_id": chat_id,
            "personality_id": personality_id,
            "toolsets": effective_toolsets,
            "enabled_toolsets": effective_enabled_toolsets,
        }
        if timeout is not None:
            kwargs["timeout"] = timeout
        if provider_key:
            kwargs["provider_key"] = provider_key
        return run_turn_subprocess(**kwargs)

    # The in-process API runs the agent inside THIS interpreter, which only
    # works when Hermes' own venv has site-packages for it. With none matching,
    # importing the agent source lets Hermes' bootstrap re-exec this process
    # under its managed Python. This process is the ROS `chat` node, whose
    # setuptools launcher cannot resolve the `voice-assistant` distribution under
    # that interpreter, so the node exits and every chat is lost until the
    # container restarts. Run the supported CLI subprocess instead.
    if not in_process_available():
        logging.info(
            "no Hermes venv site-packages match this interpreter; running the "
            "turn as a CLI subprocess instead of in-process (chat=%s)",
            chat_id,
        )
        return _subprocess_reply()

    ensure_hermes_source_on_path()

    agent_module = None
    for module_name in ("run_agent", "hermes.run_agent"):
        try:
            agent_module = importlib.import_module(module_name)
            break
        except ImportError as exc:
            logging.info(
                "run_agent import %s failed (%s); trying the next candidate",
                module_name,
                exc,
            )
            continue
    agent_cls = getattr(agent_module, "AIAgent", None)
    run_agent_main = getattr(agent_module, "main", None)

    if agent_cls is None and run_agent_main is None:
        logging.info(
            "Hermes Python API unavailable; falling back to CLI subprocess (chat=%s)",
            chat_id,
        )
        return _subprocess_reply()

    try:
        if agent_cls is not None:
            if tool_calling and "pib" in (
                _toolset_list(effective_enabled_toolsets) or []
            ):
                _ensure_mcp_tools_discovered()
            cached = _agent_for_chat(
                chat_id,
                effective_enabled_toolsets,
                effective_toolsets,
                effective_max_turns,
                agent_cls,
                provider_key,
            )
            produced: list[str] = []

            def capture_delta(delta: str) -> None:
                if isinstance(delta, str) and delta:
                    produced.append(delta)
                    if stream_callback is not None:
                        stream_callback(delta)

            with cached.lock:
                session_id = session_id_for_chat(chat_id)
                history = _load_conversation_history(_shared_session_db(), session_id)
                logging.info(
                    "[PERF_TRACE] HERMES_TURN_HISTORY chat=%s messages=%d",
                    chat_id,
                    len(history),
                )
                from public_api_client.hermes_agent_client import (
                    profile_hermes_settings,
                )

                route_model, route_provider, route_base_url, _route_env = (
                    profile_hermes_settings()
                )
                _sync_provider_capabilities_to_agent(
                    cached.agent,
                    route_provider,
                    route_base_url,
                    route_model,
                )
                attempted = planned_provider_request_fields(
                    cached.agent,
                    route_provider,
                    route_base_url,
                    route_model,
                )
                _provider_field_attempts.append(attempted)
                logging.info(
                    "PIB provider request fields planned chat=%s fields=%s",
                    chat_id,
                    sorted(attempted),
                )
                turn_result = cached.agent.run_conversation(
                    user_message=text,
                    conversation_history=history or None,
                    stream_callback=capture_delta,
                )
                _harvest_provider_refusals_from_agent(
                    cached.agent,
                    route_provider,
                    route_base_url,
                    route_model,
                )
                reply = _coerce_reply(turn_result)
                if on_tool_call is not None:
                    for record in _tools_used(turn_result):
                        on_tool_call(record)
            if not reply:
                reply = "".join(produced).strip()
        else:
            captured = io.StringIO()
            from public_api_client.hermes_agent_client import (
                profile_hermes_settings,
            )

            _model, _provider, _base_url, key_env_vars = profile_hermes_settings()
            key_scope = (
                _provider_key_in_environment(key_env_vars, provider_key)
                if provider_key and key_env_vars
                else contextlib.nullcontext()
            )
            with _stdout_capture_lock, contextlib.redirect_stdout(captured), key_scope:
                returned = run_agent_main(
                    query=text,
                    model=_model,
                    enabled_toolsets=effective_enabled_toolsets,
                    disabled_toolsets=effective_toolsets,
                    max_turns=effective_max_turns,
                )
            reply = extract_final_response(captured.getvalue())
            if not reply:
                reply = _coerce_reply(returned)
            if not reply:
                logging.warning(
                    "no FINAL RESPONSE in run_agent stdout (chat=%s, %d chars captured)",
                    chat_id,
                    len(captured.getvalue()),
                )
    except Exception as exc:
        logging.exception(
            "in-process hermes turn failed (chat=%s): %s",
            chat_id,
            exc,
        )
        return FALLBACK_REPLY

    if reply:
        return reply

    try:
        return _subprocess_reply() or FALLBACK_REPLY
    except Exception as exc:
        logging.exception(
            "subprocess fallback after empty in-process reply failed (chat=%s): %s",
            chat_id,
            exc,
        )
        return FALLBACK_REPLY


def run_turn_in_process(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    max_turns: Optional[int] = None,
    timeout: Optional[int] = None,
    stream_callback: Optional[Callable[[str], None]] = None,
    enabled_toolsets: Optional[str] = None,
    on_tool_call: Optional[Callable[[dict], None]] = None,
) -> str:
    """Execute a turn inside the personality's context-local Hermes home."""
    scope = contextlib.nullcontext()
    profile_dir = None
    if personality_id:
        from pib_hermes_config import profile_dir_for

        profile_dir = profile_dir_for(personality_id)
        scope = hermes_home_scope(profile_dir)

    with scope:
        if personality_id:
            from public_api_client.hermes_agent_client import ensure_profile

            profile_dir = ensure_profile(personality_id)
            logging.info(
                "HERMES_TURN_HOME personality=%s home=%s memory_dir=%s session_db=%s",
                personality_id,
                profile_dir,
                os.path.join(profile_dir, "memories"),
                os.path.join(profile_dir, "state.db"),
            )

        # Resolve the key inside the profile scope: the profile's config.yaml
        # names the model and provider the backend wrote for this personality.
        from public_api_client.hermes_agent_client import provider_key_for_turn

        try:
            provider_key = provider_key_for_turn()
        except Exception as exc:
            if not _missing_provider_key(exc):
                raise
            # Drop agents built with a key the store will no longer give out.
            clear_agent_cache()
            raise

        return _run_turn_in_home(
            text=text,
            chat_id=chat_id,
            personality_id=personality_id,
            toolsets=toolsets,
            max_turns=max_turns,
            timeout=timeout,
            stream_callback=stream_callback,
            enabled_toolsets=enabled_toolsets,
            provider_key=provider_key or "",
            on_tool_call=on_tool_call,
        )


def _default_turn_runner(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    max_turns: Optional[int] = None,
    timeout: Optional[int] = None,
    stream_callback: Optional[Callable[[str], None]] = None,
    enabled_toolsets: Optional[str] = None,
    on_tool_call: Optional[Callable[[dict], None]] = None,
) -> str:
    """Execute one turn in-process via Hermes Agent (subprocess fallback)."""
    return run_turn_in_process(
        text=text,
        chat_id=chat_id,
        personality_id=personality_id,
        toolsets=toolsets,
        enabled_toolsets=enabled_toolsets,
        on_tool_call=on_tool_call,
        max_turns=max_turns,
        timeout=timeout,
        stream_callback=stream_callback,
    )


class HermesDaemonHandler(BaseHTTPRequestHandler):
    """Minimal request handler for health, profile provisioning, and turns."""

    # Injected on the server instance before serve_forever.
    turn_runner: TurnRunner = staticmethod(_default_turn_runner)  # type: ignore[assignment]

    def log_message(self, fmt: str, *args) -> None:
        logging.debug("hermes-daemon: " + fmt, *args)

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _start_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
        self.end_headers()

    def _send_stream_chunk(self, payload: dict) -> None:
        self.wfile.write((json.dumps(payload) + "\n").encode("utf-8"))
        self.wfile.flush()

    def do_GET(self) -> None:  # noqa: N802 — http.server API
        parsed = urlparse(self.path)
        if parsed.path.rstrip("/") == "/health":
            self._send_json(200, {"status": "ok"})
            return
        if parsed.path.rstrip("/") == "/profile":
            personality_ids = parse_qs(parsed.query).get("personality_id", [])
            if not personality_ids or not personality_ids[0]:
                self._send_json(
                    400, {"ok": False, "error": "personality_id is required"}
                )
                return
            self._send_json(200, {"ok": True, **_profile_layout(personality_ids[0])})
            return
        self._send_json(404, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802 — http.server API
        path = urlparse(self.path).path.rstrip("/")
        if path not in {"/turn", "/profile"}:
            self._send_json(404, {"error": "not found"})
            return

        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            data = json.loads(raw.decode("utf-8") or "{}")
        except (UnicodeDecodeError, json.JSONDecodeError):
            payload = {"error": "invalid json"}
            if path == "/profile":
                payload["ok"] = False
            self._send_json(400, payload)
            return

        if not isinstance(data, dict):
            payload = {"error": "body must be a json object"}
            if path == "/profile":
                payload["ok"] = False
            self._send_json(400, payload)
            return

        if path == "/profile":
            personality_id = data.get("personality_id")
            personality_name = data.get("personality_name")
            soul_text = data.get("soul_text")
            model = data.get("model")
            provider = data.get("provider")
            endpoint_base = data.get("endpoint_base")
            if not isinstance(personality_id, str) or not personality_id:
                self._send_json(
                    400,
                    {"ok": False, "error": "personality_id is required"},
                )
                return
            if personality_name is not None and not isinstance(personality_name, str):
                self._send_json(
                    400,
                    {"ok": False, "error": "personality_name must be a string"},
                )
                return
            if soul_text is not None and not isinstance(soul_text, str):
                self._send_json(
                    400, {"ok": False, "error": "soul_text must be a string"}
                )
                return
            for value, field in (
                (model, "model"),
                (provider, "provider"),
                (endpoint_base, "endpoint_base"),
            ):
                if value is not None and not isinstance(value, str):
                    self._send_json(
                        400, {"ok": False, "error": f"{field} must be a string"}
                    )
                    return
            personality_reasoning = "reasoning_effort" in data
            reasoning_effort = (
                data.get("reasoning_effort") if personality_reasoning else None
            )
            if personality_reasoning and reasoning_effort is not None:
                from pib_hermes_config import REASONING_EFFORT_ERROR, REASONING_EFFORTS

                if (
                    not isinstance(reasoning_effort, str)
                    or reasoning_effort not in REASONING_EFFORTS
                ):
                    self._send_json(400, {"ok": False, "error": REASONING_EFFORT_ERROR})
                    return
            tool_calling = data.get("tool_calling")
            if tool_calling is not None and not isinstance(tool_calling, bool):
                self._send_json(
                    400, {"ok": False, "error": "tool_calling must be a boolean"}
                )
                return
            try:
                result = ensure_profile_home(
                    personality_id,
                    personality_name=personality_name,
                    soul_text=soul_text,
                    model=model,
                    provider=provider,
                    base_url=endpoint_base,
                    reasoning_effort=reasoning_effort,
                    personality_reasoning=personality_reasoning,
                    tool_calling=tool_calling,
                )
            except Exception as exc:
                logging.exception("hermes-daemon /profile failed: %s", exc)
                self._send_json(500, {"ok": False, "error": str(exc)})
                return
            self._send_json(200, result)
            return

        t0 = time.monotonic()
        logging.info("[PERF_TRACE] DAEMON_RECV elapsed_ms=0.00")

        text = data.get("text")
        chat_id = data.get("chat_id")
        if not isinstance(text, str) or not isinstance(chat_id, str):
            self._send_json(400, {"error": "text and chat_id are required strings"})
            return

        personality_id = data.get("personality_id")
        toolsets = data.get("toolsets")
        enabled_toolsets = data.get("enabled_toolsets")
        max_turns = data.get("max_turns")
        timeout = data.get("timeout")
        stream = data.get("stream", False)
        if personality_id is not None and not isinstance(personality_id, str):
            self._send_json(400, {"error": "personality_id must be a string"})
            return
        if toolsets is not None and not isinstance(toolsets, str):
            self._send_json(400, {"error": "toolsets must be a string"})
            return
        if enabled_toolsets is not None and not isinstance(enabled_toolsets, str):
            self._send_json(400, {"error": "enabled_toolsets must be a string"})
            return
        if max_turns is not None and (
            not isinstance(max_turns, int)
            or isinstance(max_turns, bool)
            or max_turns < 1
        ):
            self._send_json(400, {"error": "max_turns must be a positive integer"})
            return
        if timeout is not None and not isinstance(timeout, (int, float)):
            self._send_json(400, {"error": "timeout must be a number"})
            return
        if not isinstance(stream, bool):
            self._send_json(400, {"error": "stream must be a boolean"})
            return

        runner = getattr(self.server, "turn_runner", None) or _default_turn_runner
        turn_start = time.monotonic()
        logging.info(
            "[PERF_TRACE] DAEMON_TURN_START chat=%s elapsed_ms=%.2f",
            chat_id,
            (turn_start - t0) * 1000.0,
        )
        first_delta = False

        def emit_delta(delta: str) -> None:
            nonlocal first_delta
            if not isinstance(delta, str) or not delta:
                return
            if not first_delta:
                first_delta = True
                logging.info(
                    "[PERF_TRACE] DAEMON_FIRST_TOKEN chat=%s elapsed_ms=%.2f",
                    chat_id,
                    (time.monotonic() - t0) * 1000.0,
                )
            self._send_stream_chunk({"delta": delta})

        if stream:
            self._start_stream()

        tools_used: list = []
        runner_kwargs = {
            "text": text,
            "chat_id": chat_id,
            "personality_id": personality_id,
            "toolsets": toolsets,
            "enabled_toolsets": enabled_toolsets,
            "timeout": int(timeout) if timeout is not None else None,
        }
        if max_turns is not None:
            runner_kwargs["max_turns"] = max_turns
        if _runner_reports_tool_calls(runner):
            runner_kwargs["on_tool_call"] = tools_used.append
        if stream:
            runner_kwargs["stream_callback"] = emit_delta

        try:
            reply = runner(**runner_kwargs)
        except Exception as exc:
            logging.exception("hermes-daemon /turn failed: %s", exc)
            if stream:
                self._send_stream_chunk({"error": str(exc)})
            else:
                self._send_json(500, {"error": str(exc)})
            return

        if not first_delta:
            logging.info(
                "[PERF_TRACE] DAEMON_FIRST_TOKEN chat=%s elapsed_ms=%.2f",
                chat_id,
                (time.monotonic() - t0) * 1000.0,
            )
        logging.info(
            "[PERF_TRACE] DAEMON_DONE chat=%s elapsed_ms=%.2f",
            chat_id,
            (time.monotonic() - t0) * 1000.0,
        )
        payload = {
            "reply": reply if isinstance(reply, str) else str(reply),
            # The daemon may return more than the text (decision PC-84): the tools
            # this turn used, names only.
            "toolsUsed": tools_used,
        }
        if stream:
            self._send_stream_chunk(payload)
        else:
            self._send_json(200, payload)


def create_server(
    host: Optional[str] = None,
    port: Optional[int] = None,
    turn_runner: Optional[TurnRunner] = None,
) -> ThreadingHTTPServer:
    """Build a ThreadingHTTPServer bound to host:port."""
    server = ThreadingHTTPServer(
        (host or daemon_host(), port if port is not None else daemon_port()),
        HermesDaemonHandler,
    )
    server.turn_runner = turn_runner or _default_turn_runner  # type: ignore[attr-defined]
    return server


def serve_forever(
    host: Optional[str] = None,
    port: Optional[int] = None,
    turn_runner: Optional[TurnRunner] = None,
) -> None:
    """Block serving HTTP until interrupted."""
    server = create_server(host=host, port=port, turn_runner=turn_runner)
    logging.info(
        "hermes daemon listening on http://%s:%s",
        server.server_address[0],
        server.server_address[1],
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()
        clear_agent_cache()


def is_daemon_reachable(timeout: float = 0.5) -> bool:
    """True when GET /health returns HTTP 200."""
    try:
        req = Request(daemon_health_url(), method="GET")
        with urlopen(req, timeout=timeout) as resp:
            return getattr(resp, "status", 200) == 200
    except (URLError, OSError, TimeoutError, ValueError):
        return False


def start_daemon(
    host: Optional[str] = None,
    port: Optional[int] = None,
    turn_runner: Optional[TurnRunner] = None,
) -> ThreadingHTTPServer:
    """Start the daemon HTTP server in a background daemon thread.

    Idempotent within this process: a second call returns the existing server
    when it is still running. Raises OSError if the port is already taken by
    another process.
    """
    global _server, _server_thread

    with _server_lock:
        if (
            _server is not None
            and _server_thread is not None
            and _server_thread.is_alive()
        ):
            return _server

        server = create_server(host=host, port=port, turn_runner=turn_runner)
        thread = threading.Thread(
            target=server.serve_forever,
            name="hermes-daemon",
            daemon=True,
        )
        thread.start()
        _server = server
        _server_thread = thread
        logging.info(
            "hermes daemon started on http://%s:%s",
            server.server_address[0],
            server.server_address[1],
        )
        return server


def stop_daemon() -> None:
    """Shut down the in-process daemon started by start_daemon (best-effort)."""
    global _server, _server_thread

    with _server_lock:
        server = _server
        thread = _server_thread
        _server = None
        _server_thread = None

    if server is None:
        return
    try:
        server.shutdown()
    except Exception as exc:
        logging.debug("hermes daemon shutdown: %s", exc)
    try:
        server.server_close()
    except Exception:
        pass
    if thread is not None and thread.is_alive():
        thread.join(timeout=2.0)
    clear_agent_cache()


def ensure_daemon_running(
    host: Optional[str] = None,
    port: Optional[int] = None,
    wait_seconds: float = _STARTUP_WAIT_SECONDS,
) -> bool:
    """Make sure a daemon is reachable; start one in-process if needed.

    Returns True when /health answers within ``wait_seconds``.
    """
    if is_daemon_reachable():
        return True

    try:
        start_daemon(host=host, port=port)
    except OSError as exc:
        # Another process may already own the port — re-check health.
        logging.warning("hermes daemon bind failed (%s); rechecking health", exc)
        return is_daemon_reachable()

    deadline = time.monotonic() + wait_seconds
    while time.monotonic() < deadline:
        if is_daemon_reachable():
            return True
        time.sleep(_STARTUP_POLL_SECONDS)
    return is_daemon_reachable()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s hermes-daemon: %(message)s",
    )
    serve_forever()
