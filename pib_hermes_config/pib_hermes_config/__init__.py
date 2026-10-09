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
from typing import NamedTuple

DEFAULT_PROFILES_DIR = "/home/pib/.hermes/profiles"
PROFILES_DIR_ENV = "PIB_HERMES_PROFILES_DIR"
PROFILE_PREFIX = "pib_"
SOUL_FILENAME = "SOUL.md"
DEFAULT_SOUL = "Du bist pib, ein humanoider Roboter."
PROFILE_DIR_MODE = 0o700
SOUL_FILE_MODE = 0o644
ENV_FILE_MODE = 0o600

# Permanent Hermes LLM pin. Kept in sync with hermes_agent_client and setup-pib.sh.
DEFAULT_HERMES_MODEL = "gemini-3.8-flash"
DEFAULT_HERMES_LITE_MODEL = "gemini-3.5-flash-lite"
DEFAULT_HERMES_PROVIDER = "gemini"
# Seeded into agent.reasoning_effort only when that key is absent and no
# personality value is being applied. Hermes reads the agent block; a
# root-level reasoning_effort is ignored.
DEFAULT_REASONING_EFFORT = "low"
DEFAULT_MAX_TOKENS = 1024
DEFAULT_TEMPERATURE = 0.3
# Closed set Hermes accepts for agent.reasoning_effort.
REASONING_EFFORTS = (
    "none",
    "minimal",
    "low",
    "medium",
    "high",
    "xhigh",
    "max",
    "ultra",
)
REASONING_EFFORT_ERROR = (
    "Reasoning effort must be one of: " + ", ".join(REASONING_EFFORTS) + "."
)
# A new personality starts here. NULL on a stored row means unmanaged:
# the profile's existing agent.reasoning_effort is left as it is.
DEFAULT_PERSONALITY_REASONING_EFFORT = "none"

# --- pib provider name -> Hermes provider mapping -------------------------
#
# Source of every value below:
#   * provider names and model ids: the Flask registry/catalogue
#     (``pib_api/flask/provider_registry.py``) and ``local_model.PROVIDER_NAME``.
#   * Hermes provider identifiers and the env var each built-in provider reads:
#     the provider plugins shipped with Hermes
#     (https://hermes-agent.nousresearch.com/docs/integrations/providers).
#   * a user-defined route (pib.Cloud, Local): Hermes declares it under
#     ``providers:`` in config.yaml with a ``base_url`` and an optional
#     ``key_env`` (``hermes_cli/config_providers.py``). The base URL comes from
#     the registry's ``Provider.endpoint_base``.
PIB_PROVIDER_GOOGLE = "Google"
PIB_PROVIDER_OPENAI = "OpenAI"
PIB_PROVIDER_ANTHROPIC = "Anthropic"
PIB_PROVIDER_PIB_CLOUD = "pib.Cloud"
PIB_PROVIDER_LOCAL = "Local"

HERMES_PROVIDER_GEMINI = "gemini"
HERMES_PROVIDER_OPENAI = "openai"
HERMES_PROVIDER_ANTHROPIC = "anthropic"
#: Slug of a user-defined ``providers:`` entry for pib.Cloud / Local.
HERMES_PROVIDER_PIB_CLOUD = "pib-cloud"
HERMES_PROVIDER_LOCAL = "local"

GEMINI_KEY_ENV_VARS = ("GOOGLE_API_KEY", "GEMINI_API_KEY")
OPENAI_KEY_ENV_VARS = ("OPENAI_API_KEY",)
ANTHROPIC_KEY_ENV_VARS = ("ANTHROPIC_API_KEY",)
#: Env var Hermes reads for the custom pib.Cloud route. Written as the entry's
#: ``key_env`` so a pib.Cloud turn can be keyed from the store.
PIB_CLOUD_KEY_ENV_VARS = ("PIB_CLOUD_API_KEY",)

#: Built-in providers Hermes already understands; no ``providers:`` entry needed.
BUILTIN_KEY_ENV_VARS: dict[str, tuple[str, ...]] = {
    HERMES_PROVIDER_GEMINI: GEMINI_KEY_ENV_VARS,
    HERMES_PROVIDER_OPENAI: OPENAI_KEY_ENV_VARS,
    HERMES_PROVIDER_ANTHROPIC: ANTHROPIC_KEY_ENV_VARS,
}


class ProviderMapping(NamedTuple):
    """Hermes settings for one pib provider.

    ``provider`` is the Hermes provider identifier (a built-in name or the key
    of a ``providers:`` entry). ``env_vars`` are the environment variables
    Hermes reads for that provider; empty means the route needs no key.
    ``base_url`` is the custom-route base URL, or None for a built-in provider.
    """

    provider: str
    env_vars: tuple[str, ...]
    base_url: str | None


def _clean_base_url(endpoint_base: object) -> str | None:
    if isinstance(endpoint_base, str) and endpoint_base.strip():
        return endpoint_base.strip().rstrip("/")
    return None


def _provider_slug(name: str) -> str:
    return _UNSAFE.sub("-", name.strip().lower()).strip("-")


def provider_for_profile(
    provider_name: str | None, endpoint_base: object = None
) -> ProviderMapping:
    """Map a pib provider name to its Hermes provider settings.

    ``provider_name`` is the registry ``Provider.name`` (Google, OpenAI,
    Anthropic, pib.Cloud, Local). The Hermes identifier, the key env var names
    and the custom base URL follow the fixed table documented above. An
    unknown provider with an ``endpoint_base`` becomes its own user-defined
    route; an unknown provider without one falls back to the pinned Gemini
    provider.
    """
    name = (provider_name or "").strip()
    base = _clean_base_url(endpoint_base)

    if name in (PIB_PROVIDER_GOOGLE, HERMES_PROVIDER_GEMINI):
        return ProviderMapping(HERMES_PROVIDER_GEMINI, GEMINI_KEY_ENV_VARS, None)
    if name in (PIB_PROVIDER_OPENAI, HERMES_PROVIDER_OPENAI):
        return ProviderMapping(HERMES_PROVIDER_OPENAI, OPENAI_KEY_ENV_VARS, None)
    if name in (PIB_PROVIDER_ANTHROPIC, HERMES_PROVIDER_ANTHROPIC):
        return ProviderMapping(HERMES_PROVIDER_ANTHROPIC, ANTHROPIC_KEY_ENV_VARS, None)
    if name == PIB_PROVIDER_PIB_CLOUD:
        return ProviderMapping(HERMES_PROVIDER_PIB_CLOUD, PIB_CLOUD_KEY_ENV_VARS, base)
    if name == PIB_PROVIDER_LOCAL:
        if base is None:
            # No registry endpoint: the on-device OpenAI-compatible root.
            from pib_hermes_config.local_model import openai_base_url

            base = _clean_base_url(openai_base_url())
        return ProviderMapping(HERMES_PROVIDER_LOCAL, (), base)

    slug = _provider_slug(name)
    if slug and base:
        return ProviderMapping(slug, (), base)
    return ProviderMapping(DEFAULT_HERMES_PROVIDER, GEMINI_KEY_ENV_VARS, None)


def env_vars_for_provider(
    hermes_provider: str | None, key_env: object = None
) -> tuple[str, ...]:
    """Env var names to place a provider key under, given a config entry.

    Built-in providers have fixed names; a user-defined route states its own
    via ``key_env``. Empty means the provider needs no key.
    """
    names = list(BUILTIN_KEY_ENV_VARS.get(hermes_provider or "", ()))
    if isinstance(key_env, str) and key_env.strip() and key_env.strip() not in names:
        names.append(key_env.strip())
    return tuple(names)


def provider_needs_key(hermes_provider: str | None, key_env: object = None) -> bool:
    """True when Hermes must receive a key from the store for this provider."""
    return bool(env_vars_for_provider(hermes_provider, key_env))


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
    can_chown = True
    for path in paths:
        if can_chown:
            try:
                os.chown(path, intended.st_uid, intended.st_gid)
            except OSError as exc:
                can_chown = False
                logging.debug(
                    "could not chown %s to %s:%s: %s",
                    path,
                    intended.st_uid,
                    intended.st_gid,
                    exc,
                )
        try:
            if os.path.isdir(path):
                os.chmod(path, PROFILE_DIR_MODE)
            elif os.path.basename(path) == ".env":
                os.chmod(path, ENV_FILE_MODE)
            elif os.path.basename(path) == SOUL_FILENAME:
                os.chmod(path, SOUL_FILE_MODE)
        except OSError as exc:
            logging.debug("could not set permissions on %s: %s", path, exc)

    if can_chown:
        logging.debug(
            "hermes profile %s now owned by %s:%s",
            profile_dir,
            intended.st_uid,
            intended.st_gid,
        )

    try:
        os.chmod(profile_dir, PROFILE_DIR_MODE)
    except OSError as exc:
        logging.debug(
            "could not chmod %s to %o: %s", profile_dir, PROFILE_DIR_MODE, exc
        )


# The one callable name per tool, as Hermes builds it in
# tools/mcp_tool.py::mcp_prefixed_tool_name: "mcp__" + server + "__" + tool. The
# double underscores are part of the name; no shorter or single-underscore spelling
# is registered.
MCP_TOOL_NAME_PREFIX = "mcp__pib__"

# (tool, German one-line description) for every tool pib_mcp_server exports.
MCP_TOOLS = (
    (
        "list_motors",
        "Listet konfigurierte Motoren und Bricklets inklusive aktueller Motorpositionen.",
    ),
    (
        "get_state",
        "Liefert den aktuellen Gelenkzustand, Diagnosen und Roboter-Telemetrie.",
    ),
    ("list_poses", "Listet gespeicherte Posen."),
    ("list_programs", "Listet gespeicherte Blockly-/Python-Programme."),
    ("capture_image", "Nimmt ein Kamerabild als base64-kodiertes JPEG auf."),
    (
        "move_motor",
        "Bewegt einen Motor innerhalb seiner konfigurierten Rotationsgrenzen.",
    ),
    ("apply_pose", "Wendet eine gespeicherte Pose anhand ihres genauen Namens an."),
    ("run_program", "Startet ein gespeichertes Programm anhand seiner Program-ID."),
    ("set_led", "Setzt die RGB-LED eines Buttons (Button 1–3, Kanäle 0–255)."),
    ("set_relay", "Schaltet das Solid-State-Relais ein oder aus."),
    (
        "soul_append",
        "Hängt eine dauerhafte Lektion an die SOUL.md einer Persönlichkeit an; "
        "ersetzt sie nie.",
    ),
)


def mcp_tool_name(tool: str) -> str:
    """The exact name a model must emit to call one pib FastMCP tool."""
    return MCP_TOOL_NAME_PREFIX + tool


MCP_TOOL_NAMES = tuple(mcp_tool_name(tool) for tool, _ in MCP_TOOLS)


def _build_mcp_tools_soul_section() -> str:
    """Render the SOUL section that teaches the agent its real tool names.

    Generated rather than written out, because the failure this prevents is a
    second spelling creeping into the prose. A SOUL that offered both
    ``mcp__pib__list_poses`` and an ``mcp_pib_list_poses`` "alias" made
    gemini-3.5-flash pick the one Hermes never registered, and every such turn
    died as "Model generated invalid tool call" after three retries. Exactly one
    name per tool can be documented here by construction.
    """
    lines = [
        "## Verfügbare MCP-Werkzeuge (pib_mcp_server)",
        "",
        "Nutze diese Werkzeuge, um den Roboter wahrzunehmen und zu steuern.",
        "Jede Überschrift ist der vollständige, exakte Funktionsname: rufe ihn",
        "genau so auf, inklusive der doppelten Unterstriche. Kurzformen wie",
        "`list_poses` oder `pib_list_poses` existieren nicht.",
        "",
    ]
    for tool, description in MCP_TOOLS:
        lines += [f"### {mcp_tool_name(tool)}", description, ""]
    return "\n".join(lines).rstrip() + "\n"


# Seeded into every personality SOUL.md so the agent knows its real FastMCP tools.
MCP_TOOLS_SOUL_SECTION = _build_mcp_tools_soul_section()


def build_default_soul_text(
    personality_name: str,
    custom_description: str | None = None,
) -> str:
    """Build the standard SOUL.md with robot identity and MCP tools documentation."""
    name = (personality_name or "").strip() or "pib"
    parts = [f"Du bist der humanoide Roboter {name}."]
    if custom_description and custom_description.strip():
        parts.append("")
        parts.append(custom_description.strip())
    parts.append("")
    parts.append(MCP_TOOLS_SOUL_SECTION.strip())
    return "\n".join(parts) + "\n"
