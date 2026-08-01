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
            if os.path.basename(path) == ".env":
                os.chmod(path, 0o600)
            elif os.path.isfile(path):
                os.chmod(path, 0o664)
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
    ("get_state", "Liefert den aktuellen Gelenkzustand, Diagnosen und Roboter-Telemetrie."),
    ("list_poses", "Listet gespeicherte Posen."),
    ("list_programs", "Listet gespeicherte Blockly-/Python-Programme."),
    ("capture_image", "Nimmt ein Kamerabild als base64-kodiertes JPEG auf."),
    ("move_motor", "Bewegt einen Motor innerhalb seiner konfigurierten Rotationsgrenzen."),
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
    gemini-3.6-flash pick the one Hermes never registered, and every such turn
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

