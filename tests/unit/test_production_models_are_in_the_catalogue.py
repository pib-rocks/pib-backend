"""Production code may name only models the catalogue contains.

A migration records the schema it applied, so the two historical revisions
that seeded or renamed the old rows are not production names. Everything
else that ships is.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

from provider_registry import CATALOGUE, active_api_names

REPO_ROOT = Path(__file__).resolve().parents[2]

# A migration records what the schema was. These two seeded or renamed the
# old rows, and they are never edited.
HISTORICAL_MIGRATIONS = (
    "ba9e8ed1e18c_add_gemini_assistant_model.py",
    "259579f4fe12_change_assistant_model.py",
)

# Local Hermes labels name the model served on the device, not a Google or
# OpenAI catalogue id. They need a measurement before they can change, so
# this pass leaves them. pib_hermes_config carries the same pin as
# hermes_agent_client and setup-pib.sh, and is exempt for that reason.
LOCAL_DAEMON_LABELS = {
    "public_api_client/public_api_client/hermes_daemon.py": frozenset(
        {"gemini-3.8-flash"}
    ),
    "public_api_client/public_api_client/hermes_agent_client.py": frozenset(
        {"gemini-3.8-flash", "gemini-3.5-flash-lite"}
    ),
    "pib_hermes_config/pib_hermes_config/__init__.py": frozenset(
        {"gemini-3.8-flash", "gemini-3.5-flash-lite"}
    ),
    "setup/setup-pib.sh": frozenset({"gemini-3.8-flash"}),
}

# Live-session transport ids. They are not chat rows in the catalogue. The
# retired preview is named so a session can refuse it.
LIVE_TRANSPORT_IDS = {
    "pib_hermes_config/pib_hermes_config/live_session.py": frozenset(
        {
            "gemini-3.8-live",
            "gemini-2.5-flash-native-audio-preview-09-2025",
        }
    ),
}

_ROOTS = (
    "pib_api",
    "public_api_client",
    "pib_hermes_config",
    "pib_mcp_server",
    "ros_packages",
    "setup",
)
_SUFFIXES = {".py", ".sh", ".yaml", ".yml"}
_MODEL_ID = re.compile(
    r"(?<![A-Za-z0-9])"
    r"(?:(?:gpt|gemini|claude|anthropic)[-.][A-Za-z0-9._-]+|pib-cloud)"
    r"(?![A-Za-z0-9._-])"
)

# Rows the purge removes. The migration deletes by catalogue membership, so
# each of these is covered without being listed in the SQL.
_OLD_CHAT_IDS = (
    "gpt-4o",
    "gpt-4-turbo",
    "gpt-3.5-turbo",
    "gemini-3.5-flash",
    "gemini-3.5-flash-lite",
    "anthropic.claude-3-sonnet-20240229-v1:0",
)


def _catalogue_ids() -> frozenset[str]:
    return frozenset(entry.api_name for entry in CATALOGUE if entry.api_name)


def _exempt(relative: str, model_id: str) -> bool:
    allowed = LOCAL_DAEMON_LABELS.get(relative, frozenset())
    allowed = allowed | LIVE_TRANSPORT_IDS.get(relative, frozenset())
    return model_id in allowed


def _docstring_nodes(tree: ast.AST) -> set[int]:
    nodes: set[int] = set()
    for node in ast.walk(tree):
        if not isinstance(
            node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)
        ):
            continue
        body = node.body
        if not body or not isinstance(body[0], ast.Expr):
            continue
        value = body[0].value
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
            nodes.add(id(value))
    return nodes


def _python_strings(source: str) -> list[tuple[int, str]]:
    tree = ast.parse(source)
    docstrings = _docstring_nodes(tree)
    found: list[tuple[int, str]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
            continue
        if id(node) in docstrings:
            continue
        found.append((node.lineno, node.value))
    return found


def _text_lines(source: str) -> list[tuple[int, str]]:
    lines = []
    for lineno, line in enumerate(source.splitlines(), start=1):
        code = line.split("#", 1)[0]
        if code.strip():
            lines.append((lineno, code))
    return lines


def _offenders() -> list[str]:
    catalogue = _catalogue_ids()
    found: list[str] = []
    for root_name in _ROOTS:
        root = REPO_ROOT / root_name
        for path in root.rglob("*"):
            if not path.is_file() or path.suffix not in _SUFFIXES:
                continue
            if any(part in {"__pycache__", "node_modules"} for part in path.parts):
                continue
            if path.name in HISTORICAL_MIGRATIONS:
                continue
            relative = path.relative_to(REPO_ROOT).as_posix()
            source = path.read_text(encoding="utf-8")
            pieces = (
                _python_strings(source) if path.suffix == ".py" else _text_lines(source)
            )
            for lineno, text in pieces:
                for match in _MODEL_ID.finditer(text):
                    model_id = match.group(0)
                    if model_id in catalogue or _exempt(relative, model_id):
                        continue
                    found.append(f"{relative}:{lineno}: {model_id}")
    return found


def test_production_names_only_models_the_catalogue_contains():
    offenders = _offenders()
    assert (
        not offenders
    ), "production code names a model the catalogue does not contain:\n" + "\n".join(
        offenders
    )


def test_removal_migration_drops_every_row_outside_the_catalogue():
    """Deletion follows the catalogue, so an old row does not have to be named."""
    path = (
        REPO_ROOT
        / "pib_api"
        / "flask"
        / "migrations"
        / "versions"
        / "d9f2a6c41e88_remove_models_outside_the_catalogue.py"
    )
    source = path.read_text(encoding="utf-8")
    assert "active_api_names()" in source
    assert "DELETE FROM provider WHERE api_name NOT IN" in source
    assert "DELETE FROM assistant_model WHERE api_name NOT IN" in source
    assert not active_api_names() & set(_OLD_CHAT_IDS)
    later = (
        REPO_ROOT
        / "pib_api"
        / "flask"
        / "migrations"
        / "versions"
        / "a8c3e1d74f20_remove_rows_that_left_the_catalogue.py"
    )
    later_source = later.read_text(encoding="utf-8")
    assert "active_api_names()" in later_source
    assert "DELETE FROM provider WHERE api_name NOT IN" in later_source
    assert "DELETE FROM assistant_model WHERE api_name NOT IN" in later_source
    assert 'down_revision = "d9f2a6c41e88"' in later_source
    assert "hermes-agent" not in active_api_names()
