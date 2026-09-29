"""Installed revision facts written by the host-side update runner."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from service.update_service import update_directory
from service.version_service import read_app_version

UNKNOWN = "unknown"
REPOSITORIES = ("pib-backend", "cerebra")


def parse_revision(
    repository: str, document: Mapping[str, Any] | None
) -> dict[str, str]:
    """Normalize a revision document without guessing absent values."""
    document = document if isinstance(document, Mapping) else {}

    def value(name: str) -> str:
        candidate = document.get(name)
        if isinstance(candidate, str) and candidate.strip():
            return candidate.strip()
        return UNKNOWN

    return {
        "repository": repository,
        "gitSha": value("gitSha"),
        "buildTime": value("buildTime"),
        "channel": value("channel"),
    }


def _read_revision(path: Path) -> Mapping[str, Any] | None:
    try:
        with path.open(encoding="utf-8") as revision_file:
            document = json.load(revision_file)
    except (FileNotFoundError, OSError, UnicodeError, json.JSONDecodeError):
        return None
    return document if isinstance(document, dict) else None


def installed_revisions(directory: Path | None = None) -> dict[str, Any]:
    directory = directory or update_directory()
    repositories = {
        repository: parse_revision(
            repository, _read_revision(directory / f"{repository}.revision.json")
        )
        for repository in REPOSITORIES
    }
    return {
        "imageVersion": read_app_version(),
        "repositories": repositories,
    }
