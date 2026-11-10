"""Installed revision facts written by the host-side update runner."""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Any, Mapping

from service.update_service import update_directory
from service.version_service import is_release_tag, read_app_version, version_provenance

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


def _git_command(repo_dir: Path, *args: str) -> str | None:
    try:
        completed = subprocess.run(
            ["git", "-C", str(repo_dir), *args],
            capture_output=True,
            text=True,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    value = completed.stdout.strip()
    return value if value else None


def _channel_from_git_checkout(repo_dir: Path) -> str:
    branch = _git_command(repo_dir, "rev-parse", "--abbrev-ref", "HEAD")
    if not branch:
        return UNKNOWN
    if branch == "develop" or branch.startswith("PR-"):
        return "develop"
    if branch == "main":
        return "release"
    if branch == "HEAD":
        tag = _git_command(repo_dir, "describe", "--tags", "--exact-match")
        if tag and is_release_tag(tag):
            return "release"
    return UNKNOWN


def _git_metadata_from_checkout(repo_dir: Path) -> Mapping[str, str] | None:
    git_sha = _git_command(repo_dir, "rev-parse", "HEAD")
    if not git_sha:
        return None
    build_time = _git_command(repo_dir, "show", "-s", "--format=%cI", "HEAD") or UNKNOWN
    return {
        "gitSha": git_sha,
        "buildTime": build_time,
        "channel": _channel_from_git_checkout(repo_dir),
    }


def _default_checkout_candidates(repository: str) -> tuple[Path, ...]:
    if repository == "pib-backend":
        return (
            Path(os.environ.get("PIB_BACKEND_DIR", "/home/pib/app/pib-backend")),
            Path("/app"),
        )
    if repository == "cerebra":
        return (Path(os.environ.get("PIB_CEREBRA_DIR", "/home/pib/app/cerebra")),)
    return ()


def _git_revision_fallback(
    repository: str, checkout_dirs: Mapping[str, Path] | None = None
) -> Mapping[str, str] | None:
    if checkout_dirs is not None:
        candidate = checkout_dirs.get(repository)
        candidates = (candidate,) if candidate is not None else ()
    else:
        candidates = _default_checkout_candidates(repository)

    for repo_dir in candidates:
        metadata = _git_metadata_from_checkout(repo_dir)
        if metadata is not None:
            return metadata
    return None


def _fill_revision_from_git(
    parsed: dict[str, str], fallback: Mapping[str, str] | None
) -> dict[str, str]:
    if fallback is None:
        return parsed
    result = dict(parsed)
    for field in ("gitSha", "buildTime", "channel"):
        if result.get(field) != UNKNOWN:
            continue
        value = fallback.get(field)
        if isinstance(value, str) and value.strip() and value.strip() != UNKNOWN:
            result[field] = value.strip()
    return result


def installed_revisions(
    directory: Path | None = None,
    checkout_dirs: Mapping[str, Path] | None = None,
) -> dict[str, Any]:
    resolved_directory = directory or update_directory()
    use_git_fallback = (
        checkout_dirs is not None
        or resolved_directory.resolve() == update_directory().resolve()
    )
    repositories = {}
    for repository in REPOSITORIES:
        parsed = parse_revision(
            repository,
            _read_revision(resolved_directory / f"{repository}.revision.json"),
        )
        if use_git_fallback:
            parsed = _fill_revision_from_git(
                parsed, _git_revision_fallback(repository, checkout_dirs)
            )
        repositories[repository] = parsed
    image_version = read_app_version()
    return {
        "imageVersion": image_version,
        "versionProvenance": version_provenance(image_version),
        "repositories": repositories,
    }
