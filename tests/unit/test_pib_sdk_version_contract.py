"""Guard against drift between pib_sdk_version.txt and build defaults (PR-2081)."""

from __future__ import annotations

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
VERSION_FILE = REPO_ROOT / "pib_sdk_version.txt"
PROGRAMS_DOCKERFILE = REPO_ROOT / "ros_packages" / "programs" / "Dockerfile"
FLASK_DOCKERFILE = REPO_ROOT / "pib_api" / "flask" / "Dockerfile"
COMPOSE_FILE = REPO_ROOT / "docker-compose.yaml"

ARG_PATTERN = re.compile(r"^ARG PIB_SDK_VERSION=(\d+\.\d+\.\d+)\s*$", re.MULTILINE)
COMPOSE_DEFAULT = re.compile(r"^\$\{PIB_SDK_VERSION:-(\d+\.\d+\.\d+)\}$")


def _canonical() -> str:
    return VERSION_FILE.read_text(encoding="utf-8").strip()


def test_pib_sdk_version_file_is_single_line() -> None:
    text = VERSION_FILE.read_text(encoding="utf-8")
    assert text.endswith("\n")
    assert len(text.strip().splitlines()) == 1


def test_programs_dockerfile_default_matches_file() -> None:
    text = PROGRAMS_DOCKERFILE.read_text(encoding="utf-8")
    match = ARG_PATTERN.search(text)
    assert match, "programs Dockerfile missing ARG PIB_SDK_VERSION"
    assert match.group(1) == _canonical()


def test_flask_dockerfile_default_matches_file() -> None:
    text = FLASK_DOCKERFILE.read_text(encoding="utf-8")
    match = ARG_PATTERN.search(text)
    assert match, "flask Dockerfile missing ARG PIB_SDK_VERSION"
    assert match.group(1) == _canonical()


def test_compose_defaults_match_file() -> None:
    compose = yaml.safe_load(COMPOSE_FILE.read_text(encoding="utf-8"))
    for service in ("flask-app", "ros-programs"):
        args = compose["services"][service]["build"]["args"]
        raw = args["PIB_SDK_VERSION"]
        match = COMPOSE_DEFAULT.fullmatch(str(raw).strip())
        assert match, f"{service} PIB_SDK_VERSION compose default unexpected: {raw!r}"
        assert match.group(1) == _canonical()
