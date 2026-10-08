"""Container builds and the installer stop fighting themselves (PR-1910, findings 6, 7).

Static checks on the shipped Dockerfiles, the installer scripts and the
pib_mcp_server packaging: pinned third-party installs in one resolve, in-repo
packages with --no-deps, no pip-as-root warnings, no systemd-tmpfiles in a
build, apt-get instead of apt, and PEP 660 metadata for the editable install.
"""

from __future__ import annotations

import re
import shlex
from pathlib import Path

import pytest

try:
    import tomllib
except ModuleNotFoundError:  # Python < 3.11
    import tomli as tomllib  # type: ignore[no-redef]

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_DIR = REPO_ROOT / "setup"

# Every image docker-compose.yaml builds.
COMPOSE_DOCKERFILES = (
    "pib_api/flask/Dockerfile",
    "ros_packages/rosbridge/Dockerfile",
    "ros_packages/camera/Dockerfile",
    "ros_packages/motors/Dockerfile",
    "ros_packages/voice_assistant/Dockerfile",
    "ros_packages/programs/Dockerfile",
    "ros_packages/display/Dockerfile",
    "ros_packages/ros_audio_io/Dockerfile",
)

# Images whose apt packages pull in dbus/systemd and ran systemd-tmpfiles in the build.
TMPFILES_DOCKERFILES = (
    "ros_packages/voice_assistant/Dockerfile",
    "ros_packages/display/Dockerfile",
    "ros_packages/ros_audio_io/Dockerfile",
)

INSTALLER_SCRIPTS = (
    SETUP_DIR / "setup-pib.sh",
    SETUP_DIR / "installation_scripts" / "docker_install.sh",
    SETUP_DIR / "installation_scripts" / "ros_jazzy_install.sh",
    SETUP_DIR / "installation_scripts" / "set_system_settings.sh",
)


def _run_lines(dockerfile: Path) -> list[str]:
    """RUN instructions with continuation lines joined."""
    text = dockerfile.read_text(encoding="utf-8")
    text = re.sub(r"\\\n", " ", text)
    return [line for line in text.splitlines() if line.startswith("RUN ")]


def _pip_installs(dockerfile: Path) -> list[list[str]]:
    commands = []
    for line in _run_lines(dockerfile):
        for part in re.split(r"\s*&&\s*", line[len("RUN ") :]):
            words = shlex.split(part)
            if words[:2] == ["pip", "install"]:
                commands.append(words[2:])
    return commands


def _requirements(words: list[str]) -> list[str]:
    """Package requirements of a pip install, without flags and option values."""
    requirements = []
    skip = False
    for word in words:
        if skip:
            skip = False
            continue
        if word in ("-r", "-c", "--index-url", "--extra-index-url"):
            skip = True
            continue
        if word.startswith("-"):
            continue
        requirements.append(word)
    return requirements


def _is_local(requirement: str) -> bool:
    return requirement.startswith(("./", "/"))


def _is_pinned(requirement: str) -> bool:
    return bool(re.search(r"(==|<|>|~=)", requirement))


@pytest.mark.parametrize("relative", COMPOSE_DOCKERFILES)
def test_pip_runs_without_root_and_version_check_warnings(relative):
    text = (REPO_ROOT / relative).read_text(encoding="utf-8")
    if not _pip_installs(REPO_ROOT / relative):
        pytest.skip("image does not use pip")
    assert "PIP_ROOT_USER_ACTION=ignore" in text, relative
    assert text.index("PIP_ROOT_USER_ACTION=ignore") < text.index(
        "pip install"
    ), relative


@pytest.mark.parametrize("relative", COMPOSE_DOCKERFILES)
def test_third_party_packages_are_pinned_and_resolved_once(relative):
    installs = _pip_installs(REPO_ROOT / relative)
    if not installs:
        pytest.skip("image does not use pip")

    third_party_installs = []
    for words in installs:
        requirements = _requirements(words)
        local = [r for r in requirements if _is_local(r)]
        pypi = [r for r in requirements if not _is_local(r)]
        if "-r" in words:
            pypi.append("requirements.txt")
        if pypi:
            assert (
                not local
            ), f"{relative}: local and PyPI packages in one resolve: {words}"
            third_party_installs.append(words)
            for requirement in pypi:
                if requirement == "requirements.txt":
                    continue
                assert _is_pinned(requirement), f"{relative}: unpinned {requirement}"
        if local:
            assert (
                "--no-deps" in words
            ), f"{relative}: in-repo packages need --no-deps: {words}"
            assert (
                not pypi
            ), f"{relative}: in-repo packages must be installed alone: {words}"
    assert len(third_party_installs) <= 2, f"{relative}: {third_party_installs}"


def test_the_flask_requirements_file_is_fully_pinned():
    lines = (REPO_ROOT / "pib_api/flask/requirements.txt").read_text().splitlines()
    for line in lines:
        line = line.split("#", 1)[0].strip()
        if line:
            assert _is_pinned(line), line


@pytest.mark.parametrize("relative", TMPFILES_DOCKERFILES)
def test_systemd_tmpfiles_is_a_no_op_before_any_apt_install(relative):
    text = (REPO_ROOT / relative).read_text(encoding="utf-8")
    divert = text.index("dpkg-divert --local --rename --add /usr/bin/systemd-tmpfiles")
    assert "ln -s /bin/true /usr/bin/systemd-tmpfiles" in text
    assert divert < text.index("apt-get install"), relative


def test_the_voice_image_installs_the_mcp_server_dependencies_itself():
    """pib_mcp_server goes in with --no-deps, so mcp and websocket-client are pinned."""
    text = (REPO_ROOT / "ros_packages/voice_assistant/Dockerfile").read_text()
    assert re.search(r"\bmcp==\d", text)
    assert re.search(r"\bwebsocket-client==\d", text)
    assert re.search(r"\bhermes-agent==\d", text)
    assert "./pib_mcp_server/" in text


@pytest.mark.parametrize("relative", COMPOSE_DOCKERFILES)
def test_dockerfiles_use_apt_get(relative):
    for line in _run_lines(REPO_ROOT / relative):
        assert not re.search(
            r"(^|[;&|]\s*|RUN\s+)apt\s+(update|install|upgrade)", line
        ), (
            relative,
            line,
        )


@pytest.mark.parametrize("script", INSTALLER_SCRIPTS, ids=lambda p: p.name)
def test_installer_scripts_use_apt_get(script):
    for number, line in enumerate(script.read_text().splitlines(), start=1):
        code = line.split("#", 1)[0]
        assert not re.search(
            r"\bapt\s+(-\S+\s+)*(update|install|upgrade|purge)\b", code
        ), (
            script.name,
            number,
            line,
        )


def test_pib_mcp_server_is_built_with_pep_660_metadata():
    package_dir = REPO_ROOT / "pib_mcp_server"
    pyproject = tomllib.loads((package_dir / "pyproject.toml").read_text())

    assert not (
        package_dir / "setup.py"
    ).exists(), "setup.py develop is the legacy path"
    assert pyproject["build-system"]["build-backend"] == "setuptools.build_meta"
    requires = pyproject["build-system"]["requires"]
    assert any(
        r.startswith("setuptools>=") and int(r.split(">=")[1]) >= 64 for r in requires
    )
    assert pyproject["project"]["name"] == "pib_mcp_server"
    assert set(pyproject["project"]["dependencies"]) == {"mcp", "websocket-client"}
    assert (
        pyproject["project"]["scripts"]["pib-mcp-server"]
        == "pib_mcp_server.server:main"
    )
    assert pyproject["tool"]["setuptools"]["package-dir"] == {"pib_mcp_server": "."}

    setup = (SETUP_DIR / "setup-pib.sh").read_text()
    assert (
        'pip install --break-system-packages -e "$BACKEND_DIR/pib_mcp_server"' in setup
    )
