"""Tests for the docker_cleaner.service installation in docker_install.sh.

The script is executed for real in a bash subprocess; `sudo` is replaced by a
stub on PATH that records its argv and can fail `systemctl start` on demand.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DOCKER_INSTALL = REPO_ROOT / "setup" / "installation_scripts" / "docker_install.sh"

SUCCESS_TEXT = "Docker container cleanup service installed and started"
USERMOD_CALL = "usermod -aG docker pib"
START_CALL = "systemctl start docker_cleaner.service"

# Records every sudo invocation, one command per line, with the leading
# VAR=value assignments dropped so the log holds the command that was run.
SUDO_STUB = """#!/bin/bash
assignments=()
while [[ "${1:-}" == [A-Za-z_]*=* ]]; do
    assignments+=("$1")
    shift
done
args=()
for arg in "$@"; do
    args+=("$arg")
done
printf '%s\\n' "${args[*]}" >> "$STUB_LOG"

if [ "${args[0]:-}" = "docker" ] && [ "${args[1]:-}" = "compose" ]; then
    # Model sudo's default environment filtering for a non-allowlisted override.
    env -u PIB_VOICE_DATA_DIR "${assignments[@]}" "${args[@]}"
    exit $?
fi

if [ "${args[0]:-}" = "systemctl" ] && [ "${args[1]:-}" = "start" ]; then
    status=${STUB_SYSTEMCTL_START_STATUS:-0}
    if [ "$status" -ne 0 ]; then
        echo "Job for docker_cleaner.service failed because the control" \\
            "process exited with error code." >&2
    fi
    exit "$status"
fi
if [ "${args[0]:-}" = "install" ] && [ "${args[1]:-}" = "-d" ]; then
    mkdir -p "${args[-1]}"
    exit 0
fi
exit 0
"""

# `command_exists docker` must be true so install_docker_engine skips the apt work.
DOCKER_STUB = """#!/bin/bash
joined=" $* "
if [ "${1:-}" = "compose" ] && [[ "$joined" == *" config --format json"* ]]; then
    project="default"
    previous=""
    for argument in "$@"; do
        if [ "$previous" = "--project-name" ]; then project="$argument"; break; fi
        if [ "$previous" = "-f" ]; then project="$(basename "$(dirname "$argument")")"; fi
        previous="$argument"
    done
    tag="${PIB_IMAGE_TAG:-develop}"
    if [ "$project" = "pib-backend" ]; then
        image="ghcr.io/pib-rocks/pib-backend-flask:$tag"
        voice_data="${PIB_VOICE_DATA_DIR:-/data/voice}"
        printf '{"name":"%s","services":{"fixture":{"image":"%s","volumes":[{"type":"bind","source":"%s","target":"/data/voice"}]}}}\\n' \
            "$project" "$image" "$voice_data"
    else
        image="ghcr.io/pib-rocks/cerebra:$tag"
        printf '{"name":"%s","services":{"fixture":{"image":"%s"}}}\\n' \
            "$project" "$image"
    fi
    exit 0
fi
exit 0
"""

# print(), command_exists(), run_step() and require_nonempty() come from setup-pib.sh,
# which sources docker_install.sh; sourcing setup-pib.sh itself would run the
# installer. run_step is reduced to "run the command" here.
HARNESS = """
function print() {
    if [ -z "${2:-}" ]; then
        echo "[[ $1 ]]"
    else
        echo "[$1][[ $2 ]]"
    fi
}
function command_exists() { command -v "$@" >/dev/null 2>&1; }
function run_step() { shift; "$@"; }
function require_nonempty() {
    local name
    for name in "$@"; do
        [ -n "${!name}" ] || { echo "[ERROR][[ ${name} is empty ]]"; return 1; }
    done
}
source "$DOCKER_INSTALL"
"""


def _make_stub_bin(tmp_path: Path) -> Path:
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    for name, body in (("sudo", SUDO_STUB), ("docker", DOCKER_STUB)):
        stub = stub_bin / name
        stub.write_text(body, encoding="utf-8")
        stub.chmod(0o755)
    return stub_bin


def _make_backend_dir(tmp_path: Path) -> Path:
    backend_dir = tmp_path / "pib-backend"
    blockly = (
        backend_dir
        / "pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks"
        / "custom-blocks.ts"
    )
    blockly.parent.mkdir(parents=True)
    blockly.write_text("// vendored\n", encoding="utf-8")
    (backend_dir / "pib_api/flask").mkdir(parents=True)
    (backend_dir / "setup").mkdir()
    shutil.copy2(
        REPO_ROOT / "setup" / "deployment_recovery.py",
        backend_dir / "setup" / "deployment_recovery.py",
    )
    (backend_dir / "docker-compose.yaml").write_text("services: {}\n", encoding="utf-8")
    return backend_dir


def _run(tmp_path: Path, script: str, start_status: int = 0):
    log = tmp_path / "sudo.log"
    log.touch()
    stub_bin = _make_stub_bin(tmp_path)
    backend_dir = _make_backend_dir(tmp_path)
    frontend_dir = tmp_path / "cerebra"
    frontend_dir.mkdir(exist_ok=True)
    (frontend_dir / "docker-compose.yaml").write_text(
        "services: {}\n", encoding="utf-8"
    )

    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        STUB_SYSTEMCTL_START_STATUS=str(start_status),
        BACKEND_DIR=str(backend_dir),
        FRONTEND_DIR=str(frontend_dir),
        DOCKER_INSTALL=str(DOCKER_INSTALL),
        # The compose output file defaults to $HOME; keep it inside the sandbox.
        PIB_DOCKER_BUILD_LOG=str(tmp_path / "docker-build.log"),
        PIB_UPDATE_DIR=str(tmp_path / "update"),
        PIB_BACKEND_DIR=str(backend_dir),
        PIB_CEREBRA_DIR=str(tmp_path / "cerebra"),
    )
    result = subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )
    return result, log.read_text(encoding="utf-8").splitlines()


def _index_of(calls: list[str], needle: str) -> int:
    matches = [i for i, call in enumerate(calls) if needle in call]
    assert matches, f"{needle!r} was never invoked; recorded calls: {calls}"
    return matches[0]


def test_pib_joins_docker_group_before_the_cleaner_service_is_started(tmp_path):
    """The unit runs as User=pib, so it can only reach the socket after usermod."""
    result, calls = _run(tmp_path, HARNESS)

    assert _index_of(calls, USERMOD_CALL) < _index_of(calls, START_CALL)
    assert _index_of(
        calls, "systemctl start pib-voice-hermes-view.service"
    ) < _index_of(calls, "docker compose")
    assert SUCCESS_TEXT in result.stdout
    assert "Filtered Hermes view service installed and started" in result.stdout


def test_hermes_view_service_failing_start_returns_non_zero(tmp_path):
    script = HARNESS + """
setup_hermes_view_service
echo "hermes_view_status=$?"
"""
    result, _ = _run(tmp_path, script, start_status=1)

    assert "hermes_view_status=0" not in result.stdout
    assert "hermes_view_status=1" in result.stdout
    assert "failed to start pib-voice-hermes-view.service" in result.stdout


def test_deployment_recovery_service_uses_persistent_helper(tmp_path):
    script = HARNESS + "\nsetup_update_service\n"
    result, calls = _run(tmp_path, script)

    assert result.returncode == 0, result.stdout + result.stderr
    assert any(
        call.endswith(
            "setup/deployment_recovery.py "
            "/home/pib/app/.update/deployment_recovery.py"
        )
        for call in calls
    )
    unit = (
        REPO_ROOT / "setup" / "setup_files" / "pib-deployment-recovery.service"
    ).read_text(encoding="utf-8")
    assert (
        "ExecStart=/usr/bin/python3 "
        "/home/pib/app/.update/deployment_recovery.py recover"
    ) in unit


def test_start_container_forwards_release_tag_to_backend_and_frontend_compose(tmp_path):
    backend_dir = _make_backend_dir(tmp_path)
    frontend_dir = tmp_path / "cerebra"
    frontend_dir.mkdir()
    (frontend_dir / "docker-compose.yaml").write_text(
        "services: {}\n", encoding="utf-8"
    )
    for repo in (backend_dir, frontend_dir):
        subprocess.run(["git", "init", "-q"], cwd=repo, check=True)
        subprocess.run(
            ["git", "config", "user.email", "test@example.com"], cwd=repo, check=True
        )
        subprocess.run(["git", "config", "user.name", "Test"], cwd=repo, check=True)
        (repo / "README.md").write_text("hi\n", encoding="utf-8")
        subprocess.run(["git", "add", "README.md"], cwd=repo, check=True)
        subprocess.run(["git", "commit", "-q", "-m", "init"], cwd=repo, check=True)
        subprocess.run(["git", "tag", "v0.7.0"], cwd=repo, check=True)

    sudo_raw = tmp_path / "sudo_raw.log"
    sudo_raw.touch()
    (tmp_path / "update").mkdir()
    stub_bin = _make_stub_bin(tmp_path)
    (stub_bin / "sudo").write_text(
        '#!/bin/bash\nprintf "%s\\n" "$*" >> "$SUDO_RAW_LOG"\n'
        "assignments=()\n"
        'while [[ "${1:-}" == [A-Za-z_]*=* ]]; do assignments+=("$1"); shift; done\n'
        'if [ "${1:-}" = "docker" ] && [ "${2:-}" = "compose" ] && [[ " $* " == *" config --format json "* ]]; then\n'
        '  env -u PIB_VOICE_DATA_DIR "${assignments[@]}" "$@"\n  exit $?\nfi\n'
        "exit 0\n",
        encoding="utf-8",
    )
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(tmp_path / "sudo.log"),
        SUDO_RAW_LOG=str(sudo_raw),
        BACKEND_DIR=str(backend_dir),
        FRONTEND_DIR=str(frontend_dir),
        PIB_HARDWARE_VARIANT="pib5edu",
        PIB_VOICE_DATA_DIR="/caller-only-voice-data",
        DOCKER_INSTALL=str(DOCKER_INSTALL),
        PIB_DOCKER_BUILD_LOG=str(tmp_path / "docker-build.log"),
        PIB_UPDATE_DIR=str(tmp_path / "update"),
    )
    subprocess.run(["bash", "-c", HARNESS], env=env, check=True, capture_output=True)
    recorded = (tmp_path / "update" / "deployments.json").read_text(encoding="utf-8")
    state = json.loads(recorded)
    backend_state = next(
        project for project in state["projects"] if project["name"] == "pib-backend"
    )
    cerebra_state = next(
        project for project in state["projects"] if project["name"] == "cerebra"
    )
    assert backend_state["projectDirectory"] == str(backend_dir)
    assert cerebra_state["projectDirectory"] == str(frontend_dir)
    assert backend_state["composeConfig"]["services"]["fixture"]["volumes"][0][
        "source"
    ] == ("/data/voice")
    raw_calls = sudo_raw.read_text(encoding="utf-8").splitlines()
    compose_calls = [c for c in raw_calls if "docker compose" in c]
    assert len(compose_calls) == 6, compose_calls
    assert all("APP_VERSION=v0.7.0" in c for c in compose_calls)
    assert all("PIB_IMAGE_TAG=v0.7.0" in c for c in compose_calls)
    assert compose_calls[0].endswith(" pull")
    assert compose_calls[1].endswith(" up -d --no-build")
    assert compose_calls[2].endswith(" pull")
    assert compose_calls[3].endswith(" up -d --no-build")
    assert "config --format json" in compose_calls[4]
    assert "--project-name pib-backend" in compose_calls[4]
    assert "config --format json" in compose_calls[5]
    assert "--project-name cerebra" in compose_calls[5]


def test_start_container_falls_back_to_build_when_registry_disabled(tmp_path):
    backend_dir = _make_backend_dir(tmp_path)
    frontend_dir = tmp_path / "cerebra"
    frontend_dir.mkdir()
    (frontend_dir / "docker-compose.yaml").write_text(
        "services: {}\n", encoding="utf-8"
    )
    for repo in (backend_dir, frontend_dir):
        subprocess.run(["git", "init", "-q"], cwd=repo, check=True)
        subprocess.run(
            ["git", "config", "user.email", "test@example.com"], cwd=repo, check=True
        )
        subprocess.run(["git", "config", "user.name", "Test"], cwd=repo, check=True)
        (repo / "README.md").write_text("hi\n", encoding="utf-8")
        subprocess.run(["git", "add", "README.md"], cwd=repo, check=True)
        subprocess.run(["git", "commit", "-q", "-m", "init"], cwd=repo, check=True)
        subprocess.run(["git", "tag", "v0.7.0"], cwd=repo, check=True)

    sudo_raw = tmp_path / "sudo_raw.log"
    sudo_raw.touch()
    (tmp_path / "update").mkdir()
    stub_bin = _make_stub_bin(tmp_path)
    (stub_bin / "sudo").write_text(
        '#!/bin/bash\nprintf "%s\\n" "$*" >> "$SUDO_RAW_LOG"\nexit 0\n',
        encoding="utf-8",
    )
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(tmp_path / "sudo.log"),
        SUDO_RAW_LOG=str(sudo_raw),
        BACKEND_DIR=str(backend_dir),
        FRONTEND_DIR=str(frontend_dir),
        PIB_HARDWARE_VARIANT="pib5edu",
        PIB_USE_REGISTRY_IMAGES="0",
        DOCKER_INSTALL=str(DOCKER_INSTALL),
        PIB_DOCKER_BUILD_LOG=str(tmp_path / "docker-build.log"),
        PIB_UPDATE_DIR=str(tmp_path / "update"),
        PIB_BACKEND_DIR=str(backend_dir),
        PIB_CEREBRA_DIR=str(frontend_dir),
    )
    subprocess.run(["bash", "-c", HARNESS], env=env, check=True, capture_output=True)
    raw_calls = sudo_raw.read_text(encoding="utf-8").splitlines()
    compose_calls = [c for c in raw_calls if "docker compose" in c]
    assert len(compose_calls) == 4, compose_calls
    assert all("APP_VERSION=v0.7.0" in call for call in compose_calls)
    assert "--build" in compose_calls[0]
    assert "--build" in compose_calls[1]
    assert "config --format json" in compose_calls[2]
    assert "config --format json" in compose_calls[3]


def test_failing_start_returns_non_zero_and_does_not_report_success(tmp_path):
    script = HARNESS + """
setup_docker_cleaner_service
echo "cleaner_status=$?"
"""
    result, _ = _run(tmp_path, script, start_status=1)

    assert "cleaner_status=0" not in result.stdout
    assert "cleaner_status=1" in result.stdout
    assert SUCCESS_TEXT not in result.stdout
    assert "failed to start docker_cleaner.service" in result.stdout


def test_successful_start_reports_success_and_returns_zero(tmp_path):
    script = HARNESS + """
setup_docker_cleaner_service
echo "cleaner_status=$?"
"""
    result, calls = _run(tmp_path, script)

    assert "cleaner_status=0" in result.stdout
    assert SUCCESS_TEXT in result.stdout
    assert _index_of(calls, START_CALL) >= 0
