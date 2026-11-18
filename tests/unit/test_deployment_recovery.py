"""Device-free tests for saved deployment state and offline recovery."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HELPER = REPO_ROOT / "setup" / "deployment_recovery.py"

DOCKER_STUB = r"""#!/bin/bash
printf '%s\n' "$*" >> "${DOCKER_LOG:?}"
joined=" $* "
if [[ "$joined" == *" config --format json "* ]]; then
    project=""
    previous=""
    for argument in "$@"; do
        if [ "$previous" = "--project-name" ]; then project="$argument"; break; fi
        previous="$argument"
    done
    tag="${PIB_IMAGE_TAG:-develop}"
    if [ "$project" = "pib-backend" ]; then
        image="ghcr.io/pib-rocks/pib-backend-flask:$tag"
        service="flask-app"
    else
        image="ghcr.io/pib-rocks/cerebra:$tag"
        service="cerebra"
    fi
    printf '{"name":"%s","services":{"%s":{"image":"%s"}}}\n' \
      "$project" "$service" "$image"
    exit 0
fi
exit "${DOCKER_UP_STATUS:-0}"
"""


def _helper_env(tmp_path: Path) -> tuple[dict[str, str], Path]:
    backend = tmp_path / "pib-backend"
    cerebra = tmp_path / "cerebra"
    update_dir = tmp_path / ".update"
    stub_bin = tmp_path / "bin"
    for directory in (backend, cerebra, update_dir, stub_bin):
        directory.mkdir()
    for directory in (backend, cerebra):
        (directory / "docker-compose.yaml").write_text(
            "services:\n  placeholder:\n    image: placeholder:latest\n",
            encoding="utf-8",
        )
    docker = stub_bin / "docker"
    docker.write_text(DOCKER_STUB, encoding="utf-8")
    docker.chmod(0o755)
    docker_log = tmp_path / "docker.log"
    docker_log.write_text("", encoding="utf-8")
    env = os.environ.copy()
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env.get('PATH', '')}",
        PIB_UPDATE_DIR=str(update_dir),
        PIB_BACKEND_DIR=str(backend),
        PIB_CEREBRA_DIR=str(cerebra),
        DOCKER_LOG=str(docker_log),
    )
    return env, docker_log


def test_recorded_deployments_recover_with_local_images_without_build_or_pull(
    tmp_path: Path,
) -> None:
    env, docker_log = _helper_env(tmp_path)
    record = subprocess.run(
        [
            sys.executable,
            str(HELPER),
            "record",
            "--backend-tag",
            "v1.2.3",
            "--cerebra-tag",
            "v1.2.3",
            "--backend-version",
            "1.2.3",
            "--cerebra-version",
            "1.2.3",
            "--hardware-variant",
            "pib5advanced",
            "--smart-chats",
            "0",
        ],
        env=env,
        capture_output=True,
        check=False,
        text=True,
    )
    assert record.returncode == 0, record.stderr

    state = json.loads((tmp_path / ".update" / "deployments.json").read_text())
    backend = next(
        project for project in state["projects"] if project["name"] == "pib-backend"
    )
    assert backend["profiles"] == ["all"]
    assert backend["imageTag"] == "v1.2.3"

    recover = subprocess.run(
        [sys.executable, str(HELPER), "recover"],
        env=env,
        capture_output=True,
        check=False,
        text=True,
    )
    assert recover.returncode == 0, recover.stderr

    recovery_calls = [
        line
        for line in docker_log.read_text(encoding="utf-8").splitlines()
        if " up " in f" {line} "
    ]
    assert len(recovery_calls) == 2
    for call in recovery_calls:
        assert "--no-recreate" in call
        assert "--no-build" in call
        assert "--pull never" in call
        assert " build " not in f" {call} "
        assert " pull " not in f" {call} "


def test_recovery_reports_missing_state_without_invoking_docker(
    tmp_path: Path,
) -> None:
    env, docker_log = _helper_env(tmp_path)

    result = subprocess.run(
        [sys.executable, str(HELPER), "recover"],
        env=env,
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode == 1
    assert "deployment state is missing" in result.stderr
    assert docker_log.read_text(encoding="utf-8") == ""
