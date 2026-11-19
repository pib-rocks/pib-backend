"""Device-free tests for saved deployment state and offline recovery."""

from __future__ import annotations

import importlib.util
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
if [[ "$joined" == *" --pull missing "* ]]; then
    exit "${DOCKER_PULL_UP_STATUS:-${DOCKER_UP_STATUS:-0}}"
fi
if [[ "$joined" == *" --pull never "* && "$joined" == *" --build "* ]]; then
    exit "${DOCKER_BUILD_UP_STATUS:-0}"
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


def test_recorded_deployments_recover_preferring_existing_or_registry_images(
    tmp_path: Path,
) -> None:
    env, docker_log = _helper_env(tmp_path)
    backend_model = tmp_path / "backend-compose.json"
    cerebra_model = tmp_path / "cerebra-compose.json"
    backend_model.write_text(
        json.dumps(
            {
                "name": "pib-backend",
                "services": {"flask": {"image": "ghcr.io/custom/backend-flask:v1.2.3"}},
            }
        ),
        encoding="utf-8",
    )
    cerebra_model.write_text(
        json.dumps(
            {
                "name": "cerebra",
                "services": {"ui": {"image": "registry.example/robot-ui:v1.2.3"}},
            }
        ),
        encoding="utf-8",
    )
    env["PIB_VOICE_DATA_DIR"] = "/caller-environment-value"
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
            "--backend-compose-config",
            str(backend_model),
            "--cerebra-compose-config",
            str(cerebra_model),
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
    assert backend["composeConfig"]["services"]["flask"]["image"].startswith(
        "ghcr.io/custom/"
    )
    cerebra = next(
        project for project in state["projects"] if project["name"] == "cerebra"
    )
    assert cerebra["composeConfig"]["services"]["ui"]["image"] == (
        "registry.example/robot-ui:v1.2.3"
    )

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
        assert "--pull missing" in call
        assert "--no-build" in call
        assert " build " not in f" {call} "


def test_recovery_falls_back_to_local_build_when_pull_fails(tmp_path: Path) -> None:
    env, docker_log = _helper_env(tmp_path)
    env["DOCKER_PULL_UP_STATUS"] = "1"
    env["DOCKER_BUILD_UP_STATUS"] = "0"
    models = []
    for name, image in (
        ("backend", "ghcr.io/custom/backend-flask:v1.2.3"),
        ("cerebra", "registry.example/robot-ui:v1.2.3"),
    ):
        model = tmp_path / f"{name}-compose.json"
        project_name = "pib-backend" if name == "backend" else "cerebra"
        model.write_text(
            json.dumps(
                {
                    "name": project_name,
                    "services": {"app": {"image": image}},
                }
            )
        )
        models.append(model)
    record = subprocess.run(
        [
            sys.executable,
            str(HELPER),
            "record",
            "--backend-tag",
            "v1.2.3",
            "--cerebra-tag",
            "v1.2.3",
            "--backend-compose-config",
            str(models[0]),
            "--cerebra-compose-config",
            str(models[1]),
        ],
        env=env,
        capture_output=True,
        check=False,
        text=True,
    )
    assert record.returncode == 0, record.stderr

    recover = subprocess.run(
        [sys.executable, str(HELPER), "recover"],
        env=env,
        capture_output=True,
        check=False,
        text=True,
    )

    assert recover.returncode == 0, recover.stderr
    calls = [
        line for line in docker_log.read_text().splitlines() if " up " in f" {line} "
    ]
    assert len(calls) == 4
    for pull, build in zip(calls[::2], calls[1::2]):
        assert "--pull missing" in pull and "--no-build" in pull
        assert "--pull never" in build and "--build" in build


def test_recovery_does_not_require_chmod_on_a_shared_deployment_lock(
    monkeypatch, tmp_path: Path
) -> None:
    env, _docker_log = _helper_env(tmp_path)
    models = []
    for name, project in (("backend", "pib-backend"), ("cerebra", "cerebra")):
        model = tmp_path / f"{name}-compose.json"
        model.write_text(
            json.dumps(
                {
                    "name": project,
                    "services": {"app": {"image": f"registry.example/{name}:v1"}},
                }
            ),
            encoding="utf-8",
        )
        models.append(model)
    record = subprocess.run(
        [
            sys.executable,
            str(HELPER),
            "record",
            "--backend-tag",
            "v1",
            "--cerebra-tag",
            "v1",
            "--backend-compose-config",
            str(models[0]),
            "--cerebra-compose-config",
            str(models[1]),
        ],
        env=env,
        capture_output=True,
        check=False,
        text=True,
    )
    assert record.returncode == 0, record.stderr

    monkeypatch.setenv("PIB_UPDATE_DIR", env["PIB_UPDATE_DIR"])
    monkeypatch.setenv("PIB_BACKEND_DIR", env["PIB_BACKEND_DIR"])
    monkeypatch.setenv("PIB_CEREBRA_DIR", env["PIB_CEREBRA_DIR"])
    spec = importlib.util.spec_from_file_location("deployment_recovery_test", HELPER)
    assert spec is not None and spec.loader is not None
    recovery = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recovery)

    lock_path = Path(env["PIB_UPDATE_DIR"]) / "deployment.lock"
    original_chmod = os.chmod

    def chmod_without_lock_ownership(path, mode):
        if Path(path) == lock_path:
            raise PermissionError("shared lock is owned by root")
        return original_chmod(path, mode)

    monkeypatch.setattr(os, "chmod", chmod_without_lock_ownership)
    monkeypatch.setattr(
        recovery.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args[0], 0),
    )

    recovery.recover()


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


def test_record_uses_the_compose_models_captured_from_the_installer_environment(
    tmp_path: Path,
) -> None:
    env, _ = _helper_env(tmp_path)
    backend_model = tmp_path / "backend-compose.json"
    cerebra_model = tmp_path / "cerebra-compose.json"
    backend_model.write_text(
        json.dumps(
            {
                "name": "pib-backend",
                "services": {
                    "flask": {
                        "image": "registry.example/pib-flask:develop",
                        "volumes": [
                            {
                                "type": "bind",
                                "source": "/custom/voice",
                                "target": "/data/voice",
                            }
                        ],
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    cerebra_model.write_text(
        json.dumps(
            {
                "name": "cerebra",
                "services": {"ui": {"image": "registry.example/custom-ui:develop"}},
            }
        ),
        encoding="utf-8",
    )
    env["PIB_VOICE_DATA_DIR"] = "/different-caller-value"

    result = subprocess.run(
        [
            sys.executable,
            str(HELPER),
            "record",
            "--backend-tag",
            "develop",
            "--cerebra-tag",
            "develop",
            "--backend-compose-config",
            str(backend_model),
            "--cerebra-compose-config",
            str(cerebra_model),
        ],
        env=env,
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    state = json.loads((tmp_path / ".update" / "deployments.json").read_text())
    backend = next(
        project for project in state["projects"] if project["name"] == "pib-backend"
    )
    assert backend["composeConfig"]["services"]["flask"]["volumes"][0]["source"] == (
        "/custom/voice"
    )


def test_record_keeps_compatibility_with_older_installer_callers(
    tmp_path: Path,
) -> None:
    env, _ = _helper_env(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(HELPER),
            "record",
            "--backend-tag",
            "v2.0.0",
            "--cerebra-tag",
            "v2.0.0",
            "--backend-version",
            "2.0.0",
            "--cerebra-version",
            "2.0.0",
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

    assert result.returncode == 0, result.stderr
    state = json.loads((tmp_path / ".update" / "deployments.json").read_text())
    assert len(state["projects"]) == 2
    assert all(project["imageTag"] == "v2.0.0" for project in state["projects"])
