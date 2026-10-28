"""PR-2109: update runner lock and pre-build checkout guard."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "setup" / "update_runner.sh"
WITH_LOCK = REPO_ROOT / "setup" / "dev_tools" / "with-update-lock.sh"

BACKEND_SHA = "ace2efa734b9e181f7f9705eb70e1b40dcdc734a"
CEREBRA_SHA = "e6f5f09fe6f5f09fe6f5f09fe6f5f09fe6f5f09f"
MOVED_BACKEND_SHA = "7a42bcbe0ecea04583fa93a9b1c5350969fcaba7"

GIT_STUB = """#!/bin/bash
directory=""
while [ $# -gt 0 ]; do
    case "$1" in
        -C)
            directory="${2:-}"
            shift 2
            ;;
        *)
            break
            ;;
    esac
done

command_name="${1:-}"
case "$command_name" in
    rev-parse)
        base="${directory##*/}"
        if [ "${STUB_DRIFT_WHEN_BUILDING:-0}" = "1" ]; then
            status_file="${PIB_UPDATE_DIR:?}/status.json"
            if [ -f "$status_file" ] && grep -q '"building"' "$status_file" 2>/dev/null; then
                if [ "$base" = "cerebra" ]; then
                    printf '%s\\n' "${STUB_CEREBRA_SHA}"
                    exit 0
                fi
                printf '%s\\n' "${STUB_MOVED_BACKEND_SHA:?}"
                exit 0
            fi
        fi
        if [ "$base" = "cerebra" ]; then
            printf '%s\\n' "$STUB_CEREBRA_SHA"
        else
            printf '%s\\n' "$STUB_BACKEND_SHA"
        fi
        ;;
    status)
        exit 0
        ;;
    fetch|reset|clean|checkout|submodule)
        exit 0
        ;;
    tag)
        revision=""
        previous=""
        for argument in "$@"; do
            if [ "$previous" = "--points-at" ]; then
                revision="$argument"
            fi
            previous="$argument"
        done
        if [ "$revision" = 'HEAD^2' ]; then
            if [ "${STUB_HEAD2_MISSING:-0}" = "1" ]; then
                printf '%s\\n' 'fatal: Needed a single revision' >&2
                exit 128
            fi
            exit 0
        fi
        if [ "$revision" = "HEAD" ]; then
            exit 0
        fi
        ;;
    ls-remote)
        printf '%s\\n' "${STUB_BACKEND_SHA} refs/heads/develop"
        exit 0
        ;;
esac
exit 0
"""

DOCKER_STUB = """#!/bin/bash
{
    printf 'APP_VERSION=%q' "${APP_VERSION-}"
    for argument in "$@"; do
        printf ' %q' "$argument"
    done
    printf '\\n'
} >> "${DOCKER_LOG:?}"

joined=" $* "
if [[ "$joined" == *" --profile all "* && "$joined" == *" build "* ]]; then
    if [[ "$joined" == *" --pull "* ]] || [[ "$joined" == *" --pull" ]]; then
        printf 'pull\\n' >> "${STUB_PROFILE_BUILD_LOG:?}"
    else
        printf 'plain\\n' >> "${STUB_PROFILE_BUILD_LOG:?}"
    fi
fi

if [ "${1:-}" = "inspect" ]; then
    printf '%s\\n' running
    exit 0
fi

if [[ "$joined" == *" exec "* ]]; then
    if [[ "$joined" == *" ros-programs "* ]] && [[ "$joined" == *"pib_sdk"* ]]; then
        printf '%s\\n' "${STUB_PIB_SDK_INSTALLED:-0.7.4}"
        exit 0
    fi
    printf '%s\\n' alembichead
    exit 0
fi
if [[ "$joined" == *" ps "* && "$joined" == *" -q "* ]]; then
    printf '%s\\n' container123
    exit 0
fi
if [[ "$joined" == *" up "* && "$joined" == *" --force-recreate"* ]]; then
    printf 'recreate\\n' >> "${DOCKER_RECREATE_LOG:?}"
fi
exit 0
"""

CURL_STUB = """#!/bin/bash
exit 0
"""

SYSTEMCTL_STUB = """#!/bin/bash
if [ "${1:-}" = "cat" ]; then
    exit 1
fi
exit 0
"""

DPKG_QUERY_STUB = """#!/bin/bash
exit 1
"""

SUDO_STUB = """#!/bin/bash
exit 1
"""


def _install_stub(directory: Path, name: str, body: str) -> None:
    path = directory / name
    path.write_text(body, encoding="utf-8")
    path.chmod(0o755)


def _request() -> dict[str, object]:
    return {
        "jobId": "job-2109",
        "requestedAt": "2026-10-28T12:00:00Z",
        "actor": "test",
        "channel": "develop",
        "force": False,
        "confirmation": "UPDATE",
    }


def _matching_model_fixture(backend: Path, store: Path) -> None:
    content = b"fixture model blob\n"
    digest = hashlib.sha256(content).hexdigest()
    relative = Path("demo") / "demo.blob"
    manifest = (
        "models:\n"
        "- model_id: demo\n"
        f"  file: {relative.as_posix()}\n"
        f"  sha256: {digest}\n"
    )
    (backend / "models" / relative.parent).mkdir(parents=True)
    (backend / "models" / "manifest.yaml").write_text(manifest, encoding="utf-8")
    (store / relative.parent).mkdir(parents=True)
    (store / relative).write_bytes(content)
    (store / "manifest.yaml").write_text(manifest, encoding="utf-8")


def _harness_env(
    tmp_path: Path,
    *,
    drift_when_building: bool = False,
) -> tuple[dict[str, str], Path, Path, Path, Path]:
    update_dir = tmp_path / "update"
    backend = tmp_path / "backend"
    cerebra = tmp_path / "cerebra"
    update_dir.mkdir()
    backend.mkdir()
    cerebra.mkdir()
    (backend / ".git").mkdir()
    (cerebra / ".git").mkdir()
    (backend / "setup").symlink_to(REPO_ROOT / "setup")
    (backend / "pib_sdk_version.txt").write_text("0.7.4\n", encoding="utf-8")
    model_store = tmp_path / "model-store"
    _matching_model_fixture(backend, model_store)
    (update_dir / "request.json").write_text(json.dumps(_request()), encoding="utf-8")

    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    _install_stub(stub_bin, "git", GIT_STUB)
    _install_stub(stub_bin, "docker", DOCKER_STUB)
    _install_stub(stub_bin, "curl", CURL_STUB)
    _install_stub(stub_bin, "systemctl", SYSTEMCTL_STUB)
    _install_stub(stub_bin, "dpkg-query", DPKG_QUERY_STUB)
    _install_stub(stub_bin, "sudo", SUDO_STUB)

    docker_log = tmp_path / "docker.log"
    docker_log.write_text("", encoding="utf-8")
    profile_build_log = tmp_path / "profile-build.log"
    profile_build_log.write_text("", encoding="utf-8")
    recreate_log = tmp_path / "recreate.log"
    recreate_log.write_text("", encoding="utf-8")

    env = os.environ.copy()
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env.get('PATH', '')}",
        PIB_UPDATE_DIR=str(update_dir),
        PIB_BACKEND_DIR=str(backend),
        PIB_CEREBRA_DIR=str(cerebra),
        PIB_UPDATE_MIN_FREE_KIB="0",
        PIB_UPDATE_PRUNE_BELOW_KIB="0",
        PIB_UPDATE_VERIFY_ATTEMPTS="3",
        PIB_UPDATE_VERIFY_INTERVAL_SECONDS="0",
        PIB_MODEL_STORE=str(model_store),
        PIB_MODEL_CACHE=str(tmp_path / "model-cache"),
        PIB_MODEL_ASSET_URL="http://127.0.0.1:9/nope.tar.gz",
        DOCKER_LOG=str(docker_log),
        STUB_PROFILE_BUILD_LOG=str(profile_build_log),
        DOCKER_RECREATE_LOG=str(recreate_log),
        STUB_BACKEND_SHA=BACKEND_SHA,
        STUB_CEREBRA_SHA=CEREBRA_SHA,
        STUB_MOVED_BACKEND_SHA=MOVED_BACKEND_SHA,
        STUB_HEAD2_MISSING="1",
        STUB_PIB_SDK_INSTALLED="0.7.4",
        PIB_USE_REGISTRY_IMAGES="0",
        STUB_DRIFT_WHEN_BUILDING="1" if drift_when_building else "0",
    )
    return env, update_dir, docker_log, profile_build_log, recreate_log


def _run_runner(
    env: dict[str, str], tmp_path: Path
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(RUNNER)],
        capture_output=True,
        check=False,
        cwd=tmp_path,
        env=env,
        text=True,
        timeout=90,
    )


def _update_log(update_dir: Path) -> str:
    return (update_dir / "update.log").read_text(encoding="utf-8", errors="replace")


def test_runner_acquires_lock_and_completes(tmp_path: Path) -> None:
    env, update_dir, _docker_log, profile_log, _recreate = _harness_env(tmp_path)
    result = _run_runner(env, tmp_path)

    assert result.returncode == 0, _update_log(update_dir)
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "done"
    assert (update_dir / "runner.lock").is_file()
    assert profile_log.read_text(encoding="utf-8").strip() == "pull"


def test_second_runner_fails_while_lock_held(tmp_path: Path) -> None:
    env, update_dir, docker_log, _profile_log, recreate_log = _harness_env(tmp_path)
    lock_path = update_dir / "runner.lock"
    holder = subprocess.Popen(
        [
            "bash",
            "-c",
            f'exec 9>>"{lock_path}"; flock 9; while true; do sleep 1; done',
        ],
        cwd=tmp_path,
    )
    try:
        time.sleep(0.2)
        result = _run_runner(env, tmp_path)
    finally:
        holder.terminate()
        holder.wait(timeout=5)

    assert result.returncode != 0
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "failed"
    assert "runner.lock" in status["message"]
    assert "Another update or deployment holds" in status["message"]
    assert recreate_log.read_text(encoding="utf-8") == ""
    assert "--force-recreate" not in docker_log.read_text(encoding="utf-8")


def test_pre_build_checkout_mismatch_aborts_before_recreate(tmp_path: Path) -> None:
    env, update_dir, _docker_log, _profile_log, recreate_log = _harness_env(
        tmp_path, drift_when_building=True
    )
    result = _run_runner(env, tmp_path)

    assert result.returncode != 0, _update_log(update_dir)
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "failed"
    assert MOVED_BACKEND_SHA in status["message"]
    assert BACKEND_SHA in status["message"]
    assert "another process changed the working tree" in status["message"]
    assert recreate_log.read_text(encoding="utf-8") == ""


def test_with_update_lock_refuses_while_runner_holds_lock(tmp_path: Path) -> None:
    env, update_dir, *_rest = _harness_env(tmp_path)
    lock_path = update_dir / "runner.lock"
    holder = subprocess.Popen(
        [
            "bash",
            "-c",
            f'exec 9>>"{lock_path}"; flock 9; while true; do sleep 1; done',
        ],
        cwd=tmp_path,
    )
    marker = tmp_path / "ran-manual"
    try:
        time.sleep(0.2)
        result = subprocess.run(
            ["bash", str(WITH_LOCK), "--", "touch", str(marker)],
            capture_output=True,
            check=False,
            cwd=tmp_path,
            env=env,
            text=True,
            timeout=30,
        )
    finally:
        holder.terminate()
        holder.wait(timeout=5)

    assert result.returncode == 1
    assert "runner.lock" in result.stderr
    assert "The update runner holds" in result.stderr
    assert not marker.exists()


def test_with_update_lock_runs_command_when_lock_free(tmp_path: Path) -> None:
    env, update_dir, *_rest = _harness_env(tmp_path)
    marker = update_dir / "manual-ran"
    result = subprocess.run(
        ["bash", str(WITH_LOCK), "--", "touch", str(marker)],
        capture_output=True,
        check=False,
        cwd=tmp_path,
        env=env,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert marker.is_file()
