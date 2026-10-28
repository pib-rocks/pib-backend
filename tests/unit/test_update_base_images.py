"""PR-2083: backend update refreshes Dockerfile base images with offline fallback.

Executes setup/update_runner.sh with git/docker stubs (same harness shape as
test_update_version_injection.py).
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "setup" / "update_runner.sh"

BACKEND_SHA = "17cd52bf17cd52bf17cd52bf17cd52bf17cd52bf"
CEREBRA_SHA = "e6f5f09fe6f5f09fe6f5f09fe6f5f09fe6f5f09f"

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
        if [ "$base" = "cerebra" ]; then
            printf '%s\\n' "$STUB_CEREBRA_SHA"
        else
            printf '%s\\n' "$STUB_BACKEND_SHA"
        fi
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
        if [ "${STUB_PULL_BUILD_FAIL:-0}" = "1" ]; then
            printf 'simulated pull build failure\\n' >&2
            exit 42
        fi
    else
        printf 'plain\\n' >> "${STUB_PROFILE_BUILD_LOG:?}"
        if [ "${STUB_PLAIN_PROFILE_BUILD_FAIL:-0}" = "1" ]; then
            printf 'simulated plain profile build failure\\n' >&2
            exit 43
        fi
    fi
fi

if [ "${1:-}" = "inspect" ]; then
    printf '%s\\n' running
    exit 0
fi

if [[ "$joined" == *" exec "* ]]; then
    if [[ "$joined" == *" ros-programs "* ]] && [[ "$joined" == *"pib_sdk"* ]]; then
        if [ "${STUB_PIB_SDK_EXEC_FAIL:-0}" = "1" ]; then
            printf '%s\\n' 'ImportError: no module named pib_sdk' >&2
            exit 1
        fi
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
        "jobId": "job-2083",
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


def _run(
    tmp_path: Path,
    *,
    pull_build_fail: bool = False,
    plain_profile_build_fail: bool = False,
    pib_sdk_installed: str = "0.7.4",
    pib_sdk_exec_fail: bool = False,
) -> tuple[subprocess.CompletedProcess[str], Path, Path, str]:
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
    docker_log.touch()
    profile_build_log = tmp_path / "profile-build.log"
    profile_build_log.write_text("", encoding="utf-8")

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
        STUB_BACKEND_SHA=BACKEND_SHA,
        STUB_CEREBRA_SHA=CEREBRA_SHA,
        STUB_HEAD2_MISSING="1",
        STUB_PIB_SDK_INSTALLED=pib_sdk_installed,
        STUB_PIB_SDK_EXEC_FAIL="1" if pib_sdk_exec_fail else "0",
        STUB_PULL_BUILD_FAIL="1" if pull_build_fail else "0",
        STUB_PLAIN_PROFILE_BUILD_FAIL="1" if plain_profile_build_fail else "0",
    )

    result = subprocess.run(
        ["bash", str(RUNNER)],
        capture_output=True,
        check=False,
        cwd=tmp_path,
        env=env,
        text=True,
        timeout=90,
    )
    return result, update_dir, profile_build_log, docker_log.read_text(encoding="utf-8")


def _update_log(update_dir: Path) -> str:
    return (update_dir / "update.log").read_text(encoding="utf-8", errors="replace")


def _profile_build_kinds(profile_build_log: Path) -> list[str]:
    if not profile_build_log.is_file():
        return []
    return [
        line.strip()
        for line in profile_build_log.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def test_rollback_once_skips_base_image_refresh() -> None:
    text = RUNNER.read_text(encoding="utf-8")
    rollback = text.split("rollback_once() {", 1)[1].split(
        "\non_unexpected_error() {", 1
    )[0]
    assert "build_backend_stack rollback" in rollback
    assert "refresh_backend_base_images" not in rollback
    refresh = text.split("refresh_backend_base_images() {", 1)[1].split(
        "\n}\n\n# Documented injection", 1
    )[0]
    assert "build --pull" in refresh
    assert "--profile all" in refresh


def test_pull_enabled_profile_build_succeeds_without_retry(tmp_path: Path) -> None:
    result, update_dir, profile_log, _docker = _run(tmp_path)

    assert result.returncode == 0, _update_log(update_dir)
    assert _profile_build_kinds(profile_log) == ["pull"]
    log = _update_log(update_dir)
    assert (
        "Backend base images: refreshed from registry (--pull build succeeded)" in log
    )
    assert "served from local cache" not in log
    assert "WARNING: pull-enabled backend build failed" not in log
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "done"


def test_pull_build_fails_plain_profile_build_succeeds(tmp_path: Path) -> None:
    result, update_dir, profile_log, _docker = _run(tmp_path, pull_build_fail=True)

    assert result.returncode == 0, _update_log(update_dir)
    assert _profile_build_kinds(profile_log) == ["pull", "plain"]
    log = _update_log(update_dir)
    assert "WARNING: pull-enabled backend build failed (exit 42)" in log
    assert "simulated pull build failure" in log
    assert (
        "Backend base images: served from local cache (pull failed, plain build succeeded)"
        in log
    )
    assert "refreshed from registry" not in log


def test_pull_and_plain_profile_builds_both_fail(tmp_path: Path) -> None:
    result, update_dir, profile_log, _docker = _run(
        tmp_path, pull_build_fail=True, plain_profile_build_fail=True
    )

    assert result.returncode != 0
    assert _profile_build_kinds(profile_log) == ["pull", "plain"]
    log = _update_log(update_dir)
    assert "Backend base image build failed after pull and plain attempts" in log
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "failed"
    assert status["message"] == "Backend container build failed"


def test_rollback_skips_base_pull_after_forward_pull_failed(tmp_path: Path) -> None:
    """Rollback rebuild must not repeat profile-all pull builds (offline-safe)."""
    result, update_dir, profile_log, _docker = _run(
        tmp_path,
        pull_build_fail=True,
        pib_sdk_installed="0.7.3",
    )

    assert result.returncode != 0
    assert _profile_build_kinds(profile_log) == ["pull", "plain"]
    log = _update_log(update_dir)
    assert "Verification failed; attempting one rollback build" in log
    assert "Backend base image build failed" not in log
    assert "Backend container build failed" not in log
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "failed"
    assert "rollback attempt also failed" in status["message"]
