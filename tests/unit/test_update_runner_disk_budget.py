"""PR-2110: update runner disk budget, reclaim, cleanup, rollback status."""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "setup" / "update_runner.sh"

BACKEND_SHA = "17cd52bf17cd52bf17cd52bf17cd52bf17cd52bf"
CEREBRA_SHA = "e6f5f09fe6f5f09fe6f5f09fe6f5f09fe6f5f09f"
TARGET_BACKEND_SHA = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
TARGET_CEREBRA_SHA = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"

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

POST_CHECKOUT_FLAG="${STUB_POST_CHECKOUT_FLAG:-}"
if [ -n "$POST_CHECKOUT_FLAG" ] && [ -f "$POST_CHECKOUT_FLAG" ]; then
    post_checkout=1
else
    post_checkout=0
fi

command_name="${1:-}"
case "$command_name" in
    rev-parse)
        base="${directory##*/}"
        if [ "$base" = "cerebra" ]; then
            if [ "$post_checkout" = "1" ]; then
                printf '%s\\n' "$STUB_TARGET_CEREBRA_SHA"
            else
                printf '%s\\n' "$STUB_CEREBRA_SHA"
            fi
        else
            if [ "$post_checkout" = "1" ]; then
                printf '%s\\n' "$STUB_TARGET_BACKEND_SHA"
            else
                printf '%s\\n' "$STUB_BACKEND_SHA"
            fi
        fi
        ;;
    fetch | checkout | clean)
        if [ -n "$POST_CHECKOUT_FLAG" ]; then
            touch "$POST_CHECKOUT_FLAG"
        fi
        exit 0
        ;;
    reset)
        if [ -n "$POST_CHECKOUT_FLAG" ]; then
            rm -f "$POST_CHECKOUT_FLAG"
        fi
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
            if [ "$post_checkout" = "1" ] && [ -n "${STUB_TAG_HEAD_AFTER:-}" ]; then
                printf '%s\\n' "$STUB_TAG_HEAD_AFTER"
            elif [ -n "${STUB_TAG_HEAD:-}" ]; then
                printf '%s\\n' "$STUB_TAG_HEAD"
            fi
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
    else
        printf 'plain\\n' >> "${STUB_PROFILE_BUILD_LOG:?}"
    fi
fi

if [ "${1:-}" = "builder" ] && [ "${2:-}" = "prune" ]; then
    printf 'Total reclaimed space: 1.024GB\\n'
    exit 0
fi

if [ "${1:-}" = "image" ] && [ "${2:-}" = "prune" ]; then
    printf 'Total reclaimed space: 512MB\\n'
    exit 0
fi

if [ "${1:-}" = "images" ]; then
    cat "${STUB_DOCKER_IMAGES_LIST:?}"
    exit 0
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
if [[ "$joined" == *" up "* && "$joined" == *" --force-recreate "* ]]; then
    exit 0
fi
exit 0
"""

DF_STUB = """#!/bin/bash
if [ "${1:-}" = "-Pk" ]; then
    avail="${STUB_FREE_KIB:?}"
    printf 'Filesystem 1024-blocks Used Available Use Mounted\\n'
    printf '/dev/root 100000000 50000000 %s 50 %% /\\n' "$avail"
    exit 0
fi
exec /usr/bin/df "$@"
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


def _request(*, channel: str = "develop", release: str = "") -> dict[str, object]:
    document: dict[str, object] = {
        "jobId": "job-2110",
        "requestedAt": "2026-10-28T12:00:00Z",
        "actor": "test",
        "channel": channel,
        "force": False,
        "confirmation": "UPDATE",
    }
    if release:
        document["release"] = release
        document["targets"] = {
            "pib-backend": TARGET_BACKEND_SHA,
            "cerebra": TARGET_CEREBRA_SHA,
        }
        document["targetKind"] = "published-release"
    return document


def _run(
    tmp_path: Path,
    *,
    free_kib: str,
    required_free_kib: str | None = None,
    channel: str = "develop",
    release: str = "",
    tag_head: str = "",
    tag_head_after: str = "",
    pib_sdk_installed: str = "0.7.4",
    docker_images_list: str | None = None,
) -> tuple[subprocess.CompletedProcess[str], Path, str, str]:
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
    (update_dir / "request.json").write_text(
        json.dumps(_request(channel=channel, release=release)), encoding="utf-8"
    )

    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    _install_stub(stub_bin, "git", GIT_STUB)
    _install_stub(stub_bin, "docker", DOCKER_STUB)
    _install_stub(stub_bin, "curl", CURL_STUB)
    _install_stub(stub_bin, "systemctl", SYSTEMCTL_STUB)
    _install_stub(stub_bin, "dpkg-query", DPKG_QUERY_STUB)
    _install_stub(stub_bin, "sudo", SUDO_STUB)
    _install_stub(stub_bin, "df", DF_STUB)

    docker_log = tmp_path / "docker.log"
    docker_log.write_text("", encoding="utf-8")
    profile_build_log = tmp_path / "profile-build.log"
    profile_build_log.write_text("", encoding="utf-8")
    images_list = tmp_path / "docker-images.txt"
    images_list.write_text(
        docker_images_list
        or "\n".join(
            [
                "ghcr.io/pib-rocks/pib-backend-flask-app:v0.7.2",
                "ghcr.io/pib-rocks/pib-backend-flask-app:v0.7.3",
                "ghcr.io/pib-rocks/pib-backend-flask-app:v0.7.4",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    post_checkout_flag = tmp_path / "post-checkout"

    env = os.environ.copy()
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env.get('PATH', '')}",
        PIB_UPDATE_DIR=str(update_dir),
        PIB_BACKEND_DIR=str(backend),
        PIB_CEREBRA_DIR=str(cerebra),
        PIB_UPDATE_MIN_FREE_KIB="0",
        PIB_UPDATE_IMAGE_SET_KIB="0",
        PIB_UPDATE_BUILD_CACHE_HEADROOM_KIB="0",
        PIB_UPDATE_VERIFY_ATTEMPTS="3",
        PIB_UPDATE_VERIFY_INTERVAL_SECONDS="0",
        PIB_MODEL_STORE=str(model_store),
        PIB_MODEL_CACHE=str(tmp_path / "model-cache"),
        PIB_MODEL_ASSET_URL="http://127.0.0.1:9/nope.tar.gz",
        PIB_USE_REGISTRY_IMAGES="0",
        STUB_FREE_KIB=free_kib,
        STUB_BACKEND_SHA=BACKEND_SHA,
        STUB_CEREBRA_SHA=CEREBRA_SHA,
        STUB_TARGET_BACKEND_SHA=TARGET_BACKEND_SHA,
        STUB_TARGET_CEREBRA_SHA=TARGET_CEREBRA_SHA,
        STUB_HEAD2_MISSING="1",
        STUB_TAG_HEAD=tag_head,
        STUB_TAG_HEAD_AFTER=tag_head_after,
        STUB_PIB_SDK_INSTALLED=pib_sdk_installed,
        DOCKER_LOG=str(docker_log),
        STUB_PROFILE_BUILD_LOG=str(profile_build_log),
        STUB_DOCKER_IMAGES_LIST=str(images_list),
        STUB_POST_CHECKOUT_FLAG=str(post_checkout_flag),
    )
    if required_free_kib is not None:
        env["PIB_UPDATE_REQUIRED_FREE_KIB"] = required_free_kib

    result = subprocess.run(
        ["bash", str(RUNNER)],
        capture_output=True,
        check=False,
        cwd=tmp_path,
        env=env,
        text=True,
        timeout=120,
    )
    return (
        result,
        update_dir,
        docker_log.read_text(encoding="utf-8"),
        (update_dir / "update.log").read_text(encoding="utf-8", errors="replace"),
    )


def test_insufficient_disk_fails_before_recreate(tmp_path: Path) -> None:
    result, update_dir, docker_log, log = _run(
        tmp_path,
        free_kib="1048576",
        required_free_kib="2097152",
    )

    assert result.returncode != 0
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "failed"
    assert "required 2048 MiB (2097152 KiB)" in status["message"]
    assert "available 1024 MiB (1048576 KiB)" in status["message"]
    assert "PIB_UPDATE_REQUIRED_FREE_KIB" in status["message"]
    assert "--force-recreate" not in docker_log
    assert "Disk budget satisfied" not in log


def test_sufficient_disk_reaches_build_phase(tmp_path: Path) -> None:
    result, update_dir, _docker_log, log = _run(
        tmp_path,
        free_kib="4194304",
        required_free_kib="2097152",
    )

    assert result.returncode == 0, log
    assert "building: Building and recreating backend and cerebra containers" in log
    assert "Disk budget satisfied: required 2048 MiB (2097152 KiB)" in log


def test_build_phase_prunes_builder_and_dangling_images(tmp_path: Path) -> None:
    _result, _update_dir, docker_log, log = _run(
        tmp_path,
        free_kib="4194304",
        required_free_kib="1048576",
    )

    assert " builder prune -af" in docker_log or "builder prune -af" in docker_log
    assert re.search(r" image prune -f", docker_log)
    assert "Docker reclaim before build" in log
    builder_pos = docker_log.find("builder prune -af")
    image_prune_pos = docker_log.find("image prune -f")
    profile_build_pos = docker_log.find("build --pull")
    assert builder_pos != -1
    assert image_prune_pos != -1
    assert profile_build_pos != -1
    assert builder_pos < profile_build_pos
    assert image_prune_pos < profile_build_pos


def test_rollback_writes_running_status_before_rebuild(tmp_path: Path) -> None:
    result, update_dir, _docker_log, log = _run(
        tmp_path,
        free_kib="4194304",
        required_free_kib="1048576",
        pib_sdk_installed="0.7.3",
    )

    assert result.returncode != 0, log
    rolled_back_line = next(
        line for line in log.splitlines() if line.startswith("[") and "rolling_back:" in line
    )
    rebuild_line = next(
        line
        for line in log.splitlines()
        if "Rollback rebuild uses APP_VERSION=" in line
    )
    assert log.index(rolled_back_line) < log.index(rebuild_line)
    assert BACKEND_SHA in rolled_back_line
    assert CEREBRA_SHA in rolled_back_line
    assert "rebuilding previous revisions" in rolled_back_line.lower()
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "failed"
    assert "rollback attempt also failed" in status["message"]


def test_post_success_cleanup_keeps_current_and_previous_tags(tmp_path: Path) -> None:
    _result, update_dir, docker_log, log = _run(
        tmp_path,
        free_kib="8388608",
        required_free_kib="1048576",
        channel="release",
        release="v0.7.4",
        tag_head="v0.7.3",
        tag_head_after="v0.7.4",
    )

    assert "Post-update image cleanup" in log
    assert "ghcr.io/pib-rocks/pib-backend-flask-app:v0.7.2" in docker_log
    assert "image rm -f ghcr.io/pib-rocks/pib-backend-flask-app:v0.7.2" in docker_log
    assert "image rm -f ghcr.io/pib-rocks/pib-backend-flask-app:v0.7.3" not in docker_log
    assert "image rm -f ghcr.io/pib-rocks/pib-backend-flask-app:v0.7.4" not in docker_log
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "done"
