"""The update runner bakes the checkout's version and refreshes revision files.

Executes setup/update_runner.sh. git and docker on PATH are stubs, so the test
never fetches, resets a real checkout, or runs docker compose. PIB_UPDATE_DIR
points at a temporary directory.
"""

from __future__ import annotations

import json
import os
import subprocess
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "setup" / "update_runner.sh"

BACKEND_SHA = "17cd52bf17cd52bf17cd52bf17cd52bf17cd52bf"
CEREBRA_SHA = "e6f5f09fe6f5f09fe6f5f09fe6f5f09fe6f5f09f"

GIT_STUB = """#!/bin/bash
if [ -n "${GIT_ARGV_LOG:-}" ]; then
    printf '%s\\n' "$*" >> "$GIT_ARGV_LOG"
fi

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
    fetch)
        if [ "${STUB_TAGS_AFTER_FETCH:-0}" = "1" ]; then
            for argument in "$@"; do
                if [ "$argument" = "--tags" ]; then
                    printf '%s\\n' "$directory" >> "${STUB_TAG_FETCH_LOG:?}"
                    break
                fi
            done
        fi
        ;;
    rev-parse)
        base="${directory##*/}"
        if [ "$base" = "cerebra" ]; then
            printf '%s\\n' "$STUB_CEREBRA_SHA"
        else
            printf '%s\\n' "$STUB_BACKEND_SHA"
        fi
        ;;
    tag)
        if [ "${STUB_TAGS_AFTER_FETCH:-0}" = "1" ]; then
            if ! grep -Fxq -- "$directory" "${STUB_TAG_FETCH_LOG:?}"; then
                exit 0
            fi
        fi
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
            if [ -n "${STUB_TAG_HEAD2:-}" ]; then
                printf '%s\\n' "$STUB_TAG_HEAD2"
            fi
            exit 0
        fi
        if [ "$revision" = "HEAD" ]; then
            if [ -n "${STUB_TAG_HEAD:-}" ]; then
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

if [ "${1:-}" = "inspect" ]; then
    printf '%s\\n' running
    exit 0
fi

joined=" $* "
if [[ "$joined" == *" exec "* ]]; then
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


def _request(channel: str) -> dict[str, object]:
    return {
        "jobId": "job-1859",
        "requestedAt": "2026-09-29T12:00:00Z",
        "actor": "test",
        "channel": channel,
        "force": False,
        "confirmation": "UPDATE",
    }


def _run(
    tmp_path: Path,
    channel: str,
    *,
    tag_on_second_parent: str = "",
    tag_on_head: str = "",
    second_parent_missing: bool = False,
    tags_answered_only_after_tag_fetch: bool = False,
) -> tuple[subprocess.CompletedProcess[str], Path, str]:
    update_dir = tmp_path / "update"
    backend = tmp_path / "backend"
    cerebra = tmp_path / "cerebra"
    update_dir.mkdir()
    backend.mkdir()
    cerebra.mkdir()
    (backend / ".git").mkdir()
    (cerebra / ".git").mkdir()
    (backend / "setup").symlink_to(REPO_ROOT / "setup")
    (update_dir / "request.json").write_text(
        json.dumps(_request(channel)), encoding="utf-8"
    )

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
        DOCKER_LOG=str(docker_log),
        STUB_BACKEND_SHA=BACKEND_SHA,
        STUB_CEREBRA_SHA=CEREBRA_SHA,
        STUB_TAG_HEAD2=tag_on_second_parent,
        STUB_TAG_HEAD=tag_on_head,
        STUB_HEAD2_MISSING="1" if second_parent_missing else "0",
    )
    if tags_answered_only_after_tag_fetch:
        tag_fetch_log = tmp_path / "tag-fetches.log"
        tag_fetch_log.write_text("", encoding="utf-8")
        env["STUB_TAGS_AFTER_FETCH"] = "1"
        env["STUB_TAG_FETCH_LOG"] = str(tag_fetch_log)
        env["GIT_ARGV_LOG"] = str(tmp_path / "git-argv.log")

    result = subprocess.run(
        ["bash", str(RUNNER)],
        capture_output=True,
        check=False,
        cwd=tmp_path,
        env=env,
        text=True,
        timeout=90,
    )
    return result, update_dir, docker_log.read_text(encoding="utf-8")


def _failure_text(result: subprocess.CompletedProcess[str], update_dir: Path) -> str:
    parts = [result.stdout, result.stderr]
    log_path = update_dir / "update.log"
    if log_path.is_file():
        parts.append(log_path.read_text(encoding="utf-8", errors="replace")[-8000:])
    status_path = update_dir / "status.json"
    if status_path.is_file():
        parts.append(status_path.read_text(encoding="utf-8"))
    argv_log = update_dir.parent / "git-argv.log"
    if argv_log.is_file():
        parts.append(argv_log.read_text(encoding="utf-8", errors="replace"))
    return "\n".join(part for part in parts if part)


def _assert_succeeded(
    result: subprocess.CompletedProcess[str], update_dir: Path
) -> None:
    assert result.returncode == 0, _failure_text(result, update_dir)
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "done", _failure_text(result, update_dir)
    assert list(update_dir.glob(".revision.*")) == []
    assert list(update_dir.glob(".status.json.*")) == []


def _revision(update_dir: Path, repository: str) -> dict[str, str]:
    path = update_dir / f"{repository}.revision.json"
    assert path.is_file(), f"missing {path.name}"
    document = json.loads(path.read_text(encoding="utf-8"))
    assert set(document) == {"buildTime", "channel", "gitSha"}
    parsed = datetime.fromisoformat(document["buildTime"])
    assert parsed.tzinfo is not None
    return document


def _assert_revisions(update_dir: Path, channel: str) -> None:
    backend = _revision(update_dir, "pib-backend")
    cerebra = _revision(update_dir, "cerebra")
    assert backend["gitSha"] == BACKEND_SHA
    assert cerebra["gitSha"] == CEREBRA_SHA
    assert backend["channel"] == channel
    assert cerebra["channel"] == channel


def _build_lines(docker_log: str) -> list[str]:
    return [
        line
        for line in docker_log.splitlines()
        if "--build-arg" in line and "flask-app" in line
    ]


def _up_lines(docker_log: str) -> list[str]:
    return [line for line in docker_log.splitlines() if " up " in line]


def _assert_version_passed(docker_log: str, version: str) -> None:
    assert "v0.6.2" not in docker_log
    build_lines = _build_lines(docker_log)
    assert len(build_lines) == 1, docker_log
    build_line = build_lines[0]
    assert build_line.startswith(f"APP_VERSION={version} ")
    assert f"APP_VERSION={version}" in build_line
    assert " build " in build_line
    assert "flask-app" in build_line
    up_lines = _up_lines(docker_log)
    assert up_lines, docker_log
    assert all(line.startswith(f"APP_VERSION={version} ") for line in up_lines)
    build_at = docker_log.splitlines().index(build_line)
    first_up = min(docker_log.splitlines().index(line) for line in up_lines)
    assert build_at < first_up


def test_release_merge_injects_the_tag_from_the_second_parent(tmp_path: Path) -> None:
    result, update_dir, docker_log = _run(
        tmp_path,
        "release",
        tag_on_second_parent="v0.6.3",
        tag_on_head="v0.0.0-not-this",
    )

    _assert_succeeded(result, update_dir)
    _assert_version_passed(docker_log, "v0.6.3")
    assert "v0.0.0-not-this" not in docker_log
    _assert_revisions(update_dir, "release")
    log = (update_dir / "update.log").read_text(encoding="utf-8")
    assert f"Resolved APP_VERSION=v0.6.3 for pib-backend at {BACKEND_SHA}" in log


def test_fast_forward_injects_the_tag_on_head(tmp_path: Path) -> None:
    result, update_dir, docker_log = _run(
        tmp_path,
        "release",
        tag_on_head="v0.6.4",
        second_parent_missing=True,
    )

    _assert_succeeded(result, update_dir)
    _assert_version_passed(docker_log, "v0.6.4")
    _assert_revisions(update_dir, "release")


def test_develop_channel_writes_the_marker(tmp_path: Path) -> None:
    result, update_dir, docker_log = _run(
        tmp_path,
        "develop",
        second_parent_missing=True,
    )

    _assert_succeeded(result, update_dir)
    _assert_version_passed(docker_log, "develop")
    _assert_revisions(update_dir, "develop")


def test_release_without_a_tag_refuses_the_compose_fallback(tmp_path: Path) -> None:
    result, update_dir, docker_log = _run(
        tmp_path,
        "release",
        second_parent_missing=True,
    )

    assert result.returncode != 0
    status = json.loads((update_dir / "status.json").read_text(encoding="utf-8"))
    assert status["state"] == "failed"
    assert (
        f"no git tag on {BACKEND_SHA} (HEAD^2 or HEAD) "
        "after fetching branch main with tags"
        in status["message"]
    )
    assert "compose-file APP_VERSION fallback" in status["message"]
    assert _build_lines(docker_log) == []
    assert _up_lines(docker_log) == []
    assert "v0.6.2" not in docker_log
    assert not (update_dir / "pib-backend.revision.json").exists()
    assert not (update_dir / "cerebra.revision.json").exists()


def test_develop_checkout_that_is_tagged_keeps_the_tag(tmp_path: Path) -> None:
    result, update_dir, docker_log = _run(
        tmp_path,
        "develop",
        tag_on_head="v0.6.3",
        second_parent_missing=True,
    )

    _assert_succeeded(result, update_dir)
    _assert_version_passed(docker_log, "v0.6.3")
    _assert_revisions(update_dir, "develop")


def test_release_tag_resolves_only_after_a_tag_fetch(tmp_path: Path) -> None:
    result, update_dir, docker_log = _run(
        tmp_path,
        "release",
        tag_on_second_parent="v0.6.4",
        tag_on_head="v0.0.0-not-this",
        tags_answered_only_after_tag_fetch=True,
    )

    _assert_succeeded(result, update_dir)
    _assert_version_passed(docker_log, "v0.6.4")
    assert "v0.0.0-not-this" not in docker_log
    _assert_revisions(update_dir, "release")
    log = (update_dir / "update.log").read_text(encoding="utf-8")
    assert f"Resolved APP_VERSION=v0.6.4 for pib-backend at {BACKEND_SHA}" in log
