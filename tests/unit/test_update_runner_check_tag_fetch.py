"""run_check() refreshes local tag refs before release discovery."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "setup" / "update_runner.sh"

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
        exit 0
        ;;
    rev-parse)
        printf '%s\\n' deadbeefdeadbeefdeadbeefdeadbeefdeadbeef
        ;;
    ls-remote)
        printf '%s\\trefs/heads/develop\\n' abcdabcdabcdabcdabcdabcdabcdabcdabcdabcd
        ;;
esac
exit 0
"""


def _install_stub(directory: Path, name: str, body: str) -> None:
    path = directory / name
    path.write_text(body, encoding="utf-8")
    path.chmod(0o755)


def test_run_check_fetches_tags_on_both_repositories(tmp_path):
    update_dir = tmp_path / ".update"
    update_dir.mkdir()
    backend = tmp_path / "pib-backend"
    cerebra = tmp_path / "cerebra"
    for repository in (backend, cerebra):
        repository.mkdir()
        shutil.copytree(REPO_ROOT / "setup", repository / "setup")

    check = {
        "schemaVersion": 1,
        "requestedAt": "2026-11-10T12:00:00Z",
        "actor": "127.0.0.1",
        "channel": "develop",
    }
    (update_dir / "check.json").write_text(json.dumps(check), encoding="utf-8")

    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    _install_stub(stub_bin, "git", GIT_STUB)

    git_log = tmp_path / "git-argv.log"
    git_log.write_text("", encoding="utf-8")

    env = os.environ.copy()
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env.get('PATH', '')}",
        PIB_UPDATE_DIR=str(update_dir),
        PIB_BACKEND_DIR=str(backend),
        PIB_CEREBRA_DIR=str(cerebra),
        GIT_ARGV_LOG=str(git_log),
    )

    result = subprocess.run(
        ["bash", str(RUNNER), "--check"],
        capture_output=True,
        check=False,
        cwd=tmp_path,
        env=env,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    log = git_log.read_text(encoding="utf-8")
    assert f"-C {backend} fetch --tags --force --quiet origin" in log, log
    assert f"-C {cerebra} fetch --tags --force --quiet origin" in log, log
    assert (update_dir / "available.json").is_file()
