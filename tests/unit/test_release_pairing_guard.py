"""Release pairing guard (pib-sdk pin + cerebra follow-up)."""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
GUARD_PATH = REPO_ROOT / "setup" / "release_pairing_guard.py"
WORKFLOW_PATH = REPO_ROOT / ".github" / "workflows" / "release-pairing-guard.yml"
VERSION_FILE = REPO_ROOT / "pib_sdk_version.txt"


def _load_guard():
    spec = importlib.util.spec_from_file_location("release_pairing_guard", GUARD_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guard = _load_guard()


def _release(tag: str, **extra):
    document = {
        "tag_name": tag,
        "draft": False,
        "prerelease": False,
        "published_at": "2026-01-01T00:00:00Z",
    }
    document.update(extra)
    return document


def test_workflow_and_script_agree_on_sdk_version_file():
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    assert guard.PIB_SDK_VERSION_FILE in workflow
    assert "pib_sdk_version.txt" in workflow
    assert guard.PIB_SDK_VERSION_FILE == "pib_sdk_version.txt"
    assert VERSION_FILE.is_file()


def test_read_required_sdk_version(tmp_path):
    root = tmp_path / "repo"
    root.mkdir()
    (root / guard.PIB_SDK_VERSION_FILE).write_text("0.7.4\n", encoding="utf-8")
    assert guard.read_required_sdk_version(root) == "0.7.4"


def test_missing_version_file_is_an_explicit_error(tmp_path):
    with pytest.raises(FileNotFoundError, match=guard.PIB_SDK_VERSION_FILE):
        guard.read_required_sdk_version(tmp_path)


def test_sdk_published_passes():
    outcome, lines = guard.decide_sdk_guard(
        classification="published",
        required_version="0.7.4",
        backend_age_minutes=200,
        grace_minutes=120,
    )
    assert outcome == "OK"
    assert any("0.7.4" in line or "v0.7.4" in line for line in lines)


def test_sdk_missing_fails_after_grace_window():
    outcome, lines = guard.decide_sdk_guard(
        classification="missing",
        required_version="0.7.4",
        backend_age_minutes=120,
        grace_minutes=120,
    )
    assert outcome == "ERROR"
    assert any("pib_sdk_version.txt" in line for line in lines)


def test_sdk_missing_is_pending_inside_grace_window():
    outcome, lines = guard.decide_sdk_guard(
        classification="missing",
        required_version="0.7.4",
        backend_age_minutes=30,
        grace_minutes=120,
    )
    assert outcome == "PENDING"
    assert any(line.startswith("::notice::PENDING") for line in lines)
    assert any("pib_sdk_version.txt" in line for line in lines)


def test_sdk_draft_only_fails_even_inside_grace():
    outcome, _lines = guard.decide_sdk_guard(
        classification="draft",
        required_version="0.7.4",
        backend_age_minutes=5,
        grace_minutes=120,
    )
    assert outcome == "ERROR"


def test_sdk_prerelease_only_fails():
    outcome, _lines = guard.decide_sdk_guard(
        classification="prerelease",
        required_version="0.7.4",
        backend_age_minutes=5,
        grace_minutes=120,
    )
    assert outcome == "ERROR"


def test_classify_sdk_from_fixture_json():
    published = guard.classify_sdk_from_releases([_release("v0.7.4")], "0.7.4")
    assert published[0] == "published"

    draft = guard.classify_sdk_from_releases([_release("v0.7.4", draft=True)], "0.7.4")
    assert draft[0] == "draft"

    missing = guard.classify_sdk_from_releases([], "0.7.4")
    assert missing[0] == "missing"


def test_check_sdk_cli_with_fixture_json(tmp_path):
    releases_path = tmp_path / "sdk-releases.json"
    releases_path.write_text(json.dumps([_release("v0.7.4")]), encoding="utf-8")
    completed = subprocess.run(
        [
            sys.executable,
            str(GUARD_PATH),
            "check-sdk",
            str(releases_path),
            "0.7.4",
            "200",
            "--grace-minutes",
            "120",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0
    assert "OK:" in completed.stdout

    empty = tmp_path / "empty.json"
    empty.write_text("[]", encoding="utf-8")
    pending = subprocess.run(
        [
            sys.executable,
            str(GUARD_PATH),
            "check-sdk",
            str(empty),
            "0.7.4",
            "10",
            "--grace-minutes",
            "120",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert pending.returncode == 0
    assert "::notice::PENDING" in pending.stdout

    draft_only = tmp_path / "draft.json"
    draft_only.write_text(
        json.dumps([_release("v0.7.4", draft=True)]), encoding="utf-8"
    )
    draft_run = subprocess.run(
        [
            sys.executable,
            str(GUARD_PATH),
            "check-sdk",
            str(draft_only),
            "0.7.4",
            "10",
            "--grace-minutes",
            "120",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert draft_run.returncode == 1

    pre = tmp_path / "pre.json"
    pre.write_text(json.dumps([_release("v0.7.4", prerelease=True)]), encoding="utf-8")
    pre_run = subprocess.run(
        [
            sys.executable,
            str(GUARD_PATH),
            "check-sdk",
            str(pre),
            "0.7.4",
            "10",
            "--grace-minutes",
            "120",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert pre_run.returncode == 1


def test_cerebra_draft_stays_pending_in_grace():
    outcome, lines = guard.decide_cerebra_guard(
        classification="draft",
        tag="v0.6.1",
        backend_age_minutes=10,
        grace_minutes=120,
    )
    assert outcome == "PENDING"
    assert any("DRAFT" in line for line in lines)
