"""versionProvenance on installed revision facts (PR-2111)."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from service import revision_service
from service.version_service import is_release_tag, version_provenance


def _init_git_repo(path: Path, *, branch: str = "develop") -> None:
    subprocess.run(
        ["git", "init", "-b", branch, str(path)],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        ["git", "-C", str(path), "config", "user.email", "test@example.com"],
        check=True,
        capture_output=True,
    )
    subprocess.run(
        ["git", "-C", str(path), "config", "user.name", "test"],
        check=True,
        capture_output=True,
    )
    (path / "marker").write_text("x", encoding="utf-8")
    subprocess.run(
        ["git", "-C", str(path), "add", "marker"],
        check=True,
        capture_output=True,
    )
    subprocess.run(
        ["git", "-C", str(path), "commit", "-m", "init"],
        check=True,
        capture_output=True,
    )


def _head_sha(path: Path) -> str:
    return subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


@pytest.mark.parametrize(
    ("version", "expected"),
    [
        ("v0.7.4", True),
        ("dev", False),
        ("v0.7.3-43-g6a1f91d4", False),
        ("unknown", False),
        ("", False),
    ],
)
def test_is_release_tag(version: str, expected: bool) -> None:
    assert is_release_tag(version) is expected


@pytest.mark.parametrize(
    "version",
    ["dev", "v0.7.3-43-g6a1f91d4", "unknown", ""],
)
def test_version_provenance_warning_when_not_a_release_tag(version: str) -> None:
    provenance = version_provenance(version)
    assert provenance["isReleaseTag"] is False
    assert provenance["warning"]
    assert "release tag" in provenance["warning"].lower()


def test_version_provenance_no_warning_for_release_tag() -> None:
    provenance = version_provenance("v0.7.4")
    assert provenance["isReleaseTag"] is True
    assert provenance["warning"] is None
    assert provenance["imageVersion"] == "v0.7.4"


def test_installed_revisions_keeps_existing_keys_and_adds_provenance(
    tmp_path, monkeypatch
) -> None:
    (tmp_path / "pib-backend.revision.json").write_text(
        '{"gitSha":"abc","buildTime":"now","channel":"release"}',
        encoding="utf-8",
    )
    monkeypatch.setattr(revision_service, "read_app_version", lambda: "v0.7.4")

    revisions = revision_service.installed_revisions(tmp_path)

    assert set(revisions) == {"imageVersion", "versionProvenance", "repositories"}
    assert revisions["imageVersion"] == "v0.7.4"
    assert revisions["repositories"]["pib-backend"]["gitSha"] == "abc"
    assert revisions["repositories"]["cerebra"]["gitSha"] == "unknown"
    assert revisions["versionProvenance"] == {
        "imageVersion": "v0.7.4",
        "isReleaseTag": True,
        "warning": None,
    }


@pytest.mark.parametrize(
    ("branch", "expected_channel"),
    [
        ("develop", "develop"),
        ("PR-2272", "develop"),
        ("main", "release"),
    ],
)
def test_git_revision_fallback_derives_channel_from_branch(
    tmp_path: Path, branch: str, expected_channel: str
) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    _init_git_repo(repo, branch=branch)

    fallback = revision_service._git_revision_fallback(
        "pib-backend", {"pib-backend": repo}
    )

    assert fallback is not None
    assert fallback["gitSha"] == _head_sha(repo)
    assert fallback["channel"] == expected_channel
    assert fallback["buildTime"] != revision_service.UNKNOWN


def test_git_revision_fallback_release_tag_on_detached_head(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    _init_git_repo(repo, branch="main")
    subprocess.run(
        ["git", "-C", str(repo), "tag", "v1.2.3"],
        check=True,
        capture_output=True,
    )
    subprocess.run(
        ["git", "-C", str(repo), "checkout", "v1.2.3"],
        check=True,
        capture_output=True,
    )

    fallback = revision_service._git_revision_fallback("cerebra", {"cerebra": repo})

    assert fallback is not None
    assert fallback["channel"] == "release"


def test_installed_revisions_fills_from_git_when_json_missing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    backend = tmp_path / "backend"
    cerebra = tmp_path / "cerebra"
    backend.mkdir()
    cerebra.mkdir()
    _init_git_repo(backend, branch="develop")
    _init_git_repo(cerebra, branch="main")

    update_dir = tmp_path / "update"
    update_dir.mkdir()
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    monkeypatch.setattr(revision_service, "read_app_version", lambda: "dev")

    revisions = revision_service.installed_revisions(
        update_dir,
        checkout_dirs={"pib-backend": backend, "cerebra": cerebra},
    )

    assert revisions["repositories"]["pib-backend"]["gitSha"] == _head_sha(backend)
    assert revisions["repositories"]["pib-backend"]["channel"] == "develop"
    assert revisions["repositories"]["cerebra"]["gitSha"] == _head_sha(cerebra)
    assert revisions["repositories"]["cerebra"]["channel"] == "release"


def test_installed_revisions_partial_json_merges_git_fallback(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    backend = tmp_path / "backend"
    backend.mkdir()
    _init_git_repo(backend, branch="PR-2272")

    update_dir = tmp_path / "update"
    update_dir.mkdir()
    (update_dir / "pib-backend.revision.json").write_text(
        '{"gitSha":"from-file","buildTime":"unknown","channel":"unknown"}',
        encoding="utf-8",
    )

    revisions = revision_service.installed_revisions(
        update_dir,
        checkout_dirs={"pib-backend": backend, "cerebra": backend},
    )

    repo = revisions["repositories"]["pib-backend"]
    assert repo["gitSha"] == "from-file"
    assert repo["buildTime"] != revision_service.UNKNOWN
    assert repo["channel"] == "develop"


def test_installed_revisions_tmp_path_without_checkout_dirs_skips_git(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(revision_service, "read_app_version", lambda: "dev")

    revisions = revision_service.installed_revisions(tmp_path)

    assert (
        revisions["repositories"]["pib-backend"]["gitSha"] == revision_service.UNKNOWN
    )
    assert revisions["repositories"]["cerebra"]["gitSha"] == revision_service.UNKNOWN
