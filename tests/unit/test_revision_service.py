"""versionProvenance on installed revision facts (PR-2111)."""

from __future__ import annotations

import pytest

from service import revision_service
from service.version_service import is_release_tag, version_provenance


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
