"""Published release pairing. Versions here are fixtures, not product constants."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[2] / "setup" / "update_releases.py"
SHA_A = "a" * 40
SHA_B = "b" * 40
SHA_C = "c" * 40
SHA_D = "d" * 40


def _load():
    spec = importlib.util.spec_from_file_location("update_releases", MODULE_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


releases = _load()


def _release(tag, **extra):
    document = {
        "tag_name": tag,
        "draft": False,
        "prerelease": False,
        "body": f"notes for {tag}",
        "published_at": "2026-01-01T00:00:00Z",
    }
    document.update(extra)
    return document


def _commits(mapping):
    return {
        releases.BACKEND_REPO: mapping["pib-backend"],
        releases.CEREBRA_REPO: mapping["cerebra"],
    }


def test_complete_stable_pair_is_installable_and_drafts_are_not():
    paired = releases.pair_releases(
        [
            _release("v1.2.3"),
            _release("v1.2.4", draft=True),
            _release("v1.3.0-rc.1", prerelease=True),
            _release("v9.9.9", prerelease=True),
            _release("models-2026.10.08"),
        ],
        [
            _release("v1.2.3", body="cerebra notes"),
            _release("v1.2.4", draft=True),
            _release("v1.3.0-rc.1", prerelease=True),
            _release("v9.9.9", prerelease=True),
            _release("models-2026.10.08"),
        ],
        _commits(
            {
                "pib-backend": {"v1.2.3": SHA_A, "v1.2.4": SHA_C},
                "cerebra": {"v1.2.3": SHA_B, "v1.2.4": SHA_D},
            }
        ),
    )

    assert paired["latestInstallable"] == "v1.2.3"
    assert paired["releases"] == [
        {
            "tag": "v1.2.3",
            "installable": True,
            "notes": "notes for v1.2.3",
            "publishedAt": "2026-01-01T00:00:00Z",
            "targets": {
                "pib-backend": {"commit": SHA_A, "tag": "v1.2.3"},
                "cerebra": {"commit": SHA_B, "tag": "v1.2.3"},
            },
        }
    ]
    assert any(item["tag"] == "v1.2.4" and item["installable"] is False for item in paired["incomplete"])
    assert {item["tag"] for item in paired["excluded"]} >= {
        "v1.3.0-rc.1",
        "v9.9.9",
        "models-2026.10.08",
    }
    assert any(item["tag"] == "v9.9.9" and "prerelease" in item["reason"] for item in paired["excluded"])
    assert all("prerelease" in item["reason"] or "not a stable" in item["reason"] for item in paired["excluded"])


def test_incomplete_pair_and_unresolved_commit_are_not_installable():
    paired = releases.pair_releases(
        [_release("v2.0.0"), _release("v2.0.1")],
        [_release("v2.0.1")],
        _commits(
            {
                "pib-backend": {"v2.0.0": SHA_A, "v2.0.1": None},
                "cerebra": {"v2.0.0": SHA_B, "v2.0.1": SHA_D},
            }
        ),
    )

    by_tag = {item["tag"]: item for item in paired["incomplete"]}
    assert by_tag["v2.0.0"]["missing"] == ["cerebra"]
    assert "only one" in by_tag["v2.0.0"]["reason"]
    assert by_tag["v2.0.1"]["installable"] is False
    assert "commit" in by_tag["v2.0.1"]["reason"]
    assert paired["latestInstallable"] is None


def test_relations_cover_newer_current_older_drift_unknown_and_channel_change():
    document = releases.pair_releases(
        [_release("v1.2.0"), _release("v1.3.0"), _release("v1.4.0")],
        [_release("v1.2.0"), _release("v1.3.0"), _release("v1.4.0")],
        _commits(
            {
                "pib-backend": {"v1.2.0": SHA_A, "v1.3.0": SHA_C, "v1.4.0": "e" * 40},
                "cerebra": {"v1.2.0": SHA_B, "v1.3.0": SHA_D, "v1.4.0": "f" * 40},
            }
        ),
    )
    installed = {
        "imageVersion": "v1.3.0",
        "repositories": {
            "pib-backend": {"gitSha": SHA_C, "channel": "release"},
            "cerebra": {"gitSha": SHA_D, "channel": "release"},
        },
    }
    annotated = releases.annotate_relations(document, installed)
    relations = {item["tag"]: item["relation"] for item in annotated["releases"]}
    assert relations == {"v1.2.0": "older", "v1.3.0": "current", "v1.4.0": "newer"}
    assert annotated["recommendation"]["ordinaryUpdate"] is True
    assert annotated["recommendation"]["tag"] == "v1.4.0"
    assert annotated["deviceChannel"] == "release"

    drift = releases.annotate_relations(
        document,
        {
            "imageVersion": "v1.3.0",
            "repositories": {
                "pib-backend": {"gitSha": SHA_A, "channel": "release"},
                "cerebra": {"gitSha": SHA_B, "channel": "release"},
            },
        },
    )
    assert {item["tag"]: item["relation"] for item in drift["releases"]}["v1.3.0"] == "drift"

    unknown = releases.annotate_relations(
        document,
        {
            "imageVersion": "unknown",
            "repositories": {
                "pib-backend": {"gitSha": "unknown", "channel": "unknown"},
                "cerebra": {"gitSha": "unknown", "channel": "unknown"},
            },
        },
    )
    assert unknown["recommendation"]["relation"] == "unknown"
    assert unknown["recommendation"]["ordinaryUpdate"] is False
    assert unknown["deviceChannel"] == "unknown"

    develop = releases.annotate_relations(
        document,
        {
            "imageVersion": "develop",
            "repositories": {
                "pib-backend": {"gitSha": SHA_A, "channel": "develop"},
                "cerebra": {"gitSha": SHA_B, "channel": "develop"},
            },
        },
    )
    assert develop["recommendation"]["relation"] == "channel-change"
    assert develop["recommendation"]["ordinaryUpdate"] is False
    assert develop["deviceChannel"] == "develop"


def test_release_channel_sha_difference_is_not_a_newer_release_by_itself():
    document = {
        "channel": "release",
        "latestInstallable": "v1.0.0",
        "releases": [
            {
                "tag": "v1.0.0",
                "installable": True,
                "notes": "",
                "targets": {
                    "pib-backend": {"commit": SHA_A, "tag": "v1.0.0"},
                    "cerebra": {"commit": SHA_B, "tag": "v1.0.0"},
                },
            }
        ],
        "repositories": {
            "pib-backend": {
                "installed": SHA_A,
                "target": SHA_C,
                "branchTarget": SHA_C,
                "updateAvailable": True,
            }
        },
    }
    annotated = releases.annotate_relations(
        document,
        {
            "imageVersion": "v1.0.0",
            "repositories": {
                "pib-backend": {"gitSha": SHA_A, "channel": "release"},
                "cerebra": {"gitSha": SHA_B, "channel": "release"},
            },
        },
    )
    assert annotated["repositories"]["pib-backend"]["updateAvailable"] is False
    assert annotated["recommendation"]["ordinaryUpdate"] is False


def test_discover_uses_only_the_two_known_repositories():
    seen = []

    def transport(url: str):
        seen.append(url)
        if url.endswith("/commits/v1.2.3"):
            repo = "pib-backend" if "pib-backend" in url else "cerebra"
            sha = SHA_A if repo == "pib-backend" else SHA_B
            return 200, {"sha": sha}
        if "pib-rocks/pib-backend/releases" in url or "pib-rocks/cerebra/releases" in url:
            return 200, [_release("v1.2.3")]
        raise AssertionError(url)

    discovered = releases.discover(transport)
    assert discovered["latestInstallable"] == "v1.2.3"
    assert discovered["error"] is None
    assert all(url.startswith("https://api.github.com/repos/pib-rocks/") for url in seen)
    assert not any("model" in url for url in seen)

    with pytest.raises(ValueError, match="unsupported repository"):
        releases.list_releases("pib-rocks/model-registry", transport)
    with pytest.raises(ValueError, match="not a stable"):
        releases.resolve_commit(releases.BACKEND_REPO, "not-a-release", transport)


def test_discover_reports_a_transport_failure_without_inventing_a_release():
    def transport(url: str):
        raise OSError("offline")

    discovered = releases.discover(transport)
    assert discovered["releases"] == []
    assert discovered["latestInstallable"] is None
    assert "offline" in discovered["error"]


def test_changed_commit_map_does_not_rewrite_an_already_paired_document():
    commits = _commits(
        {
            "pib-backend": {"v1.2.3": SHA_A},
            "cerebra": {"v1.2.3": SHA_B},
        }
    )
    paired = releases.pair_releases([_release("v1.2.3")], [_release("v1.2.3")], commits)
    commits[releases.BACKEND_REPO]["v1.2.3"] = SHA_C
    assert paired["releases"][0]["targets"]["pib-backend"]["commit"] == SHA_A
    assert json.dumps(paired)
