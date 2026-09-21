from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[2] / "setup" / "update_check.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("update_check", MODULE_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


update_check = _load_module()
SHA_A = "a" * 40
SHA_B = "b" * 40


def test_compare_revisions_reports_same_changed_and_sorted_repositories():
    result = update_check.compare_revisions(
        {"pib-backend": SHA_A, "cerebra": SHA_A},
        {"pib-backend": SHA_B, "cerebra": SHA_A},
    )

    assert result == {
        "cerebra": {
            "installed": SHA_A,
            "target": SHA_A,
            "updateAvailable": False,
        },
        "pib-backend": {
            "installed": SHA_A,
            "target": SHA_B,
            "updateAvailable": True,
        },
    }


@pytest.mark.parametrize(
    ("installed", "target"),
    [
        (None, SHA_A),
        (SHA_A, None),
        ("unknown", SHA_A),
        (SHA_A, ""),
        ("not-a-sha", SHA_A),
        (True, SHA_A),
    ],
)
def test_compare_revisions_never_guesses_for_unknown_or_unparsable_values(
    installed, target
):
    result = update_check.compare_revisions(
        {"pib-backend": installed}, {"pib-backend": target}
    )

    assert result["pib-backend"]["updateAvailable"] == "unknown"


def test_compare_revisions_accepts_sha256_and_normalizes_case():
    result = update_check.compare_revisions(
        {"cerebra": "A" * 64}, {"cerebra": "a" * 64}
    )

    assert result["cerebra"]["installed"] == "a" * 64
    assert result["cerebra"]["updateAvailable"] is False


def test_build_document_preserves_remote_error():
    document = update_check.build_document(
        {"cerebra": SHA_A},
        {"cerebra": {"target": "unknown", "error": "network unavailable"}},
        "2026-09-21T12:00:00+00:00",
    )

    assert document["checkedAt"] == "2026-09-21T12:00:00+00:00"
    assert document["repositories"]["cerebra"] == {
        "installed": SHA_A,
        "target": "unknown",
        "updateAvailable": "unknown",
        "error": "network unavailable",
    }


def test_validate_request_requires_and_normalizes_every_field():
    request = update_check.validate_request(
        {
            "schemaVersion": 1,
            "actor": " 127.0.0.1 ",
            "channel": "develop",
            "requestedAt": " 2026-09-21T12:00:00+00:00 ",
        }
    )

    assert request == {
        "schemaVersion": 1,
        "actor": "127.0.0.1",
        "channel": "develop",
        "requestedAt": "2026-09-21T12:00:00+00:00",
    }


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("schemaVersion", None),
        ("schemaVersion", 2),
        ("actor", ""),
        ("channel", "nightly"),
        ("requestedAt", False),
    ],
)
def test_validate_request_names_missing_and_invalid_fields(field, value):
    document = {
        "schemaVersion": 1,
        "actor": "test",
        "channel": "release",
        "requestedAt": "now",
    }
    if value is None:
        del document[field]
    else:
        document[field] = value

    with pytest.raises(ValueError, match=field):
        update_check.validate_request(document)


def test_write_cli_replaces_json_with_group_writable_file(tmp_path):
    destination = tmp_path / "available.json"

    result = update_check.main(
        [
            "write",
            str(destination),
            "2026-09-21T12:00:00+00:00",
            "pib-backend",
            SHA_A,
            SHA_B,
            "",
            "cerebra",
            SHA_A,
            "unknown",
            "offline",
        ]
    )

    assert result == 0
    assert json.loads(destination.read_text(encoding="utf-8"))["schemaVersion"] == 1
    assert destination.stat().st_mode & 0o777 == 0o660
    assert not list(tmp_path.glob(".available.json.*"))
