"""GET /api/version payload (PR-2081 required pib-sdk floor)."""

from __future__ import annotations

import json

from controller import version_controller
from service import revision_service


def test_get_version_includes_required_pib_sdk(app, monkeypatch) -> None:
    monkeypatch.setattr(version_controller, "read_app_version", lambda: "develop")
    monkeypatch.setattr(
        version_controller, "read_required_pib_sdk_version", lambda: "0.7.4"
    )
    monkeypatch.setattr(
        revision_service,
        "installed_revisions",
        lambda directory=None: {
            "imageVersion": "develop",
            "repositories": {
                "pib-backend": {
                    "repository": "pib-backend",
                    "gitSha": "unknown",
                    "buildTime": "unknown",
                    "channel": "unknown",
                },
                "cerebra": {
                    "repository": "cerebra",
                    "gitSha": "unknown",
                    "buildTime": "unknown",
                    "channel": "unknown",
                },
            },
        },
    )

    response = app.test_client().get("/api/version")
    assert response.status_code == 200
    payload = json.loads(response.data)
    assert payload["version"] == "develop"
    assert payload["requiredPibSdkVersion"] == "0.7.4"
    assert set(payload["repositories"]) == {"pib-backend", "cerebra"}
