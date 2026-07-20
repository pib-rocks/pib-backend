"""BDD scenarios from docs/test-basis/backend_safety_and_edge_cases.md (Flask layer)."""

from __future__ import annotations

import json
from unittest.mock import patch

import pytest

from default_pose_constants import CALIBRATION_POSE_NAME


class TestFlaskHttpErrorHandling:
    def test_schema_validation_failure_on_post(self, client):
        response = client.post("/program", json={})
        assert response.status_code == 400
        assert response.get_json() == {"error": "Bad request."}

    def test_entity_not_found(self, client):
        response = client.get("/motor/this_motor_does_not_exist")
        assert response.status_code == 404
        assert response.get_json()["error"] == (
            "Entity not found. Please check your path parameter."
        )

    def test_database_integrity_violation(self, client):
        client.post("/program", json={"name": "integrity_dup"})
        response = client.post("/program", json={"name": "integrity_dup"})
        assert response.status_code == 400

    @patch("service.program_service.pib_blockly_client.code_visual_to_python")
    def test_blockly_compilation_failure(self, mock_compile, client):
        mock_compile.return_value = (False, None)
        pn = client.post("/program", json={"name": "bdd_compile_fail"}).get_json()[
            "programNumber"
        ]
        response = client.put(f"/program/{pn}/code", json={"codeVisual": "{}"})
        assert response.status_code == 500
        assert "an unknown error occured." in response.get_json()["error"]

    def test_non_deletable_pose_deletion(self, client):
        cal_id = next(
            p["poseId"]
            for p in client.get("/pose").get_json()["poses"]
            if p["name"] == CALIBRATION_POSE_NAME
        )
        assert client.delete(f"/pose/{cal_id}").status_code == 500

    def test_missing_content_type_returns_500_not_400(self, client):
        response = client.open(
            "/program",
            method="POST",
            data=json.dumps({"name": "no_content_type"}),
        )
        assert response.status_code == 500


class TestPoseMotorPositionMismatch:
    def test_motor_count_mismatch_500(self, client):
        startup = client.get("/pose/by-name/Startup%2FResting").get_json()
        response = client.patch(
            f"/pose/{startup['poseId']}/motor-positions",
            json={"motorPositions": [{"position": 0, "motorName": "turn_head_motor"}]},
        )
        assert response.status_code == 500
