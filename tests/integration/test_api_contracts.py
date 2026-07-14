"""API contract tests — docs/test-basis/api_contracts.md"""

from __future__ import annotations

import json
import uuid
from typing import Any
from unittest.mock import patch

import pytest

from default_pose_constants import CALIBRATION_POSE_NAME, STARTUP_POSE_NAME


def _error(response) -> dict[str, Any]:
    return response.get_json()


class TestErrorEnvelope:
    @pytest.mark.parametrize(
        "method,path,body,content_type,expected_status",
        [
            ("post", "/program", "{}", "application/json", 400),
            ("post", "/program", "not-json", "application/json", 400),
            ("get", "/program/missing-00000000-0000-0000-0000-000000000099", None, None, 404),
            ("get", "/motor/no_such_motor_xyz", None, None, 404),
        ],
    )
    def test_error_envelope(self, client, method, path, body, content_type, expected_status):
        if body is None:
            response = client.open(path, method=method.upper())
        else:
            response = client.open(
                path,
                method=method.upper(),
                data=body if body == "not-json" else json.dumps({}),
                content_type=content_type,
            )
        assert response.status_code == expected_status
        assert "error" in _error(response)


class TestProgramEndpoints:
    def test_create_program_201(self, client, programs_dir):
        response = client.post("/program", json={"name": "contract_test_program"})
        assert response.status_code == 201
        body = response.get_json()
        assert body["name"] == "contract_test_program"
        assert (programs_dir / f"{body['programNumber']}.py").exists()

    def test_list_and_get_program(self, client):
        created = client.post("/program", json={"name": "list_probe"}).get_json()
        listing = client.get("/program")
        assert listing.status_code == 200
        ids = [p["programNumber"] for p in listing.get_json()["programs"]]
        assert created["programNumber"] in ids
        fetched = client.get(f"/program/{created['programNumber']}")
        assert fetched.status_code == 200

    def test_update_and_delete_program(self, client, programs_dir):
        created = client.post("/program", json={"name": "delete_probe"}).get_json()
        pn = created["programNumber"]
        updated = client.put(f"/program/{pn}", json={"name": "delete_probe_renamed"})
        assert updated.status_code == 200
        deleted = client.delete(f"/program/{pn}")
        assert deleted.status_code == 204
        assert not (programs_dir / f"{pn}.py").exists()

    @patch("service.program_service.pib_blockly_client.code_visual_to_python")
    def test_put_code_compilation_failure_500(self, mock_compile, client):
        mock_compile.return_value = (False, None)
        pn = client.post("/program", json={"name": "compile_fail"}).get_json()["programNumber"]
        response = client.put(f"/program/{pn}/code", json={"codeVisual": "{}"})
        assert response.status_code == 500
        assert _error(response)["error"] == "an unknown error occured."

    def test_duplicate_program_name_400(self, client):
        client.post("/program", json={"name": "dup_name_test"})
        response = client.post("/program", json={"name": "dup_name_test"})
        assert response.status_code == 400


class TestMotorEndpoints:
    def test_get_motors_camel_case(self, client):
        motors = client.get("/motor").get_json()["motors"]
        assert motors
        assert "rotationRangeMin" in motors[0]
        assert "brickletPins" in motors[0]

    @pytest.mark.parametrize(
        "motor_name,min_val,max_val",
        [
            ("tilt_forward_motor", -4500, 4500),
            ("turn_head_motor", -9000, 9000),
        ],
    )
    def test_rotation_seed_boundaries(self, client, motor_name, min_val, max_val):
        body = client.get(f"/motor/{motor_name}").get_json()
        assert body["rotationRangeMin"] == min_val
        assert body["rotationRangeMax"] == max_val

    def test_update_motor_settings_roundtrip(self, client):
        original = client.get("/motor/turn_head_motor/settings").get_json()
        payload = {**original, "velocity": original["velocity"] + 1}
        updated = client.put("/motor/turn_head_motor/settings", json=payload)
        assert updated.status_code == 200
        client.put("/motor/turn_head_motor/settings", json=original)


class TestPoseEndpoints:
    def test_startup_pose_by_name(self, client):
        response = client.get(f"/pose/by-name/{STARTUP_POSE_NAME}")
        assert response.status_code == 200
        body = response.get_json()
        assert body["name"] == STARTUP_POSE_NAME
        assert body["deletable"] is False

    def test_delete_calibration_pose_500(self, client):
        poses = client.get("/pose").get_json()["poses"]
        cal_id = next(p["poseId"] for p in poses if p["name"] == CALIBRATION_POSE_NAME)
        assert client.delete(f"/pose/{cal_id}").status_code == 500

    def test_create_pose_201(self, client):
        name = f"pose_{uuid.uuid4().hex[:8]}"
        payload = {
            "name": name,
            "motorPositions": [{"position": 0, "motorName": "turn_head_motor"}],
        }
        response = client.post("/pose", json=payload)
        assert response.status_code == 201


class TestBrickletAndButtonPrograms:
    def test_get_bricklets(self, client):
        bricklets = client.get("/bricklet").get_json()["bricklets"]
        assert any(b["type"] == "RGB LED Button Bricklet" for b in bricklets)

    def test_put_button_program_mapping(self, client):
        pn = client.post("/program", json={"name": "btn_map_prog"}).get_json()["programNumber"]
        response = client.put(
            "/button-programs",
            json={"buttonProgramUpdates": [{"brickletNumber": 5, "programNumber": pn}]},
        )
        assert response.status_code == 200
        client.put(
            "/button-programs",
            json={"buttonProgramUpdates": [{"brickletNumber": 5, "programNumber": None}]},
        )


class TestCameraAndHostIp:
    def test_camera_settings_seed(self, client):
        body = client.get("/camera-settings").get_json()
        assert body["resolution"] == "SD"
        assert body["qualityFactor"] == 80

    def test_host_ip_200(self, client):
        assert client.get("/host-ip").get_json()["host_ip"] == "192.168.1.100"

    def test_host_ip_missing_file_500(self, client, app, tmp_path):
        app.config["HOST_IP_FILE"] = str(tmp_path / "missing.txt")
        assert client.get("/host-ip").status_code == 500


class TestVoiceAssistant:
    def test_personality_lifecycle(self, client):
        model_id = client.get("/assistant-model").get_json()["assistantModels"][0]["id"]
        created = client.post(
            "/voice-assistant/personality",
            json={
                "name": "PytestPersonality",
                "gender": "Female",
                "pauseThreshold": 0.8,
                "messageHistory": 5,
                "assistantModelId": model_id,
            },
        )
        assert created.status_code == 201
        pid = created.get_json()["personalityId"]
        assert client.delete(f"/voice-assistant/personality/{pid}").status_code == 204

    def test_chat_message_create(self, client):
        chat_id = client.get("/voice-assistant/chat").get_json()["voiceAssistantChats"][0]["chatId"]
        msg = client.post(
            f"/voice-assistant/chat/{chat_id}/messages",
            json={"isUser": True, "content": "contract test"},
        )
        assert msg.status_code == 201
        mid = msg.get_json()["messageId"]
        client.delete(f"/voice-assistant/chat/{chat_id}/messages/{mid}")
