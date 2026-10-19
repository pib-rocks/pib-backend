"""A workspace the Blockly server refuses answers 400 with its reason (PR-1977).

Measured on the robot: a workspace still using the removed ``get_detection_field`` block
made the pib-blockly-server answer ``400 failed to compile visual-code.``, and the API
turned that into ``500 {"error": "an unknown error occured."}``.
"""

import json
import os
from unittest.mock import MagicMock, patch

import pib_blockly_client
import requests

SERVER_REFUSAL = "failed to compile visual-code."

UNKNOWN_BLOCK_WORKSPACE = json.dumps(
    {
        "blocks": {
            "languageVersion": 0,
            "blocks": [{"type": "get_detection_field", "fields": {"FIELD": "label"}}],
        }
    }
)


def test_refused_compile_answers_bad_request_with_the_servers_reason(app):
    with app.test_client() as client:
        program_number = client.post(
            "/program", json={"name": "compile refusal"}
        ).get_json()["programNumber"]
        with patch(
            "service.program_service.pib_blockly_client.code_visual_to_python",
            return_value=(False, SERVER_REFUSAL),
        ) as compile_call:
            response = client.put(
                f"/program/{program_number}/code",
                json={"codeVisual": UNKNOWN_BLOCK_WORKSPACE},
            )

    compile_call.assert_called_once_with(UNKNOWN_BLOCK_WORKSPACE)
    assert response.status_code == 400, response.get_data(as_text=True)
    assert SERVER_REFUSAL in response.get_json()["error"]
    code_file = os.path.join(app.config["PYTHON_CODE_DIR"], f"{program_number}.py")
    with open(code_file, encoding="utf-8") as f:
        assert f.read() == ""


def test_client_returns_the_servers_refusal_message():
    refusal = MagicMock(status_code=400, text=SERVER_REFUSAL)
    refusal.raise_for_status.side_effect = requests.HTTPError(response=refusal)

    with patch.object(pib_blockly_client.requests, "request", return_value=refusal):
        result = pib_blockly_client.code_visual_to_python(UNKNOWN_BLOCK_WORKSPACE)

    assert result == (False, SERVER_REFUSAL)
