import json
import os
import uuid
import pytest
import requests

FLASK_BASE_URL = os.getenv("FLASK_BASE_URL", "http://localhost:5000")


@pytest.fixture
def api_base_url():
    return FLASK_BASE_URL


class TestProgramCRUDAndExecution:
    """Integration test suite covering Blockly program CRUD lifecycle and code compilation contract."""

    def test_program_crud_lifecycle(self, api_base_url):
        # 1. Create a new program
        unique_name = f"Test_Prog_{uuid.uuid4().hex[:8]}"
        create_payload = {"name": unique_name}

        r_create = requests.post(f"{api_base_url}/program", json=create_payload)
        assert r_create.status_code in (200, 201)
        data = r_create.json()
        assert "programNumber" in data
        assert data["name"] == unique_name
        prog_id = data["programNumber"]

        try:
            # 2. Retrieve program by ID
            r_get = requests.get(f"{api_base_url}/program/{prog_id}")
            assert r_get.status_code == 200
            assert r_get.json()["programNumber"] == prog_id

            # 3. Retrieve default code (should be valid JSON or empty dict string)
            r_code_get = requests.get(f"{api_base_url}/program/{prog_id}/code")
            assert r_code_get.status_code == 200
            assert "codeVisual" in r_code_get.json()

            # 4. Update program visual code (Hello World print block)
            hello_world_visual = json.dumps({
                "blocks": {
                    "languageVersion": 0,
                    "blocks": [
                        {
                            "type": "text_print",
                            "id": "print_block_001",
                            "x": 100,
                            "y": 100,
                            "inputs": {
                                "TEXT": {
                                    "shadow": {
                                        "type": "text",
                                        "id": "text_block_001",
                                        "fields": {"TEXT": "Hello World Automated Test"}
                                    }
                                }
                            }
                        }
                    ]
                }
            })

            put_code_payload = {"codeVisual": hello_world_visual}
            r_code_put = requests.put(f"{api_base_url}/program/{prog_id}/code", json=put_code_payload)
            assert r_code_put.status_code == 200
            assert "codeVisual" in r_code_put.json()

            # 5. Verify updated code via GET
            r_code_verify = requests.get(f"{api_base_url}/program/{prog_id}/code")
            assert r_code_verify.status_code == 200
            saved_visual = r_code_verify.json()["codeVisual"]
            assert "Hello World Automated Test" in saved_visual

            # 6. List all programs and confirm unique_name is present
            r_list = requests.get(f"{api_base_url}/program")
            assert r_list.status_code == 200
            programs = r_list.json().get("programs", [])
            program_ids = [p["programNumber"] for p in programs]
            assert prog_id in program_ids

        finally:
            # 7. Clean up / Delete program
            r_del = requests.delete(f"{api_base_url}/program/{prog_id}")
            assert r_del.status_code in (200, 204)

            # Confirm 404 after deletion
            r_get_deleted = requests.get(f"{api_base_url}/program/{prog_id}")
            assert r_get_deleted.status_code == 404

    def test_program_duplicate_name_handling(self, api_base_url):
        unique_name = f"Dup_Prog_{uuid.uuid4().hex[:8]}"
        r1 = requests.post(f"{api_base_url}/program", json={"name": unique_name})
        assert r1.status_code in (200, 201)
        prog_id1 = r1.json()["programNumber"]

        try:
            # Attempt duplicate creation
            r2 = requests.post(f"{api_base_url}/program", json={"name": unique_name})
            assert r2.status_code == 400
        finally:
            requests.delete(f"{api_base_url}/program/{prog_id1}")
