import os
import pytest
import requests

FLASK_BASE_URL = os.getenv("FLASK_BASE_URL", "http://localhost:5000")


@pytest.fixture
def api_base_url():
    return FLASK_BASE_URL


class TestSTTConfigurationAPI:
    """Test Personality STT engine configuration persistence and REST API."""

    def test_personality_default_stt_engine(self, api_base_url):
        r = requests.get(f"{api_base_url}/voice-assistant/personality")
        assert r.status_code == 200
        data = r.json()
        personalities = data.get("voiceAssistantPersonalities") or data.get("personalities") or []
        assert len(personalities) > 0

        first_p = personalities[0]
        p_id = first_p.get("personalityId") or first_p.get("personality_id") or first_p.get("personalityNumber")
        assert p_id is not None

        # Fetch detail
        r_detail = requests.get(f"{api_base_url}/voice-assistant/personality/{p_id}")
        assert r_detail.status_code == 200
        p_detail = r_detail.json()
        assert p_detail.get("sttEngine") in ["local_whisper", "tryb_api"]

    def test_update_stt_engine_setting(self, api_base_url):
        r = requests.get(f"{api_base_url}/voice-assistant/personality")
        assert r.status_code == 200
        data = r.json()
        personalities = data.get("voiceAssistantPersonalities") or data.get("personalities") or []
        first_p = personalities[0]
        p_id = first_p.get("personalityId") or first_p.get("personality_id") or first_p.get("personalityNumber")

        # Switch to tryb_api
        payload_tryb = {"sttEngine": "tryb_api"}
        r_put = requests.put(f"{api_base_url}/voice-assistant/personality/{p_id}", json=payload_tryb)
        assert r_put.status_code in [200, 204]

        # Verify detail
        r_get = requests.get(f"{api_base_url}/voice-assistant/personality/{p_id}")
        assert r_get.status_code == 200
        assert r_get.json().get("sttEngine") == "tryb_api"

        # Switch back to local_whisper
        payload_local = {"sttEngine": "local_whisper"}
        r_put_back = requests.put(f"{api_base_url}/voice-assistant/personality/{p_id}", json=payload_local)
        assert r_put_back.status_code in [200, 204]

        # Verify detail
        r_get_back = requests.get(f"{api_base_url}/voice-assistant/personality/{p_id}")
        assert r_get_back.status_code == 200
        assert r_get_back.json().get("sttEngine") == "local_whisper"
