import os
import pytest
import requests

FLASK_BASE_URL = os.getenv("FLASK_BASE_URL", "http://localhost:5000")


class _ResponseWrapper:
    def __init__(self, res):
        self._res = res
        self.status_code = res.status_code

    def json(self):
        return self._res.get_json()

    def get_json(self):
        return self._res.get_json()


class _ClientWrapper:
    def __init__(self, client):
        self._c = client

    def _url(self, path):
        if path.startswith("http://") or path.startswith("https://"):
            return path.split("http://localhost:5000")[-1].split("http://127.0.0.1:5000")[-1]
        return path

    def get(self, url, **kwargs):
        kwargs.pop("timeout", None)
        return _ResponseWrapper(self._c.get(self._url(url), **kwargs))

    def put(self, url, **kwargs):
        kwargs.pop("timeout", None)
        return _ResponseWrapper(self._c.put(self._url(url), **kwargs))


@pytest.fixture
def http_client(client):
    try:
        r = requests.get(f"{FLASK_BASE_URL}/voice-assistant/personality", timeout=1)
        if r.status_code < 500:
            return requests
    except Exception:
        pass
    return _ClientWrapper(client)


class TestSTTConfigurationAPI:
    """Test Personality STT engine configuration persistence and REST API."""

    def test_personality_default_stt_engine(self, http_client):
        r = http_client.get(f"{FLASK_BASE_URL}/voice-assistant/personality")
        assert r.status_code == 200
        data = r.json()
        personalities = data.get("voiceAssistantPersonalities") or data.get("personalities") or []
        assert len(personalities) > 0

        first_p = personalities[0]
        p_id = first_p.get("personalityId") or first_p.get("personality_id") or first_p.get("personalityNumber")
        assert p_id is not None

        # Fetch detail
        r_detail = http_client.get(f"{FLASK_BASE_URL}/voice-assistant/personality/{p_id}")
        assert r_detail.status_code == 200
        p_detail = r_detail.json()
        assert p_detail.get("sttEngine") in ["local_whisper", "tryb_api"]

    def test_update_stt_engine_setting(self, http_client):
        r = http_client.get(f"{FLASK_BASE_URL}/voice-assistant/personality")
        assert r.status_code == 200
        data = r.json()
        personalities = data.get("voiceAssistantPersonalities") or data.get("personalities") or []
        first_p = personalities[0]
        p_id = first_p.get("personalityId") or first_p.get("personality_id") or first_p.get("personalityNumber")

        # Switch to tryb_api
        payload_tryb = {"sttEngine": "tryb_api"}
        r_put = http_client.put(f"{FLASK_BASE_URL}/voice-assistant/personality/{p_id}", json=payload_tryb)
        assert r_put.status_code in [200, 204]

        # Verify detail
        r_get = http_client.get(f"{FLASK_BASE_URL}/voice-assistant/personality/{p_id}")
        assert r_get.status_code == 200
        assert r_get.json().get("sttEngine") == "tryb_api"

        # Switch back to local_whisper
        payload_local = {"sttEngine": "local_whisper"}
        r_put_back = http_client.put(f"{FLASK_BASE_URL}/voice-assistant/personality/{p_id}", json=payload_local)
        assert r_put_back.status_code in [200, 204]

        # Verify detail
        r_get_back = http_client.get(f"{FLASK_BASE_URL}/voice-assistant/personality/{p_id}")
        assert r_get_back.status_code == 200
        assert r_get_back.json().get("sttEngine") == "local_whisper"
