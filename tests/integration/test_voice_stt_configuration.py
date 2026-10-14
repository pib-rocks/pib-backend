class TestSTTConfigurationAPI:
    """Test Personality STT engine configuration persistence and REST API."""

    def test_personality_default_stt_engine(self, client):
        r = client.get("/voice-assistant/personality")
        assert r.status_code == 200
        data = r.get_json()
        personalities = (
            data.get("voiceAssistantPersonalities") or data.get("personalities") or []
        )
        assert len(personalities) > 0

        first_p = personalities[0]
        p_id = (
            first_p.get("personalityId")
            or first_p.get("personality_id")
            or first_p.get("personalityNumber")
        )
        assert p_id is not None

        # Fetch detail
        r_detail = client.get(f"/voice-assistant/personality/{p_id}")
        assert r_detail.status_code == 200
        p_detail = r_detail.get_json()
        assert p_detail.get("sttEngine") == "local_whisper"
        assert p_detail.get("ttsEngine") == "supertone"

    def test_update_stt_engine_setting(self, client):
        r = client.get("/voice-assistant/personality")
        assert r.status_code == 200
        data = r.get_json()
        personalities = (
            data.get("voiceAssistantPersonalities") or data.get("personalities") or []
        )
        first_p = personalities[0]
        p_id = (
            first_p.get("personalityId")
            or first_p.get("personality_id")
            or first_p.get("personalityNumber")
        )

        # A name that is not the local engine and not a capable provider is refused.
        payload_named = {"sttEngine": "tryb_api"}
        r_put = client.put(f"/voice-assistant/personality/{p_id}", json=payload_named)
        assert r_put.status_code == 400

        r_get = client.get(f"/voice-assistant/personality/{p_id}")
        assert r_get.status_code == 200
        assert r_get.get_json().get("sttEngine") == "local_whisper"

        payload_local = {"sttEngine": "local_whisper", "ttsEngine": "supertone"}
        r_put_back = client.put(
            f"/voice-assistant/personality/{p_id}", json=payload_local
        )
        assert r_put_back.status_code in [200, 204]

        r_get_back = client.get(f"/voice-assistant/personality/{p_id}")
        assert r_get_back.status_code == 200
        assert r_get_back.get_json().get("sttEngine") == "local_whisper"
