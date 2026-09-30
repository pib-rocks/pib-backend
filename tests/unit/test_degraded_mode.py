"""Startup password: named degraded mode, local voice, display unlock."""

from __future__ import annotations

import json
import stat
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
DISPLAY_ROOT = REPO_ROOT / "ros_packages" / "display"
for path in (str(VOICE_ROOT), str(DISPLAY_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

from display.display_web_request import validate_document  # noqa: E402
from display.password_prompt import (  # noqa: E402
    MODE_DEGRADED as DISPLAY_DEGRADED,
)
from display.password_prompt import (  # noqa: E402
    MODE_UNLOCKED as DISPLAY_UNLOCKED,
)
from display.password_prompt import (  # noqa: E402
    password_prompt_url,
    prompt_decision,
    read_operating_mode,
)
from service import key_store_service  # noqa: E402
from service.key_store_service import (  # noqa: E402
    MODE_DEGRADED as FLASK_DEGRADED,
)
from service.key_store_service import (  # noqa: E402
    MODE_UNLOCKED as FLASK_UNLOCKED,
)
from voice_assistant.degraded_chat import (  # noqa: E402
    LOCAL_VOICE_IN,
    LOCAL_VOICE_OUT,
    MODE_DEGRADED,
    MODE_UNLOCKED,
    allows_cloud_chat,
    fetch_operating_mode,
    local_voice_available,
    refusal_sentence,
    spoken_reply,
)

PASSWORD = "operator-secret"
OTHER_PASSWORD = "different-secret"
SECRET = "sk-test-ROUNDTRIP-9f3a"


class _Body:
    def __init__(self, payload: bytes):
        self.payload = payload

    def read(self) -> bytes:
        return self.payload

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False


@pytest.fixture(autouse=True)
def key_store_path(tmp_path, monkeypatch):
    path = tmp_path / "secrets" / "provider_key_store.json"
    monkeypatch.setenv("PIB_KEY_STORE_PATH", str(path))
    monkeypatch.delenv("PIB_UPDATE_DIR", raising=False)
    key_store_service.lock()
    yield path
    key_store_service.lock()


def test_mode_name_is_shared():
    assert FLASK_DEGRADED == DISPLAY_DEGRADED == MODE_DEGRADED == "degraded"
    assert FLASK_UNLOCKED == DISPLAY_UNLOCKED == MODE_UNLOCKED == "unlocked"


def test_local_voice_stays_available_and_cloud_chats_do_not():
    assert local_voice_available(MODE_DEGRADED) is True
    assert local_voice_available(MODE_UNLOCKED) is True
    assert allows_cloud_chat(MODE_DEGRADED) is False
    assert allows_cloud_chat(MODE_UNLOCKED) is True
    assert LOCAL_VOICE_IN == "faster-whisper"
    assert LOCAL_VOICE_OUT == "supertone"


def test_refusal_is_spoken_in_the_personality_voice():
    reply = spoken_reply("smart", gender="Female", language="German")
    assert reply["engine"] == "supertone"
    assert reply["gender"] == "Female"
    assert reply["language"] == "German"
    assert reply["text"] == refusal_sentence("smart")
    assert "Smart" in reply["text"]
    assert "operator password" in reply["text"]
    assert reply["text"].endswith(".")

    direct = spoken_reply("direct", gender="Male", language="German")
    assert direct["gender"] == "Male"
    assert "Direct" in direct["text"]
    assert "operator password" in direct["text"]


def test_unreadable_key_store_is_degraded_not_an_exception():
    def opener(_url, timeout):
        assert timeout == 2.0
        raise OSError("flask is down")

    assert fetch_operating_mode(opener=opener) == MODE_DEGRADED

    def unlocked(_url, timeout):
        return _Body(b'{"mode": "unlocked", "encryptKeyStorage": true}')

    assert fetch_operating_mode(opener=unlocked) == MODE_UNLOCKED


def test_startup_status_names_degraded_mode(app):
    body = app.test_client().get("/system/key-store").get_json()
    assert body["mode"] == "degraded"
    assert body["encryptKeyStorage"] is True


def test_cancel_leaves_degraded_mode_and_closes_the_display(app, tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_UPDATE_DIR", str(tmp_path))
    response = app.test_client().post("/system/key-store/cancel", json={})
    assert response.status_code == 200
    body = response.get_json()
    assert body["successful"] is True
    assert body["mode"] == "degraded"
    assert "error" not in body
    assert key_store_service.operating_mode() == "degraded"
    hide_path = tmp_path / "display-web.json"
    document = json.loads(hide_path.read_text(encoding="utf-8"))
    assert validate_document(document)["action"] == "hide"
    assert stat.S_IMODE(hide_path.stat().st_mode) == 0o644


def test_display_cancel_is_not_an_error(app):
    response = app.test_client().post("/system/key-store/display/cancel", json={})
    assert response.status_code == 200
    assert response.get_json()["mode"] == "degraded"
    assert response.get_json()["successful"] is True


def test_cancel_does_not_lock_an_open_store(app, app_ctx):
    from model.provider_model import Provider

    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    opened = key_store_service.unlock(PASSWORD)
    assert SECRET in opened.values()

    response = app.test_client().post("/system/key-store/cancel", json={})
    assert response.status_code == 200
    assert response.get_json()["mode"] == "unlocked"
    assert key_store_service.unlocked_credentials()[f"provider-{vision.id}"] == SECRET


def test_display_path_unlocks_the_store(app, app_ctx, tmp_path, monkeypatch):
    from model.provider_model import Provider

    monkeypatch.setenv("PIB_UPDATE_DIR", str(tmp_path))
    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    key_store_service.lock()
    assert key_store_service.operating_mode() == "degraded"

    page = app.test_client().get("/system/key-store/display")
    html = page.get_data(as_text=True)
    assert page.status_code == 200
    assert page.mimetype == "text/html"
    assert 'type="password"' in html
    assert ">OK<" in html
    assert ">Cancel<" in html
    assert "/system/key-store/display/unlock" in html
    assert "/system/key-store/display/cancel" in html

    wrong = app.test_client().post(
        "/system/key-store/display/unlock",
        json={"password": OTHER_PASSWORD},
    )
    assert wrong.status_code == 401
    assert wrong.get_json()["successful"] is False
    assert key_store_service.operating_mode() == "degraded"
    assert not (tmp_path / "display-web.json").exists()

    opened = app.test_client().post(
        "/system/key-store/display/unlock",
        json={"password": PASSWORD},
    )
    assert opened.status_code == 200
    body = opened.get_json()
    assert body["successful"] is True
    assert body["mode"] == "unlocked"
    assert body["credentials"] == [{"credentialRef": f"provider-{vision.id}"}]
    assert SECRET not in opened.get_data(as_text=True)
    assert key_store_service.unlocked_credentials()[f"provider-{vision.id}"] == SECRET
    hide_path = tmp_path / "display-web.json"
    document = json.loads(hide_path.read_text(encoding="utf-8"))
    assert validate_document(document)["action"] == "hide"
    assert stat.S_IMODE(hide_path.stat().st_mode) == 0o644


def test_encryption_off_skips_the_startup_password_prompt(app, key_store_path):
    """The prompt opens only for degraded mode. Encryption off is unlocked."""
    settings = key_store_path.parent / "key_store_settings.json"
    settings.parent.mkdir(parents=True, exist_ok=True)
    settings.write_text('{"encrypt_key_storage": false}', encoding="utf-8")
    mode = app.test_client().get("/system/key-store").get_json()["mode"]
    assert mode == "unlocked"
    assert prompt_decision(mode) == "skip"


def test_display_opens_the_prompt_only_while_degraded(monkeypatch):
    assert prompt_decision("degraded") == "open"
    assert prompt_decision("unlocked") == "skip"
    assert prompt_decision(None) == "retry"
    monkeypatch.setenv(
        "PIB_KEY_STORE_PROMPT_URL", "http://127.0.0.1:5000/system/key-store/display"
    )
    assert password_prompt_url().endswith("/system/key-store/display")

    def opener(url, timeout):
        assert url.endswith("/system/key-store")
        assert timeout == 1.5
        return _Body(b'{"mode": "degraded"}')

    assert read_operating_mode(opener=opener) == "degraded"

    def down(_url, timeout):
        raise TimeoutError("flask down")

    assert read_operating_mode(opener=down) is None


def test_local_engines_and_the_speech_block_ignore_the_key_store():
    player = (
        REPO_ROOT / "ros_packages/voice_assistant/voice_assistant/audio_player.py"
    ).read_text(encoding="utf-8")
    recorder = (
        REPO_ROOT / "ros_packages/voice_assistant/voice_assistant/audio_recorder.py"
    ).read_text(encoding="utf-8")
    speech = (
        REPO_ROOT
        / "pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/util/function-declarations.ts"
    ).read_text(encoding="utf-8")
    for source in (player, recorder, speech):
        assert "key_store" not in source
        assert "degraded" not in source
    assert "SupertoneTTSEngine" in player
    assert "self.tts_engine.synthesize" in player
    assert 'os.getenv("STT_ENGINE", "local_whisper")' in recorder
    assert "FasterWhisperSTTEngine" in recorder
    assert "PlayAudioFromSpeech" in speech
    assert "local Supertonic TTS" in speech
