"""Live session lifecycle: pinned model, idle timeout, exclusive voice channel."""

from datetime import date
from pathlib import Path
from unittest.mock import MagicMock

from model.provider_model import RegistryModel
from pib_hermes_config.live_session import (
    DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS,
    GEMINI_LIVE_MODEL,
    OPENAI_LIVE_MODEL,
    RETIRED_LIVE_MODEL,
    VOICE_MODE_LIVE,
    VOICE_MODE_TURN_BASED,
    channel_turn_on_allowed,
    context_window_compression,
    gemini_live_connect_model,
    idle_expired,
    live_output_language,
    live_session_ends_on_handover,
    openai_models_url,
    voice_start_mode,
)
from pib_hermes_config.voice_backends import LIVE_VOICE_NOTE
from service import live_model_service, personality_service

REPO_ROOT = Path(__file__).resolve().parents[2]
AUDIO_LOOP = (
    REPO_ROOT / "ros_packages" / "voice_assistant" / "voice_assistant" / "audio_loop.py"
)


def test_retired_preview_is_not_hardcoded_and_compression_stays_on():
    source = AUDIO_LOOP.read_text(encoding="utf-8")
    assert RETIRED_LIVE_MODEL not in source
    assert "ENABLE_CONTEXT_COMPRESSION" not in source
    assert "context_window_compression(" in source
    assert gemini_live_connect_model(RETIRED_LIVE_MODEL) is None
    assert gemini_live_connect_model(GEMINI_LIVE_MODEL) == GEMINI_LIVE_MODEL
    assert gemini_live_connect_model(OPENAI_LIVE_MODEL) is None
    compressed = context_window_compression(100000, 80000)
    assert compressed["trigger_tokens"] == 100000
    assert compressed["sliding_window"]["target_tokens"] == 80000


def test_idle_timeout_expires_only_after_silence():
    assert idle_expired(0.0, 59.9, 60) is False
    assert idle_expired(0.0, 60.0, 60) is True
    assert idle_expired(10.0, 20.0, 60) is False


def test_another_personality_cannot_take_the_voice_channel():
    assert channel_turn_on_allowed(False, "holder", "other") is True
    assert channel_turn_on_allowed(True, "holder", "holder") is True
    assert channel_turn_on_allowed(True, "holder", "other") is False
    assert live_session_ends_on_handover(True, "chat-a", "chat-b", True) is True
    assert live_session_ends_on_handover(True, "chat-a", "chat-a", True) is False
    assert live_session_ends_on_handover(False, "chat-a", "chat-b", True) is False


def test_voice_start_mode_is_gated_by_the_live_flag_and_the_pinned_model():
    assert voice_start_mode(VOICE_MODE_LIVE, True, GEMINI_LIVE_MODEL) == VOICE_MODE_LIVE
    assert (
        voice_start_mode(VOICE_MODE_TURN_BASED, True, GEMINI_LIVE_MODEL)
        == VOICE_MODE_TURN_BASED
    )
    assert voice_start_mode(VOICE_MODE_LIVE, False, GEMINI_LIVE_MODEL) == (
        VOICE_MODE_TURN_BASED
    )
    assert voice_start_mode(VOICE_MODE_LIVE, True, None) == VOICE_MODE_TURN_BASED
    assert voice_start_mode(VOICE_MODE_LIVE, True, OPENAI_LIVE_MODEL) == (
        VOICE_MODE_TURN_BASED
    )


def test_the_live_model_is_its_own_row_and_flash_is_not_pinned(app_ctx):
    flash = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
    live = RegistryModel.query.filter_by(api_name="gemini-3.8-live").one()
    assert flash.capabilities["live"] is False
    assert flash.live_model is None
    assert flash.live_model_checked_on is None
    assert live.visual_name == "Gemini 3.8 Live"
    assert live.api_name == GEMINI_LIVE_MODEL
    assert live.capabilities["live"] is True
    assert live.live_model is None
    assert live.provider_id == flash.provider_id
    openai = RegistryModel.query.filter_by(api_name="gpt-6").one()
    assert openai.live_model is None
    assert openai.live_model_checked_on is None
    assert openai.capabilities["live"] is False


def test_provider_api_lists_the_live_model_beside_flash(app):
    body = app.test_client().get("/provider").get_json()
    google = next(row for row in body["providers"] if row["name"] == "Google")
    by_name = {model["visualName"]: model for model in google["models"]}
    assert by_name["Gemini 3.8 Flash"]["apiName"] == "gemini-3.8-flash"
    assert by_name["Gemini 3.8 Flash"]["capabilities"]["live"] is False
    assert "liveModel" not in by_name["Gemini 3.8 Flash"]
    assert by_name["Gemini 3.8 Live"]["apiName"] == GEMINI_LIVE_MODEL
    assert by_name["Gemini 3.8 Live"]["capabilities"]["live"] is True


def test_personality_live_settings_choose_the_mode_the_button_starts(app, monkeypatch):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    with app.app_context():
        flash_id = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one().id
        live_id = RegistryModel.query.filter_by(api_name="gemini-3.8-live").one().id
    client = app.test_client()
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "FlashChat",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": flash_id,
        },
    )
    assert created.status_code == 201
    body = created.get_json()
    assert body["voiceMode"] == VOICE_MODE_TURN_BASED
    assert body["liveIdleTimeout"] == DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS
    assert body["liveModel"] is None
    assert body["voiceStartMode"] == VOICE_MODE_TURN_BASED
    assert body["localVoiceApplies"] is True
    assert body["liveVoiceNote"] is None

    personality_id = body["personalityId"]
    updated = client.put(
        f"/voice-assistant/personality/{personality_id}",
        json={"liveIdleTimeout": 45},
    )
    assert updated.status_code == 200
    body = updated.get_json()
    assert body["voiceMode"] == VOICE_MODE_TURN_BASED
    assert body["liveIdleTimeout"] == 45
    assert body["voiceStartMode"] == VOICE_MODE_TURN_BASED

    rejected = client.put(
        f"/voice-assistant/personality/{personality_id}",
        json={"voiceMode": VOICE_MODE_LIVE, "liveIdleTimeout": 45},
    )
    assert rejected.status_code == 400
    kept = client.get(f"/voice-assistant/personality/{personality_id}").get_json()
    assert kept["voiceMode"] == VOICE_MODE_TURN_BASED
    assert kept["liveIdleTimeout"] == 45

    refused = client.put(
        f"/voice-assistant/personality/{personality_id}",
        json={"liveIdleTimeout": 0},
    )
    assert refused.status_code == 400

    live = client.post(
        "/voice-assistant/personality",
        json={
            "name": "LiveModel",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": live_id,
        },
    )
    assert live.status_code == 201
    live_body = live.get_json()
    assert live_body["voiceMode"] == VOICE_MODE_LIVE
    assert live_body["liveModel"] == GEMINI_LIVE_MODEL
    assert live_body["voiceStartMode"] == VOICE_MODE_LIVE
    assert live_body["localVoiceApplies"] is False
    assert live_body["liveVoiceNote"] == LIVE_VOICE_NOTE


def test_pin_confirms_the_named_live_model_and_leaves_the_chat_model(app_ctx):
    from app.app import db

    flash = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
    live = RegistryModel.query.filter_by(api_name="gemini-3.8-live").one()
    openai = RegistryModel.query.filter_by(api_name="gpt-6").one()
    flash.provider.credential_ref = "provider-gemini"
    live.capabilities = {**live.capabilities, "live": False}
    openai.provider.credential_ref = "provider-openai"
    db.session.commit()

    def fetch(api_name, endpoint_base, api_key):
        assert api_name == GEMINI_LIVE_MODEL
        assert api_key == "gemini-secret"
        assert endpoint_base is None
        return [GEMINI_LIVE_MODEL, "gemini-3.8-flash"]

    live_model_service.pin_providers(
        RegistryModel.query.all(),
        {
            "provider-gemini": "gemini-secret",
            "provider-openai": "openai-secret",
        },
        fetch=fetch,
        checked_on=date(2026, 9, 30),
    )
    db.session.commit()

    flash = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
    live = RegistryModel.query.filter_by(api_name="gemini-3.8-live").one()
    openai = RegistryModel.query.filter_by(api_name="gpt-6").one()
    assert flash.live_model is None
    assert flash.capabilities["live"] is False
    assert flash.live_model_checked_on is None
    assert live.live_model is None
    assert live.api_name == GEMINI_LIVE_MODEL
    assert live.live_model_checked_on == date(2026, 9, 30)
    assert live.capabilities["live"] is True
    assert openai.live_model is None
    assert openai.live_model_checked_on is None
    assert openai.capabilities["live"] is False


def test_a_list_without_the_named_model_clears_its_live_flag(app_ctx):
    from app.app import db

    live = RegistryModel.query.filter_by(api_name="gemini-3.8-live").one()
    flash = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
    live.provider.credential_ref = "provider-gemini"
    db.session.commit()

    live_model_service.pin_providers(
        [live, flash],
        {"provider-gemini": "gemini-secret"},
        fetch=lambda api_name, endpoint_base, api_key: ["gemini-3.8-flash"],
        checked_on=date(2026, 9, 30),
    )
    db.session.commit()
    live = RegistryModel.query.filter_by(api_name="gemini-3.8-live").one()
    flash = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one()
    assert live.live_model is None
    assert live.capabilities["live"] is False
    assert live.live_model_checked_on == date(2026, 9, 30)
    assert flash.live_model is None
    assert flash.capabilities["live"] is False
    assert flash.live_model_checked_on is None


def test_model_list_urls_and_payloads_do_not_put_the_key_in_the_query():
    assert openai_models_url(None) == "https://api.openai.com/v1/models"
    assert openai_models_url("https://example.test/v1") == (
        "https://example.test/v1/models"
    )
    assert openai_models_url("https://example.test") == (
        "https://example.test/v1/models"
    )

    ids, token = live_model_service.ids_from_list_payload(
        {
            "models": [{"name": "models/" + GEMINI_LIVE_MODEL}],
            "nextPageToken": "next",
        },
        "gemini",
    )
    assert ids == [GEMINI_LIVE_MODEL]
    assert token == "next"
    openai_ids, openai_token = live_model_service.ids_from_list_payload(
        {"data": [{"id": OPENAI_LIVE_MODEL}]},
        "openai",
    )
    assert openai_ids == [OPENAI_LIVE_MODEL]
    assert openai_token is None

    class _Body:
        def __init__(self, payload):
            self._payload = payload

        def read(self):
            import json

            return json.dumps(self._payload).encode("utf-8")

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

    pages = [
        {
            "models": [{"name": "models/gemini-3.8-flash"}],
            "nextPageToken": "page-2",
        },
        {"models": [{"name": "models/" + GEMINI_LIVE_MODEL}]},
    ]

    def opener(request, timeout):
        assert timeout == 10.0
        assert "key=" not in request.full_url
        assert request.get_header("X-goog-api-key") == "secret"
        assert request.full_url.startswith(
            "https://generativelanguage.googleapis.com/v1beta/models?"
        )
        return _Body(pages.pop(0))

    found = live_model_service.fetch_model_ids(
        "gemini-3.8-flash", None, "secret", opener=opener
    )
    assert found == ["gemini-3.8-flash", GEMINI_LIVE_MODEL]
    assert pages == []


# --- GeminiAudioLoop pib.Cloud session lifecycle (fake provider + fake live client) ---

import asyncio
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from ros_stubs import install_ros_stubs

# Load the real client before the stubs run: install_ros_stubs() replaces
# pib_api_client.voice_assistant_client with a bare module when it is not
# imported yet, and every later test then sees a module without its functions.
import pib_api_client.voice_assistant_client  # noqa: F401,E402

install_ros_stubs()

import types
from unittest.mock import MagicMock

_pyaudio = types.ModuleType("pyaudio")
_pyaudio.PyAudio = MagicMock()
_pyaudio.paInt16 = 8
_prev_pyaudio = sys.modules.get("pyaudio")
sys.modules["pyaudio"] = _pyaudio

_prev_google = sys.modules.get("google")
_prev_genai = sys.modules.get("google.genai")
_google = types.ModuleType("google")
_genai = types.ModuleType("google.genai")
_genai.Client = MagicMock()
_google.genai = _genai
sys.modules["google"] = _google
sys.modules["google.genai"] = _genai

from voice_assistant import audio_loop as audio_loop_module  # noqa: E402
from voice_assistant.audio_loop import (  # noqa: E402
    CloudSessionEnded,
    GeminiAudioLoop,
)
from voice_assistant.cloud_session import (  # noqa: E402
    CascadeResult,
    CloudSession,
    CloudSessionError,
)

# The stubs exist only to import the module under test. Leaving them in
# sys.modules broke every later test that imports the real google package
# ("ImportError ... unknown location" in test_provider_registry).
for _name, _previous in (
    ("google.genai", _prev_genai),
    ("google", _prev_google),
    ("pyaudio", _prev_pyaudio),
):
    if _previous is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _previous

_LIVE_EPHEMERAL = "ephemeral-live-token-77-SECRET"
_LIVE_MODEL = GEMINI_LIVE_MODEL


def _cloud_session(
    session_id: str = "sess-1",
    *,
    token: str = _LIVE_EPHEMERAL,
    ttl_seconds: int = 600,
) -> CloudSession:
    return CloudSession(
        session_id=session_id,
        ephemeral_token=token,
        issued_at="2026-01-01T00:00:00Z",
        expires_at="2026-01-01T00:10:00Z",
        ttl_seconds=ttl_seconds,
        model=_LIVE_MODEL,
        transport="websocket",
        endpoint="wss://live.example/v1",
        pre_hold_pib_tokens=1,
        daily_slots_used=1,
        daily_slots_limit=10,
        daily_slots_remaining=9,
    )


def test_cloud_live_first_connect_uses_minted_token_not_api_key(caplog):
    caplog.set_level("INFO")
    usage_calls: list[tuple] = []

    def provider(model, previous_session_id=None):
        assert previous_session_id is None
        assert model == _LIVE_MODEL
        return _cloud_session()

    def reporter(session_id, usage_metadata=None):
        usage_calls.append((session_id, usage_metadata))

    client_keys: list[str] = []

    def client_factory(*, api_key):
        client_keys.append(api_key)
        return MagicMock()

    original_client = audio_loop_module.genai.Client
    audio_loop_module.genai.Client = client_factory
    try:
        loop = GeminiAudioLoop(
            api_key="static-key-store-secret",
            cloud_session_mode=True,
            session_provider=provider,
            usage_reporter=reporter,
        )
        loop._model = _LIVE_MODEL
        loop._usage_metadata_accum["total_token_count"] = 7

        async def exercise():
            session = await loop._open_cloud_session(None)
            audio_loop_module.genai.Client(api_key=session.ephemeral_token)
            await loop._settle_cloud_usage()

        asyncio.run(exercise())
    finally:
        audio_loop_module.genai.Client = original_client

    assert client_keys == [_LIVE_EPHEMERAL]
    assert _LIVE_EPHEMERAL not in caplog.text
    assert "sess-1" in caplog.text
    assert usage_calls == [("sess-1", {"total_token_count": 7})]


def test_cloud_live_renewal_reports_usage_before_mint_with_previous_id():
    mint_calls: list[str | None] = []
    usage_calls: list[tuple] = []

    def provider(model, previous_session_id=None):
        mint_calls.append(previous_session_id)
        if previous_session_id is None:
            return _cloud_session("sess-a")
        return _cloud_session("sess-b", token="token-b")

    def reporter(session_id, usage_metadata=None):
        usage_calls.append((session_id, usage_metadata))

    loop = GeminiAudioLoop(
        api_key="",
        cloud_session_mode=True,
        session_provider=provider,
        usage_reporter=reporter,
    )
    loop._model = _LIVE_MODEL
    loop._cloud_session_id = "sess-a"
    loop._usage_metadata_accum["total_token_count"] = 7

    async def exercise():
        await loop._settle_cloud_usage("sess-a")
        return await loop._open_cloud_session("sess-a")

    session = asyncio.run(exercise())

    assert mint_calls == ["sess-a"]
    assert usage_calls == [("sess-a", {"total_token_count": 7})]
    assert session.session_id == "sess-b"


def test_cascade_result_ends_session_and_calls_callback():
    reasons: list[str] = []

    def provider(model, previous_session_id=None):
        return CascadeResult(reason="budget")

    loop = GeminiAudioLoop(
        api_key="",
        cloud_session_mode=True,
        session_provider=provider,
        usage_reporter=lambda *_a, **_k: None,
        on_session_end=reasons.append,
    )

    async def exercise():
        with pytest.raises(CloudSessionEnded, match="budget"):
            await loop._open_cloud_session(None)

    asyncio.run(exercise())
    assert reasons == ["budget"]
    assert loop._is_listening is False


def test_cloud_session_error_ends_session_and_calls_callback():
    reasons: list[str] = []

    def provider(model, previous_session_id=None):
        raise CloudSessionError("budget_exhausted", "no budget")

    loop = GeminiAudioLoop(
        api_key="",
        cloud_session_mode=True,
        session_provider=provider,
        usage_reporter=lambda *_a, **_k: None,
        on_session_end=reasons.append,
    )

    async def exercise():
        with pytest.raises(CloudSessionEnded, match="budget_exhausted"):
            await loop._open_cloud_session(None)

    asyncio.run(exercise())
    assert reasons == ["budget_exhausted"]
    assert loop._is_listening is False


def test_an_unexpected_error_ends_the_session_so_the_button_is_not_stuck():
    reasons: list[str] = []

    loop = GeminiAudioLoop(
        api_key="",
        cloud_session_mode=True,
        session_provider=lambda *a, **k: _cloud_session(),
        usage_reporter=lambda *a, **k: None,
        on_session_end=reasons.append,
    )
    loop._cloud_session_id = "sess-1"

    async def exercise():
        await loop._fail_session("connection_failed")

    asyncio.run(exercise())
    assert reasons == ["connection_failed"]
    assert loop._is_listening is False
    assert loop._cloud_session_id is None


def test_the_run_error_path_closes_the_session():
    source = AUDIO_LOOP.read_text(encoding="utf-8")
    assert 'await self._fail_session("connection_failed")' in source


def test_a_non_cloud_loop_leaves_the_error_path_alone():
    reasons: list[str] = []

    loop = GeminiAudioLoop(api_key="key", on_session_end=reasons.append)

    async def exercise():
        await loop._fail_session("connection_failed")

    asyncio.run(exercise())
    assert reasons == []


def test_the_live_output_language_is_pinned_and_overridable(monkeypatch):
    monkeypatch.delenv("PIB_LIVE_LANGUAGE", raising=False)
    assert live_output_language() == "de-DE"

    monkeypatch.setenv("PIB_LIVE_LANGUAGE", "en-GB")
    assert live_output_language() == "en-GB"

    monkeypatch.setenv("PIB_LIVE_LANGUAGE", "   ")
    assert live_output_language() == "de-DE"


def test_the_live_config_asks_for_the_pinned_language():
    source = AUDIO_LOOP.read_text(encoding="utf-8")
    assert '"speech_config": {"language_code": LIVE_LANGUAGE}' in source
    assert "LIVE_LANGUAGE = live_output_language()" in source


def test_ephemeral_token_never_logged(caplog):
    caplog.set_level("DEBUG")

    async def exercise():
        loop = GeminiAudioLoop(
            api_key="",
            cloud_session_mode=True,
            session_provider=lambda *_a, **_k: _cloud_session(token=_LIVE_EPHEMERAL),
            usage_reporter=lambda *_a, **_k: None,
        )
        await loop._open_cloud_session(None)

    asyncio.run(exercise())
    assert _LIVE_EPHEMERAL not in caplog.text
