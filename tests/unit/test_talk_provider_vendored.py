"""The vendored Hermes Talk provider and TalkLiveLoop audio/barge-in architecture."""

import base64
import sys
import types
from unittest.mock import MagicMock

import numpy as np

from voice_assistant.talk_provider import talk_gemini_realtime as provider
from voice_assistant.talk_provider import talk_realtime as rt


def _setup(**overrides):
    base = dict(
        model="models/gemini-3.8-live",
        voice="Puck",
        instructions="Du bist pib.",
    )
    base.update(overrides)
    return rt.SessionSetup(**base)


def test_a_setup_carries_model_voice_and_instructions():
    message = provider.build_setup_message(_setup())
    body = message["setup"]
    assert body["model"] == "models/gemini-3.8-live"
    assert (
        body["generationConfig"]["speechConfig"]["voiceConfig"]["prebuiltVoiceConfig"][
            "voiceName"
        ]
        == "Puck"
    )
    assert body["systemInstruction"]["parts"][0]["text"] == "Du bist pib."
    vad = body["realtimeInputConfig"]["automaticActivityDetection"]
    assert vad["silenceDurationMs"] == 350
    assert vad["prefixPaddingMs"] == 60


def test_tool_schemas_are_reduced_to_the_live_vocabulary():
    """The Live API closes the socket with 1007 on any unknown schema key."""

    definition = rt.ToolDefinition(
        name="search_memory",
        description="Look something up.",
        parameters={
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "query": {"type": "string", "minLength": 1, "minimum": 0},
                "limit": {
                    "type": "integer",
                    "minimum": 1,
                    "maximum": 20,
                    "default": 5,
                },
            },
            "required": ["query"],
        },
    )

    message = provider.build_setup_message(_setup(tools=(definition,)))
    params = message["setup"]["tools"][0]["functionDeclarations"][0]["parameters"]

    assert params["type"] == "OBJECT"
    assert set(params["properties"]) == {"query", "limit"}
    assert "additionalProperties" not in params
    assert "minimum" not in params["properties"]["limit"]
    assert "default" not in params["properties"]["limit"]
    assert params["properties"]["query"]["type"] == "STRING"


def test_an_ephemeral_token_is_not_used_as_a_query_key():
    """Tokens ride Authorization: Token auth_tokens/… on the constrained route.

    Asserted at the source level because the branch lives inside the socket
    handshake; the SDK does exactly this for api_key.startswith("auth_tokens/").
    """

    import inspect

    source = inspect.getsource(provider)
    assert "BidiGenerateContentConstrained" in source
    assert 'ws_headers = {"Authorization": f"Token {auth_token}"}' in source
    assert 'auth_token.startswith("auth_tokens/")' in source


def test_16khz_input_bypasses_resampler():
    """When input_sample_rate=16000, PCM bytes pass through untouched."""

    session = provider.GeminiRealtimeSession(
        auth_token="auth_tokens/test",
        auth_source="pib-cloud",
        input_sample_rate=16000,
    )
    assert session._resampler is None
    raw_pcm = (np.arange(320, dtype="<i2") * 10).tobytes()
    encoded = session._encode([rt.AppendInputAudio(data=raw_pcm)])
    assert len(encoded) == 1
    b64 = encoded[0]["realtimeInput"]["audio"]["data"]
    assert base64.b64decode(b64) == raw_pcm


def test_uplink_silence_shaper_and_barge_in(monkeypatch):
    """Quiet chunks become digital zeros; speech flushes pre-roll; barge-in flushes playback."""
    from ros_stubs import install_ros_stubs

    install_ros_stubs()
    _pyaudio = types.ModuleType("pyaudio")
    _pyaudio.PyAudio = MagicMock()
    _pyaudio.paInt16 = 8
    monkeypatch.setitem(sys.modules, "pyaudio", _pyaudio)
    _google = types.ModuleType("google")
    _genai = types.ModuleType("google.genai")
    _genai.Client = MagicMock()
    _google.genai = _genai
    monkeypatch.setitem(sys.modules, "google", _google)
    monkeypatch.setitem(sys.modules, "google.genai", _genai)

    from voice_assistant.talk_live_loop import HANGOVER_CHUNKS, TalkLiveLoop

    node = MagicMock()
    loop = TalkLiveLoop(node)
    loop._mute_uplink_until = 0.0

    quiet = (np.ones(1024, dtype="<i2") * 100).tobytes()
    loud = (np.ones(1024, dtype="<i2") * 2000).tobytes()

    # 1) Quiet room chunk -> digital silence, buffered in pre-roll
    out1 = loop._shape_uplink_chunk(quiet, 10.0)
    assert out1 == b"\x00" * len(quiet)
    assert len(loop._preroll) == 1

    # 2) Speech onset -> flushes pre-roll + loud chunk
    out2 = loop._shape_uplink_chunk(loud, 10.064)
    assert out2 == quiet + loud
    assert loop._in_user_speech is True

    # 3) Hangover keeps speech open for HANGOVER_CHUNKS, then returns to digital silence
    for i in range(HANGOVER_CHUNKS):
        out_h = loop._shape_uplink_chunk(quiet, 10.2 + i * 0.064)
        assert out_h == quiet
    assert loop._in_user_speech is False
    out_silent = loop._shape_uplink_chunk(quiet, 11.0)
    assert out_silent == b"\x00" * len(quiet)

    # 4) Barge-in while player is playing and server is generating
    loop._player._writing = True
    loop._server_generating = True
    loop._hw_vad_active = True
    loop._hw_vad_at = 12.0
    loop._hw_speech_at = 12.0
    out_barge = loop._shape_uplink_chunk(loud, 12.05)
    assert loop._barge_in_fenced is True
    assert loop._player.is_playing is False
    assert out_barge.endswith(loud)
