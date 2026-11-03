"""PC-109: the audio player speaks a clause through pib.Cloud.

The real player source is compiled with only ROS/native names replaced, as in
``test_audio_player_cancellation``. The HTTP layer and the key-store read are
stubs, so these tests assert the request shape and the deterministic fallback
without a network, a device or a ROS runtime.
"""

from __future__ import annotations

import ast
import contextlib
import io
import logging
import os
import sys
import time
import wave
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Lock, Thread
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import requests

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros_packages/voice_assistant"))

from pib_hermes_config.turn_taking import clauses_to_synthesize
from pib_hermes_config.visible_state import (
    VOICE_USING_FALLBACK_TOPIC,
    engine_is_fallback,
)
from pib_hermes_config.voice_backends import (
    CLOUD_TTS_ROUTE,
    LOCAL_TTS_ID,
    effective_tts_engine,
    resolve_tts_route,
)
from pib_hermes_config.voice_catalogue import resolve_voice_id
from voice_assistant.cloud_token import CloudTokenError
from voice_assistant.pib_cloud_stream import pib_cloud_base_url

SOURCE = ROOT / "ros_packages/voice_assistant/voice_assistant/audio_player.py"

CLOUD_PCM = b"\x10\x00\x20\x00" * 800
LOCAL_PCM = b"\x01\x00" * 400


def wav_container(pcm: bytes, rate: int = 16000) -> bytes:
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(rate)
        target.writeframes(pcm)
    return buffer.getvalue()


def build_player(post, token="cloud-token"):
    """Compile the real player with ROS/native imports replaced by stubs."""
    local_calls = []

    def resolve_token():
        if token is None:
            raise CloudTokenError("No keys are available for provider pib-cloud.")
        return token

    ns = dict(
        __name__="_pc109_audio_player",
        Node=object,
        pyaudio=None,
        io=io,
        os=os,
        sys=sys,
        time=time,
        wave=wave,
        logging=logging,
        Queue=Queue,
        Empty=Empty,
        Lock=Lock,
        Event=Event,
        Thread=Thread,
        Bool=lambda: SimpleNamespace(data=None),
        String=lambda: SimpleNamespace(data=None),
        clauses_to_synthesize=clauses_to_synthesize,
        resolve_tts_route=resolve_tts_route,
        effective_tts_engine=effective_tts_engine,
        LOCAL_TTS_ID=LOCAL_TTS_ID,
        resolve_voice_id=resolve_voice_id,
        CLOUD_TTS_ROUTE=CLOUD_TTS_ROUTE,
        util=SimpleNamespace(surpress_stderr=contextlib.nullcontext),
        requests=SimpleNamespace(post=post),
        resolve_cloud_token=resolve_token,
        CloudTokenError=CloudTokenError,
        pib_cloud_base_url=pib_cloud_base_url,
        public_voice_client=SimpleNamespace(
            text_to_speech=lambda *a, **kw: pytest.fail("public voice client used")
        ),
        engine_is_fallback=engine_is_fallback,
        VOICE_USING_FALLBACK_TOPIC=VOICE_USING_FALLBACK_TOPIC,
    )
    tree = ast.parse(SOURCE.read_text())
    body = [
        ast.ImportFrom(
            module="__future__", names=[ast.alias(name="annotations")], level=0
        )
    ]
    body += [
        node
        for node in tree.body
        if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.Assign, ast.AnnAssign))
    ]
    module = ast.Module(body=body, type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(SOURCE), "exec"), ns)

    def local_synthesize(**kwargs):
        local_calls.append(kwargs)
        return wav_container(LOCAL_PCM, rate=44100)

    cls = ns["AudioPlayerNode"]
    node = cls.__new__(cls)
    node.playback_queue = Queue()
    node.counter = 0
    node.counter_lock = Lock()
    node.token = None
    node._published_fallback = None
    node.fallback_events = []

    def capture_fallback(msg):
        node.fallback_events.append(getattr(msg, "data", msg))

    node.voice_using_fallback_publisher = SimpleNamespace(publish=capture_fallback)
    node.tts_engine = SimpleNamespace(
        synthesize=local_synthesize, active_backend="supertone-supertonic-3"
    )
    log = SimpleNamespace(
        debug=MagicMock(), info=MagicMock(), warning=MagicMock(), error=MagicMock()
    )
    node.get_logger = lambda: log
    ns["bind_playback_queue"](node.playback_queue)
    return node, ns, log, local_calls


def request(text="Ein Satz.", join=False, engine="cloud_tts", voice_id=None):
    return SimpleNamespace(
        speech=text,
        gender="Male",
        language="de",
        tts_engine=engine,
        voice_id=voice_id,
        join=join,
    )


def queued_pcm(node) -> list[bytes]:
    items = []
    while not node.playback_queue.empty():
        items.append(node.playback_queue.get_nowait())
    return [b"".join(item.data) for item in items]


def test_cloud_speech_posts_the_contract_shape_and_queues_the_wav(monkeypatch):
    calls = []

    def post(url, **kwargs):
        calls.append((url, kwargs))
        return SimpleNamespace(status_code=200, content=wav_container(CLOUD_PCM))

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, log, local_calls = build_player(post)

    node.play_audio_from_speech(request(), SimpleNamespace())

    assert len(calls) == 1
    url, kwargs = calls[0]
    assert url == "https://cloud.example/v1/audio/speech"
    assert kwargs["headers"]["Authorization"] == "Bearer cloud-token"
    assert kwargs["headers"]["Content-Type"] == "application/json"
    assert kwargs["json"] == {
        "input": "Ein Satz.",
        "language": "de",
        "voice": "M3",
    }
    assert "timeout" in kwargs
    assert queued_pcm(node) == [CLOUD_PCM]
    assert local_calls == []
    assert not log.warning.called


@pytest.mark.parametrize(
    "gender, expected",
    [
        ("Female", "F2"),
        ("Male", "M3"),
        ("female", "F2"),
        ("male", "M3"),
        ("Maennlich", "M3"),
        ("", "F2"),
        (None, "F2"),
    ],
)
def test_the_voice_dropdown_decides_the_cloud_voice(gender, expected):
    """The dialog offers Female and Male; Female speaks as F2 (Domi), Male as M3 (Adam).

    The robot sends the endpoint's own voice name, never a provider id - a raw id is
    rejected by /v1/audio/speech with 400.
    """
    _, ns, _, _ = build_player(post=None)
    assert ns["cloud_voice"](gender) == expected


def test_the_base_url_without_v1_still_reaches_the_speech_path(monkeypatch):
    calls = []

    def post(url, **kwargs):
        calls.append(url)
        return SimpleNamespace(status_code=200, content=wav_container(CLOUD_PCM))

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example")
    node, _, _, _ = build_player(post)
    node.play_audio_from_speech(request(), SimpleNamespace())
    assert calls == ["https://cloud.example/v1/audio/speech"]


def test_one_post_per_clause(monkeypatch):
    calls = []

    def post(url, **kwargs):
        calls.append(kwargs["json"]["input"])
        return SimpleNamespace(status_code=200, content=wav_container(CLOUD_PCM))

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, _ = build_player(post)
    node.play_audio_from_speech(
        request("Erster Satz. Zweiter Satz."), SimpleNamespace()
    )
    assert calls == ["Erster Satz.", "Zweiter Satz."]
    assert len(queued_pcm(node)) == 2


def test_english_and_female_voice_are_sent_as_en_and_f1(monkeypatch):
    calls = []

    def post(url, **kwargs):
        calls.append(kwargs["json"])
        return SimpleNamespace(status_code=200, content=wav_container(CLOUD_PCM))

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, _ = build_player(post)
    spoken = request("One sentence.")
    spoken.gender = "Female"
    spoken.language = "English"
    node.play_audio_from_speech(spoken, SimpleNamespace())
    assert calls[0]["language"] == "en"
    assert calls[0]["voice"] == "F2"


@pytest.mark.parametrize(
    "reply, reason",
    [
        (SimpleNamespace(status_code=402, content=b"{}"), "cloud_budget_exhausted:402"),
        (SimpleNamespace(status_code=401, content=b""), "cloud_auth_rejected:401"),
        (SimpleNamespace(status_code=403, content=b""), "cloud_auth_rejected:403"),
        (SimpleNamespace(status_code=200, content=b""), "cloud_empty_body"),
        (SimpleNamespace(status_code=502, content=b""), "cloud_http_error:502"),
        (requests.Timeout("too slow"), "cloud_transport_error"),
    ],
)
def test_cloud_failure_speaks_the_clause_locally_and_logs_once(
    monkeypatch, reply, reason
):
    calls = []

    def post(url, **kwargs):
        calls.append(url)
        if isinstance(reply, Exception):
            raise reply
        return reply

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, log, local_calls = build_player(post)

    node.play_audio_from_speech(request(), SimpleNamespace())

    assert len(calls) == 1
    assert queued_pcm(node) == [LOCAL_PCM]
    assert len(local_calls) == 1
    assert local_calls[0]["text"] == "Ein Satz."
    assert local_calls[0]["voice"] == "Male"
    assert log.warning.call_count == 1
    logged = log.warning.call_args.args[0]
    assert reason in logged
    assert "local Supertone" in logged


def test_a_fallback_is_logged_once_for_the_whole_turn(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, log, local_calls = build_player(
        lambda url, **kwargs: SimpleNamespace(status_code=402, content=b"")
    )

    node.play_audio_from_speech(
        request("Erster Satz. Zweiter Satz."), SimpleNamespace()
    )

    assert len(local_calls) == 2
    assert len(queued_pcm(node)) == 2
    assert log.warning.call_count == 1


def test_a_missing_key_store_token_falls_back_to_supertone(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, log, local_calls = build_player(
        lambda url, **kwargs: pytest.fail("cloud was called without a token"),
        token=None,
    )

    node.play_audio_from_speech(request(), SimpleNamespace())

    assert len(local_calls) == 1
    assert queued_pcm(node) == [LOCAL_PCM]
    assert "cloud_token_unavailable" in log.warning.call_args.args[0]


def test_an_explicit_local_engine_never_opens_the_cloud(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, log, local_calls = build_player(
        lambda url, **kwargs: pytest.fail("cloud used for a local engine")
    )

    node.play_audio_from_speech(request(engine="supertone"), SimpleNamespace())

    assert len(local_calls) == 1
    assert queued_pcm(node) == [LOCAL_PCM]
    assert not log.warning.called


def test_an_engine_without_a_speech_client_stays_silent(monkeypatch):
    node, _, log, local_calls = build_player(
        lambda url, **kwargs: pytest.fail("cloud used for an unsupported engine")
    )
    node.play_audio_from_speech(request(engine="some-provider"), SimpleNamespace())
    assert local_calls == []
    assert queued_pcm(node) == []
    assert log.error.called


def test_cloud_wav_sample_rate_is_preserved_in_playback_item_encoding(monkeypatch):
    """PR-2126: 16 kHz pib.Cloud WAV must play at 16,000 Hz, not hardcoded 44,100 Hz (2.75x chipmunk speed)."""
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, _ = build_player(
        lambda url, **kwargs: SimpleNamespace(
            status_code=200, content=wav_container(CLOUD_PCM, rate=16000)
        )
    )

    node.play_audio_from_speech(request("Hallo."), SimpleNamespace())

    item = node.playback_queue.get_nowait()
    assert item.encoding.frames_per_second == 16000
    assert item.encoding.num_channels == 1
    assert item.encoding.bytes_per_sample == 2
    assert b"".join(item.data) == CLOUD_PCM


def test_local_fallback_preserves_supertone_44100_sample_rate(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, _ = build_player(
        lambda url, **kwargs: SimpleNamespace(status_code=502, content=b"")
    )

    node.play_audio_from_speech(request("Hallo."), SimpleNamespace())

    item = node.playback_queue.get_nowait()
    assert item.encoding.frames_per_second == 44100
    assert item.encoding.num_channels == 1
    assert item.encoding.bytes_per_sample == 2
    assert b"".join(item.data) == LOCAL_PCM


def test_cloud_success_publishes_no_fallback_state(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, _ = build_player(
        lambda url, **kwargs: SimpleNamespace(
            status_code=200, content=wav_container(CLOUD_PCM)
        )
    )
    node.play_audio_from_speech(request(), SimpleNamespace())
    assert node.fallback_events == [False]


def test_cloud_failure_publishes_fallback_once_for_the_turn(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, _ = build_player(
        lambda url, **kwargs: SimpleNamespace(status_code=502, content=b"")
    )
    node.play_audio_from_speech(
        request("Erster Satz. Zweiter Satz."), SimpleNamespace()
    )
    assert node.fallback_events == [True]


def test_personality_voice_id_is_sent_when_set(monkeypatch):
    calls = []

    def post(url, **kwargs):
        calls.append(kwargs["json"])
        return SimpleNamespace(status_code=200, content=wav_container(CLOUD_PCM))

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, _ = build_player(post)
    spoken = request(voice_id="F2")
    spoken.gender = "Male"
    node.play_audio_from_speech(spoken, SimpleNamespace())
    assert calls[0]["voice"] == "F2"


def test_default_route_is_cloud_when_token_is_available(monkeypatch):
    calls = []

    def post(url, **kwargs):
        calls.append(url)
        return SimpleNamespace(status_code=200, content=wav_container(CLOUD_PCM))

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, _ = build_player(post)
    node.play_audio_from_speech(request(engine=""), SimpleNamespace())
    assert len(calls) == 1


def test_explicit_local_engine_never_opens_the_cloud_with_empty_stored_engine(
    monkeypatch,
):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, _, local_calls = build_player(
        lambda url, **kwargs: pytest.fail("cloud used for a local engine")
    )
    node.play_audio_from_speech(request(engine="supertone"), SimpleNamespace())
    assert len(local_calls) == 1
    assert True not in node.fallback_events


def test_cloud_timeout_falls_back_to_local(monkeypatch):
    class SlowPost:
        def __call__(self, url, **kwargs):
            timeout = kwargs.get("timeout")
            assert timeout == 3.0
            raise requests.Timeout("too slow")

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    monkeypatch.setenv("PIB_CLOUD_TTS_TIMEOUT_SECONDS", "3")
    node, _, log, local_calls = build_player(SlowPost())
    node.play_audio_from_speech(request(), SimpleNamespace())
    assert len(local_calls) == 1
    assert node.fallback_events == [True]
    assert "cloud_transport_error" in log.warning.call_args.args[0]


def test_cloud_turn_logs_speech_characters_and_usage_headers(monkeypatch):
    headers = {
        "X-Usage-Speech-Characters": "12",
        "X-Speech-Cost": "0.004",
    }

    def post(url, **kwargs):
        return SimpleNamespace(
            status_code=200,
            content=wav_container(CLOUD_PCM),
            headers=headers,
        )

    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example/v1")
    node, _, log, _ = build_player(post)
    node.play_audio_from_speech(request("Hallo Welt."), SimpleNamespace())
    logged = " ".join(str(c.args[0]) for c in log.info.call_args_list if c.args)
    assert "speech_characters_sent=11" in logged
    assert "speech_characters=12" in logged
    assert "speech_cost=0.004" in logged
