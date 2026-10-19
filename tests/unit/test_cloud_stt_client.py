"""Real-TCP contract/deadline and executed recorder routing (PC-44)."""

from __future__ import annotations

import ast
import contextlib
import json
import socket
import sys
import threading
import time
import types
import uuid
import wave
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock, Event

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros_packages/voice_assistant"))
from voice_assistant import cloud_stt as client
from voice_assistant import cloud_stt_dispatch as dispatch
from voice_assistant.stt_transcription import STTTranscriptionError
from pib_hermes_config.voice_backends import resolve_stt_route

AUDIO = b"RIFF-AUDIO-PRIVATE-CANARY"
TRANSCRIPT = "private transcript canary"


@contextlib.contextmanager
def server(status=200, body=None, drip=False):
    seen = []
    data = (
        json.dumps({"text": "  " + TRANSCRIPT + "  "}).encode()
        if body is None
        else body
    )

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            payload = self.rfile.read(int(self.headers["Content-Length"]))
            seen.append((self.path, dict(self.headers), payload))
            self.send_response(status)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            try:
                for b in data if drip else [data]:
                    self.wfile.write(bytes([b]) if drip else b)
                    self.wfile.flush()
                    if drip:
                        time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    http = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=http.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{http.server_port}/v1", seen
    finally:
        http.shutdown()
        http.server_close()
        thread.join(2)


@pytest.fixture(autouse=True)
def config(monkeypatch):
    monkeypatch.setenv("PIB_CLOUD_MODE", "test")
    monkeypatch.setenv("PIB_CLOUD_STT_TIMEOUT_SECONDS", "3")
    monkeypatch.setenv("FLASK_API_BASE_URL", "http://not-the-cloud.invalid")
    monkeypatch.setenv("PIB_CLOUD_API_KEY", "secret-credential-canary")


def test_real_tcp_upload_and_correlation(monkeypatch):
    with server() as (url, seen):
        monkeypatch.setenv("PIB_CLOUD_BASE_URL", url)
        result = client.transcribe(AUDIO, language="de")
    assert result.text == TRANSCRIPT
    assert result.duration_ms > 0
    assert result.http_status == 200
    assert str(uuid.UUID(result.request_id)) == result.request_id
    path, headers, body = seen[0]
    assert path == "/v1/voice-assistant/speech-to-text"
    assert headers["X-Request-ID"] == result.request_id
    assert "Authorization" not in headers
    assert "secret-credential-canary" not in repr(headers)
    assert AUDIO in body
    assert b'name="language"' in body and b"de" in body
    assert b'name="file"; filename="speech.wav"' in body


@pytest.mark.parametrize(
    "status,body,cause",
    [
        (502, b"secret-credential-canary", "cloud_http_error"),
        (302, b"", "cloud_http_error"),
        (200, b"not json", "cloud_invalid_json"),
        (200, b'{"text": 3}', "cloud_invalid_text"),
        (200, b'{"text": "  "}', "cloud_empty_text"),
        (200, b"x" * 65537, "cloud_response_too_large"),
    ],
)
def test_failure_is_typed_and_contains_no_body(monkeypatch, status, body, cause):
    with server(status, body) as (url, seen):
        monkeypatch.setenv("PIB_CLOUD_BASE_URL", url)
        with pytest.raises(client.CloudSTTError) as exc:
            client.transcribe(AUDIO)
    assert exc.value.cause == cause
    assert exc.value.http_status == status
    assert "secret-credential-canary" not in str(exc.value)
    assert len(seen) == 1


def test_total_deadline_catches_a_dripping_response(monkeypatch):
    with server(drip=True) as (url, _):
        monkeypatch.setenv("PIB_CLOUD_BASE_URL", url)
        monkeypatch.setenv("PIB_CLOUD_STT_TIMEOUT_SECONDS", "0.15")
        start = time.monotonic()
        with pytest.raises(client.CloudSTTError) as exc:
            client.transcribe(AUDIO)
        elapsed = time.monotonic() - start
    assert exc.value.cause == "cloud_timeout"
    assert elapsed < 0.6, elapsed
    assert elapsed >= 0.1


@pytest.mark.parametrize("raw", ["nan", "inf", "0", "-1", "abc"])
def test_invalid_deadline_never_opens_transport(monkeypatch, raw):
    monkeypatch.setenv("PIB_CLOUD_STT_TIMEOUT_SECONDS", raw)
    monkeypatch.setattr(
        client.httpx, "AsyncClient", lambda **kw: pytest.fail("opened transport")
    )
    with pytest.raises(client.CloudSTTError, match="invalid_cloud_stt_timeout"):
        client.transcribe(AUDIO)


@pytest.mark.parametrize(
    "url",
    [
        "http://user:secret@host/v1",
        "http://host/v1?key=secret",
        "http://host/v1#secret",
        "ftp://host/v1",
        "http://host/other",
    ],
)
def test_invalid_destination_never_sends_audio(monkeypatch, url):
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", url)
    monkeypatch.setattr(
        client.httpx, "AsyncClient", lambda **kw: pytest.fail("opened transport")
    )
    with pytest.raises(client.CloudSTTError, match="invalid_cloud_stt_url"):
        client.transcribe(AUDIO)


def test_production_requires_tls_and_flask_is_not_destination(monkeypatch):
    monkeypatch.delenv("PIB_CLOUD_BASE_URL", raising=False)
    assert (
        client.speech_to_text_url()
        == "https://api.pib.rocks/v1/voice-assistant/speech-to-text"
    )
    monkeypatch.setenv("PIB_CLOUD_MODE", "production")
    # The shipped default is TLS now, so production no longer refuses it.
    assert (
        client.speech_to_text_url()
        == "https://api.pib.rocks/v1/voice-assistant/speech-to-text"
    )
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "http://cloud.example")
    with pytest.raises(ValueError):
        client.speech_to_text_url()
    monkeypatch.setenv("PIB_CLOUD_BASE_URL", "https://cloud.example")
    assert (
        client.speech_to_text_url()
        == "https://cloud.example/v1/voice-assistant/speech-to-text"
    )


class Log:
    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(message)

    warning = error = info


def recorder(tmp_path, monkeypatch, local_text="local sentence", local_error=None):
    # Compile the actual production record_audio method, not a rewritten test
    # implementation. Only ROS message/types are substituted; its routing,
    # failure handling, WAV bytes and goal lifecycle all execute unchanged.
    path = ROOT / "ros_packages/voice_assistant/voice_assistant/audio_recorder.py"
    tree = ast.parse(path.read_text())
    cls = next(
        n
        for n in tree.body
        if isinstance(n, ast.ClassDef) and n.name == "AudioRecorderNode"
    )
    method = next(
        n
        for n in cls.body
        if isinstance(n, ast.FunctionDef) and n.name == "record_audio"
    )
    module = ast.Module(
        body=[
            ast.ImportFrom(
                module="__future__", names=[ast.alias(name="annotations")], level=0
            ),
            method,
        ],
        type_ignores=[],
    )
    namespace = dict(
        os=__import__("os"),
        time=time,
        wave=wave,
        OUTPUT_FILE_PATH=str(tmp_path / "capture.wav"),
        BYTES_PER_SAMPLE=2,
        resolve_stt_route=resolve_stt_route,
        transcribe_cloud_or_local=dispatch.transcribe_cloud_or_local,
        STTTranscriptionError=STTTranscriptionError,
        RecordAudio=types.SimpleNamespace(Feedback=lambda: None),
    )
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    events = []
    log = Log()
    calls = []

    def transcribe(data, *, language):
        events.append("local")
        calls.append((data, language))
        if local_error:
            raise local_error
        return local_text, {"backend": "faster-whisper-small", "language": "de"}

    node = types.SimpleNamespace(
        audio_chunks=deque(),
        audio_chunks_lock=Lock(),
        audio_chunk_event=Event(),
        mic_channels=1,
        sample_rate=16000,
        stt_language="de",
        stt_engine=types.SimpleNamespace(
            transcribe=transcribe, active_backend="faster-whisper-small"
        ),
        get_next_chunk=lambda: b"\x00\x00" * 1600,
        is_silent=lambda _: True,
        _emit_end_of_speech=lambda: None,
        _publish_engine_fallback=lambda value: events.append(value),
        _publish_stt_failure=lambda value: events.append(("failure", value)),
        get_logger=lambda: log,
        create_result=lambda text: types.SimpleNamespace(transcribed_text=text),
    )

    def run(engine="cloud_whisper"):
        goal = types.SimpleNamespace(
            request=types.SimpleNamespace(
                max_silent_seconds_before=0.01,
                max_silent_seconds_after=0.01,
                stt_engine=engine,
            ),
            is_cancel_requested=False,
            publish_feedback=lambda _: None,
            abort=lambda: events.append("abort"),
            succeed=lambda: events.append("succeed"),
        )
        result = namespace["record_audio"](node, goal)
        return result

    return run, events, calls, log


def test_recorder_cloud_success_never_calls_local_and_clears_fallback(
    tmp_path, monkeypatch
):
    run, events, calls, log = recorder(tmp_path, monkeypatch)
    monkeypatch.setattr(
        dispatch,
        "transcribe",
        lambda data, **kw: client.CloudSTTResult(
            TRANSCRIPT, "correlation-1", 12.5, 200
        ),
    )
    result = run()
    assert result.transcribed_text == TRANSCRIPT
    assert events == ["cloud", "succeed"] and not calls
    assert "correlation-1" in log.lines[0]
    assert TRANSCRIPT not in repr(log.lines)


@pytest.mark.parametrize(
    "failure",
    ["cloud_connection_error", "cloud_timeout", "cloud_http_error", "cloud_empty_text"],
)
def test_recorder_fallback_then_recovery(tmp_path, monkeypatch, failure):
    run, events, calls, log = recorder(tmp_path, monkeypatch)

    def broken(data, **kw):
        raise client.CloudSTTError(failure, "correlation-2", 3, 502)

    monkeypatch.setattr(dispatch, "transcribe", broken)
    result = run()
    assert result.transcribed_text == "local sentence"
    assert events == ["fallback", "local", "succeed"]
    assert len(calls) == 1 and calls[0][1] == "de"
    assert calls[0][0] == (tmp_path / "capture.wav").read_bytes()
    assert "fallback_local" in repr(log.lines) and "correlation-2" in repr(log.lines)
    assert "local sentence" not in repr(log.lines)
    monkeypatch.setattr(
        dispatch,
        "transcribe",
        lambda data, **kw: client.CloudSTTResult("recovered", "recovery", 1, 200),
    )
    assert run().transcribed_text == "recovered"
    assert events[-2:] == ["cloud", "succeed"]


@pytest.mark.parametrize("empty", [False, True])
def test_recorder_total_failure_is_visible_and_aborts(tmp_path, monkeypatch, empty):
    run, events, calls, log = recorder(
        tmp_path,
        monkeypatch,
        local_text="" if empty else "local",
        local_error=(
            None if empty else STTTranscriptionError("broken", cause="model_missing")
        ),
    )

    def broken(data, **kw):
        raise client.CloudSTTError("cloud_timeout", "correlation-3", 3000)

    monkeypatch.setattr(dispatch, "transcribe", broken)
    assert run().transcribed_text == ""
    assert "abort" in events and "succeed" not in events
    causes = [e[1] for e in events if isinstance(e, tuple)]
    assert len(causes) == 1 and "cloud_timeout" in causes[0]
    assert ("local_empty_text" if empty else "model_missing") in causes[0]
    assert events.index("fallback") < events.index("local")


def test_unsupported_engine_never_uploads(tmp_path, monkeypatch):
    run, events, calls, log = recorder(tmp_path, monkeypatch)
    monkeypatch.setattr(
        dispatch, "transcribe", lambda *a, **kw: pytest.fail("foreign upload")
    )
    assert run("foreign_engine").transcribed_text == ""
    assert events == ["abort"] and not calls
