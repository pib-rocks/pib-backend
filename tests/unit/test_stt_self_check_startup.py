"""STT start-up self-check must not block the audio recorder (PR-2054)."""

from __future__ import annotations

import sys
import threading
import time
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VOICE_ROOT))

from voice_assistant.stt_transcription import (  # noqa: E402
    FasterWhisperSTTEngine,
    SELF_CHECK_EXPECTED_TEXT,
    stt_self_check_enabled,
)


class _Segment:
    def __init__(self, text: str) -> None:
        self.text = text


class _Info:
    language = "de"
    language_probability = 1.0
    duration = 1.0


class _SlowModel:
    def __init__(self) -> None:
        self.started = threading.Event()
        self.release = threading.Event()

    def transcribe(self, audio, language=None, beam_size=5, vad_filter=True):
        self.started.set()
        assert self.release.wait(timeout=5.0)
        return [_Segment(SELF_CHECK_EXPECTED_TEXT)], _Info()


def _stub_engine(
    monkeypatch, tmp_path: Path, model: _SlowModel
) -> FasterWhisperSTTEngine:
    model_dir = tmp_path / "small"
    model_dir.mkdir(parents=True)
    (model_dir / "model.bin").write_bytes(b"stub")
    fake = types.ModuleType("faster_whisper")
    fake.WhisperModel = lambda *args, **kwargs: model
    monkeypatch.setitem(sys.modules, "faster_whisper", fake)
    return FasterWhisperSTTEngine(model_path=str(tmp_path))


def test_self_check_disabled_by_default(monkeypatch):
    monkeypatch.delenv("PIB_STT_SELF_CHECK", raising=False)
    assert stt_self_check_enabled() is False
    monkeypatch.setenv("PIB_STT_SELF_CHECK", "1")
    assert stt_self_check_enabled() is True
    monkeypatch.setenv("PIB_STT_SELF_CHECK", "0")
    assert stt_self_check_enabled() is False


def test_engine_loads_without_running_self_check_when_disabled(monkeypatch, tmp_path):
    monkeypatch.setenv("PIB_STT_SELF_CHECK", "0")
    model = _SlowModel()
    engine = _stub_engine(monkeypatch, tmp_path, model)

    assert engine.is_loaded is True
    assert engine.self_check_passed is False
    assert not model.started.is_set()


def test_background_self_check_does_not_block_caller(monkeypatch, tmp_path):
    monkeypatch.setenv("PIB_STT_SELF_CHECK", "1")
    model = _SlowModel()
    engine = _stub_engine(monkeypatch, tmp_path, model)

    started = time.monotonic()
    thread = engine.start_self_check_in_background()
    assert thread is not None
    elapsed = time.monotonic() - started
    assert elapsed < 0.5

    model.release.set()
    thread.join(timeout=5.0)
    assert engine.self_check_passed is True


def test_recorder_source_defers_self_check_to_background():
    text = (VOICE_ROOT / "voice_assistant" / "audio_recorder.py").read_text(
        encoding="utf-8"
    )
    assert "start_self_check_in_background" in text
    assert "FasterWhisperSTTEngine()" in text
