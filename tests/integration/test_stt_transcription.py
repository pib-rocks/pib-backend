import io
import struct
import sys
import types
import wave
from pathlib import Path

import pytest

# Ensure voice_assistant package is in sys.path
sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "ros_packages" / "voice_assistant")
)

from voice_assistant.stt_transcription import (
    FasterWhisperSTTEngine,
    DEFAULT_WHISPER_MODEL_SIZE,
    SELF_CHECK_EXPECTED_TEXT,
    STTTranscriptionError,
    UNAVAILABLE_BACKEND,
)

SAMPLE = (
    Path(__file__).resolve().parents[2]
    / "ros_packages"
    / "voice_assistant"
    / "assets"
    / "stt_self_check.wav"
)
METADATA_ERRORS = (
    "TypeError: open() got an unexpected keyword argument 'metadata_errors'"
)


def generate_dummy_wav_bytes(
    duration_sec: float = 1.0, sample_rate: int = 16000
) -> bytes:
    """Generate a dummy 16kHz PCM WAV byte stream for testing."""
    num_samples = int(sample_rate * duration_sec)
    raw_pcm = struct.pack(f"<{num_samples}h", *[0] * num_samples)

    buf = io.BytesIO()
    with wave.open(buf, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(raw_pcm)

    return buf.getvalue()


class _Segment:
    def __init__(self, text: str) -> None:
        self.text = text


class _Info:
    language = "de"
    language_probability = 1.0
    duration = 1.0


class _FakeModel:
    """One scripted answer per transcribe() call."""

    def __init__(self, answers) -> None:
        self.answers = list(answers)

    def transcribe(self, audio, language=None, beam_size=5, vad_filter=True):
        answer = self.answers.pop(0)
        if isinstance(answer, BaseException):
            raise answer
        return [_Segment(answer)] if answer else [], _Info()


def _engine_around(monkeypatch: pytest.MonkeyPatch, tmp_path: Path, model):
    model_dir = tmp_path / "small"
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / "model.bin").write_bytes(b"stub")
    fake = types.ModuleType("faster_whisper")
    fake.WhisperModel = lambda *args, **kwargs: model
    monkeypatch.setitem(sys.modules, "faster_whisper", fake)
    monkeypatch.setenv("WHISPER_SELF_CHECK_SAMPLE", str(SAMPLE))
    return FasterWhisperSTTEngine(model_path=str(tmp_path))


class TestSTTEngineInitialization:
    """Test engine initialization and model configuration."""

    def test_default_model_size_resolution(self, monkeypatch: pytest.MonkeyPatch):
        monkeypatch.delenv("WHISPER_MODEL_SIZE", raising=False)
        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        assert engine.model_size == "small"
        assert engine.compute_type == "int8"

    def test_custom_env_var_model_size(self, monkeypatch: pytest.MonkeyPatch):
        monkeypatch.setenv("WHISPER_MODEL_SIZE", "medium")
        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        assert engine.model_size == "medium"
        assert engine.model_size != DEFAULT_WHISPER_MODEL_SIZE

    def test_an_unusable_engine_is_named_and_not_reported_as_silence(
        self, monkeypatch: pytest.MonkeyPatch
    ):
        """The old fallback returned ("", {"backend": "fallback"}) here.

        That made a broken engine look exactly like a user who said nothing
        (PC-35). The engine now reports the state and raises with its cause.
        """
        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        assert engine.is_loaded is False
        assert engine.active_backend == UNAVAILABLE_BACKEND
        assert "not provisioned" in engine.engine_failure

        wav_bytes = generate_dummy_wav_bytes(1.0)
        with pytest.raises(STTTranscriptionError) as raised:
            engine.transcribe(wav_bytes)

        assert "not provisioned" in raised.value.cause

    def test_a_broken_decode_stack_fails_the_start_up_self_check(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ):
        model = _FakeModel([TypeError(METADATA_ERRORS)])
        engine = _engine_around(monkeypatch, tmp_path, model)

        assert engine.self_check_passed is False
        assert "metadata_errors" in engine.engine_failure


class TestSTTBoundaryConditions:
    """Test boundary conditions (empty bytes, silent audio, None inputs)."""

    def test_empty_audio_bytes(self):
        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        text, meta = engine.transcribe(b"")

        assert text == ""
        assert meta["backend"] == "empty_input"

    def test_silent_audio_transcription(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ):
        """Silence through a working engine is still an empty transcript."""
        model = _FakeModel([SELF_CHECK_EXPECTED_TEXT, ""])
        engine = _engine_around(monkeypatch, tmp_path, model)

        wav_bytes = generate_dummy_wav_bytes(0.5)
        text, meta = engine.transcribe(wav_bytes)

        assert text == ""
        assert meta["backend"] == "faster-whisper-small"
