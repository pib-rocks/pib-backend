import io
import os
import struct
import sys
import wave
from pathlib import Path
import pytest

# Ensure voice_assistant package is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros_packages" / "voice_assistant"))

from voice_assistant.stt_transcription import FasterWhisperSTTEngine, DEFAULT_WHISPER_MODEL_SIZE


def generate_dummy_wav_bytes(duration_sec: float = 1.0, sample_rate: int = 16000) -> bytes:
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


class TestSTTEngineInitialization:
    """Test engine initialization and model configuration."""

    def test_default_model_size_resolution(self):
        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        assert engine.model_size == "base"

    def test_custom_env_var_model_size(self, monkeypatch: pytest.MonkeyPatch):
        monkeypatch.setenv("WHISPER_MODEL_SIZE", "base")
        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        assert engine.model_size == "base"

    def test_fallback_when_model_load_fails(self, monkeypatch: pytest.MonkeyPatch):
        # Force Exception during WhisperModel initialization to test fallback path
        def mock_init(*args, **kwargs):
            raise RuntimeError("Simulated model load failure")

        try:
            import faster_whisper
            monkeypatch.setattr(faster_whisper, "WhisperModel", mock_init)
        except ImportError:
            pass

        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        assert engine.is_loaded is False
        assert engine.active_backend == "fallback"

        wav_bytes = generate_dummy_wav_bytes(1.0)
        text, meta = engine.transcribe(wav_bytes)

        assert text == ""
        assert meta["backend"] == "fallback"


class TestSTTBoundaryConditions:
    """Test boundary conditions (empty bytes, silent audio, None inputs)."""

    def test_empty_audio_bytes(self):
        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        text, meta = engine.transcribe(b"")

        assert text == ""
        assert meta["backend"] == "empty_input"

    def test_silent_audio_transcription(self):
        engine = FasterWhisperSTTEngine(model_path="/non_existent_path")
        wav_bytes = generate_dummy_wav_bytes(0.5)
        text, meta = engine.transcribe(wav_bytes)

        assert text == ""
        assert "backend" in meta
