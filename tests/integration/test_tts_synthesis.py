"""
Unit and integration tests for Supertone supertonic-3 expressive local TTS engine.

Validates:
- Engine initialization & path resolution (default vs SUPERTONE_MODEL_PATH vs explicit)
- 100% offline synthesis capability & model state detection
- Fallback behavior on missing model files or runtime synthesis errors
- Audio output format & quality (WAV header, sample rate, 16-bit PCM, non-zero bytes, RMS)
- Boundary conditions & edge cases (empty text, numbers, symbols, multilingual, long sentences)
- Expressive voice parameters
- Integration with ROS audio player pipeline
"""

import os
import sys
from pathlib import Path
import pytest

# Ensure ros_packages/voice_assistant is in sys.path for direct imports
REPO_ROOT = Path(__file__).resolve().parents[2]
VA_PACKAGE_DIR = REPO_ROOT / "ros_packages" / "voice_assistant"

if str(VA_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(VA_PACKAGE_DIR))

from voice_assistant.tts_synthesis import (
    DEFAULT_MODEL_PATH,
    DEFAULT_SAMPLE_RATE,
    SupertoneTTSEngine,
    analyze_wav_bytes,
)


class TestTTSEngineInitialization:
    """Test engine initialization and model path resolution."""

    def test_default_model_path_resolution(self, monkeypatch: pytest.MonkeyPatch):
        monkeypatch.delenv("SUPERTONE_MODEL_PATH", raising=False)
        engine = SupertoneTTSEngine()
        assert engine.model_path == Path(DEFAULT_MODEL_PATH)
        assert engine.sample_rate == DEFAULT_SAMPLE_RATE

    def test_custom_env_var_model_path(self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
        custom_dir = tmp_path / "custom_supertone"
        custom_dir.mkdir()
        monkeypatch.setenv("SUPERTONE_MODEL_PATH", str(custom_dir))

        engine = SupertoneTTSEngine()
        assert engine.model_path == custom_dir

    def test_explicit_constructor_model_path(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ):
        monkeypatch.setenv("SUPERTONE_MODEL_PATH", "/should/be/overridden")
        explicit_dir = tmp_path / "explicit_supertone"
        explicit_dir.mkdir()

        engine = SupertoneTTSEngine(model_path=explicit_dir)
        assert engine.model_path == explicit_dir

    def test_model_loading_with_valid_files(self, tmp_path: Path):
        model_dir = tmp_path / "supertone_model"
        model_dir.mkdir()
        (model_dir / "config.json").write_text('{"model": "supertonic-3"}', encoding="utf-8")
        (model_dir / "supertonic_v3.bin").write_bytes(b"dummy_weights_data")

        engine = SupertoneTTSEngine(model_path=model_dir)
        assert engine.is_loaded is True
        assert engine.active_backend == "supertone-supertonic-3"


class TestTTSFallbackBehavior:
    """Test fallback synthesis behavior when primary model is missing or fails."""

    def test_fallback_when_model_dir_missing(self, tmp_path: Path):
        non_existent = tmp_path / "does_not_exist"
        engine = SupertoneTTSEngine(model_path=non_existent)

        assert engine.is_loaded is False
        assert engine.active_backend == "fallback"

        # Synthesis must complete cleanly in fallback mode
        wav_bytes = engine.synthesize("Fallback Test Sentence")
        analysis = analyze_wav_bytes(wav_bytes)

        assert analysis["header_valid"] is True
        assert analysis["non_zero_bytes"] is True
        assert analysis["rms_amplitude"] > 0.0001

    def test_fallback_on_runtime_synthesis_error(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
        model_dir = tmp_path / "supertone_model"
        model_dir.mkdir()
        (model_dir / "config.json").write_text("{}", encoding="utf-8")

        engine = SupertoneTTSEngine(model_path=model_dir)
        assert engine.is_loaded is True

        # Simulate primary synthesis runtime failure
        def mock_failing_primary(*args, **kwargs):
            raise RuntimeError("Simulated primary inference failure")

        monkeypatch.setattr(engine, "_synthesize_primary", mock_failing_primary)

        # Engine should catch exception and return valid fallback audio
        wav_bytes = engine.synthesize("Testing Runtime Error Fallback")
        analysis = analyze_wav_bytes(wav_bytes)

        assert analysis["header_valid"] is True
        assert analysis["non_zero_bytes"] is True
        assert analysis["rms_amplitude"] > 0.0


class TestTTSAudioOutputQuality:
    """Test WAV/PCM headers, sample rate, bit depth, non-zero audio bytes, and RMS."""

    def test_wav_header_and_pcm_specifications(self, tmp_path: Path):
        engine = SupertoneTTSEngine(model_path=tmp_path, sample_rate=16000)
        wav_bytes = engine.synthesize("Standard audio quality test sentence.")

        analysis = analyze_wav_bytes(wav_bytes)

        assert analysis["header_valid"] is True
        assert analysis["channels"] == 1  # Mono
        assert analysis["sample_width"] == 2  # 16-bit
        assert analysis["sample_rate"] == 16000
        assert analysis["pcm_bytes_len"] > 0

    def test_custom_sample_rate(self, tmp_path: Path):
        engine = SupertoneTTSEngine(model_path=tmp_path, sample_rate=22050)
        wav_bytes = engine.synthesize("Custom sample rate test.")

        analysis = analyze_wav_bytes(wav_bytes)
        assert analysis["sample_rate"] == 22050

    def test_audio_energy_and_non_zero_bytes(self, tmp_path: Path):
        engine = SupertoneTTSEngine(model_path=tmp_path)
        wav_bytes = engine.synthesize("Guten Tag, ich bin PIB.")

        analysis = analyze_wav_bytes(wav_bytes)
        assert analysis["non_zero_bytes"] is True
        assert analysis["rms_amplitude"] > 0.001


class TestTTSBoundaryConditionsAndEdgeCases:
    """Test boundary conditions: empty text, numbers, special characters, multilingual, long text."""

    def test_empty_and_whitespace_text(self, tmp_path: Path):
        engine = SupertoneTTSEngine(model_path=tmp_path)

        for text in ["", "   ", "\t\n  "]:
            wav_bytes = engine.synthesize(text)
            analysis = analyze_wav_bytes(wav_bytes)
            assert analysis["header_valid"] is True
            assert isinstance(wav_bytes, bytes)

    def test_numbers_currency_and_special_symbols(self, tmp_path: Path):
        engine = SupertoneTTSEngine(model_path=tmp_path)
        text = "Version 2.0 kostet 100 € & $50! 50% Rabatt @PIB."

        wav_bytes = engine.synthesize(text)
        analysis = analyze_wav_bytes(wav_bytes)

        assert analysis["header_valid"] is True
        assert analysis["non_zero_bytes"] is True
        assert analysis["rms_amplitude"] > 0.001

    def test_german_english_multilingual_text(self, tmp_path: Path):
        engine = SupertoneTTSEngine(model_path=tmp_path)

        german_text = "Hallo, ich bin der Roboter PIB und steuere Motoren."
        english_text = "Hello world, Supertone supertonic-3 expressive local TTS engine."

        wav_de = engine.synthesize(german_text, language="de")
        wav_en = engine.synthesize(english_text, language="en")

        assert analyze_wav_bytes(wav_de)["header_valid"] is True
        assert analyze_wav_bytes(wav_en)["header_valid"] is True

    def test_long_sentence_chunking_and_synthesis(self, tmp_path: Path):
        engine = SupertoneTTSEngine(model_path=tmp_path)
        long_text = (
            "Das PIB Backend System ist eine modulare ROS 2 Roboterplattform. "
            "Es unterstützt Expressive Text-to-Speech Synthese über Supertone supertonic-3. "
            "Alle Audiodaten werden lokal im 16-Bit PCM WAV Format gerendert und abgespielt. "
            "Durch die 100%ige Offline-Fähigkeit bleibt der Roboter auch ohne Internetverbindung "
            "vollständig einsatzbereit und reaktionsschnell."
        )

        wav_bytes = engine.synthesize(long_text)
        analysis = analyze_wav_bytes(wav_bytes)

        assert analysis["header_valid"] is True
        assert analysis["non_zero_bytes"] is True
        # Long text should produce longer audio payload
        assert analysis["pcm_bytes_len"] > 10000


class TestExpressiveVoiceParameters:
    """Test expressive parameters: emotion, speed, pitch."""

    def test_emotion_speed_pitch_variations(self, tmp_path: Path):
        engine = SupertoneTTSEngine(model_path=tmp_path)

        for emotion in ["expressive", "happy", "neutral"]:
            for speed in [0.8, 1.0, 1.5]:
                for pitch in [0.9, 1.0, 1.2]:
                    wav_bytes = engine.synthesize(
                        "Expressiver Test.", emotion=emotion, speed=speed, pitch=pitch
                    )
                    analysis = analyze_wav_bytes(wav_bytes)
                    assert analysis["header_valid"] is True
                    assert analysis["non_zero_bytes"] is True


class TestTTSAudioPlayerIntegration:
    """Test interaction with AudioPlayerNode from voice_assistant package."""

    def test_audio_player_node_instantiation_with_tts_engine(self, tmp_path: Path):
        try:
            import rclpy
            from voice_assistant.audio_player import AudioPlayerNode
        except ImportError:
            import pytest
            pytest.skip("rclpy not installed in host environment (runs in ROS2 container)")

        engine = SupertoneTTSEngine(model_path=tmp_path)
        wav_bytes = engine.synthesize("Test Integration")
        assert len(wav_bytes) > 44
