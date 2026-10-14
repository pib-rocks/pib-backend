"""The local engine is told which language it transcribes (PC-39).

Measured on the test robot with the shipped weights: a request without a
language makes faster-whisper run its own detection pass, which cost about
half the turn (8 s recording: 37.0 s vs 19.2 s), reported a much less certain
language (p=0.78 instead of 1.00) and answered silence with 'en' p=0.34.

These tests pin the wiring: one helper resolves the shipped default and the
overrides, the recorder hands the result to the engine, and the engine passes
it through instead of letting the library decide.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VOICE_ROOT))

from pib_hermes_config.voice_backends import (  # noqa: E402
    DEFAULT_STT_LANGUAGE,
    STT_LANGUAGE_ENV,
    configured_stt_language,
    whisper_language_code,
)
from voice_assistant.stt_transcription import (  # noqa: E402
    SELF_CHECK_EXPECTED_TEXT,
    FasterWhisperSTTEngine,
)

RECORDER = VOICE_ROOT / "voice_assistant" / "audio_recorder.py"
DOCKERFILE = VOICE_ROOT / "Dockerfile"


class _Model:
    """Stand-in for faster_whisper.WhisperModel that records its arguments."""

    def __init__(self, answers) -> None:
        self.answers = list(answers)
        self.calls: list[dict] = []

    def transcribe(self, audio, language=None, beam_size=5, vad_filter=True):
        self.calls.append({"language": language, "beam_size": beam_size})
        answer = self.answers.pop(0)
        return [types.SimpleNamespace(text=answer)], types.SimpleNamespace(
            language=language or "de", language_probability=1.0, duration=1.0
        )


def _engine(monkeypatch, tmp_path: Path, model: _Model) -> FasterWhisperSTTEngine:
    model_dir = tmp_path / "small"
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / "model.bin").write_bytes(b"stub")
    fake = types.ModuleType("faster_whisper")
    fake.WhisperModel = lambda *args, **kwargs: model
    monkeypatch.setitem(sys.modules, "faster_whisper", fake)
    return FasterWhisperSTTEngine(model_path=str(tmp_path))


def test_language_codes_accept_codes_and_spelled_names():
    assert whisper_language_code("de") == "de"
    assert whisper_language_code("de-DE") == "de"
    assert whisper_language_code("German") == "de"
    assert whisper_language_code("Deutsch") == "de"
    assert whisper_language_code("deu") == "de"
    assert whisper_language_code("en_US") == "en"
    assert whisper_language_code("English") == "en"


def test_an_unusable_language_leaves_detection_alone():
    # Guessing here would silently transcribe in the wrong language.
    assert whisper_language_code("") == ""
    assert whisper_language_code("   ") == ""
    assert whisper_language_code(None) == ""
    assert whisper_language_code(42) == ""
    assert whisper_language_code("auto") == ""
    assert whisper_language_code("Klingon") == ""


def test_the_shipped_default_is_pinned_and_can_be_overridden(monkeypatch):
    monkeypatch.delenv(STT_LANGUAGE_ENV, raising=False)
    assert configured_stt_language() == DEFAULT_STT_LANGUAGE == "de"

    monkeypatch.setenv(STT_LANGUAGE_ENV, "English")
    assert configured_stt_language() == "en"

    monkeypatch.setenv(STT_LANGUAGE_ENV, "auto")
    assert configured_stt_language() == ""

    monkeypatch.setenv(STT_LANGUAGE_ENV, "Klingon")
    assert configured_stt_language() == ""


def test_the_image_ships_the_pinned_language():
    text = DOCKERFILE.read_text(encoding="utf-8")
    assert "ENV STT_LANGUAGE=de" in text
    assert "STT_LANGUAGE=auto" in text


def test_the_recorder_hands_its_language_to_the_engine():
    source = RECORDER.read_text(encoding="utf-8")
    assert "configured_stt_language()" in source
    assert "self.stt_language" in source
    assert "language=self.stt_language or None" in source
    # The turn log names the language that was used, so a turn can be read back.
    assert "language " in source


def test_the_engine_passes_the_language_through(monkeypatch, tmp_path):
    model = _Model(
        [SELF_CHECK_EXPECTED_TEXT, "Wie viele Arme hast du?", "Und noch etwas."]
    )
    engine = _engine(monkeypatch, tmp_path, model)
    # The start-up self-check pins German on purpose.
    assert model.calls[0]["language"] == "de"

    engine.transcribe(b"RIFF....WAVEfmt ", language="de")
    assert model.calls[-1]["language"] == "de"

    engine.transcribe(b"RIFF....WAVEfmt ", language=None)
    assert model.calls[-1]["language"] is None


def test_a_missing_language_is_the_library_default_not_an_empty_string(
    monkeypatch, tmp_path
):
    """faster-whisper must see None (detect), never '' (a broken request)."""
    model = _Model([SELF_CHECK_EXPECTED_TEXT, "Hallo."])
    engine = _engine(monkeypatch, tmp_path, model)
    engine.transcribe(b"RIFF....WAVEfmt ", language=None)
    assert model.calls[-1]["language"] is None


def test_the_budget_note_stays_honest():
    """The repo budget is 700 ms; a pinned language must not claim it is met."""
    import yaml

    document = yaml.safe_load(
        (REPO_ROOT / "voice" / "whisper-model.yaml").read_text(encoding="utf-8")
    )
    assert document["first_token_budget_ms"] == 700
    # Not re-measured for small, and not claimed anywhere else.
    assert document["first_token_budget_rechecked_for_small"] is False


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(pytest.main([__file__, "-q"]))
