"""A local STT engine that cannot transcribe fails visibly (PC-35).

Measured defect (test robot 192.168.1.172, 2026-10-14): the voice image
installed faster-whisper 1.2.1 with PyAV 19.0.1. faster-whisper decodes via
``av.open(..., metadata_errors="ignore")`` and PyAV 19.0.0 removed that
keyword, so every turn raised

    stt_transcription.py:191: Error during primary faster-whisper
    transcription: open() got an unexpected keyword argument 'metadata_errors'

and the ``except`` branch returned ``("", {"backend": "fallback"})``. A
broken engine was therefore indistinguishable from a user who said nothing:
the chat got the canned "I could not hear you" line and no ``is_user`` row.

These tests pin the new contract: an unusable engine raises
``STTTranscriptionError`` carrying the technical cause, real silence still
transcribes to an empty string, and the start-up self-check proves the
decode stack once per process.
"""

from __future__ import annotations

import sys
import types
import wave
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VOICE_ROOT))

from voice_assistant.stt_transcription import (  # noqa: E402
    SELF_CHECK_EXPECTED_TEXT,
    SELF_CHECK_SAMPLE_NAME,
    STTTranscriptionError,
    UNAVAILABLE_BACKEND,
    FasterWhisperSTTEngine,
    resolve_self_check_sample,
    self_check_matches,
    visible_failure_hint,
)
from pib_hermes_config.visible_state import (
    VOICE_STT_FAILURE_TOPIC,
    engine_is_fallback,
)  # noqa: E402

RECORDER = VOICE_ROOT / "voice_assistant" / "audio_recorder.py"
ASSISTANT = VOICE_ROOT / "voice_assistant" / "assistant.py"
DOCKERFILE = VOICE_ROOT / "Dockerfile"
SAMPLE = VOICE_ROOT / "assets" / SELF_CHECK_SAMPLE_NAME

#: The measured failure string this story fixes.
METADATA_ERRORS = (
    "TypeError: open() got an unexpected keyword argument 'metadata_errors'"
)


class _Segment:
    def __init__(self, text: str) -> None:
        self.text = text


class _Info:
    language = "de"
    language_probability = 1.0
    duration = 3.6


class _ScriptedModel:
    """Stands in for faster_whisper.WhisperModel without the dependency.

    Each call takes the next entry of ``answers``: a string becomes one
    segment, a ``[]`` becomes no speech at all, and an exception instance is
    raised the way the real decode stack raises it.
    """

    def __init__(self, answers) -> None:
        self.answers = list(answers)
        self.calls: list[dict] = []

    def transcribe(self, audio, language=None, beam_size=5, vad_filter=True):
        self.calls.append({"language": language, "beam_size": beam_size})
        answer = self.answers.pop(0)
        if isinstance(answer, BaseException):
            raise answer
        return [_Segment(answer)] if answer else [], _Info()


def _build_engine(monkeypatch, tmp_path: Path, model: _ScriptedModel):
    """A real engine around a scripted model, pointed at the bundled sample."""
    model_dir = tmp_path / "small"
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / "model.bin").write_bytes(b"stub")
    fake = types.ModuleType("faster_whisper")
    fake.WhisperModel = lambda *args, **kwargs: model
    monkeypatch.setitem(sys.modules, "faster_whisper", fake)
    monkeypatch.setenv("WHISPER_SELF_CHECK_SAMPLE", str(SAMPLE))
    engine = FasterWhisperSTTEngine(model_path=str(tmp_path))
    engine.run_self_check()
    return engine


def test_the_bundled_sample_exists_and_is_16khz_mono():
    assert SAMPLE.is_file(), SAMPLE
    assert resolve_self_check_sample() == SAMPLE
    with wave.open(str(SAMPLE), "rb") as wav:
        assert wav.getnchannels() == 1
        assert wav.getsampwidth() == 2
        assert wav.getframerate() == 16000
        assert wav.getnframes() > 16000


def test_self_check_passes_and_names_the_engine(monkeypatch, tmp_path):
    model = _ScriptedModel([SELF_CHECK_EXPECTED_TEXT])
    engine = _build_engine(monkeypatch, tmp_path, model)

    assert engine.self_check_passed is True
    assert engine.engine_failure is None
    assert engine.active_backend == "faster-whisper-small"
    # The self-check goes through the same path as a turn: de, beam 1, the
    # bundled sample bytes.
    assert model.calls == [{"language": "de", "beam_size": 1}]


def test_a_broken_decode_stack_is_marked_unavailable_with_its_cause(
    monkeypatch, tmp_path
):
    """The measured 'metadata_errors' TypeError must not become a silence."""
    model = _ScriptedModel([TypeError(METADATA_ERRORS)])
    engine = _build_engine(monkeypatch, tmp_path, model)

    assert engine.is_loaded is False
    assert engine.self_check_passed is False
    assert engine.active_backend == UNAVAILABLE_BACKEND
    assert "metadata_errors" in engine.engine_failure
    assert engine_is_fallback(engine.active_backend) is True


def test_transcribe_raises_instead_of_returning_an_empty_text(monkeypatch, tmp_path):
    """This is the assertion that was red before the fix.

    Before PC-35 an unusable engine answered ``("", {"backend": "fallback"})``,
    which the chat turned into the canned "I could not hear you" line.
    """
    model = _ScriptedModel([TypeError(METADATA_ERRORS)])
    engine = _build_engine(monkeypatch, tmp_path, model)

    with pytest.raises(STTTranscriptionError) as raised:
        engine.transcribe(b"RIFF....WAVEfmt ", language="de")

    assert "metadata_errors" in raised.value.cause
    assert raised.value.backend == UNAVAILABLE_BACKEND
    assert "metadata_errors" in raised.value.hint


def test_a_failing_turn_raises_with_the_cause_of_that_turn(monkeypatch, tmp_path):
    """A working start-up self-check does not excuse a later decode failure."""
    model = _ScriptedModel([SELF_CHECK_EXPECTED_TEXT, RuntimeError("decode exploded")])
    engine = _build_engine(monkeypatch, tmp_path, model)

    assert engine.self_check_passed is True
    with pytest.raises(STTTranscriptionError) as raised:
        engine.transcribe(b"RIFF....WAVEfmt ", language="de")

    assert "RuntimeError: decode exploded" == raised.value.cause
    assert raised.value.backend == "faster-whisper-small"


def test_real_silence_still_transcribes_to_an_empty_text(monkeypatch, tmp_path):
    """An empty transcript from a working engine is not an engine failure."""
    model = _ScriptedModel([SELF_CHECK_EXPECTED_TEXT, ""])
    engine = _build_engine(monkeypatch, tmp_path, model)

    text, metadata = engine.transcribe(b"RIFF....WAVEfmt ", language="de")

    assert text == ""
    assert metadata["backend"] == "faster-whisper-small"


def test_a_missing_model_is_a_named_failure_not_an_empty_text(tmp_path):
    engine = FasterWhisperSTTEngine(model_path=str(tmp_path / "not-provisioned"))

    assert engine.is_loaded is False
    assert engine.active_backend == UNAVAILABLE_BACKEND
    assert "not provisioned" in engine.engine_failure
    with pytest.raises(STTTranscriptionError) as raised:
        engine.transcribe(b"RIFF....WAVEfmt ")
    assert "not provisioned" in raised.value.cause


def test_a_self_check_mismatch_is_a_named_failure(monkeypatch, tmp_path):
    model = _ScriptedModel(["Guten Morgen, ich bin ein Roboter."])
    engine = _build_engine(monkeypatch, tmp_path, model)

    assert engine.self_check_passed is False
    assert "expected" in engine.engine_failure
    assert SELF_CHECK_EXPECTED_TEXT in engine.engine_failure


def test_self_check_matching_ignores_case_and_punctuation():
    assert self_check_matches(SELF_CHECK_EXPECTED_TEXT)
    assert self_check_matches(SELF_CHECK_EXPECTED_TEXT.upper())
    assert self_check_matches("Hallo ich bin ein Roboter und ich kann sprechen")
    assert not self_check_matches("Guten Morgen")


def test_the_hint_names_the_technical_cause():
    hint = visible_failure_hint(METADATA_ERRORS)

    assert METADATA_ERRORS in hint
    assert "Spracherkennung" in hint
    # An empty cause must not produce a hint that names nothing.
    assert "unbekannte Ursache" in visible_failure_hint("   ")


def test_the_recorder_reports_the_failure_and_aborts_the_goal():
    source = RECORDER.read_text(encoding="utf-8")

    assert "STTTranscriptionError" in source
    assert "_publish_stt_failure(" in source
    assert VOICE_STT_FAILURE_TOPIC in source or "VOICE_STT_FAILURE_TOPIC" in source
    assert (
        "engine {metadata.get('backend')}" in source
        or "metadata.get('backend')" in source
    )
    # The failure path aborts, so no empty transcript reaches the chat.
    assert "goal_handle.abort()" in source


def test_the_assistant_speaks_the_cause_instead_of_the_canned_line():
    source = ASSISTANT.read_text(encoding="utf-8")

    assert "VOICE_STT_FAILURE_TOPIC" in source
    assert "_on_stt_failure" in source
    assert "visible_failure_hint(" in source
    assert "self._stt_failure_cause" in source


def test_the_image_pins_an_av_that_still_accepts_metadata_errors():
    """PyAV 19.0.0 removed the keyword faster-whisper 1.2.1 still passes."""
    text = DOCKERFILE.read_text(encoding="utf-8")

    assert "faster-whisper==1.2.1" in text
    assert "av==18.1.0" in text
    pinned = [line for line in text.splitlines() if line.strip().startswith("av==")]
    assert len(pinned) == 1, pinned
    major = int(pinned[0].strip().split("==")[1].split(".")[0])
    assert major < 19, pinned
    assert "metadata_errors" in text or "PyAV" in text
