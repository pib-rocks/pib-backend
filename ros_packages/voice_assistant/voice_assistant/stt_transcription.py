"""
Local Speech-to-Text (STT) transcription engine powered by faster-whisper.

The default size is ``small``. ``medium`` is a permitted override, not the
default: the yardstick is the smallest shipped unit, a pib5edu with 4045 MB
of RAM. Compute type stays ``int8`` and the engine keeps three CPU threads -
three, because the decode does not get faster on the fourth core and holding
all four starves the rest of the robot (PC-39, ``CPU_THREADS``).

Weights are read from ``/data/voice/models/whisper/`` (see
``voice/whisper-model.yaml``). That path is not ``models/manifest.yaml``,
which lists the camera blobs only. A missing or empty directory is not a
download: the engine does not load by model name.

A broken engine is never reported as an empty transcript. ``transcribe``
raises ``STTTranscriptionError`` carrying the technical cause, and a start-up
self-check transcribes a bundled sample so a mismatched dependency pair
(faster-whisper against its PyAV audio backend) shows up at node start
instead of at the user's first sentence.
"""

from __future__ import annotations

import io
import logging
import os
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, Union

logger = logging.getLogger(__name__)

#: Shipped default. ``medium`` may be set through WHISPER_MODEL_SIZE.
DEFAULT_WHISPER_MODEL_SIZE = "small"
DEFAULT_WHISPER_MODEL_PATH = "/data/voice/models/whisper/"
DEFAULT_COMPUTE_TYPE = "int8"
#: Three, not four: the Pi has four cores and the decode does not get faster
#: with all of them. Measured on the test robot with the shipped weights and an
#: 8 s recording (PC-39): 4 threads 11.9 s, 3 threads 9.6 s, 2 threads 9.3 s -
#: and holding every core starves the rest of the container (rosbridge's
#: parameter reads time out while a turn is being transcribed, see PC-37).
CPU_THREADS = 3
CT2_MODEL_FILE = "model.bin"

#: ``active_backend`` while the engine cannot transcribe at all.
UNAVAILABLE_BACKEND = "unavailable"

#: Bundled utterance the start-up self-check transcribes through the same
#: decode path a real turn uses. The file is the sample; the text is what
#: the shipped model must answer with.
SELF_CHECK_SAMPLE_NAME = "stt_self_check.wav"
#: Measured on the test robot (2026-10-14, faster-whisper 1.2.1 + av 18.1.0):
#: the bundled sample transcribes to exactly this sentence, language de,
#: language probability 1.0. Punctuation differences do not fail the check.
SELF_CHECK_EXPECTED_TEXT = "Hallo, ich bin ein Roboter und ich kann sprechen."
SELF_CHECK_SAMPLE_ENV = "WHISPER_SELF_CHECK_SAMPLE"


class STTTranscriptionError(RuntimeError):
    """The local engine could not transcribe.

    Raised instead of returning an empty string: an engine that cannot start
    and a user who said nothing must not reach the caller as the same thing.
    ``cause`` carries the technical reason, so a caller can name it out loud.
    """

    def __init__(self, message: str, *, cause: object, backend: str = "") -> None:
        super().__init__(message)
        self.cause = str(cause).strip() or "unknown cause"
        self.backend = backend or UNAVAILABLE_BACKEND

    @property
    def hint(self) -> str:
        """The spoken sentence a caller can use to make this visible."""
        return visible_failure_hint(self.cause)


def visible_failure_hint(cause: object) -> str:
    """Spoken line naming the technical cause of a failed transcription."""
    detail = str(cause).strip() or "unbekannte Ursache"
    return (
        "Die lokale Spracherkennung konnte den Satz nicht transkribieren. "
        f"Technische Ursache: {detail}"
    )


def self_check_sample_candidates() -> list[Path]:
    """Where the bundled self-check sample may sit, in order of preference."""
    candidates: list[Path] = []
    configured = os.getenv(SELF_CHECK_SAMPLE_ENV)
    if configured and configured.strip():
        candidates.append(Path(configured.strip()))
    package_dir = Path(__file__).resolve().parent
    candidates.append(package_dir / "assets" / SELF_CHECK_SAMPLE_NAME)
    candidates.append(package_dir.parent / "assets" / SELF_CHECK_SAMPLE_NAME)
    assistant_dir = os.getenv("VOICE_ASSISTANT_DIR")
    if assistant_dir and assistant_dir.strip():
        candidates.append(
            Path(assistant_dir.strip()) / "assets" / SELF_CHECK_SAMPLE_NAME
        )
    return candidates


def resolve_self_check_sample() -> Optional[Path]:
    """The bundled sample utterance, or None when it is missing."""
    for candidate in self_check_sample_candidates():
        if candidate.is_file():
            return candidate
    return None


def self_check_matches(text: str, expected: str = SELF_CHECK_EXPECTED_TEXT) -> bool:
    """Compare a self-check transcript to the expected sentence.

    Punctuation and case are stripped: the check proves the decode stack and
    the model work together, not that the model punctuates like the sample.
    """

    def normalize(value: str) -> str:
        stripped = "".join(
            char if char.isalnum() or char.isspace() else " " for char in value
        )
        return " ".join(stripped.lower().split())

    return normalize(text) == normalize(expected)


def configured_model_size() -> str:
    """Size selected for this process. Unset means the shipped default."""
    raw = os.getenv("WHISPER_MODEL_SIZE")
    if raw is None or not raw.strip():
        return DEFAULT_WHISPER_MODEL_SIZE
    return raw.strip()


def configured_compute_type() -> str:
    raw = os.getenv("WHISPER_COMPUTE_TYPE")
    if raw is None or not raw.strip():
        return DEFAULT_COMPUTE_TYPE
    return raw.strip()


def configured_model_path() -> str:
    raw = os.getenv("WHISPER_MODEL_PATH")
    if raw is None or not raw.strip():
        return DEFAULT_WHISPER_MODEL_PATH
    return raw.strip()


def resolve_whisper_directory(model_path: Path, model_size: str) -> Optional[Path]:
    """Directory that already holds a CTranslate2 model, or None.

    A size subdirectory (``small/``, ``medium/``) wins when it contains
    ``model.bin``. Otherwise the flat store path is used when it contains
    ``model.bin``. A name such as ``small`` is never returned: passing that
    name to faster-whisper downloads on first use.
    """
    sized = model_path / model_size / CT2_MODEL_FILE
    if sized.is_file():
        return sized.parent
    flat = model_path / CT2_MODEL_FILE
    if flat.is_file():
        return model_path
    return None


class FasterWhisperSTTEngine:
    """
    Local offline STT engine using CTranslate2 / faster-whisper.
    """

    def __init__(
        self,
        model_size: Optional[str] = None,
        model_path: Optional[Union[str, Path]] = None,
        compute_type: Optional[str] = None,
        device: str = "cpu",
    ) -> None:
        self.model_size = model_size or configured_model_size()
        self.model_path = (
            Path(model_path) if model_path else Path(configured_model_path())
        )
        self.compute_type = compute_type or configured_compute_type()
        self.device = device

        self.is_loaded = False
        self.active_backend = "uninitialized"
        #: Technical cause of the last hard failure, None while the engine works.
        self.engine_failure: Optional[str] = None
        self.self_check_passed = False
        self._model = None

        self.load_model()

    def load_model(self) -> bool:
        """
        Attempt to load faster-whisper CTranslate2 model.
        """
        directory = resolve_whisper_directory(self.model_path, self.model_size)
        if directory is None:
            self._mark_failed(
                f"faster-whisper model '{self.model_size}' is not provisioned "
                f"under {self.model_path}; refusing to download it"
            )
            logger.warning(
                "faster-whisper model '%s' is not provisioned under %s; "
                "refusing to download it.",
                self.model_size,
                self.model_path,
            )
            return False

        try:
            from faster_whisper import WhisperModel  # type: ignore

            self._model = WhisperModel(
                str(directory),
                device=self.device,
                compute_type=self.compute_type,
                cpu_threads=CPU_THREADS,
            )
        except Exception as e:
            self._mark_failed(
                f"faster-whisper model '{self.model_size}' failed to load "
                f"from {directory}: {type(e).__name__}: {e}"
            )
            logger.warning(
                f"Failed to load faster-whisper model '{self.model_size}': {e}"
            )
            return False

        self.is_loaded = True
        self.active_backend = f"faster-whisper-{self.model_size}"
        self.engine_failure = None
        logger.info(f"Loaded faster-whisper model '{self.model_size}' successfully.")
        return self.run_self_check()

    def _mark_failed(self, cause: str) -> None:
        """The engine cannot transcribe. Remember why, and say so."""
        self.is_loaded = False
        self.active_backend = UNAVAILABLE_BACKEND
        self.engine_failure = cause

    def run_self_check(self) -> bool:
        """Prove once, at start-up, that this build can decode and transcribe.

        The bundled sample goes through the same PyAV / CTranslate2 decode
        path a real turn uses, so a version mismatch between faster-whisper
        and its audio backend fails here - visibly, with the technical cause -
        instead of silently at the user's first sentence.
        """
        sample = resolve_self_check_sample()
        if sample is None:
            self._mark_failed(
                "bundled self-check sample "
                f"'{SELF_CHECK_SAMPLE_NAME}' is missing (looked in "
                + ", ".join(str(path) for path in self_check_sample_candidates())
                + ")"
            )
            logger.error(f"STT self-check: {self.engine_failure}")
            return False

        try:
            with open(sample, "rb") as sample_file:
                audio = sample_file.read()
            text, _metadata = self.transcribe(audio, language="de", beam_size=1)
        except Exception as e:
            self._mark_failed(
                f"self-check transcription of {sample} failed: "
                f"{type(e).__name__}: {e}"
            )
            logger.error(f"STT self-check: {self.engine_failure}")
            return False

        if not self_check_matches(text):
            self._mark_failed(
                f"self-check transcribed {text!r} from {sample}, "
                f"expected {SELF_CHECK_EXPECTED_TEXT!r}"
            )
            logger.error(f"STT self-check: {self.engine_failure}")
            return False

        self.self_check_passed = True
        logger.info(
            f"STT self-check passed: {sample} transcribed to "
            f"'{text.strip()}' via {self.active_backend}."
        )
        return True

    def transcribe(
        self,
        audio_input: Union[bytes, io.BytesIO, str, Path],
        language: Optional[str] = None,
        beam_size: int = 5,
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Transcribe audio buffer or WAV file into text string.

        Args:
            audio_input: WAV bytes, io.BytesIO, or file path.
            language: Target language code ('de', 'en', or None for auto-detect).
            beam_size: Beam search width (default: 5).

        Returns:
            Tuple of (transcribed_text, metadata_dict)

        Raises:
            STTTranscriptionError: the local engine cannot transcribe. An
                empty string is never used to report a broken engine - it
                means the engine ran and heard nothing.
        """
        if not audio_input:
            return "", {
                "language": "unknown",
                "probability": 0.0,
                "backend": "empty_input",
            }

        if not (self.is_loaded and self._model is not None):
            raise STTTranscriptionError(
                f"local speech engine '{self.model_size}' is not available",
                cause=self.engine_failure
                or f"engine state '{self.active_backend}' cannot transcribe",
                backend=self.active_backend,
            )

        try:
            # Prepare audio stream or filepath
            if isinstance(audio_input, bytes):
                audio_stream = io.BytesIO(audio_input)
            elif isinstance(audio_input, (str, Path)):
                audio_stream = str(audio_input)
            else:
                audio_stream = audio_input

            segments, info = self._model.transcribe(
                audio_stream,
                language=language,
                beam_size=beam_size,
                vad_filter=True,
            )

            text_parts = [segment.text.strip() for segment in segments if segment.text]
            full_text = " ".join(text_parts).strip()

            metadata = {
                "language": getattr(info, "language", language or "unknown"),
                "probability": getattr(info, "language_probability", 1.0),
                "duration": getattr(info, "duration", 0.0),
                "backend": self.active_backend,
            }
            return full_text, metadata

        except Exception as e:
            logger.error(f"Error during primary faster-whisper transcription: {e}")
            raise STTTranscriptionError(
                f"primary faster-whisper transcription failed: {e}",
                cause=f"{type(e).__name__}: {e}",
                backend=self.active_backend,
            ) from e
