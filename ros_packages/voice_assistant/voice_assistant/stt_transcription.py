"""
Local Speech-to-Text (STT) transcription engine powered by faster-whisper.

The default size is ``small``. ``medium`` is a permitted override, not the
default: the yardstick is the smallest shipped unit, a pib5edu with 4045 MB
of RAM. Compute type stays ``int8`` and the engine keeps four CPU threads.

Weights are read from ``/data/voice/models/whisper/`` (see
``voice/whisper-model.yaml``). That path is not ``models/manifest.yaml``,
which lists the camera blobs only. A missing or empty directory is not a
download: the engine does not load by model name.
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
CPU_THREADS = 4
CT2_MODEL_FILE = "model.bin"


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
        self._model = None

        self.load_model()

    def load_model(self) -> bool:
        """
        Attempt to load faster-whisper CTranslate2 model.
        """
        directory = resolve_whisper_directory(self.model_path, self.model_size)
        if directory is None:
            logger.warning(
                "faster-whisper model '%s' is not provisioned under %s; "
                "refusing to download it.",
                self.model_size,
                self.model_path,
            )
            self.is_loaded = False
            self.active_backend = "fallback"
            return False

        try:
            from faster_whisper import WhisperModel  # type: ignore

            self._model = WhisperModel(
                str(directory),
                device=self.device,
                compute_type=self.compute_type,
                cpu_threads=CPU_THREADS,
            )
            self.is_loaded = True
            self.active_backend = f"faster-whisper-{self.model_size}"
            logger.info(
                f"Loaded faster-whisper model '{self.model_size}' successfully."
            )
            return True

        except Exception as e:
            logger.warning(
                f"Failed to load faster-whisper model '{self.model_size}': {e}"
            )
            self.is_loaded = False
            self.active_backend = "fallback"
            return False

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
        """
        if not audio_input:
            return "", {
                "language": "unknown",
                "probability": 0.0,
                "backend": "empty_input",
            }

        if self.is_loaded and self._model is not None:
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

                text_parts = [
                    segment.text.strip() for segment in segments if segment.text
                ]
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

        # Fallback transcription
        return self._fallback_transcribe(audio_input, language)

    def _fallback_transcribe(
        self,
        audio_input: Union[bytes, io.BytesIO, str, Path],
        language: Optional[str] = None,
    ) -> Tuple[str, Dict[str, Any]]:
        """
        Fallback path when primary engine is uninitialized or fails.
        """
        return "", {
            "language": language or "unknown",
            "probability": 0.0,
            "backend": "fallback",
        }
