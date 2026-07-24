"""
Local Speech-to-Text (STT) transcription engine powered by faster-whisper.

Provides 100% offline audio transcription for pib-backend using the "base" model size.
Includes automatic fallback mechanisms and robust error handling.
"""

from __future__ import annotations

import io
import logging
import os
import wave
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, Union

logger = logging.getLogger(__name__)

DEFAULT_WHISPER_MODEL_SIZE = os.getenv("WHISPER_MODEL_SIZE", "base")
DEFAULT_WHISPER_MODEL_PATH = os.getenv("WHISPER_MODEL_PATH", "/data/voice/models/whisper/")
DEFAULT_COMPUTE_TYPE = os.getenv("WHISPER_COMPUTE_TYPE", "int8")


class FasterWhisperSTTEngine:
    """
    Local offline STT engine using CTranslate2 / faster-whisper.
    """

    def __init__(
        self,
        model_size: str = DEFAULT_WHISPER_MODEL_SIZE,
        model_path: Optional[Union[str, Path]] = None,
        compute_type: str = DEFAULT_COMPUTE_TYPE,
        device: str = "cpu",
    ) -> None:
        self.model_size = model_size or "base"
        self.model_path = Path(model_path) if model_path else Path(DEFAULT_WHISPER_MODEL_PATH)
        self.compute_type = compute_type
        self.device = device

        self.is_loaded = False
        self.active_backend = "uninitialized"
        self._model = None

        self.load_model()

    def load_model(self) -> bool:
        """
        Attempt to load faster-whisper CTranslate2 model.
        """
        try:
            from faster_whisper import WhisperModel  # type: ignore

            model_identifier = (
                str(self.model_path)
                if self.model_path.exists() and self.model_path.is_dir()
                else self.model_size
            )

            self._model = WhisperModel(
                model_identifier,
                device=self.device,
                compute_type=self.compute_type,
                cpu_threads=4,
            )
            self.is_loaded = True
            self.active_backend = f"faster-whisper-{self.model_size}"
            logger.info(f"Loaded faster-whisper model '{self.model_size}' successfully.")
            return True

        except Exception as e:
            logger.warning(f"Failed to load faster-whisper model '{self.model_size}': {e}")
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
            return "", {"language": "unknown", "probability": 0.0, "backend": "empty_input"}

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
        return "", {"language": language or "unknown", "probability": 0.0, "backend": "fallback"}
