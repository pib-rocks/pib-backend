"""Visible cloud-to-local fallback; no empty successful failed turn (PC-44)."""

from __future__ import annotations

import time
from voice_assistant.cloud_stt import CloudSTTError, transcribe
from voice_assistant.stt_transcription import STTTranscriptionError


def transcribe_cloud_or_local(
    wav_data, *, language, local_engine, log, publish_fallback, publish_failure
):
    try:
        result = transcribe(wav_data, language=language)
    except CloudSTTError as cloud:
        # Publish before potentially slow local inference, never clear this on
        # local success. A later successful cloud request clears it.
        publish_fallback("fallback")
        log.warning(
            f"cloud STT request_id={cloud.request_id} status=fallback_local "
            f"cause={cloud.cause} http_status={cloud.http_status} "
            f"duration_ms={cloud.duration_ms:.3f} audio_bytes={len(wav_data)}"
        )
        started = time.monotonic()
        try:
            text, metadata = local_engine.transcribe(wav_data, language=language)
            text = text.strip()
            if not text:
                raise STTTranscriptionError(
                    "local fallback empty", cause="local_empty_text"
                )
        except Exception as exc:
            local_cause = (
                exc.cause
                if isinstance(exc, STTTranscriptionError)
                else type(exc).__name__
            )
            cause = f"cloud STT {cloud.cause} (HTTP {cloud.http_status}); local STT {local_cause}"
            publish_fallback("unavailable")
            publish_failure(cause)
            log.error(f"STT total failure request_id={cloud.request_id}: {cause}")
            raise STTTranscriptionError("both STT paths failed", cause=cause) from None
        log.info(
            f"local STT request_id={cloud.request_id} status=fallback_local "
            f"duration_ms={(time.monotonic() - started) * 1000:.3f} "
            f"audio_bytes={len(wav_data)}"
        )
        return text
    publish_fallback("cloud")
    log.info(
        f"cloud STT request_id={result.request_id} status=ok "
        f"http_status={result.http_status} duration_ms={result.duration_ms:.3f} "
        f"audio_bytes={len(wav_data)}"
    )
    return result.text
