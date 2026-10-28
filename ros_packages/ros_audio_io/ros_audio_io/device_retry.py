"""Pure retry and status helpers for microphone device connections.

Retry backoff in seconds: 1, 2, 4, 8, 16, then 30 capped for every later attempt.
"""

MAX_RETRY_DELAY_SECONDS = 30.0
STATUS_OWNER = "ros-audio-io"
INPUT_FORMAT_S16LE = "S16_LE"


class DeviceNotFoundError(Exception):
    """Raised when no matching or default microphone device exists."""


def next_retry_delay(attempt):
    """Return an exponential retry delay capped at 30 seconds."""

    if attempt < 1:
        raise ValueError("attempt must be at least 1")
    if attempt >= 6:
        return MAX_RETRY_DELAY_SECONDS
    return float(2 ** (attempt - 1))


def describe_open_failure(exc):
    """Return the stable operator-facing reason for a device-open failure."""

    if isinstance(exc, DeviceNotFoundError):
        return "no matching input device"
    if getattr(exc, "errno", None) == -9999 or "[Errno -9999]" in str(exc):
        return "device busy"
    if getattr(exc, "errno", None) == -9994 or "[Errno -9994]" in str(exc):
        return "sample format not supported"
    return "device open failure"


def capture_backend_name(host_api_name):
    """Map a PortAudio host API label to the short backend name used in logs."""

    label = str(host_api_name).lower()
    if "pulse" in label:
        return "pulse"
    return "alsa"


def format_input_capabilities(device_info, *, audio_format=INPUT_FORMAT_S16LE):
    """Summarize PortAudio input facts for operator-facing failure logs."""

    name = str(device_info.get("name", ""))
    max_in = int(device_info.get("maxInputChannels", 0))
    default_rate = int(float(device_info.get("defaultSampleRate", 0)))
    return (
        f"maxInputChannels={max_in} defaultSampleRate={default_rate} "
        f"format={audio_format} name={name!r}"
    )


def audio_device_log_line(
    backend, device_name, rate, channels, audio_format=INPUT_FORMAT_S16LE
):
    """Structured start-up line aligned with voice_assistant open_speaker logging."""

    return (
        f"audio_device={backend} device={device_name} "
        f"rate={rate} channels={channels} format={audio_format}"
    )


def open_failure_signature(*, device_index, device_name, reason, detail):
    """Key for deduplicating identical open failures across retries."""

    return (device_index, device_name, reason, detail)


def format_microphone_open_failure_log(
    *,
    reason,
    detail,
    device_name,
    capabilities,
    attempt,
    delay,
):
    """One readable operator line when the microphone cannot be opened."""

    return (
        f"Microphone open failed: reason={reason}; detail={detail}; "
        f"device={device_name}; advertised={capabilities}; "
        f"attempt={attempt}; next attempt in {delay:g}s"
    )


def microphone_recovery_log(device_name, rate, channels, backend="alsa"):
    """Log line when capture becomes available after a prior failure."""

    configured = audio_device_log_line(backend, device_name, rate, channels)
    return f"Microphone input recovered: {configured}"


def device_status_payload(
    *,
    available,
    reason=None,
    detail=None,
    attempts=None,
    next_retry_in_seconds=None,
    device_name=None,
    channels=None,
    rate=None,
    processed_channel=None,
):
    """Build an honest microphone status payload without unavailable facts."""

    payload = {"available": bool(available)}
    if available:
        facts = {
            "deviceName": device_name,
            "channels": channels,
            "rate": rate,
            "processedChannel": processed_channel,
        }
        if any(value is None for value in facts.values()):
            raise ValueError("available status requires all device facts")
        payload.update(facts)
    else:
        failure = {
            "reason": reason,
            "detail": detail,
            "attempts": attempts,
            "nextRetryInSeconds": next_retry_in_seconds,
        }
        if any(value is None for value in failure.values()):
            raise ValueError("unavailable status requires complete failure details")
        payload.update(failure)
    payload["owner"] = STATUS_OWNER
    return payload
