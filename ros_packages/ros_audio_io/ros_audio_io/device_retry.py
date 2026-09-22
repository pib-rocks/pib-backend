"""Pure retry and status helpers for microphone device connections."""

MAX_RETRY_DELAY_SECONDS = 30.0
STATUS_OWNER = "ros-audio-io"


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
    return "device open failure"


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
