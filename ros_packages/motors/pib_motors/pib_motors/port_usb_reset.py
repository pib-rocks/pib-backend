"""Sysfs unbind/bind recovery for a wedged CDC-ACM USB serial adapter.

A reopen only drops the host file handle. When the bridge firmware stops answering
but stays enumerated, the driver must detach and reattach the interface - the same
effect as unplugging the adapter. Kept behind an explicit enable flag so a healthy
bus never sees a reset.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Callable

_LOG = logging.getLogger(__name__)

DEFAULT_CDC_ACM_DRIVER_DIR = Path("/sys/bus/usb/drivers/cdc_acm")
DEFAULT_SYS_CLASS_TTY = Path("/sys/class/tty")

WriteText = Callable[[Path, str], None]


def usb_reset_enabled() -> bool:
    """True when the operator opted in via ``PIB_MOTORS_USB_RESET``."""
    return os.environ.get("PIB_MOTORS_USB_RESET", "").strip().lower() in (
        "1",
        "true",
        "yes",
    )


def should_attempt_usb_reset(
    reopens_without_success: int,
    *,
    enabled: bool | None = None,
) -> bool:
    """Only after at least one reopen failed, and only when explicitly enabled."""
    if enabled is None:
        enabled = usb_reset_enabled()
    if not enabled:
        return False
    return reopens_without_success >= 1


def usb_bind_id_for_device(
    device: str,
    sys_class_tty: Path = DEFAULT_SYS_CLASS_TTY,
) -> str | None:
    """Return the sysfs node name written to the driver's unbind file (e.g. ``1-2.3:1.0``)."""
    cleaned = device.strip()
    if not cleaned:
        return None
    try:
        tty_name = Path(cleaned).resolve().name
    except OSError:
        tty_name = Path(cleaned).name
    device_link = sys_class_tty / tty_name / "device"
    try:
        if not device_link.exists():
            return None
        return device_link.resolve().name
    except OSError:
        return None


def reset_cdc_acm_adapter(
    device: str,
    *,
    driver_dir: Path = DEFAULT_CDC_ACM_DRIVER_DIR,
    sys_class_tty: Path = DEFAULT_SYS_CLASS_TTY,
    write_text: WriteText | None = None,
    probe: Callable[[], bool] | None = None,
) -> bool:
    """Unbind then bind the adapter's USB interface under ``cdc_acm``.

    ``write_text`` and ``sys_class_tty`` are injectable so unit tests never touch
    real sysfs. An optional ``probe`` runs after bind to confirm the port answers.
    """
    bind_id = usb_bind_id_for_device(device, sys_class_tty=sys_class_tty)
    if not bind_id:
        _LOG.error(
            "[PERF_TRACE] PORT_USB_RESET device=%s stage=USB_RESET outcome=missing_sysfs_id",
            device,
        )
        return False

    writer = write_text or _write_sysfs
    unbind_path = driver_dir / "unbind"
    bind_path = driver_dir / "bind"
    try:
        writer(unbind_path, bind_id)
        writer(bind_path, bind_id)
    except OSError as error:
        _LOG.error(
            "[PERF_TRACE] PORT_USB_RESET device=%s stage=USB_RESET bind_id=%s outcome=failed "
            "error=%s",
            device,
            bind_id,
            error,
        )
        return False

    if probe is not None and not probe():
        _LOG.error(
            "[PERF_TRACE] PORT_USB_RESET device=%s stage=USB_RESET bind_id=%s "
            "outcome=probe_failed",
            device,
            bind_id,
        )
        return False

    _LOG.error(
        "[PERF_TRACE] PORT_USB_RESET device=%s stage=USB_RESET bind_id=%s outcome=ok",
        device,
        bind_id,
    )
    return True


def _write_sysfs(path: Path, value: str) -> None:
    path.write_text(f"{value}\n", encoding="ascii")
