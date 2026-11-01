"""Unit tests for motor adapter USB sysfs reset (PR-2157)."""

from __future__ import annotations

import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
if str(PIB_MOTORS_DIR) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_DIR))

from pib_motors.port_usb_reset import (  # noqa: E402
    reset_cdc_acm_adapter,
    should_attempt_usb_reset,
    usb_bind_id_for_device,
    usb_reset_enabled,
)


def test_should_attempt_usb_reset_respects_the_guard() -> None:
    assert should_attempt_usb_reset(0, enabled=True) is False
    assert should_attempt_usb_reset(1, enabled=True) is True
    assert should_attempt_usb_reset(1, enabled=False) is False


def test_usb_reset_is_off_by_default(monkeypatch) -> None:
    monkeypatch.delenv("PIB_MOTORS_USB_RESET", raising=False)
    assert usb_reset_enabled() is False
    assert should_attempt_usb_reset(1) is False


def test_usb_bind_id_follows_sys_class_tty(tmp_path: Path) -> None:
    sys_class = tmp_path / "class" / "tty"
    tty_dir = sys_class / "ttyACM0"
    device_dir = tmp_path / "devices" / "1-2.3:1.0"
    device_dir.mkdir(parents=True)
    tty_dir.mkdir(parents=True)
    (tty_dir / "device").symlink_to(device_dir)

    assert (
        usb_bind_id_for_device("/dev/ttyACM0", sys_class_tty=sys_class) == "1-2.3:1.0"
    )


def test_reset_writes_unbind_then_bind(tmp_path: Path, caplog) -> None:
    driver_dir = tmp_path / "cdc_acm"
    driver_dir.mkdir()
    unbind = driver_dir / "unbind"
    bind = driver_dir / "bind"
    unbind.touch()
    bind.touch()

    sys_class = tmp_path / "class" / "tty"
    tty_dir = sys_class / "ttyTEST"
    device_dir = tmp_path / "devices" / "9-9.9:1.0"
    device_dir.mkdir(parents=True)
    tty_dir.mkdir(parents=True)
    (tty_dir / "device").symlink_to(device_dir)

    writes: list[tuple[str, str]] = []

    def record_write(path: Path, value: str) -> None:
        writes.append((path.name, value.strip()))

    with caplog.at_level(logging.ERROR):
        ok = reset_cdc_acm_adapter(
            "/dev/ttyTEST",
            driver_dir=driver_dir,
            sys_class_tty=sys_class,
            write_text=record_write,
            probe=lambda: True,
        )

    assert ok is True
    assert writes == [("unbind", "9-9.9:1.0"), ("bind", "9-9.9:1.0")]
    assert "PORT_USB_RESET" in caplog.text
    assert "stage=USB_RESET" in caplog.text


def test_zero_failures_never_touch_sysfs(tmp_path: Path) -> None:
    """Healthy ports must not reset USB even when the feature flag is on."""
    driver_dir = tmp_path / "cdc_acm"
    driver_dir.mkdir()
    (driver_dir / "unbind").touch()
    (driver_dir / "bind").touch()
    writes: list[str] = []

    def record_write(path: Path, value: str) -> None:
        writes.append(value)

    assert (
        reset_cdc_acm_adapter(
            "",
            driver_dir=driver_dir,
            write_text=record_write,
        )
        is False
    )
    assert writes == []
