"""rclpy loggers take exactly one message argument.

A printf-style call (``logger.warning("x %s", exc)``) raises
``TypeError: RcutilsLogger.warning() takes 2 positional arguments but 3 were
given`` at call time. On the voice path that killed the assistant node: the
voice assistant could not be switched on at all (measured on the test robot,
see PC-34). This guard walks the ROS packages and fails on any call that would
raise, so the class of defect cannot come back unnoticed.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
ROS_PACKAGES = REPO_ROOT / "ros_packages"

LOGGER_LEVELS = frozenset(
    {"debug", "info", "warn", "warning", "error", "fatal", "critical"}
)


def _logger_calls(path: Path) -> list[tuple[int, str, int]]:
    """(line, level, positional argument count) for every get_logger() call."""
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(path))
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Attribute) or func.attr not in LOGGER_LEVELS:
            continue
        target = ast.get_source_segment(source, func) or ""
        if "get_logger()" not in target.replace(" ", ""):
            continue
        found.append((node.lineno, func.attr, len(node.args)))
    return found


def _python_files() -> list[Path]:
    return sorted(
        path for path in ROS_PACKAGES.rglob("*.py") if "__pycache__" not in path.parts
    )


def test_the_scan_actually_looks_at_files():
    files = _python_files()
    assert len(files) > 50, f"scan found only {len(files)} files under {ROS_PACKAGES}"


def test_no_logger_call_passes_more_than_one_argument():
    offenders = []
    for path in _python_files():
        for line, level, arg_count in _logger_calls(path):
            if arg_count > 1:
                offenders.append(
                    f"{path.relative_to(REPO_ROOT)}:{line} logger.{level}() "
                    f"with {arg_count} positional arguments"
                )
    assert not offenders, (
        "rclpy loggers take one message argument; these calls raise TypeError "
        "at runtime:\n" + "\n".join(offenders)
    )


@pytest.mark.parametrize("level", sorted(LOGGER_LEVELS))
def test_the_detector_flags_a_printf_style_call(tmp_path: Path, level: str):
    """The scan must be able to see the defect it guards against."""
    sample = tmp_path / "sample.py"
    sample.write_text(
        f'node.get_logger().{level}("did not finish within 30s: %s", text)\n',
        encoding="utf-8",
    )
    calls = _logger_calls(sample)
    assert calls == [(1, level, 2)]

    clean = tmp_path / "clean.py"
    clean.write_text(f'node.get_logger().{level}(f"x {{text}}")\n', encoding="utf-8")
    assert _logger_calls(clean) == [(1, level, 1)]
