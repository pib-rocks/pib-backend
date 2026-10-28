"""rclpy loggers reject stdlib logging options such as ``exc_info``.

Passing them raises TypeError from inside the error path, which kills the node
and hides the original failure (measured on .172: an ALSA error turned into
`TypeError: parameter "exc_info" is not one of the recognized logging options`).
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "ros_packages" / "voice_assistant"


def _is_get_logger_call(node: ast.AST) -> bool:
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "get_logger"
    )


def test_no_rclpy_logger_call_passes_exc_info():
    offenders: list[str] = []
    for path in sorted(ROOT.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(
                node.func, ast.Attribute
            ):
                continue
            if not _is_get_logger_call(node.func.value):
                continue
            for keyword in node.keywords:
                if keyword.arg in {"exc_info", "stack_info"}:
                    offenders.append(
                        f"{path.relative_to(ROOT)}:{node.lineno} {keyword.arg}"
                    )
    assert offenders == [], offenders
