"""The installed launch file must import its sibling module (deploy bug on .172)."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
LAUNCH_DIR = REPO / "ros_packages" / "voice_assistant" / "launch"


def _write_package(root: Path, module: str, names: list[str]) -> None:
    """Create ``module`` as a package (or submodule) exposing ``names``."""
    parts = module.split(".")
    pkg = root
    for part in parts[:-1]:
        pkg = pkg / part
        pkg.mkdir(parents=True, exist_ok=True)
        (pkg / "__init__.py").touch()
    target = pkg / (parts[-1] + ".py")
    target.write_text(
        "\n".join(f"{n} = object" for n in names) + "\n", encoding="utf-8"
    )


def _write_top_package(root: Path, module: str, names: list[str]) -> None:
    pkg = root / module
    pkg.mkdir(parents=True, exist_ok=True)
    (pkg / "__init__.py").write_text(
        "\n".join(f"{n} = object" for n in names) + "\n", encoding="utf-8"
    )


def test_launch_file_imports_with_only_its_own_directory(tmp_path):
    """The launch file runs from share/voice_assistant/launch, not from the repo.

    A plain ``from node_supervision import ...`` fails there because the launch
    context does not put the file's directory on sys.path - measured on .172 as
    "ModuleNotFoundError: No module named 'node_supervision'" in a restart loop.
    """
    install_dir = tmp_path / "launch"
    install_dir.mkdir()
    for source in LAUNCH_DIR.glob("*.py"):
        shutil.copy2(source, install_dir / source.name)

    stubs = tmp_path / "stubs"
    _write_top_package(stubs, "launch", ["LaunchDescription"])
    _write_top_package(stubs, "launch_ros", [])
    _write_package(
        stubs, "launch.actions", ["LogInfo", "RegisterEventHandler", "TimerAction"]
    )
    _write_package(stubs, "launch.event_handlers", ["OnProcessExit"])
    _write_package(stubs, "launch_ros.actions", ["Node"])

    script = (
        "import importlib.util, sys;"
        f"spec = importlib.util.spec_from_file_location('installed_launch', r'{install_dir / 'launch.py'}');"
        "mod = importlib.util.module_from_spec(spec);"
        "spec.loader.exec_module(mod);"
        "print('ok')"
    )

    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=tmp_path,
        env={"PYTHONPATH": str(stubs), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout
