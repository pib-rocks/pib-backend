"""Shell-level proof of the display web runner. Chromium is a stub on PATH."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO = REPO_ROOT / "tests" / "unit" / "display_web_runner_scenario.sh"

REQUIRED = {
    "FIRST_OPEN_BROWSERS": "1",
    "SECOND_OPEN_BROWSERS": "0",
    "HIDE_TERMINATED": "yes",
    "PIDFILE_AFTER_HIDE": "absent",
    "FAILING_STUB_REQUEST": "absent",
    "FAILING_STUB_STATUS": "failed",
}


def test_stub_chromium_is_idempotent_and_cleans_up_on_failure():
    result = subprocess.run(
        ["bash", str(SCENARIO)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
        env=os.environ.copy(),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    observed = {}
    module_path = None
    for line in result.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        observed[key] = value
        if key == "MODULE_PATH":
            module_path = Path(value)
        if key == "IMPORTED_MODULE":
            assert (
                Path(value)
                == (
                    REPO_ROOT
                    / "ros_packages"
                    / "display"
                    / "display"
                    / "display_web_request.py"
                ).resolve()
            )
    for key, value in REQUIRED.items():
        assert observed.get(key) == value, observed
    assert (
        module_path
        == (
            REPO_ROOT
            / "ros_packages"
            / "display"
            / "display"
            / "display_web_request.py"
        ).resolve()
    )
    assert observed["WORKTREE_PWD"] == str(REPO_ROOT.resolve())
