"""Shell-level proof of the display web runner. Chromium is a stub on PATH."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO = REPO_ROOT / "tests" / "unit" / "display_web_runner_scenario.sh"

URL_A = "http://localhost"
URL_B = "http://localhost:8080/program"

REQUIRED = {
    # open A: one browser on A, URL stored
    "FIRST_OPEN_BROWSERS": "1",
    "FIRST_OPEN_STUB_URL": URL_A,
    "FIRST_OPEN_STORED_URL": URL_A,
    # open A again: nothing started, the first browser is still the only one
    "SECOND_OPEN_BROWSERS": "0",
    "SECOND_OPEN_LIVE_BROWSERS": "1",
    "SECOND_OPEN_STORED_URL": URL_A,
    # open B: the first browser is gone, exactly one browser shows B
    "THIRD_OPEN_NEW_BROWSERS": "1",
    "THIRD_OPEN_LIVE_BROWSERS": "1",
    "THIRD_OPEN_STUB_URL": URL_B,
    "THIRD_OPEN_STORED_URL": URL_B,
    "THIRD_OPEN_FIRST_BROWSER_ALIVE": "no",
    # hide: no browser, no pidfile, no stored URL
    "HIDE_TERMINATED": "yes",
    "HIDE_LIVE_BROWSERS": "0",
    "PIDFILE_AFTER_HIDE": "absent",
    "URLFILE_AFTER_HIDE": "absent",
    "FAILING_STUB_REQUEST": "absent",
    "FAILING_STUB_STATUS": "failed",
}


def test_stub_chromium_replaces_url_stays_idempotent_and_cleans_up():
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
