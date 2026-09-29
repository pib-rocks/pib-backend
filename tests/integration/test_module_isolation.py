"""Guards against test modules that leak stand-ins into ``sys.modules``.

A module that installs a stand-in for a client library has to remove it again
when its tests are done. Otherwise the next module that probes the library with
a plain import believes it is present, skips building its own stand-in, and then
misses the parts it needs. That ran as seven red tests in
``tests/unit/test_stereo_camera_optimization.py`` whenever the full suite ran,
while the file stayed green on its own.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
# The order matters: the first module installs the stand-ins that used to leak
# into the second one.
ORDERED_MODULES = (
    "tests/integration/test_motor_current.py",
    "tests/unit/test_stereo_camera_optimization.py",
)


def test_stand_in_modules_do_not_leak_into_a_later_test_module():
    """Both modules in one process; the later one has to stay green."""
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            *ORDERED_MODULES,
            "-q",
            "--no-header",
            "-p",
            "no:cacheprovider",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, (
        "the second module only fails after the first one ran, so a stand-in "
        "module leaked into sys.modules:\n" + result.stdout[-4000:]
    )
