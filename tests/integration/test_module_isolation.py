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
# Same polluter, different victim: its import-time tinkerforge stand-in used to
# make get_connected_bricklets() return an empty list (PR-1860).
BRICKLET_ORDERED_MODULES = (
    "tests/integration/test_motor_current.py",
    "tests/unit/test_bricklet_discovery_service.py",
)


def _run_pair(modules) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            *modules,
            "-q",
            "--no-header",
            "-rs",
            "-p",
            "no:cacheprovider",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )


def _assert_victim_stayed_green(result, victim: str, message: str) -> None:
    """A leak shows up as a failure in the victim module, not anywhere else.

    The polluter module is allowed to skip itself (it needs the colcon-built
    'datatypes' package); that must not silently turn this guard into a no-op.
    A victim module that cannot even be collected (missing third-party import in
    this environment) also fails here, because a guard whose subject never ran
    proves nothing (PR-2171).
    """
    output = result.stdout
    victim_name = Path(victim).name

    collect_errors = [
        line for line in output.splitlines() if "ERROR collecting" in line
    ]
    assert not collect_errors, (
        f"{message}: the victim or polluter module could not be collected in this "
        "environment, so this guard proves nothing - install the module's "
        "dependencies first:\n" + "\n".join(collect_errors) + "\n" + output[-2000:]
    )

    leaked = [
        line
        for line in output.splitlines()
        if line.startswith(("FAILED ", "ERROR ")) and victim_name in line
    ]
    assert not leaked, f"{message}\n" + "\n".join(leaked) + "\n" + output[-4000:]

    polluter = "tests/integration/test_motor_current.py"
    skipped_polluter = [
        line
        for line in output.splitlines()
        if line.startswith("SKIPPED") and Path(polluter).name in line
    ]
    assert not skipped_polluter, (
        f"{message}: the polluter module {polluter} did not run "
        f"({skipped_polluter[0]}), so this guard proves nothing\n" + output[-2000:]
    )
    assert " passed" in output, f"{message}: no test actually ran\n" + output[-2000:]


def test_stand_in_modules_do_not_leak_into_a_later_test_module():
    """Both modules in one process; the later one has to stay green."""
    result = _run_pair(ORDERED_MODULES)

    _assert_victim_stayed_green(
        result,
        "tests/unit/test_stereo_camera_optimization.py",
        "the second module only fails after the first one ran, so a stand-in "
        "module leaked into sys.modules",
    )


def test_tinkerforge_stand_in_does_not_empty_later_bricklet_enumeration():
    """Bricklet enumeration stays populated after motor_current's stand-in."""
    result = _run_pair(BRICKLET_ORDERED_MODULES)

    _assert_victim_stayed_green(
        result,
        "tests/unit/test_bricklet_discovery_service.py",
        "get_connected_bricklets() returns an empty list when this module runs "
        "after test_motor_current.py installed its tinkerforge stand-in",
    )
