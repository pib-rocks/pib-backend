"""The install log is navigable: a header and result per step, a summary at the end,
image builds in their own file, and the locale generated first (PR-1910, findings 3, 8).

The shell functions are cut out of setup-pib.sh / docker_install.sh and run in a
bash subprocess with stubs on PATH; nothing outside tmp_path is touched.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"
DOCKER_INSTALL = REPO_ROOT / "setup" / "installation_scripts" / "docker_install.sh"

PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
function command_exists() { command -v "$@" >/dev/null 2>&1; }
"""


def _extract(script: Path, name: str) -> str:
    text = script.read_text(encoding="utf-8")
    match = re.search(
        rf"^(?:function )?{re.escape(name)}\(\) \{{\n.*?^\}}\n",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert match, f"function {name} not found in {script}"
    return match.group(0)


def _step_runner() -> str:
    text = SETUP_PIB.read_text(encoding="utf-8")
    arrays = re.search(
        r"^STEP_NAMES=\(\)\nSTEP_RESULTS=\(\)\nSTEP_DURATIONS=\(\)\nSTEP_FAILURES=0\n",
        text,
        re.MULTILINE,
    )
    assert arrays, "step runner state not found in setup-pib.sh"
    return (
        arrays.group(0)
        + _extract(SETUP_PIB, "run_step")
        + _extract(SETUP_PIB, "print_step_summary")
        + _extract(SETUP_PIB, "abort_setup")
    )


def _bash(script: str, env_overrides: dict | None = None, cwd: Path | None = None):
    env = dict(os.environ)
    env.update(env_overrides or {})
    return subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
        cwd=str(cwd) if cwd else None,
    )


# ---- run_step / summary ------------------------------------------------------------------


def test_each_step_gets_a_header_a_result_line_and_a_summary_row(tmp_path):
    step_log = tmp_path / "steps.tsv"
    script = PRELUDE + _step_runner() + """
good() { echo "doing the good thing"; }
bad() { echo "about to fail"; return 3; }
run_step "First thing" good; echo "status=$?"
run_step "Second thing" bad; echo "status=$?"
run_step "Third thing" good; echo "status=$?"
print_step_summary
echo "failures=$STEP_FAILURES"
"""
    result = _bash(script, {"PIB_SETUP_STEP_LOG": str(step_log)})
    out = result.stdout

    assert "==== step 1: First thing (started" in out
    assert "==== step 1: First thing: ok (" in out
    assert "==== step 2: Second thing: FAILED rc=3 (" in out
    assert out.index("about to fail") < out.index("Second thing: FAILED")
    assert ["status=0", "status=3", "status=0"] == re.findall(r"status=\d", out)
    assert "Setup summary: 3 steps, 1 failed" in out
    summary = out.split("Setup summary")[1]
    assert re.search(r"^\s+1\.\s+First thing\s+ok\s+\d+s$", summary, re.M)
    assert re.search(r"^\s+2\.\s+Second thing\s+FAILED rc=3\s+\d+s$", summary, re.M)
    assert re.search(r"^\s+3\.\s+Third thing\s+ok\s+\d+s$", summary, re.M)
    assert "failures=1" in out
    rows = [line.split("\t") for line in step_log.read_text().splitlines()]
    assert [row[:2] for row in rows] == [
        ["First thing", "ok"],
        ["Second thing", "FAILED rc=3"],
        ["Third thing", "ok"],
    ]


def test_run_step_can_wrap_a_sourced_installer_script(tmp_path):
    sourced = tmp_path / "part.sh"
    sourced.write_text("echo from-part\nreturn 2\n", encoding="utf-8")
    script = (
        PRELUDE
        + _step_runner()
        + f'run_step "Sourced part" source "{sourced}"; echo "status=$?"\n'
    )
    result = _bash(script)

    assert "from-part" in result.stdout
    assert "Sourced part: FAILED rc=2" in result.stdout
    assert "status=2" in result.stdout


def test_abort_setup_prints_the_summary_so_far_and_exits_one():
    script = PRELUDE + _step_runner() + """
run_step "Done before" true
abort_setup "cannot go on"
echo "not reached"
"""
    result = _bash(script)

    assert result.returncode == 1
    assert "cannot go on" in result.stdout
    assert "Setup summary: 1 steps, 0 failed" in result.stdout
    assert "Done before" in result.stdout.split("Setup summary")[1]
    assert "not reached" not in result.stdout


def test_the_installer_wraps_every_step_and_ends_with_the_summary():
    text = SETUP_PIB.read_text(encoding="utf-8")
    main = text.split("# ---------- SETUP STARTS FROM HERE -----------", 1)[1]

    for label, command in (
        ("Generate locale en_US.UTF-8", "install_locale"),
        ("Install system packages", "install_system_packages"),
        ("Clone repositories", "clone_repositories"),
        ("Provision curated OAK models", "provision_curated_models provision"),
        ("Provision whisper model", "provision_whisper_model"),
        ("Install pib Python packages", "install_pib_python_packages"),
        ("Put ~/.local/bin on PATH", "install_local_bin_path"),
        ("Install Hermes CLI", "install_hermes_cli"),
        ("Install setup files", "move_setup_files"),
        ("Set up pib Marimo service", "setup_pib_marimo_service"),
        ("Install DB browser", "install_DBbrowser"),
        ("Install Tinkerforge", "install_tinkerforge"),
        ("Set up IP dispatcher", "setup_ip_dispatcher"),
        ("Install wireplumber volume drop-in", "install_wireplumber_volume_defaults"),
        ("Set default output volume", "set_default_output_volume"),
        ("Clean up", "cleanup"),
    ):
        assert f'run_step "{label}" {command}' in main, label
    assert 'run_step "Install ROS 2 Jazzy" source' in main
    assert 'run_step "Adjust system settings" source' in main
    # No step runs bare at the top level any more.
    for bare in (
        "\ninstall_system_packages ||",
        "\nclone_repositories ||",
        "\ncleanup\n",
    ):
        assert bare not in main, bare
    # `return 1` at the top level of the script is not an exit; fatal steps abort.
    assert "return 1; }" not in main
    assert main.count("abort_setup ") >= 4
    assert main.index("print_step_summary") < main.index('"Installation completed"')
    assert "Installation completed with ${STEP_FAILURES} failed step(s)" in main


def test_docker_install_reports_each_service_as_its_own_step():
    text = DOCKER_INSTALL.read_text(encoding="utf-8")

    for label, command in (
        ("Install Docker Engine", "install_docker_engine"),
        ("Add user pib to the docker group", "add_pib_to_docker_group"),
        ("Build and start containers", "start_container"),
        ("Set up Docker cleaner service", "setup_docker_cleaner_service"),
        ("Set up host-side update service", "setup_update_service"),
        ("Set up host-side display web service", "setup_display_web_service"),
        ("Open database permissions", "open_database_permissions"),
    ):
        assert f'run_step "{label}" {command}' in text, label


# ---- docker build output -----------------------------------------------------------------

# `sudo docker compose ...` is replaced by a stub that prints build noise and exits as told.
SUDO_STUB = """#!/bin/bash
echo "#1 [internal] load build definition"
echo "#2 pip install noise"
echo "warning: build noise on stderr" >&2
exit "${STUB_COMPOSE_STATUS:-0}"
"""


def _run_start_container(tmp_path: Path, status: int):
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    (stub_bin / "sudo").write_text(SUDO_STUB, encoding="utf-8")
    (stub_bin / "sudo").chmod(0o755)
    backend = tmp_path / "backend"
    backend.mkdir()
    build_log = tmp_path / "docker-build.log"
    script = (
        PRELUDE
        + 'DOCKER_BUILD_LOG="${PIB_DOCKER_BUILD_LOG:-$HOME/setup-pib-docker-build.log}"\n'
        + _extract(DOCKER_INSTALL, "run_logged_compose")
        + _extract(DOCKER_INSTALL, "start_container")
        + "\nstart_container\necho rc=$?\n"
    )
    result = _bash(
        script,
        {
            "PATH": f"{stub_bin}{os.pathsep}{os.environ['PATH']}",
            "STUB_COMPOSE_STATUS": str(status),
            "BACKEND_DIR": str(backend),
            "FRONTEND_DIR": str(tmp_path / "cerebra"),
            "PIB_DOCKER_BUILD_LOG": str(build_log),
            "PIB_HARDWARE_VARIANT": "pib5edu",
        },
    )
    return result, build_log


def test_build_output_goes_to_its_own_file_and_the_main_log_points_there(tmp_path):
    result, build_log = _run_start_container(tmp_path, status=0)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert f"output goes to {build_log}" in result.stdout
    assert "pip install noise" not in result.stdout
    assert "build noise on stderr" not in result.stderr
    text = build_log.read_text(encoding="utf-8")
    assert "pip install noise" in text
    assert "build noise on stderr" in text
    assert text.count("#1 [internal] load build definition") == 2  # backend + cerebra


def test_a_failed_build_shows_its_last_lines_in_the_main_log(tmp_path):
    result, build_log = _run_start_container(tmp_path, status=17)

    assert "rc=1" in result.stdout
    assert "pib-backend image build failed (rc=17)" in result.stdout
    assert "pip install noise" in result.stdout  # the tail, after the failure
    assert "Started pib-backend container" not in result.stdout
    assert build_log.read_text(encoding="utf-8").count("#1 ") == 1


def test_the_default_build_log_lives_next_to_the_setup_log():
    text = DOCKER_INSTALL.read_text(encoding="utf-8")
    assert (
        'DOCKER_BUILD_LOG="${PIB_DOCKER_BUILD_LOG:-$HOME/setup-pib-docker-build.log}"'
        in text
    )
    assert "docker compose -f" in text
    assert "run_logged_compose" in _extract(DOCKER_INSTALL, "start_container")


# ---- locale --------------------------------------------------------------------------------

LOCALE_STUB = """#!/bin/bash
if [ "${1:-}" = "-a" ]; then
    if [ -f "$STUB_LOCALE_MARKER" ]; then
        echo "C"
        echo "C.utf8"
        echo "en_US.utf8"
    else
        echo "C"
        echo "C.utf8"
    fi
    exit 0
fi
exit 0
"""

# Records every sudo call; `locale-gen` creates the marker the `locale` stub looks for.
LOCALE_SUDO_STUB = """#!/bin/bash
printf '%s\\n' "$*" >> "$STUB_LOG"
case "$1" in
    locale-gen) touch "$STUB_LOCALE_MARKER" ;;
    tee) cat >/dev/null ;;
esac
exit 0
"""


def _run_install_locale(tmp_path: Path, generated: bool):
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    for name, body in (("locale", LOCALE_STUB), ("sudo", LOCALE_SUDO_STUB)):
        (stub_bin / name).write_text(body, encoding="utf-8")
        (stub_bin / name).chmod(0o755)
    (stub_bin / "locale-gen").write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
    (stub_bin / "locale-gen").chmod(0o755)
    marker = tmp_path / "generated"
    if generated:
        marker.touch()
    log = tmp_path / "sudo.log"
    log.touch()
    script = (
        PRELUDE
        + _extract(SETUP_PIB, "locale_is_generated")
        + _extract(SETUP_PIB, "install_locale")
        + '\ninstall_locale; echo "rc=$?"; echo "LC_ALL=$LC_ALL"\n'
    )
    result = _bash(
        script,
        {
            "PATH": f"{stub_bin}{os.pathsep}{os.environ['PATH']}",
            "STUB_LOG": str(log),
            "STUB_LOCALE_MARKER": str(marker),
            "LC_ALL": "en_US.UTF-8",
        },
    )
    return result, log.read_text(encoding="utf-8").splitlines()


def test_a_missing_locale_is_generated_and_exported(tmp_path):
    result, calls = _run_install_locale(tmp_path, generated=False)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert "locale-gen en_US.UTF-8" in calls
    assert "tee -a /etc/locale.gen" in calls
    assert "update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8" in calls
    assert "Generated locale en_US.UTF-8" in result.stdout
    assert "LC_ALL=en_US.UTF-8" in result.stdout
    assert not [call for call in calls if "apt-get" in call], calls


def test_an_existing_locale_is_left_alone(tmp_path):
    result, calls = _run_install_locale(tmp_path, generated=True)

    assert "rc=0" in result.stdout
    assert "already generated" in result.stdout
    assert not [call for call in calls if "locale-gen" in call or "sed" in call], calls


def test_the_locale_is_the_first_step_and_the_script_runs_under_c_utf8_until_then():
    text = SETUP_PIB.read_text(encoding="utf-8")
    main = text.split("# ---------- SETUP STARTS FROM HERE -----------", 1)[1]

    first_step = main.index("run_step ")
    assert main[first_step:].startswith(
        'run_step "Generate locale en_US.UTF-8" install_locale'
    )
    assert first_step < main.index("/etc/pib_hardware_variant")
    assert first_step < main.index("check_distribution")
    fallback = main.index("export LANG=C.UTF-8 LC_ALL=C.UTF-8")
    assert main.index('exec > >(tee -a "$LOG_FILE")') < fallback < first_step
