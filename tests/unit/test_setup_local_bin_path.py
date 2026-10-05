"""`which hermes` must answer in every shell of user pib (PR-1910, finding 2).

install_local_bin_path() from setup-pib.sh is run for real against scratch
directories; `sudo install` is executed by a stub that drops the ownership flags.
The resulting files are then sourced by real shells: /etc/profile.d by `sh`, the
~/.bashrc by a non-interactive bash, which is what sshd starts for
`ssh pib@robot which hermes`.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"

# The guard every Debian / Raspberry Pi OS ~/.bashrc starts with.
DEBIAN_BASHRC = """# ~/.bashrc: executed by bash(1) for non-login shells.

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

HISTCONTROL=ignoreboth
"""

# Records the call and performs `install` without the -o/-g flags (no root here).
SUDO_STUB = """#!/bin/bash
printf '%s\\n' "$*" >> "$STUB_LOG"
if [ "$1" = "install" ]; then
    shift
    args=()
    while [ $# -gt 0 ]; do
        case "$1" in
            -o|-g) shift 2 ;;
            *) args+=("$1"); shift ;;
        esac
    done
    exec install "${args[@]}"
fi
exit 0
"""

PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
function command_exists() { command -v "$@" >/dev/null 2>&1; }
"""


def _extract(name: str) -> str:
    text = SETUP_PIB.read_text(encoding="utf-8")
    match = re.search(
        rf"^(?:function )?{re.escape(name)}\(\) \{{\n.*?^\}}\n",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert match, f"function {name} not found in setup-pib.sh"
    return match.group(0)


def _marker() -> str:
    match = re.search(r'^PIB_LOCAL_BIN_MARKER="(.*)"$', SETUP_PIB.read_text(), re.M)
    assert match, "PIB_LOCAL_BIN_MARKER not found in setup-pib.sh"
    return f'PIB_LOCAL_BIN_MARKER="{match.group(1)}"\n'


def _run(tmp_path: Path, bashrc: str | None = DEBIAN_BASHRC):
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir(exist_ok=True)
    (stub_bin / "sudo").write_text(SUDO_STUB, encoding="utf-8")
    (stub_bin / "sudo").chmod(0o755)
    log = tmp_path / "sudo.log"
    log.touch()

    home = tmp_path / "home" / "pib"
    (home / ".local" / "bin").mkdir(parents=True, exist_ok=True)
    hermes = home / ".local" / "bin" / "hermes"
    hermes.write_text("#!/bin/sh\necho hermes-cli\n", encoding="utf-8")
    hermes.chmod(0o755)
    if bashrc is not None:
        (home / ".bashrc").write_text(bashrc, encoding="utf-8")
    profile_d = tmp_path / "profile.d"
    profile_d.mkdir(exist_ok=True)

    script = (
        PRELUDE
        + _marker()
        + _extract("require_nonempty")
        + _extract("local_bin_path_snippet")
        + _extract("install_local_bin_path")
        + "\ninstall_local_bin_path\necho rc=$?\n"
    )
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        PIB_PROFILE_D=str(profile_d),
        PIB_USER_HOME=str(home),
    )
    result = subprocess.run(
        ["bash", "-c", script], capture_output=True, check=False, env=env, text=True
    )
    return result, home, profile_d


def _clean_path() -> str:
    """A PATH without any ~/.local/bin, as sshd or a fresh login hands out."""
    return "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"


def _which_hermes(shell: list[str], home: Path, source_file: Path) -> str:
    result = subprocess.run(
        shell + [f'. "{source_file}"; command -v hermes'],
        capture_output=True,
        check=False,
        text=True,
        env={"HOME": str(home), "PATH": _clean_path()},
    )
    return result.stdout.strip()


def test_login_shells_find_hermes_through_profile_d(tmp_path):
    result, home, profile_d = _run(tmp_path)

    drop_in = profile_d / "pib-local-bin.sh"
    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert drop_in.is_file()
    expected = str(home / ".local" / "bin" / "hermes")
    assert _which_hermes(["bash", "-c"], home, drop_in) == expected
    sh = shutil.which("dash") or "/bin/sh"
    assert _which_hermes([sh, "-c"], home, drop_in) == expected


def test_non_interactive_bash_finds_hermes_through_bashrc(tmp_path):
    """sshd runs `ssh host cmd` in a non-interactive bash; ~/.bashrc returns early
    for those unless the PATH block sits above the guard."""
    result, home, _ = _run(tmp_path)

    bashrc = home / ".bashrc"
    assert "rc=0" in result.stdout, result.stdout + result.stderr
    text = bashrc.read_text(encoding="utf-8")
    assert text.index(".local/bin") < text.index("*i*) ;;")
    assert text.endswith(DEBIAN_BASHRC), "the original ~/.bashrc must be kept intact"
    expected = str(home / ".local" / "bin" / "hermes")
    assert _which_hermes(["bash", "-c"], home, bashrc) == expected


def test_the_block_is_written_once_and_the_path_entry_is_not_duplicated(tmp_path):
    first, home, profile_d = _run(tmp_path)
    second, _, _ = _run(tmp_path, bashrc=None)

    assert "rc=0" in first.stdout and "rc=0" in second.stdout
    assert "already puts ~/.local/bin on PATH" in second.stdout
    text = (home / ".bashrc").read_text(encoding="utf-8")
    assert text.count("setup-pib.sh") == 1
    assert text.endswith(DEBIAN_BASHRC)

    result = subprocess.run(
        [
            "bash",
            "-c",
            f'. "{profile_d / "pib-local-bin.sh"}"; . "{home / ".bashrc"}"; echo "$PATH"',
        ],
        capture_output=True,
        check=False,
        text=True,
        env={"HOME": str(home), "PATH": _clean_path()},
    )
    assert result.stdout.count(str(home / ".local" / "bin")) == 1


def test_a_missing_bashrc_is_created_for_pib(tmp_path):
    result, home, _ = _run(tmp_path, bashrc=None)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert (home / ".bashrc").is_file()
    installs = [
        line
        for line in (tmp_path / "sudo.log").read_text().splitlines()
        if ".bashrc" in line
    ]
    assert installs and all("-o pib -g pib" in line for line in installs), installs


def test_setup_puts_local_bin_on_path_before_the_hermes_installer_runs():
    text = SETUP_PIB.read_text(encoding="utf-8")

    path_step = text.index('run_step "Put ~/.local/bin on PATH" install_local_bin_path')
    hermes_step = text.index('run_step "Install Hermes CLI" install_hermes_cli')
    assert path_step < hermes_step
    # The installer itself runs with ~/.local/bin on PATH, so it stops warning about it.
    assert (
        'export PATH="$HOME/.local/bin:$PATH"; curl -fsSL https://hermes-agent' in text
    )
