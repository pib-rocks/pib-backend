import os
import stat
import sys
import types
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

from public_api_client.hermes_agent_client import (
    DEFAULT_HERMES_MODEL,
    DEFAULT_HERMES_PROVIDER,
    PIB_MCP_SERVER,
    build_default_soul_text,
    ensure_profile,
    profile_dir_for,
)

BASE_ENV = "OPENROUTER_API_KEY=sk-base-key\n"
BASE_CONFIG = "model: anthropic/claude-opus-5\n"


@pytest.fixture(autouse=True)
def canonical_profile_factory(monkeypatch):
    profiles_module = types.ModuleType("hermes_cli.profiles")
    calls = []

    def create_profile(**kwargs):
        calls.append(kwargs)
        pdir = Path(os.environ["PIB_HERMES_PROFILES_DIR"]) / kwargs["name"]
        pdir.mkdir(parents=True)
        for dirname in (
            "memories",
            "sessions",
            "skills",
            "skins",
            "logs",
            "plans",
            "workspace",
            "cron",
            "home",
        ):
            (pdir / dirname).mkdir()
        home = Path(os.environ["HERMES_HOME"])
        for relative in ("config.yaml", ".env", "SOUL.md"):
            source = home / relative
            if source.is_file():
                (pdir / relative).write_bytes(source.read_bytes())
        for relative in ("memories/MEMORY.md", "memories/USER.md"):
            source = home / relative
            if source.is_file():
                (pdir / relative).write_bytes(source.read_bytes())
        if not (pdir / "config.yaml").exists():
            (pdir / "config.yaml").write_text("{}\n", encoding="utf-8")
        if not (pdir / ".env").exists():
            (pdir / ".env").write_text(
                "# Per-profile secrets for this Hermes profile.\n", encoding="utf-8"
            )
        return pdir

    profiles_module.create_profile = create_profile
    profiles_module.profile_exists = lambda _name: False
    package = types.ModuleType("hermes_cli")
    package.__path__ = []
    monkeypatch.setitem(sys.modules, "hermes_cli", package)
    monkeypatch.setitem(sys.modules, "hermes_cli.profiles", profiles_module)
    return calls


def _absent_binary(tmp_path, monkeypatch):
    """Reproduce the flask container, where the hermes CLI is not mounted."""
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))


def _base_install_with_credentials(home):
    """A HERMES_HOME that has been through `hermes setup`."""
    (home / ".env").write_text(BASE_ENV, encoding="utf-8")
    (home / "config.yaml").write_text(BASE_CONFIG, encoding="utf-8")


def _load_profile_config(pdir):
    with open(os.path.join(pdir, "config.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _assert_mcp_servers_pib(cfg):
    assert cfg["mcp_servers"]["pib"] == PIB_MCP_SERVER


def _assert_pinned_gemini_model(cfg):
    assert cfg["model"] == DEFAULT_HERMES_MODEL
    assert cfg["provider"] == DEFAULT_HERMES_PROVIDER


def test_ensure_profile_creates_profile_with_canonical_factory(
    tmp_path, monkeypatch, canonical_profile_factory
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    assert pdir == os.path.join(str(tmp_path), "pib_p-9")
    assert canonical_profile_factory == [
        {
            "name": "pib_p-9",
            "clone_from": None,
            "clone_all": False,
            "clone_config": True,
            "no_alias": True,
            "no_skills": False,
            "description": "pib personality p-9",
            "clone_channels": False,
        }
    ]
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == build_default_soul_text("pib", "Du bist pib.")


def test_ensure_profile_survives_a_factory_without_clone_channels(
    tmp_path, monkeypatch
):
    """The container's hermes_cli.create_profile has no `clone_channels` argument.

    Live, passing it failed every provisioning attempt with "create_profile() got
    an unexpected keyword argument 'clone_channels'", so the daemon must pass only
    what the INSTALLED signature accepts.
    """
    profiles_module = types.ModuleType("hermes_cli.profiles")
    calls = []

    def create_profile(
        name,
        clone_from=None,
        clone_all=False,
        clone_config=False,
        no_alias=False,
        no_skills=False,
        description=None,
    ):
        calls.append(
            {"name": name, "clone_config": clone_config, "description": description}
        )
        pdir = Path(os.environ["PIB_HERMES_PROFILES_DIR"]) / name
        pdir.mkdir(parents=True)
        for dirname in (
            "memories",
            "sessions",
            "skills",
            "skins",
            "logs",
            "plans",
            "workspace",
            "cron",
            "home",
        ):
            (pdir / dirname).mkdir()
        (pdir / "config.yaml").write_text("{}\n", encoding="utf-8")
        return pdir

    profiles_module.create_profile = create_profile
    profiles_module.profile_exists = lambda _name: False
    package = types.ModuleType("hermes_cli")
    package.__path__ = []
    monkeypatch.setitem(sys.modules, "hermes_cli", package)
    monkeypatch.setitem(sys.modules, "hermes_cli.profiles", profiles_module)
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    pdir = ensure_profile("p-container")

    assert calls == [
        {
            "name": "pib_p-container",
            "clone_config": True,
            "description": "pib personality p-container",
        }
    ]
    assert os.path.isdir(os.path.join(pdir, "memories"))


def test_ensure_profile_is_idempotent_when_present(
    tmp_path, monkeypatch, canonical_profile_factory
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")
    ensure_profile("p-9", soul_text="Du bist pib.")

    assert len(canonical_profile_factory) == 1
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == build_default_soul_text("pib", "Du bist pib.")


def test_ensure_profile_surfaces_unavailable_factory_without_writing_profile(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setitem(sys.modules, "hermes_cli.profiles", None)

    with pytest.raises(RuntimeError, match="factory is unavailable"):
        ensure_profile("p-9", soul_text="Du bist pib.")

    assert not os.path.exists(profile_dir_for("p-9"))


def test_ensure_profile_copies_base_credentials_with_factory(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    _base_install_with_credentials(sandboxed_hermes_home)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == build_default_soul_text("pib", "Du bist pib.")
    with open(os.path.join(pdir, ".env"), encoding="utf-8") as fh:
        assert fh.read() == BASE_ENV
    cfg = _load_profile_config(pdir)
    _assert_pinned_gemini_model(cfg)
    _assert_mcp_servers_pib(cfg)


def test_credentials_are_copied_not_symlinked(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """`hermes profile delete` must not be able to gut the base install."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    assert not os.path.islink(os.path.join(pdir, ".env"))
    assert not os.path.islink(os.path.join(pdir, "config.yaml"))


def test_profile_and_env_permissions_are_private(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    _base_install_with_credentials(sandboxed_hermes_home)
    (sandboxed_hermes_home / ".env").chmod(0o644)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    assert stat.S_IMODE(os.stat(os.path.join(pdir, ".env")).st_mode) == 0o600
    assert stat.S_IMODE(os.stat(pdir).st_mode) == 0o700
    assert stat.S_IMODE(os.stat(os.path.join(pdir, "SOUL.md")).st_mode) == 0o644
    assert all(
        stat.S_IMODE(os.stat(os.path.join(pdir, dirname)).st_mode) == 0o700
        for dirname in (
            "memories",
            "sessions",
            "skills",
            "skins",
            "logs",
            "plans",
            "workspace",
            "cron",
            "home",
        )
    )


def test_ensure_profile_does_not_overwrite_an_existing_profile_env(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)
    pdir = profile_dir_for("p-9")
    os.makedirs(pdir)
    with open(os.path.join(pdir, ".env"), "w", encoding="utf-8") as fh:
        fh.write("OPENROUTER_API_KEY=sk-customized-by-the-operator\n")

    ensure_profile("p-9", soul_text="Du bist pib.")

    with open(os.path.join(pdir, ".env"), encoding="utf-8") as fh:
        assert fh.read() == "OPENROUTER_API_KEY=sk-customized-by-the-operator\n"


def test_ensure_profile_factory_seeds_defaults_without_base_credentials(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    assert stat.S_IMODE(os.stat(os.path.join(pdir, ".env")).st_mode) == 0o600
    cfg = _load_profile_config(pdir)
    _assert_pinned_gemini_model(cfg)
    _assert_mcp_servers_pib(cfg)


def test_ensure_profile_takes_its_owner_from_the_profiles_directory(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """The pib user owns the profiles dir; uid 1000 is never assumed."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)
    profiles_root = str(tmp_path / "profiles-root")
    os.makedirs(profiles_root)
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", profiles_root)

    real = os.stat(profiles_root)
    spoofed = os.stat_result(
        (
            real.st_mode,
            real.st_ino,
            real.st_dev,
            real.st_nlink,
            4242,
            4343,
            real.st_size,
            int(real.st_atime),
            int(real.st_mtime),
            int(real.st_ctime),
        )
    )
    real_stat = os.stat

    def stat_with_spoofed_owner(path, *args, **kwargs):
        if str(path) == profiles_root:
            return spoofed
        return real_stat(path, *args, **kwargs)

    chowned = []
    monkeypatch.setattr(os, "stat", stat_with_spoofed_owner)
    monkeypatch.setattr(
        os, "chown", lambda path, uid, gid: chowned.append((str(path), uid, gid))
    )

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    assert (pdir, 4242, 4343) in chowned
    assert (os.path.join(pdir, "SOUL.md"), 4242, 4343) in chowned
    assert (os.path.join(pdir, ".env"), 4242, 4343) in chowned


def test_ensure_profile_survives_a_refused_chown(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Running as a non-root user must not turn into a failed chat turn."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)

    def refuse(*_args, **_kwargs):
        raise PermissionError("chown: Operation not permitted")

    monkeypatch.setattr(os, "chown", refuse)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    assert os.path.isfile(os.path.join(pdir, "SOUL.md"))
    assert os.path.isfile(os.path.join(pdir, ".env"))


def test_profile_directory_stays_private_to_its_owner(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    assert stat.S_IMODE(os.stat(pdir).st_mode) == 0o700


def test_ensure_profile_seeds_mcp_servers_pib_on_fresh_profile(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Fresh profiles get mcp_servers.pib even when the base config lacks it."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    cfg = _load_profile_config(pdir)
    _assert_mcp_servers_pib(cfg)
    _assert_pinned_gemini_model(cfg)


def test_ensure_profile_seeds_mcp_servers_pib_into_existing_config(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """An existing profile config.yaml still receives mcp_servers.pib and the pinned model."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)
    pdir = profile_dir_for("p-9")
    os.makedirs(pdir)
    with open(os.path.join(pdir, "config.yaml"), "w", encoding="utf-8") as fh:
        fh.write("model: custom/operator-model\n")

    ensure_profile("p-9", soul_text="Du bist pib.")

    cfg = _load_profile_config(pdir)
    _assert_pinned_gemini_model(cfg)
    _assert_mcp_servers_pib(cfg)


def _write_profile_config(pdir, cfg):
    os.makedirs(pdir, exist_ok=True)
    with open(os.path.join(pdir, "config.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(cfg, fh)


def test_ensure_profile_keeps_an_existing_mcp_servers_pib_entry(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """A customized entry keeps its command/args, but still gets the missing env.

    Hermes spawns the MCP server without this process' environment, so an entry
    without `env` sends every tool call to pib_mcp_server's own localhost:5000
    default. The repair therefore adds env and touches nothing else.
    """
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)
    pdir = profile_dir_for("p-9")
    custom = {"command": "python3", "args": ["-m", "custom_mcp"]}
    _write_profile_config(
        pdir, {"model": "custom/operator-model", "mcp_servers": {"pib": dict(custom)}}
    )

    ensure_profile("p-9", soul_text="Du bist pib.")

    entry = _load_profile_config(pdir)["mcp_servers"]["pib"]
    assert entry["command"] == custom["command"]
    assert entry["args"] == custom["args"]
    assert entry["env"] == PIB_MCP_SERVER["env"]
    _assert_pinned_gemini_model(_load_profile_config(pdir))


def test_ensure_profile_adds_the_missing_env_to_a_seeded_mcp_servers_pib_entry(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Migration for installs seeded before the entry carried its env block."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)
    pdir = profile_dir_for("p-9")
    _write_profile_config(
        pdir,
        {
            "mcp_servers": {
                "pib": {"command": "python3", "args": ["-m", "pib_mcp_server"]}
            }
        },
    )

    ensure_profile("p-9", soul_text="Du bist pib.")

    cfg = _load_profile_config(pdir)
    _assert_mcp_servers_pib(cfg)


def test_ensure_profile_never_overwrites_an_operator_set_mcp_env_value(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Only missing env keys are filled; the operator's own values survive."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)
    pdir = profile_dir_for("p-9")
    _write_profile_config(
        pdir,
        {
            "mcp_servers": {
                "pib": {
                    "command": "python3",
                    "args": ["-m", "pib_mcp_server"],
                    "env": {
                        "FLASK_API_BASE_URL": "http://operators-own-host:5000",
                        "PIB_MCP_EXTRA": "keep-me",
                    },
                }
            }
        },
    )

    ensure_profile("p-9", soul_text="Du bist pib.")

    env = _load_profile_config(pdir)["mcp_servers"]["pib"]["env"]
    defaults = PIB_MCP_SERVER["env"]
    assert env["FLASK_API_BASE_URL"] == "http://operators-own-host:5000"
    assert env["PIB_MCP_EXTRA"] == "keep-me"
    assert env["PIB_MCP_API_BASE_URL"] == defaults["PIB_MCP_API_BASE_URL"]
    assert env["PIB_MCP_ROSBRIDGE_URL"] == defaults["PIB_MCP_ROSBRIDGE_URL"]


def test_ensure_profile_pins_gemini_model_even_when_already_configured(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Model/provider are permanently pinned even if mcp_servers.pib already exists."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)
    pdir = profile_dir_for("p-9")
    os.makedirs(pdir)
    with open(os.path.join(pdir, "config.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "model": "anthropic/claude-opus-5",
                "provider": "openrouter",
                "mcp_servers": {"pib": dict(PIB_MCP_SERVER)},
            },
            fh,
        )

    ensure_profile("p-9", soul_text="Du bist pib.")

    cfg = _load_profile_config(pdir)
    _assert_pinned_gemini_model(cfg)
    _assert_mcp_servers_pib(cfg)
