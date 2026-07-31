import os
import stat
from unittest.mock import patch

from public_api_client.hermes_agent_client import ensure_profile, profile_dir_for

BASE_ENV = "OPENROUTER_API_KEY=sk-base-key\n"
BASE_CONFIG = "model: anthropic/claude-opus-5\n"


def _absent_binary(tmp_path, monkeypatch):
    """Reproduce the flask container, where the hermes CLI is not mounted."""
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))


def _base_install_with_credentials(home):
    """A HERMES_HOME that has been through `hermes setup`."""
    (home / ".env").write_text(BASE_ENV, encoding="utf-8")
    (home / "config.yaml").write_text(BASE_CONFIG, encoding="utf-8")


def test_ensure_profile_creates_profile_when_missing(
    tmp_path, monkeypatch, installed_hermes_bin
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    args = run.call_args_list[0].args[0]
    assert "profile" in args and "create" in args and "pib_p-9" in args
    assert "--clone" in args and "--no-alias" in args

    assert pdir == os.path.join(str(tmp_path), "pib_p-9")
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == "Du bist pib."


def test_ensure_profile_is_idempotent_when_present(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    pdir = profile_dir_for("p-9")
    os.makedirs(pdir)

    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        ensure_profile("p-9", soul_text="Du bist pib.")

    run.assert_not_called()          # no re-create
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == "Du bist pib."


def test_ensure_profile_still_writes_the_soul_without_a_binary(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    _absent_binary(tmp_path, monkeypatch)

    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    run.assert_not_called()
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == "Du bist pib."


def test_ensure_profile_copies_base_credentials_without_the_binary(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """A profile with only a SOUL.md fails every turn with 'No LLM provider'."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == "Du bist pib."
    with open(os.path.join(pdir, ".env"), encoding="utf-8") as fh:
        assert fh.read() == BASE_ENV
    with open(os.path.join(pdir, "config.yaml"), encoding="utf-8") as fh:
        assert fh.read() == BASE_CONFIG


def test_credentials_are_copied_not_symlinked(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """`hermes profile delete` must not be able to gut the base install."""
    _base_install_with_credentials(sandboxed_hermes_home)
    _absent_binary(tmp_path, monkeypatch)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    assert not os.path.islink(os.path.join(pdir, ".env"))
    assert not os.path.islink(os.path.join(pdir, "config.yaml"))


def test_copied_env_keeps_owner_only_permissions(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    _base_install_with_credentials(sandboxed_hermes_home)
    (sandboxed_hermes_home / ".env").chmod(0o644)
    _absent_binary(tmp_path, monkeypatch)

    pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    mode = stat.S_IMODE(os.stat(os.path.join(pdir, ".env")).st_mode)
    assert mode == 0o600


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


def test_ensure_profile_warns_about_a_base_install_without_credentials(
    tmp_path, monkeypatch, sandboxed_hermes_home, caplog
):
    _absent_binary(tmp_path, monkeypatch)

    with caplog.at_level("WARNING"):
        ensure_profile("p-9", soul_text="Du bist pib.")

    warnings = "\n".join(
        record.getMessage() for record in caplog.records if record.levelname == "WARNING"
    )
    assert ".env" in warnings and "config.yaml" in warnings
    assert "fallback" in warnings


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
    spoofed = os.stat_result((
        real.st_mode, real.st_ino, real.st_dev, real.st_nlink, 4242, 4343,
        real.st_size, int(real.st_atime), int(real.st_mtime), int(real.st_ctime),
    ))
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
