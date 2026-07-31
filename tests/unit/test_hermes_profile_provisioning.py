import os
from unittest.mock import patch

from public_api_client.hermes_agent_client import ensure_profile, profile_dir_for


def test_ensure_profile_creates_profile_when_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(
        "public_api_client.hermes_agent_client.HERMES_HOME", str(tmp_path)
    )
    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        pdir = ensure_profile("p-9", soul_text="Du bist pib.")

    args = run.call_args_list[0].args[0]
    assert "profile" in args and "create" in args and "pib_p-9" in args
    assert "--clone" in args and "--no-alias" in args

    assert pdir == os.path.join(str(tmp_path), "profiles", "pib_p-9")
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == "Du bist pib."


def test_ensure_profile_is_idempotent_when_present(tmp_path, monkeypatch):
    monkeypatch.setattr(
        "public_api_client.hermes_agent_client.HERMES_HOME", str(tmp_path)
    )
    pdir = profile_dir_for("p-9")
    os.makedirs(pdir)

    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        ensure_profile("p-9", soul_text="Du bist pib.")

    run.assert_not_called()          # no re-create
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        assert fh.read() == "Du bist pib."
