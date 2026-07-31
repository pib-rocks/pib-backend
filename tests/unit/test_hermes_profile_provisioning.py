from unittest.mock import patch
from public_api_client.hermes_agent_client import ensure_profile


def test_ensure_profile_creates_profile_when_missing(tmp_path):
    with patch("os.path.isdir", return_value=False), \
         patch("subprocess.run") as run:
        ensure_profile("p-9", soul_text="Du bist pib.")
        args = run.call_args_list[0].args[0]
        assert "profile" in args and "create" in args and "pib_p-9" in args


def test_ensure_profile_is_idempotent_when_present(tmp_path):
    with patch("os.path.isdir", return_value=True), \
         patch("subprocess.run") as run, \
         patch("builtins.open"):
        ensure_profile("p-9", soul_text="Du bist pib.")
        run.assert_not_called()          # no re-create
