"""The flask API and the voice assistant must resolve the identical SOUL.md.

They run in different containers. Before this was a shared setting, the API wrote
a SOUL.md the agent never read, and the API still reported success. These tests
fail if the two locations drift apart again.
"""

import pib_hermes_config
from public_api_client import hermes_agent_client
from service import soul_service


def test_api_and_agent_agree_on_the_soul_path(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    api_path = soul_service.soul_path_for("p-9")
    agent_dir = hermes_agent_client.profile_dir_for("p-9")

    assert api_path == f"{agent_dir}/SOUL.md"
    assert api_path.startswith(str(tmp_path))


def test_agent_reads_the_soul_the_api_wrote(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))

    soul_service.write_soul("p-9", "Du bist pib.")

    written = hermes_agent_client.soul_path_for("p-9")
    with open(written, encoding="utf-8") as fh:
        assert fh.read() == "Du bist pib."


def test_default_profiles_dir_matches_the_documented_bind_mount(monkeypatch):
    monkeypatch.delenv("PIB_HERMES_PROFILES_DIR", raising=False)

    assert pib_hermes_config.profiles_dir() == "/home/pib/.hermes/profiles"
    assert soul_service.profiles_dir() == hermes_agent_client.profiles_dir()


def test_profile_name_sanitization_is_shared():
    assert soul_service.profile_name_for("a/b c!") == "pib_ab_c"
    assert hermes_agent_client.profile_name_for("a/b c!") == "pib_ab_c"
