import os
from service.soul_service import soul_path_for, write_soul, read_soul


def test_soul_path_is_inside_the_personality_profile(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    p = soul_path_for("abc-123")
    assert p == os.path.join(str(tmp_path), "pib_abc-123", "SOUL.md")


def test_soul_path_defaults_to_the_shared_mount(monkeypatch):
    monkeypatch.delenv("PIB_HERMES_PROFILES_DIR", raising=False)
    assert soul_path_for("abc-123") == (
        "/home/pib/.hermes/profiles/pib_abc-123/SOUL.md"
    )


def test_write_then_read_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    write_soul("abc-123", "Du bist pib.")
    assert read_soul("abc-123") == "Du bist pib."


def test_read_missing_returns_empty(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    assert read_soul("nope") == ""
