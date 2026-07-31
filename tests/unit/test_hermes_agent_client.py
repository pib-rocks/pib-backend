from public_api_client.hermes_agent_client import (
    session_name_for,
    build_command,
    profile_name_for,
)


def test_session_name_is_prefixed_and_sanitized():
    assert session_name_for("abc-123") == "pib_chat_abc-123"


def test_session_name_strips_unsafe_chars():
    assert session_name_for("a/b c!") == "pib_chat_ab_c"


def test_profile_name_is_derived_from_personality():
    assert profile_name_for("abc-123") == "pib_abc-123"


def test_build_command_uses_oneshot_named_session_and_profile():
    cmd = build_command("hallo", "chat-1", personality_id="p-9")
    assert cmd[0].endswith("hermes")
    assert "-p" in cmd and "pib_p-9" in cmd          # profile carries the SOUL.md
    assert "-z" in cmd and "hallo" in cmd
    assert "-c" in cmd and "pib_chat_chat-1" in cmd  # durable per-chat session


def test_build_command_without_personality_omits_profile():
    cmd = build_command("hallo", "chat-1", personality_id=None)
    assert "-p" not in cmd
