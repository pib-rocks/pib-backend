from public_api_client.hermes_agent_client import session_name_for


def test_session_name_is_prefixed_and_sanitized():
    assert session_name_for("abc-123") == "pib_chat_abc-123"


def test_session_name_strips_unsafe_chars():
    assert session_name_for("a/b c!") == "pib_chat_ab_c"
