"""GET /voice-assistant/chat/{id}/messages: ETag, 304, since, limit (PR-2095)."""

from app.app import db
from model.chat_model import Chat
from model.personality_model import Personality


def _chat_id(app) -> str:
    with app.app_context():
        personality = Personality.query.first()
        chat = Chat(topic="etag-test", personality_id=personality.personality_id)
        db.session.add(chat)
        db.session.commit()
        return chat.chat_id


def _post(client, chat_id: str, content: str, is_user: bool = True):
    return client.post(
        f"/voice-assistant/chat/{chat_id}/messages",
        json={"isUser": is_user, "content": content},
    )


def test_get_messages_returns_200_with_etag(app):
    chat_id = _chat_id(app)
    with app.test_client() as client:
        _post(client, chat_id, "one")
        response = client.get(f"/voice-assistant/chat/{chat_id}/messages")

    assert response.status_code == 200
    assert response.headers.get("ETag")


def test_if_none_match_matching_etag_returns_304_with_empty_body(app):
    chat_id = _chat_id(app)
    with app.test_client() as client:
        _post(client, chat_id, "one")
        first = client.get(f"/voice-assistant/chat/{chat_id}/messages")
        etag = first.headers.get("ETag")
        second = client.get(
            f"/voice-assistant/chat/{chat_id}/messages",
            headers={"If-None-Match": etag},
        )

    assert second.status_code == 304
    assert second.get_data() == b""


def test_new_message_changes_etag_and_conditional_get_returns_200(app):
    chat_id = _chat_id(app)
    with app.test_client() as client:
        _post(client, chat_id, "one")
        first = client.get(f"/voice-assistant/chat/{chat_id}/messages")
        etag = first.headers["ETag"]
        _post(client, chat_id, "two")
        second = client.get(
            f"/voice-assistant/chat/{chat_id}/messages",
            headers={"If-None-Match": etag},
        )

    assert second.status_code == 200
    assert second.headers["ETag"] != etag
    messages = second.get_json()["messages"]
    assert len(messages) == 2
    assert messages[-1]["content"] == "two"


def test_limit_returns_at_most_n_messages_in_envelope(app):
    chat_id = _chat_id(app)
    with app.test_client() as client:
        for i in range(5):
            _post(client, chat_id, f"msg-{i}")
        response = client.get(f"/voice-assistant/chat/{chat_id}/messages?limit=2")

    assert response.status_code == 200
    payload = response.get_json()
    assert set(payload.keys()) == {"messages"}
    assert len(payload["messages"]) == 2
    assert payload["messages"][0]["content"] == "msg-3"
    assert payload["messages"][1]["content"] == "msg-4"


def test_since_message_id_returns_only_newer_messages_in_envelope(app):
    chat_id = _chat_id(app)
    with app.test_client() as client:
        _post(client, chat_id, "first")
        mid = _post(client, chat_id, "anchor").get_json()["messageId"]
        _post(client, chat_id, "after")

        response = client.get(f"/voice-assistant/chat/{chat_id}/messages?since={mid}")

    assert response.status_code == 200
    payload = response.get_json()
    assert set(payload.keys()) == {"messages"}
    assert len(payload["messages"]) == 1
    assert payload["messages"][0]["content"] == "after"


def test_default_get_keeps_today_response_shape(app):
    chat_id = _chat_id(app)
    with app.test_client() as client:
        _post(client, chat_id, "hello")
        response = client.get(f"/voice-assistant/chat/{chat_id}/messages")

    payload = response.get_json()
    assert set(payload.keys()) == {"messages"}
    assert len(payload["messages"]) == 1
    msg = payload["messages"][0]
    assert {"messageId", "content", "isUser", "timestamp"}.issubset(msg.keys())
