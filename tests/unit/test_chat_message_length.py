"""POST chat message content length: 413 when over the column limit (PR-2021)."""

from app.app import db
from model.chat_message_model import ChatMessage
from model.chat_model import Chat
from model.personality_model import Personality

META = {
    "source": "pib-cloud",
    "model": "pib-cloud",
    "stagesMs": {"end_of_speech": 0.0, "request_sent": 1.0, "first_token": 2.0},
    "streamEnd": "done",
}

CONTENT_LIMIT = ChatMessage.__table__.columns["content"].type.length


def _chat_id(app) -> str:
    with app.app_context():
        personality = Personality.query.first()
        chat = Chat(topic="length-test", personality_id=personality.personality_id)
        db.session.add(chat)
        db.session.commit()
        return chat.chat_id


def _post_message(client, chat_id: str, content: str, **extra):
    body = {"isUser": True, "content": content, **extra}
    return client.post(f"/voice-assistant/chat/{chat_id}/messages", json=body)


def test_content_at_the_column_limit_is_stored(app):
    chat_id = _chat_id(app)
    content = "x" * CONTENT_LIMIT

    with app.test_client() as client:
        response = _post_message(client, chat_id, content)

    assert response.status_code == 201
    assert response.get_json()["content"] == content
    with app.app_context():
        stored = ChatMessage.query.filter_by(chat_id=chat_id).one()
        assert len(stored.content) == CONTENT_LIMIT


def test_content_one_character_over_the_limit_returns_413(app):
    chat_id = _chat_id(app)
    content = "x" * (CONTENT_LIMIT + 1)

    with app.test_client() as client:
        response = _post_message(client, chat_id, content)

    assert response.status_code == 413
    body = response.get_json()
    assert body["field"] == "content"
    assert body["limit"] == CONTENT_LIMIT
    assert body["receivedLength"] == CONTENT_LIMIT + 1
    assert "error" in body
    assert str(CONTENT_LIMIT) in body["error"]
    assert "Traceback" not in response.get_data(as_text=True)


def test_a_very_large_payload_matches_received_length(app):
    """Reproduces the measured URDF-sized body (~313k characters)."""
    chat_id = _chat_id(app)
    received = 313029
    content = "x" * received

    with app.test_client() as client:
        response = _post_message(client, chat_id, content)

    assert response.status_code == 413
    body = response.get_json()
    assert body["field"] == "content"
    assert body["limit"] == CONTENT_LIMIT
    assert body["receivedLength"] == received


def test_malformed_json_stays_400_and_differs_from_413(app):
    chat_id = _chat_id(app)

    with app.test_client() as client:
        bad_json = client.post(
            f"/voice-assistant/chat/{chat_id}/messages",
            data="{not json",
            content_type="application/json",
        )
        over_limit = _post_message(client, chat_id, "x" * (CONTENT_LIMIT + 1))

    assert bad_json.status_code == 400
    assert over_limit.status_code == 413
    assert bad_json.get_json() != over_limit.get_json()


def test_valid_content_with_meta_still_succeeds(app):
    chat_id = _chat_id(app)

    with app.test_client() as client:
        response = _post_message(client, chat_id, "Kurz.", isUser=False, meta=META)

    assert response.status_code == 201
    assert response.get_json()["meta"] == META
