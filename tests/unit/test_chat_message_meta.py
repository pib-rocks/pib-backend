"""Chat message metadata (PC-82): stored per answer, absent for user lines."""

from app.app import db
from model.chat_message_model import ChatMessage
from model.chat_model import Chat
from model.personality_model import Personality

META = {
    "source": "pib-cloud",
    "model": "pib-cloud",
    "stagesMs": {"end_of_speech": 0.0, "request_sent": 84.5, "first_token": 332.1},
    "streamEnd": "done",
}


def _chat_id(app) -> str:
    """A chat of the seeded catalogue, so no fixture data has to be invented."""
    with app.app_context():
        personality = Personality.query.first()
        chat = Chat(topic="meta-test", personality_id=personality.personality_id)
        db.session.add(chat)
        db.session.commit()
        return chat.chat_id


def test_a_message_keeps_the_metadata_it_was_created_with(app):
    chat_id = _chat_id(app)

    with app.test_client() as client:
        response = client.post(
            f"/voice-assistant/chat/{chat_id}/messages",
            json={"isUser": False, "content": "Hallo.", "meta": META},
        )

    assert response.status_code == 201
    assert response.get_json()["meta"] == META
    with app.app_context():
        stored = ChatMessage.query.filter_by(chat_id=chat_id).one()
        assert stored.meta == META


def test_a_message_without_metadata_stays_valid(app):
    """Old clients keep working: the column is nullable."""
    chat_id = _chat_id(app)

    with app.test_client() as client:
        response = client.post(
            f"/voice-assistant/chat/{chat_id}/messages",
            json={"isUser": True, "content": "Nur Text."},
        )

    assert response.status_code == 201
    assert response.get_json()["meta"] is None
    with app.app_context():
        assert ChatMessage.query.filter_by(chat_id=chat_id).one().meta is None


def test_an_update_sets_the_metadata_and_a_later_update_without_it_keeps_it(app):
    """The turn writes its facts once the stream is finished; text updates follow."""
    chat_id = _chat_id(app)

    with app.test_client() as client:
        created = client.post(
            f"/voice-assistant/chat/{chat_id}/messages",
            json={"isUser": False, "content": "Teil."},
        ).get_json()
        message_id = created["messageId"]

        first = client.put(
            f"/voice-assistant/chat/{chat_id}/messages/{message_id}",
            json={"isUser": False, "content": "Teil eins.", "meta": META},
        )
        second = client.put(
            f"/voice-assistant/chat/{chat_id}/messages/{message_id}",
            json={"isUser": False, "content": "Teil eins, Teil zwei."},
        )

    assert first.status_code == 200
    assert first.get_json()["meta"] == META
    assert second.status_code == 200
    assert second.get_json()["meta"] == META
    assert second.get_json()["content"] == "Teil eins, Teil zwei."


def test_the_message_list_carries_the_metadata(app):
    chat_id = _chat_id(app)

    with app.test_client() as client:
        client.post(
            f"/voice-assistant/chat/{chat_id}/messages",
            json={"isUser": False, "content": "Hallo.", "meta": META},
        )
        listed = client.get(f"/voice-assistant/chat/{chat_id}/messages").get_json()

    messages = (
        listed["chatMessages"] if "chatMessages" in listed else listed["messages"]
    )
    assert messages[0]["meta"] == META
