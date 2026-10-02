"""Personality filler and per-chat first-token latency on the API."""

import pytest


@pytest.fixture()
def client(app):
    return app.test_client()


def test_pause_threshold_update_is_returned(client, make_personality):
    personality = make_personality(pause_threshold=0.8)
    response = client.put(
        f"/voice-assistant/personality/{personality.personality_id}",
        json={"pauseThreshold": 1.4},
    )
    assert response.status_code == 200
    assert response.json["pauseThreshold"] == 1.4

    again = client.get(f"/voice-assistant/personality/{personality.personality_id}")
    assert again.json["pauseThreshold"] == 1.4


def test_thinking_filler_is_authored_text_and_blank_clears_it(client, make_personality):
    personality = make_personality()
    created = client.get(f"/voice-assistant/personality/{personality.personality_id}")
    assert created.json["thinkingFiller"] is None

    stored = client.put(
        f"/voice-assistant/personality/{personality.personality_id}",
        json={"thinkingFiller": " Moment mal. "},
    )
    assert stored.status_code == 200
    assert stored.json["thinkingFiller"] == "Moment mal."

    cleared = client.put(
        f"/voice-assistant/personality/{personality.personality_id}",
        json={"thinkingFiller": "   "},
    )
    assert cleared.status_code == 200
    assert cleared.json["thinkingFiller"] is None


def test_pause_update_does_not_clear_an_authored_filler(client, make_personality):
    personality = make_personality()
    client.put(
        f"/voice-assistant/personality/{personality.personality_id}",
        json={"thinkingFiller": "Moment mal."},
    )
    updated = client.put(
        f"/voice-assistant/personality/{personality.personality_id}",
        json={"pauseThreshold": 1.2},
    )
    assert updated.json["pauseThreshold"] == 1.2
    assert updated.json["thinkingFiller"] == "Moment mal."


def test_first_token_latency_is_readable_on_the_chat(client, make_personality):
    personality = make_personality()
    created = client.post(
        "/voice-assistant/chat",
        json={"topic": "latency", "personalityId": personality.personality_id},
    )
    assert created.status_code == 201
    chat_id = created.json["chatId"]
    assert created.json["firstTokenLatencyMs"] is None
    assert created.json["firstTokenBudgetMs"] == 700

    recorded = client.put(
        f"/voice-assistant/chat/{chat_id}/first-token-latency",
        json={"latencyMs": 812.5},
    )
    assert recorded.status_code == 200
    assert recorded.json["firstTokenLatencyMs"] == 812.5
    assert recorded.json["firstTokenBudgetMs"] == 700

    fetched = client.get(f"/voice-assistant/chat/{chat_id}")
    assert fetched.json["firstTokenLatencyMs"] == 812.5


def test_first_token_latency_rejects_a_negative_measurement(client, make_personality):
    personality = make_personality()
    created = client.post(
        "/voice-assistant/chat",
        json={"topic": "latency", "personalityId": personality.personality_id},
    )
    chat_id = created.json["chatId"]
    rejected = client.put(
        f"/voice-assistant/chat/{chat_id}/first-token-latency",
        json={"latencyMs": -1},
    )
    assert rejected.status_code == 400
