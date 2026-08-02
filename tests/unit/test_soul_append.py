import pytest

from service import personality_service, soul_service


@pytest.fixture()
def client(app):
    return app.test_client()


def test_soul_append_is_bounded_and_appends(
    client, make_personality, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    personality = make_personality(description="Basis.")
    base_description = personality.description

    response = client.post(
        f"/voice-assistant/personality/{personality.personality_id}/soul/append",
        json={"lesson": "Der Nutzer heißt Jürgen."},
    )

    assert response.status_code == 200
    description = personality_service.get_personality(
        personality.personality_id
    ).description
    # Default SOUL already ends with "\n", so append_soul_lesson adds no extra separator.
    assert description == base_description + "Der Nutzer heißt Jürgen."
    assert soul_service.read_soul(personality.personality_id) == description

    empty_response = client.post(
        f"/voice-assistant/personality/{personality.personality_id}/soul/append",
        json={"lesson": "   "},
    )
    assert empty_response.status_code == 400
    assert (
        personality_service.get_personality(personality.personality_id).description
        == description
    )


def test_soul_append_rejects_oversized_lesson(
    client, make_personality, tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    personality = make_personality(description="Basis.")
    original = personality.description

    response = client.post(
        f"/voice-assistant/personality/{personality.personality_id}/soul/append",
        json={"lesson": "x" * 5000},
    )

    assert response.status_code == 400
    assert (
        personality_service.get_personality(personality.personality_id).description
        == original
    )
    assert soul_service.read_soul(personality.personality_id) == original
