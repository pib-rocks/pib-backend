"""Personality view: memory size, and character apart from experience."""

from pathlib import Path

import pytest

from pib_hermes_config.memory import (
    CHARACTER_LABEL,
    ENTRY_DELIMITER,
    EXPERIENCE_LABEL,
    STABLE_HEADING,
    consolidate_memory_file,
    memory_path_for,
    parse_entries,
)


@pytest.fixture()
def client(app):
    return app.test_client()


def test_personality_view_reports_memory_size_and_the_two_labels(
    client, make_personality
):
    personality = make_personality(description="Sei freundlich.")
    path = Path(memory_path_for(personality.personality_id))
    path.parent.mkdir(parents=True, exist_ok=True)
    experience = "The kettle is blue.\n"
    path.write_text(experience, encoding="utf-8")

    body = client.get(f"/voice-assistant/personality/{personality.personality_id}").json
    assert body["characterLabel"] == CHARACTER_LABEL
    assert body["experienceLabel"] == EXPERIENCE_LABEL
    assert body["description"] != body["memory"]
    assert body["memory"] == experience
    assert body["memorySize"] == len(experience)

    listed = client.get("/voice-assistant/personality").json
    shown = [
        item
        for item in listed["voiceAssistantPersonalities"]
        if item["personalityId"] == personality.personality_id
    ]
    assert shown[0]["memorySize"] == len(experience)
    assert shown[0]["characterLabel"] == "Character (SOUL)"
    assert shown[0]["experienceLabel"] == "Experience (MEMORY)"


def test_editing_experience_does_not_change_character(client, make_personality):
    personality = make_personality(description="Sei freundlich.")
    character = client.get(
        f"/voice-assistant/personality/{personality.personality_id}"
    ).json["description"]

    updated = client.put(
        f"/voice-assistant/personality/{personality.personality_id}",
        json={"memory": "The door code is 221."},
    )
    assert updated.status_code == 200
    body = updated.json
    assert body["description"] == character
    assert body["memory"] == "The door code is 221."
    assert body["memorySize"] == len("The door code is 221.")
    assert (
        Path(memory_path_for(personality.personality_id)).read_text(encoding="utf-8")
        == "The door code is 221."
    )

    again = client.put(
        f"/voice-assistant/personality/{personality.personality_id}",
        json={"description": character + "\nExtra."},
    )
    assert again.json["description"] == character + "\nExtra."
    assert again.json["memory"] == "The door code is 221."


def test_consolidated_memory_is_the_size_the_personality_view_shows(
    client, make_personality
):
    personality = make_personality()
    path = memory_path_for(personality.personality_id)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(
        ENTRY_DELIMITER.join(
            [
                "The garden is on the left of the house.",
                "The kettle on the hob is blue.",
                "keep me, exactly.",
            ]
        ),
        encoding="utf-8",
    )
    status = consolidate_memory_file(
        path,
        lambda _material: "Garden left, kettle blue.",
        now=50,
        interval=0,
        keep_recent=1,
    )
    assert status == "consolidated"

    body = client.get(f"/voice-assistant/personality/{personality.personality_id}").json
    stored = Path(path).read_text(encoding="utf-8")
    assert body["memorySize"] == len(stored)
    assert body["memory"] == stored
    entries = parse_entries(stored)
    assert entries[0].startswith(STABLE_HEADING + "\n")
    assert entries[-1] == "keep me, exactly."
    assert body["memorySize"] < len(
        ENTRY_DELIMITER.join(
            [
                "The garden is on the left of the house.",
                "The kettle on the hob is blue.",
                "keep me, exactly.",
            ]
        )
    )
