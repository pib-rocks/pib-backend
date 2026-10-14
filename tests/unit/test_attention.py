"""Section 6.4: look at the speaker, name them, and do not greet twice."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from pib_hermes_config.attention import (  # noqa: E402
    HEAD_MOTOR,
    HEAD_POSITION_MAX,
    REID_DIMENSION,
    AttentionPlan,
    Body,
    SceneBuffer,
    assess,
    body_from_detection,
    cosine_distance,
    enroll_person,
    greeting_line,
    head_position,
    match_person,
    write_person_memory,
)
from pib_hermes_config.memory import MEMORY_FILENAME, iter_memory_files  # noqa: E402
from voice_assistant.attention import AttentionHandle  # noqa: E402


@pytest.fixture(autouse=True)
def people_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    return tmp_path


def _vector(first: float, second: float = 0.0) -> list[float]:
    values = [0.0] * REID_DIMENSION
    values[0] = first
    values[1] = second
    return values


def _unit(first: float) -> list[float]:
    second = math.sqrt(max(0.0, 1.0 - first * first))
    return _vector(first, second)


def _face(cx: float, *, embedding=None, gaze_yaw=None, frame_width=1280) -> Body:
    return Body(
        kind="face",
        x_min=cx - 20,
        y_min=300,
        x_max=cx + 20,
        y_max=340,
        frame_width=frame_width,
        frame_height=720,
        embedding=None if embedding is None else tuple(embedding),
        gaze_yaw=gaze_yaw,
    )


def _person(cx: float, size: float = 80) -> Body:
    return Body(
        kind="person",
        x_min=cx - size / 2,
        y_min=100,
        x_max=cx + size / 2,
        y_max=100 + size * 2,
        frame_width=1280,
        frame_height=720,
    )


def _hold(buffer: SceneBuffer, body: Body, start: float = 0.0) -> None:
    buffer.note_bodies("test", [body], start)
    buffer.note_bodies("test", [body], start + 0.5)


def test_the_head_turns_toward_a_face_on_the_right_before_the_plan_is_used():
    buffer = SceneBuffer()
    order = []
    handle = AttentionHandle(
        buffer, lambda motor, position: order.append((motor, position))
    )
    buffer.note_face_center(460, 0, now=1.0)

    plan = handle.before_answer(now=1.0)
    order.append("used")

    assert order[0] == (HEAD_MOTOR, 2480)
    assert order[1] == "used"
    assert plan.head_position == 2480
    assert plan.for_hermes("Hi") == "Hi"


def test_a_missing_face_does_not_turn_the_head():
    buffer = SceneBuffer()
    called = []
    handle = AttentionHandle(buffer, lambda motor, position: called.append(motor))
    buffer.note_face_center(0, 0, now=1.0)

    plan = handle.before_answer(now=1.0)

    assert called == []
    assert plan.head_motor is None


def test_the_head_position_is_clamped_to_the_seeded_yaw_range():
    body = Body(
        kind="person",
        x_min=12000,
        y_min=0,
        x_max=13600,
        y_max=100,
        frame_width=1280,
        frame_height=720,
    )
    assert head_position(body) == HEAD_POSITION_MAX


def test_direction_of_arrival_selects_the_person_on_that_side():
    buffer = SceneBuffer()
    buffer.note_doa(90)
    buffer.note_bodies("test", [_person(200), _person(1100)], now=1.0)

    plan = assess(buffer, now=1.0)

    assert plan.head_motor == HEAD_MOTOR
    assert plan.head_position == 2480


def test_sound_from_behind_the_camera_does_not_pick_someone_in_front():
    buffer = SceneBuffer()
    buffer.note_doa(180)
    buffer.note_bodies("test", [_person(200, size=200), _person(1100)], now=1.0)

    plan = assess(buffer, now=1.0)

    assert plan.head_motor is None


def test_a_face_is_the_speaker_ahead_of_a_larger_body():
    buffer = SceneBuffer()
    buffer.note_bodies(
        "test",
        [_face(200), _person(1100, size=400)],
        now=1.0,
    )

    plan = assess(buffer, now=1.0)

    assert plan.head_position == -2372


def test_a_recognised_person_is_named_and_keeps_only_their_memory(tmp_path):
    ada = _vector(1.0)
    bea = _vector(0.0, 1.0)
    enroll_person("Ada", ada)
    enroll_person("Bea", bea)
    write_person_memory("ada", "Ada likes tea.")
    write_person_memory("bea", "Bea secret.")

    buffer = SceneBuffer()
    _hold(buffer, _face(640, embedding=ada))
    plan = assess(buffer, now=0.5)

    assert plan.name == "Ada"
    assert "Ada likes tea." in plan.preface
    assert "Address them by that name." in plan.preface
    assert "Bea" not in plan.preface
    assert "Bea secret." not in plan.preface
    assert plan.for_hermes("What time is it?") == (
        plan.preface + "\n\nWhat time is it?"
    )
    assert plan.for_direct("Du bist pib.") == "Du bist pib.\n\n" + plan.preface
    assert iter_memory_files(str(tmp_path / "profiles")) == []
    assert not (tmp_path / "profiles").joinpath("memories", MEMORY_FILENAME).exists()


def test_an_unknown_embedding_is_not_given_a_name():
    enroll_person("Ada", _vector(1.0))
    buffer = SceneBuffer()
    _hold(buffer, _face(640, embedding=_vector(0.0, 1.0)))

    plan = assess(buffer, now=0.5)

    assert plan.name is None
    assert plan.preface == ""
    assert match_person(_vector(0.0, 1.0)) is None


def test_cosine_distance_accepts_the_same_direction_and_rejects_another():
    same = _unit(0.51)
    other = _unit(0.49)
    assert cosine_distance(_vector(1.0), same) == pytest.approx(0.49)
    assert cosine_distance(_vector(1.0), other) == pytest.approx(0.51)
    enroll_person("Ada", _vector(1.0))
    assert match_person(same)["name"] == "Ada"
    assert match_person(other) is None
    with pytest.raises(ValueError):
        enroll_person("No", [1.0, 0.0])


def test_a_still_face_is_looking_and_a_fast_crossing_is_walking_past():
    ada = _vector(1.0)
    enroll_person("Ada", ada)
    write_person_memory("ada", "Ada likes tea.")

    looking = SceneBuffer()
    _hold(looking, _face(640, embedding=ada))
    looked = assess(looking, now=0.5)
    assert looked.audience == "looking"
    assert looked.greeting == "Hallo Ada."

    passing = SceneBuffer()
    passing.note_bodies("test", [_face(100, embedding=ada)], 0.0)
    passing.note_bodies("test", [_face(400, embedding=ada)], 0.5)
    walked = assess(passing, now=0.5)
    assert walked.audience == "passing"
    assert walked.greeting is None
    assert walked.head_motor == HEAD_MOTOR
    assert "walking past" in walked.preface
    assert "Do not greet them." in walked.preface
    assert "Ada likes tea." in walked.preface


def test_face_center_motion_distinguishes_a_look_from_a_pass_by():
    """The live signal is the Haar face_center topic, not a started detector."""
    still = SceneBuffer()
    still.note_face_center(20, 0, now=0.0)
    still.note_face_center(24, 0, now=0.5)
    assert assess(still, now=0.5).audience == "looking"

    crossing = SceneBuffer()
    crossing.note_face_center(-400, 0, now=0.0)
    crossing.note_face_center(200, 0, now=0.5)
    passed = assess(crossing, now=0.5)
    assert passed.audience == "passing"
    assert passed.greeting is None
    assert "Do not greet them." in passed.preface


def test_a_face_turned_away_is_not_looking():
    buffer = SceneBuffer()
    body = _face(640, gaze_yaw=40)
    _hold(buffer, body)
    plan = assess(buffer, now=0.5)
    assert plan.audience == "none"
    assert plan.greeting is None


def test_one_frame_turns_the_head_but_does_not_yet_greet():
    ada = _vector(1.0)
    enroll_person("Ada", ada)
    buffer = SceneBuffer()
    buffer.note_bodies("test", [_face(1100, embedding=ada)], 0.0)
    plan = assess(buffer, now=0.0)
    assert plan.head_position == 2480
    assert plan.audience == "none"
    assert plan.greeting is None
    assert plan.name == "Ada"


def test_a_cooldown_prevents_greeting_the_same_person_twice():
    ada = _vector(1.0)
    enroll_person("Ada", ada)
    buffer = SceneBuffer()
    _hold(buffer, _face(640, embedding=ada))
    handle = AttentionHandle(buffer, lambda _motor, _position: None)

    first = handle.take_opening_greeting(language="German", now=0.5)
    buffer.note_bodies("test", [_face(640, embedding=ada)], 9.5)
    buffer.note_bodies("test", [_face(640, embedding=ada)], 10.0)
    second = handle.take_opening_greeting(language="German", now=10.0)
    assert "Do not greet them again." in assess(buffer, now=10.0).preface
    later_at = 0.5 + 600 + 1
    buffer.note_bodies("test", [_face(640, embedding=ada)], later_at - 0.5)
    buffer.note_bodies("test", [_face(640, embedding=ada)], later_at)
    later = handle.take_opening_greeting(language="English", now=later_at)

    assert first == "Hallo Ada."
    assert second is None
    assert later == "Hello Ada."


def test_answering_a_recognised_person_uses_up_the_greeting():
    ada = _vector(1.0)
    enroll_person("Ada", ada)
    buffer = SceneBuffer()
    _hold(buffer, _face(640, embedding=ada))
    handle = AttentionHandle(buffer, lambda _motor, _position: None)

    plan = handle.before_answer(now=0.5)

    assert plan.name == "Ada"
    assert handle.take_opening_greeting(now=1.0) is None


def test_greeting_line_follows_the_personality_language():
    assert greeting_line("Ada", "German") == "Hallo Ada."
    assert greeting_line("Ada", None) == "Hallo Ada."
    assert greeting_line("Ada", "English") == "Hello Ada."


def test_a_detection_message_keeps_a_face_yaw_and_drops_other_labels():
    face = body_from_detection(
        SimpleNamespace(
            label="Face",
            score=0.9,
            x_min=10,
            y_min=20,
            x_max=30,
            y_max=40,
            scalar_names=["yaw", "pitch", "roll"],
            scalar_values=[4.0, 1.0, 0.0],
        ),
        1280,
        720,
    )
    assert face is not None
    assert face.kind == "face"
    assert face.gaze_yaw == 4.0
    assert face.embedding is None
    assert (
        body_from_detection(
            SimpleNamespace(
                label="car",
                x_min=0,
                y_min=0,
                x_max=1,
                y_max=1,
                scalar_names=[],
                scalar_values=[],
            ),
            1280,
            720,
        )
        is None
    )


def test_an_empty_preface_leaves_the_turn_text_unchanged():
    plan = AttentionPlan()
    assert plan.for_hermes("Hi") == "Hi"
    assert plan.for_direct("Du bist pib.") == "Du bist pib."
