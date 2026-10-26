"""One system-prompt builder for Smart (SOUL.md) and Direct."""

from pib_hermes_config import build_default_soul_text
from pib_hermes_config.channel import (
    ROBOT_CONTEXT_INSTRUCTION,
    ROBOT_CONTEXT_MAX_CHARS,
    direct_system_prompt,
    personality_system_prompt,
)


def test_smart_and_direct_share_the_same_base_prompt():
    body = "Body (centidegrees: 1 = 0.01 degree)."
    custom = "Sei freundlich."
    soul = build_default_soul_text("Eva", custom).strip()
    smart = personality_system_prompt(
        personality_name="Eva",
        custom_description=custom,
        robot_context=body,
    )
    direct = personality_system_prompt(soul_text=soul, robot_context=body)
    assert smart == direct


def test_robot_context_is_present_in_both_modes():
    body = "Head: turn_head_motor -9000..9000."
    prompt = personality_system_prompt(
        personality_name="Eva",
        custom_description="Hi.",
        robot_context=body,
    )
    assert ROBOT_CONTEXT_INSTRUCTION in prompt
    assert body in prompt
    assert len(body) <= ROBOT_CONTEXT_MAX_CHARS


def test_robot_context_is_capped_at_two_thousand_characters():
    body = "x" * 2500
    prompt = personality_system_prompt(
        personality_name="Eva",
        robot_context=body,
    )
    embedded = prompt.split(ROBOT_CONTEXT_INSTRUCTION, 1)[-1].strip()
    assert len(embedded) <= ROBOT_CONTEXT_MAX_CHARS


def test_code_generation_prefix_is_applied(monkeypatch):
    soul = build_default_soul_text("Eva").strip()
    prefix = "Generate Blockly code only.\n"
    monkeypatch.setattr(
        "pib_hermes_config.channel.CODE_GENERATION_DESCRIPTION_PREFIX", prefix
    )
    with_prefix = personality_system_prompt(soul_text=soul, code_generation=True)
    assert with_prefix.startswith(prefix + soul)


def test_direct_system_prompt_adds_only_transport_specific_image_sentence():
    body = "Body line."
    wrapped = direct_system_prompt(
        "Du bist Eva.",
        images_shown=True,
        robot_context=body,
    )
    assert wrapped.startswith("Du bist Eva.")
    assert "attached to your answer" in wrapped
    assert wrapped.endswith(body)


def test_build_default_soul_text_includes_robot_context_when_given():
    body = "Body line."
    text = build_default_soul_text("Eva", "Sei nett.", robot_context=body)
    assert body in text
    assert text.endswith("\n")
