"""Verify seed_db includes the hermes-agent assistant model."""


def test_seed_includes_hermes_agent_assistant_model(app_ctx):
    from model.assistant_model import AssistantModel

    hermes = AssistantModel.query.filter_by(api_name="hermes-agent").one()
    assert hermes.visual_name == "Hermes Agent (selbstlernend)"
    assert hermes.has_image_support is True
