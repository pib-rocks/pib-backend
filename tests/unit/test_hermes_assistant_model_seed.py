"""Verify seed_db includes hermes-agent and Gemini 3.5 Flash assistant models."""


def test_seed_includes_hermes_agent_assistant_model(app_ctx):
    from model.assistant_model import AssistantModel

    hermes = AssistantModel.query.filter_by(api_name="hermes-agent").one()
    assert hermes.visual_name == "Hermes Agent (selbstlernend)"
    assert hermes.has_image_support is True


def test_seed_includes_gemini_3_5_flash_assistant_model(app_ctx):
    from model.assistant_model import AssistantModel

    gemini = AssistantModel.query.filter_by(api_name="gemini-3.5-flash").one()
    assert gemini.visual_name == "Gemini 3.5 Flash"
    assert gemini.has_image_support is False


def test_get_all_assistant_models_returns_gemini_3_5_flash(app_ctx):
    from service.assistant_model_service import get_all_assistant_models

    api_names = {model.api_name for model in get_all_assistant_models()}
    assert "gemini-3.5-flash" in api_names
