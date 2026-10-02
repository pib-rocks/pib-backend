"""Verify seed_db includes Gemini 3.8 Flash and omits the removed hermes row."""


def test_seed_does_not_include_hermes_agent(app_ctx):
    from model.assistant_model import AssistantModel

    assert AssistantModel.query.filter_by(api_name="hermes-agent").count() == 0


def test_seed_includes_gemini_3_8_flash_assistant_model(app_ctx):
    from model.assistant_model import AssistantModel

    gemini = AssistantModel.query.filter_by(api_name="gemini-3.8-flash").one()
    assert gemini.visual_name == "Gemini 3.8 Flash"
    assert gemini.has_image_support is True


def test_get_all_assistant_models_returns_gemini_3_8_flash(app_ctx):
    from service.assistant_model_service import get_all_assistant_models

    api_names = {model.api_name for model in get_all_assistant_models()}
    assert "gemini-3.8-flash" in api_names
