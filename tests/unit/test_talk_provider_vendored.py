"""The vendored Hermes Talk provider builds the Live setup the robot needs."""

from voice_assistant.talk_provider import talk_gemini_realtime as provider
from voice_assistant.talk_provider import talk_realtime as rt


def _setup(**overrides):
    base = dict(
        model="models/gemini-3.8-live",
        voice="Puck",
        instructions="Du bist pib.",
    )
    base.update(overrides)
    return rt.SessionSetup(**base)


def test_a_setup_carries_model_voice_and_instructions():
    message = provider.build_setup_message(_setup())
    body = message["setup"]
    assert body["model"] == "models/gemini-3.8-live"
    assert body["generationConfig"]["speechConfig"]["voiceConfig"][
        "prebuiltVoiceConfig"
    ]["voiceName"] == "Puck"
    assert body["systemInstruction"]["parts"][0]["text"] == "Du bist pib."


def test_tool_schemas_are_reduced_to_the_live_vocabulary():
    """The Live API closes the socket with 1007 on any unknown schema key."""

    definition = rt.ToolDefinition(
        name="search_memory",
        description="Look something up.",
        parameters={
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "query": {"type": "string", "minLength": 1, "minimum": 0},
                "limit": {
                    "type": "integer",
                    "minimum": 1,
                    "maximum": 20,
                    "default": 5,
                },
            },
            "required": ["query"],
        },
    )

    message = provider.build_setup_message(_setup(tools=(definition,)))
    params = message["setup"]["tools"][0]["functionDeclarations"][0]["parameters"]

    assert params["type"] == "OBJECT"
    assert set(params["properties"]) == {"query", "limit"}
    assert "additionalProperties" not in params
    assert "minimum" not in params["properties"]["limit"]
    assert "default" not in params["properties"]["limit"]
    assert params["properties"]["query"]["type"] == "STRING"


def test_an_ephemeral_token_is_not_used_as_a_query_key():
    """Tokens ride Authorization: Token auth_tokens/… on the constrained route.

    Asserted at the source level because the branch lives inside the socket
    handshake; the SDK does exactly this for api_key.startswith("auth_tokens/").
    """

    import inspect

    source = inspect.getsource(provider)
    assert "BidiGenerateContentConstrained" in source
    assert 'ws_headers = {"Authorization": f"Token {auth_token}"}' in source
    assert 'auth_token.startswith("auth_tokens/")' in source
