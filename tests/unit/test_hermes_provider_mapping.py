"""PR-1930b: the smart chat runs the personality's model.

Covers the provider mapping helper, the profile config.yaml writer, and the
settings reader a turn uses to run the personality's model instead of a pinned
default.
"""

from __future__ import annotations

import os
from unittest.mock import patch

import yaml

import pib_hermes_config as cfg
from pib_hermes_config import (
    provider_for_profile,
    provider_needs_key,
    env_vars_for_provider,
)

from public_api_client.hermes_agent_client import (
    DEFAULT_HERMES_MODEL,
    DEFAULT_HERMES_PROVIDER,
    DEFAULT_MAX_TOKENS,
    DEFAULT_REASONING_EFFORT,
    DEFAULT_TEMPERATURE,
    PIB_MCP_SERVER,
    _child_environment,
    _ensure_mcp_servers_pib,
    profile_hermes_settings,
    provider_key_for_turn,
)


def _load(pdir):
    with open(os.path.join(pdir, "config.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# --- provider mapping: each of the five registry providers -----------------


def test_google_maps_to_the_gemini_provider():
    mapping = provider_for_profile("Google")
    assert mapping.provider == "gemini"
    assert mapping.env_vars == ("GOOGLE_API_KEY", "GEMINI_API_KEY")
    assert mapping.base_url is None


def test_openai_maps_to_the_openai_provider():
    mapping = provider_for_profile("OpenAI")
    assert mapping.provider == "openai"
    assert mapping.env_vars == ("OPENAI_API_KEY",)
    assert mapping.base_url is None


def test_anthropic_maps_to_the_anthropic_provider():
    mapping = provider_for_profile("Anthropic")
    assert mapping.provider == "anthropic"
    assert mapping.env_vars == ("ANTHROPIC_API_KEY",)
    assert mapping.base_url is None


def test_pib_cloud_maps_to_a_user_defined_route():
    mapping = provider_for_profile("pib.Cloud", "https://cloud.example.com/v1")
    assert mapping.provider == "pib-cloud"
    assert mapping.env_vars == ("PIB_CLOUD_API_KEY",)
    assert mapping.base_url == "https://cloud.example.com/v1"
    assert provider_needs_key(mapping.provider, mapping.env_vars[0]) is True


def test_pib_cloud_without_a_registry_endpoint_defaults_to_pib_rocks(monkeypatch):
    monkeypatch.delenv("PIB_CLOUD_BASE_URL", raising=False)
    mapping = provider_for_profile("pib.Cloud", None)
    assert mapping.provider == "pib-cloud"
    assert mapping.env_vars == ("PIB_CLOUD_API_KEY",)
    assert mapping.base_url == "https://api.pib.rocks/v1"


def test_local_maps_to_a_keyless_openai_route():
    mapping = provider_for_profile("Local", "http://host.docker.internal:11434/v1")
    assert mapping.provider == "local"
    assert mapping.env_vars == ()
    assert mapping.base_url == "http://host.docker.internal:11434/v1"
    assert provider_needs_key(mapping.provider) is False


def test_local_without_a_registry_endpoint_falls_back_to_ollama():
    with patch.dict(os.environ, {"PIB_OLLAMA_BASE_URL": "http://127.0.0.1:11434"}):
        mapping = provider_for_profile("Local")
    assert mapping.provider == "local"
    assert mapping.base_url == "http://127.0.0.1:11434/v1"


def test_unknown_provider_without_a_route_falls_back_to_gemini():
    mapping = provider_for_profile("Mystery")
    assert mapping.provider == DEFAULT_HERMES_PROVIDER
    assert mapping.env_vars == ("GOOGLE_API_KEY", "GEMINI_API_KEY")


def test_unknown_provider_with_a_route_becomes_its_own_entry():
    mapping = provider_for_profile("Mistral", "https://api.mistral.ai/v1")
    assert mapping.provider == "mistral"
    assert mapping.base_url == "https://api.mistral.ai/v1"
    assert mapping.env_vars == ()


def test_env_vars_for_provider_builtin_and_custom_key_env_table():
    assert env_vars_for_provider("local") == ()
    assert env_vars_for_provider("pib-cloud") == ("PIB_CLOUD_API_KEY",)
    assert env_vars_for_provider("pib-cloud", "PIB_CLOUD_API_KEY") == (
        "PIB_CLOUD_API_KEY",
    )
    assert env_vars_for_provider("gemini") == ("GOOGLE_API_KEY", "GEMINI_API_KEY")
    assert env_vars_for_provider("openai") == ("OPENAI_API_KEY",)
    assert env_vars_for_provider("anthropic") == ("ANTHROPIC_API_KEY",)


# --- config.yaml writer ----------------------------------------------------


def test_writer_stores_the_personality_model_and_provider(
    tmp_path, sandboxed_hermes_home
):
    pdir = tmp_path / "pib_pers"
    pdir.mkdir()

    _ensure_mcp_servers_pib(str(pdir), model="gpt-6", provider="openai")

    cfg = _load(str(pdir))
    assert cfg["model"] == "gpt-6"
    assert cfg["provider"] == "openai"


def test_writer_repairs_mcp_servers_pib_while_keeping_the_model(tmp_path):
    """The existing mcp_servers.pib repair stays exactly as it was."""
    pdir = tmp_path / "pib_pers"
    pdir.mkdir()
    custom = {"command": "python3", "args": ["-m", "custom_mcp"]}
    with open(os.path.join(str(pdir), "config.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {"model": "gpt-6", "provider": "openai", "mcp_servers": {"pib": custom}},
            fh,
        )

    _ensure_mcp_servers_pib(str(pdir), model="gpt-6", provider="openai")

    cfg = _load(str(pdir))
    entry = cfg["mcp_servers"]["pib"]
    assert entry["command"] == custom["command"]
    assert entry["args"] == custom["args"]
    assert entry["env"] == PIB_MCP_SERVER["env"]


def test_writer_adds_a_user_defined_route_for_local(tmp_path):
    pdir = tmp_path / "pib_local"
    pdir.mkdir()

    _ensure_mcp_servers_pib(
        str(pdir),
        model="qwen-fast",
        provider="local",
        base_url="http://host.docker.internal:11434/v1",
        env_vars=(),
    )

    cfg = _load(str(pdir))
    assert cfg["model"] == "qwen-fast"
    assert cfg["provider"] == "local"
    entry = cfg["providers"]["local"]
    assert entry["base_url"] == "http://host.docker.internal:11434/v1"
    assert "key_env" not in entry


def test_writer_adds_the_key_env_for_a_keyed_custom_route(tmp_path):
    pdir = tmp_path / "pib_cloud"
    pdir.mkdir()

    _ensure_mcp_servers_pib(
        str(pdir),
        model="pib-cloud",
        provider="pib-cloud",
        base_url="https://cloud.example.com/v1",
        env_vars=("PIB_CLOUD_API_KEY",),
    )

    cfg = _load(str(pdir))
    entry = cfg["providers"]["pib-cloud"]
    assert entry["base_url"] == "https://cloud.example.com/v1"
    assert entry["key_env"] == "PIB_CLOUD_API_KEY"


def test_writer_keeps_an_existing_model_when_none_is_supplied(tmp_path):
    pdir = tmp_path / "pib_keep"
    pdir.mkdir()
    with open(os.path.join(str(pdir), "config.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump({"model": "gpt-6", "provider": "openai"}, fh)

    _ensure_mcp_servers_pib(str(pdir))

    cfg = _load(str(pdir))
    assert cfg["model"] == "gpt-6"
    assert cfg["provider"] == "openai"


def test_writer_seeds_the_pinned_default_into_an_empty_profile(tmp_path):
    pdir = tmp_path / "pib_empty"
    pdir.mkdir()

    _ensure_mcp_servers_pib(str(pdir))

    cfg = _load(str(pdir))
    assert cfg["model"] == DEFAULT_HERMES_MODEL
    assert cfg["provider"] == DEFAULT_HERMES_PROVIDER


def test_writer_seeds_speed_defaults_only_when_absent(tmp_path):
    pdir = tmp_path / "pib_speed"
    pdir.mkdir()

    _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")
    cfg = _load(str(pdir))
    assert cfg["agent"]["reasoning_effort"] == DEFAULT_REASONING_EFFORT
    assert cfg["agent"]["max_tokens"] == DEFAULT_MAX_TOKENS
    assert cfg["agent"]["temperature"] == DEFAULT_TEMPERATURE
    assert "reasoning_effort" not in cfg
    assert "max_tokens" not in cfg
    assert "temperature" not in cfg

    with open(os.path.join(str(pdir), "config.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "model": "gemini-3.8-flash",
                "provider": "gemini",
                "agent": {
                    "reasoning_effort": "high",
                    "max_tokens": 8192,
                    "temperature": 1.0,
                },
            },
            fh,
        )
    _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")
    cfg = _load(str(pdir))
    assert cfg["agent"]["reasoning_effort"] == "high"
    assert cfg["agent"]["max_tokens"] == 8192
    assert cfg["agent"]["temperature"] == 1.0
    assert "reasoning_effort" not in cfg


# --- settings reader -------------------------------------------------------


def test_profile_settings_resolve_pib_cloud_env_vars_without_yaml_key_env(tmp_path):
    """PC-114: keyed routes must resolve from the builtin table alone."""
    pdir = tmp_path / "pib_read"
    pdir.mkdir()
    with open(os.path.join(str(pdir), "config.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "model": "pib-cloud",
                "provider": "pib-cloud",
                "providers": {
                    "pib-cloud": {"base_url": "https://api.pib.rocks/v1"},
                },
            },
            fh,
        )

    _model, provider, _base_url, env_vars = profile_hermes_settings(str(pdir))
    assert provider == "pib-cloud"
    assert env_vars == ("PIB_CLOUD_API_KEY",)


def test_profile_settings_read_back_the_written_model_and_key_env(tmp_path):
    pdir = tmp_path / "pib_read"
    pdir.mkdir()
    _ensure_mcp_servers_pib(
        str(pdir),
        model="pib-cloud",
        provider="pib-cloud",
        base_url="https://cloud.example.com/v1",
        env_vars=("PIB_CLOUD_API_KEY",),
    )

    model, provider, base_url, env_vars = profile_hermes_settings(str(pdir))
    assert model == "pib-cloud"
    assert provider == "pib-cloud"
    assert base_url == "https://cloud.example.com/v1"
    assert env_vars == ("PIB_CLOUD_API_KEY",)


def test_profile_settings_default_when_no_config(tmp_path):
    model, provider, base_url, env_vars = profile_hermes_settings(str(tmp_path))
    assert model == DEFAULT_HERMES_MODEL
    assert provider == DEFAULT_HERMES_PROVIDER
    assert base_url is None
    assert env_vars == ("GOOGLE_API_KEY", "GEMINI_API_KEY")


def test_config_module_exposes_the_mapping_table():
    """The helper lives in pib_hermes_config, the two-process shared module."""
    assert cfg.HERMES_PROVIDER_LOCAL == "local"
    assert cfg.PIB_PROVIDER_LOCAL == "Local"
    assert cfg.provider_for_profile("Local").provider == "local"


# --- PC-114: pib-cloud key resolution on the Smart / Hermes path -------------


def _write_pib_cloud_profile(pdir: str) -> None:
    _ensure_mcp_servers_pib(
        pdir,
        model="pib-cloud",
        provider="pib-cloud",
        base_url="https://api.pib.rocks/v1",
        env_vars=("PIB_CLOUD_API_KEY",),
    )


def test_smart_pib_cloud_turn_fetches_pib_cloud_key_and_sets_turn_env(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """A pib-cloud profile asks the store for pib-cloud, not gemini."""
    import sys
    from pathlib import Path

    from public_api_client import hermes_agent_client as hac
    from voice_assistant import direct_tool_loop

    voice = Path(__file__).resolve().parents[2] / "ros_packages" / "voice_assistant"
    if str(voice) not in sys.path:
        sys.path.insert(0, str(voice))

    pdir = tmp_path / "pib_j1"
    pdir.mkdir()
    _write_pib_cloud_profile(str(pdir))

    seen: list[str] = []

    def fetch(provider, api_name=None):
        seen.append(provider)
        return {"mode": "unlocked", "secret": "cloud-store-secret"}

    monkeypatch.setattr(direct_tool_loop, "_fetch_store_key", fetch)
    monkeypatch.setattr(
        hac,
        "profile_hermes_settings",
        lambda profile_dir=None: profile_hermes_settings(str(pdir)),
    )

    key = provider_key_for_turn()
    _model, provider, _base_url, env_vars = profile_hermes_settings(str(pdir))
    child_env = _child_environment(key, env_vars)

    assert seen == ["pib-cloud"]
    assert key == "cloud-store-secret"
    assert child_env is not None
    assert child_env["PIB_CLOUD_API_KEY"] == "cloud-store-secret"
    assert provider == "pib-cloud"


def test_missing_pib_cloud_key_error_names_pib_cloud_not_gemini(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    import sys
    from pathlib import Path

    import pytest

    from public_api_client import hermes_agent_client as hac
    from voice_assistant import direct_tool_loop
    from voice_assistant.direct_tool_loop import DirectToolLoopError

    voice = Path(__file__).resolve().parents[2] / "ros_packages" / "voice_assistant"
    if str(voice) not in sys.path:
        sys.path.insert(0, str(voice))

    pdir = tmp_path / "pib_j1"
    pdir.mkdir()
    _write_pib_cloud_profile(str(pdir))

    monkeypatch.setattr(
        direct_tool_loop,
        "_fetch_store_key",
        lambda provider, api_name=None: {"mode": "unlocked", "secret": None},
    )
    monkeypatch.setattr(
        hac,
        "profile_hermes_settings",
        lambda profile_dir=None: profile_hermes_settings(str(pdir)),
    )

    with pytest.raises(DirectToolLoopError) as caught:
        provider_key_for_turn()

    message = str(caught.value)
    assert "pib-cloud" in message
    assert "gemini" not in message.lower()


def test_catalogue_pib_cloud_maps_through_profile_key_resolution(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Flask catalogue values -> provider_for_profile -> provider_key_for_turn."""
    import sys
    from pathlib import Path

    flask_root = Path(__file__).resolve().parents[2] / "pib_api" / "flask"
    if str(flask_root) not in sys.path:
        sys.path.insert(0, str(flask_root))

    from provider_registry import PIB_CLOUD_API_NAME, provider_name_for

    from public_api_client import hermes_agent_client as hac
    from voice_assistant import direct_tool_loop

    voice = Path(__file__).resolve().parents[2] / "ros_packages" / "voice_assistant"
    if str(voice) not in sys.path:
        sys.path.insert(0, str(voice))

    api_name = PIB_CLOUD_API_NAME
    registry_provider = provider_name_for(api_name)
    mapping = provider_for_profile(registry_provider)
    assert api_name == "pib-cloud"
    assert registry_provider == "pib.Cloud"
    assert mapping.provider == "pib-cloud"
    assert mapping.env_vars == ("PIB_CLOUD_API_KEY",)

    pdir = tmp_path / "pib_catalogue"
    pdir.mkdir()
    _ensure_mcp_servers_pib(
        str(pdir),
        model=api_name,
        provider=mapping.provider,
        base_url=mapping.base_url,
        env_vars=mapping.env_vars,
    )

    seen: list[str] = []

    def fetch(provider, api_name=None):
        seen.append(provider)
        return {"mode": "unlocked", "secret": "catalogue-key"}

    monkeypatch.setattr(direct_tool_loop, "_fetch_store_key", fetch)
    monkeypatch.setattr(
        hac,
        "profile_hermes_settings",
        lambda profile_dir=None: profile_hermes_settings(str(pdir)),
    )

    key = provider_key_for_turn()

    assert seen == ["pib-cloud"]
    assert key == "catalogue-key"
