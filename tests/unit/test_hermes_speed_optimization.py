"""PR-1524: Hermes high-speed Gemini Flash / Flash-Lite configuration defaults."""
import os
from unittest.mock import patch

import yaml

import pib_hermes_config as hermes_cfg
from public_api_client.hermes_agent_client import (
    DEFAULT_HERMES_LITE_MODEL,
    DEFAULT_HERMES_MODEL,
    DEFAULT_HERMES_PROVIDER,
    DEFAULT_MAX_TOKENS,
    DEFAULT_REASONING_EFFORT,
    DEFAULT_TEMPERATURE,
    ensure_profile,
    profile_dir_for,
)


def _absent_binary(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))


def _load_profile_config(pdir):
    with open(os.path.join(pdir, "config.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _assert_speed_defaults(cfg):
    assert cfg["reasoning_effort"] == DEFAULT_REASONING_EFFORT
    assert cfg["max_tokens"] == DEFAULT_MAX_TOKENS
    assert cfg["temperature"] == DEFAULT_TEMPERATURE


def test_speed_constants_in_hermes_agent_client():
    assert DEFAULT_HERMES_MODEL == "gemini-3.5-flash"
    assert DEFAULT_HERMES_LITE_MODEL == "gemini-3.5-flash-lite"
    assert DEFAULT_HERMES_PROVIDER == "gemini"
    assert DEFAULT_REASONING_EFFORT == "low"
    assert DEFAULT_MAX_TOKENS == 1024
    assert DEFAULT_TEMPERATURE == 0.3


def test_speed_constants_in_pib_hermes_config():
    assert hermes_cfg.DEFAULT_HERMES_MODEL == "gemini-3.5-flash"
    assert hermes_cfg.DEFAULT_HERMES_LITE_MODEL == "gemini-3.5-flash-lite"
    assert hermes_cfg.DEFAULT_HERMES_PROVIDER == "gemini"
    assert hermes_cfg.DEFAULT_REASONING_EFFORT == "low"
    assert hermes_cfg.DEFAULT_MAX_TOKENS == 1024
    assert hermes_cfg.DEFAULT_TEMPERATURE == 0.3


def test_gemini_flash_lite_model_string_is_supported():
    """Testfall 3: gemini-3.5-flash-lite is a first-class model constant."""
    assert DEFAULT_HERMES_LITE_MODEL == "gemini-3.5-flash-lite"
    assert hermes_cfg.DEFAULT_HERMES_LITE_MODEL == "gemini-3.5-flash-lite"
    assert DEFAULT_HERMES_LITE_MODEL.endswith("-lite")
    assert "flash" in DEFAULT_HERMES_LITE_MODEL


def test_ensure_profile_seeds_speed_defaults_on_fresh_profile(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Testfall 5: new personalities inherit optimized speed settings."""
    _absent_binary(tmp_path, monkeypatch)

    pdir = ensure_profile("speed-fresh", soul_text="Du bist pib.")

    cfg = _load_profile_config(pdir)
    assert cfg["model"] == DEFAULT_HERMES_MODEL
    assert cfg["provider"] == DEFAULT_HERMES_PROVIDER
    _assert_speed_defaults(cfg)


def test_ensure_profile_seeds_speed_defaults_into_existing_config(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Existing profile config.yaml is repaired with speed defaults on ensure_profile."""
    (sandboxed_hermes_home / "config.yaml").write_text(
        "model: anthropic/claude-opus-5\n", encoding="utf-8"
    )
    _absent_binary(tmp_path, monkeypatch)
    pdir = profile_dir_for("speed-existing")
    os.makedirs(pdir)
    with open(os.path.join(pdir, "config.yaml"), "w", encoding="utf-8") as fh:
        fh.write("model: custom/operator-model\nprovider: openrouter\n")

    ensure_profile("speed-existing", soul_text="Du bist pib.")

    cfg = _load_profile_config(pdir)
    assert cfg["model"] == DEFAULT_HERMES_MODEL
    assert cfg["provider"] == DEFAULT_HERMES_PROVIDER
    _assert_speed_defaults(cfg)


def test_ensure_profile_overwrites_non_speed_reasoning_settings(
    tmp_path, monkeypatch, sandboxed_hermes_home
):
    """Testfall 1/2: reasoning_effort and max_tokens are permanently pinned low/1024."""
    (sandboxed_hermes_home / "config.yaml").write_text(
        "model: gemini-3.5-flash\n", encoding="utf-8"
    )
    _absent_binary(tmp_path, monkeypatch)
    pdir = profile_dir_for("speed-pin")
    os.makedirs(pdir)
    with open(os.path.join(pdir, "config.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "model": "gemini-3.5-flash",
                "provider": "gemini",
                "reasoning_effort": "high",
                "max_tokens": 8192,
                "temperature": 1.0,
            },
            fh,
        )

    with patch("public_api_client.hermes_agent_client.subprocess.run"):
        ensure_profile("speed-pin", soul_text="Du bist pib.")

    cfg = _load_profile_config(pdir)
    _assert_speed_defaults(cfg)
