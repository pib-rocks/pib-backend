"""Verify setup-pib.sh permanently pins Gemini in the Hermes base config."""

from __future__ import annotations

import re
import textwrap
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"


def _extract_seed_hermes_python() -> str:
    """Pull the inline python used by seed_hermes_mcp_config out of setup-pib.sh."""
    script = SETUP_PIB.read_text(encoding="utf-8")
    match = re.search(
        r'sudo -u pib -H python3 -c "\n(?P<body>.*?)\n"',
        script,
        re.DOTALL,
    )
    assert match, "seed_hermes_mcp_config python snippet not found in setup-pib.sh"
    return textwrap.dedent(match.group("body"))


def test_setup_pib_pins_gemini_model_in_hermes_config(tmp_path, monkeypatch):
    cfg_path = tmp_path / "config.yaml"
    cfg_path.write_text("model: anthropic/claude-opus-5\n", encoding="utf-8")

    snippet = _extract_seed_hermes_python()
    snippet = snippet.replace("'/home/pib/.hermes/config.yaml'", repr(str(cfg_path)))

    namespace: dict = {"__name__": "__main__"}
    exec(compile(snippet, str(SETUP_PIB), "exec"), namespace)

    with open(cfg_path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)

    assert cfg["model"] == "gemini-3.5-flash"
    assert cfg["provider"] == "gemini"
    assert cfg["mcp_servers"]["pib"] == {
        "command": "python3",
        "args": ["-m", "pib_mcp_server"],
    }


def test_setup_pib_overwrites_existing_model_and_keeps_mcp(tmp_path):
    cfg_path = tmp_path / "config.yaml"
    with open(cfg_path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "model": "custom/operator-model",
                "provider": "openrouter",
                "mcp_servers": {
                    "pib": {"command": "python3", "args": ["-m", "pib_mcp_server"]}
                },
            },
            fh,
        )

    snippet = _extract_seed_hermes_python()
    snippet = snippet.replace("'/home/pib/.hermes/config.yaml'", repr(str(cfg_path)))
    exec(compile(snippet, str(SETUP_PIB), "exec"), {"__name__": "__main__"})

    with open(cfg_path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)

    assert cfg["model"] == "gemini-3.5-flash"
    assert cfg["provider"] == "gemini"
    assert cfg["mcp_servers"]["pib"]["args"] == ["-m", "pib_mcp_server"]
