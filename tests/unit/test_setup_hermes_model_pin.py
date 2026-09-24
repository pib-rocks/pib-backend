"""Verify setup-pib.sh permanently pins Gemini in the Hermes base config."""

from __future__ import annotations

import re
import textwrap
from pathlib import Path

import yaml
from public_api_client.hermes_agent_client import PIB_MCP_SERVER

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"

# What the installer must write when no override is in the environment. The MCP
# server is spawned by Hermes without the parent environment, so the entry has
# to carry these URLs itself.
EXPECTED_PIB_ENTRY = {
    "command": "python3",
    "args": ["-m", "pib_mcp_server"],
    "env": {
        "FLASK_API_BASE_URL": "http://flask-app:5000",
        "PIB_MCP_API_BASE_URL": "http://flask-app:5000",
        "PIB_MCP_ROSBRIDGE_URL": "ws://rosbridge-ws:9090",
    },
}


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


def _run_seed(cfg_path, monkeypatch) -> dict:
    """Execute the installer's seeding snippet against cfg_path and load the result."""
    for name in ("FLASK_API_BASE_URL", "PIB_MCP_ROSBRIDGE_URL"):
        monkeypatch.delenv(name, raising=False)

    snippet = _extract_seed_hermes_python()
    snippet = snippet.replace("'/home/pib/.hermes/config.yaml'", repr(str(cfg_path)))
    exec(compile(snippet, str(SETUP_PIB), "exec"), {"__name__": "__main__"})

    with open(cfg_path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def test_setup_pib_pins_gemini_model_in_hermes_config(tmp_path, monkeypatch):
    cfg_path = tmp_path / "config.yaml"
    cfg_path.write_text("model: anthropic/claude-opus-5\n", encoding="utf-8")

    cfg = _run_seed(cfg_path, monkeypatch)

    assert cfg["model"] == "gemini-3.5-flash"
    assert cfg["provider"] == "gemini"
    assert cfg["mcp_servers"]["pib"] == EXPECTED_PIB_ENTRY


def test_setup_pib_seeds_the_same_mcp_entry_as_the_client(tmp_path, monkeypatch):
    """Installer and hermes_agent_client must describe one and the same entry.

    The client keeps the entry as PIB_MCP_SERVER and the installer writes it in
    inline python; this is what fails when one of the two is changed alone.
    """
    cfg = _run_seed(tmp_path / "config.yaml", monkeypatch)

    entry = cfg["mcp_servers"]["pib"]
    assert entry["command"] == PIB_MCP_SERVER["command"]
    assert entry["args"] == PIB_MCP_SERVER["args"]
    assert set(entry["env"]) == set(PIB_MCP_SERVER["env"])
    assert entry == EXPECTED_PIB_ENTRY


def test_setup_pib_overwrites_existing_model_and_keeps_mcp(tmp_path, monkeypatch):
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

    cfg = _run_seed(cfg_path, monkeypatch)

    assert cfg["model"] == "gemini-3.5-flash"
    assert cfg["provider"] == "gemini"
    assert cfg["mcp_servers"]["pib"]["args"] == ["-m", "pib_mcp_server"]
    # Re-running the installer repairs an entry that was seeded without env.
    assert cfg["mcp_servers"]["pib"]["env"] == EXPECTED_PIB_ENTRY["env"]


def test_setup_pib_keeps_an_operator_customized_mcp_entry(tmp_path, monkeypatch):
    """Only missing keys are added: custom command/args/env values stay."""
    cfg_path = tmp_path / "config.yaml"
    with open(cfg_path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "mcp_servers": {
                    "pib": {
                        "command": "python3",
                        "args": ["-m", "custom_mcp"],
                        "env": {"FLASK_API_BASE_URL": "http://operators-own-host:5000"},
                    }
                },
            },
            fh,
        )

    entry = _run_seed(cfg_path, monkeypatch)["mcp_servers"]["pib"]

    assert entry["args"] == ["-m", "custom_mcp"]
    assert entry["env"]["FLASK_API_BASE_URL"] == "http://operators-own-host:5000"
    assert entry["env"]["PIB_MCP_API_BASE_URL"] == "http://flask-app:5000"
    assert entry["env"]["PIB_MCP_ROSBRIDGE_URL"] == "ws://rosbridge-ws:9090"
