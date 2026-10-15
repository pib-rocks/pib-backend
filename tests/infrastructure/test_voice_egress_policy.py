"""Load-bearing checks for the voice egress allowlist, renderer, and scripts.

These tests do not touch the robot or the network. They fail if the allowlist
stops being the source of the ruleset, if a required destination disappears,
or if the apply script no longer reverts a failed self-check.
"""

from __future__ import annotations

import importlib.util
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
EGRESS = ROOT / "setup" / "voice_egress"
ALLOWLIST_PATH = EGRESS / "allowlist.json"
APPLY = EGRESS / "apply.sh"
VERIFY = EGRESS / "verify.sh"


def _load():
    spec = importlib.util.spec_from_file_location(
        "voice_egress", EGRESS / "voice_egress.py"
    )
    module = importlib.util.module_from_spec(spec)
    # The module must be registered before execution: it defines dataclasses, and
    # dataclasses resolves annotations through sys.modules[cls.__module__].
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


mod = _load()


def _binding(**dns):
    addresses = dns or {
        "generativelanguage.googleapis.com": ("203.0.113.10", "203.0.113.11"),
    }
    return mod.RuntimeBinding(
        veth="vethvoice1",
        bridge="br-pibtest01",
        bridge_subnet="172.18.0.0/16",
        gateway="172.18.0.1",
        dns_addresses=addresses,
    )


def _tcp_tokens(text: str) -> set[tuple[str, str]]:
    found = set(re.findall(r"(\d+\.\d+\.\d+\.\d+) \. (\d+)", text))
    found |= set(re.findall(r"ip daddr (\d+\.\d+\.\d+\.\d+) tcp dport (\d+)", text))
    return found


def _chain(text: str, name: str) -> str:
    marker = f"chain {name} {{"
    start = text.index(marker)
    rest = text[start + len(marker) :]
    nxt = rest.find("\n    chain ")
    return rest if nxt < 0 else rest[:nxt]


def test_shipped_allowlist_keeps_required_destinations_and_disables_tryb():
    allow = mod.load_allowlist(str(ALLOWLIST_PATH))
    by_id = {entry.id: entry for entry in allow.entries}
    assert by_id["pib-cloud"].enabled is True
    assert by_id["pib-cloud"].destination == "192.168.1.252"
    assert by_id["pib-cloud"].port == 8090
    assert by_id["pib-network"].enabled is True
    assert by_id["gemini"].enabled is True
    assert {"direct", "smart", "live"} <= set(by_id["gemini"].modes)
    assert by_id["tryb"].enabled is False
    assert by_id["tryb"].destination == "platform.tryb.ai"


def test_static_preflight_refuses_a_missing_cloud_destination():
    raw = json.loads(ALLOWLIST_PATH.read_text(encoding="utf-8"))
    for entry in raw["entries"]:
        if entry["id"] == "pib-cloud":
            entry["destination"] = "10.0.0.1"
    with pytest.raises(mod.PolicyError, match="192.168.1.252"):
        mod.allowlist_from_dict(raw)


def test_rendered_tcp_allows_match_the_enabled_allowlist_exactly():
    allow = mod.load_allowlist(str(ALLOWLIST_PATH))
    text = mod.render_ruleset(allow, _binding())
    assert _tcp_tokens(text) == {
        ("192.168.1.252", "8090"),
        ("203.0.113.10", "443"),
        ("203.0.113.11", "443"),
        ("172.18.0.1", "11434"),
    }
    assert "platform.tryb.ai" not in text
    assert "198.51.100.20" not in text
    assert mod.PATH_LIMITATION in text
    assert mod.DNS_NOTE in text
    assert mod.IPV6_NOTE in text


def test_enabling_tryb_adds_only_that_destination():
    raw = json.loads(ALLOWLIST_PATH.read_text(encoding="utf-8"))
    for entry in raw["entries"]:
        if entry["id"] == "tryb":
            entry["enabled"] = True
    allow = mod.allowlist_from_dict(raw)
    text = mod.render_ruleset(
        allow,
        _binding(
            **{
                "generativelanguage.googleapis.com": ("203.0.113.10",),
                "platform.tryb.ai": ("198.51.100.20",),
            }
        ),
    )
    assert ("198.51.100.20", "443") in _tcp_tokens(text)
    assert ("203.0.113.10", "443") in _tcp_tokens(text)


def test_reject_is_reached_only_from_the_voice_veth():
    text = mod.render_ruleset(mod.load_allowlist(str(ALLOWLIST_PATH)), _binding())
    forward = _chain(text, "forward")
    voice = _chain(text, "voice_forward")
    assert "reject" not in forward
    assert 'iifname "vethvoice1" jump voice_forward' in forward
    assert "reject with tcp reset" in voice
    assert voice.index("meta nfproto ipv6 reject") < voice.index("ct state established")
    assert "224.0.0.0/4" in voice
    assert "oifname" in voice


def test_multicast_rule_follows_the_allowlist_entry():
    raw = json.loads(ALLOWLIST_PATH.read_text(encoding="utf-8"))
    for entry in raw["entries"]:
        if entry["id"] == "ros-dds-multicast":
            entry["enabled"] = False
    text = mod.render_ruleset(mod.allowlist_from_dict(raw), _binding())
    assert "224.0.0.0/4" not in text


def test_mode_filter_cannot_drop_direct_smart_or_live():
    allow = mod.load_allowlist(str(ALLOWLIST_PATH))
    with pytest.raises(mod.PolicyError, match="gemini"):
        mod.render_ruleset(allow, _binding(), modes=["common"])


def test_bridge_subnet_must_not_swallow_the_cloud_host_or_the_default_route():
    allow = mod.load_allowlist(str(ALLOWLIST_PATH))
    wide = mod.RuntimeBinding(
        veth="vethvoice1",
        bridge="br-pibtest01",
        bridge_subnet="192.168.1.0/24",
        gateway="192.168.1.1",
        dns_addresses={"generativelanguage.googleapis.com": ("203.0.113.10",)},
    )
    with pytest.raises(mod.PolicyError, match="cloud host"):
        mod.render_ruleset(allow, wide)
    default = mod.RuntimeBinding(
        veth="vethvoice1",
        bridge="br-pibtest01",
        bridge_subnet="0.0.0.0/0",
        gateway="172.18.0.1",
        dns_addresses={"generativelanguage.googleapis.com": ("203.0.113.10",)},
    )
    with pytest.raises(mod.PolicyError):
        mod.render_ruleset(allow, default)


def test_interface_names_cannot_inject_nft_syntax():
    allow = mod.load_allowlist(str(ALLOWLIST_PATH))
    hostile = mod.RuntimeBinding(
        veth='veth"; flush ruleset',
        bridge="br-pibtest01",
        bridge_subnet="172.18.0.0/16",
        gateway="172.18.0.1",
        dns_addresses={"generativelanguage.googleapis.com": ("203.0.113.10",)},
    )
    with pytest.raises(mod.PolicyError, match="veth"):
        mod.render_ruleset(allow, hostile)


def test_secret_scan_lists_names_and_does_not_open_them(tmp_path, monkeypatch):
    profile = tmp_path / "profiles" / "pib_a"
    profile.mkdir(parents=True)
    secret = profile / ".env"
    secret.write_text("GROQ_API_KEY=super-secret-canary\n", encoding="utf-8")
    (profile / "SOUL.md").write_text("hello\n", encoding="utf-8")
    (profile / "config.yaml").write_text("provider: gemini\n", encoding="utf-8")
    (tmp_path / "credentials.json").write_text("{}\n", encoding="utf-8")
    real_open = open

    def guard(path, *args, **kwargs):
        if str(path).endswith(".env") or str(path).endswith("credentials.json"):
            raise AssertionError(f"secret file was opened: {path}")
        return real_open(path, *args, **kwargs)

    monkeypatch.setattr("builtins.open", guard)
    found = mod.secret_relpaths(str(tmp_path))
    assert found == ["credentials.json", "profiles/pib_a/.env"]
    assert "super-secret-canary" not in "\n".join(found)
    assert mod.is_secret_filename("SOUL.md") is False
    assert mod.is_secret_filename("config.yaml") is False


def test_deadline_is_bounded_and_apply_reverts():
    assert mod.PROBE_TIMEOUT_SECONDS <= 3
    assert mod.ROLLBACK_DEADLINE_SECONDS <= 20
    text = APPLY.read_text(encoding="utf-8")
    assert "py constants" in text
    assert 'sleep "$ROLLBACK_DEADLINE"' in text
    assert "self_check=failed" in text
    after_failure = text.split("self_check=failed", 1)[1]
    assert "rollback" in after_failure[:500]
    assert "table inet pib_voice_egress {" not in text
    assert "http://192.168.1.252:8090/ready" in text
    assert "flask-app" in text
    assert "rosbridge-ws" in text


def test_apply_preflight_accepts_the_shipped_allowlist_and_rejects_a_cut():
    ok = subprocess.run(
        ["bash", str(APPLY), "preflight"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert ok.returncode == 0, ok.stderr
    assert "preflight=ok" in ok.stdout

    data = json.loads(ALLOWLIST_PATH.read_text(encoding="utf-8"))
    for entry in data["entries"]:
        if entry["id"] == "pib-network":
            entry["enabled"] = False
    env = os.environ.copy()
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "allowlist.json"
        path.write_text(json.dumps(data), encoding="utf-8")
        env["PIB_VOICE_EGRESS_ALLOWLIST"] = str(path)
        bad = subprocess.run(
            ["bash", str(APPLY), "preflight"],
            check=False,
            capture_output=True,
            text=True,
            env=env,
        )
    assert bad.returncode != 0
    assert "policy_error=" in bad.stderr


def test_shell_scripts_are_syntactically_valid():
    subprocess.run(["bash", "-n", str(APPLY)], check=True)
    subprocess.run(["bash", "-n", str(VERIFY)], check=True)


def test_verify_script_does_not_print_credential_material():
    text = VERIFY.read_text(encoding="utf-8")
    assert "printenv" not in text
    assert "print(digest" not in text
    assert "super-secret" not in text
    assert "groq_env_present=" in text
    assert "policy_self_modify=" in text
    assert "allowed_stt_http=" in text


def test_voice_compose_drops_privilege_and_the_raw_hermes_mount(compose_dict):
    voice = compose_dict["services"]["ros-voice-assistant"]
    assert voice["privileged"] is False
    for cap in ("NET_ADMIN", "NET_RAW", "SYS_ADMIN"):
        assert cap in voice["cap_drop"]
    assert "no-new-privileges:true" in voice["security_opt"]
    volumes = voice["volumes"]
    assert all(
        not (isinstance(item, str) and "/home/pib/.hermes:" in item) for item in volumes
    )
    bind = next(item for item in volumes if isinstance(item, dict))
    assert bind["target"] == "/home/pib/.hermes"
    assert bind["bind"]["propagation"] == "rslave"
    assert bind["bind"]["create_host_path"] is False
    assert "/dev/snd:/dev/snd" in voice["devices"]
    assert any("116:" in rule for rule in voice["device_cgroup_rules"])
    env = "\n".join(voice["environment"])
    assert "PIB_CLOUD_BASE_URL=" in env
    assert "PIB_OLLAMA_BASE_URL=" in env
    assert compose_dict["services"]["ros-audio-io"]["privileged"] is True
