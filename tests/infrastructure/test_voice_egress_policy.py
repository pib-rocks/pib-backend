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
    # Every DNS-resolved allowlist entry needs a fake address here, because the
    # renderer refuses to render a host it cannot resolve.
    addresses = dns or {
        "generativelanguage.googleapis.com": ("203.0.113.10", "203.0.113.11"),
        "huggingface.co": ("203.0.113.20", "203.0.113.21"),
        "api.pib.rocks": ("203.0.113.30",),
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
    assert by_id["pib-cloud"].destination == "api.pib.rocks"
    assert by_id["pib-cloud"].port == 443
    assert by_id["pib-cloud"].resolve == "dns"
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
    with pytest.raises(mod.PolicyError, match="api.pib.rocks"):
        mod.allowlist_from_dict(raw)


def test_rendered_tcp_allows_match_the_enabled_allowlist_exactly():
    allow = mod.load_allowlist(str(ALLOWLIST_PATH))
    text = mod.render_ruleset(allow, _binding())
    assert _tcp_tokens(text) == {
        ("203.0.113.30", "443"),
        ("203.0.113.10", "443"),
        ("203.0.113.11", "443"),
        ("203.0.113.20", "443"),
        ("203.0.113.21", "443"),
        ("172.18.0.1", "11434"),
        # The Hermes turn daemon on the host's 3.14 runtime (PC-89).
        ("172.18.0.1", "8088"),
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
                "huggingface.co": ("203.0.113.20",),
                "api.pib.rocks": ("203.0.113.30",),
                "platform.tryb.ai": ("198.51.100.20",),
            }
        ),
    )
    assert ("198.51.100.20", "443") in _tcp_tokens(text)
    assert ("203.0.113.10", "443") in _tcp_tokens(text)


def test_bridge_hooks_enforce_without_br_netfilter():
    """L3 iifname is the bridge, not its port; enforce at L2 on the voice port."""
    text = mod.render_ruleset(mod.load_allowlist(str(ALLOWLIST_PATH)), _binding())
    assert "table bridge pib_voice_egress {" in text
    assert "table inet" not in text
    for hook in ("forward", "input"):
        base = _chain(text, hook)
        assert f"hook {hook}" in base
        assert (
            f'iifname "vethvoice1" ether saddr != {mod.load_allowlist(str(ALLOWLIST_PATH)).mac_address} counter jump voice_egress'
            in base
        )
        assert "drop" not in base
    voice = _chain(text, "voice_egress")
    assert "ether type arp accept" in voice
    assert "ether type ip6 counter drop" in voice
    # No grandfathering of outbound connections, and no dependency on the
    # nf_conntrack_bridge module (not shipped by this robot kernel).
    assert "ct state" not in voice
    assert "ct direction" not in voice
    assert "224.0.0.0/4" in voice
    assert "counter drop" in voice


def test_ollama_uses_actual_host_gateway_not_assumed_network_gateway():
    import dataclasses

    binding = dataclasses.replace(_binding(), host_address="172.17.0.1")
    text = mod.render_ruleset(mod.load_allowlist(str(ALLOWLIST_PATH)), binding)
    assert ("172.17.0.1", "11434") in _tcp_tokens(text)
    assert ("172.18.0.1", "11434") not in _tcp_tokens(text)


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
    """The cloud is a public name now, so the guard runs on its resolved address."""
    allow = mod.load_allowlist(str(ALLOWLIST_PATH))
    wide = mod.RuntimeBinding(
        veth="vethvoice1",
        bridge="br-pibtest01",
        bridge_subnet="192.168.1.0/24",
        gateway="192.168.1.1",
        dns_addresses={
            "generativelanguage.googleapis.com": ("203.0.113.10",),
            "huggingface.co": ("203.0.113.20",),
            "api.pib.rocks": ("192.168.1.50",),
        },
    )
    with pytest.raises(mod.PolicyError, match="inside the bridge subnet"):
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
    assert "recover_policy" in after_failure[:500]
    assert "table inet pib_voice_egress {" not in text
    assert "https://api.pib.rocks/ready" in text
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


@pytest.mark.parametrize("has_checked_policy", [False, True])
def test_apply_failure_executes_rollback(tmp_path, has_checked_policy):
    """Recover a checked policy on refresh; initial setup can roll back explicitly."""
    checked = tmp_path / "checked.nft"
    if has_checked_policy:
        checked.write_text("previous checked rules")
    library = tmp_path / "library.sh"
    library.write_text(APPLY.read_text().split('command="${1:-}"', 1)[0])
    result = subprocess.run(
        [
            "bash",
            "-c",
            """
source "$1"
need_root() { :; }
load_constants() { :; }
py() { :; }
discover() { :; }
render_rules() { printf 'test rules\n'; }
baseline() { :; }
install_rules() { :; }
start_watchdog() { :; }
stop_watchdog() { :; }
self_check() { return 1; }
rollback() { echo ROLLBACK_EXECUTED; }
restore_policy() { echo RESTORE_CHECKED; }
apply_policy
""",
            "test",
            str(library),
        ],
        env={
            **os.environ,
            "PIB_VOICE_EGRESS_STATE": str(tmp_path / "state"),
            "PIB_VOICE_EGRESS_LIB": str(tmp_path / "lib"),
            "PIB_VOICE_EGRESS_APPLIED": str(checked),
        },
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "self_check=failed" in result.stderr
    if has_checked_policy:
        assert "RESTORE_CHECKED" in result.stdout
        assert "ROLLBACK_EXECUTED" not in result.stdout
        assert checked.read_text() == "previous checked rules"
    else:
        assert "ROLLBACK_EXECUTED" in result.stdout


def test_refresh_unit_does_not_undo_rules_when_it_finishes():
    service = (EGRESS / "pib-voice-egress.service").read_text()
    timer = (EGRESS / "pib-voice-egress.timer").read_text()
    assert "RemainAfterExit=yes" not in service
    assert "\nExecStop=" not in service
    assert "OnUnitInactiveSec=900s" in timer
    assert "Unit=pib-voice-egress.service" in timer


def test_verification_checks_actual_ros_nodes_and_requires_a_policy():
    script = VERIFY.read_text()
    assert "--no-daemon --spin-time 5" in script
    assert (
        "for expected in voice_assistant audio_player audio_recorder chat token_service"
        in script
    )
    assert '"$HOST_BEFORE" == "1" && "$HOST_AFTER" == "1"' in script
    assert 'table bridge "$TABLE"' in script
    assert "getsize(path)" not in script


def test_missing_secret_cache_does_not_hide_other_secret_files(tmp_path):
    cache = tmp_path / "profiles" / "pib_a" / "cache" / "scratch"
    cache.mkdir(parents=True)
    (cache / ".env").touch()
    (tmp_path / ".env").touch()
    assert mod.secret_relpaths(str(tmp_path)) == [
        ".env",
        "profiles/pib_a/cache/scratch/.env",
    ]
    script = APPLY.read_text()
    assert "secret_cover=skipped-disappeared" in script
    assert "symlink at secret path; refusing to follow" in script
    assert 'mount -o remount,bind,ro "$target"' in script


def test_policy_identity_survives_port_recreation(compose_dict):
    import dataclasses

    allow = mod.load_allowlist(str(ALLOWLIST_PATH))
    mac = allow.mac_address
    voice = compose_dict["services"]["ros-voice-assistant"]
    assert voice["mac_address"] == mac
    first = mod.render_ruleset(allow, _binding())
    second = mod.render_ruleset(
        allow, dataclasses.replace(_binding(), veth="vethnew42")
    )
    for rules in (first, second):
        for hook in ("input", "forward"):
            assert f"ether saddr {mac} counter jump voice_egress" in _chain(rules, hook)


def test_restore_failure_blocks_startup_without_calling_docker(tmp_path):
    library = tmp_path / "library.sh"
    library.write_text(APPLY.read_text().split('command="${1:-}"', 1)[0])
    result = subprocess.run(
        [
            "bash",
            "-c",
            'source "$1"; need_root() { :; }; restore_policy',
            "test",
            str(library),
        ],
        env={**os.environ, "PIB_VOICE_EGRESS_APPLIED": str(tmp_path / "missing.nft")},
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "refusing Docker startup" in result.stderr


def test_docker_requires_policy_restore_before_it_can_start():
    restore = (EGRESS / "pib-voice-egress-restore.service").read_text()
    dependency = (EGRESS / "docker-voice-egress.conf").read_text()
    assert "Requires=pib-voice-egress-restore.service" in dependency
    assert "After=pib-voice-egress-restore.service" in dependency
    assert "Before=docker.service" in restore
    assert " restore'" in restore
    script = APPLY.read_text()
    assert "restore_policy()" in script
    assert "policy_error=no saved policy; refusing Docker startup" in script


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
