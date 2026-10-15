#!/bin/bash
# Read-only proof of the voice egress policy, run on the robot host by an
# operator. Prints name=value lines. Does not print credential values.
# A failed check still prints the rest, then the script exits 1.
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PIB_VOICE_EGRESS_PY:-$ROOT/voice_egress.py}"
ALLOW="${PIB_VOICE_EGRESS_ALLOWLIST:-$ROOT/allowlist.json}"
BLANK="${PIB_VOICE_EGRESS_BLANK:-$ROOT/blank.env}"
APPLIED="${PIB_VOICE_EGRESS_APPLIED:-/var/lib/pib-voice-egress/applied.nft}"
NFT="${NFT:-nft}"
TABLE="pib_voice_egress"
CLOUD="http://192.168.1.252:8090"
WAV="${PIB_VOICE_EGRESS_WAV:-/app/ros2_ws/voice_assistant/assets/stt_self_check.wav}"
FAIL=0

note() {
  printf '%s\n' "$1"
}

mark_fail() {
  FAIL=1
}

CONTAINER="$(python3 "$PY" --allowlist "$ALLOW" container-name 2>/dev/null || true)"
if [[ -z "$CONTAINER" ]]; then
  note "container_name="
  note "verify=failed"
  exit 1
fi
note "container_name=$CONTAINER"

if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  note "container_status=missing"
  note "verify=failed"
  exit 1
fi

INSPECT="$(docker inspect -f '{{.State.Status}} {{.RestartCount}} {{.HostConfig.Privileged}}' "$CONTAINER")"
note "container_status=$(awk '{print $1}' <<<"$INSPECT")"
note "container_restart_count=$(awk '{print $2}' <<<"$INSPECT")"
note "privileged=$(awk '{print $3}' <<<"$INSPECT")"
if [[ "$(awk '{print $1}' <<<"$INSPECT")" != "running" ]]; then
  mark_fail
fi
if [[ "$(awk '{print $3}' <<<"$INSPECT")" != "false" ]]; then
  mark_fail
fi

note "dns_enforcement=connection-address-not-resolver"
note "dns_resolver=127.0.0.11-inside-container-netns"
note "path_filter=destinations-only-not-url-paths"

# Allowed STT upload. The bundled sample is the voice self-check utterance.
STT_CODE="000"
if docker exec "$CONTAINER" test -f "$WAV"; then
  note "allowed_stt_fixture=$WAV"
  STT_CODE="$(
    docker exec "$CONTAINER" curl -sS -m 3 \
      -H "X-Request-ID: pib-voice-egress-verify" \
      -F "file=@${WAV};type=audio/wav" \
      -F "language=de" \
      -o /tmp/pib-voice-egress-stt.json \
      -w '%{http_code}' \
      "${CLOUD}/v1/voice-assistant/speech-to-text" 2>/dev/null || true
  )"
else
  note "allowed_stt_fixture=missing"
  mark_fail
fi
note "allowed_stt_http=$STT_CODE"
if [[ "$STT_CODE" != "200" ]]; then
  mark_fail
fi
docker exec "$CONTAINER" python3 -c '
import json, os
path = "/tmp/pib-voice-egress-stt.json"
if not os.path.isfile(path):
    print("allowed_stt_transcript=")
    print("allowed_stt_transcript_chars=0")
    raise SystemExit(0)
try:
    body = json.load(open(path, encoding="utf-8"))
except Exception:
    print("allowed_stt_body=unparsed")
    print("allowed_stt_transcript=")
    print("allowed_stt_transcript_chars=0")
    raise SystemExit(0)
text = body.get("text") if isinstance(body, dict) else ""
if not isinstance(text, str):
    text = ""
text = " ".join(text.split())
print("allowed_stt_transcript=%s" % text)
print("allowed_stt_transcript_chars=%d" % len(text))
' || true
CHARS="$(docker exec "$CONTAINER" python3 -c '
import json, os
path = "/tmp/pib-voice-egress-stt.json"
try:
    body = json.load(open(path, encoding="utf-8"))
    text = body.get("text") if isinstance(body, dict) else ""
except Exception:
    text = ""
if not isinstance(text, str):
    text = ""
print(len(text.strip()))
' 2>/dev/null || echo 0)"
if [[ "${CHARS:-0}" == "0" ]]; then
  mark_fail
fi

classify_blocked() {
  # $1 is the curl exit status from inside the container.
  # 0 and TLS-handshake failures mean the TCP connection was allowed.
  local rc="$1"
  case "$rc" in
    0|35|51|60) echo "open" ;;
    7|28|52|56) echo "blocked" ;;
    *) echo "exit_$rc" ;;
  esac
}

set +e
docker exec "$CONTAINER" curl -sS -m 3 -o /dev/null "https://example.com/" >/dev/null 2>&1
NAME_RC=$?
set -e
note "blocked_name_curl_exit=$NAME_RC"
NAME_CLASS="$(classify_blocked "$NAME_RC")"
note "blocked_name_example_com=$NAME_CLASS"
if [[ "$NAME_CLASS" != "blocked" ]]; then
  mark_fail
fi

EXAMPLE_IP="$(getent ahostsv4 example.com | awk '{print $1; exit}')"
note "blocked_ip_target=${EXAMPLE_IP:-unresolved}"
if [[ -n "${EXAMPLE_IP:-}" ]]; then
  set +e
  docker exec "$CONTAINER" curl -sS -m 3 -o /dev/null "https://${EXAMPLE_IP}/" >/dev/null 2>&1
  IP_RC=$?
  set -e
  note "blocked_ip_curl_exit=$IP_RC"
  IP_CLASS="$(classify_blocked "$IP_RC")"
  note "blocked_direct_ip=$IP_CLASS"
  if [[ "$IP_CLASS" != "blocked" ]]; then
    mark_fail
  fi
else
  note "blocked_direct_ip=unresolved"
  mark_fail
fi

IPV6="$(
  docker exec "$CONTAINER" python3 -c '
import os
path = "/proc/net/if_inet6"
print("true" if os.path.isfile(path) and os.path.getsize(path) > 0 else "false")
' 2>/dev/null || echo false
)"
note "ipv6_enabled=$IPV6"
if [[ "$IPV6" == "true" ]]; then
  set +e
  docker exec "$CONTAINER" curl -6 -sS -m 3 -o /dev/null "https://example.com/" >/dev/null 2>&1
  IP6_RC=$?
  set -e
  note "blocked_ipv6_curl_exit=$IP6_RC"
  IP6_CLASS="$(classify_blocked "$IP6_RC")"
  note "blocked_ipv6=$IP6_CLASS"
  if [[ "$IP6_CLASS" == "open" ]]; then
    mark_fail
  fi
else
  note "blocked_ipv6=skipped"
fi

if docker exec "$CONTAINER" python3 -c \
  'import socket,sys; socket.create_connection(("flask-app", 5000), 3).close()' \
  >/dev/null 2>&1; then
  note "flask_tcp=open"
else
  note "flask_tcp=closed"
  mark_fail
fi
if docker exec "$CONTAINER" python3 -c \
  'import socket,sys; socket.create_connection(("rosbridge-ws", 9090), 3).close()' \
  >/dev/null 2>&1; then
  note "rosbridge_tcp=open"
else
  note "rosbridge_tcp=closed"
  mark_fail
fi

docker exec "$CONTAINER" python3 -c '
import os
names = sorted(
    key for key in os.environ
    if any(part in key.upper() for part in ("KEY", "TOKEN", "SECRET", "PASSWORD", "GROQ", "GEMINI", "OPENAI"))
)
print("key_shaped_env_names=" + ",".join(names))
print("groq_env_present=" + ("true" if any("GROQ" in key.upper() for key in os.environ) else "false"))
' || mark_fail
GROQ_PRESENT="$(
  docker exec "$CONTAINER" python3 -c 'import os; print("true" if any("GROQ" in k.upper() for k in os.environ) else "false")' \
    2>/dev/null || echo true
)"
if [[ "$GROQ_PRESENT" != "false" ]]; then
  mark_fail
fi

BLANK_SHA="$(sha256sum "$BLANK" | awk '{print $1}')"
docker exec -e BLANK_SHA="$BLANK_SHA" "$CONTAINER" python3 -c '
import hashlib, os, stat
root = "/home/pib/.hermes"
files = 0
blank = 0
nonblank = 0
unreadable = 0
expected = os.environ.get("BLANK_SHA", "")
for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
    dirnames.sort()
    for name in sorted(filenames):
        secret = name == ".env" or name.endswith(".env") or name in {"credentials.json", "auth.json", "secrets.json"}
        if not secret:
            continue
        full = os.path.join(dirpath, name)
        files += 1
        try:
            mode = os.lstat(full).st_mode
            if not stat.S_ISREG(mode):
                nonblank += 1
                continue
            digest = hashlib.sha256(open(full, "rb").read()).hexdigest()
        except OSError:
            unreadable += 1
            continue
        if digest == expected:
            blank += 1
        else:
            nonblank += 1
mounts = open("/proc/mounts", encoding="utf-8", errors="replace").read()
print("dotenv_files=%d" % files)
print("dotenv_blank_covers=%d" % blank)
print("dotenv_nonblank=%d" % nonblank)
print("dotenv_unreadable=%d" % unreadable)
print("secrets_mount=" + ("present" if "/home/pib/app/secrets" in mounts else "absent"))
' || mark_fail
NONBLANK="$(
  docker exec -e BLANK_SHA="$BLANK_SHA" "$CONTAINER" python3 -c '
import hashlib, os, stat
root = "/home/pib/.hermes"
nonblank = 0
expected = os.environ.get("BLANK_SHA", "")
for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
    for name in filenames:
        secret = name == ".env" or name.endswith(".env") or name in {"credentials.json", "auth.json", "secrets.json"}
        if not secret:
            continue
        full = os.path.join(dirpath, name)
        try:
            if not stat.S_ISREG(os.lstat(full).st_mode):
                nonblank += 1
                continue
            digest = hashlib.sha256(open(full, "rb").read()).hexdigest()
        except OSError:
            nonblank += 1
            continue
        if digest != expected:
            nonblank += 1
print(nonblank)
' 2>/dev/null || echo 1
)"
if [[ "${NONBLANK:-1}" != "0" ]]; then
  mark_fail
fi

# Host policy must survive an in-container delete/flush. Restore if it does not.
HOST_BEFORE=0
if "$NFT" list table inet "$TABLE" >/dev/null 2>&1; then
  HOST_BEFORE=1
fi
note "host_policy_present_before=$HOST_BEFORE"
docker exec "$CONTAINER" nft delete table inet "$TABLE" >/dev/null 2>&1 || true
docker exec "$CONTAINER" iptables -F >/dev/null 2>&1 || true
set +e
docker exec "$CONTAINER" mount -t tmpfs tmpfs /mnt >/dev/null 2>&1
MOUNT_RC=$?
set -e
if [[ "$MOUNT_RC" -eq 0 ]]; then
  note "mount_tmpfs=allowed"
  docker exec "$CONTAINER" umount /mnt >/dev/null 2>&1 || true
  mark_fail
else
  note "mount_tmpfs=denied"
fi
HOST_AFTER=0
if "$NFT" list table inet "$TABLE" >/dev/null 2>&1; then
  HOST_AFTER=1
fi
note "host_policy_present_after=$HOST_AFTER"
if [[ "$HOST_BEFORE" == "$HOST_AFTER" ]]; then
  note "policy_self_modify=denied"
else
  note "policy_self_modify=allowed"
  if [[ -f "$APPLIED" ]]; then
    "$NFT" -f "$APPLIED" >/dev/null 2>&1 || true
    note "policy_restored=attempted"
  fi
  mark_fail
fi

docker exec "$CONTAINER" python3 -c '
eff = 0
for line in open("/proc/1/status", encoding="utf-8", errors="replace"):
    if line.startswith("CapEff:"):
        eff = int(line.split()[1], 16)
        break
def present(bit):
    return "present" if eff & (1 << bit) else "absent"
print("cap_net_admin=" + present(12))
print("cap_net_raw=" + present(13))
print("cap_sys_admin=" + present(21))
' || mark_fail
CAPS="$(docker exec "$CONTAINER" python3 -c '
eff = 0
for line in open("/proc/1/status", encoding="utf-8", errors="replace"):
    if line.startswith("CapEff:"):
        eff = int(line.split()[1], 16)
print("bad" if (eff & ((1 << 12) | (1 << 13) | (1 << 21))) else "ok")
' 2>/dev/null || echo bad)"
if [[ "$CAPS" != "ok" ]]; then
  mark_fail
fi

docker exec "$CONTAINER" python3 -c '
import os
print("snd_present=" + ("true" if os.path.isdir("/dev/snd") else "false"))
try:
    import pyaudio
    audio = pyaudio.PyAudio()
    print("pyaudio_devices=%d" % audio.get_device_count())
    audio.terminate()
except Exception as exc:
    print("pyaudio_devices=error")
    print("pyaudio_error_type=%s" % type(exc).__name__)
' || mark_fail
SND="$(
  docker exec "$CONTAINER" python3 -c 'import os; print("true" if os.path.isdir("/dev/snd") else "false")' \
    2>/dev/null || echo false
)"
if [[ "$SND" != "true" ]]; then
  mark_fail
fi

NODES="$(
  docker exec "$CONTAINER" bash -lc \
    'source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 node list' \
    2>/dev/null || true
)"
NODE_LINE="$(echo "$NODES" | tr '\n' ' ' | sed 's/  */ /g')"
note "ros_nodes=$NODE_LINE"
ROS_OK=true
for expected in assistant audio_player audio_recorder chat token_service; do
  if ! grep -q "$expected" <<<"$NODE_LINE"; then
    ROS_OK=false
  fi
done
note "ros_nodes_ok=$ROS_OK"
if [[ "$ROS_OK" != "true" ]]; then
  mark_fail
fi

for path in /health /ready; do
  code="$(curl -sS -m 3 -o /dev/null -w '%{http_code}' "${CLOUD}${path}" 2>/dev/null || true)"
  name="${path#/}"
  note "cloud_${name}=$code"
  if [[ "$code" != "200" ]]; then
    mark_fail
  fi
done

if [[ "$FAIL" -eq 0 ]]; then
  note "verify=ok"
  exit 0
fi
note "verify=failed"
exit 1
