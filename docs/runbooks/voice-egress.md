# Voice runtime egress (PC-45)

## Boundary and enforcement

The shared voice runtime keeps Direct, Smart, native Live, cloud STT and local
fallback. Host nftables table **`bridge pib_voice_egress`** enforces a default-deny
policy for packets entering through the voice container's bridge port. The host
administrator and other robot services are trusted. This is a shared-container
boundary, not per-function isolation or HTTPS URL-path filtering.

Use **bridge-family input and forward hooks**, not an `inet` hook matched on a
veth name. On the test robot the L3 hook sees the bridge master, so the old veth
matcher counted zero packets and left arbitrary internet access open. Native
bridge hooks see the actual port without enabling `br_netfilter` globally. They
also do not need `nf_conntrack_bridge` (not available in the robot kernel).
Counters on the jumps, permitted TCP destinations and final drop make this
observable. Other ingress ports do not enter the voice-specific chain.

ARP, required internal bridge peers and DDS multicast remain available. IPv6
frames from the voice port are dropped; a loopback-only IPv6 address is not an
external IPv6 route. Inbound traffic from other ports is not filtered by this
policy. There is no generic established-outbound-flow exception: an existing
connection still has to match its destination after application.

Drops surface as a bounded connection timeout; the cloud-STT caller retains its
configured total deadline and visible local fallback. Docker's embedded resolver
at `127.0.0.11` remains usable; DNS success is not evidence that a destination is
allowed. The self-check requires DNS to resolve and **no HTTP response** from the
unlisted destination. Even HTTP 403/404/500 would mean the connection was allowed.

## Checked configuration

`setup/voice_egress/allowlist.json` is the sole destination catalogue. The renderer
validates it and resolves the current Docker/network facts at apply time.

| Destination | Port | Purpose |
|---|---|---|
| `api.pib.rocks` | TCP 443 | pib.Cloud STT and streaming chat (public HTTPS host) |
| `host.docker.internal` | TCP 11434 | Local Ollama; resolve inside the container |
| Robot bridge peers except its gateway | Internal protocols | Flask, rosbridge and ROS DDS |
| `224.0.0.0/4`, IGMP on this bridge | Multicast | ROS DDS discovery |
| `generativelanguage.googleapis.com` | TCP 443 | Enabled Direct, Smart and native Live |
| `huggingface.co` | TCP 443 | Measured TTS asset requests |
| `platform.tryb.ai` | TCP 443, disabled by default | Legacy route; enable only for a required measured mode |

**Do not assume Docker host-gateway equals this network's gateway.** On .172,
`host.docker.internal` resolves to `172.17.0.1`, whereas the robot bridge's gateway
is `172.18.0.1`. The runtime discovery and regression test cover this distinction.

Provider addresses are DNS snapshots, not SNI restrictions. The optional timer
refreshes the snapshot and current port binding every 30 seconds. Address changes
can interrupt an existing connection if its old address is no longer in the set;
continuous-stream behavior across DNS rotation has not been accepted as proven.

Only `huggingface.co` was observed for the cached TTS assets. Large cold-cache
weights may redirect elsewhere: provision assets before enforcement or measure
and explicitly approve the required host. Do not add guessed CDN names or allow
arbitrary internet access as a fallback.

## Privileges and credentials

The voice container is unprivileged, drops NET_ADMIN, NET_RAW, SYS_ADMIN and the
other listed capabilities, and uses no-new-privileges. ALSA device mappings and
the necessary audio group remain. The owning audio-I/O node retains microphone
DSP control; this story does not remove another container's privileges.

The voice container mounts `/var/lib/pib-voice-egress/view`, not the raw Hermes
home. `prepare-view` overlays discovered credential filenames with **read-only**
blank covers. It never prints credential values. A file disappearing between
scan and covering is skipped because it no longer exists; a symlink at a secret
path is rejected, never followed by mount. Do not blindly exclude cache or
snapshot directories from the secret scan: they can contain copied credentials.

This is filename-based coverage of existing files, **not a guarantee against all
future secret creation or secrets embedded in arbitrary files**. Re-run covering
and verification after provisioning/changing profiles. Host administrators must
not add cloud-operator keys to shared runtime data. Necessary user-owned Direct
keys and short-lived Live tokens are distinct from the cloud-STT operator key.
Flask's own mounts/key store remain separate and are not made secret-free by a
voice-container environment scan.

## Deployment and rollback on the authorized test robot (.172 only)

From `/home/pib/app/pib-backend` containing this change:

```bash
sudo bash setup/voice_egress/apply.sh prepare-view
# Recreate only when changing container mounts/capabilities; do not rebuild others.
docker compose --profile voice_assistant up -d --no-deps --force-recreate ros-voice-assistant
sudo bash setup/voice_egress/apply.sh apply
sudo bash setup/voice_egress/verify.sh
```

The preflight checks cloud readiness and Flask/rosbridge before installing.
Installation is an nft transaction preceded by `nft -c`. A failed refresh or
stalled post-check restores the last checked ruleset rather than opening egress.
Only a successful positive/negative self-check atomically publishes the saved
root-owned policy. The watchdog also marks recovery so a late self-check cannot
publish a candidate after the watchdog has already restored the previous policy.
On initial setup only, before any checked policy exists, failure rolls back to
the pre-install state; do not enable Docker's boot dependency until first apply
has succeeded. An explicit operator `rollback` remains an intentional removal.

A fixed, locally administered MAC (`02:50:49:42:00:45`) is the policy identity,
pinned identically in compose and the allowlist and checked by regression tests.
Native bridge input/forward match that identity before any per-port migration
fallback. A new container therefore stays restricted even before the next DNS/
port refresh. The container lacks NET_ADMIN/NET_RAW, so it cannot change that
identity or inject arbitrary Ethernet frames with its retained capabilities.
The fallback on the current veth permits migration from the previous dynamic MAC;
it does not replace the stable-identity rule.

`pib-voice-egress-restore.service` restores the last checked, root-owned ruleset
before Docker starts. The Docker drop-in requires successful restoration, so a
missing or invalid saved policy stops Docker startup rather than starting voice
workloads with open egress. SSH and host administration remain available for
repair. The restore path needs neither running Docker nor DNS; DNS refresh runs
later. This deliberately affects Docker startup on the host and must be installed
only after a successful apply has saved a checked policy. Never install it on .35.

For boot preparation and periodic refresh:

```bash
sudo cp setup/voice_egress/pib-voice-hermes-view.service /etc/systemd/system/
sudo cp setup/voice_egress/pib-voice-egress.{service,timer} /etc/systemd/system/
sudo cp setup/voice_egress/pib-voice-egress-restore.service /etc/systemd/system/
# Apply/save successfully BEFORE installing the startup dependency.
sudo bash setup/voice_egress/apply.sh apply
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo cp setup/voice_egress/docker-voice-egress.conf /etc/systemd/system/docker.service.d/50-voice-egress.conf
sudo systemctl daemon-reload
sudo systemctl start pib-voice-egress-restore.service
sudo systemctl enable pib-voice-hermes-view.service
sudo systemctl enable --now pib-voice-egress.timer
sudo systemctl start pib-voice-egress.service
```

The refresh service must not use RemainAfterExit or ExecStop rollback, otherwise
it either never refreshes or deletes its rules after every refresh. A failed
start before the container is ready is retried by the timer. Units refer to the
checkout above. Enable them only after installing that checkout. Boot behavior
requires its own measured restart evidence; installing a unit is not that proof.

Explicit firewall rollback (container shape remains reduced):

```bash
sudo systemctl stop pib-voice-egress.timer
sudo rm /etc/systemd/system/docker.service.d/50-voice-egress.conf
sudo systemctl daemon-reload
sudo bash setup/voice_egress/apply.sh rollback
```

Container-shape rollback requires restoring the known pre-PC-45 compose service
and recreating only the voice container. Do not use a moving `origin/develop`
reference after this story merges. The filtered view can be removed only once
no running voice container uses it (`apply.sh remove-view`). Never operate .35.

## Evidence and remaining acceptance

The operator proof emits `name=value` lines and a nonzero exit on failures. It
checks a real WAV upload, blocked name/IP, exact voice-node names using fresh ROS
discovery (`--no-daemon`), capabilities, covered secret files, ALSA and cloud
health. A missing host policy cannot pass the self-modification check merely
because it was missing both before and after.

Successful transport probes do **not** prove a complete Direct/Smart/Live turn,
repeated audible playback, or a microphone/AEC interaction. Report those limits
separately and keep PC-45 open while its acceptance criteria remain unverified.
Offline tests cover rendering, actual host-address binding, apply failure invoking
rollback, timer lifecycle, and secret-scan behavior. They never access the robot.
