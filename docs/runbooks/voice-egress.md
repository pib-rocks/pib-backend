# Voice runtime egress

The voice container (`ros-voice-assistant`) gets a default-deny outbound
policy. The policy is enforced on the **host**, in nftables table
`inet pib_voice_egress`. The container is not the enforcement point: it runs
as root, and a privileged root can mount the host disk and rewrite host
firewall rules.

There is no `deploy/` directory in this repository. Host scripts live under
`setup/`, next to the other robot setup scripts. The allowlist is the only
list of destinations. `apply.sh` renders the ruleset from it; the script does
not carry a second copy.

## What the policy does and does not filter

The policy limits **destinations (address and port)**, not URL paths on a
host. `generativelanguage.googleapis.com:443` allows Direct `generateContent`,
the Hermes gemini provider, and the native Live websocket. Those three modes
share one host and port, so they cannot be separated by this filter.

Docker's embedded DNS (`127.0.0.11`) is inside the container network
namespace. It is not a forwarded flow, and this policy does not filter it. A
name that is not on the allowlist can still resolve. The connection to the
resolved address is what is accepted or rejected. Re-run `apply` to refresh
addresses after DNS changes.

IPv6 egress from the voice veth is rejected, so an AAAA answer is not an
alternate route. The voice image prefers IPv4 (`/etc/gai.conf` in the
Dockerfile).

Packets are matched only when they enter on the voice container's veth.
Other containers on `pib-network` are not matched. Intra-bridge unicast is
allowed, except to the bridge gateway (that address is the host). Multicast
is allowed only back out the bridge interface, for ROS 2 DDS discovery.
Ollama (`host.docker.internal:11434`) is allowed on the host input path,
because `host-gateway` is the bridge gateway.

A rejected TCP connection is reset, so the caller sees a transport error
instead of a silent drop. The cloud STT client still bounds the call at
`PIB_CLOUD_STT_TIMEOUT_SECONDS` (default 3).

## Allowlist

Machine-readable source: `setup/voice_egress/allowlist.json`.

| Destination | Port | Protocol | Mode | Why it is here |
|---|---|---|---|---|
| `192.168.1.252` | 8090 | tcp | common | pib.Cloud STT and streaming chat. Measured 2026-10-15: `GET /ready` returned 200. Not re-measured from the implementation session. |
| bridge gateway (`host.docker.internal`) | 11434 | tcp | common | Ollama. Measured 2026-10-15: `GET /` returned 200. |
| pib-network bridge subnet, except the gateway | any | any | common | flask-app:5000, rosbridge-ws:9090, blockly:2442, ROS 2 DDS unicast. Container address measured as `172.18.0.11`. The prefix is read from Docker at apply time. |
| `224.0.0.0/4` out the bridge only | any | any | common | ROS 2 DDS multicast. **Unproven on the wire.** Kept so discovery is not cut. |
| `huggingface.co` | 443 | tcp | common | TTS assets: `audio_player` loads the Supertone/supertonic-3 voice styles at container start. Measured 2026-10-15 in the recreated container's log: 54 requests, and no other external host. Without it speech output breaks. |
| `cdn-lfs.huggingface.co` | 443 | tcp | common | Hugging Face LFS asset host. **Inferred, not measured**: `huggingface.co` answers 307 redirects for `resolve/` paths and the redirect target is not in the log. |
| `generativelanguage.googleapis.com` | 443 | tcp | direct, smart, live | Direct, Hermes gemini, native Live. Host reachability measured 2026-10-15 (HTTPS 404). A live turn was not captured in the implementation session. Addresses are resolved at apply time. |
| `platform.tryb.ai` | 443 | tcp | legacy-tryb | **Disabled.** `TRYB_URL_PREFIX` is set. No connection was observed. Enable the entry only after a measurement shows the legacy route in use. |

`example.com` was reachable (HTTP 200) on 2026-10-15 with no policy. It is not on the allowlist. After apply it must fail.

## Residual trust boundary

The host administrator is trusted. The shared voice runtime is one boundary:
Direct, Smart, and native Live stay in the same container. This is not
per-function isolation.

The cloud-operator key used for cloud speech-to-text stays in the cloud
service. The robot client does not send a provider credential. A user-owned
Direct key is read from the Flask key store (or, if that store is locked,
from `GEMINI_API_KEY` / `GOOGLE_API_KEY` already supplied by `password.env`).
It is not supplied by mounting `~/.hermes/.env`.

`flask-app` still bind-mounts `/home/pib/.hermes/profiles`, so a profile
`.env` on the host remains visible to that API container. That mount is
outside this story. `flask-app` also still has the Docker socket.

Cover mounts hide secret filenames only while the container cannot unmount
them. That requires `privileged: false` and the dropped `SYS_ADMIN`. A
recreate that restores `privileged: true` undoes the credential boundary
even if the covers are in place.

Gemini addresses are a snapshot from apply time. `nftables` does not track
URL paths or SNI.

## Privilege and mounts

`ros-voice-assistant` is no longer `privileged`. Dropped capabilities include
`NET_ADMIN`, `NET_RAW`, and `SYS_ADMIN`, so the container cannot change host
nftables, open raw sockets, or mount the host disk. ALSA stays available
through `/dev/snd` and device cgroup major 116. `/dev/dsp` stays major 14.
The existing USB device mapping stays (major 189). The ReSpeaker control
endpoint `2886:0018` is opened by `ros-audio-io` (`tuning.py`), which remains
privileged. Whether the voice container itself needs that USB node was **not
measured** on a recreated container.

`group_add` uses `PIB_AUDIO_GID` (default 29, the usual `audio` gid on
Raspberry Pi OS). Confirm with `getent group audio` before recreate.

The Hermes home mounted into the voice container is
`/var/lib/pib-voice-egress/view`, produced by `prepare-view`. It is a bind
of `/home/pib/.hermes` with a blank file mounted over each secret filename.
The container's compose mount uses `rslave` so those covers are visible.
`/home/pib/.local/bin/hermes` and `/home/pib/.local/share/uv` are unchanged.

This compose change does nothing until the voice container is recreated.
Recreate changes the veth name, so the nftables policy must be applied
**after** recreate.

Hardware behaviour after dropping `privileged` (ALSA open, USB, Hermes
`--version`, a real voice turn) was **not** run from the implementation
session. `verify.sh` is the check.

## Commands on 192.168.1.172

Run these from the checkout that contains this branch (the install path is
`/home/pib/app/pib-backend` when that is where the branch is deployed).
Do not point them at `192.168.1.35`.

Prepare the filtered Hermes view (host mounts only):

```bash
sudo bash setup/voice_egress/apply.sh prepare-view
```

Recreate only the voice container so the new mounts and capabilities apply:

```bash
sudo docker compose --profile voice_assistant up -d --no-deps --force-recreate ros-voice-assistant
```

Apply the egress policy. This refuses to install if `192.168.1.252:8090` or
the flask/rosbridge peers are already unreachable. After install it checks
those paths again, checks that `https://example.com/` is not open, and
removes the table if the check fails. A watchdog removes the table if the
check does not finish within `ROLLBACK_DEADLINE` (20 seconds).

```bash
sudo bash setup/voice_egress/apply.sh apply
```

Prove it:

```bash
sudo bash setup/voice_egress/verify.sh
```

Roll back the firewall only (containers keep running; egress is open again):

```bash
sudo bash setup/voice_egress/apply.sh rollback
```

Roll back the container shape as well: restore the previous compose service
definition, recreate `ros-voice-assistant`, then remove the view once the
container is no longer using it.

```bash
sudo bash setup/voice_egress/apply.sh rollback
git checkout origin/develop -- docker-compose.yaml
sudo docker compose --profile voice_assistant up -d --no-deps --force-recreate ros-voice-assistant
sudo bash setup/voice_egress/apply.sh remove-view
```

Optional boot units, after the checkout path is the one in the unit files
(`PIB_BACKEND_DIR`, default `/home/pib/app/pib-backend`):

```bash
sudo cp setup/voice_egress/pib-voice-hermes-view.service /etc/systemd/system/
sudo cp setup/voice_egress/pib-voice-egress.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pib-voice-hermes-view.service
sudo systemctl enable --now pib-voice-egress.service
```

The egress unit must start after the voice container exists, because it
looks up that container's veth. Enable it only after the recreated container
is up.

## Files

| Path | Role |
|---|---|
| `setup/voice_egress/allowlist.json` | Destinations, modes, and the evidence for each |
| `setup/voice_egress/voice_egress.py` | Parser, preflight, ruleset renderer |
| `setup/voice_egress/apply.sh` | `preflight`, `prepare-view`, `apply`, `rollback`, `remove-view` |
| `setup/voice_egress/blank.env` | Cover file mounted over Hermes secret filenames |
| `setup/voice_egress/verify.sh` | Operator proof script. Prints `name=value` lines |
| `tests/infrastructure/test_voice_egress_policy.py` | Offline tests for the renderer, preflight, and scripts |
