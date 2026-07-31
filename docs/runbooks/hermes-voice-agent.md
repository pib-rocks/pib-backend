# Hermes Voice Agent Runbook

## Host deployment requirements

These are preconditions discovered on a live robot. Without them, hermes-agent
personalities appear healthy in Cerebra / the API while every turn falls back.

### 1. Hermes CLI on the host (pib user)

The CLI must be installed for the `pib` user at:

```text
/home/pib/.local/bin/hermes
```

`setup/setup-pib.sh` installs it idempotently (`install_hermes_cli`). Manual
equivalent:

```bash
sudo -u pib -H bash -c \
  'curl -fsSL https://hermes-agent.nousresearch.com/install.sh | bash -s -- --skip-setup --skip-browser'
```

Provider credentials are still a one-time step after install:

```bash
sudo -u pib -H hermes setup
# or write keys into /home/pib/.hermes/.env
```

### 2. Shared profiles directory (bind-mounted into both services)

`/home/pib/.hermes/profiles` must exist on the **host** and is bind-mounted into
**both** `flask-app` and `ros-voice-assistant` at the same path. The API writes
`SOUL.md` there; the agent reads it. If the mount is missing, the API reports
success (`soulPath=...`) while the agent sees nothing on disk.

`ros-voice-assistant` additionally mounts all of `/home/pib/.hermes` (sessions,
credentials, the hermes-agent venv) and the CLI wrapper
`/home/pib/.local/bin/hermes`. Those two are not sufficient on their own — see
2a.

### 2a. uv-managed Python directory (easy to miss, breaks everything)

`/home/pib/.local/share/uv` must be bind-mounted into `ros-voice-assistant` at
**the same path**, read-only. Three mounts are required together, and the CLI is
non-functional if any one of them is absent:

| Host path | Why |
|---|---|
| `/home/pib/.hermes` | profiles, sessions, credentials, the hermes-agent venv |
| `/home/pib/.local/bin/hermes` | the CLI wrapper (entry point) |
| `/home/pib/.local/share/uv` | the interpreter and stdlib the venv symlinks to |

The reason for the third one: `/home/pib/.local/bin/hermes` is a tiny wrapper
script that execs the interpreter inside the venv, and that interpreter is a
symlink out of `~/.hermes` into uv-managed Python:

```text
/home/pib/.hermes/hermes-agent/venv/bin/python
  -> /home/pib/.local/share/uv/python/cpython-3.11-linux-<arch>-gnu/bin/python3.11
```

Mounting only `~/.hermes` and the wrapper leaves that symlink dangling inside the
container, so every hermes call dies immediately:

```text
/home/pib/.local/bin/hermes: line 4:
  /home/pib/.hermes/hermes-agent/venv/bin/python: No such file or directory
```

The exit status is `127`, and the only user-visible symptom is that every
hermes-agent personality answers with the fallback sentence. The container's own
`python3.12` cannot be substituted, because the venv is built against 3.11.

Mount the whole `uv` directory rather than one versioned `cpython-3.11.x` path: a
`hermes update` can change the patch version, which would silently break a
pinned mount.

### 3. Environment variables

Set in `docker-compose.yaml` and forwarded into the chat node via
`ros_packages/voice_assistant/launch/launch.py`:

| Variable | Typical value | Purpose |
|---|---|---|
| `PIB_HERMES_PROFILES_DIR` | `/home/pib/.hermes/profiles` | Shared profiles root (flask-app + ros-voice-assistant) |
| `PIB_HERMES_BIN` | `/home/pib/.local/bin/hermes` | CLI wrapper path (ros-voice-assistant / ChatNode) |
| `HERMES_HOME` | `/home/pib/.hermes` | Sessions, credentials, hermes-agent install (ros-voice-assistant) |
| `PIB_HERMES_TIMEOUT` | `120` (default) | Per-turn subprocess timeout in seconds |

Defaults in code match those values; prefer setting them explicitly in compose.

## Verify the wiring end to end

1. **Run the CLI inside the container — the definitive check:**

   ```bash
   docker exec <ros-voice-assistant-container> /home/pib/.local/bin/hermes --version
   ```

   Use the absolute path: `/home/pib/.local/bin` is not on the container's `PATH`,
   so a bare `docker exec <container> hermes --version` fails with "executable
   file not found" even on a healthy robot. The absolute path is also exactly what
   `PIB_HERMES_BIN` points at, so this runs the same binary the chat node runs.

   A zero exit status with a version banner is the only trustworthy evidence that
   the agent path works. Do **not** rely on file-existence checks such as
   `ls -l /home/pib/.local/bin/hermes`: the wrapper file was present, executable
   and correctly mounted on a robot where every turn failed with exit `127`,
   because the interpreter it execs was not mounted (see 2a). That check reported
   a false green while the agent was completely broken.

   If it exits non-zero, read the stderr it prints — that message names the exact
   missing path. Exit `127` naming a path under `venv/bin` means the uv mount from
   2a is missing.

2. **ChatNode preflight** — at startup the chat node now *executes*
   `<PIB_HERMES_BIN> --version` with a short timeout, and logs either:
   - `hermes agent binary available at /home/pib/.local/bin/hermes (Hermes Agent
     v...)`, or
   - `hermes agent preflight failed for '<path>': ...` including the captured
     stderr, plus a note that hermes-agent personalities will fall back.

   A failed probe never blocks or crashes startup; legacy personalities keep
   working.
3. **SOUL.md on the host** — edit a personality description in Cerebra (or PUT
   the personality). Confirm the file appears on the **host**, not only inside
   the flask container:
   ```bash
   ls -l /home/pib/.hermes/profiles/pib_<personality_id>/SOUL.md
   ```

## Switch a personality to Hermes

In Cerebra, open **Voice Assistant**, edit the personality, and select
**Hermes Agent (selbstlernend)** as its assistant model. Save the personality;
no service redeploy is required.

For API-based operation, first find the IDs:

```bash
curl -s http://localhost/api/v1/assistant-model
curl -s http://localhost/api/v1/voice-assistant/personality
```

Then update only the selected personality's model:

```bash
curl -X PUT http://localhost/api/v1/voice-assistant/personality/<personality_id> \
  -H 'Content-Type: application/json' \
  -d '{"assistantModelId": <hermes_model_id>}'
```

The first Hermes turn ensures that profile `pib_<personality_id>` exists,
materializes the personality description as its `SOUL.md`, and uses session
`pib_chat_<chat_id>`.

## Roll back without a redeploy

In the same personality editor, select any legacy model (for example the model
that was selected before Hermes) and save. The next turn uses the existing
public-api backend immediately. Existing Hermes sessions and profiles remain on
disk so that switching models does not destroy memory.

The equivalent API operation is:

```bash
curl -X PUT http://localhost/api/v1/voice-assistant/personality/<personality_id> \
  -H 'Content-Type: application/json' \
  -d '{"assistantModelId": <legacy_model_id>}'
```

## Storage layout

Hermes data belongs to the robot user under `HERMES_HOME`, which defaults to
`/home/pib/.hermes`:

- Sessions: `/home/pib/.hermes/sessions/`, named `pib_chat_<chat_id>`.
- Personality profiles:
  `/home/pib/.hermes/profiles/pib_<personality_id>/`.
- Personality SOUL:
  `/home/pib/.hermes/profiles/pib_<personality_id>/SOUL.md`.
- Interim skill template:
  `ros_packages/voice_assistant/skills/pib-robot-control/SKILL.md`; a provisioned
  copy belongs under the profile's `skills/pib-robot-control/`.

The database `personality.description` is authoritative for SOUL text. Normal
personality updates and the bounded append endpoint re-materialize `SOUL.md`.
Do not edit session backing files directly.

## Inspect and prune

Run Hermes as the same OS user and with the same `HERMES_HOME` as the
voice-assistant service:

```bash
sudo -u pib -H hermes sessions list
sudo -u pib -H hermes profile list
sudo -u pib -H hermes sessions prune
```

`sessions prune` removes stale sessions; inspect the list before pruning.
Deleting a pib chat also requests best-effort deletion of its corresponding
Hermes session. To inspect one personality's materialized SOUL:

```bash
sudo -u pib -H less /home/pib/.hermes/profiles/pib_<personality_id>/SOUL.md
```

Compare that file with the personality returned by:

```bash
curl -s http://localhost/api/v1/voice-assistant/personality/<personality_id>
```

## Fresh-profile credential pitfall

A newly created Hermes profile has isolated configuration and credentials. A
plain `hermes profile create` can therefore fail on its first turn with
`No LLM provider configured`. Always clone the active profile when provisioning:

```bash
sudo -u pib -H hermes profile create pib_<personality_id> \
  --clone --no-alias --description "pib personality <personality_id>"
```

The backend's automatic profile provisioning uses `--clone`. Verify the cloned
profile appears in `hermes profile list` before diagnosing model or network
errors. Never copy credentials into logs or support tickets.
