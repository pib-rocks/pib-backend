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

Provider credentials are still a one-time step after install, and they must be in
place **before** a hermes-agent personality is used: each personality profile
inherits its credentials by copying them from this base install (see "How a
personality profile is provisioned").

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
materializes the personality description as its `SOUL.md`, copies the base
install's `config.yaml` and `.env` into it, and uses session
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
- Personality provider config, copied from the base install:
  `.../pib_<personality_id>/config.yaml` and `.../pib_<personality_id>/.env`
  (mode `0600`). All of it owned by the `pib` user.
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

## How a personality profile is provisioned

A Hermes profile has its own configuration and credentials, and `hermes -p
<profile>` resolves the LLM provider from the profile — not from the base
install. A profile that holds only a `SOUL.md` therefore fails every turn with
`agent failed: No LLM provider configured`, while the default profile keeps
working, which is exactly what a `docker exec <container> hermes -z "..."` check
shows.

The backend provisions a profile with **filesystem operations only**, because the
hermes CLI is not mounted into every container that provisions one (the flask
service does not have it). On the first turn of a hermes-agent personality it:

1. creates `<profiles_dir>/pib_<personality_id>/`,
2. writes `SOUL.md` from the personality description,
3. **copies `config.yaml` and `.env` from the base `HERMES_HOME`** into the
   profile when the profile does not have them yet,
4. aligns ownership of the profile with the owner of the profiles directory.

Consequences to plan for:

- **The base install must have working credentials _before_ any hermes-agent
  personality is used.** There is nothing to copy otherwise, and the profile is
  provisioned without a provider. Run `sudo -u pib -H hermes setup` (or write the
  keys into `/home/pib/.hermes/.env`) first, then verify:

  ```bash
  sudo -u pib -H ls -l /home/pib/.hermes/.env /home/pib/.hermes/config.yaml
  ```

  When either file is absent, the log carries a `WARNING` naming the missing file
  and stating that hermes-agent personalities fall back until it is configured.
- **Copies, not symlinks.** A later `hermes profile delete` cannot damage the base
  install, and a per-personality key can be set without affecting other
  personalities.
- **An existing profile `.env`/`config.yaml` is never overwritten**, so a manual
  customization survives every personality update. To re-inherit the base
  credentials, delete the file from the profile and trigger one turn.
- The copied `.env` keeps mode `0600`. Never copy credentials into logs or
  support tickets.
- The CLI is used only as an optional extra: when `PIB_HERMES_BIN` exists,
  `hermes profile create --clone --no-alias` runs first and the log says so;
  when it does not, the log says the profile was provisioned from the filesystem
  alone. Both paths yield a usable profile.

### Profile ownership (must be the pib user)

Both writers (flask-app and ros-voice-assistant) run as **root** inside their
containers, so everything they create in the bind-mounted profiles directory is
root-owned. The profile must instead belong to the **`pib` user** that owns
`/home/pib/.hermes/profiles` on the host; otherwise the host user is locked out
of its own directory:

```text
drwx------ 13 root root  /home/pib/.hermes/profiles/pib_<personality_id>
$ ls /home/pib/.hermes/profiles/pib_<personality_id>
ls: cannot open directory ...: Permission denied
```

Provisioning therefore `chown`s the profile and its contents to the owner of the
parent profiles directory (read with `stat`, never hardcoded to uid 1000) and
keeps the directory at mode `0700`. The `chown` is best effort: when the caller
is not root it is only logged at debug level and never fails the request. Check
it on the host with:

```bash
ls -ld /home/pib/.hermes/profiles/pib_<personality_id>
```

Expect `pib pib`. If it shows `root root`, the profile predates this fix — one
personality update or one hermes turn repairs it, or fix it by hand:

```bash
sudo chown -R pib:pib /home/pib/.hermes/profiles/pib_<personality_id>
```
