# Hermes Voice Agent Runbook

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
