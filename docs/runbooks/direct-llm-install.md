# Direct LLM install (`--direct-llm`)

Use this when the robot should run **Direct** chat only: no Hermes CLI on the
host, no `hermes-daemon` container, and no `~/.hermes/profiles/pib_*`
directories created during setup.

## Install

```bash
./setup/setup-pib.sh --direct-llm
```

The installer writes `disabled` to `/etc/pib_smart_chats`, exports
`PIB_SMART_CHATS=0` into the backend compose environment, and skips:

- `install_hermes_cli` (no `/home/pib/.local/bin/hermes`)
- `pib-voice-hermes-view.service`
- the `hermes` compose profile (`hermes-daemon` is not built or started)

Default install (no flag) keeps Hermes: `PIB_SMART_CHATS=1`, the Hermes CLI
step, the view service, and `docker compose --profile all --profile hermes`.

## Re-run without the flag

Running `./setup/setup-pib.sh` again **without** `--direct-llm` restores Hermes
mode: the smart-chats marker becomes `enabled`, `PIB_SMART_CHATS=1`, and the
Hermes install/compose steps run. No separate direct-llm marker is left behind.

## Latency

A full backend `up --build` on a Pi still builds every service in the `all`
profile. Omitting `--profile hermes` skips the `hermes-daemon` image (Python
3.14 agent stack). Expect roughly **one fewer image build and one fewer
long-running container** at install time compared with the default path; the
daemon image is typically the slowest backend build after `flask-app`.
