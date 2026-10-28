# Update runner lock (`runner.lock`)

The host update runner and manual deployments that change the same git checkouts must not run in parallel. Both use an exclusive non-blocking `flock` on:

`$PIB_UPDATE_DIR/runner.lock` (default `/home/pib/app/.update/runner.lock`).

## Automatic updates (`setup/update_runner.sh`)

At the start of each job (after retry policy, before preflight), the runner acquires the lock and holds it until the process exits (success, failure, or signal).

If another update or manual deployment already holds the lock, the runner writes `state: failed` and exits without changing containers. The status message is:

```text
Another update or deployment holds /home/pib/app/.update/runner.lock; wait for it to finish or inspect the running job in /home/pib/app/.update/status.json and /home/pib/app/.update/update.log
```

(Paths follow `PIB_UPDATE_DIR` when set.)

Before recreating containers, the runner verifies each checkout is still at the recorded target revision. If a parallel process moved the tree, the job fails immediately with a message like:

```text
pib-backend checkout moved before container build: at <observed-sha>, expected <target-sha>; another process changed the working tree while this update was running
```

The same pattern applies to cerebra. Post-build verification in `verify_result()` remains as a final guard.

## Manual deployments (`setup/dev_tools/with-update-lock.sh`)

Wrap commands that fetch, check out, or rebuild against the device checkouts:

```bash
/path/to/pib-backend/setup/dev_tools/with-update-lock.sh -- docker compose -f /home/pib/app/pib-backend/docker-compose.yaml build
```

If the update runner holds the lock, the helper prints to stderr and exits `1`:

```text
The update runner holds /home/pib/app/.update/runner.lock; wait for the job to finish or inspect /home/pib/app/.update/status.json and /home/pib/app/.update/update.log before running manual deployments
```

When the lock is free, the helper holds it for the duration of the wrapped command so an update job cannot start mid-deployment.

## Requirements

`flock` from util-linux must be on `PATH`. If it is missing, both scripts fail with:

```text
The flock command is required for update locking but was not found on PATH; install util-linux
```
