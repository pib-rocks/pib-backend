# System update API

The host runner (`setup/update_runner.sh`) still performs fetch, build, backup, migration, and rollback. Flask only writes the file protocol and reads status. There is no second executor, lock, or updater daemon.

The device channel is the `channel` field in `pib-backend.revision.json` and `cerebra.revision.json`. `system_properties` has no update-channel key, so this change does not add a schema migration. `software.version` stays unrelated. A development device is not rewritten as a release device.

## Compatible requests

`POST /system/update` still returns `202` with three fields:

| Field | Meaning |
|---|---|
| `job` | Accepted request. No `state` and no `classification`. |
| `status` | Queue or runner document, including `classification`. |
| `programRunningSignal` | `available` or `unavailable`. Today the hook is unavailable. |

A channel-only body is unchanged and remains a **legacy moving-branch** install:

```json
{"channel": "release", "force": false, "confirmation": "UPDATE"}
```

`channel` is `release` or `develop`. Omitted `channel` still defaults to `release`. `force` still defaults to false. The confirmation text remains exactly `UPDATE`. Client `targets` and `targetKind` are rejected. The server copies commits from the completed check.

Pinned release install, after `POST /system/update/check` has finished:

```json
{
  "channel": "release",
  "release": "v1.2.3",
  "checkId": "<uuid from that check>",
  "force": false,
  "confirmation": "UPDATE"
}
```

`release` must be a stable `vMAJOR.MINOR.PATCH` tag that the confirmed check marked installable for both `pib-rocks/pib-backend` and `pib-rocks/cerebra`. The stored job then contains `targetKind: "published-release"` and both commits. Later movement of `origin/main` does not change that job. The runner checks out those commits. Tag lookup still follows `docs/RELEASE.md`: `git tag --points-at HEAD^2`, then `HEAD`.

Pinned develop install:

```json
{"channel": "develop", "pin": true, "checkId": "<uuid>", "force": false, "confirmation": "UPDATE"}
```

`targetKind` is `develop-pin`. This is not a published system release.

## Availability checks

`POST /system/update/check` with `{"channel": "release"}` or `{"channel": "develop"}` returns `202` and a `checkId`. `GET /system/update/available` is `state: "pending"` until the host writes `available.json` with that same `checkId`. The previous document is under `previous` and is not the new result.

For `channel: "release"`, an unequal branch SHA is not a newer system release. Installability comes from the paired tag list. Drafts, prereleases, non-semver tags, and one-sided publications are reported and are not installable. Model-registry releases are never queried.

`GET /system/update/available` adds `recommendation.relation`: `newer`, `current`, `older`, `channel-change`, `drift`, or `unknown`. Only `newer` is an ordinary update. Unknown installed versions stay unknown.

## Readiness, liveness, and stale jobs

`service.json` records installation. It is not proof that the executor is alive.

`GET /system/update/status` includes `readiness`. `ready` requires the shared directory, the installer marker, a recorded runner path, the check runner advertisement, and the host units `pib-update.path`, `pib-update.service`, `pib-update-check.path`, and `pib-update-check.service`. The API does not call systemctl. A marker that does not list those units is not ready. Repair is the update section of `setup/installation_scripts/docker_install.sh` on the host. Queueing an update cannot install missing units.

PR-1812 is implemented here as one `classification: "stale"` outcome, not a second lock:

- A nonterminal `status.json` with no `request.json` is stale.
- A queued request with no `executor.json` heartbeat that is older than `PIB_UPDATE_START_DEADLINE_SECONDS` (default 180) is stale.
- A heartbeat older than `PIB_UPDATE_HEARTBEAT_DEADLINE_SECONDS` (default 120) is stale.

The stale job stays visible, including as `interruptedJob` after a new request is accepted, and `blocksNewUpdate` is false. An in-progress job from a runner that does not write `executor.json` is left active so a long source build is not marked dead between phases. Cancellation remains the runner's existing phase gate: `cancelSafe` is true for `queued`, `preflight`, and `fetching` only. A failed cancel is not a rollback. User-data rollback is not claimed; the WAL-safe backup is kept and not applied automatically.

## Rollout

1. Deploy this pib-backend tree, including `setup/update_runner.sh` and the installer marker, before the Cerebra page that sends `release` and `checkId`.
2. On a device whose `service.json` has no `units` array, re-run the host installer update section or the new page will not offer Install.
3. Deploy Cerebra after the API. Until then, existing channel-only callers keep the moving-branch behaviour.
4. Do not point the new page at an API that ignores `release`. That API would queue an unpinned job; the page reports that the accepted job has no `targets`.

## Local contract tests

These exercise the Flask client and the runner with stubbed git/docker. They are not a robot install.

```bash
python -m pytest \
  tests/unit/test_update_service.py \
  tests/unit/test_update_check.py \
  tests/unit/test_update_releases.py \
  tests/unit/test_update_status_reader.py \
  tests/unit/test_update_healthcheck.py \
  tests/unit/test_update_version_injection.py \
  tests/unit/test_update_watchdog.py \
  tests/unit/test_update_ollama_listen.py \
  tests/integration/test_update_api.py \
  tests/integration/test_update_operator_contract.py \
  -q --tb=short
```

## Hardware acceptance — NOT EXECUTED

Do not treat the tests above, a merge, or a screenshot as story completion. The production UI install below has not been run.

1. Owner designates one fully installed robot and reconfirms its address. Do not use `.92` or `.217` for a destructive run.
2. Snapshot programs, poses, keys, and unrelated configuration. Confirm `pib-update.path` and `pib-update-check.path` are enabled and `/home/pib/app/.update/service.json` lists the units.
3. Record the installed image version and both repository revisions from System / Update.
4. Through the production Cerebra page only, check for a published stable pair newer than that install, type `UPDATE`, and install. Do not SSH, write `request.json`, or clean the update directory by hand.
5. While the stack restarts, leave the page up. It must show reconnecting rather than a finished failure, and a reload must reattach to the same job id.
6. After success, record the job id, the API replies, the log, the image version, both commits, and `flask db current` against the Alembic head. Confirm programs, poses, and keys are still present and the WAL backup file exists and was not restored over the database.
7. Separately, on that same authorised robot or a fixture that cannot destroy `.92` or `.217`, record unavailable-runner, failed check, dirty checkout refusal, and a stale interrupted job. Label each path live or simulated.

Until that record exists, this story is not accepted.
