# PR-1780 — fix the docker_cleaner.service ordering defect (verified facts)

Repo: `pib-rocks/pib-backend`, branch to work on: **`PR-1780`** (already created from real `origin/develop`).
Ticket: Jira **PR-1780** (project pib.rocks), Story, status *Bereit*.

## Symptom

A fresh install finishes, but `docker_cleaner.service` fails on its first start and the setup
script still reports success. Measured on a fresh pib5edu (192.168.1.217, 2026-09-20):

```
# setup-pib.log
[Sun Sep 20 09:35:40 PM UTC 2026][[ Setting up Docker container cleanup service ]]
Job for docker_cleaner.service failed because the control process exited with error code.
[Sun Sep 20 09:35:42 PM UTC 2026][[ Docker container cleanup service installed and started ]]
[Sun Sep 20 09:35:42 PM UTC 2026][[ Installation completed ]]

# journalctl -u docker_cleaner.service
Sep 20 23:35:42 docker[69708]: ERROR: permission denied while trying to connect to the docker API at unix:///var/run/docker.sock
Sep 20 23:35:42 systemd[1]: docker_cleaner.service: Failed with result 'exit-code'.

# /etc/group mtime = 2026-09-20 23:35:42.798  (the usermod ran in the same second, after the start)
```

Impact after the fresh install: `docker system df` showed `Build Cache 160 entries, 14.11GB, 13.82GB reclaimable` — never pruned.

## Root cause (two defects, same code path)

1. **Ordering.** `setup/setup-pib.sh:984` sources `docker_install.sh`, whose tail
   (`docker_install.sh:107`) calls `setup_docker_cleaner_service()`. That function installs and
   **starts** the unit. Only afterwards, at `setup/setup-pib.sh:985`, does
   `sudo usermod -aG docker pib` run. The unit runs as `User=pib` but pib is not yet in the
   `docker` group at spawn time → the docker socket is not accessible → the first start
   always fails on a fresh install.
   Note: `usermod -aG docker pib` cannot simply move above the docker install — the `docker`
   group only exists after `install_docker_engine` ran (`docker_install.sh:104`).
2. **Swallowed failure.** `setup_docker_cleaner_service()` never checks the result:

```bash
function setup_docker_cleaner_service() {          # setup/installation_scripts/docker_install.sh:72
    print INFO "Setting up Docker container cleanup service"
    sudo cp "$BACKEND_DIR/setup/setup_files/docker_cleaner.service" /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable docker_cleaner.service
    sudo systemctl start docker_cleaner.service
    print SUCCESS "Docker container cleanup service installed and started"   # unconditional
}
```

`systemctl start` returns non-zero on failure; the function then still prints SUCCESS and returns 0.

The unit template (`setup/setup_files/docker_cleaner.service`):

```ini
[Service]
Type=oneshot
ExecStart=/usr/bin/docker builder prune -f
User=pib
```

## Fix (recommended shape — implementer may refine, must keep the ACs)

1. Grant the group **after the engine is installed and before the cleaner service is started** —
   i.e. inside `docker_install.sh`, between `install_docker_engine` (line 104) and
   `setup_docker_cleaner_service` (line 107). Then remove the now-redundant
   `usermod` line from `setup-pib.sh:985` (it is only reached in the same branch that sources
   `docker_install.sh`, so behaviour is unchanged).
   Alternative if you can argue it better: run the oneshot as root (drop `User=pib` from the unit)
   — but then the group ordering no longer matters and the ACL story changes. Prefer the
   ordering fix; it keeps least privilege.
2. Make `setup_docker_cleaner_service()` fail loudly: check the `systemctl start` exit status,
   `print ERROR` the systemctl output/reason and `return 1` — never print the SUCCESS line when
   the start failed. The caller at `docker_install.sh:107` already does
   `setup_docker_cleaner_service || print ERROR "failed to setup docker cleaner service"`.
3. Keep the change minimal and idempotent. No unrelated refactors, no reformatting.

## Tests that must exist afterwards

Add **one** test module under `tests/unit/` (e.g. `test_setup_docker_cleaner_service.py`) that
exercises the REAL script code with stubs on `PATH` (no source-text assertions):

- Build a temp dir with a fake `sudo` (and/or `systemctl`) that records its argv to a log file and
  exits with a configurable status; prepend it to `PATH`.
- **Sequencing invariant:** run the relevant installer path (or the extracted function plus the
  group grant) and assert the recorder log shows `usermod -aG docker pib` **before**
  `systemctl start docker_cleaner.service`.
- **Failure invariant:** with the fake `systemctl start` exiting non-zero, assert
  `setup_docker_cleaner_service` returns non-zero and that the SUCCESS line
  `Docker container cleanup service installed and started` was NOT printed.

Existing test styles to copy: `tests/unit/test_setup_hardware_variant.py` (sources a real script
fragment in a bash subprocess) and `tests/unit/test_setup_hermes_model_pin.py` (extracts and
executes installer code). Do not write change-detector or source-shape tests.

## Verified environment facts

- Repo root: `/home/pib/opencode/pib-backend`.
- Run tests with the repo's venv python, e.g.
  `python3 -m pytest tests/unit/test_setup_docker_cleaner_service.py -q`, and also re-run
  `tests/unit/test_setup_hardware_variant.py tests/unit/test_model_provisioning.py tests/unit/test_system_property_service.py -q`.
- `bash -n setup/setup-pib.sh` and `bash -n setup/installation_scripts/docker_install.sh` must pass.
- Do NOT run the real `setup-pib.sh`: it writes `/etc/sudoers.d/$USER`, installs packages and
  builds Docker images.
