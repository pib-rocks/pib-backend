# PR-1781 — fresh-install hardening (verified facts)

Repo: `pib-rocks/pib-backend`, working branch: **`PR-1781`** (created from `origin/develop` = `bf0d7d28`).
Ticket: Jira **PR-1781** (project pib.rocks), Bug, status *Bereit*. Description + evidence live in the ticket;
this file holds the code-level facts you cannot see from the ticket alone.

## Defect A + B — `setup/setup-pib.sh`, function `disable_power_notification()` (lines 536-559)

```bash
function disable_power_notification() {
	local file="/boot/firmware/config.txt"

	if [ -f "$file" ]; then
    	echo "Disabling under-voltage warnings..."
		  echo "avoid_warnings=2" | sudo tee -a "$file" > /dev/null

    	echo "Preventing CPU throttling..."
    	echo "force_turbo=1" | sudo tee -a "$file" > /dev/null
	fi

	echo "Installing and configuring watchdog service..."
	sudo apt-get install -y watchdog
	sudo systemctl enable watchdog
	sudo systemctl start watchdog

	echo "Modifying watchdog configuration..."
	sudo sed -i 's/#reboot=1/reboot=0/' /etc/watchdog.conf

	echo "Disabling kernel panic reboots..."
	echo "kernel.panic = 0" | sudo tee -a /etc/sysctl.conf

	sudo sysctl -p
}
```

Measured on a pib5edu (Raspberry Pi 5, 4 GB, Raspberry Pi OS trixie, 2026-09-20):

- **B (idempotency):** both `tee -a` calls append unconditionally. After two setup runs
  `/boot/firmware/config.txt` contains
  `55:avoid_warnings=2`, `56:force_turbo=1`, `57:avoid_warnings=2`, `58:force_turbo=1`.
- **A (watchdog):** `systemd` already owns the hardware watchdog on this image —
  `systemctl show -p RuntimeWatchdogUSec` -> `1min`, `/sys/class/watchdog/watchdog0` ->
  `state=active timeout=60`, kernel cmdline contains `reboot=w`, and the journal says
  `Watchdog running with a hardware timeout of 1min`. The function then installs a **second**
  owner (`apt-get install -y watchdog`, `systemctl enable/start watchdog`); its
  `/etc/watchdog.conf` has no checks configured, its only active lines are
  `realtime = yes` / `priority = 1`. During `docker compose up -d --build` of a documented fresh
  install the board performed an **unclean reset** (no shutdown sequence in the journal, setup log
  ends mid build step). `wdctl` showed `Timeout: 60 seconds, Timeleft: 15 seconds`.
- **A (no-ops):** `sed -i 's/#reboot=1/reboot=0/' /etc/watchdog.conf` matches nothing —
  that line does not exist in the shipped config, so the sed is a silent no-op.
  `kernel.panic = 0` disables panic reboots, not a watchdog reset.

### What to implement

1. **One explicit watchdog policy.** Recommended: do **not** add a second owner — systemd already
   pings the hardware watchdog, so drop the `enable`/`start` of the `watchdog` daemon (installing
   the package is optional; if you keep it, leave it disabled and say why). Whatever you choose,
   it must be stated in the commit message and in a comment next to the code.
2. **Make the timeout safe for a long silent build** and document the value. The install runs a
   `docker compose up -d --build` that can take 40+ minutes at high load; a 60 s deadline with a
   missed ping is what reset the board. Prefer relying on systemd's watchdog with a deadline that
   is demonstrably not reached, or disable the runtime watchdog during setup — your call, but it
   must be verifiable.
3. **Remove or fix the two no-op steps.** Either implement them correctly (a real line in the
   watchdog daemon's config, a kernel knob that actually prevents a watchdog reset) or delete them
   with a one-line comment saying why they are gone. Leaving a no-op with a log message that claims
   it worked is the defect.
4. **Make the config.txt writes idempotent**: only append a directive when it is not already
   present. Keep the existing behaviour (both directives still end up there on a fresh image) and
   do not reorder/rewrite the file otherwise.

## Defect C — `tests/run_all_tests.sh`, function `ensure_venv()` (lines 74-79)

```bash
ensure_venv() {
    section "Python virtualenv"
    if [[ ! -d "${VENV_DIR}" ]]; then
        echo "Creating ${VENV_DIR} ..."
        python3 -m venv "${VENV_DIR}"
    fi
    # shellcheck source=/dev/null
    source "${VENV_DIR}/bin/activate"
    ...
```

A `.test-venv` directory that exists but has **no `bin/`** (interrupted creation — e.g. an install
or a first run that got killed) is treated as ready, the repair is skipped, and the script dies
with `.test-venv/bin/activate: No such file or directory`. Observed locally and on a pib5edu.
Fix: detect a *usable* venv (e.g. `"${VENV_DIR}/bin/python"` exists and runs) and rebuild it when
it is not, so the run continues.

## Tests you must add/keep green

- One new test module (e.g. `tests/unit/test_setup_power_and_watchdog.py`) that **executes the real
  script code** with stub executables on `PATH` (no source-text assertions):
  - run the config.txt helper **twice** against a temp file and assert exactly one
    `force_turbo=1` and one `avoid_warnings=2` line;
  - assert the watchdog step no longer issues a second owner (e.g. the stub records that
    `systemctl enable/start watchdog` is not called, or — if you keep the daemon — that the chosen
    policy is applied), and that no `sed`/`tee` step is a silent no-op against a fixture config
    that actually exists in the shipped image.
  - for `tests/run_all_tests.sh`: create a directory without `bin/`, then invoke `ensure_venv`
    (source the script with a stub `python3 -m venv` or run it in a sandbox) and assert it rebuilds
    and continues instead of aborting.
- Keep the setup unit tests green: `python3 -m pytest tests/unit/test_setup_hardware_variant.py
  tests/unit/test_model_provisioning.py tests/unit/test_system_property_service.py
  tests/unit/test_setup_docker_cleaner_service.py -q`.

## Hard constraints

- **Never run the real `setup-pib.sh`**: it writes `/etc/sudoers.d/$USER`, installs packages,
  appends to `/boot/firmware/config.txt` and builds Docker images. Work with stubs/scratch copies.
- Do not touch unrelated functions, do not reformat, do not rename anything.
- English only for code, comments, commit message and test names.
- Real-device verification (fresh install on a pib5edu) is done by the release manager afterwards,
  not by you.
