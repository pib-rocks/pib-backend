# Software setup

This script assumes:

- that the newest Raspberry Pi OS is installed
- the user running it is **pib**

## Installing pibs software

All the software pib requires can be installed by running our setup script.
Follow these steps to run it:

1. Open a terminal in Raspberry Pi OS

2. Insert the following command into the terminal to download the script:

        wget https://raw.githubusercontent.com/pib-rocks/pib-backend/main/setup/setup-pib.sh

   (or download it manually: https://github.com/pib-rocks/pib-backend/blob/main/setup/setup-pib.sh)

3. Insert this command to run the script:

        bash setup-pib.sh

   If you want to run the setup-script in legacy mode (for Raspberry Pi 4), insert:
               
         bash setup-pib.sh -l

### Installing a development branch

To install Cerebra and pib-backend from a specific branch (for example `develop`), download
`setup-pib.sh` from that same branch and pass both branches to the script:

        wget https://raw.githubusercontent.com/pib-rocks/pib-backend/develop/setup/setup-pib.sh
        bash setup-pib.sh -f=develop -b=develop

      `-f` selects the Cerebra branch, `-b` the pib-backend branch. Both default to `main`.
      The script fetches the helper scripts it needs from the branch you selected, so
      downloading `setup-pib.sh` on its own is enough.

### Hardware variants

The script selects a hardware variant; without a flag the default `pib5edu` is used.

        --pib4edu        pib 4 educational
        --pib4advanced   pib 4 advanced
        --pib5advanced   pib 5 advanced
        --pib5museum     pib 5 museum

A variant takes effect on a fresh install. To change it later, run
`flask seed_hardware --variant <variant> --force`.

The setup then adds Cerebra and it's dependencies, including ROS2, Tinkerforge,...
Once the installation is complete, please restart the system to apply all the changes.

# Updating the Software

This script assumes that the setup script was executed successfully

1. Open a terminal
2. Enter this command: `update-pib`

This script will update your docker containers (Front- and Backend)

### Backend update service

The backend can request the same host update through these LAN API endpoints:

- `POST /system/update` with JSON
  `{"channel":"release","force":false,"confirmation":"UPDATE"}`
- `GET /system/update/status`
- `GET /system/update/log?offset=0` (the returned `nextOffset` is a byte offset)
- `POST /system/update/cancel`
- `GET /system/revision`

There is **no authentication in this backend**. The API is intended only for a
trusted LAN, and the exact typed confirmation `UPDATE` is the guard against an
accidental request; it is not an authorization mechanism. Do not expose these
routes to the internet.

Flask writes an atomic request to `/app/.update`, which is the bind-mounted host
directory `/home/pib/app/.update`. `pib-update.path` starts the oneshot
`pib-update.service`, and the runner executes on the host as `pib`. It must stay
host-side because rebuilding the backend recreates `flask-app` itself. Status is
stored in `status.json`, the append-only live log in `update.log`, and installed
revision facts in `pib-backend.revision.json` and `cerebra.revision.json`.
Missing revision fields are returned as `unknown`.

Before changing either checkout, the runner refuses dirty repositories unless
`force` was explicitly requested, checks free disk space, and creates and
integrity-checks a WAL-safe SQLite backup using Python's SQLite backup API
inside the existing Flask container. It also refuses if the `watchdog` package
or its unit exists, or if a `/dev/watchdog*` device is held by a process other
than systemd. systemd owning the hardware watchdog is the expected single-owner
state (PR-1781) and is only logged, never a reason to abort. It never installs or
starts a watchdog.

The current software has no authoritative signal that distinguishes a running
user program from an idle `ros-programs` container. The update API therefore
reports `programRunningSignal: unavailable` and does not pretend container
liveness is that signal. A future execution owner must provide the signal
before updates can enforce that refusal.

To disable API-triggered updates while leaving the rest of the backend running:

```bash
sudo systemctl disable --now pib-update.path
```

### Health gate: only regressions roll an update back (decision D12)

After the rebuild the runner requires the API to answer, both checkouts to sit at the
recorded target revision, and **no service that ran before the update to be missing
afterwards**. It takes that snapshot in the preflight phase, before anything is fetched or
built, and compares it once the stacks are up again (`setup/update_healthcheck.py`, unit
tested in `tests/unit/test_update_healthcheck.py`).

Services that were already not running before the update do not block it: they are named in
the log and in the job status as `unhealthyServices`, because the strict rule deadlocked the
robot - the damage blocked the very update that would have fixed it (an invalid Bricklet UID
crash-looped `ros-motors`, and every update rolled back; PR-1796/PR-1797). A service that ran
before and is gone now still fails the update and triggers the rollback. If nothing at all was
running before the update, the strict rule applies for that run.

An end-to-end runner check must be performed on a disposable Pi checkout: queue
a develop request, observe the documented states in order, verify that the
backup passes `PRAGMA integrity_check`, and induce a revision mismatch to
confirm there is only one rollback build. This test is intentionally not run in
the development test suite because it resets both host git checkouts and
rebuilds every container.

## Webots

Starting the webots simulation:

1. Complete all steps of the "Installing pibs software"-section of this readme document
2. Webots GUI is intended to run **natively on the host Wayland session** (not inside Docker).
3. Install Webots on the host OS and then run the simulator launch on the host:
   - `ros2 launch pibsim_webots pib_launch.py`
4. Ensure ROS networking between host and Docker works (same `ROS_DOMAIN_ID`, multicast not blocked).

Webots may throw error messages saying it crashed (especially on VM). This can usually be ignored by clicking on "wait".

## Clustering pibs

To synchronize communication between pibs on default ROS_DOMAIN_ID=0:

1. Open a Terminal:
2. Run the following command:  
   `gedit ~/.bashrc`  
   OR for users connected through terminal:  
   `vim ~/.bashrc`
3. Within .bashrc  
   delete: export ROS_LOCALHOST_ONLY=1  
   or replace it with: ROS_LOCALHOST_ONLY=0
4. Restart pib

To add pib to a distinct logical network:

1. Open a Terminal
2. Run the following command:  
   `gedit ~/.bashrc`  
   OR for users connected through terminal:  
   `vim ~/.bashrc`
3. Delete: "export ROS_LOCALHOST_ONLY=1"
4. Append: "export ROS_DOMAIN_ID=YOUR_DOMAIN_ID"
5. Restart pib

For a range of available ROS_DOMAIN_IDs please check the official documentation at:  
https://docs.ros.org/en/dashing/Concepts/About-Domain-ID.html

### Docker

The backend can be started via `docker compose`. Since the software requires to interface with the OS hardware (USB,
sound and GPIO) Docker for Windows and Mac is not supported.
Running `docker compose up` will start the Flask API, rosbridge and the blockly node server. To run the full backend,
including camera, motors, programs and the voice assistant, profiles can be used:

```bash
docker compose --profile all up
```

`password.env` required to run the voice assistant:

```
TRYB_URL_PREFIX=<BASE_URL_Tryb>
```

### Contributing to pib

For the development process, external developers are requested to refer to the following explanation: https://pib-rocks.atlassian.net/wiki/spaces/kb/pages/435486721/Contributing+to+pib

## Custom backend extensions (PR-1461)

This branch integrates custom features previously maintained in a separate repository:

- Vendored `pib-blockly` sources with button, audio, object detection, and display blocks
- `ros_packages/button_service` for TinkerForge RGB button control
- Facial expressions and display text support in the ROS display service
- Face tracking and vision prompt integration in the camera stack

Cerebra UI updates may be required for new Blockly block categories to appear in the editor.
