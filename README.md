# Installation

`setup-pib.sh` is a single script that installs the complete pib software stack.
It detects the operating system automatically and chooses the appropriate installation method:

| OS | Method | Intended for |
|----|--------|--------------|
| Raspberry Pi OS (bookworm / trixie) | Docker | **Users** (production) |
| Ubuntu 24.04 Noble | Native (ROS2 Jazzy, systemd) | **Developers** |

> **The script must be run as the user `pib`.**

---

## Method 1 — Docker on Raspberry Pi OS (recommended for users)

This is the default production method. Cerebra and pib-backend run inside Docker containers managed by `docker compose`. No ROS installation on the host is required.

**Requirements:** Raspberry Pi OS Bookworm or Trixie, user `pib`.

```bash
# 1. Download the setup script
wget https://raw.githubusercontent.com/pib-rocks/pib-backend/main/setup/setup-pib.sh

# 2. Run it
bash setup-pib.sh
```

The script will:
- Install Docker and pull the pib containers
- Configure all required systemd services
- Add user `pib` to the `docker` group
- Install [OpenClaw](https://openclaw.ai) AI assistant and start its gateway service

Once complete, **reboot** to apply all changes.

---

## Method 2 — Native install on Ubuntu 24.04 (for developers)

On Ubuntu 24.04 Noble the script installs everything directly on the host:
ROS2 Jazzy, Flask API, Blockly server, Tinkerforge tools, Cerebra frontend (Angular), and all ROS nodes — each managed by a dedicated systemd service.

Python packages are installed into a dedicated virtual environment at `~/pib-venv` (PEP 668 compliance).

**Requirements:** Ubuntu 24.04 Noble, user `pib`.

```bash
# 1. Download the setup script
wget https://raw.githubusercontent.com/pib-rocks/pib-backend/main/setup/setup-pib.sh

# 2. Run it
bash setup-pib.sh
```

The script will:
- Install ROS2 Jazzy, rosbridge, and colcon
- Create `~/pib-venv` and install all Python dependencies into it
- Install and configure Flask API, Blockly server, Tinkerforge, phpLiteAdmin, nginx
- Build and deploy the Cerebra Angular frontend
- Register and enable all systemd boot services
- Install [OpenClaw](https://openclaw.ai) AI assistant and start its gateway service

Once complete, **reboot** to apply all changes.

---

## Script flags

All flags are optional. When no flags are given, both repositories are cloned from the branch that matches the current git checkout of the script (falling back to `main`).

| Flag | Long form | Description |
|------|-----------|-------------|
| `-b=BRANCH` | `--backend-branch=BRANCH` | Branch of [pib-backend](https://github.com/pib-rocks/pib-backend) to clone and install. |
| `-f=BRANCH` | `--frontend-branch=BRANCH` | Branch of [cerebra](https://github.com/pib-rocks/cerebra) to clone and install. |
| `-h` | `--help` | Print usage information and exit. |

**Examples:**

```bash
# Install the main branch of both repos (default)
bash setup-pib.sh

# Install a specific backend PR branch, keep frontend on main
bash setup-pib.sh -b=PR-1098

# Install specific branches for both repos
bash setup-pib.sh -b=PR-1098 -f=PR-566

# Verbose form
bash setup-pib.sh --backend-branch=PR-1098 --frontend-branch=PR-566
```

A detailed log of the installation is written to `setup-pib.log` in the same directory as the script.

---

## OpenClaw AI Assistant

`setup-pib.sh` automatically installs [OpenClaw](https://openclaw.ai) — a personal AI assistant — on both Raspberry Pi OS and Ubuntu 24.04.

**What gets installed:**

- Node 24 (via NodeSource, system-wide — does not affect the pib Node used by Cerebra)
- `openclaw` npm package (global, in `~/.npm-global`)
- OpenClaw gateway configured on port `18789` (loopback only)
- `openclaw-gateway.service` systemd user unit, enabled to start on boot

**After installation, configure your AI provider:**

```bash
# Interactive setup wizard (adds API keys, channels, etc.)
openclaw onboard
```

**Useful commands:**

```bash
openclaw status          # check gateway health
openclaw daemon restart  # restart the gateway service
openclaw agent --message "Hello pib!"  # send a message to the assistant
```

The gateway runs as a user service. If it is not running, start it with:

```bash
systemctl --user start openclaw-gateway.service
```

---

# Updating the Software

- **Docker (Raspberry Pi OS):** Open a terminal and run `update-pib` to pull and restart the latest containers.
- **Native (Ubuntu 24.04):** Pull the latest code in `~/app/pib-backend` and `~/app/cerebra`, then restart the relevant systemd services (e.g. `sudo systemctl restart pib_api_boot.service`).

## Webots

Starting the webots simulation:

1. Complete all steps of the "Installing pibs software"-section of this readme document
2. If you are running simulation in a virtual machine ubuntu type the following command
   `xhost +local:root`
4. Navigate to app/pib-backend
   `cd app/pib-backend`
5. Enter the following command into a terminal:  
   `sudo docker compose --profile pibsim_webots up`  
   (The first time this command is entered, webots will be installed. Webots should open automatically afterwards, to close it you should stop the container by closing the terminal window which is open or by pressing ctrl + c. To run it again just restart the container and if you turned off the virtual machine repeat step 2)

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
