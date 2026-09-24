#!/bin/bash

SETUP_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# `resolve_hardware_variant` lives in installation_scripts/ next to this script inside the
# repository, but the documented install (README) downloads setup-pib.sh on its own, so the
# helper is not guaranteed to be next to it. Load it lazily - once the branch is known - and
# fall back to fetching it from the branch we are installing.
load_hardware_variant_resolver() {
  local candidates=(
    "$SETUP_SCRIPT_DIR/installation_scripts/resolve_hardware_variant.sh"
    "$SETUP_INSTALLATION_DIR/resolve_hardware_variant.sh"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    if [ -f "$candidate" ]; then
      # shellcheck source=/dev/null
      source "$candidate"
      return 0
    fi
  done

  local url="https://raw.githubusercontent.com/pib-rocks/pib-backend/${BRANCH_BACKEND}/setup/installation_scripts/resolve_hardware_variant.sh"
  local download_dir
  download_dir="$(mktemp -d)"
  local target="$download_dir/resolve_hardware_variant.sh"

  if command -v curl >/dev/null 2>&1; then
    curl -fsSL "$url" -o "$target" 2>/dev/null
  elif command -v wget >/dev/null 2>&1; then
    wget -q "$url" -O "$target" 2>/dev/null
  fi

  if [ -s "$target" ]; then
    # shellcheck source=/dev/null
    source "$target"
    rm -rf "$download_dir"
    return 0
  fi
  rm -rf "$download_dir"

  print ERROR "resolve_hardware_variant.sh is missing in: ${candidates[*]}"
  print ERROR "and could not be downloaded from $url"
  print INFO "Download the repository and run setup/setup-pib.sh from there: git clone --depth 1 --branch ${BRANCH_BACKEND} ${BACKEND}"
  return 1
}

# Color definitions for logging
export ERROR="\e[31m"
export WARN="\e[33m"
export SUCCESS="\e[32m"
export INFO="\e[36m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

# Github repositories
export FRONTEND="https://github.com/pib-rocks/cerebra.git"
export BACKEND="https://github.com/pib-rocks/pib-backend.git"
export APP_DIR="$HOME/app"
export BACKEND_DIR="$APP_DIR/pib-backend"
export FRONTEND_DIR="$APP_DIR/cerebra"
export SETUP_INSTALLATION_DIR="$BACKEND_DIR/setup/installation_scripts"

# Function to support printing consistent log messages
function print() {
    local color=$1
    local text=$2

    # If only one argument is provided, assume it is the text
    if [ -z "$text" ]; then
        text=$color
        color="RESET_TEXT_COLOR"
    fi

    # Check if the provided color exists
    if [ -n "$color" ] && [ -z "${!color}" ]; then
        color="RESET_TEXT_COLOR"
    fi

    # Print the text in the specified color
    echo -e "${!color}[$(date -u)][[ ${text} ]]${RESET_TEXT_COLOR}"
}

function command_exists() {
    command -v "$@" >/dev/null 2>&1
}

function set_default_output_volume() {
  local pib_uid

  if ! command_exists wpctl; then
    print WARN "wpctl is not installed; default output volume was not changed"
    return 0
  fi

  pib_uid="$(id -u pib 2>/dev/null)" || {
    print WARN "user 'pib' does not exist; default output volume was not changed"
    return 0
  }

  if ! sudo -u pib XDG_RUNTIME_DIR="/run/user/${pib_uid}" \
    wpctl set-volume @DEFAULT_AUDIO_SINK@ 1.0; then
    print WARN "PipeWire is not available for user 'pib'; default output volume was not changed"
    return 0
  fi

  local volume
  if ! volume="$(sudo -u pib XDG_RUNTIME_DIR="/run/user/${pib_uid}" \
    wpctl get-volume @DEFAULT_AUDIO_SINK@)"; then
    print WARN "could not verify the default output volume for user 'pib'"
    return 0
  fi

  print INFO "Default output volume: ${volume}"
  if [[ "$volume" != *"Volume: 1.00"* ]]; then
    print WARN "default output volume verification did not report Volume: 1.00"
  fi
}

# set_default_output_volume() only reaches the sink that is default while setup runs. Wireplumber
# stores volumes per route and a route it has never seen - a sink on another USB port, a newly
# recognised card - falls back to device.routes.default-sink-volume, 0.064 in the stock config and
# displayed as 40 %. The drop-in pins that fallback, so every new route starts at full volume.
# PIB_WIREPLUMBER_CONF_DIR lets the unit tests install into a scratch directory.
function install_wireplumber_volume_defaults() {
  local drop_in="$BACKEND_DIR/setup/setup_files/50-pib-volume.conf"
  local conf_dir="${PIB_WIREPLUMBER_CONF_DIR:-/home/pib/.config/wireplumber/wireplumber.conf.d}"
  local pib_uid

  if [ ! -f "$drop_in" ]; then
    print ERROR "wireplumber drop-in not found at $drop_in"
    return 1
  fi

  pib_uid="$(id -u pib 2>/dev/null)" || {
    print WARN "user 'pib' does not exist; the wireplumber volume drop-in was not installed"
    return 0
  }

  sudo mkdir -p "$conf_dir"
  sudo cp "$drop_in" "$conf_dir/50-pib-volume.conf"
  sudo chown -R pib:pib "$conf_dir"
  sudo chmod 644 "$conf_dir/50-pib-volume.conf"
  print SUCCESS "Installed wireplumber volume drop-in to $conf_dir"

  # Restart so the drop-in applies without a reboot. Stored routes keep their own volume.
  if ! sudo -u pib XDG_RUNTIME_DIR="/run/user/${pib_uid}" \
    systemctl --user restart wireplumber; then
    print WARN "could not restart wireplumber for user 'pib'; the drop-in applies after the next reboot"
    return 0
  fi

  # Give the restarted service time to accept connections before the next step talks to it.
  if command_exists wpctl; then
    local attempt
    for attempt in 1 2 3 4 5 6 7 8 9 10; do
      if sudo -u pib XDG_RUNTIME_DIR="/run/user/${pib_uid}" \
        wpctl status >/dev/null 2>&1; then
        break
      fi
      sleep 1
    done
  fi

  print SUCCESS "Restarted wireplumber for user 'pib'"
}

function warn_on_hardware_generation_mismatch() {
  local model_file="/proc/device-tree/model"
  local model expected_generation

  if [ ! -r "$model_file" ]; then
    return 0
  fi

  model="$(tr -d '\0' < "$model_file")"
  case "$PIB_HARDWARE_VARIANT" in
    pib4*)
      expected_generation="Raspberry Pi 4"
      ;;
    pib5*)
      expected_generation="Raspberry Pi 5"
      ;;
  esac

  if [[ "$model" == *"Raspberry Pi 4"* || "$model" == *"Raspberry Pi 5"* ]] &&
    [[ "$model" != *"$expected_generation"* ]]; then
    print WARN "Hardware variant ${PIB_HARDWARE_VARIANT} expects ${expected_generation}, but this device reports: ${model}"
  fi
}

# Get Linux distribution name, e.g. 'ubuntu', 'debian'
get_distribution() {
    local distribution=""
    if [ -r /etc/os-release ]; then
        distribution="$(. /etc/os-release && echo "$ID")"
    fi
    echo "$distribution"
}

# Get Linux distribution version, e.g. (ubuntu) 'noble', (debian) 'bookworm'
get_dist_version() {
  local distribution=$1
  case "$distribution" in

    ubuntu)
        if command_exists lsb_release; then
            dist_version="$(lsb_release --codename | cut -f2)"
        fi
        if [ -z "$dist_version" ] && [ -r /etc/lsb-release ]; then
            dist_version="$(. /etc/lsb-release && echo "$DISTRIB_CODENAME")"
        fi
        ;;

    debian | raspbian)
        dist_version="$(sed 's/\/.*//' /etc/debian_version | sed 's/\..*//')"
        case "$dist_version" in
        13)
            dist_version="trixie"
            ;;
        12)
            dist_version="bookworm"
            ;;
        11)
            dist_version="bullseye"
            ;;
        10)
            dist_version="buster"
            ;;
        esac
        ;;
    esac
    echo "$dist_version" |  tr '[:upper:]' '[:lower:]'
}

function is_ubuntu_noble() {
  [[ "$DISTRIBUTION" == "ubuntu" && "$DIST_VERSION" == "noble" ]]
}

function is_supported_raspbian(){
  local supported_versions=("bookworm" "trixie")
  [[ ("$DISTRIBUTION" == "raspbian" || "$DISTRIBUTION" == "debian") &&
  " ${supported_versions[@]} " =~ " ${DIST_VERSION} " ]]
}

function check_distribution() {
  if is_ubuntu_noble || is_supported_raspbian; then
    print INFO "You are running the setup-script on: $DISTRIBUTION $DIST_VERSION which is one of the supported operating-systems! So, we can happily start the setup…"
    if is_supported_raspbian && [ "$DIST_VERSION" = "bookworm" ]; then
      print WARN "Raspberry Pi OS bookworm is deprecated for pib setup. ROS 2 Jazzy (Rospian) requires Trixie. Consider upgrading to Pi OS Trixie."
    fi
    return 0
  else
    print WARN "This script expects Raspberry Pi OS on pib or Ubuntu 24.04 for systems that run the digital twin only. We detected $DISTRIBUTION $DIST_VERSION. Do you want to continue? (Y/N):"
    read -r answer
      case "$answer" in
        [Yy]*)
          echo "Continuing..."
          return 0
          ;;
        *)
          echo "Stopping setup, no changes were made."
          exit 1
          ;;
      esac
    return 1
  fi
}

function remove_apps() {
    print INFO "Removing unused default software"

    PACKAGES_TO_BE_REMOVED=("aisleriot" "gnome-sudoku" "ace-of-penguins" "gbrainy" "gnome-mines" "gnome-mahjongg" "libreoffice*" "thunderbird*")
    installed_packages_to_be_removed=""

    # Create a list of all currently installed packaged that should be removed to reduce software bloat
    for package_name in "${PACKAGES_TO_BE_REMOVED[@]}"; do
      if dpkg-query -W -f='${Status}\n' "$package_name" 2>/dev/null | grep -q "install ok installed"; then
        installed_packages_to_be_removed+="$package_name "
      fi
    done

    # Remove unnecessary packages, if any are found
    if  [ -n "$installed_packages_to_be_removed" ]; then
      sudo apt-get -y purge "$installed_packages_to_be_removed"
      sudo apt-get autoclean
    fi

    print SUCCESS "Removed unused default software"
}


function install_system_packages() {
    print INFO "Installing system packages"
    sudo apt update -qq && \
    sudo apt-get install -y git curl gnupg openssh-server >/dev/null

    # Install Node.js (LTS) via NodeSource — needed for Cerebra frontend build
    # and for running Jest/Blockly generator tests without a Docker fallback.
    if ! command_exists node; then
        print INFO "Installing Node.js (LTS) via NodeSource"
        curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash - >/dev/null 2>&1
        sudo apt-get install -y nodejs >/dev/null
        if command_exists node; then
            print SUCCESS "Node.js $(node --version) installed"
        else
            print ERROR "Failed to install Node.js"
        fi
    else
        print INFO "Node.js $(node --version) already installed"
    fi

    print SUCCESS "Installing system packages completed"
}

function install_locale() {
  sudo apt-get install -y locales
  sudo sed -i '/en_US.UTF-8/d' /etc/locale.gen
  echo "en_US.UTF-8 UTF-8" | sudo tee -a /etc/locale.gen
  sudo locale-gen en_US.UTF-8
  sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
  export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
}

# function to clone pib repositories to APP_DIR (~/app) directory
function clone_repositories() {
  # Validate branches
  if ! command_exists git; then
    print ERROR "git not found"
    exit 1
  fi

  if ! git ls-remote --exit-code --heads "$FRONTEND" "$BRANCH_FRONTEND" >/dev/null 2>&1; then
    print ERROR "Branch '${BRANCH_FRONTEND}' for Cerebra not found"
    exit 1
  fi
  if ! git ls-remote --exit-code --heads "$BACKEND" "$BRANCH_BACKEND" >/dev/null 2>&1; then
    print ERROR "Branch '${BRANCH_BACKEND}' for pib-backend not found"
    exit 1
  fi

  print INFO "Using branch '${BRANCH_FRONTEND}' for Cerebra, '${BRANCH_BACKEND}' for pib-backend"

  # Clone Repositories
  if [ ! -d "$APP_DIR" ]; then
    mkdir $APP_DIR
    print INFO "${APP_DIR} created"
  fi

  git clone -b "$BRANCH_BACKEND" "$BACKEND" "$BACKEND_DIR" || print WARN "pib-backend repository already exists"
  git clone --recurse-submodules -b "$BRANCH_FRONTEND" "$FRONTEND" "$FRONTEND_DIR" || print WARN "cerebra repository already exists"

  if [ -d "$FRONTEND_DIR/.git" ]; then
    cd "$FRONTEND_DIR" || return 1
    git submodule update --init --recursive || return 1
  fi

  local blockly_blocks="$BACKEND_DIR/pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/custom-blocks.ts"
  if [ ! -f "$blockly_blocks" ]; then
    print ERROR "vendored pib-blockly is missing required files at ${blockly_blocks}"
    return 1
  fi

  print SUCCESS "Completed cloning repositories to $APP_DIR"
}


# Install Luxonis udev rules so depthai can access the OAK camera as a non-root user.
# Without this, depthai logs "Insufficient permissions to communicate with
# X_LINK_UNBOOTED device ... Make sure udev rules are set" and cannot boot the device.
function install_depthai_udev_rules() {
  local rules_file="/etc/udev/rules.d/80-movidius.rules"
  local rule='SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'

  if [ -f "$rules_file" ] && grep -q '03e7' "$rules_file"; then
    print INFO "Luxonis udev rules already present"
  else
    echo "$rule" | sudo tee "$rules_file" > /dev/null
    print INFO "Installed Luxonis udev rules to $rules_file"
  fi

  # Reload so the rule applies without a reboot (no-op effect if udev is unavailable).
  sudo udevadm control --reload-rules && sudo udevadm trigger \
    || print WARN "could not reload udev rules; a reboot/replug may be required"
}

# Install pib-sdk and pib_mcp_server into system Python so Marimo notebooks,
# Hermes MCP, and host scripts can import them without sys.path hacks.
function install_pib_python_packages() {
  print INFO "Installing pib-sdk and pib_mcp_server into system Python"
  pip install --break-system-packages pib-sdk \
    || print WARN "failed to install pib-sdk"
  pip install --break-system-packages -e "$BACKEND_DIR/pib_mcp_server" \
    || print WARN "failed to install pib_mcp_server"
  print SUCCESS "Installed pib Python packages system-wide"
}


# Install the Hermes CLI for the pib user.
#
# WHY: docker-compose bind-mounts /home/pib/.hermes and
# /home/pib/.local/bin/hermes into ros-voice-assistant (and the profiles/
# subtree into flask-app). hermes-agent personalities need that host-side CLI
# plus the ~/.hermes profiles dir; without the install those mounts resolve to
# nothing and every hermes turn silently falls back.
#
# Provider credentials are NOT configured here — after install, run
# `sudo -u pib -H hermes setup` (or write /home/pib/.hermes/.env) once before
# hermes-agent personalities can talk to a model.
# Seed mcp_servers.pib into the Hermes base config so fresh installs expose
# pib_mcp_server tools without a manual `hermes mcp add`.
function seed_hermes_mcp_config() {
  sudo apt-get install -y python3-yaml >/dev/null
  sudo -u pib -H mkdir -p /home/pib/.hermes
  sudo -u pib -H python3 -c "
import copy, yaml, os
cfg_path = '/home/pib/.hermes/config.yaml'
# Hermes starts the MCP server as a subprocess without handing it this process'
# environment, so the entry has to carry its own env: without it pib_mcp_server
# talks to its http://localhost:5000 default and every robot tool call fails.
# Same entry as public_api_client.hermes_agent_client.PIB_MCP_SERVER;
# tests/unit/test_setup_hermes_model_pin.py fails if the two ever drift.
flask_url = os.environ.get('FLASK_API_BASE_URL', 'http://flask-app:5000')
pib_mcp_server = {
    'command': 'python3',
    'args': ['-m', 'pib_mcp_server'],
    'env': {
        'FLASK_API_BASE_URL': flask_url,
        'PIB_MCP_API_BASE_URL': flask_url,
        'PIB_MCP_ROSBRIDGE_URL': os.environ.get(
            'PIB_MCP_ROSBRIDGE_URL', 'ws://rosbridge-ws:9090'
        ),
    },
}
cfg = {}
if os.path.exists(cfg_path):
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f) or {}
changed = False
if cfg.get('model') != 'gemini-3.5-flash' or cfg.get('provider') != 'gemini':
    cfg['model'] = 'gemini-3.5-flash'
    cfg['provider'] = 'gemini'
    changed = True
servers = cfg.get('mcp_servers')
if not isinstance(servers, dict):
    servers = {}
    cfg['mcp_servers'] = servers
entry = servers.get('pib')
if not isinstance(entry, dict):
    servers['pib'] = copy.deepcopy(pib_mcp_server)
    changed = True
else:
    # Re-run on an install seeded by an older setup: add only what is missing,
    # never overwrite a command, args or env value the operator chose.
    env = entry.get('env')
    if not isinstance(env, dict):
        env = {}
        entry['env'] = env
        changed = True
    for key, value in pib_mcp_server['env'].items():
        if key not in env:
            env[key] = value
            changed = True
if changed:
    with open(cfg_path, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False)
"
  print SUCCESS "Seeded mcp_servers.pib into /home/pib/.hermes/config.yaml"
}

function install_hermes_cli() {
  local hermes_bin="/home/pib/.local/bin/hermes"
  local hermes_profiles="/home/pib/.hermes/profiles"

  if [ -x "$hermes_bin" ]; then
    print INFO "Hermes CLI already installed at $hermes_bin"
    # Keep the shared profiles dir present for the flask/voice-assistant mounts.
    sudo -u pib -H mkdir -p "$hermes_profiles"
    seed_hermes_mcp_config
    return 0
  fi

  print INFO "Installing Hermes CLI for user pib (idempotent; lands under /home/pib/.hermes)"

  # Official installer downloads Node as a .tar.xz; curl is already installed above.
  sudo apt-get install -y xz-utils >/dev/null

  # Match the NodeSource install style already used in this file (curl | bash).
  # --skip-setup keeps provisioning non-interactive; credentials come later.
  # --skip-browser skips Playwright — the voice path does not need Chromium.
  if ! sudo -u pib -H bash -c \
    'curl -fsSL https://hermes-agent.nousresearch.com/install.sh | bash -s -- --skip-setup --skip-browser'; then
    print ERROR "Hermes CLI installer failed"
    return 1
  fi

  sudo -u pib -H mkdir -p "$hermes_profiles"

  if [ -x "$hermes_bin" ]; then
    print SUCCESS "Hermes CLI installed at $hermes_bin"
  else
    print ERROR "Hermes CLI install finished but $hermes_bin is missing"
    return 1
  fi

  seed_hermes_mcp_config
}


function setup_pib_marimo_service() {
  print INFO "Setting up pib Marimo Reactive Python Notebook Service..."
  sudo -u pib -H mkdir -p /home/pib/programs/notebooks
  sudo chmod 777 /home/pib/programs/notebooks 2>/dev/null || true
  pip install --break-system-packages marimo 2>/dev/null || true

  local service_src="$BACKEND_DIR/setup/setup_files/pib-marimo.service"
  local service_target="/etc/systemd/system/pib-marimo.service"
  if [ -f "$service_src" ]; then
    sudo cp "$service_src" "$service_target"
    sudo chmod 644 "$service_target"
    sudo systemctl daemon-reload
    sudo systemctl enable pib-marimo.service
    sudo systemctl restart pib-marimo.service 2>/dev/null || true
    print SUCCESS "Installed and enabled pib-marimo.service"
  else
    print ERROR "pib-marimo.service template not found at $service_src"
  fi
}

# Install update script; move animated eyes, etc.
function move_setup_files() {
  local update_target_dir="/usr/local/bin"
  local source_file="$BACKEND_DIR/setup/update-pib.sh"
  local target_file="$update_target_dir/update-pib"

  if [[ ! -f "$source_file" ]]; then
    print ERROR "$source_file not found"
    return 1
  fi

  sudo ln -s "$source_file" "$target_file"

  sudo chmod 755 "$source_file"
  print SUCCESS "Installed update script"

  cp "$BACKEND_DIR/setup/setup_files/pib-eyes-animated.gif" "$HOME/Desktop/pib-eyes-animated.gif"
  print SUCCESS "Moved animated eyes to Desktop"

  # Add HTML that opens Cerebra + Database to the Desktop
  printf '<meta content="0; url=http://localhost" http-equiv=refresh>' > "$HOME/Desktop/Cerebra.html"
  printf '<meta content="0; url=http://localhost:8000" http-equiv=refresh>' > "$HOME/Desktop/pib_data.html"
}

function install_DBbrowser() {
  sudo apt install -y sqlitebrowser
  print SUCCESS "Installed DB browser"
}

function install_tinkerforge() {
  wget https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.asc -q -O - | sudo tee /etc/apt/trusted.gpg.d/tinkerforge.asc > /dev/null
  echo "deb https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/tinkerforge.list
  sudo apt update
  sudo apt install -y brickd brickv python3-tinkerforge

  # Disable unused mesh gateway server to save CPU resources
  if [ -f /etc/brickd.conf ]; then
    sudo sed -i 's/^listen\.mesh_gateway_port = .*/listen.mesh_gateway_port = 0/' /etc/brickd.conf
  fi

  # Apply CPUQuota and Nice limit override for brickd
  sudo mkdir -p /etc/systemd/system/brickd.service.d
  cat << 'EOF' | sudo tee /etc/systemd/system/brickd.service.d/override.conf > /dev/null
[Service]
CPUQuota=15%
Nice=10
EOF

  sudo systemctl daemon-reload
  sudo systemctl restart brickd

  print SUCCESS "Installed tinkerforge"
}

# Append a config.txt directive only when that exact line is not already in the file, so a
# second setup run does not duplicate it.
function append_config_directive_once() {
	local directive="$1"
	local file="$2"

	if grep -qxF "$directive" "$file"; then
		echo "${directive} is already set in ${file}"
		return 0
	fi

	# An unterminated last line would swallow the appended directive and defeat the check above.
	if [ -s "$file" ] && [ -n "$(tail -c 1 "$file")" ]; then
		printf '\n' | sudo tee -a "$file" > /dev/null
	fi

	echo "$directive" | sudo tee -a "$file" > /dev/null
}

# Watchdog policy: systemd is the only owner of the hardware watchdog.
# Raspberry Pi OS boots with RuntimeWatchdogUSec=1min, /dev/watchdog0 active with a 60 s
# hardware timeout and reboot=w on the kernel command line. PID 1 pings the device every
# 30 s (RuntimeWatchdogUSec/2) from its own event loop, so the 60 s deadline is only reached
# when PID 1 itself stops running - not by a `docker compose up -d --build` that runs 40+
# minutes at high load. The `watchdog` daemon this function used to install added a second,
# unconfigured owner (no checks, realtime priority 1) on the same device; a fresh install
# that did so reset the board uncleanly during the build. The package is not installed
# anymore either, because installing it on Debian enables and starts watchdog.service.
function disable_watchdog_daemon() {
	if ! command_exists systemctl; then
		return 0
	fi

	if ! systemctl list-unit-files watchdog.service 2>/dev/null | grep -q '^watchdog\.service'; then
		echo "Watchdog daemon is not installed; systemd keeps the hardware watchdog"
		return 0
	fi

	echo "Disabling the watchdog daemon installed by an earlier setup run..."
	sudo systemctl disable --now watchdog
}

function disable_power_notification() {
	# PIB_BOOT_CONFIG lets the unit tests run this function against a scratch file.
	local file="${PIB_BOOT_CONFIG:-/boot/firmware/config.txt}"

	if [ -f "$file" ]; then
		echo "Disabling under-voltage warnings..."
		append_config_directive_once "avoid_warnings=2" "$file"

		echo "Preventing CPU throttling..."
		append_config_directive_once "force_turbo=1" "$file"
	fi

	# See disable_watchdog_daemon: systemd owns /dev/watchdog0, this installer adds no second owner.
	disable_watchdog_daemon

	# Dropped here: `sed -i 's/#reboot=1/reboot=0/' /etc/watchdog.conf` matched nothing in the
	# shipped config and `kernel.panic = 0` disables reboot-on-panic, not a watchdog reset -
	# both only logged a change that never happened.
}

# Install a NetworkManager dispatcher script that observes IP changes and writes the current host IP to a file
setup_ip_dispatcher() {
  local dispatcher_script="/etc/NetworkManager/dispatcher.d/99-update-ip.sh"
  local outfile="/home/pib/app/pib-backend/pib_api/flask/host_ip.txt"

  print INFO "Creating dispatcher script..."

  sudo tee "$dispatcher_script" > /dev/null << 'EOF'
#!/bin/bash
LOG="/tmp/nm-dispatcher.log"
OUTFILE="/home/pib/app/pib-backend/pib_api/flask/host_ip.txt"

echo "$(date): Dispatcher triggered with IFACE=$1 STATE=$2" >> "$LOG"

IP=$(ip route get 1 | grep -oP 'src \K[\d.]+' || echo "")

CURRENT_IP=""
if [[ -f "$OUTFILE" ]]; then
    CURRENT_IP=$(cat "$OUTFILE")
fi

if [[ "$IP" != "$CURRENT_IP" ]]; then
    if [[ -n "$IP" ]]; then
        echo "$IP" > "$OUTFILE"
        echo "$(date): Updated IP to $IP" >> "$LOG"
    else
        > "$OUTFILE"
        echo "$(date): No IP found" >> "$LOG"
    fi
fi
EOF

  sudo chmod +x "$dispatcher_script"

  print INFO "Manually running dispatcher script to generate host_ip.txt..."
  sudo bash -c "$dispatcher_script wlan0 dhcp4-change"

  if [[ -f "$outfile" ]]; then
    print INFO "host_ip.txt was filled with the following IP:"
    cat "$outfile"
  else
    print WARN "host_ip.txt does not exist!"
  fi
}

# Persistent OAK blob store (bind-mounted into ros-camera). Never downloaded or
# compiled on the robot; setup copies vendored artefacts from models/.
PIB_MODEL_STORE_DEFAULT="/home/pib/app/pib-models"

function curated_models_dir() {
  local script_root
  script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  if [ -f "$script_root/models/manifest.yaml" ]; then
    echo "$script_root/models"
  else
    echo "$BACKEND_DIR/models"
  fi
}

function file_sha256() {
  local path="$1"
  if command_exists sha256sum; then
    sha256sum -- "$path" 2>/dev/null | awk '{print $1}'
  elif command_exists python3; then
    python3 -c 'import hashlib,sys; h=hashlib.sha256(); f=open(sys.argv[1],"rb");
h.update(f.read()); print(h.hexdigest())' "$path" 2>/dev/null
  fi
}

# Emit model_id<TAB>file<TAB>sha256 for each models/manifest.yaml entry.
function parse_model_manifest() {
  local manifest="$1"
  awk '
    /^[[:space:]]*-[[:space:]]*model_id:[[:space:]]*/ {
      sub(/^[[:space:]]*-[[:space:]]*model_id:[[:space:]]*/, "")
      gsub(/[[:space:]]+$/, "")
      model_id = $0
      file = ""
      sha = ""
      next
    }
    /^[[:space:]]*file:[[:space:]]*/ {
      sub(/^[[:space:]]*file:[[:space:]]*/, "")
      gsub(/[[:space:]]+$/, "")
      file = $0
      next
    }
    /^[[:space:]]*sha256:[[:space:]]*/ {
      sub(/^[[:space:]]*sha256:[[:space:]]*/, "")
      gsub(/[[:space:]]+$/, "")
      sha = $0
      if (model_id != "" && file != "" && sha != "") {
        printf "%s\t%s\t%s\n", model_id, file, sha
      }
      next
    }
  ' "$manifest"
}

# Make a model-store directory writable by the invoking user. Docker creates a
# missing bind-mount source as root, so recover that specific ownership problem
# without recursively changing ownership of an existing store.
function ensure_model_store_directory() {
  local directory="$1"
  local owner
  owner="$(id -u):$(id -g)"

  if ! mkdir -p -- "$directory" 2>/dev/null || [ ! -w "$directory" ]; then
    if command_exists sudo; then
      sudo mkdir -p -- "$directory" 2>/dev/null || true
      sudo chown "$owner" -- "$directory" 2>/dev/null || true
    fi
  fi

  if [ ! -d "$directory" ] || [ ! -w "$directory" ]; then
    print ERROR "model store directory is not writable: ${directory}. Run: sudo chown ${owner} '${directory}'"
    return 1
  fi
}

# Reusable by full install, --models, and --verify-models.
# mode=provision copies missing/stale blobs and manifest.yaml.
# Both modes return non-zero on any failure.
function provision_curated_models() {
  local mode="${1:-provision}"
  local store="${PIB_MODEL_STORE:-$PIB_MODEL_STORE_DEFAULT}"
  local models_dir manifest
  local placed=0 already_current=0 failed=0
  local model_id rel_file expected_sha src dest actual

  models_dir="$(curated_models_dir)"
  manifest="${models_dir}/manifest.yaml"

  print INFO "Curated model store: ${store}"

  if [ ! -f "$manifest" ]; then
    print ERROR "model manifest not found at ${manifest}"
    failed=1
    print INFO "Models summary: placed=${placed} already current=${already_current} failed=${failed} store=${store}"
    return 1
  fi

  if [ "$mode" != "verify" ]; then
    if ! ensure_model_store_directory "$store"; then
      print INFO "Models summary: placed=${placed} already current=${already_current} failed=1 store=${store}"
      return 1
    fi
  fi

  while IFS=$'\t' read -r model_id rel_file expected_sha; do
    [ -n "$model_id" ] || continue
    src="${models_dir}/${rel_file}"
    dest="${store}/${rel_file}"

    if [ "$mode" = "verify" ]; then
      if [ ! -f "$dest" ]; then
        print WARN "${model_id}: missing in store (${dest})"
        failed=$((failed + 1))
        continue
      fi
      actual="$(file_sha256 "$dest")"
      if [ "$actual" = "$expected_sha" ]; then
        print INFO "${model_id}: already current"
        already_current=$((already_current + 1))
      else
        print WARN "${model_id}: sha256 mismatch in store"
        failed=$((failed + 1))
      fi
      continue
    fi

    if [ -f "$dest" ]; then
      actual="$(file_sha256 "$dest")"
      if [ "$actual" = "$expected_sha" ]; then
        print INFO "${model_id}: already current"
        already_current=$((already_current + 1))
        continue
      fi
    fi

    if [ ! -f "$src" ]; then
      print WARN "${model_id}: vendored file missing (${src})"
      failed=$((failed + 1))
      continue
    fi

    actual="$(file_sha256 "$src")"
    if [ "$actual" != "$expected_sha" ]; then
      print WARN "${model_id}: vendored file sha256 mismatch"
      failed=$((failed + 1))
      continue
    fi

    if ! ensure_model_store_directory "$(dirname "$dest")"; then
      print ERROR "${model_id}: could not prepare writable store directory"
      failed=$((failed + 1))
      continue
    fi

    local tmp="${dest}.tmp.$$"
    if cp -f "$src" "$tmp" && mv -f "$tmp" "$dest"; then
      print INFO "${model_id}: placed"
      placed=$((placed + 1))
    else
      rm -f "$tmp"
      print WARN "${model_id}: failed to copy into store"
      failed=$((failed + 1))
    fi
  done < <(parse_model_manifest "$manifest")

  if [ "$placed" -eq 0 ] && [ "$already_current" -eq 0 ] && [ "$failed" -eq 0 ]; then
    print ERROR "no models parsed from ${manifest}"
    failed=1
  fi

  local store_manifest="${store}/manifest.yaml"
  if [ "$mode" = "verify" ]; then
    if [ ! -f "$store_manifest" ]; then
      print WARN "manifest.yaml: missing in store (${store_manifest})"
      failed=$((failed + 1))
    elif cmp -s "$manifest" "$store_manifest"; then
      print INFO "manifest.yaml: already current"
    else
      print WARN "manifest.yaml: store copy differs from vendored manifest"
      failed=$((failed + 1))
    fi
  elif [ "$failed" -eq 0 ]; then
    if cmp -s "$manifest" "$store_manifest"; then
      print INFO "manifest.yaml: already current"
    else
      local manifest_tmp="${store_manifest}.tmp.$$"
      if cp -f "$manifest" "$manifest_tmp" && mv -f "$manifest_tmp" "$store_manifest"; then
        print INFO "manifest.yaml: placed"
      else
        rm -f "$manifest_tmp"
        print ERROR "manifest.yaml: failed to copy into store"
        failed=$((failed + 1))
      fi
    fi
  fi

  print INFO "Models summary: placed=${placed} already current=${already_current} failed=${failed} store=${store}"

  if [ "$failed" -gt 0 ]; then
    print ERROR "Model provisioning failed. Fix the errors above, then run './setup/setup-pib.sh --models' before starting Docker containers."
    return 1
  fi
  return 0
}

# clean setup files if local install + remove user from sudoers file again
function cleanup() {
  if [ "$INSTALL_METHOD" = "legacy" ]; then
    sudo rm -r "$HOME/app"
    print INFO "Removed repositories from $HOME due to local installation"
  fi
  sudo rm /etc/sudoers.d/"$USER"
}


show_help()
{
	echo -e "The setup-pib.sh script has two execution modes:"
	echo -e "(normal mode and development mode)""$NEW_LINE"
	echo -e "$INFO""Normal mode (don't add any arguments or options)""$RESET_TEXT_COLOR"
	echo -e "$INFO""If you are do not know what the flags for development mode do, use the normal mode""$RESET_TEXT_COLOR"
	echo -e "Example: ./setup-pib""$NEW_LINE"
	echo -e "$INFO""Development mode (specify the branches you want to install)""$RESET_TEXT_COLOR"

	echo -e "You can either use the short or verbose command versions:"
	echo -e "-f=YourBranchName or --frontend-branch=YourBranchName"
	echo -e "-b=YourBranchName or --backend-branch=YourBranchName"
	echo -e "-l or --local for a local installation of the software over using a containerized setup using Docker"
	echo -e "--models refresh the persistent OAK model store from models/ and exit"
	echo -e "--verify-models check the model store against models/manifest.yaml and exit non-zero on mismatch"
	echo -e "--pib4edu select the pib 4 educational hardware variant"
	echo -e "--pib4advanced select the pib 4 advanced hardware variant"
	echo -e "--pib5advanced select the pib 5 advanced hardware variant"
	echo -e "--pib5museum select the pib 5 museum hardware variant"
	echo -e "Use exactly one variant flag or none (then the default pib5edu applies)."
	echo -e "A variant takes effect on a fresh install. To change it later, run:"
	echo -e "    flask seed_hardware --variant <v> --force"

	echo -e "$NEW_LINE""Examples:"
	echo -e "    ./setup-pib -b=main -f=PR-566"
    echo -e "    ./setup-pib --backend-branch=main --frontend-branch=PR-566"
	echo -e "    ./setup-pib --models"
	echo -e "    ./setup-pib --verify-models"
	echo -e "Provision models before starting Docker containers so the bind-mount store is created with the correct owner."

	exit
}


# ---------- SETUP STARTS FROM HERE -----------

# VALIDATE CLI ARGUMENTS (before sudo so --models / --verify-models stay offline and local)
BRANCH_BACKEND="main"
BRANCH_FRONTEND="main"
INSTALL_METHOD="docker"
MODELS_ONLY=false
VERIFY_MODELS_ONLY=false
HARDWARE_VARIANT_ARGUMENTS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -f=* | --frontend-branch=*)
      BRANCH_FRONTEND="${1#*=}"
      ;;
    -b=* | --backend-branch=*)
      BRANCH_BACKEND="${1#*=}"
      ;;
    -l | --legacy)
      INSTALL_METHOD="legacy"
      ;;
    --models)
      MODELS_ONLY=true
      ;;
    --verify-models)
      VERIFY_MODELS_ONLY=true
      ;;
    --pib4edu | --pib4advanced | --pib5advanced | --pib5museum | --pib*)
      HARDWARE_VARIANT_ARGUMENTS+=("$1")
      ;;
    -h | --help)
      show_help
      ;;
    *)
      print ERROR "invalid input options"
      exit 1
      ;;
  esac
  shift
done

if ! load_hardware_variant_resolver; then
  exit 1
fi

if ! PIB_HARDWARE_VARIANT="$(
  resolve_hardware_variant "${HARDWARE_VARIANT_ARGUMENTS[@]}"
)"; then
  exit 1
fi
export PIB_HARDWARE_VARIANT

if [ "$MODELS_ONLY" = true ] && [ "$VERIFY_MODELS_ONLY" = true ]; then
  print ERROR "use either --models or --verify-models, not both"
  exit 1
fi

if [ "$MODELS_ONLY" = true ]; then
  provision_curated_models provision
  exit $?
fi

if [ "$VERIFY_MODELS_ONLY" = true ]; then
  provision_curated_models verify
  exit $?
fi

# Reduplicate output to an extra log file as well
LOG_FILE="$HOME/setup-pib.log"
exec > >(tee -a "$LOG_FILE") 2>&1

warn_on_hardware_generation_mismatch

echo "Hello $USER! We start the setup by allowing you permanently to run commands with admin-privileges. This change is reverted at the end of the setup."
if [[ "$(id)" == *"(sudo)"* ]]; then
	echo "For this change please enter your password..."
	sudo bash -c "echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
else
	echo "For this change please enter the root-password. It is most likely just your normal one..."
	su root bash -c "usermod -aG sudo $USER ; echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
fi

printf '%s\n' "$PIB_HARDWARE_VARIANT" |
  sudo tee /etc/pib_hardware_variant >/dev/null
print INFO "Selected hardware variant: ${PIB_HARDWARE_VARIANT}"

DISTRIBUTION=$(get_distribution) # e.g., 'ubuntu'
export DISTRIBUTION
DIST_VERSION=$(get_dist_version "$DISTRIBUTION")  # e.g., 'noble'
export DIST_VERSION
check_distribution

if is_ubuntu_noble; then
  remove_apps || print ERROR "failed to remove default software"
fi

if is_supported_raspbian; then
  disable_power_notification || print ERROR "failed to disable power notifications"
fi

install_system_packages || { print ERROR "failed to install system packages"; return 1; }
install_locale || { print ERROR "failed to install locale"; return 1; }
clone_repositories || { print ERROR "failed to clone repositories"; return 1; }
# After checkout, before containers: copy vendored OAK blobs into the bind-mounted store.
provision_curated_models provision || {
  print ERROR "Model provisioning must succeed before containers are started"
  exit 1
}
install_pib_python_packages || print ERROR "failed to install pib Python packages"
# Before docker-compose starts: hermes must exist on the host so the
# ros-voice-assistant / flask-app bind mounts resolve to real paths.
install_hermes_cli || print ERROR "failed to install Hermes CLI"
# The legacy ~/imitation host script is retired. ros_packages/imitation consumes
# the camera owner's typed topic and must never run beside another OAK device owner.
if is_supported_raspbian && [ "$DIST_VERSION" = "trixie" ]; then
  source "$SETUP_INSTALLATION_DIR/ros_jazzy_install.sh" || { print ERROR "failed to install ROS 2 Jazzy"; return 1; }
fi
move_setup_files || print ERROR "failed to move setup files"
setup_pib_marimo_service || print ERROR "failed to setup pib marimo service"
install_DBbrowser || print ERROR "failed to install DB browser"
install_tinkerforge || print ERROR "failed to install tinkerforge"
setup_ip_dispatcher || print ERROR "failed to setup ip dispatcher"
source "$SETUP_INSTALLATION_DIR/set_system_settings.sh" || print ERROR "failed to set system settings"
install_wireplumber_volume_defaults || print WARN "failed to install the wireplumber volume drop-in"
set_default_output_volume || print WARN "failed to set default output volume"
print INFO "${INSTALL_METHOD}"
if [ "$INSTALL_METHOD" = "legacy" ]; then
  print INFO "Going to install Cerebra locally (LEGACY MODE NOT WORKING ON RASPBERRY PI 5)"
  source "$SETUP_INSTALLATION_DIR/local_install.sh" || print ERROR "failed to install Cerebra locally"
elif is_ubuntu_noble || is_supported_raspbian; then
  print INFO "Going to install Cerebra via Docker"
  source "$SETUP_INSTALLATION_DIR/docker_install.sh" || print ERROR "failed to install Cerebra via Docker"
fi
cleanup

print SUCCESS "Installation completed"
print SUCCESS "Reboot pib to apply all changes"
