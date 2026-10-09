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
  # Not `--depth 1`: the release tag sits on the merge's second parent, so a
  # depth-1 checkout cannot resolve the installed version (see resolve_app_version
  # in setup/update_runner.sh). Measured: --depth 2 keeps HEAD^2 and the tag,
  # --depth 1 loses both.
  print INFO "Download the repository and run setup/setup-pib.sh from there: git clone --branch ${BRANCH_BACKEND} ${BACKEND}"
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

# require_nonempty NAME...: fail loudly when a variable a file operation is about to use is
# empty. `cp "" ...`, `grep ... ""` and `sed -i ... ""` only print a one-line complaint and the
# surrounding step still reports success; this turns that into a named error. A global `set -u`
# is not an option here because the ROS setup.bash files this script sources rely on unset
# variables, so the guard sits at the call sites instead.
function require_nonempty() {
  local name
  for name in "$@"; do
    if [ -z "${!name}" ]; then
      print ERROR "internal error: variable ${name} is empty (${FUNCNAME[1]:-top level})"
      return 1
    fi
  done
}

# ---- step runner --------------------------------------------------------------------------
# The install log mixes this script's progress with everything the steps print, so each step
# gets a header, a footer with duration and result, and a line in the closing summary.
# PIB_SETUP_STEP_LOG lets the unit tests read the summary data without parsing the output.
STEP_NAMES=()
STEP_RESULTS=()
STEP_DURATIONS=()
STEP_FAILURES=0

# run_step LABEL COMMAND [ARGS...]: run COMMAND, record and print its result, and return its
# exit status so callers keep deciding whether a failure is fatal.
function run_step() {
  local label="$1"
  shift
  local index=$(( ${#STEP_NAMES[@]} + 1 ))
  local started finished duration status result

  started="$(date +%s)"
  print INFO "==== step ${index}: ${label} (started $(date -u +%H:%M:%S) UTC) ===="
  "$@"
  status=$?
  finished="$(date +%s)"
  duration=$((finished - started))

  if [ "$status" -eq 0 ]; then
    result="ok"
    print SUCCESS "==== step ${index}: ${label}: ok (${duration}s) ===="
  else
    result="FAILED rc=${status}"
    STEP_FAILURES=$((STEP_FAILURES + 1))
    print ERROR "==== step ${index}: ${label}: FAILED rc=${status} (${duration}s) ===="
  fi
  STEP_NAMES+=("$label")
  STEP_RESULTS+=("$result")
  STEP_DURATIONS+=("$duration")
  if [ -n "${PIB_SETUP_STEP_LOG:-}" ]; then
    printf '%s\t%s\t%s\n' "$label" "$result" "$duration" >> "$PIB_SETUP_STEP_LOG"
  fi
  return "$status"
}

# One line per step, in order. This is the part of the log to read first.
function print_step_summary() {
  local index total="${#STEP_NAMES[@]}"
  print INFO "Setup summary: ${total} steps, ${STEP_FAILURES} failed"
  for (( index = 0; index < total; index++ )); do
    printf '  %2d. %-48s %-16s %6ss\n' \
      "$((index + 1))" "${STEP_NAMES[$index]}" "${STEP_RESULTS[$index]}" "${STEP_DURATIONS[$index]}"
  done
}

# A step that must not be survived: print the summary so far and leave with status 1.
function abort_setup() {
  print ERROR "$1"
  print_step_summary
  exit 1
}

# wpctl addresses the default sink as @DEFAULT_AUDIO_SINK@. Right after wireplumber (re)starts
# there is no default node yet and wpctl answers "Translate ID error: '-1' is not a valid ID",
# so wait for one to appear before touching its volume. PIB_WPCTL_WAIT_SECONDS is for tests.
function wait_for_default_audio_sink() {
  local pib_uid="$1"
  local attempts="${PIB_WPCTL_WAIT_SECONDS:-15}"
  local attempt
  for (( attempt = 1; attempt <= attempts; attempt++ )); do
    if sudo -u pib XDG_RUNTIME_DIR="/run/user/${pib_uid}" \
      wpctl inspect @DEFAULT_AUDIO_SINK@ >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
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

  if ! wait_for_default_audio_sink "$pib_uid"; then
    print WARN "no default audio sink appeared within ${PIB_WPCTL_WAIT_SECONDS:-15}s; default output volume was not changed"
    return 0
  fi

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
    # python3-yaml: installation_scripts/provision_whisper_model.py and seed_hermes_mcp_config
    sudo apt-get update -qq && \
    sudo apt-get install -y git curl gnupg openssh-server python3-yaml >/dev/null

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

# True when the C library can already switch to en_US.UTF-8 on this machine.
function locale_is_generated() {
  locale -a 2>/dev/null | grep -qixE 'en_US\.(UTF-8|utf8)'
}

# Runs first thing after sudo is available: an ssh session forwards the client's LC_ALL and
# every command before the locale exists complains "setlocale: LC_ALL: cannot change locale".
# Until then the script runs under C.UTF-8, which glibc always provides.
function install_locale() {
  if locale_is_generated; then
    print INFO "Locale en_US.UTF-8 is already generated"
  else
    export LANG=C.UTF-8 LC_ALL=C.UTF-8
    if ! command_exists locale-gen; then
      sudo apt-get update -qq
      sudo apt-get install -y locales >/dev/null || return 1
    fi
    if [ -f /etc/locale.gen ]; then
      sudo sed -i '/^#\? *en_US.UTF-8 UTF-8/d' /etc/locale.gen
    fi
    echo "en_US.UTF-8 UTF-8" | sudo tee -a /etc/locale.gen >/dev/null
    sudo locale-gen en_US.UTF-8 || return 1
    if ! locale_is_generated; then
      print ERROR "locale-gen ran but en_US.UTF-8 is still not available"
      return 1
    fi
    print SUCCESS "Generated locale en_US.UTF-8"
  fi
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
  # No --recurse-submodules: neither repository has a .gitmodules, so the flag and
  # the explicit `git submodule update --init --recursive` that used to follow it
  # were both no-ops. The update path still runs `git submodule update --init
  # --recursive` on the frontend (setup/update_runner.sh, setup/update-pib.sh), and
  # that one reads .gitmodules from the working tree - so a submodule added later
  # would be initialised by the first update.
  git clone -b "$BRANCH_FRONTEND" "$FRONTEND" "$FRONTEND_DIR" || print WARN "cerebra repository already exists"

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
if cfg.get('model') != 'gemini-3.8-flash' or cfg.get('provider') != 'gemini':
    cfg['model'] = 'gemini-3.8-flash'
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

# The Hermes installer drops its CLI into ~/.local/bin, which no shell on a fresh Raspberry Pi
# OS has on PATH: `which hermes` over ssh answers nothing. Two files cover every shell kind:
#   /etc/profile.d/pib-local-bin.sh  - login shells (console, `ssh pib@robot`, desktop session)
#   the top of /home/pib/.bashrc     - non-login shells: terminals, and the commands sshd runs
#                                      (`ssh pib@robot which hermes`; Debian's bash reads
#                                      ~/.bashrc for those, but ~/.bashrc returns early for
#                                      non-interactive shells, so the block must sit above
#                                      that guard - appending it would not work).
# PIB_PROFILE_D and PIB_USER_HOME let the unit tests write into a scratch directory.
PIB_LOCAL_BIN_MARKER="# pib: ~/.local/bin on PATH (setup-pib.sh)"

function local_bin_path_snippet() {
  cat <<'EOF'
case ":${PATH}:" in
  *":${HOME}/.local/bin:"*) ;;
  *) PATH="${HOME}/.local/bin:${PATH}" ;;
esac
export PATH
EOF
}

function install_local_bin_path() {
  local profile_d="${PIB_PROFILE_D:-/etc/profile.d}"
  local user_home="${PIB_USER_HOME:-/home/pib}"
  local profile_file="${profile_d}/pib-local-bin.sh"
  local bashrc="${user_home}/.bashrc"
  local staged

  require_nonempty profile_d user_home || return 1

  staged="$(mktemp)" || return 1
  {
    echo "$PIB_LOCAL_BIN_MARKER"
    echo "# Login shells. The Hermes CLI installs into ~/.local/bin."
    local_bin_path_snippet
  } > "$staged"
  sudo install -o root -g root -m 0644 "$staged" "$profile_file" || { rm -f "$staged"; return 1; }
  rm -f "$staged"
  print INFO "Installed ${profile_file}"

  if [ -f "$bashrc" ] && grep -qF "$PIB_LOCAL_BIN_MARKER" "$bashrc"; then
    print INFO "${bashrc} already puts ~/.local/bin on PATH"
  else
    staged="$(mktemp)" || return 1
    {
      echo "$PIB_LOCAL_BIN_MARKER"
      echo "# Kept above the interactive-shell guard so ssh commands see it too."
      local_bin_path_snippet
      echo ""
      if [ -f "$bashrc" ]; then
        cat "$bashrc"
      fi
    } > "$staged"
    sudo install -o pib -g pib -m 0644 "$staged" "$bashrc" || { rm -f "$staged"; return 1; }
    rm -f "$staged"
    print INFO "Prepended the ~/.local/bin PATH block to ${bashrc}"
  fi

  print SUCCESS "PATH of user pib includes ~/.local/bin in login and non-login shells"
}

# Run a Hermes command as pib with ~/.local/bin on PATH. sudo's secure_path
# drops that directory, and that is where the installer puts the CLI.
function hermes_as_pib() {
  sudo -u pib -H bash -c 'export PATH="$HOME/.local/bin:$PATH"; "$@"' bash "$@"
}

# The installer can leave an executable wrapper whose dependency environment was
# never committed. `hermes --version` is the check: it exits non-zero with
# "run hermes pm repair" until that environment exists. A binary that is merely
# present is not a working CLI. A second run whose version already answers
# does not repair again.
function verify_hermes_cli() {
  local output status

  output="$(hermes_as_pib hermes --version 2>&1)"
  status=$?
  if [ "$status" -ne 0 ] || [ -z "$output" ]; then
    print INFO "Hermes CLI is not usable (${output:-no output}); running hermes pm repair"
    if ! hermes_as_pib hermes pm repair; then
      print ERROR "hermes pm repair failed"
      return 1
    fi
    output="$(hermes_as_pib hermes --version 2>&1)"
    status=$?
    if [ "$status" -ne 0 ] || [ -z "$output" ]; then
      print ERROR "hermes --version still fails after pm repair (${output:-no output})"
      return 1
    fi
  fi
  print SUCCESS "Hermes CLI: ${output}"
}

function install_hermes_cli() {
  local hermes_bin="/home/pib/.local/bin/hermes"
  local hermes_profiles="/home/pib/.hermes/profiles"

  if [ -x "$hermes_bin" ]; then
    print INFO "Hermes CLI already installed at $hermes_bin"
    # Keep the shared profiles dir present for the flask/voice-assistant mounts.
    sudo -u pib -H mkdir -p "$hermes_profiles"
    seed_hermes_mcp_config
    verify_hermes_cli
    return $?
  fi

  print INFO "Installing Hermes CLI for user pib (idempotent; lands under /home/pib/.hermes)"

  # Official installer downloads Node as a .tar.xz; curl is already installed above.
  sudo apt-get install -y xz-utils >/dev/null

  # Match the NodeSource install style already used in this file (curl | bash).
  # --skip-setup keeps provisioning non-interactive; credentials come later.
  # --skip-browser skips Playwright — the voice path does not need Chromium.
  # sudo resets PATH to secure_path, so ~/.local/bin is put back explicitly: the installer
  # (and the uv tool installs it runs) otherwise warn once per tool that it is not on PATH.
  if ! sudo -u pib -H bash -c \
    'export PATH="$HOME/.local/bin:$PATH"; curl -fsSL https://hermes-agent.nousresearch.com/install.sh | bash -s -- --skip-setup --skip-browser'; then
    print ERROR "Hermes CLI installer failed"
    return 1
  fi

  sudo -u pib -H mkdir -p "$hermes_profiles"

  if [ ! -x "$hermes_bin" ]; then
    print ERROR "Hermes CLI install finished but $hermes_bin is missing"
    return 1
  fi

  seed_hermes_mcp_config
  verify_hermes_cli
}


# 2xx/3xx from the editor. Connection refused and 4xx/5xx are not "active".
function marimo_http_ok() {
  local code
  code="$(curl -sS -o /dev/null -w '%{http_code}' --max-time 2 http://127.0.0.1:2718/ || true)"
  case "$code" in
    2*|3*) return 0 ;;
    *) return 1 ;;
  esac
}

function setup_pib_marimo_service() {
  local notebooks="${PIB_MARIMO_NOTEBOOKS:-/home/pib/programs/notebooks}"
  local service_src="$BACKEND_DIR/setup/setup_files/pib-marimo.service"
  local service_target="${PIB_MARIMO_UNIT:-/etc/systemd/system/pib-marimo.service}"
  local unit_changed=0 attempt state

  print INFO "Setting up pib Marimo Reactive Python Notebook Service..."
  sudo -u pib -H mkdir -p "$notebooks" || return 1
  sudo chmod 777 "$notebooks" || return 1

  if [ ! -f "$service_src" ]; then
    print ERROR "pib-marimo.service template not found at $service_src"
    return 1
  fi

  # The unit runs /usr/bin/python3 as user pib. Installing as that interpreter
  # and then checking it as pib is the same import the unit will do. A missing
  # module used to be discarded (`|| true`) and the unit restart-looped in
  # "activating" while this step reported success.
  if ! sudo -u pib -H /usr/bin/python3 -m marimo --version >/dev/null 2>&1; then
    if ! /usr/bin/python3 -m pip --version >/dev/null 2>&1; then
      sudo apt-get install -y python3-pip || return 1
    fi
    if ! /usr/bin/python3 -m pip install --break-system-packages 'marimo>=0.10.0'; then
      print ERROR "could not install marimo for /usr/bin/python3"
      return 1
    fi
  fi
  if ! sudo -u pib -H /usr/bin/python3 -m marimo --version >/dev/null 2>&1; then
    print ERROR "marimo is not importable for user pib"
    return 1
  fi

  sudo mkdir -p "$(dirname "$service_target")" || return 1
  if ! cmp -s "$service_src" "$service_target"; then
    unit_changed=1
    sudo cp "$service_src" "$service_target" || return 1
    sudo chmod 644 "$service_target" || return 1
    sudo systemctl daemon-reload || return 1
  fi
  sudo systemctl enable pib-marimo.service || return 1

  # A second run whose unit file is unchanged and whose server already answers
  # does not restart it.
  if [ "$unit_changed" -eq 1 ] || ! systemctl is-active --quiet pib-marimo.service || ! marimo_http_ok; then
    if ! sudo systemctl restart pib-marimo.service; then
      print ERROR "could not restart pib-marimo.service"
      return 1
    fi
  else
    print INFO "pib-marimo already active"
  fi

  attempt=1
  while [ "$attempt" -le 20 ]; do
    if systemctl is-active --quiet pib-marimo.service && marimo_http_ok; then
      print SUCCESS "pib-marimo is active and answering on port 2718"
      return 0
    fi
    attempt=$((attempt + 1))
    sleep 1
  done

  state="$(systemctl is-active pib-marimo.service 2>&1 || true)"
  print ERROR "pib-marimo did not become active (systemctl is-active: ${state})"
  return 1
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

  require_nonempty HOME BACKEND_DIR || return 1
  mkdir -p "$HOME/Desktop" || return 1
  cp "$BACKEND_DIR/setup/setup_files/pib-eyes-animated.gif" "$HOME/Desktop/pib-eyes-animated.gif" || return 1
  print SUCCESS "Moved animated eyes to Desktop"

  # Add HTML that opens Cerebra + Database to the Desktop
  printf '<meta content="0; url=http://localhost" http-equiv=refresh>' > "$HOME/Desktop/Cerebra.html"
  printf '<meta content="0; url=http://localhost:8000" http-equiv=refresh>' > "$HOME/Desktop/pib_data.html"
}

function install_DBbrowser() {
  sudo apt-get install -y sqlitebrowser
  print SUCCESS "Installed DB browser"
}

function install_tinkerforge() {
  wget https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.asc -q -O - | sudo tee /etc/apt/trusted.gpg.d/tinkerforge.asc > /dev/null
  echo "deb https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/tinkerforge.list
  sudo apt-get update
  sudo apt-get install -y brickd brickv python3-tinkerforge

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

# `ip -4 route get 1` asks for a route to 0.0.0.1, which follows the default
# route and prints `src <address>`. That address is not configured here.
# awk, not grep -P: a grep without PCRE used to yield an empty string, and an
# empty string compared equal to a missing file, so the file was never created.
function host_ip_file_ok() {
  local path="$1"
  local value
  [ -f "$path" ] || return 1
  value="$(tr -d '[:space:]' < "$path")"
  [[ "$value" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]
}

# Install a NetworkManager dispatcher that writes the current host IPv4 to
# /etc/pib_host_ip (mounted into flask-app) and to the legacy flask file.
# PIB_NM_DISPATCHER, PIB_HOST_IP_FILE and PIB_HOST_IP_LEGACY_FILE let the
# unit tests use a scratch directory. A missing address fails the step.
setup_ip_dispatcher() {
  local dispatcher_script="${PIB_NM_DISPATCHER:-/etc/NetworkManager/dispatcher.d/99-update-ip.sh}"
  local primary="${PIB_HOST_IP_FILE:-/etc/pib_host_ip}"
  local legacy="${PIB_HOST_IP_LEGACY_FILE:-${BACKEND_DIR}/pib_api/flask/host_ip.txt}"
  local primary_q legacy_q

  print INFO "Creating dispatcher script..."
  sudo mkdir -p "$(dirname "$dispatcher_script")" "$(dirname "$primary")" "$(dirname "$legacy")" || return 1

  primary_q="$(printf '%q' "$primary")"
  legacy_q="$(printf '%q' "$legacy")"
  sudo tee "$dispatcher_script" > /dev/null <<EOF
#!/bin/bash
PRIMARY=${primary_q}
LEGACY=${legacy_q}
IP=\$(ip -4 route get 1 2>/dev/null | awk '{for (i = 1; i <= NF; i++) if (\$i == "src") { print \$(i + 1); exit }}')
if [[ ! "\$IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+\$ ]]; then
  exit 0
fi
for outfile in "\$PRIMARY" "\$LEGACY"; do
  current=""
  if [[ -f "\$outfile" ]]; then
    current=\$(tr -d '[:space:]' < "\$outfile")
  fi
  if [[ "\$IP" != "\$current" ]]; then
    mkdir -p "\$(dirname "\$outfile")"
    printf '%s\n' "\$IP" > "\$outfile"
  fi
done
EOF

  sudo chmod 755 "$dispatcher_script" || return 1

  print INFO "Running the dispatcher once to record the host IP..."
  sudo bash "$dispatcher_script" || return 1

  if ! host_ip_file_ok "$primary" || ! host_ip_file_ok "$legacy"; then
    print ERROR "host IP was not written to ${primary} and ${legacy}"
    return 1
  fi
  print SUCCESS "host IP recorded in ${primary}: $(tr -d '[:space:]' < "$primary")"
}

# Persistent OAK blob store (bind-mounted into ros-camera at /models). The
# compiled blobs are not in git and are not reproducible on the robot, so the
# published release asset is the master copy. setup downloads that tarball into
# a cache outside the repo, checks its sha256 before extracting, and copies
# from the cache into PIB_MODEL_STORE. models/manifest.yaml in the repository
# stays the checksum source of truth. --verify-models reads only the store and
# that manifest; it does not fetch.
PIB_MODEL_STORE_DEFAULT="/home/pib/app/pib-models"
# Published as a pre-release: pre-releases are excluded from releases/latest, so
# this data release never becomes the repository's latest release and never
# enters the version pairing guard.
PIB_MODEL_ASSET_TAG_DEFAULT="model-registry-2026-09-15"
PIB_MODEL_ASSET_NAME_DEFAULT="models-2026-09-15.tar.gz"
PIB_MODEL_ASSET_SHA256_DEFAULT="07357ec99e0b91014285aad2e6a03fcd1e6ca95fd7ed686670f7ab73c1f3ff02"

function model_asset_tag() {
  echo "${PIB_MODEL_ASSET_TAG:-$PIB_MODEL_ASSET_TAG_DEFAULT}"
}

function model_asset_name() {
  echo "${PIB_MODEL_ASSET_NAME:-$PIB_MODEL_ASSET_NAME_DEFAULT}"
}

function model_asset_sha256() {
  echo "${PIB_MODEL_ASSET_SHA256:-$PIB_MODEL_ASSET_SHA256_DEFAULT}"
}

# Default URL is the GitHub release asset for the tag and name above. Override
# PIB_MODEL_ASSET_URL, PIB_MODEL_ASSET_TAG, PIB_MODEL_ASSET_NAME, or
# PIB_MODEL_ASSET_SHA256 when the published file moves; no code change is required.
function model_asset_url() {
  if [ -n "${PIB_MODEL_ASSET_URL:-}" ]; then
    echo "$PIB_MODEL_ASSET_URL"
    return
  fi
  local tag name
  tag="$(model_asset_tag)"
  name="$(model_asset_name)"
  echo "https://github.com/pib-rocks/pib-backend/releases/download/${tag}/${name}"
}

function model_blob_cache_dir() {
  echo "${PIB_MODEL_CACHE:-${HOME}/app/.cache/pib-models}"
}

function curated_manifest_path() {
  local script_root
  script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  if [ -f "$script_root/models/manifest.yaml" ]; then
    echo "$script_root/models/manifest.yaml"
  else
    echo "$BACKEND_DIR/models/manifest.yaml"
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

# Return 0 when every blob named by the manifest exists under root and matches
# sha256. Composite entries have no file and are ignored. Used for the cache
# and, when a download fails, for an already-populated store.
function path_matches_model_manifest() {
  local root="$1"
  local manifest="$2"
  local model_id rel_file expected_sha src actual count=0

  [ -d "$root" ] || return 1
  while IFS=$'\t' read -r model_id rel_file expected_sha; do
    [ -n "$model_id" ] || continue
    count=$((count + 1))
    src="${root}/${rel_file}"
    if [ ! -f "$src" ]; then
      return 1
    fi
    actual="$(file_sha256 "$src")"
    if [ "$actual" != "$expected_sha" ]; then
      return 1
    fi
  done < <(parse_model_manifest "$manifest")
  [ "$count" -gt 0 ]
}

function model_store_has_any_blob() {
  local store="$1"
  local manifest="$2"
  local model_id rel_file expected_sha

  while IFS=$'\t' read -r model_id rel_file expected_sha; do
    [ -n "$rel_file" ] || continue
    if [ -f "${store}/${rel_file}" ]; then
      return 0
    fi
  done < <(parse_model_manifest "$manifest")
  return 1
}

function report_model_asset_unavailable() {
  local store="$1"
  local manifest="$2"
  local url tag sha256 state
  url="$(model_asset_url)"
  tag="$(model_asset_tag)"
  sha256="$(model_asset_sha256)"
  if model_store_has_any_blob "$store" "$manifest"; then
    state="incomplete"
  else
    state="empty"
  fi
  print ERROR "OAK model asset could not be fetched and the model store at ${store} is ${state}."
  print ERROR "The compiled blobs are not in git. They are published as release asset ${tag}: ${url} (sha256 ${sha256})."
  print ERROR "Set PIB_MODEL_ASSET_URL to a reachable copy of that tarball, and PIB_MODEL_ASSET_SHA256 if that copy's checksum differs, then re-run './setup/setup-pib.sh --models'."
}

# Download the release asset, verify its sha256, then extract it. The checksum
# is checked before tar runs so a corrupt download never replaces the cache.
function fetch_and_unpack_model_asset() {
  local cache="$1"
  local url tag expected actual archive staging parent installed

  if [ -z "$cache" ] || [ "$cache" = "/" ] || [ "$cache" = "." ]; then
    print ERROR "refusing to use model cache path '${cache}'"
    return 1
  fi

  url="$(model_asset_url)"
  tag="$(model_asset_tag)"
  expected="$(model_asset_sha256)"
  if [ -z "$expected" ]; then
    print ERROR "PIB_MODEL_ASSET_SHA256 is empty; refusing to extract an unverified OAK model asset"
    return 1
  fi

  print INFO "Fetching OAK model asset ${tag} from ${url}"
  archive="$(mktemp)"
  if command_exists curl; then
    if ! curl -fsSL "$url" -o "$archive"; then
      rm -f "$archive"
      print ERROR "could not download the OAK model asset from ${url}"
      return 1
    fi
  elif command_exists wget; then
    if ! wget -q "$url" -O "$archive"; then
      rm -f "$archive"
      print ERROR "could not download the OAK model asset from ${url}"
      return 1
    fi
  else
    rm -f "$archive"
    print ERROR "neither curl nor wget is available to download ${url}"
    return 1
  fi

  actual="$(file_sha256 "$archive")"
  if [ "$actual" != "$expected" ]; then
    rm -f "$archive"
    print ERROR "OAK model asset sha256 mismatch (got ${actual}, expected ${expected}). Refusing to extract ${url}."
    return 1
  fi
  print INFO "OAK model asset sha256 verified"

  if tar -tzf "$archive" | grep -E '(^|/)\.\.(/|$)|^/' >/dev/null; then
    rm -f "$archive"
    print ERROR "OAK model asset contains unsafe paths; refusing to extract ${url}"
    return 1
  fi

  staging="$(mktemp -d)"
  if ! tar -xzf "$archive" -C "$staging"; then
    rm -f "$archive"
    rm -rf "$staging"
    print ERROR "could not unpack the OAK model asset from ${url}"
    return 1
  fi
  rm -f "$archive"

  if [ ! -f "$staging/manifest.yaml" ]; then
    rm -rf "$staging"
    print ERROR "OAK model asset has no manifest.yaml at its archive root (${url})"
    return 1
  fi

  parent="$(dirname "$cache")"
  if ! ensure_model_store_directory "$parent"; then
    rm -rf "$staging"
    return 1
  fi

  installed="${cache}.new.$$"
  rm -rf "$installed"
  if ! mv "$staging" "$installed"; then
    rm -rf "$staging" "$installed"
    print ERROR "could not stage the unpacked OAK model asset for ${cache}"
    return 1
  fi
  rm -rf "$cache"
  if ! mv "$installed" "$cache"; then
    print ERROR "could not install the unpacked OAK model asset into ${cache}"
    return 1
  fi
  return 0
}

# Reusable by full install, --models, and --verify-models.
# mode=provision copies missing/stale blobs from the release-asset cache and
# copies manifest.yaml from the repository. mode=verify is offline.
# Both modes return non-zero on any failure.
function provision_curated_models() {
  local mode="${1:-provision}"
  local store="${PIB_MODEL_STORE:-$PIB_MODEL_STORE_DEFAULT}"
  local models_dir manifest
  local placed=0 already_current=0 failed=0
  local model_id rel_file expected_sha src dest actual

  manifest="$(curated_manifest_path)"
  models_dir=""

  print INFO "Curated model store: ${store}"

  if [ ! -f "$manifest" ]; then
    print ERROR "model manifest not found at ${manifest}"
    failed=1
    print INFO "Models summary: placed=${placed} already current=${already_current} failed=${failed} store=${store}"
    return 1
  fi

  # Verify never downloads. Provision reads blobs from the cache, fetching the
  # release asset only when that cache does not already match the manifest.
  if [ "$mode" != "verify" ]; then
    models_dir="$(model_blob_cache_dir)"
    if path_matches_model_manifest "$models_dir" "$manifest"; then
      print INFO "OAK model cache already matches models/manifest.yaml (${models_dir})"
    elif ! fetch_and_unpack_model_asset "$models_dir"; then
      if path_matches_model_manifest "$store" "$manifest"; then
        print WARN "OAK model asset could not be fetched; the store at ${store} already matches models/manifest.yaml, so it was left in place."
      else
        report_model_asset_unavailable "$store" "$manifest"
        print ERROR "Model provisioning failed. Fix the errors above, then run './setup/setup-pib.sh --models' before starting Docker containers."
        print INFO "Models summary: placed=${placed} already current=${already_current} failed=1 store=${store}"
        return 1
      fi
    elif ! path_matches_model_manifest "$models_dir" "$manifest"; then
      print ERROR "Unpacked OAK model asset at ${models_dir} does not match ${manifest}."
      print ERROR "The tarball sha256 matched, but a blob does not match the in-repo manifest. Refusing to provision."
      print ERROR "Model provisioning failed. Fix the errors above, then run './setup/setup-pib.sh --models' before starting Docker containers."
      print INFO "Models summary: placed=${placed} already current=${already_current} failed=1 store=${store}"
      return 1
    fi

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
      print WARN "${model_id}: cached model file missing (${src})"
      failed=$((failed + 1))
      continue
    fi

    actual="$(file_sha256 "$src")"
    if [ "$actual" != "$expected_sha" ]; then
      print WARN "${model_id}: cached model file sha256 mismatch"
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
      print WARN "manifest.yaml: store copy differs from the in-repo manifest"
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

# Voice weights are not in models/manifest.yaml. The checkout that carries voice/whisper-model.yaml:
# the repository this script lives in when run from a clone, otherwise the clone in BACKEND_DIR.
function whisper_repo_root() {
  local script_root
  script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  if [ -f "$script_root/voice/whisper-model.yaml" ]; then
    echo "$script_root"
  else
    echo "$BACKEND_DIR"
  fi
}

# Place the faster-whisper weights into /data/voice/models/whisper/ with the self-contained
# installation_scripts/provision_whisper_model.py: vendored voice/whisper/ tree first, else the
# pinned files from voice/whisper-model.yaml, fetched now while the network is up. Nothing here
# imports the voice_assistant ROS package. The step's stderr goes to the log unchanged, so a
# failure shows its reason, and the step returns non-zero so the summary shows it as failed.
function provision_whisper_model() {
  local repo_root dest helper output result
  repo_root="$(whisper_repo_root)"
  dest="${WHISPER_MODEL_PATH:-/data/voice/models/whisper}"
  helper="$repo_root/setup/installation_scripts/provision_whisper_model.py"

  if [ ! -f "$repo_root/voice/whisper-model.yaml" ]; then
    print WARN "whisper model: ${repo_root}/voice/whisper-model.yaml not found; nothing to place"
    return 0
  fi
  if [ ! -f "$helper" ]; then
    print ERROR "whisper model: ${helper} is missing"
    return 1
  fi
  if ! command_exists python3; then
    print ERROR "whisper model: python3 is missing"
    return 1
  fi
  if ! ensure_model_store_directory "$dest"; then
    print ERROR "whisper model: store directory ${dest} is not writable"
    return 1
  fi

  print INFO "whisper model: store ${dest}"
  # stdout carries only the result word; stderr (the reasons) passes straight into the log.
  output="$(python3 "$helper" --repo-root "$repo_root" --destination "$dest")"
  result="${output##*result=}"
  case "$result" in
    placed)
      print SUCCESS "whisper model: placed into ${dest}"
      ;;
    already_current)
      print INFO "whisper model: already current in ${dest}"
      ;;
    *)
      print ERROR "whisper model: provisioning failed (${result:-no result}); see the lines above"
      return 1
      ;;
  esac
  if [ ! -f "$dest/small/model.bin" ] && [ ! -f "$dest/model.bin" ] && \
    ! compgen -G "$dest/*/model.bin" >/dev/null; then
    print ERROR "whisper model: no model.bin under ${dest} after provisioning"
    return 1
  fi
  return 0
}

# Local Ollama with a CPU-tuned qwen2.5:1.5b named qwen-fast. The official installer
# (https://ollama.com/install.sh) creates a systemd service that runs as its own user
# and owns the model store. This step installs only when the binary is missing, starts
# the service only when it is down (never restarts a healthy one), and pulls or creates
# a model only when `ollama list` does not already show it. A second run therefore
# re-downloads nothing and does not fail on an existing qwen-fast.
#
# Whether to install is decided from PIB_HARDWARE_VARIANT, which this setup has already
# resolved. A variant that is not in that list logs one skip line and returns success,
# so the step still appears in the setup summary and is not a failure. The RAM check
# below stays as a warning for the variants that do install.
#
# qwen2.5:1.5b Q4 weights are about 1.0-1.1 GiB plus the KV cache for num_ctx 2048.
# 1200 MiB is that requirement. Less than this is a warning before the pull, not a
# hard stop, so an operator on a small machine can still proceed deliberately.
#
# Containers reach the daemon at host.docker.internal, which is the bridge
# address, not 127.0.0.1. The official unit binds the loopback interface, so
# the tags probe from flask fails and Cerebra never offers qwen-fast. The
# drop-in binds every interface. PIB_OLLAMA_DROPIN points the unit tests at a
# scratch file. An already-active unit is not restarted; a fresh install
# writes the drop-in before the first start, so that start picks it up.
function ensure_ollama_listen_dropin() {
  local dropin="${PIB_OLLAMA_DROPIN:-/etc/systemd/system/ollama.service.d/override.conf}"
  local staged="/tmp/pib-ollama-dropin.$$"
  local already_active=0

  # Absolute paths: the unit tests run this function with a PATH that contains
  # only their stubs, and a bare mkdir/cmp would miss the real coreutils.
  printf '%s\n' '[Service]' 'Environment="OLLAMA_HOST=0.0.0.0:11434"' > "$staged" || return 1
  if [ -f "$dropin" ] && /usr/bin/cmp -s "$staged" "$dropin"; then
    /usr/bin/rm -f "$staged"
    print INFO "ollama: container listen address already configured"
    return 0
  fi
  if systemctl is-active --quiet ollama; then
    already_active=1
  fi
  sudo /usr/bin/mkdir -p "$(/usr/bin/dirname "$dropin")" || { /usr/bin/rm -f "$staged"; return 1; }
  sudo /usr/bin/tee "$dropin" >/dev/null < "$staged" || { /usr/bin/rm -f "$staged"; return 1; }
  /usr/bin/rm -f "$staged"
  sudo systemctl daemon-reload || return 1
  if [ "$already_active" -eq 1 ]; then
    print WARN "ollama: OLLAMA_HOST applies on the next start; this step does not restart a service that is already active"
  else
    print INFO "ollama: configured OLLAMA_HOST=0.0.0.0:11434"
  fi
}

function install_ollama_qwen_fast() {
  local modelfile="" version="" available_kib="" available_mib="" required_mib
  local curl_status=0 installer_status=0 model_names="" line="" name=""
  local has_base=0 has_fast=0 service_user="" ready_attempts attempt=0 tags=""

  # pib5edu, pib5advanced and pib5museum are the generation-5 (8 GiB) variants.
  case "${PIB_HARDWARE_VARIANT:-}" in
    pib5edu | pib5advanced | pib5museum)
      ;;
    *)
      print INFO "ollama: skipping variant ${PIB_HARDWARE_VARIANT:-unset}; the local model requires 8 GiB"
      return 0
      ;;
  esac

  if [ -f "${BACKEND_DIR}/setup/ollama/Modelfile" ]; then
    modelfile="${BACKEND_DIR}/setup/ollama/Modelfile"
  elif [ -f "${SETUP_SCRIPT_DIR}/ollama/Modelfile" ]; then
    modelfile="${SETUP_SCRIPT_DIR}/ollama/Modelfile"
  else
    print ERROR "ollama: Modelfile not found under ${BACKEND_DIR}/setup/ollama or ${SETUP_SCRIPT_DIR}/ollama"
    return 1
  fi

  if command_exists ollama; then
    print INFO "ollama: already installed"
  else
    # The current official installer prefers a .tar.zst archive and exits if zstd
    # is missing. Install it first so that failure is not the only signal.
    if ! command_exists zstd; then
      print INFO "ollama: installing zstd so the official installer can extract its archive"
      if ! command_exists apt-get; then
        print ERROR "ollama: zstd is required by https://ollama.com/install.sh and apt-get is missing"
        return 1
      fi
      if ! sudo apt-get install -y zstd; then
        print ERROR "ollama: could not install zstd, which the official installer needs"
        return 1
      fi
    fi
    print INFO "ollama: installing from https://ollama.com/install.sh"
    # This script does not set pipefail. Both statuses have to be read in one
    # assignment: a failed download would otherwise look like success when sh
    # exits 0 on empty input.
    curl -fsSL https://ollama.com/install.sh | sh
    curl_status=${PIPESTATUS[0]} installer_status=${PIPESTATUS[1]}
    if [ "$curl_status" -ne 0 ]; then
      print ERROR "ollama: could not download https://ollama.com/install.sh (network unavailable?)"
      return 1
    fi
    if [ "$installer_status" -ne 0 ]; then
      print ERROR "ollama: official installer failed (rc=${installer_status})"
      return 1
    fi
    hash -r
    if ! command_exists ollama; then
      if [ -x /usr/local/bin/ollama ]; then
        export PATH="/usr/local/bin:${PATH}"
      elif [ -x /usr/bin/ollama ]; then
        export PATH="/usr/bin:${PATH}"
      fi
    fi
    if ! command_exists ollama; then
      print ERROR "ollama: installer finished but the ollama binary is still missing"
      return 1
    fi
  fi

  if ! version="$(ollama --version 2>&1)"; then
    print ERROR "ollama: could not read the installed version"
    return 1
  fi
  print INFO "ollama: version ${version}"

  if ! command_exists systemctl; then
    print ERROR "ollama: systemctl is missing; cannot enable the ollama service"
    return 1
  fi

  ensure_ollama_listen_dropin || return 1

  # Enable, then start only when the unit is down. Neither call restarts a unit
  # that is already active, and this step never calls restart. Pulling while the
  # unit is down would start a second server as pib and write ~/.ollama instead
  # of the service user's store.
  if systemctl is-enabled --quiet ollama; then
    print INFO "ollama: service already enabled"
  elif ! sudo systemctl enable ollama; then
    print ERROR "ollama: could not enable the ollama service"
    return 1
  else
    print INFO "ollama: service enabled"
  fi

  if systemctl is-active --quiet ollama; then
    print INFO "ollama: service already active; leaving it running"
  elif ! sudo systemctl start ollama; then
    print ERROR "ollama: could not start the ollama service"
    return 1
  elif ! systemctl is-active --quiet ollama; then
    print ERROR "ollama: service is not active after start"
    return 1
  else
    print INFO "ollama: service started"
  fi

  service_user="$(systemctl show -p User --value ollama 2>/dev/null || true)"
  if [ -n "$service_user" ]; then
    print INFO "ollama: service runs as user ${service_user}; the model store stays with that user"
  else
    print WARN "ollama: could not read the service user; the model store must stay with the ollama service, not with pib"
  fi

  required_mib=1200
  available_kib=""
  if ! command_exists free; then
    print WARN "ollama: free is missing; cannot check RAM (qwen2.5:1.5b needs ${required_mib} MiB)"
  else
    while IFS= read -r line; do
      if [[ "$line" == Mem:* ]]; then
        available_kib="${line##* }"
      fi
    done < <(free -k)
    if ! [[ "$available_kib" =~ ^[0-9]+$ ]]; then
      print WARN "ollama: could not read available RAM; qwen2.5:1.5b needs ${required_mib} MiB"
    else
      available_mib=$((available_kib / 1024))
      if [ "$available_mib" -lt "$required_mib" ]; then
        print WARN "ollama: ${available_mib} MiB available RAM is below the ${required_mib} MiB qwen2.5:1.5b needs (Q4 weights plus the 2048-token KV cache)"
      else
        print INFO "ollama: ${available_mib} MiB available RAM (${required_mib} MiB required for qwen2.5:1.5b)"
      fi
    fi
  fi

  # The unit can be active before it accepts connections. PIB_OLLAMA_READY_ATTEMPTS
  # is for tests; on the device this waits up to 15s.
  ready_attempts="${PIB_OLLAMA_READY_ATTEMPTS:-15}"
  attempt=1
  while true; do
    if model_names="$(ollama list 2>/dev/null)"; then
      break
    fi
    if [ "$attempt" -ge "$ready_attempts" ]; then
      print ERROR "ollama: the service is up but 'ollama list' failed"
      return 1
    fi
    attempt=$((attempt + 1))
    sleep 1
  done

  has_base=0
  has_fast=0
  while IFS= read -r line || [ -n "$line" ]; do
    [ -z "$line" ] && continue
    name="${line%% *}"
    if [ "$name" = "qwen2.5:1.5b" ]; then
      has_base=1
    elif [ "$name" = "qwen-fast" ] || [ "$name" = "qwen-fast:latest" ]; then
      has_fast=1
    fi
  done <<< "$model_names"

  if [ "$has_base" -eq 1 ]; then
    print INFO "ollama: qwen2.5:1.5b already present; not pulling"
  else
    print INFO "ollama: pulling qwen2.5:1.5b"
    if ! ollama pull qwen2.5:1.5b; then
      print ERROR "ollama: pull of qwen2.5:1.5b failed"
      return 1
    fi
  fi

  if [ "$has_fast" -eq 1 ]; then
    print INFO "ollama: qwen-fast already present; not recreating"
  else
    print INFO "ollama: creating qwen-fast from ${modelfile}"
    if ! ollama create qwen-fast -f "$modelfile"; then
      print ERROR "ollama: could not create qwen-fast from ${modelfile}"
      return 1
    fi
  fi

  # The catalogue offers qwen-fast only when this document lists it. A unit
  # that is "active" but whose tags endpoint does not name the model is the
  # same false green as a missing host IP file.
  if ! tags="$(curl -fsS --max-time 5 http://127.0.0.1:11434/api/tags)"; then
    print ERROR "ollama: http://127.0.0.1:11434/api/tags did not answer"
    return 1
  fi
  case "$tags" in
    *qwen-fast*)
      print INFO "ollama: /api/tags lists qwen-fast"
      ;;
    *)
      print ERROR "ollama: /api/tags does not list qwen-fast"
      return 1
      ;;
  esac
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
	echo -e "--models fetch the OAK release asset into \$HOME/app/.cache/pib-models when the cache does not match models/manifest.yaml (override with PIB_MODEL_CACHE, PIB_MODEL_ASSET_URL, PIB_MODEL_ASSET_TAG, PIB_MODEL_ASSET_NAME, PIB_MODEL_ASSET_SHA256), refresh the persistent OAK model store from that cache, place the whisper weights (fetched once if voice/whisper/ is empty; PIB_WHISPER_DOWNLOAD=0 forbids that) and exit"
	echo -e "--verify-models check the model store against models/manifest.yaml offline and exit non-zero on mismatch"
	echo -e "--no-smart-chats install without the Hermes channel; Direct is the only chat path"
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

# VALIDATE CLI ARGUMENTS (before the sudoers step, so --models / --verify-models stay local;
# --verify-models is offline and does not fetch. --models fetches the OAK release
# asset when the cache does not match, and the whisper weights only when they are missing)
BRANCH_BACKEND="main"
BRANCH_FRONTEND="main"
INSTALL_METHOD="docker"
MODELS_ONLY=false
VERIFY_MODELS_ONLY=false
SMART_CHATS_ENABLED=1
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
    --no-smart-chats)
      SMART_CHATS_ENABLED=0
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
  models_status=$?
  provision_whisper_model || models_status=1
  exit "$models_status"
fi

if [ "$VERIFY_MODELS_ONLY" = true ]; then
  provision_curated_models verify
  exit $?
fi

# Reduplicate output to an extra log file as well
LOG_FILE="$HOME/setup-pib.log"
exec > >(tee -a "$LOG_FILE") 2>&1

# An ssh client may forward LC_ALL=en_US.UTF-8 before that locale exists on a fresh image; every
# command would then warn "setlocale: LC_ALL: cannot change locale". Run under C.UTF-8, which
# glibc always has, until install_locale has generated en_US.UTF-8.
if ! locale_is_generated; then
  export LANG=C.UTF-8 LC_ALL=C.UTF-8
fi

warn_on_hardware_generation_mismatch

echo "Hello $USER! We start the setup by allowing you permanently to run commands with admin-privileges. This change is reverted at the end of the setup."
if [[ "$(id)" == *"(sudo)"* ]]; then
	echo "For this change please enter your password..."
	sudo bash -c "echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
else
	echo "For this change please enter the root-password. It is most likely just your normal one..."
	su root bash -c "usermod -aG sudo $USER ; echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
fi

# First step on purpose: everything below runs commands that honour LC_ALL.
run_step "Generate locale en_US.UTF-8" install_locale || print ERROR "failed to install locale"

printf '%s\n' "$PIB_HARDWARE_VARIANT" |
  sudo tee /etc/pib_hardware_variant >/dev/null
print INFO "Selected hardware variant: ${PIB_HARDWARE_VARIANT}"

# Durable record of --no-smart-chats. Personality rows are not rewritten:
# clearing the marker restores Smart for personalities that stored it.
if [ -f "$SETUP_SCRIPT_DIR/installation_scripts/smart_chats.sh" ]; then
  # shellcheck source=/dev/null
  source "$SETUP_SCRIPT_DIR/installation_scripts/smart_chats.sh"
  SMART_CHATS_MARKER="$(smart_chats_marker "$SMART_CHATS_ENABLED")"
elif [ "$SMART_CHATS_ENABLED" = "0" ]; then
  SMART_CHATS_MARKER="disabled"
else
  SMART_CHATS_MARKER="enabled"
fi
printf '%s\n' "$SMART_CHATS_MARKER" | sudo tee /etc/pib_smart_chats >/dev/null
if [ "$SMART_CHATS_ENABLED" = "0" ]; then
  export PIB_SMART_CHATS=0
else
  export PIB_SMART_CHATS=1
fi
print INFO "Hermes channel: ${SMART_CHATS_MARKER}"

DISTRIBUTION=$(get_distribution) # e.g., 'ubuntu'
export DISTRIBUTION
DIST_VERSION=$(get_dist_version "$DISTRIBUTION")  # e.g., 'noble'
export DIST_VERSION
check_distribution

if is_ubuntu_noble; then
  run_step "Remove unused default software" remove_apps || print ERROR "failed to remove default software"
fi

if is_supported_raspbian; then
  run_step "Disable power notifications" disable_power_notification || print ERROR "failed to disable power notifications"
fi

# Steps whose failure leaves nothing to build on end the setup through abort_setup; the rest
# are reported in the summary and the setup goes on.
run_step "Install system packages" install_system_packages || abort_setup "failed to install system packages"
run_step "Clone repositories" clone_repositories || abort_setup "failed to clone repositories"
# After checkout, before containers: fetch the OAK release asset when the cache
# does not match models/manifest.yaml, then copy blobs into the bind-mounted store.
run_step "Provision curated OAK models" provision_curated_models provision ||
  abort_setup "Model provisioning must succeed before containers are started"
run_step "Provision whisper model" provision_whisper_model || print ERROR "failed to provision the whisper model"
# After the clone: the Modelfile is setup/ollama/Modelfile in the backend checkout.
# A failure is recorded in the summary and setup continues; nothing else is pointed
# at this model yet. A skipped variant returns success, so this step stays in the
# summary for every variant.
run_step "Install Ollama qwen-fast" install_ollama_qwen_fast || print ERROR "failed to install Ollama qwen-fast"
run_step "Install pib Python packages" install_pib_python_packages || print ERROR "failed to install pib Python packages"
# Before the Hermes installer runs: ~/.local/bin must be on PATH for every shell of user pib.
run_step "Put ~/.local/bin on PATH" install_local_bin_path || print ERROR "failed to put ~/.local/bin on PATH"
# Before docker-compose starts: hermes must exist on the host so the
# ros-voice-assistant / flask-app bind mounts resolve to real paths.
run_step "Install Hermes CLI" install_hermes_cli || print ERROR "failed to install Hermes CLI"
# The legacy ~/imitation host script is retired. ros_packages/imitation consumes
# the camera owner's typed topic and must never run beside another OAK device owner.
if is_supported_raspbian && [ "$DIST_VERSION" = "trixie" ]; then
  run_step "Install ROS 2 Jazzy" source "$SETUP_INSTALLATION_DIR/ros_jazzy_install.sh" || abort_setup "failed to install ROS 2 Jazzy"
fi
run_step "Install setup files" move_setup_files || print ERROR "failed to move setup files"
run_step "Set up pib Marimo service" setup_pib_marimo_service || print ERROR "failed to setup pib marimo service"
run_step "Install DB browser" install_DBbrowser || print ERROR "failed to install DB browser"
run_step "Install Tinkerforge" install_tinkerforge || print ERROR "failed to install tinkerforge"
run_step "Set up IP dispatcher" setup_ip_dispatcher || abort_setup "failed to setup ip dispatcher"
run_step "Adjust system settings" source "$SETUP_INSTALLATION_DIR/set_system_settings.sh" || print ERROR "failed to set system settings"
run_step "Install wireplumber volume drop-in" install_wireplumber_volume_defaults || print WARN "failed to install the wireplumber volume drop-in"
run_step "Set default output volume" set_default_output_volume || print WARN "failed to set default output volume"
print INFO "${INSTALL_METHOD}"
if [ "$INSTALL_METHOD" = "legacy" ]; then
  print INFO "Going to install Cerebra locally (LEGACY MODE NOT WORKING ON RASPBERRY PI 5)"
  run_step "Install Cerebra locally" source "$SETUP_INSTALLATION_DIR/local_install.sh" || print ERROR "failed to install Cerebra locally"
elif is_ubuntu_noble || is_supported_raspbian; then
  print INFO "Going to install Cerebra via Docker"
  # docker_install.sh runs its own run_step calls, one per service it sets up.
  source "$SETUP_INSTALLATION_DIR/docker_install.sh" || print ERROR "failed to install Cerebra via Docker"
fi
run_step "Clean up" cleanup || print WARN "cleanup failed"

print_step_summary
if [ "$STEP_FAILURES" -eq 0 ]; then
  print SUCCESS "Installation completed"
else
  print ERROR "Installation completed with ${STEP_FAILURES} failed step(s); see the summary above"
fi
print SUCCESS "Reboot pib to apply all changes"
