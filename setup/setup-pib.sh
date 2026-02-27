#!/bin/bash

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

# Log file in same directory as this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="${SCRIPT_DIR}/setup-pib.log"

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
    return 0
  else
    print WARN "This script expects Raspberry Pi OS on pib or Ubuntu 24.04 for systems that run the digital twin only. We detected $DISTRIBUTION $DIST_VERSION. Do you want to continue? (Y/N):"
    echo -e "${WARN}Do you want to continue? (Y/N):${RESET_TEXT_COLOR}" >&3
    read -r answer
      case "$answer" in
        [Yy]*)
          echo "Continuing..."
          echo "Continuing..." >&3
          return 0
          ;;
        *)
          echo "Stopping setup, no changes were made."
          console_error "Stopping setup, no changes were made."
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
    sudo apt-get install -y git curl openssh-server >/dev/null
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
    console_error "git not found"
    exit 1
  fi

  if ! git ls-remote --exit-code --heads "$FRONTEND" "$BRANCH_FRONTEND" >/dev/null 2>&1; then
    print ERROR "Branch '${BRANCH_FRONTEND}' for Cerebra not found"
    console_error "Branch '${BRANCH_FRONTEND}' for Cerebra not found"
    exit 1
  fi
  if ! git ls-remote --exit-code --heads "$BACKEND" "$BRANCH_BACKEND" >/dev/null 2>&1; then
    print ERROR "Branch '${BRANCH_BACKEND}' for pib-backend not found"
    console_error "Branch '${BRANCH_BACKEND}' for pib-backend not found"
    exit 1
  fi

  print INFO "Using branch '${BRANCH_FRONTEND}' for Cerebra, '${BRANCH_BACKEND}' for pib-backend"

  # Clone Repositories
  if [ ! -d "$APP_DIR" ]; then
    mkdir $APP_DIR
    print INFO "${APP_DIR} created"
  fi

  git clone --recurse-submodules -b "$BRANCH_BACKEND" $BACKEND "$BACKEND_DIR" || print WARN "pib-backend repository already exists"
  git clone --recurse-submodules -b "$BRANCH_FRONTEND" $FRONTEND "$FRONTEND_DIR" || print WARN "cerebra repository already exists"

  print SUCCESS "Completed cloning repositories to $APP_DIR"
}


# Install update script; move animated eyes, etc.
function move_setup_files() {
  local update_target_dir="/usr/local/bin"
  local source_file="$BACKEND_DIR/setup/update-pib.sh"
  local target_file="$update_target_dir/update-pib"

  if [[ ! -f "$source_file" ]]; then
    print ERROR "$source_file not found"
    console_error "$source_file not found"
    return 1
  fi

  sudo ln -s "$source_file" "$target_file"

  sudo chmod 755 "$source_file"
  print SUCCESS "Installed update script"

  # Ensure Desktop exists (headless/server installs may not have it)
  mkdir -p "$HOME/Desktop"

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
  print SUCCESS "Installed tinkerforge"
}

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

# Install OpenClaw AI assistant (Node ≥ 22 + npm global package + gateway daemon)
function install_openclaw() {
  print INFO "Installing OpenClaw"

  # ── 1. Ensure Node 22 is available ──────────────────────────────────────────
  # On Noble, install_frontend already set up nvm with Node 18 for the Angular
  # build. OpenClaw requires Node ≥ 22, so we install it separately via the
  # NodeSource binary package — this puts a system-wide node22 binary at
  # /usr/bin/node that does not interfere with the nvm Node 18 used by pib.
  # On Raspbian, no Node is present at all, so we install the same way.
  local NODE_BIN
  NODE_BIN=$(command -v node 2>/dev/null || true)
  local NODE_VERSION=0
  if [ -n "$NODE_BIN" ]; then
    NODE_VERSION=$("$NODE_BIN" -e "process.stdout.write(String(process.versions.node.split('.')[0]))" 2>/dev/null || echo 0)
  fi

  if [ "$NODE_VERSION" -lt 22 ] 2>/dev/null; then
    print INFO "Node < 22 detected (found: $NODE_VERSION); installing Node 22 via NodeSource"
    curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash - && \
    sudo apt-get install -y nodejs || { print ERROR "Failed to install Node 22"; return 1; }
    NODE_BIN=$(command -v node)
    print INFO "Node $("$NODE_BIN" --version) installed"
  else
    print INFO "Node $NODE_VERSION already satisfies ≥ 22 requirement"
  fi

  # ── 2. Configure npm global prefix to a user-writable directory ─────────────
  # Avoids EACCES errors on 'npm install -g' without sudo.
  # IMPORTANT: We must NOT use 'npm config set prefix' here because that writes
  # a `prefix=` line into ~/.npmrc, which nvm detects as incompatible and prints
  # a warning on every new terminal session. Instead, we export NPM_CONFIG_PREFIX
  # as an environment variable and persist it in .bashrc.
  local NPM_PREFIX="$HOME/.npm-global"
  mkdir -p "$NPM_PREFIX"

  # Clean up any pre-existing prefix/globalconfig lines from ~/.npmrc that a
  # previous run of this script (or any other tool) may have written. These
  # lines cause the "incompatible with nvm" warning on every terminal open.
  if [ -f "$HOME/.npmrc" ]; then
    sed -i '/^prefix\s*=/d' "$HOME/.npmrc"
    sed -i '/^globalconfig\s*=/d' "$HOME/.npmrc"
    print INFO "Removed prefix/globalconfig from ~/.npmrc to prevent nvm conflict"
  fi

  export NPM_CONFIG_PREFIX="$NPM_PREFIX"
  export PATH="$NPM_PREFIX/bin:$PATH"
  # Persist for future shells via env var (not ~/.npmrc) to stay nvm-compatible.
  if ! grep -q 'NPM_CONFIG_PREFIX' "$HOME/.bashrc"; then
    echo "export NPM_CONFIG_PREFIX=\"$NPM_PREFIX\"" >> "$HOME/.bashrc"
    echo "export PATH=\"$NPM_PREFIX/bin:\$PATH\"" >> "$HOME/.bashrc"
  fi

  # ── 3. Install OpenClaw CLI ──────────────────────────────────────────────────
  if command_exists openclaw; then
    print WARN "OpenClaw already installed ($(openclaw --version 2>/dev/null || echo unknown)); skipping npm install"
  else
    npm install -g openclaw@latest || { print ERROR "Failed to install OpenClaw"; return 1; }
    print INFO "OpenClaw $(openclaw --version 2>/dev/null || echo installed)"
  fi

  # ── 4. Non-interactive onboard: workspace + gateway config (no auth/channels)
  # Defers API-key setup to the user; --skip-skills avoids interactive prompts.
  # The workspace is seeded at the default location (~/.openclaw/workspace).
  openclaw onboard \
    --non-interactive \
    --mode local \
    --gateway-port 18789 \
    --gateway-bind loopback \
    --skip-skills \
    || print WARN "OpenClaw onboard returned non-zero; gateway may need manual auth setup"

  # ── 5. Install and enable the gateway systemd user service ──────────────────
  # 'openclaw daemon install' writes ~/.config/systemd/user/openclaw-gateway.service
  # and enables it. loginctl enable-linger ensures the user service survives logout.
  openclaw daemon install --runtime node || { print ERROR "Failed to install OpenClaw gateway service"; return 1; }
  sudo loginctl enable-linger "$USER" || print WARN "loginctl enable-linger failed; gateway may not start on boot"
  systemctl --user daemon-reload
  systemctl --user enable openclaw-gateway.service --now || print WARN "Could not start openclaw-gateway.service immediately; it will start on next login"

  print SUCCESS "OpenClaw installed — run 'openclaw onboard' to configure channels and API keys"
}


# Remove temporary sudoers entry (repositories in $HOME/app are kept for both native and Docker)
function cleanup() {
  sudo rm -f /etc/sudoers.d/"$USER"
}


show_help()
{
	echo -e "The setup-pib.sh script installs Cerebra and pib-backend.""$NEW_LINE"
	echo -e "$INFO""On Ubuntu 24.04 the script installs Cerebra natively (development: ROS2 Jazzy, no Docker).""$RESET_TEXT_COLOR"
	echo -e "$INFO""On Raspberry Pi OS (bookworm/trixie) it installs Cerebra via Docker (production).""$RESET_TEXT_COLOR"
	echo -e "Example: ./setup-pib""$NEW_LINE"
	echo -e "$INFO""Development mode (specify the branches you want to install):""$RESET_TEXT_COLOR"
	echo -e "-f=YourBranchName or --frontend-branch=YourBranchName"
	echo -e "-b=YourBranchName or --backend-branch=YourBranchName"
	echo -e "$NEW_LINE""Examples:"
	echo -e "    ./setup-pib -b=main -f=PR-566"
	echo -e "    ./setup-pib --backend-branch=main --frontend-branch=PR-566"
	exit
}


# ---------- SETUP STARTS FROM HERE -----------

# Handle --help before redirecting stdout so help is shown on the terminal
for arg in "$@"; do
  if [[ "$arg" == "-h" || "$arg" == "--help" ]]; then
    show_help
    exit 0
  fi
done

# Redirect stdout/stderr to log only; keep terminal on fd 3 for minimal console output
exec 3>&1
exec 1>>"$LOG_FILE" 2>>"$LOG_FILE"

# Console output helpers: minimal lines to terminal (fd 3) and one line to log
console_success() {
  echo "[$(date -u)] [SUCCESS] $*"
  echo -e "${SUCCESS}${*}${RESET_TEXT_COLOR}" >&3
}
console_error() {
  echo "[$(date -u)] [ERROR] $*"
  echo -e "${ERROR}${*}${RESET_TEXT_COLOR}" >&3
}
console_info() {
  echo "[$(date -u)] [INFO] $*"
  echo -e "${INFO}${*}${RESET_TEXT_COLOR}" >&3
}

echo "Detailed log: $LOG_FILE"
console_info "Log file: $LOG_FILE"

echo "Requesting sudo privileges for setup..."
console_info "Hello $USER! We start the setup by allowing you to run commands with admin-privileges (reverted at the end)."
if [[ "$(id)" == *"(sudo)"* ]]; then
	echo "For this change please enter your password..." >&3
	sudo bash -c "echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
else
	echo "For this change please enter the root-password. It is most likely just your normal one..." >&3
	su root bash -c "usermod -aG sudo $USER ; echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
fi
console_success "Privileges set"


DISTRIBUTION=$(get_distribution) # e.g., 'ubuntu'
export DISTRIBUTION
DIST_VERSION=$(get_dist_version "$DISTRIBUTION")  # e.g., 'noble'
export DIST_VERSION
check_distribution


# VALIDATE CLI ARGUMENTS
# Default backend branch: use the branch this script is running from (if inside a git repo),
# so that running setup-pib.sh from a PR checkout automatically installs that PR branch.
_detected_branch=""
if command_exists git; then
  _detected_branch="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --abbrev-ref HEAD 2>/dev/null)"
  # Ignore detached HEAD or empty result
  if [[ "$_detected_branch" == "HEAD" || -z "$_detected_branch" ]]; then
    _detected_branch=""
  fi
fi
BRANCH_BACKEND="${_detected_branch:-main}"
BRANCH_FRONTEND="main"
while [ $# -gt 0 ]; do
  case "$1" in
    -f=* | --frontend-branch=*)
      BRANCH_FRONTEND="${1#*=}"
      ;;
    -b=* | --backend-branch=*)
      BRANCH_BACKEND="${1#*=}"
      ;;
    -h | --help)
      show_help
      ;;
    *)
      print ERROR "invalid input options"
      console_error "Invalid input options"
      exit 1
  esac
  shift
done

if is_ubuntu_noble; then
  remove_apps || { print ERROR "failed to remove default software"; console_error "Failed to remove default software"; exit 1; }
  console_success "Removed default software"
fi

if is_supported_raspbian; then
  disable_power_notification || { print ERROR "failed to disable power notifications"; console_error "Failed to disable power notifications"; exit 1; }
  console_success "Power notifications disabled"
fi

install_system_packages || { print ERROR "failed to install system packages"; console_error "Failed to install system packages"; exit 1; }
console_success "System packages installed"
install_locale || { print ERROR "failed to install locale"; console_error "Failed to install locale"; exit 1; }
console_success "Locale installed"
clone_repositories || { print ERROR "failed to clone repositories"; console_error "Failed to clone repositories"; exit 1; }
console_success "Repositories cloned"
move_setup_files || { print ERROR "failed to move setup files"; console_error "Failed to move setup files"; exit 1; }
console_success "Setup files moved"
install_DBbrowser || { print ERROR "failed to install DB browser"; console_error "Failed to install DB browser"; exit 1; }
console_success "DB browser installed"
install_tinkerforge || { print ERROR "failed to install tinkerforge"; console_error "Failed to install tinkerforge"; exit 1; }
console_success "Tinkerforge installed"
setup_ip_dispatcher || { print ERROR "failed to setup ip dispatcher"; console_error "Failed to setup IP dispatcher"; exit 1; }
console_success "IP dispatcher configured"
source "$SETUP_INSTALLATION_DIR/set_system_settings.sh" || { print ERROR "failed to set system settings"; console_error "Failed to set system settings"; exit 1; }
console_success "System settings applied"
if is_ubuntu_noble; then
  print INFO "Installing Cerebra natively (Ubuntu 24.04 development setup)"
  source "$SETUP_INSTALLATION_DIR/local_install.sh" || { print ERROR "failed to install Cerebra natively"; console_error "Failed to install Cerebra natively"; exit 1; }
  console_success "Cerebra installed natively"
elif is_supported_raspbian; then
  print INFO "Installing Cerebra via Docker (Raspberry Pi OS production)"
  source "$SETUP_INSTALLATION_DIR/docker_install.sh" || { print ERROR "failed to install Cerebra via Docker"; console_error "Failed to install Cerebra via Docker"; exit 1; }
  sudo usermod -aG docker pib || { print ERROR "failed to add user 'pib' to docker group"; console_error "Failed to add user to docker group"; exit 1; }
  console_success "Cerebra installed via Docker"
else
  print ERROR "Unsupported OS. Use Ubuntu 24.04 for native (development) install or Raspberry Pi OS (bookworm/trixie) for Docker (production) install."
  console_error "Unsupported OS. Use Ubuntu 24.04 or Raspberry Pi OS."
  exit 1
fi
install_openclaw || print WARN "OpenClaw installation failed; pib will work normally but the AI assistant will not be available"
console_success "OpenClaw installed"
cleanup
console_success "Cleanup done"

print SUCCESS "Finished installation, for more information on how to use pib and Cerebra, visit https://pib-rocks.atlassian.net/wiki/spaces/kb/overview?homepageId=65077450"
print SUCCESS "Reboot pib to apply all changes"
console_success "Installation finished. Reboot to apply all changes."
