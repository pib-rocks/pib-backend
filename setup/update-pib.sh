#!/bin/bash
#
# This script assumes:
#   - that setup-pib was already executed
#   - the user "pib" is executing it

# Color definitions for logging
export ERROR="\e[31m"
export WARN="\e[33m"
export SUCCESS="\e[32m"
export INFO="\e[36m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

set -e  # Stop on errors

# Configuration
DEFAULT_USER="pib"
APP_DIR="$HOME/app"
BACKEND_DIR="$APP_DIR/pib-backend"
FRONTEND_DIR="$APP_DIR/cerebra"
LOG_FILE="$HOME/update-pib.log"

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

# Prints the release tag of the current checkout. docs/RELEASE.md: the tag published on
# develop rides on the second parent of the `develop -> main` merge, or on HEAD after a
# fast-forward. Fails (non-zero) when the checkout carries no tag.
function release_tag_here() {
    local tag
    tag="$(git tag --points-at 'HEAD^2' 2>/dev/null | head -n 1)"
    if [ -z "$tag" ]; then
        tag="$(git tag --points-at HEAD 2>/dev/null | head -n 1)"
    fi
    [ -n "$tag" ] || return 1
    printf '%s\n' "$tag"
}

# Ensure that the update-pib command is installed as a symlink
function ensure_symlink_self() {
    local update_bin="/usr/local/bin/update-pib"
    local source_file="$BACKEND_DIR/setup/update-pib.sh"

    # If update-pib exists but is not a symlink
    if [[ -n "$update_bin" && ! -L "$update_bin" ]]; then
        print WARN "update-pib is not installed as a symlink. Fixing..."
        sudo rm -f "$update_bin"
        sudo ln -s "$source_file" "$update_bin"
        sudo chmod +x "$source_file"
        print INFO "Symlink re-created: $update_bin -> $source_file"
    fi
}

# 4 GiB images never install ollama (variant gate in setup-pib.sh). A missing
# unit must not fail this update and must not create the drop-in. When the
# unit exists, the shared helper restarts it only if the drop-in changed:
# without that restart the daemon keeps its old bind until reboot. The
# restart choice and the idempotency rule are in
# installation_scripts/ollama_listen.sh.
function ensure_ollama_for_update() {
    local helper="${PIB_OLLAMA_LISTEN_HELPER:-$BACKEND_DIR/setup/installation_scripts/ollama_listen.sh}"
    if [ ! -f "$helper" ]; then
        print ERROR "ollama: listen helper not found at ${helper}"
        return 1
    fi
    # shellcheck source=installation_scripts/ollama_listen.sh
    source "$helper"
    if ! ollama_unit_installed; then
        print INFO "ollama: not installed; listen drop-in left unset"
        return 0
    fi
    ensure_ollama_listen_dropin restart
}

function ensure_host_ip() {
    local primary="/etc/pib_host_ip"
    local legacy="$BACKEND_DIR/pib_api/flask/host_ip.txt"
    local dispatcher_script="/etc/NetworkManager/dispatcher.d/99-update-ip.sh"
    local value

    if [ -s "$primary" ] && [ -s "$legacy" ]; then
        print INFO "host IP files already exist, skipping dispatcher setup"
        return 0
    fi

    print INFO "host IP file missing, ensuring dispatcher script exists..."
    sudo mkdir -p "$(dirname "$dispatcher_script")" "$(dirname "$primary")" "$(dirname "$legacy")"
    sudo tee "$dispatcher_script" > /dev/null << 'EOF'
#!/bin/bash
PRIMARY=/etc/pib_host_ip
LEGACY=/home/pib/app/pib-backend/pib_api/flask/host_ip.txt
IP=$(ip -4 route get 1 2>/dev/null | awk '{for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit }}')
if [[ ! "$IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  exit 0
fi
for outfile in "$PRIMARY" "$LEGACY"; do
  current=""
  if [[ -f "$outfile" ]]; then
    current=$(tr -d '[:space:]' < "$outfile")
  fi
  if [[ "$IP" != "$current" ]]; then
    mkdir -p "$(dirname "$outfile")"
    printf '%s\n' "$IP" > "$outfile"
  fi
done
EOF
    sudo chmod 755 "$dispatcher_script"
    sudo bash "$dispatcher_script"
    if [ ! -s "$primary" ]; then
        print ERROR "host IP was not written to ${primary}"
        return 1
    fi
    value="$(tr -d '[:space:]' < "$primary")"
    print INFO "host IP recorded in ${primary}: ${value}"
}

function update_backend() {
    if [ -d "$BACKEND_DIR" ]; then
        print INFO "Updating backend:"
        cd "$BACKEND_DIR" || { print ERROR "Cannot get to $BACKEND_DIR"; exit 1; }
        git pull --ff-only origin main || { print ERROR "backend git pull error"; exit 1; }
        # Ensure that the IP dispatcher script is set up so the IP display works correctly
        ensure_host_ip
        # After the pull, before flask is recreated: the new container probes
        # Ollama as soon as it starts, so the listen drop-in has to be in
        # effect first. A missing ollama unit returns success.
        ensure_ollama_for_update
        # Inject the release tag into the flask-app image, exactly as docs/RELEASE.md and
        # setup/update_runner.sh do. The export is required too: the following `up --build`
        # interpolates ${APP_VERSION:-v0.6.2} and would otherwise rebuild the image with that
        # fallback, so the compose call needs `sudo -E` to keep the value.
        if APP_VERSION="$(release_tag_here)"; then
            export APP_VERSION
            print INFO "flask-app APP_VERSION=$APP_VERSION"
            sudo -E docker compose --profile all build --build-arg "APP_VERSION=$APP_VERSION" flask-app || { print ERROR "docker compose backend build error"; exit 1; }
        else
            unset APP_VERSION
            print WARN "no release tag on the checkout; flask-app keeps the compose-file APP_VERSION fallback"
        fi
        sudo -E docker compose --profile all up --force-recreate --build -d || { print ERROR "docker compose backend build error"; exit 1; }
    else
        print ERROR "Directory $BACKEND_DIR does not exist"
        exit 1 
    fi
}

function update_frontend() {
    if [ -d "$FRONTEND_DIR" ]; then
        print INFO "Updating frontend:"
        cd "$FRONTEND_DIR" || { print ERROR "Cannot get to $FRONTEND_DIR"; exit 1; }
        git pull --recurse-submodules || { print ERROR "frontend git pull error"; exit 1; }
        git submodule update --init --recursive || { print ERROR "frontend submodule update error"; exit 1; }
        # Same tag injection as the backend: cerebra's Dockerfile writes APP_VERSION into the
        # footer, and `up --build` would otherwise re-interpolate the compose-file fallback.
        if APP_VERSION="$(release_tag_here)"; then
            export APP_VERSION
            print INFO "cerebra APP_VERSION=$APP_VERSION"
            sudo -E docker compose build --build-arg "APP_VERSION=$APP_VERSION" angular-app || { print ERROR "docker compose frontend build error"; exit 1; }
        else
            unset APP_VERSION
            print WARN "no release tag on the checkout; cerebra keeps the compose-file APP_VERSION fallback"
        fi
        sudo -E docker compose up --force-recreate --build -d || { print ERROR "docker compose frontend build error"; exit 1; }
    else
        print ERROR "Directory $FRONTEND_DIR does not exist"
        exit 1 
    fi
}

function update_database() {
    print INFO "Updating database:"

    UPDATE_DB_SCRIPT="$BACKEND_DIR/pib_api/flask/update_db.py"

    if [ -f "$UPDATE_DB_SCRIPT" ]; then
        sudo docker compose exec flask-app python3 -m update_db || { print ERROR "Failed to run $UPDATE_DB_SCRIPT"; exit 1; }
    else
        print ERROR "Python script $UPDATE_DB_SCRIPT not found"
        exit 1
    fi
}

function update_docker_cleaner() {
    if [ -f "$BACKEND_DIR/setup/setup_files/docker_cleaner.service" ]; then
        sudo cp "$BACKEND_DIR/setup/setup_files/docker_cleaner.service" /etc/systemd/system/
        sudo systemctl daemon-reload
        sudo systemctl restart docker_cleaner.service
        print SUCCESS "Docker container cleanup service updated"
    else
        print ERROR "Docker cleaner service file does not exist"
    fi

    sudo usermod -aG docker pib 
}

# Check correct user
if [ "$(whoami)" != "$DEFAULT_USER" ]; then
    print INFO "Run this as user: $DEFAULT_USER"
    exit 1
fi

# Setup sudo without password (temporary)
print INFO "Setting up temporary sudo access..."
sudo bash -c "echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/$DEFAULT_USER"
sudo chmod 0440 "/etc/sudoers.d/$DEFAULT_USER"

# Start logging
print INFO "Logging to $LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1
print INFO "Update started: "

ensure_symlink_self

update_backend

update_frontend

update_database

update_docker_cleaner


# Cleanup
print INFO "Cleaning up:"
sudo rm -v "/etc/sudoers.d/$DEFAULT_USER"

print SUCCESS "Update successful."
