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

function ensure_host_ip() {
    local outfile="$BACKEND_DIR/pib_api/flask/host_ip.txt"
    local dispatcher_script="/etc/NetworkManager/dispatcher.d/99-update-ip.sh"

    if [ ! -f "$outfile" ]; then
        print INFO "host_ip.txt missing, ensuring dispatcher script exists..."

        if [[ ! -f "$dispatcher_script" ]]; then
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
        fi

        print INFO "Manually running dispatcher script to generate host_ip.txt..."
        sudo bash -c "$dispatcher_script wlan0 dhcp4-change"
    else
        print INFO "host-ip.txt already exists, skipping dispatcher setup"
    fi
}

function update_backend() {
    if [ -d "$BACKEND_DIR" ]; then
        print INFO "Updating backend:"
        cd "$BACKEND_DIR" || { print ERROR "Cannot get to $BACKEND_DIR"; exit 1; }
        git pull --recurse-submodules || { print ERROR "backend git pull error"; exit 1; }
        git submodule update --init --recursive || { print ERROR "backend submodule update error"; exit 1; }
        # Ensure that the IP dispatcher script is set up so the IP display works correctly
        ensure_host_ip
        sudo docker compose --profile all up --force-recreate --build -d || { print ERROR "docker compose backend build error"; exit 1; }
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
        sudo docker compose up --force-recreate --build -d || { print ERROR "docker compose frontend build error"; exit 1; }
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

update_database

update_frontend


# Cleanup
print INFO "Cleaning up:"
sudo rm -v "/etc/sudoers.d/$DEFAULT_USER"

print SUCCESS "Update successful."