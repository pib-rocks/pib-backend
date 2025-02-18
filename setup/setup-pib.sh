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

# Get Linux distribution version, e.g. (ubuntu) 'jammy', (debian) 'bookworm'
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


function remove_apps() {
  print INFO "Removing unused default software"

  if ! [ "$DISTRIBUTION" == "ubuntu" ]; then
    print INFO "Not using Ubuntu 22.04; skipping removing unused default software"
    return
  fi

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

  git clone --recurse-submodules -b "$BRANCH_BACKEND" $BACKEND "$BACKEND_DIR" || print WARN "pib-backend repository already exists"
  git clone --recurse-submodules -b "$BRANCH_FRONTEND" $FRONTEND "$FRONTEND_DIR" || print WARN "cerebra repository already exists"

  print SUCCESS "Completed cloning repositories to $APP_DIR"
}


# Install update script; move animated eyes, etc.
function move_setup_files() {
  local update_target_dir="/usr/local/bin"
  sudo cp "$BACKEND_DIR/setup/update-pib.sh" "$update_target_dir/update-pib"
  sudo chmod 755 "$update_target_dir/update-pib"
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

function install_BrickV() {
  wget https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.asc -q -O - | sudo tee /etc/apt/trusted.gpg.d/tinkerforge.asc > /dev/null
  echo "deb https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/tinkerforge.list
  sudo apt update
  sudo apt install -y brickv
  sudo apt install python3-tinkerforge #python API Bindings
  print SUCCESS "Installed brick viewer and python API bindings"
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

	echo -e "$NEW_LINE""Examples:"
	echo -e "    ./setup-pib -b=main -f=PR-566"
    echo -e "    ./setup-pib --backend-branch=main --frontend-branch=PR-566"

	exit
}


# ---------- SETUP STARTS FROM HERE -----------

# Reduplicate output to an extra log file as well
LOG_FILE="$HOME/setup-pib.log"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "Hello $USER! We start the setup by allowing you permanently to run commands with admin-privileges. This change is reverted at the end of the setup."
if [[ "$(id)" == *"(sudo)"* ]]; then
	echo "For this change please enter your password..."
	sudo bash -c "echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
else
	echo "For this change please enter the root-password. It is most likely just your normal one..."
	su root bash -c "usermod -aG sudo $USER ; echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
fi


DISTRIBUTION=$(get_distribution) # e.g., 'ubuntu'
export DISTRIBUTION
DIST_VERSION=$(get_dist_version "$DISTRIBUTION")  # e.g., 'jammy'
export DIST_VERSION
print INFO "$DISTRIBUTION $DIST_VERSION"


# VALIDATE CLI ARGUMENTS
BRANCH_BACKEND="main"
BRANCH_FRONTEND="main"
INSTALL_METHOD="docker"
# Check if branch was specified
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
    -h | --help)
      show_help
      ;;
    *)
      print ERROR "invalid input options"
  esac
  shift
done

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

remove_apps || print INFO "Skipped removing default software"
install_system_packages || { print ERROR "failed to install system packages"; return 1; }
disable_power_notification || print ERROR "failed to disable power notifications"
clone_repositories || { print ERROR "failed to clone repositories"; return 1; }
move_setup_files || print ERROR "failed to move setup files"
install_DBbrowser || print ERROR "failed to install DB browser"
install_BrickV || print ERROR "failed to install Brick viewer"
source "$SETUP_INSTALLATION_DIR/set_system_settings.sh" || print INFO "skipped setting system settings"
print INFO "${INSTALL_METHOD}"
if [ "$INSTALL_METHOD" = "legacy" ]; then
  print INFO "Going to install Cerebra locally (LEGACY MODE NOT WORKING ON RASPBERRY PI 5)"
  source "$SETUP_INSTALLATION_DIR/local_install.sh" || print ERROR "failed to install Cerebra locally"
else
  print INFO "Going to install Cerebra via Docker"
  source "$SETUP_INSTALLATION_DIR/docker_install.sh" || print ERROR "failed to install Cerebra via Docker"
fi
cleanup

print SUCCESS "Finished installation, for more information on how to use pib and Cerebra, visit https://pib-rocks.atlassian.net/wiki/spaces/kb/overview?homepageId=65077450"
print SUCCESS "Reboot pib to apply all changes"