#!/bin/bash
#
# This script installs all necessary software and sets required configurations for running pib
#
# It assumes:
#   - that Ubuntu Desktop 22.04 is installed
#   - the default-user "pib" is executing it
#

# Codes for "echo -e" output text formatting (Exported constants made available for all subshells)
export RED_TEXT_COLOR="\e[31m"
export YELLOW_TEXT_COLOR="\e[33m"
export GREEN_TEXT_COLOR="\e[32m"
export CYAN_TEXT_COLOR="\e[36m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

show_help() 
{
	echo -e "The setup-pib.sh script has two execution modes:"
	echo -e "(normal mode and development mode)""$NEW_LINE"
	echo -e "$CYAN_TEXT_COLOR""Normal mode (don't add any arguments or options)""$RESET_TEXT_COLOR"
	echo -e "Example: ./setup-pib""$NEW_LINE"
	echo -e "$CYAN_TEXT_COLOR""Development mode (specify the branches you want to install)""$RESET_TEXT_COLOR"

	echo -e "You can either use the short or verbose command versions:"
	echo -e "-f=YourBranchName or --frontend-branch=YourBranchName"
	echo -e "-b=YourBranchName or --backend-branch=YourBranchName"
	echo -e "-p=YourBranchName or --pib-blockly-branch=YourBranchName"

	echo -e "$NEW_LINE""Examples:"
	echo -e "    ./setup-pib -b=main -f=PR-566"
    echo -e "    ./setup-pib --backend-branch=main --frontend-branch=PR-566 --pib-blockly-branch=PR-develop"
	
	exit
}

echo -e "$NEW_LINE""$YELLOW_TEXT_COLOR""-- Checking user input option syntax --""$RESET_TEXT_COLOR""$NEW_LINE"

# Github repo origins and branches (branch values will be replaced in dev-mode)
export FRONTEND_REPO="https://github.com/pib-rocks/cerebra.git"
export BACKEND_REPO="https://github.com/pib-rocks/pib-backend.git"
export frontend_branch="main"
export backend_branch="main"

# Iterate through all user input parameters
export is_dev_mode=false
while [ $# -gt 0 ]; do
	case "$1" in
		# Assign default and feature branches for dev-mode
		-f=* | --frontend-branch=*)
			is_dev_mode=true
			frontend_branch="${1#*=}"
			;;
		-b=* | --backend-branch=*)
			is_dev_mode=true
			backend_branch="${1#*=}"
			;;
		-h | --help)
			show_help
			;;
		*)
			echo -e "$RED_TEXT_COLOR""Invalid input options. Here are some infos about the possible script inputs:""$RESET_TEXT_COLOR""$NEW_LINE"
			show_help
	esac
	shift
done

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- User input options syntax correct --""$RESET_TEXT_COLOR""$NEW_LINE"

# Default ubuntu paths
export DEFAULT_USER="pib"
export USER_HOME="/home/$DEFAULT_USER"

# We want the user pib to setup things without password (sudo without password)
# Yes, we are aware of the security-issues..
echo "Hello pib! We start the setup by allowing you permanently to run commands with admin-privileges."
if [[ "$(id)" == *"(sudo)"* ]]; then
	echo "For this change please enter your password..."
	sudo bash -c "echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$DEFAULT_USER"
else
	echo "For this change please enter the root-password. It is most likely just your normal one..."
	su root bash -c "usermod -aG sudo $DEFAULT_USER ; echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$DEFAULT_USER"
fi

# Redirect console output to a log file
LOG_FILE="$USER_HOME/setup-pib.log"
exec > >(tee -a "$LOG_FILE") 2>&1

# Disable IPv6
echo -e "$NEW_LINE""$YELLOW_TEXT_COLOR""-- Disabling IPv6  --""$RESET_TEXT_COLOR""$NEW_LINE"

sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1
sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Disabled IPv6 --""$RESET_TEXT_COLOR""$NEW_LINE"


# Delete unnecessary apps
echo -e "$NEW_LINE""$YELLOW_TEXT_COLOR""-- Removing unused default software packages --""$RESET_TEXT_COLOR""$NEW_LINE"

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
	sudo apt-get -y purge $installed_packages_to_be_removed
	sudo apt-get autoclean
fi
echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Removal of unused default software packages completed --""$RESET_TEXT_COLOR""$NEW_LINE"

# Refresh the linux packages list (sometimes necessary for packages that are required in the installion scripts)
sudo apt update
# These packages are installed seperately, since the installation scripts are dependent on them
sudo apt-get install -y git curl

# In dev-mode check if the specified branches exist for each repo
if [ "$is_dev_mode" = true ] 
then
	echo -e "$NEW_LINE""$YELLOW_TEXT_COLOR""-- Checking if user-specified branches exist --""$RESET_TEXT_COLOR""$NEW_LINE"

	if git ls-remote --exit-code --heads "$FRONTEND_REPO" "$frontend_branch" >/dev/null 2>&1; then
		echo -e "$CYAN_TEXT_COLOR""Frontend repo branch used: ""$RESET_TEXT_COLOR""$frontend_branch"
	else
		echo -e "$RED_TEXT_COLOR""Frontend repo: no branch called $frontend_branch was found""$RESET_TEXT_COLOR""$NEW_LINE"
		show_help
	fi

	if git ls-remote --exit-code --heads "$BACKEND_REPO" "$backend_branch" >/dev/null 2>&1; then
		echo -e "$CYAN_TEXT_COLOR""Backend repo branch used: ""$RESET_TEXT_COLOR""$backend_branch"
	else
		echo -e "$RED_TEXT_COLOR""Backend repo: no branch called $backend_branch was found""$RESET_TEXT_COLOR""$NEW_LINE"
		show_help
	fi

	echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Check for user-specified branches completed --""$RESET_TEXT_COLOR""$NEW_LINE"
fi

# Create temporary directory for installation files and define path constants
export TEMPORARY_SETUP_DIR=""
TEMPORARY_SETUP_DIR=$(mktemp --directory /var/tmp/pib-temp.XXX)
export FRONTEND_DIR="$TEMPORARY_SETUP_DIR/frontend"
export BACKEND_DIR="$TEMPORARY_SETUP_DIR/backend"
export SETUP_DIR="$BACKEND_DIR/setup"
export SETUP_FILES="$SETUP_DIR/setup_files"
export INSTALLATION_SCRIPTS="$SETUP_DIR/installation_scripts"
export PIB_API_SETUP_DIR="$BACKEND_DIR/pib_api"
export PIB_BLOCKLY_SETUP_DIR="$BACKEND_DIR/pib_blockly"
export UPDATE_TARGET_DIR="/usr/bin"

# clone frontend repo and initialize submodules
git clone -b "$frontend_branch" "$FRONTEND_REPO" "$FRONTEND_DIR"
cd "$FRONTEND_DIR"
git submodule update --init

# clone backend repo and initialize submodules
git clone -b "$backend_branch" "$BACKEND_REPO" "$BACKEND_DIR"
cd "$BACKEND_DIR"
git submodule update --init

# create working directory for ros
export ROS_WORKING_DIR="$USER_HOME/ros_working_dir"
mkdir "$ROS_WORKING_DIR"

# The following scripts are sourced into the same shell as this script,
# allowing them to acces all variables and context
# Check system variables
source "$INSTALLATION_SCRIPTS/check_system_variables.sh"
# Install system packages
source "$INSTALLATION_SCRIPTS/install_system_packages.sh"
# Install tinkerforge
source "$INSTALLATION_SCRIPTS/install_tinkerforge.sh"
# Install Cerebra
source "$INSTALLATION_SCRIPTS/install_cerebra.sh"
# Install pib ros-packages
source "$INSTALLATION_SCRIPTS/setup_packages.sh"
# Adjust system settings
source "$INSTALLATION_SCRIPTS/set_system_settings.sh"
# prepares JSON-Server
source "$INSTALLATION_SCRIPTS/prepare_json_server.sh"
# Install pib-blockly-server
source "$INSTALLATION_SCRIPTS/install_pib_blockly_server.sh"

# install update-pip
sudo cp "$SETUP_DIR/update-pib.sh" "$UPDATE_TARGET_DIR/update-pib"
sudo chmod 755 "$UPDATE_TARGET_DIR/update-pib"

# Get ros_config
cp "$SETUP_FILES/ros_config.sh" "$ROS_WORKING_DIR/ros_config.sh"

# Setup system to start Cerebra and ROS2 at boot time
# Create boot script for ros_bridge_server
cp "$SETUP_FILES/ros_cerebra_boot.sh" "$ROS_WORKING_DIR/ros_cerebra_boot.sh"
sudo chmod 700 $ROS_WORKING_DIR/ros_cerebra_boot.sh

# Create JSON Server boot script
cp "$SETUP_FILES/start_json_server.sh" $USER_HOME
sudo chmod 700 $USER_HOME/start_json_server.sh

# Create service which starts ros and cerebra by system boot
cp "$SETUP_FILES/ros_cerebra_boot.service" "$ROS_WORKING_DIR/ros_cerebra_boot.service"
sudo chmod 700 $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo mv $ROS_WORKING_DIR/ros_cerebra_boot.service /etc/systemd/system

# Enable new services
sudo systemctl daemon-reload
sudo systemctl enable ros_cerebra_boot.service --now
# Enable and start ssh server
sudo systemctl enable ssh --now

# Download animated pib eyes
cp "$SETUP_FILES/pib-eyes-animated.gif" "$USER_HOME/Desktop/pib-eyes-animated.gif"

# Move log file to temporary setup folder
mv "$LOG_FILE" "$TEMPORARY_SETUP_DIR"

echo -e "$NEW_LINE""Congratulations! The setup completed succesfully!"
echo -e "$NEW_LINE""Please restart the system to apply changes..."