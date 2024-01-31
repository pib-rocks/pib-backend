#!/bin/bash
#
# This script installs all necessary software and sets required configurations for running pib
#
# It assumes:
#   - that Ubuntu Desktop 22.04 is installed
#   - the default-user "pib" is executing it
#

# Exported variables for all subshells: Codes for "echo -e" output text formatting
export RED_TEXT_COLOR="\e[31m"
export YELLOW_TEXT_COLOR="\e[33m"
export GREEN_TEXT_COLOR="\e[32m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

# Exported variables for all subshells: Exit codes for error detection
export INPUT_OUTPUT_ERROR_STATUS=5
export FAILED_SUBSCRIPT_STATUS=254
export FAILED_CHECK_STATUS=255

# Exported variables for all subshells: Boolean constants for checks
export TRUE="true"
export FALSE="false"

# Default ubuntu paths
export DEFAULT_USER="pib"
export USER_HOME="/home/$DEFAULT_USER"
export ROS_WORKING_DIR="$USER_HOME/ros_working_dir"
mkdir "$ROS_WORKING_DIR"
mkdir "$ROS_WORKING_DIR"/src
mkdir "$USER_HOME/cerebra_programs"

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

# Activate automatic login settings via regex
sudo sed -i '/#  AutomaticLogin/{s/#//;s/user1/pib/}' /etc/gdm3/custom.conf

# Disabling power saving settings
gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.settings-daemon.plugins.power power-saver-profile-on-low-battery false
gsettings set org.gnome.settings-daemon.plugins.power ambient-enabled false
gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'

# Create temporary directory for installation files
export TEMPORARY_SETUP_DIR="$(mktemp --directory /tmp/pib-temp.XXX)"

# Installation folder will be created inside the temporary directory.
# The folder name is dependend on the corresponding branch, so it's defined after the branch check.
export installation_files_dir=""

# This variable is specifically for downloading the installation scripts from the setup repo
# These files are left out of the dynamic branch selection, since they are a prerequisite for the check itself
# If you want to get the installation scripts from a specific branch, you need to change this variable manually
export SETUP_PIB_BRANCH="main"

# Refresh the linux packages list (sometimes necessary for packages that are required in the installion scripts)
sudo apt update

# These packages are installed seperately, since the installation scripts are dependent on them
sudo apt-get install -y git curl

# Get setup files needed for the pib software installation
readonly GET_SETUP_FILES_SCRIPT_NAME="get_setup_files.sh"
readonly GET_SETUP_FILES_SCRIPT="$TEMPORARY_SETUP_DIR""/$GET_SETUP_FILES_SCRIPT_NAME"
curl "https://raw.githubusercontent.com/pib-rocks/setup-pib/""$SETUP_PIB_BRANCH""/installation_scripts/""$GET_SETUP_FILES_SCRIPT_NAME" --location --output "$GET_SETUP_FILES_SCRIPT" 
chmod 755 "$GET_SETUP_FILES_SCRIPT"
source "$GET_SETUP_FILES_SCRIPT"

# Variables for user input options and arguments
export FIRST_USER_INPUT=$1
export SECOND_USER_INPUT=$2
export THIRD_USER_INPUT=$3
export is_dev_mode="$FALSE"
export user_default_branch=""
export user_feature_branch=""

# Variables for github branch checking:
# Github repo origin URLs
readonly SETUP_PIB_ORIGIN="https://github.com/pib-rocks/setup-pib.git"
readonly PIB_API_ORIGIN="https://github.com/pib-rocks/pib-api.git"
readonly ROS_PACKAGES_ORIGIN="https://github.com/pib-rocks/ros-packages.git"
readonly DATATYPES_ORIGIN="https://github.com/pib-rocks/datatypes.git"
readonly MOTORS_ORIGIN="https://github.com/pib-rocks/motors.git"
readonly OAK_D_LITE_ORIGIN="https://github.com/pib-rocks/ros2_oak_d_lite.git"
readonly VOICE_ASSISTANT_ORIGIN="https://github.com/pib-rocks/voice-assistant.git"

# Create an associative array (=map). This will be filled with repo-origin branch-name pairs in the check_github_branches.sh script
declare -A repo_map

# The following scripts are sourced into the same shell as this script,
# allowing them to acces all variables and context
# Check user inputs (options and arguments) for dev mode
source "$installation_files_dir""/check_user_input.sh"
# Check system variables
source "$installation_files_dir""/check_system_variables.sh"
# Check which github branches are available based on user input
source "$installation_files_dir""/check_github_branches.sh"
# Install system packages
source "$installation_files_dir""/install_system_packages.sh"
# Install python packages
source "$installation_files_dir""/install_python_packages.sh"
# Install tinkerforge
source "$installation_files_dir""/install_tinkerforge.sh"
# Install Cerebra
source "$installation_files_dir""/install_cerebra.sh"
# Install pib ros-packages
source "$installation_files_dir""/setup_packages.sh"


# Github direct download URLs, from the selected branch
readonly ROS_UPDATE_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/update-pib.sh"
readonly ROS_CONFIG_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/ros_config.sh"
readonly ROS_CEREBRA_BOOT_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/ros_cerebra_boot.sh"
readonly ROS_CEREBRA_BOOT_SERVICE_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/ros_cerebra_boot.service"

# install update-pip
if [ -f "$USER_HOME""/update-pib.sh" ]; then
  sudo rm update-pib.sh
fi
curl "$ROS_UPDATE_URL" --location --output "$USER_HOME""/update-pib.sh"
sudo chmod 777 update-pib.sh
echo "if [ -f /home/pib/update-pib.sh ]; then
        alias update-pib='/home/pib/update-pib.sh'
      fi
" >> $USER_HOME/.bashrc

# Download ros_config
curl "$ROS_CONFIG_URL" --location --output "$ROS_WORKING_DIR/ros_config.sh"  

# Setup system to start Cerebra and ROS2 at boot time
# Create boot script for ros_bridge_server
curl "$ROS_CEREBRA_BOOT_URL" --location --output  "$ROS_WORKING_DIR/ros_cerebra_boot.sh" 
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.sh

# Create service which starts ros and cerebra by system boot
curl "$ROS_CEREBRA_BOOT_SERVICE_URL" --location --output "$ROS_WORKING_DIR/ros_cerebra_boot.service" 
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo mv $ROS_WORKING_DIR/ros_cerebra_boot.service /etc/systemd/system

# Enable new services
sudo systemctl daemon-reload
sudo systemctl enable ros_cerebra_boot.service
# Enable and start ssh server
sudo systemctl enable ssh --now

echo -e "$NEW_LINE""Congratulations! The setup completed succesfully!"
echo -e "$NEW_LINE""Please restart the system to apply changes..."