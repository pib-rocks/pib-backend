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
export SUCCESS_STATUS=0
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

# Todo: Check if this still works and is necessary
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

# Create temporary folder for installation files #TODO: Change branch to main/develop
export SETUP_PIB_BRANCH="PR-368"
export TEMPORARY_SETUP_DIR="temp"
export INSTALLATION_FILES_DIR=""
mkdir "$TEMPORARY_SETUP_DIR"

# Get setup files needed for the pib installation
wget -O get_setup_files.sh "https://raw.githubusercontent.com/pib-rocks/setup-pib/""$SETUP_PIB_BRANCH""/installation_files/get_setup_files.sh"
readonly GET_SETUP_FILES_SCRIPT="get_setup_files.sh"
chmod 755 "$GET_SETUP_FILES_SCRIPT"
source "$GET_SETUP_FILES_SCRIPT"


# Variables for user input options and arguments
export FIRST_USER_INPUT=$1
export SECOND_USER_INPUT=$2
export THIRD_USER_INPUT=$3
export is_dev_mode="$FALSE"
export user_default_branch=""
export user_feature_branch=""

# Check the user inputs (options and arguments) for the dev mode. Run it in the same shell as this script.
readonly CHECK_USER_INPUT_SCRIPT="$INSTALLATION_FILES_DIR"/"check_user_input.sh"
chmod 755 "$CHECK_USER_INPUT_SCRIPT"
source "$CHECK_USER_INPUT_SCRIPT"

# Run the script for checking the system variables
readonly CHECK_SYSTEM_VARIABLES_SCRIPT="$INSTALLATION_FILES_DIR"/"check_system_variables.sh"
chmod 755 "$CHECK_SYSTEM_VARIABLES_SCRIPT"
source "$CHECK_SYSTEM_VARIABLES_SCRIPT"

# Git is installed seperately, since the CHECK_BRANCHES_SCRIPT is dependent on it
sudo apt-get install -y git 

# Variables for github branch checking:
# Github repo origin links
readonly SETUP_PIB_ORIGIN="https://github.com/pib-rocks/setup-pib.git"
readonly PIB_API_ORIGIN="https://github.com/pib-rocks/pib-api.git"
readonly ROS_PACKAGES_ORIGIN="https://github.com/pib-rocks/ros-packages.git"
readonly DATATYPES_ORIGIN="https://github.com/pib-rocks/datatypes.git"
readonly MOTORS_ORIGIN="https://github.com/pib-rocks/motors.git"
readonly OAK_D_LITE_ORIGIN="https://github.com/pib-rocks/ros2_oak_d_lite.git"
readonly VOICE_ASSISTANT_ORIGIN="https://github.com/pib-rocks/voice-assistant.git"

# Create an associative array (=map). This will be filled with repo-origin-branch-name pairs
declare -A repo_map

# Check the user inputs (options and arguments) for the dev mode. Run it in the same shell as this script.
readonly CHECK_BRANCHES_SCRIPT="$INSTALLATION_FILES_DIR"/"check_github_branches.sh"
chmod 755 "$CHECK_BRANCHES_SCRIPT"
source "$CHECK_BRANCHES_SCRIPT"

# Run the script for installing system packages
readonly INSTALL_SYSTEM_PACKAGES_SCRIPT="$INSTALLATION_FILES_DIR"/"install_system_packages.sh"
chmod 755 "$INSTALL_SYSTEM_PACKAGES_SCRIPT"
source "$INSTALL_SYSTEM_PACKAGES_SCRIPT"

# Run the script for installing python packages
readonly INSTALL_PYTHON_PACKAGES_SCRIPT="$INSTALLATION_FILES_DIR"/"install_python_packages.sh"
chmod 755 "$INSTALL_PYTHON_PACKAGES_SCRIPT"
source "$INSTALL_PYTHON_PACKAGES_SCRIPT"


# TODO/TBE: Check if this can be moved to installations script
# Install rosbridge-server
echo 'Install rosbridge-server...'
sudo apt install -y ros-humble-rosbridge-server

# Run the script for installing tinkerforge
readonly INSTALL_TINKERFORGE_SCRIPT="$INSTALLATION_FILES_DIR"/"install_tinkerforge.sh"
chmod 755 "$INSTALL_TINKERFORGE_SCRIPT"
source "$INSTALL_TINKERFORGE_SCRIPT"

# Run the script for installing Cerebra
readonly INSTALL_CEREBRA_SCRIPT="$INSTALLATION_FILES_DIR"/"install_cerebra.sh"
chmod 755 "$INSTALL_CEREBRA_SCRIPT"
source "$INSTALL_CEREBRA_SCRIPT"

# Run the script for installing all ros-packages. Run it in the same shell as this script.
readonly SETUP_ROS_PACKAGES_SCRIPT="$INSTALLATION_FILES_DIR"/"setup_packages.sh"
chmod 755 "$SETUP_ROS_PACKAGES_SCRIPT"
source "$SETUP_ROS_PACKAGES_SCRIPT"


# Links for github direct downloads, from selected branch
readonly ROS_UPDATE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/update-pib.sh"
readonly ROS_CONFIG_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/ros_config.sh"
readonly ROS_CEREBRA_BOOT_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/ros_cerebra_boot.sh"
readonly ROS_CEREBRA_BOOT_SERVICE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/ros_cerebra_boot.service"

# install update-pip
if [ -f $USER_HOME/update-pib.sh ]; then
  sudo rm update-pib.sh
fi
wget -O update-pib.sh $ROS_UPDATE_LINK
sudo chmod 777 update-pib.sh
echo "if [ -f /home/pib/update-pib.sh ]; then
        alias update-pib='/home/pib/update-pib.sh'
      fi
" >> $USER_HOME/.bashrc

# set permissions
cd $ROS_WORKING_DIR
colcon build
sudo chmod -R 777 $ROS_WORKING_DIR/build
sudo chmod -R 777 $ROS_WORKING_DIR/install
sudo chmod -R 777 $ROS_WORKING_DIR/log

# Get config
curl $ROS_CONFIG_LINK -L --output $ROS_WORKING_DIR/ros_config.sh

# Setup system to start Cerebra and ROS2 at boot time
# Create boot script for ros_bridge_server
curl $ROS_CEREBRA_BOOT_LINK -L --output $ROS_WORKING_DIR/ros_cerebra_boot.sh
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.sh

# Create service which starts ros and cerebra by system boot
curl $ROS_CEREBRA_BOOT_SERVICE_LINK -L --output $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo mv $ROS_WORKING_DIR/ros_cerebra_boot.service /etc/systemd/system

# Clean-up: remove unnecessary .zip directories
rm -r phpliteadmin_v1_9_9_dev.zip
rm -r cerebra-latest.zip

# Enable new services
sudo systemctl daemon-reload
sudo systemctl enable ros_cerebra_boot.service
# Enable and start ssh server
sudo systemctl enable ssh --now

echo -e '\nCongratulations! The setup completed succesfully!'
echo -e '\nPlease restart the system to apply changes...'