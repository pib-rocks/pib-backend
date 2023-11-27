#!/bin/bash
#
# This script installs all necessary software and sets required configurations for running pib
#
# It assumes:
#   - that Ubuntu Desktop 22.04 is installed
#   - the default-user "pib" is executing it
#

# Codes for "echo -e" output text formatting, made available to all subshells
export RED_TEXT_COLOR="\e[31m"
export YELLOW_TEXT_COLOR="\e[33m"
export GREEN_TEXT_COLOR="\e[32m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

# Exit codes for error detection, made available to all subshells
export SUCCESS_STATUS=0
export INPUT_OUTPUT_ERROR_STATUS=5
export FAILED_SUBSCRIPT_STATUS=254
export FAILED_CHECK_STATUS=255

# Boolean variables for checks
export TRUE="true"
export FALSE="false"

# Variables for user input options and arguments
export FIRST_USER_INPUT=$1
export SECOND_USER_INPUT=$2
export THIRD_USER_INPUT=$3
export is_dev_mode="$FALSE"
export user_default_branch=""
export user_feature_branch=""

UBUNTU_VERSION=$(lsb_release -rs)
DEFAULT_USER="pib"
export USER_HOME="/home/$DEFAULT_USER"
export ROS_WORKING_DIR="$USER_HOME/ros_working_dir"
export DEFAULT_NGINX_DIR="/etc/nginx"
export DEFAULT_NGINX_HTML_DIR="$DEFAULT_NGINX_DIR/html"
export NGINX_CONF_FILE="nginx.conf"
export NGINX_CONF_FILE_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/nginx.conf"
#
export CEREBRA_ARCHIVE_URL_PATH="https://pib.rocks/wp-content/uploads/pib_data/cerebra-latest.zip"
export CEREBRA_ARCHIVE_NAME="cerebra-latest.zip"
#
ROS_CONFIG_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_config.sh"
#
ROS_CEREBRA_BOOT_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_cerebra_boot.sh"
ROS_CEREBRA_BOOT_SERVICE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_cerebra_boot.service"
#
ROS_PACKAGES_LINK="https://raw.githubusercontent.com/pib-rocks/ros-packages/main/packages-set-up.sh"
ROS_UPDATE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/update-pib.sh"
#

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

# TODO/TBE: rename _file to _script
# TODO/TBE: make path constants available to subscripts and move to beginning of script
export THIS_DIRECTORY=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
export INSTALLATION_FILES_DIR="$THIS_DIRECTORY""/installation_files"

# Check the user inputs (options and arguments) for the dev mode
readonly USER_ARGUMENTS_CHECK_FILE="$INSTALLATION_FILES_DIR"/"check_input_arguments.sh"
chmod 755 "$USER_ARGUMENTS_CHECK_FILE"
source "$USER_ARGUMENTS_CHECK_FILE"

# Run the script for checking the system variables
readonly SYSTEM_VARIABLES_CHECK_FILE="$INSTALLATION_FILES_DIR"/"system_variable_checks.sh"
chmod 755 "$SYSTEM_VARIABLES_CHECK_FILE"
"$SYSTEM_VARIABLES_CHECK_FILE"

# Exit this script if any other exit code than 'successfull' was returned by the previous script
if [ $? -ne "$SUCCESS_STATUS" ]; then
	exit "$FAILED_SUBSCRIPT_STATUS"
fi

# Run the script for installing system packages
readonly SYSTEM_INSTALLATIONS_FILE="$INSTALLATION_FILES_DIR"/"system_packages_installation.sh"
chmod 755 "$SYSTEM_INSTALLATIONS_FILE"
"$SYSTEM_INSTALLATIONS_FILE"

# Run the script for installing python packages
readonly PYTHON_INSTALLATIONS_FILE="$INSTALLATION_FILES_DIR"/"python_packages_installation.sh"
chmod 755 "$PYTHON_INSTALLATIONS_FILE"
"$PYTHON_INSTALLATIONS_FILE"


# TODO/TBE: Check if this can be moved to installations script
# Install rosbridge-server
echo 'Install rosbridge-server...'
sudo apt install -y ros-humble-rosbridge-server

# Run the script for installing tinkerforge
readonly TINKERFORGE_INSTALLATION_FILE="$INSTALLATION_FILES_DIR"/"install_tinkerforge.sh"
chmod 755 "$TINKERFORGE_INSTALLATION_FILE"
"$TINKERFORGE_INSTALLATION_FILE"

# Run the script for installing Cerebra
readonly CEREBRA_INSTALLATION_FILE="$INSTALLATION_FILES_DIR"/"install_cerebra.sh"
chmod 755 "$CEREBRA_INSTALLATION_FILE"
"$CEREBRA_INSTALLATION_FILE"

# Run the script for installing all ros-packages
readonly ROS_PACKAGE_INSTALLATION_FILE="$INSTALLATION_FILES_DIR"/"setup_packages.sh"
chmod 755 "$ROS_PACKAGE_INSTALLATION_FILE"
"$ROS_PACKAGE_INSTALLATION_FILE"

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