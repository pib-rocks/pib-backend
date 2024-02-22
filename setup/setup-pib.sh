#!/bin/bash
#
# This script installs all necessary software and sets required configurations for running pib
#
# It assumes:
#   - that Ubuntu Desktop 22.04 is installed
#   - the default-user "pib" is executing it
#

if [ "$1" == "-h" ] ; then
    echo "Usage: `basename $0` <branch_name> [-h]"
	echo -e "Info: <branch_name> defaults to 'main' if not specified"
    exit 0
fi

# Exported variables for all subshells: Codes for "echo -e" output text formatting
export RED_TEXT_COLOR="\e[31m"
export YELLOW_TEXT_COLOR="\e[33m"
export GREEN_TEXT_COLOR="\e[32m"
export CYAN_TEXT_COLOR="\e[36m"
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

# Create temporary directory for installation files
export TEMPORARY_SETUP_DIR="$(mktemp --directory /var/tmp/pib-temp.XXX)"
export SETUP_DIR=$TEMPORARY_SETUP_DIR/setup
export SETUP_FILES=$TEMPORARY_SETUP_DIR/setup/setup_files

# Redirect console output to a log file
LOG_FILE="$USER_HOME/setup-pib.log"
exec > >(tee -a "$LOG_FILE") 2>&1

# Github Branch selection, if no input param, use main
if [ -z "$1" ]; then
    export BRANCH="main"
else
    export BRANCH="$1"
fi

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

# Refresh the linux packages list (sometimes necessary for packages that are required in the installion scripts)
sudo apt update
# These packages are installed seperately, since the installation scripts are dependent on them
sudo apt-get install -y git curl

# get pib-backend repo
git clone -b $BRANCH https://github.com/pib-rocks/pib-backend.git $TEMPORARY_SETUP_DIR

# Check if clone failed.
# $? checks the exit status of the previous command.
if [ $? -ne 0 ]; then
    echo -e "$red_text_color""Cloning repository failed. Make sure the branch you specified exists."
	exit -1
fi


# The following scripts are sourced into the same shell as this script,
# allowing them to acces all variables and context
# Check system variables
source $SETUP_DIR/installation_scripts/check_system_variables.sh
# Install system packages
source $SETUP_DIR/installation_scripts/install_system_packages.sh
# Install python packages
source $SETUP_DIR/installation_scripts/install_python_packages.sh
# Install tinkerforge
source $SETUP_DIR/installation_scripts/install_tinkerforge.sh
# Install Cerebra
source $SETUP_DIR/installation_scripts/install_cerebra.sh
# Install pib ros-packages
source $SETUP_DIR/installation_scripts/setup_packages.sh
# Adjust system settings
source $SETUP_DIR/installation_scripts/set_system_settings.sh

# install update-pip
cp $SETUP_DIR/update-pib.sh ~/update-pib.sh
sudo chmod 777 ~/update-pib.sh

# Get ros_config
cp $SETUP_FILES/ros_config.sh $ROS_WORKING_DIR/ros_config.sh

# Setup system to start Cerebra and ROS2 at boot time
# Create boot script for ros_bridge_server
cp $SETUP_FILES/ros_cerebra_boot.sh $ROS_WORKING_DIR/ros_cerebra_boot.sh
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.sh

# Create service which starts ros and cerebra by system boot
cp $SETUP_FILES/ros_cerebra_boot.service $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo mv $ROS_WORKING_DIR/ros_cerebra_boot.service /etc/systemd/system

# Enable new services
sudo systemctl daemon-reload
sudo systemctl enable ros_cerebra_boot.service
# Enable and start ssh server
sudo systemctl enable ssh --now

# Download animated pib eyes
cp $SETUP_FILES/pib-eyes-animated.gif ~/Desktop/pib-eyes-animated.gif

# Move log file to temporary setup folder
mv "$LOG_FILE" "$TEMPORARY_SETUP_DIR"

echo -e "$NEW_LINE""Congratulations! The setup completed succesfully!"
echo -e "$NEW_LINE""Please restart the system to apply changes..."