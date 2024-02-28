#!/bin/bash
#
# This script assumes:
#   - that Ubuntu Desktop 22.04.2 is installed
#   - that setup-pib was already executed
#   - the default-user "pib" is executing it
#

DEFAULT_USER="pib"
USER_HOME="/home/$DEFAULT_USER"
ROS_WORKING_DIR="$USER_HOME/ros_working_dir"

DEFAULT_NGINX_DIR="/etc/nginx"
DEFAULT_NGINX_HTML_DIR="$DEFAULT_NGINX_DIR/html"

CEREBRA_ARCHIVE_URL_PATH="https://pib.rocks/wp-content/uploads/pib_data/cerebra-latest.zip"
CEREBRA_ARCHIVE_NAME="cerebra-latest.zip"

NGINX_CONF_FILE="nginx.conf"
NGINX_CONF_FILE_URL="https://raw.githubusercontent.com/pib-rocks/pib-backend/main/setup/setup_files/nginx.conf"

export TEMPORARY_SETUP_DIR="$(mktemp --directory /tmp/pib-temp.XXX)"

# We make sure that this script is run by the user "pib"
if [ "$(whoami)" != "pib" ]; then
        echo "This script must be run as user: pib"
        exit 255
fi

# We want the user pib to setup things without password (sudo without password)
# Yes, we are aware of the security-issues..
if [[ "$(id)" == *"(sudo)"* ]]; then
        echo "For this change please enter your password..."
        sudo bash -c "echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$DEFAULT_USER"
else
        echo "For this change please enter the root-password. It is most likely just your normal one..."
        su root bash -c "usermod -aG sudo $DEFAULT_USER ; echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$DEFAULT_USER"
fi

if [ ! -z "$1" ]; then
	if [ ! "$1" == '-Cerebra' ]; then
		echo 'This script can only be runned with no parameter, to upgrade cerebra and all Packages'
		echo 'or with the parameter "-Cerebra" to update only the frontend.'
	        exit 255
	fi
fi

# Update all packages
sudo apt-get update
sudo apt-get -y upgrade

echo -e '\nClean up the html directory...'
cd $DEFAULT_NGINX_HTML_DIR && sudo -S rm -r *
cd $USER_HOME
# Download Cerebra artifact to the working directory
echo -e '\nDownloading Cerebra application'
curl $CEREBRA_ARCHIVE_URL_PATH -L --output $ROS_WORKING_DIR/$CEREBRA_ARCHIVE_NAME
#
# Unzip cerebra files to nginx
echo -e '\nUnzip cerebra...'
#cd $RASP_TMP_FOLDER
if [ ! -d $DEFAULT_NGINX_HTML_DIR ]; then
        echo 'Path not found: ' + $ROS_WORKING_DIR
        exit 2
fi
sudo unzip $ROS_WORKING_DIR/$CEREBRA_ARCHIVE_NAME -d $DEFAULT_NGINX_HTML_DIR
#
# Setting up nginx to serve Cerebra locally
echo -e '\nDownloading nginx configuration file...'
sudo curl $NGINX_CONF_FILE_URL --output $DEFAULT_NGINX_DIR/$NGINX_CONF_FILE

# Ask the user if he whants to update Cerebra
if [ -z "$1" ]; then
        sudo rm -r $ROS_WORKING_DIR/src
        mkdir $ROS_WORKING_DIR/src
        git clone https://github.com/pib-rocks/pib-backend.git $TEMPORARY_SETUP_DIR
        cp -r $TEMPORARY_SETUP_DIR/ros_packages $ROS_WORKING_DIR/src
	cd $ROS_WORKING_DIR
	colcon build
	sudo chmod -R 777 $ROS_WORKING_DIR/build
	sudo chmod -R 777 $ROS_WORKING_DIR/install
	sudo chmod -R 777 $ROS_WORKING_DIR/log
fi

echo "Checking BrickletsIDs..."
readonly MOTOR_UTILS_DIR="/home/pib/ros_working_dir/src/motors/utils"
readonly PYTHON_UID_SCRIPT_IMPORT="import sys; sys.path.insert(0, '$MOTOR_UTILS_DIR')"

readonly CHANGE_DETECTED=$(python3 -c "$PYTHON_UID_SCRIPT_IMPORT; from update_bricklet_uids import detect_uid_changes; print(detect_uid_changes())")

if [ "$CHANGE_DETECTED" == True ]; then
    while true; do
        read -p "UID changes were detected. Do you want to update your Bricklet-UIDs? (yes/no): " yn
        case $yn in
                [Yy]* ) python3 -c "$PYTHON_UID_SCRIPT_IMPORT; from update_bricklet_uids import update_uids; update_uids()"; break;;
                [Nn]* ) break;;
                * ) echo "Please answer yes or no.";;
        esac
    done
fi

echo "Everything is up to date"