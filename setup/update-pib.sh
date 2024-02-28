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

FRONTEND_REPO="https://github.com/pib-rocks/cerebra.git"
BACKEND_REPO="https://github.com/pib-rocks/pib-backend.git"

TEMPORARY_SETUP_DIR="$(mktemp --directory /tmp/pib-update-temp.XXX)"
FRONTEND_DIR="$TEMPORARY_SETUP_DIR/frontend"
BACKEND_DIR="$TEMPORARY_SETUP_DIR/backend"
SETUP_FILES="$BACKEND_DIR/setup/setup_files"
LOG_FILE="$USER_HOME/update-pib.log"

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

# Redirect console output to a log file
exec > >(tee -a "$LOG_FILE") 2>&1

# Update all packages
sudo apt-get update
sudo apt-get -y upgrade

git clone "$BACKEND_REPO" "$BACKEND_DIR"
git clone "$FRONTEND_REPO" "$FRONTEND_DIR"


# Update Cerebra
# Install app dependencies and build app
echo -e "Build Cerebra Frontend"
NVM_DIR="/etc/nvm"
source "$NVM_DIR/nvm.sh"
nvm use 18
npm --prefix "$FRONTEND_DIR" install
cd "$FRONTEND_DIR"
ng build --configuration production

if [ ! -d $DEFAULT_NGINX_HTML_DIR ]; then
        echo 'Path not found: ' + $ROS_WORKING_DIR
        exit 2
fi

echo -e '\nClean up the html directory...'
sudo -S rm -r $DEFAULT_NGINX_HTML_DIR/*
sudo mv "$FRONTEND_DIR/dist/cerebra" "$DEFAULT_NGINX_HTML_DIR"
sudo mv "$SETUP_FILES/nginx.conf" "$DEFAULT_NGINX_DIR"


# Update backend
echo -e "Update backend services (ROS packages and Flask API)"

sudo rm -r $ROS_WORKING_DIR/src/* 
cp -r $BACKEND_DIR/ros_packages/* $ROS_WORKING_DIR/src
mv $USER_HOME/flask/pibdata.db $TEMPORARY_SETUP_DIR/pibdata.db
sudo rm -r $USER_HOME/flask 
cp -r $BACKEND_DIR/pib_api/flask $USER_HOME/flask
mv $TEMPORARY_SETUP_DIR/pibdata.db $USER_HOME/flask

# Update Boot services
ROS_CAMERA_BOOT_DIR="$ROS_WORKING_DIR"/src/camera/boot_scripts
ROS_MOTORS_BOOT_DIR="$ROS_WORKING_DIR"/src/motors/boot_scripts
ROS_VOICE_ASSISTANT_BOOT_DIR="$ROS_WORKING_DIR"/src/voice_assistant/boot_scripts
ROS_PROGRAMS_BOOT_DIR="$ROS_WORKING_DIR"/src/programs/boot_scripts

# Boot camera
sudo chmod 755 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.sh"
sudo chmod 755 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service"
sudo mv "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service" /etc/systemd/system
sudo systemctl enable ros_camera_boot.service

# Boot bricklet uid script
sudo chmod 755 "$ROS_WORKING_DIR/src/motors/utils/update_bricklet_uids.py"
sudo chmod 755 "$ROS_MOTORS_BOOT_DIR/bricklet_uid_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/bricklet_uid_boot.service" /etc/systemd/system
sudo systemctl enable bricklet_uid_boot.service

# Boot motor control node
pip install "$ROS_WORKING_DIR/src/motors/pib_motors"
sudo chmod 755 "$ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.sh"
sudo chmod 755 "$ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.service" /etc/systemd/system
sudo systemctl enable ros_motor_control_node_boot.service

# Boot motor current node
sudo chmod 755 "$ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.sh"
sudo chmod 755 "$ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.service" /etc/systemd/system
sudo systemctl enable ros_motor_current_node_boot.service

# Boot voice-assistant
sudo chmod 755 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.sh"
sudo chmod 755 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service"
sudo mv "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service" /etc/systemd/system
sudo systemctl enable ros_voice_assistant_boot.service

# Boot program node
sudo chmod 755 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.sh"
sudo chmod 755 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service"
sudo mv "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service" /etc/systemd/system
sudo systemctl enable ros_program_boot.service

# Boot program proxy node
sudo chmod 755 "$ROS_PROGRAMS_BOOT_DIR/ros_proxy_program_boot.sh"
sudo chmod 755 "$ROS_PROGRAMS_BOOT_DIR/ros_proxy_program_boot.service"
sudo mv "$ROS_PROGRAMS_BOOT_DIR/ros_proxy_program_boot.service" /etc/systemd/system
sudo systemctl enable ros_proxy_program_boot.service


cd $ROS_WORKING_DIR
colcon build
sudo chmod -R 777 $ROS_WORKING_DIR/build
sudo chmod -R 777 $ROS_WORKING_DIR/install
sudo chmod -R 777 $ROS_WORKING_DIR/log


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

echo "Update successful. Reboot system to apply all changes."