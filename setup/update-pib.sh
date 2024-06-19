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

# Install local utility packages
pip install "$ROS_WORKING_DIR""/src/motors/pib_motors"

# Boot flask api
sudo chmod 700 "$USER_HOME/flask/pib_api_boot.service"
sudo mv "$USER_HOME/flask/pib_api_boot.service" /etc/systemd/system
sudo systemctl enable pib_api_boot.service

# Boot camera
sudo chmod 700 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.sh"
sudo chmod 700 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service"
sudo mv "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service" /etc/systemd/system
sudo systemctl enable ros_camera_boot.service

# Boot bricklet uid script
sudo chmod 700 "$ROS_WORKING_DIR/src/motors/pib_motors/pib_motors/update_bricklet_uids.py"
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/bricklet_uid_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/bricklet_uid_boot.service" /etc/systemd/system
sudo systemctl enable bricklet_uid_boot.service

# Boot motor control node
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_boot.sh"
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/ros_motor_boot.service" /etc/systemd/system
sudo systemctl enable ros_motor_boot.service


# Boot voice assistant
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.sh"
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service"
sudo mv "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service" /etc/systemd/system
sudo systemctl enable ros_voice_assistant_boot.service

# Boot audio player
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_audio_player_boot.sh"
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_audio_player_boot.service"
sudo mv "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_audio_player_boot.service" /etc/systemd/system
sudo systemctl enable ros_audio_player_boot.service

# Boot audio recorder
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_audio_recorder_boot.sh"
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_audio_recorder_boot.service"
sudo mv "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_audio_recorder_boot.service" /etc/systemd/system
sudo systemctl enable ros_audio_recorder_boot.service

# Boot chat
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_chat_boot.sh"
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_chat_boot.service"
sudo mv "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_chat_boot.service" /etc/systemd/system
sudo systemctl enable ros_chat_boot.service

# Boot cerebra
sudo chmod 700 "$BACKEND_DIR/setup/setup_files/ros_cerebra_boot.service"
sudo mv "$BACKEND_DIR/setup/setup_files/ros_cerebra_boot.service" /etc/systemd/system
sudo systemctl enable ros_cerebra_boot.service

# Boot program node
sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.sh"
sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service"
sudo mv "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service" /etc/systemd/system
sudo systemctl enable ros_program_boot.service


cd $ROS_WORKING_DIR
colcon build
sudo chmod -R 777 $ROS_WORKING_DIR/build
sudo chmod -R 777 $ROS_WORKING_DIR/install
sudo chmod -R 777 $ROS_WORKING_DIR/log


echo "Checking BrickletsIDs..."
readonly MOTOR_UTILS_DIR="/home/pib/ros_working_dir/src/motors/pib_motors/pib_motors"
readonly PYTHON_UID_SCRIPT_IMPORT="import sys; sys.path.insert(0, '$MOTOR_UTILS_DIR')"

readonly CHANGE_DETECTED=$(python3 -c "$PYTHON_UID_SCRIPT_IMPORT; from update_bricklet_uids import no_uids_in_database; print(no_uids_in_database())")

if [ "$CHANGE_DETECTED" == True ]; then
    while true; do
        read -p "UID changes were detected. Do you want to update your Bricklet-UIDs? (yes/no): " yn
        case $yn in
                [Yy]* ) python3 -c "$PYTHON_UID_SCRIPT_IMPORT; from update_bricklet_uids import update_bricklet_uids; check_and_update()"; break;;
                [Nn]* ) break;;
                * ) echo "Please answer yes or no.";;
        esac
    done
fi

echo "Update successful. Reboot system to apply all changes."