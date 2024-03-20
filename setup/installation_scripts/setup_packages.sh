#!/bin/bash
#
# This script sets up our custom ros packages
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Setting up custom ros packages --""$RESET_TEXT_COLOR"		

# Boot script file locations
ROS_CAMERA_BOOT_DIR="$ROS_WORKING_DIR"/src/camera/boot_scripts
ROS_MOTORS_BOOT_DIR="$ROS_WORKING_DIR"/src/motors/boot_scripts
ROS_VOICE_ASSISTANT_BOOT_DIR="$ROS_WORKING_DIR"/src/voice_assistant/boot_scripts
ROS_PROGRAMS_BOOT_DIR="$ROS_WORKING_DIR"/src/programs/boot_scripts

#
# Installing dependencies
# Depth-AI
sudo curl --silent --location https://docs.luxonis.com/install_dependencies.sh | sudo bash
python3 -m pip install depthai
# Setting up the motor packages
pip3.10 install tinkerforge
sudo apt-get -y install libusb-1.0-0-dev
# Setting up the voice-assistant packages
pip3.10 install openai google-cloud-speech google-cloud-texttospeech pyaudio
sudo apt-get install flac
# Git examples for Depth-AI
git clone --recurse-submodules https://github.com/luxonis/depthai-python.git
cd depthai-python/examples
python3 install_requirements.py
# Hand tracker
git clone https://github.com/geaxgx/depthai_hand_tracker.git
cd depthai_hand_tracker
pip install -r requirements.txt

# Install SLAM dependencies
sudo apt install -y ros-humble-depthai-ros
sudo apt install -y ros-humble-rtabmap

# Currently, ros-humble-rtabmap-ros does not work on ARM devices anymore. 
# Once it does, packages below can be replaced with it
sudo apt install -y ros-humble-rtabmap-launch
sudo apt install -y ros-humble-rtabmap-examples

# move ros-packages into working directory
cp -r "$BACKEND_DIR/ros_packages" "$ROS_WORKING_DIR/src"
sudo chmod -R 700 "$ROS_WORKING_DIR"

# Create credentials folder and files required for the voice-assistant
readonly VOICE_ASSISTANT_CREDENTIALS_DIR="$ROS_WORKING_DIR/src/voice_assistant/credentials"
mkdir "$VOICE_ASSISTANT_CREDENTIALS_DIR"
touch "$VOICE_ASSISTANT_CREDENTIALS_DIR/openai-key"
touch "$VOICE_ASSISTANT_CREDENTIALS_DIR/google-key"
printf '{\n\t"access_key_id": "",\n\t"secret_access_key": "",\n\t"region_name": ""\n}\n' > "$VOICE_ASSISTANT_CREDENTIALS_DIR/aws-key"

# Create virtual-environment for user programs
sudo apt-get install -y python3.10-venv
readonly USER_PROGRAM_ENV_DIR="$ROS_WORKING_DIR/src/programs/user_program_env"
mkdir "$USER_PROGRAM_ENV_DIR"
sudo chmod 700 "$USER_PROGRAM_ENV_DIR"
python3 -m venv "$USER_PROGRAM_ENV_DIR"
source "$USER_PROGRAM_ENV_DIR/bin/activate"
python3 -m pip install numpy==1.26.3
python3 -m pip install depthai==2.24.0.0
python3 -m pip install blobconverter==1.4.2
deactivate

echo "Install local packages..."

# install local utility packages
pip install "$ROS_WORKING_DIR""/src/motors/pib_motors"
pip install "$ROS_WORKING_DIR""/src/voice_assistant/pib_voice"

echo "Booting all nodes..."

# Boot camera
sudo chmod 700 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.sh"
sudo chmod 700 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service"
sudo mv "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service" /etc/systemd/system
sudo systemctl enable ros_camera_boot.service

# Boot bricklet uid script
sudo chmod 700 "$ROS_WORKING_DIR/src/motors/utils/update_bricklet_uids.py"
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/bricklet_uid_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/bricklet_uid_boot.service" /etc/systemd/system
sudo systemctl enable bricklet_uid_boot.service

# Boot motor control node
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.sh"
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.service" /etc/systemd/system
sudo systemctl enable ros_motor_control_node_boot.service

# Boot motor current node
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.sh"
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.service" /etc/systemd/system
sudo systemctl enable ros_motor_current_node_boot.service

# Boot voice-assistant
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.sh"
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service"
sudo mv "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service" /etc/systemd/system
sudo systemctl enable ros_voice_assistant_boot.service

# Boot text-to-speech
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR""/ros_text_to_speech_boot.sh"
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR""/ros_text_to_speech_boot.service"
sudo mv "$ROS_VOICE_ASSISTANT_BOOT_DIR""/ros_text_to_speech_boot.service" /etc/systemd/system
sudo systemctl enable ros_text_to_speech_boot.service

# Boot program node
sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.sh"
sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service"
sudo mv "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service" /etc/systemd/system
sudo systemctl enable ros_program_boot.service

# Boot program proxy node
sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_proxy_program_boot.sh"
sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_proxy_program_boot.service"
sudo mv "$ROS_PROGRAMS_BOOT_DIR/ros_proxy_program_boot.service" /etc/systemd/system
sudo systemctl enable ros_proxy_program_boot.service

cd "$ROS_WORKING_DIR"
colcon build

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Custom ros package setup completed --""$RESET_TEXT_COLOR""$NEW_LINE"