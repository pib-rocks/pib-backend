#!/bin/bash
#
# This script sets up our custom ros packages
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

print_colored_line_of_text "$YELLOW_TEXT_COLOR" "-- Setting up custom ros packages --"

# Boot script file locations
ROS_CAMERA_BOOT_DIR="$ROS_WORKING_DIR"/src/camera/boot_scripts
ROS_MOTORS_BOOT_DIR="$ROS_WORKING_DIR"/src/motors/boot_scripts
ROS_VOICE_ASSISTANT_BOOT_DIR="$ROS_WORKING_DIR"/src/voice_assistant/boot_scripts
ROS_PROGRAMS_BOOT_DIR="$ROS_WORKING_DIR"/src/programs/boot_scripts



# Install motor package dependencies
sudo apt-get -y install libusb-1.0-0-dev
pip3.10 install -r "$BACKEND_DIR/ros_packages/motors/requirements.txt"

# Install voice assistant package dependencies
sudo apt-get install -y portaudio19-dev
sudo apt-get install flac
pip3.10 install -r "$BACKEND_DIR/ros_packages/voice_assistant/requirements.txt"
pip3.10 install "$BACKEND_DIR/public_api_client"
pip3.10 install pyaudio==0.2.14
mkdir "$USER_HOME/public_api"
printf "{\n\t\"trybUrlPrefix\": \"\",\n\t\"publicApiToken\": \"\"\n}\n" > "$USER_HOME/public_api/config.json"

# Install camera package dependencies
# Depth-AI
sudo curl --silent --location https://docs.luxonis.com/install_dependencies.sh | sudo bash
python3 -m pip install depthai

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

# Move ros-packages into working directory
cp -r "$BACKEND_DIR/ros_packages" "$ROS_WORKING_DIR/src"
sudo chmod -R 700 "$ROS_WORKING_DIR"

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

# Install local utility packages
pip install "$ROS_WORKING_DIR""/src/motors/pib_motors"

echo "Booting all nodes..."

# Boot camera
sudo chmod 700 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.sh"
sudo chmod 700 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service"
sudo mv "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service" /etc/systemd/system
sudo systemctl enable ros_camera_boot.service

# Boot motor nodes
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_boot.sh"
sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_boot.service"
sudo mv "$ROS_MOTORS_BOOT_DIR/ros_motor_boot.service" /etc/systemd/system
sudo systemctl enable ros_motor_boot.service

# Boot voice-assistant
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.sh"
sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service"
sudo mv "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service" /etc/systemd/system
sudo systemctl enable ros_voice_assistant_boot.service

# Boot program node
sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.sh"
sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service"
sudo mv "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service" /etc/systemd/system
sudo systemctl enable ros_program_boot.service

cd "$ROS_WORKING_DIR"
colcon build

print_colored_line_of_text "$GREEN_TEXT_COLOR" "-- Custom ros package setup completed --"