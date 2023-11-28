#!/bin/bash
#
# This script sets up our custom ros packages

echo -e "$YELLOW_TEXT_COLOR""-- Setting up custom ros packages --""$RESET_TEXT_COLOR"		

# Boot script file locations
ROS_CAMERA_BOOT_DIR="$ROS_WORKING_DIR"/src/ros2_oak_d_lite/boot_scripts
ROS_MOTORS_BOOT_DIR="$ROS_WORKING_DIR"/src/motors/boot_scripts
ROS_VOICE_ASSISTANT_BOOT_DIR="$ROS_WORKING_DIR"/src/voice-assistant/boot_scripts

#
# Installing dependencies
# Depth-AI
sudo curl -sSL https://docs.luxonis.com/install_dependencies.sh | sudo bash
python3 -m pip install depthai
# Setting up the motor packages
pip3.10 install tinkerforge
sudo apt-get install libusb-1.0-0-dev
# Setting up the voice-assistant packages
pip3.10 install openai google-cloud-speech google-cloud-texttospeech pyaudio
sudo apt-get install flac
#Git examples for Depth-AI
git clone --recurse-submodules https://github.com/luxonis/depthai-python.git
cd depthai-python/examples
python3 install_requirements.py
# Hand tracker
git clone https://github.com/geaxgx/depthai_hand_tracker.git
cd depthai_hand_tracker
pip install -r requirements.txt
#
#check on git
echo 'check if git init is done'
cd $ROS_WORKING_DIR/src
if [ ! -f .git ]; then
	git init
fi
#git pull packages with sub modules
echo 'git pull packages with sub modules'
git pull https://github.com/pib-rocks/ros-packages.git
chmod +x package_set_up.sh

# Run the script for creating a custom gitmodules file
readonly CREATE_GITMODULE_FILE="$INSTALLATION_FILES_DIR"/"create_gitmodule_file.sh"
chmod 755 "$CREATE_GITMODULE_FILE"
if [ "$is_dev_mode" = "$TRUE"]; then
	source "$CREATE_GITMODULE_FILE" "-d" "$user_default_branch" "$user_feature_branch"
else
	source "$CREATE_GITMODULE_FILE"
fi
git submodule init
git submodule update --remote
echo 'Done with installing packages'
#
echo "Booting all nodes..."
# Boot camera
sudo chmod 755 $ROS_CAMERA_BOOT_DIR/ros_camera_boot.sh
sudo chmod 755 $ROS_CAMERA_BOOT_DIR/ros_camera_boot.service
sudo mv $ROS_CAMERA_BOOT_DIR/ros_camera_boot.service /etc/systemd/system
sudo systemctl enable ros_camera_boot.service

# Boot motor control node
sudo chmod 755 $ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.sh
sudo chmod 755 $ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.service
sudo mv $ROS_MOTORS_BOOT_DIR/ros_motor_control_node_boot.service /etc/systemd/system
sudo systemctl enable ros_motor_control_node_boot.service

# Boot motor current node
sudo chmod 755 $ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.sh
sudo chmod 755 $ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.service
sudo mv $ROS_MOTORS_BOOT_DIR/ros_motor_current_node_boot.service /etc/systemd/system
sudo systemctl enable ros_motor_current_node_boot.service

# Boot voice-assistant
sudo chmod 755 $ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.sh
sudo chmod 755 $ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service
sudo mv $ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service /etc/systemd/system
sudo systemctl enable ros_voice_assistant_boot.service

cd $ROS_WORKING_DIR
colcon build

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Custom ros package setup completed --""$RESET_TEXT_COLOR""$NEW_LINE"