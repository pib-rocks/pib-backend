#!/bin/bash
#
# This script installs all ubuntu packages necessary for running pib
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Installing system packages --""$RESET_TEXT_COLOR"		

# Adding universe repo, to update ubuntu
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get -y upgrade
# libusb-1.0-0 libudev1 procps are dependencies of later installed Tinkerforge brick-deamon
sudo apt-get install -y python3 python3-pip openssh-server software-properties-common unzip sqlite3 locales libusb-1.0-0 libudev1 procps php8.1-fpm php-sqlite3 nodejs npm

# Setting up ROS2
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
sudo curl --silent --location https://raw.githubusercontent.com/ros/rosdistro/master/ros.key --output /usr/share/keyrings/ros-archive-keyring.gpg
echo "Adding ros2.list to repositories..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt -y upgrade
sudo apt install -y ros-humble-ros-base ros-dev-tools
source /opt/ros/humble/setup.bash
echo 'source /opt/ros/humble/setup.bash' >> "$USER_HOME/.bashrc"
sudo apt-get install colcon
echo 'source /home/pib/ros_working_dir/install/setup.bash' >> "$USER_HOME/.bashrc"
echo "export ROS_LOCALHOST_ONLY=1" >> "$USER_HOME/.bashrc"

# Install rosbridge-server
sudo apt install -y ros-humble-rosbridge-server

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- System package installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"