#!/bin/bash
#
# This script installs the TinkerForge software bundle, 
# including: Brick Daemon, Brick Viewer and Python API bindings

# Brick daemon
PLATFORM_TYPE=$(uname -m)
if [ $PLATFORM_TYPE != 'aarch64' ]; then
	echo "Installing Brick daemon for $PLATFORM_TYPE"
	curl -O https://download.tinkerforge.com/tools/brickd/linux/brickd_linux_latest_amd64.deb
	sudo dpkg -i brickd_linux_latest_amd64.deb
else
	echo "Installing Brick daemon for $PLATFORM_TYPE"
	curl -O https://download.tinkerforge.com/tools/brickd/linux/brickd_linux_latest_arm64.deb
	sudo dpkg -i brickd_linux_latest_arm64.deb
fi
# Brick viewer
sudo apt-get install -y python3-pyqt5 python3-pyqt5.qtopengl python3-serial python3-tz python3-tzlocal
curl -O https://download.tinkerforge.com/tools/brickv/linux/brickv_linux_latest.deb
sudo dpkg -i brickv_linux_latest.deb
# Tinkerforge python APIs
curl -sSL https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.gpg | sudo tee /etc/apt/trusted.gpg.d/tinkerforge.gpg > /dev/null
echo "deb https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/tinkerforge.list
sudo apt-get update
sudo apt-get install -y python3-tinkerforge