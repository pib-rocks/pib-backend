# This script tests the setup-script
# Please restart once bevor you restart this script

# Some variablen

u="$USER"
file=/etc/lsb-release
ubuntu_v=$(grep DISTRIB_RELEASE "$file" | cut -f2 -d'=') 
YELLOW='\033[0;33'
NC='\033[0m'



while true; do
    read -p "Did you restart once after the setup script?" yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) exit 1;;
        * ) echo "Please answer yes or no.";;
    esac
done

# Check some default settings and installations
echo "Check some detault settings and installations. Thise checks can take some minutes, so pleas lean back and wait :)"

if [ $u != "pib" ]; then
    echo "You are not logged in with the correct user. Pleas login with pib."
    exit1
fi
echo "Correct user: " $u

if [ $ubuntu_v == 22.04 ]; then
    echo "Ubuntu version is recommandet."
else
    echo -e "\033[0;31m Ubuntu version is not recommandet! \033[0m"
fi

if ! colcon version-check >/dev/null 2>&1; then
    echo -e "\033[0;31m The package colcon is not installed \033[0m"
fi

#Setup-Script installations
INSTALLATIONS=([1]=python3-pip [2]=git [3]=python3 [4]=curl [5]=openssh-server [6]=oftware-properties-common [7]=unzip [8]=sqlite3 [9]=locales [10]=libusb-1.0-0 [11]=libudev1 [12]=procps [13]=php8.1-fpm [14]=python3-pyqt5 [15]=python3-pyqt5.qtopengl [16]=python3-serial [17]=python3-tz [18]=python3-tzlocal [19]=libusb-1.0-0-dev [20]=flac)


for installation in "${INSTALLATIONS[@]}"
do
    if ! dpkg-query -W -f='${Status}' $installation 2>/dev/null | grep -q "install ok installed"; then
        echo -e "\033[0;31m The package $installation is not installed \033[0m"
    else
        echo $installation "checked"
    fi
done
# Package installations
PIP_PACKAGES=([1]=depthai [2]=tinkerforge [3]=openai [4]=google-cloud-speech [5]=google-cloud-texttospeech [6]=pyaudio [7]=opencv-python [8]=setuptools)
for package in "${PIP_PACKAGES[@]}"
do
    if ! pip show $package >/dev/null 2>&1; then
        echo -e "\033[0;31m The Python-Package $package is not installed \033[0m"
    else
        echo $package" checked"
    fi
done

echo "---installations checked---"
while true; do
    read -p "Do you want to contionue with a check on packages and services?" yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) exit 1;;
        * ) echo "Please answer yes or no.";;
    esac
done

# package check
COLCON_INFO=$(colcon info)

SYS_CTLS=([1]=ros_motor_control_node_boot.service [2]=ros_motor_current_node_boot.service [3]=pib_api_boot.service [4]=ros_camera_boot.service [5]=ros_cerebra_boot.service [6]=ros_voice_assistant_boot.service)

SERVICE_STATUS="Active: active (running)"
ROS_WORKING_DIR_SRC="/home/pib/ros_working_dir/src"
COUNT=0
FOLDERS=([1]=motors [2]=datatypes [3]=voice-assistant [4]=ros2_oak_d_lite)
PACKAGE_NAMES=([1]=name: motors [2]=name: datatypes [3]=name: oak_d_lite [4]=name: voice_assistant)

for folder in "${FOLDERS[@]}"
do
    if [ -d "$ROS_WORKING_DIR_SRC/$folder" ];then
        if [[ ! $COLCON_INFO == *$PACKAGE_NAMES[$COUNT]* ]]; then
            echo -e "\033[0;31m The package $folder is not built \033[0m"
        else
            echo $folder "checked"
        fi
    else
        echo -e "\033[0;31m The package $folder is not installed \033[0m"
    fi
done

for service_name in "${SYS_CTLS[@]}"
do
    if [[ ! $(systemctl status $service_name) == *$SERVICE_STATUS* ]]; then
        echo -e "\033[0;31m The service $service_name is not active \033[0m"
    else 
        echo $service_name "checked"
    fi
done
echo "---services and packages checked---"

exit 0