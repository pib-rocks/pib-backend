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

#Setup-Script installations
echo -ne '                          (0%)\r'
if ! dpkg-query -W -f='${Status}' python3-pip 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package python3-pip is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' git 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package git is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' python3 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package python3 is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' curl 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package curl is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' openssh-server 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package openssh-server is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' software-properties-common 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package software-properties-common is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' unzip 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package unzip is not installed \033[0m"
fi
echo -ne '#########                 (25%)\r'
if ! dpkg-query -W -f='${Status}' sqlite3 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package sqlite3 is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' locales 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package locales is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' libusb-1.0-0 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package libusb-1.0-0 is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' libudev1 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package libudev1 is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' procps 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package procps is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' php8.1-fpm 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package php8.1-fpm is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' php-sqlite3 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package php-sqlite3 is not installed \033[0m"
fi
if ! colcon version-check >/dev/null 2>&1; then
    echo -e "\033[0;31m The package colcon is not installed \033[0m"
fi
echo -ne '##############            (50%)\r'
if ! dpkg-query -W -f='${Status}' python3-pyqt5 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package python3-pyqt5 is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' python3-pyqt5.qtopengl 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package python3-pyqt5.qtopengl is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' python3-serial 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package python3-serial is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' python3-tz 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package python3-tz is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' python3-tzlocal 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package python3-tzlocal is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' libusb-1.0-0-dev 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package libusb-1.0-0-dev is not installed \033[0m"
fi
if ! dpkg-query -W -f='${Status}' flac 2>/dev/null | grep -q "install ok installed"; then
    echo -e "\033[0;31m The package flac is not installed \033[0m"
fi

# Package installations
if ! pip show depthai >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'depthai' is not installed \033[0m"
fi
if ! pip show tinkerforge >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'tinkerforge' is not installed \033[0m"
fi
echo -ne '#####################     (75%)\r'
if ! pip show openai >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'openai' is not installed \033[0m"
fi
if ! pip show google-cloud-speech >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'google-cloud-speech' is not installed \033[0m"
fi
if ! pip show google-cloud-texttospeech >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'google-cloud-texttospeech' is not installed \033[0m"
fi
if ! pip show pyaudio >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'pyaudio' is not installed \033[0m"
fi
if ! pip show opencv-python >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'opencv-python' is not installed \033[0m"
fi
if ! pip show depthai >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'depthai' is not installed \033[0m"
fi
if ! pip show setuptools  >/dev/null 2>&1; then
    echo -e "\033[0;31m The Python-Package 'setuptools ' is not installed \033[0m"
fi
echo -ne '##########################(100%)\r'

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

PACKAGE_NAME_MOTORS="name: motors"
PACKAGE_NAME_DATATYPES="name: datatypes"
PACKAGE_NAME_DLITE="name: oak_d_lite"
PACKAGE_NAME_VOICE_ASSISTANT="name: voice_assistant"

SYS_CTL_MOTORS_CONTROL=$(systemctl status ros_motor_control_node_boot.service)
SYS_CTL_MOTORS_CURRENT=$(systemctl status ros_motor_current_node_boot.service)
SYS_CTL_API=$(systemctl status pib_api_boot.service)
SYS_CTL_CAMERA=$(systemctl status ros_camera_boot.service)
SYS_CTL_CEREBRA=$(systemctl status ros_cerebra_boot.service)
SYS_CTL_VOICE_ASSISTANT=$(systemctl status ros_voice_assistant_boot.service)

SERVICE_STATUS="Active: active (running)"

echo -ne '                          (0%)\r'
if [[ ! $COLCON_INFO == *$PACKAGE_NAME_MOTORS* ]]; then
    echo -e "\033[0;31m The package motors is not installed \033[0m"
fi
if [[ ! $COLCON_INFO == *$PACKAGE_NAME_DATATYPES* ]]; then
    echo -e "\033[0;31m The package motors is not installed \033[0m"
fi
echo -ne '#########                 (25%)\r'
if [[ ! $COLCON_INFO == *$PACKAGE_NAME_DLITE* ]]; then
    echo -e "\033[0;31m The package oak_d_lite is not installed \033[0m"
fi
if [[ ! $COLCON_INFO == *$PACKAGE_NAME_VOICE_ASSISTANT* ]]; then
    echo -e "\033[0;31m The package voice_assistant is not installed \033[0m"
fi

if [[ ! $SYS_CTL_MOTORS_CONTROL == *$SERVICE_STATUS* ]]; then
    echo -e "\033[0;31m The service ros_motor_control_node_boot is not active \033[0m"
fi
echo -ne '##############            (50%)\r'
if [[ ! $SYS_CTL_MOTORS_CURRENT == *$SERVICE_STATUS* ]]; then
    echo -e "\033[0;31m The service ros_motor_current_node_boot is not active \033[0m"
fi
if [[ ! $SYS_CTL_API == *$SERVICE_STATUS* ]]; then
    echo -e "\033[0;31m The service pib_api_boot is not active \033[0m"
fi
if [[ ! $SYS_CTL_CAMERA == *$SERVICE_STATUS* ]]; then
    echo -e "\033[0;31m The service ros_camera_boot is not active \033[0m"
fi
echo -ne '#####################     (75%)\r'
if [[ ! $SYS_CTL_CEREBRA == *$SERVICE_STATUS* ]]; then
    echo -e "\033[0;31m The service ros_cerebra_boot is not active \033[0m"
fi
if [[ ! $SYS_CTL_VOICE_ASSISTANT == *$SERVICE_STATUS* ]]; then
    echo -e "\033[0;31m The service ros_voice_assistant_boot is not active \033[0m"
fi
echo -ne '##########################(100%)\r'
echo "---services and packages checked---"

exit 0