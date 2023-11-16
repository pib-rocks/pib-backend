# This script checks if all requirements for running pibs software are met
# Please restart your system at least once after finishing the setup-script before you run this check

# Expected values
expected_user_name="pib"
expected_ubuntu_version='"22.04"'

# System variables
os_release_info_filepath=/etc/os-release
ubuntu_version=$(grep VERSION_ID "$os_release_info_filepath" | cut -f2 -d'=') 

# Variables for "echo -e" output text formatting
red_text_color="\e[31m"
reset_text_color="\e[0m"
new_line="\n"

# Ask if the user restarted the system after running the setup script
while true; do
    read -rep $'\nDid you restart your system after running the setup script? \nAnswer with yes or no: ' user_answer
    case $user_answer in
        [Yy]* ) break;;
        [Nn]* ) exit 1;;
        * ) echo "Please answer yes or no.";;
    esac
done

# Check the system variables
echo -e $new_line"Checking system settings and installations."$new_line"These checks can take some time, so please lean back and wait :)"$new_line

if [ $USER != $expected_user_name ]; then
    echo -e $red_text_color"Your user name is not $expected_user_name. Please change your user name or switch to a user named pib."$reset_color_flag
    exit 1
fi
echo -e "You're using the correct user name: " $expected_user_name$new_line

if [ $ubuntu_version == $expected_ubuntu_version ]; then
    echo -e "You're using the recommended Ubuntu version."$new_line
else
    echo -e $red_text_color"This Ubuntu version is not recommended to use with pib!"$reset_text_color
fi

# Check system installations
if ! colcon version-check >/dev/null 2>&1; then
    echo -e $red_text_color"The colcon package is not installed"$reset_text_color
fi

INSTALLATIONS=([1]=python3-pip [2]=git [3]=python3 [4]=curl [5]=openssh-server [6]=software-properties-common [7]=unzip [8]=sqlite3 [9]=locales [10]=libusb-1.0-0 [11]=libudev1 [12]=procps [13]=php8.1-fpm [14]=python3-pyqt5 [15]=python3-pyqt5.qtopengl [16]=python3-serial [17]=python3-tz [18]=python3-tzlocal [19]=libusb-1.0-0-dev [20]=flac)

for installation in "${INSTALLATIONS[@]}"
do
    if ! dpkg-query -W -f='${Status}' $installation 2>/dev/null | grep -q "install ok installed"; then
        echo -e $red_text_color"The package $installation is not installed"$reset_text_color
    else
        echo -e $installation "checked"
    fi
done

# Check python package installations
PIP_PACKAGES=([1]=depthai [2]=tinkerforge [3]=openai [4]=google-cloud-speech [5]=google-cloud-texttospeech [6]=pyaudio [7]=opencv-python [8]=setuptools)
for package in "${PIP_PACKAGES[@]}"
do
    if ! pip show $package >/dev/null 2>&1; then
        echo -e $red_text_color"The Python-Package $package is not installed"$reset_text_color
    else
        echo -e $package" checked"
    fi
done

echo -e $new_line"---python installations checked---"

# Ask if the user wants to check pib packages and services
while true; do
    read -rep $'\nDo you want to continue with a check on pib packages and services? \nAnswer with yes or no: ' user_answer
    case $user_answer in
        [Yy]* ) break;;
        [Nn]* ) exit 1;;
        * ) echo "Please answer yes or no.";;
    esac
done

# Check pib packages and services
ROS_WORKING_DIR_SRC="/home/pib/ros_working_dir/src"

FOLDERS=([1]=motors [2]=datatypes [3]=voice-assistant [4]=ros2_oak_d_lite)
PACKAGE_NAMES=([1]="name: motors" [2]="name: datatypes" [3]="name: voice_assistant" [4]="name: oak_d_lite")
SYS_CTLS=([1]=ros_motor_control_node_boot.service [2]=ros_motor_current_node_boot.service [3]=pib_api_boot.service [4]=ros_camera_boot.service [5]=ros_cerebra_boot.service [6]=ros_voice_assistant_boot.service)

# Change directory to make the colcon command work independently of the location of this shell script
cd $ROS_WORKING_DIR_SRC
COLCON_INFO=$(colcon info)

for i in "${!FOLDERS[@]}"
do
    if [ -d "$ROS_WORKING_DIR_SRC/${FOLDERS[$i]}" ];then
        if [[ ! $COLCON_INFO == *${PACKAGE_NAMES[$i]}* ]]; then
            echo -e $red_text_color"The package ${FOLDERS[$i]} is not built"$reset_text_color
        else
            echo -e ${FOLDERS[$i]} "package checked"
        fi
    else
        echo -e $red_text_color"The package ${FOLDERS[$i]} is not installed"$reset_text_color
    fi
done

SERVICE_STATUS="Active: active (running)"
for service_name in "${SYS_CTLS[@]}"
do
    if [[ ! $(systemctl status $service_name) == *$SERVICE_STATUS* ]]; then
        echo -e $red_text_color"The service $service_name is not active"$reset_text_color
    else 
        echo -e $service_name "checked"
    fi
done
echo -e $new_line"---pib packages and services checked---"

exit 0