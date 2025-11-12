#!/bin/bash

# This script checks if all requirements for running pibs software are met

# Expected values
expected_user_name="pib"
expected_ubuntu_version='"22.04"'

# System variables
os_release_info_filepath=/etc/os-release
ubuntu_version=$(grep VERSION_ID "$os_release_info_filepath" | cut -f2 -d'=') 

# Variables for "echo -e" output text formatting
red_text_color="\e[31m"
yellow_text_color="\e[33m"
reset_text_color="\e[0m"
new_line="\n"

# Show infos about the command-line parameter options, then exit the script
help_function() 
{
    echo -e "$red_text_color""Invalid command-line option used""$reset_text_color"
    echo -e "The only available command-line parameter is '-d'."
    echo -e "You can use -d (dev-mode) to run only part of the health check."
    echo -e "$new_line""Use the command like this: $0 -d"
    exit 1
}

# Input variables
true="true"
false="false"

# Check if script was started in dev-mode
is_dev_mode=$false
if [ "$1" = "-d" ] 
then
    is_dev_mode=$true
# Show help if any other option was called
elif [ -n "$1" ] 
then
    help_function
fi

# When running in dev mode, ask the user which checks should be done
run_ubuntu_check=$true
run_python_package_check=$true
run_pip_package_check=$true

if [ $is_dev_mode = $true ]; then
    # Ask if the user wants to run a ubuntu setup and installation check
    while true; do
        read -rep $'\nDo you want to check the general ubuntu setup and installations? \nAnswer with yes or no: ' user_answer
        case $user_answer in
            [Yy]* ) break;;
            [Nn]* ) run_ubuntu_check=$false; break;;  
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Ask if the user wants to check python packages
    while true; do
        read -rep $'\nDo you want to check the python packages? \nAnswer with yes or no: ' user_answer
        case $user_answer in
            [Yy]* ) break;;
            [Nn]* ) run_python_package_check=$false; break;;  
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Ask if the user wants to check pib packages and services
    while true; do
        read -rep $'\nDo you want to check on pib packages and services? \nAnswer with yes or no: ' user_answer
        case $user_answer in
            [Yy]* ) break;;
            [Nn]* ) run_pip_package_check=$false; break;;  
            * ) echo "Please answer yes or no.";;
        esac
    done
fi

# Check general Ubuntu setup and installation 
if [ $run_ubuntu_check = $true ]; then
    # Check the system variables
    echo -e "$new_line""$yellow_text_color""--- checking ubuntu system settings and installations ---""$reset_text_color"

    if [ "$USER" != $expected_user_name ]; then
        echo -e "$red_text_color""Your user name is not $expected_user_name.""$new_line""Please change your user name or switch to a user named $expected_user_name.""$reset_text_color"
        exit 1
    fi
    echo -e "You're using the correct user name: " $expected_user_name"$new_line"

    if [ "$ubuntu_version" == $expected_ubuntu_version ]; then
        echo -e "You're using the recommended Ubuntu version.""$new_line"
    else
        echo -e "$red_text_color""This is not the recommended ubuntu version ($ubuntu_version).""$new_line""Please use $expected_ubuntu_version instead.""$reset_text_color"
    fi

    # Check system installations
    if ! colcon version-check >/dev/null 2>&1; then
        echo -e "$red_text_color""The colcon package is not installed""$reset_text_color"
    fi

    installations=([1]=python3-pip [2]=git [3]=python3 [4]=curl [5]=openssh-server [6]=software-properties-common [7]=unzip [8]=sqlite3 [9]=locales [10]=libusb-1.0-0 [11]=libudev1 [12]=procps [13]=php8.1-fpm [14]=python3-pyqt5 [15]=python3-pyqt5.qtopengl [16]=python3-serial [17]=python3-tz [18]=python3-tzlocal [19]=libusb-1.0-0-dev [20]=flac [21]=portaudio19-dev [22]=python3-tinkerforge)
    for installation in "${installations[@]}"
    do
        if ! dpkg-query -W -f='${Status}' "$installation" 2>/dev/null | grep -q "install ok installed"; then
            echo -e "$red_text_color""The package $installation is not installed""$reset_text_color"
        else
            echo -e "$installation is installed"
        fi
    done
    echo -e "$yellow_text_color""--- ubuntu check completed ---""$reset_text_color"
fi

# Check python package installations
if [ $run_python_package_check = $true ]; then
    echo -e "$new_line""$yellow_text_color""--- checking python packages ---""$reset_text_color"

    pip_packages=([1]=depthai [2]=tinkerforge [3]=openai [4]=google-cloud-speech [5]=google-cloud-texttospeech [6]=pyaudio [7]=opencv-python [8]=setuptools)
    for package in "${pip_packages[@]}"
    do
        if ! pip show "$package" >/dev/null 2>&1; then
            echo -e "$red_text_color""The Python-Package $package is not installed""$reset_text_color"
        else
            echo -e "$package is installed"
        fi
    done
    echo -e "$yellow_text_color""--- python package check completed ---""$reset_text_color"
fi

# Check pib packages and services
if [ $run_pip_package_check = $true ]; then
    echo -e "$new_line""$yellow_text_color""--- checking pib packages and services ---""$reset_text_color"

    ros_working_dir_src="/home/pib/ros_working_dir/src"

    folders=([1]=motors [2]=datatypes [3]=voice_assistant [4]=camera [5]=programs)
    package_names=([1]="name: motors" [2]="name: datatypes" [3]="name: voice_assistant" [4]="name: oak_d_lite" [5]="name: programs")
    sys_ctls=([1]=ros_motor_control_node_boot.service [2]=ros_motor_current_node_boot.service [3]=pib_api_boot.service [4]=ros_camera_boot.service [5]=ros_cerebra_boot.service [6]=ros_voice_assistant_boot.service [7]=ros_chat_boot.service [8]=ros_audio_recorder.service [9]=ros_audio_player.service [10]=ros_program_boot.service [11]=ros_proxy_program_boot.service)

    # Change directory to make the colcon command work independently of the location of this shell script
    cd $ros_working_dir_src
    COLCON_INFO=$(colcon info)

    for i in "${!folders[@]}"
    do
        if [ -d "$ros_working_dir_src/${folders[$i]}" ];then
            if [[ ! $COLCON_INFO == *${package_names[$i]}* ]]; then
                echo -e "$red_text_color""The package ${folders[$i]} is not built""$reset_text_color"
            else
                echo -e "${folders[$i]} package is built and installed"
            fi
        else
            echo -e "$red_text_color""The package ${folders[$i]} is not installed""$reset_text_color"
        fi
    done

    service_status_active="Active: active (running)"
    service_status_not_found="could not be found"
    for service_name in "${sys_ctls[@]}"
    do
        # Save the standard output and standard error (2>&1) in a variable
        service_status=$(systemctl status "$service_name" 2>&1)
        
        if [[ $service_status == *$service_status_active* ]]; then
            echo -e "$service_name is active"
        else 
            if [[ $service_status == *$service_status_not_found* ]]; then
                echo -e "$red_text_color""The service $service_name could not be found""$reset_text_color"
            else
                echo -e "$red_text_color""The service $service_name is not active""$reset_text_color"
            fi
        fi
    done
    echo -e "$yellow_text_color""--- pib packages and services check completed ---""$reset_text_color"
fi

echo -e "$new_line""$yellow_text_color""+++ system health check completed +++""$reset_text_color""$new_line"

exit 0