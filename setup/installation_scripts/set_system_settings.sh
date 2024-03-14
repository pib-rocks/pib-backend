#!/bin/bash
#
# This script adjust ubuntu system settings
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Adjusting system settings --""$RESET_TEXT_COLOR"	

# This config file exists only on RaspberryPi, but not VMs 
CONFIG_FILE="/boot/firmware/config.txt"

# Apply display and resolution settings if config file exists
if [ -e "$CONFIG_FILE" ]; then
    declare -A settingsMap=(
    ["hdmi_force_edid_audio"]="hdmi_force_edid_audio=1"
    ["max_usb_current"]="max_usb_current=1"
    ["hdmi_force_hotplug"]="hdmi_force_hotplug=1"
    ["config_hdmi_boost"]="config_hdmi_boost=7"
    ["hdmi_group"]="hdmi_group=2"
    ["hdmi_mode"]="hdmi_mode=87"
    ["hdmi_drive"]="hdmi_drive=2"
    ["display_rotate"]="display_rotate=0"
    ["hdmi_cvt"]="hdmi_cvt 1024 600 60 6 0 0 0"
    ["dtoverlay=vc4-kms-v3d"]="dtoverlay=vc4-fkms-v3d"
    )
    for setting in "${!settingsMap[@]}"
    do
        if grep -q "$setting" "$CONFIG_FILE"; then
            sudo sed -i "/$setting/c\\${settingsMap[$setting]}" "$CONFIG_FILE"
        elif ! grep -q "$setting" "$CONFIG_FILE" && ! grep -q "${settingsMap[$setting]}" "$CONFIG_FILE"; then
            echo "${settingsMap[$setting]}" | sudo tee -a "$CONFIG_FILE" > /dev/null
        fi
    done
fi

# Activate automatic login settings via regex
sudo sed -i '/#  AutomaticLogin/{s/#//;s/user1/pib/}' /etc/gdm3/custom.conf

# Disabling power saving settings
gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.settings-daemon.plugins.power power-saver-profile-on-low-battery false
gsettings set org.gnome.settings-daemon.plugins.power ambient-enabled false
gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'

# Add default ubuntu terminal to favorites
gsettings set org.gnome.shell favorite-apps "$(gsettings get org.gnome.shell favorite-apps | sed s/.$//), 'org.gnome.Terminal.desktop']"

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- System settings adjustment completed --""$RESET_TEXT_COLOR""$NEW_LINE"