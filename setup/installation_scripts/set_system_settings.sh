#!/bin/bash
#
# This script adjusts system settings
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

print INFO "Adjusting system settings"

sudo systemctl daemon-reload
sudo systemctl enable ssh --now

if is_supported_raspbian; then
    local CONFIG_FILE="/boot/firmware/config.txt"

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
        print INFO "Adjusted display resolution and settings"
    fi
fi


if is_ubuntu_noble; then
    # Activate automatic login settings via regex
    sudo sed -i '/#  AutomaticLogin/{s/#//;s/user1/pib/}' /etc/gdm3/custom.conf
    print INFO "Activated automatic login"

    # Disabling power saving settings
    gsettings set org.gnome.desktop.session idle-delay 0
    gsettings set org.gnome.settings-daemon.plugins.power power-saver-profile-on-low-battery false
    gsettings set org.gnome.settings-daemon.plugins.power ambient-enabled false
    gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
    gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'
    print INFO "Disabled power saving settings"

    # Add default ubuntu terminal to favorites
    gsettings set org.gnome.shell favorite-apps "$(gsettings get org.gnome.shell favorite-apps | sed s/.$//), 'org.gnome.Terminal.desktop']"
    print INFO "Added terminal to favorites"
fi

print SUCCESS "System settings adjusted"
