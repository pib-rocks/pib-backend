#!/bin/bash
#
# This script adjust ubuntu system settings
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Adjusting system settings --""$RESET_TEXT_COLOR"	

# Activate automatic login settings via regex
sudo sed -i '/#  AutomaticLogin/{s/#//;s/user1/pib/}' /etc/gdm3/custom.conf

# Disabling power saving settings
gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.settings-daemon.plugins.power power-saver-profile-on-low-battery false
gsettings set org.gnome.settings-daemon.plugins.power ambient-enabled false
gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'