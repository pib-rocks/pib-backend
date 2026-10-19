#!/bin/bash
#
# This script adjusts system settings
# To properly run this script relies on being sourced by the "setup-pib.sh"-script
#
# Everything lives in functions on purpose. The previous version declared `local` variables at
# the top level of this sourced file; bash rejects that ("local: can only be used in a
# function"), the variables stayed empty, and the install log showed `cp: cannot create
# regular file ''` while the step still reported success. The call sites now check their
# inputs with require_nonempty (from setup-pib.sh) before any cp/grep/sed runs.

# Raspberry Pi OS ships full KMS (vc4-kms-v3d) and its Wayland session (labwc) needs it.
# Earlier versions of this step rewrote it to the legacy vc4-fkms-v3d on every run; on a
# Raspberry Pi 5 the kernel then reported "No displays found", labwc found 0 GPUs and LightDM
# stopped starting the session. Active vc4-fkms-v3d lines are turned back in place, so their
# config.txt section and overlay options stay, after a backup next to the file. Comments are
# not touched. The new overlay is loaded at the next boot.
function ensure_full_kms_overlay() {
    local config_file="$1"
    local fkms='^[[:space:]]*dtoverlay=vc4-fkms-v3d([,[:space:]]|$)'
    local kms='^[[:space:]]*dtoverlay=vc4-kms-v3d(-pi[0-9]+)?([,[:space:]]|$)'
    local backup

    require_nonempty config_file || return 1

    if grep -Eq "$fkms" "$config_file"; then
        backup="${config_file}.pib-backup-$(date +%Y%m%d%H%M%S)"
        if [ -e "$backup" ]; then
            backup="${backup}-$$"
        fi
        sudo cp "$config_file" "$backup" || return 1
        sudo sed -i -E "s/${fkms}/dtoverlay=vc4-kms-v3d\\1/" "$config_file" || return 1
        print WARN "Restored full KMS (dtoverlay=vc4-kms-v3d) in ${config_file}; previous file: ${backup}. Reboot pib to apply it"
        return 0
    fi

    if ! grep -Eq "$kms" "$config_file"; then
        # An unterminated last line would swallow the appended overlay.
        if [ -s "$config_file" ] && [ -n "$(tail -c 1 "$config_file")" ]; then
            printf '\n' | sudo tee -a "$config_file" > /dev/null || return 1
        fi
        echo "dtoverlay=vc4-kms-v3d" | sudo tee -a "$config_file" > /dev/null || return 1
        print WARN "Added dtoverlay=vc4-kms-v3d to ${config_file}. Reboot pib to apply it"
    fi
}

# PIB_BOOT_CONFIG lets the unit tests run this against a scratch file.
function configure_display_settings() {
    local config_file="${PIB_BOOT_CONFIG:-/boot/firmware/config.txt}"
    local setting

    require_nonempty config_file || return 1

    # Apply display and resolution settings if config file exists
    if [ ! -e "$config_file" ]; then
        print INFO "${config_file} does not exist; display settings were not changed"
        return 0
    fi

    ensure_full_kms_overlay "$config_file" || return 1

    # These firmware directives describe the 1024x600 HDMI panel. Under full KMS the kernel
    # takes the mode from the panel's EDID (disable_fw_kms_setup=1 is the shipped default), so
    # they are kept as they were and nothing here forces a mode or a connector.
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
    )
    for setting in "${!settingsMap[@]}"
    do
        if grep -q "$setting" "$config_file"; then
            sudo sed -i "/$setting/c\\${settingsMap[$setting]}" "$config_file"
        elif ! grep -q "$setting" "$config_file" && ! grep -q "${settingsMap[$setting]}" "$config_file"; then
            echo "${settingsMap[$setting]}" | sudo tee -a "$config_file" > /dev/null
        fi
    done
    print INFO "Adjusted display resolution and settings"
}

# PIB_BROWSER_DESKTOP_SOURCE and HOME let the unit tests point this at scratch files.
function configure_chromium_password_store() {
    local source_desktop="${PIB_BROWSER_DESKTOP_SOURCE:-/usr/share/applications/x-www-browser.desktop}"
    local browser_desktop="$HOME/.local/share/applications/x-www-browser.desktop"

    require_nonempty HOME source_desktop browser_desktop || return 1

    if [ ! -f "$source_desktop" ]; then
        print INFO "${source_desktop} does not exist; Chromium password store was not changed"
        return 0
    fi

    mkdir -p "$(dirname "$browser_desktop")" || return 1
    cp "$source_desktop" "$browser_desktop" || return 1
    if ! grep -q 'password-store=basic' "$browser_desktop"; then
        sed -i 's|^Exec=x-www-browser %U|Exec=x-www-browser --password-store=basic %U|' "$browser_desktop" || return 1
    fi
    update-desktop-database "$HOME/.local/share/applications" 2>/dev/null || true
    print INFO "Configured Chromium to use basic password store"
}

function configure_gnome_settings() {
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
}

function set_system_settings() {
    local status=0

    print INFO "Adjusting system settings"

    sudo systemctl daemon-reload
    sudo systemctl enable ssh --now

    if is_supported_raspbian; then
        configure_display_settings || status=1
    fi

    if is_supported_raspbian && [ "$DIST_VERSION" = "trixie" ]; then
        configure_chromium_password_store || status=1
    fi

    if is_ubuntu_noble; then
        configure_gnome_settings || status=1
    fi

    if [ "$status" -eq 0 ]; then
        print SUCCESS "System settings adjusted"
    else
        print ERROR "System settings were not fully adjusted"
    fi
    return "$status"
}

# PIB_SYSTEM_SETTINGS_DEFINE_ONLY=1 lets the unit tests source the functions without running them.
if [ "${PIB_SYSTEM_SETTINGS_DEFINE_ONLY:-0}" != "1" ]; then
    set_system_settings
fi
