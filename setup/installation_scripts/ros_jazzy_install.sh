#!/bin/bash

# Installs ROS 2 Jazzy on Raspberry Pi OS Trixie via the Rospian APT repository.
# Relies on being sourced by setup-pib.sh (print, is_supported_raspbian, DIST_VERSION).

function configure_ros_jazzy_bashrc() {
    if ! grep -q "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc"; then
        {
            echo ""
            echo "# ROS 2 Jazzy (installed by setup-pib.sh)"
            echo "source /opt/ros/jazzy/setup.bash"
            echo "# ROS_LOCALHOST_ONLY restricts DDS to localhost. Leave unset so the host CLI can"
            echo "# discover ROS nodes in Docker containers. Uncomment the line below on shared"
            echo "# networks to avoid talking to other robots (see README clustering section):"
            echo "# export ROS_LOCALHOST_ONLY=1"
        } >> "$HOME/.bashrc"
        print INFO "Added ROS 2 Jazzy sourcing to ~/.bashrc"
    else
        print INFO "ROS 2 Jazzy sourcing already present in ~/.bashrc"
    fi
}

function install_ros_jazzy() {
    if ! is_supported_raspbian || [ "$DIST_VERSION" != "trixie" ]; then
        print WARN "ROS 2 Jazzy (Rospian) is only supported on Raspberry Pi OS Trixie; skipping"
        return 0
    fi

    if [ "$(dpkg --print-architecture)" != "arm64" ]; then
        print WARN "ROS 2 Jazzy (Rospian) requires arm64; skipping"
        return 0
    fi

    if [ -f /opt/ros/jazzy/setup.bash ]; then
        print WARN "ROS 2 Jazzy already installed; skipping installation"
        configure_ros_jazzy_bashrc
        return 0
    fi

    print INFO "Installing ROS 2 Jazzy (Rospian)"

    local rospian_list="/etc/apt/sources.list.d/rospian.list"
    local rospian_key="/usr/share/keyrings/rospian-archive-keyring.gpg"

    if [ ! -f "$rospian_key" ]; then
        curl -fsSL https://rospian.github.io/rospian-repo/public/rospian-archive-keyring.asc \
            | gpg --dearmor | sudo tee "$rospian_key" >/dev/null
        print INFO "Added Rospian GPG key"
    fi

    if [ ! -f "$rospian_list" ]; then
        echo "deb [arch=arm64 signed-by=$rospian_key] https://rospian.github.io/rospian-repo trixie-jazzy main" \
            | sudo tee "$rospian_list" > /dev/null
        print INFO "Added Rospian APT repository"
    fi

    sudo apt update -qq
    sudo apt install -y ros-jazzy-ros-base

    configure_ros_jazzy_bashrc
    print SUCCESS "Installed ROS 2 Jazzy (ros-jazzy-ros-base)"
}

install_ros_jazzy
