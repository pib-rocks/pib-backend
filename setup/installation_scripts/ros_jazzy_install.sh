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

    # Source the locally-built pib datatypes overlay. This is guarded independently
    # from the Jazzy block above so re-running setup repairs an existing ~/.bashrc
    # that only has the Jazzy line, and the [ -f ] guard keeps it safe even before
    # the overlay has been built (e.g. on the first pass / if the build failed).
    if ! grep -q "ros_working_dir/install/setup.bash" "$HOME/.bashrc"; then
        {
            echo ""
            echo "# pib custom ROS 2 datatypes overlay (built by setup-pib.sh)"
            echo '[ -f "$HOME/ros_working_dir/install/setup.bash" ] && source "$HOME/ros_working_dir/install/setup.bash"'
        } >> "$HOME/.bashrc"
        print INFO "Added datatypes overlay sourcing to ~/.bashrc"
    else
        print INFO "datatypes overlay sourcing already present in ~/.bashrc"
    fi
}

function build_ros_datatypes() {
    if [ -z "$BACKEND_DIR" ]; then
        print ERROR "BACKEND_DIR is not set; cannot locate ros_packages/datatypes"
        return 1
    fi

    print INFO "Building pib ROS 2 datatypes"

    # ros-jazzy-ros-base ships the rosidl runtime but not the build-time tooling
    # needed to compile an interface package, so install those here.
    #
    # NOTE: Debian Trixie / Rospian does NOT provide the python3-colcon-common-extensions
    # metapackage, so we install the individual colcon packages that do exist. Without
    # python3-colcon-bash, colcon build would not generate install/setup.bash.
    sudo apt install -y \
        python3-colcon-core python3-colcon-ros python3-colcon-cmake \
        python3-colcon-bash python3-colcon-python-setup-py \
        python3-colcon-defaults python3-colcon-library-path \
        python3-colcon-package-information python3-colcon-package-selection \
        python3-colcon-parallel-executor python3-colcon-recursive-crawl \
        python3-colcon-output python3-colcon-metadata python3-colcon-notification \
        python3-colcon-test-result python3-colcon-argcomplete \
        ros-jazzy-ament-cmake \
        ros-jazzy-rosidl-default-generators \
        ros-jazzy-trajectory-msgs ros-jazzy-action-msgs \
        || { print ERROR "failed to install ROS 2 build tooling (colcon/ament)"; return 1; }

    if ! command -v colcon >/dev/null 2>&1; then
        print ERROR "colcon is not on PATH after installation; cannot build datatypes"
        return 1
    fi

    # Build in a separate workspace rather than in-place to keep the git checkout
    # free of build/, install/ and log/ artifacts.
    local ws="$HOME/ros_working_dir"
    mkdir -p "$ws/src"
    cp -r "$BACKEND_DIR/ros_packages/datatypes" "$ws/src/"

    source /opt/ros/jazzy/setup.bash
    ( cd "$ws" && colcon build --packages-select datatypes ) \
        || { print ERROR "colcon build of datatypes failed"; return 1; }

    if [ ! -f "$ws/install/setup.bash" ]; then
        print ERROR "datatypes build did not produce $ws/install/setup.bash"
        return 1
    fi

    # ~/.bashrc sourcing of this overlay is handled (idempotently and independently)
    # by configure_ros_jazzy_bashrc.

    print SUCCESS "Built pib ROS 2 datatypes"
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
        build_ros_datatypes || return 1
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
    build_ros_datatypes || return 1
    print SUCCESS "Installed ROS 2 Jazzy (ros-jazzy-ros-base)"
}

install_ros_jazzy
