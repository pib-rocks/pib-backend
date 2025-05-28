#!/bin/bash

# This script is responsible to setup the Docker Engine + Containers to run Cerebra
# To properly run it, is relies on being sourced by the setup-pib.sh script
# Also see: https://github.com/docker/docker-install for a more comprehensive docker installation script


version_gte() {
	if [ -z "$VERSION" ]; then
			return 0
	fi
	version_compare "$VERSION" "$1"
}

# create a systemd-service that adds the 'local:'-host to the x-server's access-control-list on each startup,
# allowing applications from within a docker container to access the x-server on the host os
function create_xhost_service() {
    sudo mv "$BACKEND_DIR/setup/setup_files/xhost_enable_local.service" /etc/systemd/xhost_enable_local.service
    sudo chmod 700 /etc/systemd/xhost_enable_local.service
    sudo systemctl enable /etc/systemd/xhost_enable_local.service --now
}


# Installs the Docker Engine on supported linux distributions (ubuntu, debian, raspbian)
function install_docker_engine() {
    print INFO "Installing Docker Engine"

    local sh_c='sudo sh -c'
    if command_exists docker; then
        print WARN "Docker Engine already installed; skipping installation"
        return
    fi

    print INFO "Installing Docker Engine for ${DISTRIBUTION} ${DIST_VERSION}"
    print INFO "$USER"

    # Install Docker Engine
    apt_repo="deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/$DISTRIBUTION $DIST_VERSION stable"
    (
        $sh_c 'apt-get update -qq >/dev/null'
        $sh_c "DEBIAN_FRONTEND=noninteractive apt-get install -y -qq apt-transport-https ca-certificates curl >/dev/null"
        $sh_c 'install -m 0755 -d /etc/apt/keyrings'
        $sh_c "curl -fsSL \"https://download.docker.com/linux/$DISTRIBUTION/gpg\" -o /etc/apt/keyrings/docker.asc"
        $sh_c "chmod a+r /etc/apt/keyrings/docker.asc"
        $sh_c "echo \"$apt_repo\" > /etc/apt/sources.list.d/docker.list"
        $sh_c 'apt-get update -qq >/dev/null'
    )
    pkg_version=""
    if [ -n "$VERSION" ]; then
        pkg_pattern="$(echo "$VERSION" | sed 's/-ce-/~ce~.*/g' | sed 's/-/.*/g')"
        search_command="apt-cache madison docker-ce | grep '$pkg_pattern' | head -1 | awk '{\$1=\$1};1' | cut -d' ' -f 3"
        pkg_version="$($sh_c "$search_command")"
        if [ -z "$pkg_version" ]; then
            print ERROR "${VERSION} not found"
            return 1
        fi
        if version_gte "18.09"; then
            search_command="apt-cache madison docker-ce-cli | grep '$pkg_pattern' | head -1 | awk '{\$1=\$1};1' | cut -d' ' -f 3"
            cli_pkg_version="=$($sh_c "$search_command")"
        fi
        pkg_version="=$pkg_version"
    fi

    (
        pkgs="docker-ce${pkg_version%=}"
        if version_gte "18.09"; then
            pkgs="$pkgs docker-ce-cli${cli_pkg_version%=} containerd.io"
        fi
        if version_gte "20.10"; then
            pkgs="$pkgs docker-compose-plugin docker-ce-rootless-extras$pkg_version"
        fi
        if version_gte "23.0"; then
            pkgs="$pkgs docker-buildx-plugin"
        fi
        $sh_c "DEBIAN_FRONTEND=noninteractive apt-get install -y -qq $pkgs >/dev/null"
    )
    print SUCCESS "Docker Engine installed"
}


function start_container() {
    print INFO "Starting container"
    echo "TRYB_URL_PREFIX=https://tryb.ai" > "$BACKEND_DIR"/password.env
    sudo docker compose -f "$BACKEND_DIR/docker-compose.yaml" --profile all up -d || return 1
    print SUCCESS "Started pib-backend container"
    sudo docker compose -f "$FRONTEND_DIR/docker-compose.yaml" up -d || return 1
    print SUCCESS "Started cerebra container"
}

create_xhost_service || print ERROR "failed to create service for xhost permission management"
install_docker_engine || print ERROR "failed to install docker engine"
start_container || print ERROR "failed to start containers"
sudo chmod 777 "$BACKEND_DIR/pib_api/flask/pibdata.db"