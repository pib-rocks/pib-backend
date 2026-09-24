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

function setup_docker_cleaner_service() {
    print INFO "Setting up Docker container cleanup service"
    sudo cp "$BACKEND_DIR/setup/setup_files/docker_cleaner.service" /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable docker_cleaner.service
    local start_output
    if ! start_output=$(sudo systemctl start docker_cleaner.service 2>&1); then
        print ERROR "failed to start docker_cleaner.service: ${start_output}"
        return 1
    fi
    print SUCCESS "Docker container cleanup service installed and started"
}

function setup_update_watchdog_helper() {
    print INFO "Setting up update watchdog helper"
    sudo install -o root -g root -m 0755 \
      "$BACKEND_DIR/setup/setup_files/pib-update-watchdog.sh" \
      /usr/local/sbin/pib-update-watchdog || return 1

    local sudoers_temp
    sudoers_temp="$(mktemp)" || return 1
    printf '%s\n' \
      'pib ALL=(root) NOPASSWD: /usr/local/sbin/pib-update-watchdog' \
      > "$sudoers_temp"
    if ! sudo visudo -c -f "$sudoers_temp"; then
        rm -f "$sudoers_temp"
        print ERROR "invalid sudoers rule for update watchdog helper"
        return 1
    fi
    if ! sudo install -o root -g root -m 0440 \
      "$sudoers_temp" /etc/sudoers.d/pib-update-watchdog; then
        rm -f "$sudoers_temp"
        return 1
    fi
    rm -f "$sudoers_temp"
    print SUCCESS "Update watchdog helper installed"
}

function setup_update_service() {
    print INFO "Setting up host-side update service"
    # setgid (2770) so files created by the root flask container inherit the pib
    # group; together with 0660 in the backend's atomic write this is what lets
    # the runner (User=pib) read request.json at all.
    sudo install -d -o pib -g pib -m 2770 /home/pib/app/.update
    sudo install -o root -g root -m 0644 \
      "$BACKEND_DIR/setup/setup_files/pib-update.service" \
      /etc/systemd/system/pib-update.service
    sudo install -o root -g root -m 0644 \
      "$BACKEND_DIR/setup/setup_files/pib-update.path" \
      /etc/systemd/system/pib-update.path
    sudo install -o root -g root -m 0644 \
      "$BACKEND_DIR/setup/setup_files/pib-update-check.service" \
      /etc/systemd/system/pib-update-check.service
    sudo install -o root -g root -m 0644 \
      "$BACKEND_DIR/setup/setup_files/pib-update-check.path" \
      /etc/systemd/system/pib-update-check.path
    # The runner's executable bit lives in git (mode 100755). Do NOT chmod it
    # here: the runner resets the checkout to the remote revision on every
    # update, so a locally granted bit would be a permanent dirty file that
    # blocks the next update - and after the reset ExecStart could not execute
    # the runner at all.
    sudo systemctl daemon-reload

    local enable_output
    if ! enable_output=$(sudo systemctl enable pib-update.path 2>&1); then
        print ERROR "failed to enable pib-update.path: ${enable_output}"
        return 1
    fi
    if ! enable_output=$(sudo systemctl enable pib-update-check.path 2>&1); then
        print ERROR "failed to enable pib-update-check.path: ${enable_output}"
        return 1
    fi
    local start_output
    if ! start_output=$(sudo systemctl start pib-update.path 2>&1); then
        print ERROR "failed to start pib-update.path: ${start_output}"
        return 1
    fi
    if ! start_output=$(sudo systemctl start pib-update-check.path 2>&1); then
        print ERROR "failed to start pib-update-check.path: ${start_output}"
        return 1
    fi
    # Marker the backend checks: without it the API reports runner_missing and
    # refuses to queue an update, because docker creates the bind-mount point for
    # a missing host directory on its own - a bare directory is not a runner.
    local marker
    marker="$(mktemp)"
    printf '{"schemaVersion":1,"installedAt":"%s","runner":"%s","updateCheck":true}\n' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
        "$BACKEND_DIR/setup/update_runner.sh" > "$marker"
    sudo install -o pib -g pib -m 0664 "$marker" /home/pib/app/.update/service.json
    rm -f "$marker"
    print SUCCESS "Host-side update service installed and watching for requests"
}

function verify_vendored_blockly() {
    print INFO "Verifying vendored pib-blockly sources"

    local blockly_blocks="$BACKEND_DIR/pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/custom-blocks.ts"
    if [ ! -f "$blockly_blocks" ]; then
        print ERROR "vendored pib-blockly is missing required files at ${blockly_blocks}"
        return 1
    fi

    print SUCCESS "Vendored pib-blockly sources found"
}

function start_container() {
    print INFO "Starting container"
    echo "TRYB_URL_PREFIX=https://platform.tryb.ai" > "$BACKEND_DIR"/password.env
    sudo PIB_HARDWARE_VARIANT="$PIB_HARDWARE_VARIANT" \
      docker compose -f "$BACKEND_DIR/docker-compose.yaml" --profile all up -d --build \
      || return 1
    print SUCCESS "Started pib-backend container"
    sudo docker compose -f "$FRONTEND_DIR/docker-compose.yaml" up -d || return 1
    print SUCCESS "Started cerebra container"
}

install_docker_engine || print ERROR "failed to install docker engine"
# The docker group only exists after the engine is installed, and
# docker_cleaner.service runs as User=pib, so the membership has to be granted
# before that unit is started.
sudo usermod -aG docker pib || { print ERROR "failed to add user 'pib' to docker group"; return 1; }
verify_vendored_blockly || print ERROR "failed to verify vendored pib-blockly sources"
start_container || print ERROR "failed to start containers"
setup_docker_cleaner_service || print ERROR "failed to setup docker cleaner service"
setup_update_watchdog_helper || print ERROR "failed to setup update watchdog helper"
setup_update_service || print ERROR "failed to setup host-side update service"
sudo chmod 777 "$BACKEND_DIR/pib_api/flask/pibdata.db"