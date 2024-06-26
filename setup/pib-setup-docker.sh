#!/bin/bash

# Color definitions for logging
export ERROR="\e[31m"
export WARN="\e[33m"
export SUCCESS="\e[32m"
export INFO="\e[36m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

# Github repositories
export FRONTEND="https://github.com/pib-rocks/cerebra.git"
export BACKEND="https://github.com/pib-rocks/pib-backend.git"
export APP_DIR="$HOME/app"
export BACKEND_DIR="$APP_DIR/pib-backend"
export FRONTEND_DIR="$APP_DIR/cerebra"

# Function to support printing consistent log messages
function print() {
    local color=$1
    local text=$2

    # If only one argument is provided, assume it is the text
    if [ -z "$text" ]; then
        text=$color
        color="RESET_TEXT_COLOR"
    fi

    # Check if the provided color exists
    if [ -n "$color" ] && [ -z "${!color}" ]; then
        color="RESET_TEXT_COLOR"
    fi

    # Print the text in the specified color
    echo -e "${!color}[[ ${text} ]]${RESET_TEXT_COLOR}"
}

function command_exists() {
    command -v "$@" >/dev/null 2>&1
}

get_distribution() {
    lsb_dist=""
    if [ -r /etc/os-release ]; then
        lsb_dist="$(. /etc/os-release && echo "$ID")"
    fi
    echo "$lsb_dist"
}

get_dist_version() {
  local lsb_dist=$1
  case "$lsb_dist" in

    ubuntu)
        if command_exists lsb_release; then
            dist_version="$(lsb_release --codename | cut -f2)"
        fi
        if [ -z "$dist_version" ] && [ -r /etc/lsb-release ]; then
            dist_version="$(. /etc/lsb-release && echo "$DISTRIB_CODENAME")"
        fi
        ;;

    debian | raspbian)
        dist_version="$(sed 's/\/.*//' /etc/debian_version | sed 's/\..*//')"
        case "$dist_version" in
        12)
            dist_version="bookworm"
            ;;
        11)
            dist_version="bullseye"
            ;;
        10)
            dist_version="buster"
            ;;
        9)
            dist_version="stretch"
            ;;
        8)
            dist_version="jessie"
            ;;
        esac
        ;;
    esac
    echo "$dist_version"
}



# function to clone pib repositories to APP_DIR directory
function clone_repositories() {
  # Validate branches
  if ! command_exists git; then
    print ERROR "git not found"
    exit 1
  fi

  if ! git ls-remote --exit-code --heads "$FRONTEND" "$BRANCH_FRONTEND" >/dev/null 2>&1; then
    print ERROR "Branch '${BRANCH_FRONTEND}' for Cerebra not found"
    exit 1
  fi
  if ! git ls-remote --exit-code --heads "$BACKEND" "$BRANCH_BACKEND" >/dev/null 2>&1; then
    print ERROR "Branch '${BRANCH_BACKEND}' for pib-backend not found"
    exit 1
  fi

  print INFO "Using branch '${BRANCH_FRONTEND}' for Cerebra, '${BRANCH_BACKEND}' for pib-backend"

  # Clone Repositories
  if [ ! -d "$APP_DIR" ]; then
    mkdir $APP_DIR
    print INFO "${APP_DIR} created"
  fi

  git clone --recurse-submodules -b "$BRANCH_BACKEND" $BACKEND "$BACKEND_DIR" || print WARN "pib-backend repository already exists"
  git clone --recurse-submodules -b "$BRANCH_FRONTEND" $FRONTEND "$FRONTEND_DIR" || print WARN "cerebra repository already exists"

  print SUCCESS "Completed cloning repositories to $APP_DIR"
}


echo "Hello $USER! We start the setup by allowing you permanently to run commands with admin-privileges."
if [[ "$(id)" == *"(sudo)"* ]]; then
	echo "For this change please enter your password..."
	sudo bash -c "echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
else
	echo "For this change please enter the root-password. It is most likely just your normal one..."
	su root bash -c "usermod -aG sudo $USER ; echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
fi


DISTRIBUTION=$(get_distribution) # e.g., 'ubuntu'
export DISTRIBUTION
DIST_VERSION=$(get_dist_version "$DISTRIBUTION")  # e.g., 'jammy'
export DIST_VERSION


# VALIDATE CLI ARGUMENTS
BRANCH_BACKEND="main"
BRANCH_FRONTEND="main"
  # Check if branch was specified
while [ $# -gt 0 ]; do
  case "$1" in
    -f=* | --frontendBranch=*)
      BRANCH_FRONTEND="${1#*=}"
      ;;
    -b=* | --backendBranch=*)
      BRANCH_BACKEND="${1#*=}"
      ;;
    -h | --help)
      show_help
      ;;
    *)
      print ERROR "invalid input options"
  esac
  shift
done


function install_docker_engine() {
    print INFO "Installing Docker Engine"

    local download_url="https://download.docker.com"
    local channel="stable"
    local sh_c='sudo sh -c'
    if command_exists docker; then
        print WARN "Docker Engine already installed; skipping installation"
        return
    fi

    # Get OS/Linux distribution and check if compatible
    lsb_dist=$DISTRIBUTION
    lsb_dist="$(echo "$lsb_dist" | tr '[:upper:]' '[:lower:]')"

    if [[ "$lsb_dist" != "ubuntu" && "$lsb_dist" != "debian" && "$lsb_dist" != "raspbian" ]]; then
        print ERROR "Unsupported distribution: $lsb_dist"
        return 1
    fi

    # Get distribution version
    local dist_version=$DIST_VERSION

    if [ -z "$dist_version" ]; then
        print ERROR "could not find version of linux distribution"
        return 1
    fi

    print INFO "Installing Docker Engine for ${lsb_dist} ${dist_version}"
    print INFO "$USER"

    # Install Docker Engine
    pre_reqs="apt-transport-https ca-certificates curl"
    apt_repo="deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] $download_url/linux/$lsb_dist $dist_version $channel"
    (
        $sh_c 'apt-get update -qq >/dev/null'
        $sh_c "DEBIAN_FRONTEND=noninteractive apt-get install -y -qq $pre_reqs >/dev/null"
        $sh_c 'install -m 0755 -d /etc/apt/keyrings'
        $sh_c "curl -fsSL \"$download_url/linux/$lsb_dist/gpg\" -o /etc/apt/keyrings/docker.asc"
        $sh_c "chmod a+r /etc/apt/keyrings/docker.asc"
        $sh_c "echo \"$apt_repo\" > /etc/apt/sources.list.d/docker.list"
        $sh_c 'apt-get update -qq >/dev/null'
    )
    pkg_version=""
    if [ -n "$VERSION" ]; then
        pkg_pattern="$(echo "$VERSION" | sed 's/-ce-/~ce~.*/g' | sed 's/-/.*/g')"
        search_command="apt-cache madison docker-ce | grep '$pkg_pattern' | head -1 | awk '{\$1=\$1};1' | cut -d' ' -f 3"
        pkg_version="$($sh_c "$search_command")"
        echo "INFO: Searching repository for VERSION '$VERSION'"
        echo "INFO: $search_command"
        if [ -z "$pkg_version" ]; then
            echo
            echo "ERROR: '$VERSION' not found amongst apt-cache madison results"
            echo
            exit 1
        fi
        if version_gte "18.09"; then
            search_command="apt-cache madison docker-ce-cli | grep '$pkg_pattern' | head -1 | awk '{\$1=\$1};1' | cut -d' ' -f 3"
            echo "INFO: $search_command"
            cli_pkg_version="=$($sh_c "$search_command")"
        fi
        pkg_version="=$pkg_version"
    fi

    (
        pkgs="docker-ce${pkg_version%=}"
        if version_gte "18.09"; then
            # older versions didn't ship the cli and containerd as separate packages
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
    echo "TRYB_URL_PREFIX=<TRYB_URL>" > "$BACKEND_DIR"/password.env
    echo "TRYB_API_TOKEN=<TRYB_API_TOKEN>" >> "$BACKEND_DIR"/password.env
    sudo docker compose -f "$BACKEND_DIR/docker-compose.yaml" --profile voice_assistant --profile motors --profile camera up -d || return 1
    print SUCCESS "Started pib-backend container"
    sudo docker compose -f "$FRONTEND_DIR/docker-compose.yaml" up -d || return 1
    print SUCCESS "Started cerebra container"
}

install_docker_engine || print ERROR "failed to install docker engine"
start_container || print ERROR "failed to start containers"
print SUCCESS "Installed Cerebra, files can be found at ${HOME}/app"
print INFO "For more information on how to use Cerebra with docker take a look at the documentation: https://pib-rocks.atlassian.net/wiki/spaces/kb/pages/176521254/Setting+up+pibs+software+with+Docker"