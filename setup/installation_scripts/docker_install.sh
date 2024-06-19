command_exists() {
    command -v "$@" >/dev/null 2>&1
}

get_distribution() {
    lsb_dist=""
    # Every system that we officially support has /etc/os-release
    if [ -r /etc/os-release ]; then
        lsb_dist="$(. /etc/os-release && echo "$ID")"
    fi
    # Returning an empty string here should be alright since the
    # case statements don't act unless you provide an actual value
    echo "$lsb_dist"
}

function install_docker_engine() {
    print "Installing Docker Engine"

    local download_url="https://download.docker.com"
    local channel="stable"
    local sh_c='sh -c'
    if command_exists docker; then
        print WARN "Docker Engine already installed; skipping installation"
        #    return
    fi

    # Get OS/Linux distribution and check if compatible
    lsb_dist=$(get_distribution)
    lsb_dist="$(echo "$lsb_dist" | tr '[:upper:]' '[:lower:]')"

    if [[ "$lsb_dist" != "ubuntu" && "$lsb_dist" != "debian" && "$lsb_dist" != "raspbian" ]]; then
        print ERROR "Unsupported distribution: $lsb_dist"
        return 1
    fi

    # Get distribution version
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

    if [ -z "$dist_version" ]; then
        print ERROR "could not find version of linux distribution"
        return 1
    fi

    print "Installing Docker Engine for ${lsb_dist} ${dist_version}"

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
        if ! is_dry_run; then
            set -x
        fi
        $sh_c "DEBIAN_FRONTEND=noninteractive apt-get install -y -qq $pkgs >/dev/null"
    )
    print SUCCESS "Docker Engine installed"
}

function setup_cerebra() {
    print "Setup Cerebra via Docker Engine"
    print "Clone cerebra (${FRONTEND}), pib-backend (${BACKEND})"

}

#install_docker_engine || print ERROR "failed to install docker engine"
setup_cerebra