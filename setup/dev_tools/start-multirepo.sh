#!/bin/bash
set -euo pipefail

PROJECT_NAME="multirepo"
ROLE="local"
REMOTE_IP=""
WITH_WEBOTS=false
WITH_ALL=false
REBUILD=false

print_help() {
  cat <<'EOF'
Usage:
  ./setup/dev_tools/start-multirepo.sh [options]

General options:
  --rebuild                 Build required services before start.
  --all                     In local mode, start full backend profile.
  -h, --help                Show help.

Local modes:
  --with-webots             Start local motors + webots stack (no cross-host relay).

Relay modes (recommended for cross-VM):
  --relay-source            Run source side (physical/controller side).
  --relay-webots            Run webots side (digital twin side).
  --remote-ip <IP>          Required for --relay-source. Webots VM IP.

Examples:
  ./setup/dev_tools/start-multirepo.sh
  ./setup/dev_tools/start-multirepo.sh --with-webots
  ./setup/dev_tools/start-multirepo.sh --relay-source --remote-ip 192.168.220.149
  ./setup/dev_tools/start-multirepo.sh --relay-webots
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --relay-source)
      ROLE="relay-source"
      shift
      ;;
    --relay-webots)
      ROLE="relay-webots"
      shift
      ;;
    --remote-ip)
      REMOTE_IP="${2:-}"
      shift 2
      ;;
    --with-webots)
      WITH_WEBOTS=true
      shift
      ;;
    --all)
      WITH_ALL=true
      shift
      ;;
    --rebuild)
      REBUILD=true
      shift
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      print_help
      exit 1
      ;;
  esac
done

if [[ ! -f "docker-compose.yaml" ]]; then
  echo "Error: run from repository root containing docker-compose.yaml"
  exit 1
fi

PROFILE_ARGS=()
SERVICE_ARGS=()
PRESERVE_ENV="COMPOSE_PROJECT_NAME"
export COMPOSE_PROJECT_NAME="$PROJECT_NAME"

if [[ "$ROLE" == "relay-source" ]]; then
  if [[ -z "$REMOTE_IP" ]]; then
    echo "Error: --remote-ip is required with --relay-source"
    exit 1
  fi
  export TRAJECTORY_RELAY_REMOTE_HOST="$REMOTE_IP"
  PRESERVE_ENV="COMPOSE_PROJECT_NAME,TRAJECTORY_RELAY_REMOTE_HOST"
  PROFILE_ARGS+=(--profile motors --profile relay_sender)
  SERVICE_ARGS+=(rosbridge-ws ros-motors trajectory-relay-sender)
elif [[ "$ROLE" == "relay-webots" ]]; then
  PROFILE_ARGS+=(--profile motors --profile pibsim_webots --profile relay_receiver)
  SERVICE_ARGS+=(rosbridge-ws ros-motors pibsim_webots trajectory-relay-receiver)
  if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
  fi
else
  PROFILE_ARGS+=(--profile motors)
  SERVICE_ARGS+=(rosbridge-ws ros-motors)
  if [[ "$WITH_WEBOTS" == true ]]; then
    PROFILE_ARGS+=(--profile pibsim_webots)
    SERVICE_ARGS+=(pibsim_webots)
    if command -v xhost >/dev/null 2>&1; then
      xhost +local:root >/dev/null 2>&1 || true
    fi
  fi
fi

if [[ "$WITH_ALL" == true ]]; then
  PROFILE_ARGS+=(--profile all)
  if [[ "$ROLE" == "local" ]]; then
    SERVICE_ARGS=()
  fi
fi

if [[ "$REBUILD" == true ]]; then
  sudo --preserve-env="$PRESERVE_ENV" docker compose -p "$PROJECT_NAME" build "${SERVICE_ARGS[@]}"
fi

sudo --preserve-env="$PRESERVE_ENV" docker compose -p "$PROJECT_NAME" \
  "${PROFILE_ARGS[@]}" up -d --force-recreate "${SERVICE_ARGS[@]}"

echo "Started project '$PROJECT_NAME' in mode '$ROLE'."
if [[ "$ROLE" == "relay-source" ]]; then
  echo "Relay source target: $TRAJECTORY_RELAY_REMOTE_HOST:15555"
fi
