#!/bin/bash
set -euo pipefail

if [[ ! -f "docker-compose.yaml" ]]; then
  echo "Error: run from repository root containing docker-compose.yaml"
  exit 1
fi

git fetch --all --prune
CURRENT_BRANCH="$(git branch --show-current)"
git pull --rebase origin "$CURRENT_BRANCH"

./setup/dev_tools/start-multirepo.sh "$@"
