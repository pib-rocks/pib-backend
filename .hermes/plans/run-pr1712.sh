#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
cursor-agent --trust --model auto --output-format stream-json --stream-partial-output -p "$(cat .hermes/plans/PR-1712-prompt.txt)" 2>&1 | tail -30
