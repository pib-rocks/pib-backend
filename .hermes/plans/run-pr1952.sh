#!/usr/bin/env bash
set -uo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="$(grep -E '^CURSOR_API_KEY=' /home/pib/.hermes/.env | cut -d= -f2- | tr -d '\"')"
PROMPT_FILE=".hermes/plans/PR-1952-prompt.txt"
[ -s "$PROMPT_FILE" ] || { echo "ABBRUCH: Prompt fehlt"; exit 2; }
echo "BRANCH_BEFORE=$(git branch --show-current)"
echo "BRANCH_HEAD=$(git rev-parse --short HEAD)"
echo "PROMPT_LINES=$(wc -l < "$PROMPT_FILE")"
echo "PROMPT_SHA256=$(sha256sum < "$PROMPT_FILE" | cut -c1-16)"
timeout 3600 cursor-agent --print --force --trust --output-format text "$(cat "$PROMPT_FILE")"
echo "CURSOR_EXIT=$?"
echo "=== git status:"; git status --short | grep -v '^??'
echo "=== diff stat:"; git diff --stat
echo "=== Compose-Wert jetzt:"; grep -n "PIB_MCP_ENABLE_ACTUATION" docker-compose.yaml
echo "=== git log:"; git log --oneline -2
