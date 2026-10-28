#!/usr/bin/env bash
set -uo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="$(grep -E '^CURSOR_API_KEY=' /home/pib/.hermes/.env | cut -d= -f2- | tr -d '"')"
PROMPT_FILE=".hermes/plans/PR-1955-prompt.txt"
[ -s "$PROMPT_FILE" ] || { echo "ABBRUCH: Prompt fehlt"; exit 2; }
echo "BRANCH_BEFORE=$(git branch --show-current)"
echo "BRANCH_HEAD=$(git rev-parse --short HEAD)"
echo "PROMPT_LINES=$(wc -l < "$PROMPT_FILE")"
echo "PROMPT_SHA256=$(sha256sum < "$PROMPT_FILE" | cut -c1-16)"
timeout 3600 cursor-agent --print --force --trust --output-format text "$(cat "$PROMPT_FILE")"
echo "CURSOR_EXIT=$?"
echo "=== git status:"; git status --short | grep -v '^??'
echo "=== diff stat:"; git diff --stat
echo "=== neue Migrationen:"; git status --short | grep -i migration | sed 's/^/   /'
echo "=== Spalte im Modell:"; grep -n "reasoning_effort" pib_api/flask/model/personality_model.py | sed 's/^/   /'
echo "=== Schreibposition im Client (muss der agent-Block sein):"
grep -n -B3 -A6 'reasoning_effort' public_api_client/public_api_client/hermes_agent_client.py | sed 's/^/   /'
echo "=== steht noch ein Schreibzugriff auf die Wurzel?"; grep -n 'cfg\["reasoning_effort"\]' public_api_client/public_api_client/hermes_agent_client.py || echo "   keiner mehr"
echo "=== DTO-Feld:"; grep -n "reasoning_effort\|reasoningEffort" pib_api/flask/schema/personality_schema.py | sed 's/^/   /'
echo "=== git log:"; git log --oneline -3
