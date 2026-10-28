#!/usr/bin/env bash
set -uo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="$(grep -E '^CURSOR_API_KEY=' /home/pib/.hermes/.env | cut -d= -f2- | tr -d '\"')"
echo "BRANCH_BEFORE=$(git branch --show-current)"
echo "BRANCH_HEAD=$(git rev-parse --short HEAD)"
echo "PROMPT_FILE=.hermes/plans/PR-1929-fix-prompt.txt"
echo "PROMPT_LINES=$(wc -l < "$PROMPT_FILE")"
echo "PROMPT_SHA256=$(sha256sum < "$PROMPT_FILE" | cut -c1-16)"
timeout 3000 cursor-agent --print --force --output-format text "$(cat "$PROMPT_FILE")"
echo "CURSOR_EXIT=$?"
echo "=== git status (nur versionierte Aenderungen):"; git status --short | grep -v '^??'
echo "=== diff stat:"; git diff --stat
echo "=== git log:"; git log --oneline -3
