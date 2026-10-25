#!/usr/bin/env bash
set -uo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="$(grep -E '^CURSOR_API_KEY=' /home/pib/.hermes/.env | cut -d= -f2- | tr -d '\"')"
echo "BRANCH_BEFORE=$(git branch --show-current)"
echo "PROMPT_LINES=$(wc -l < .hermes/plans/PR-1921-prompt.txt)"
timeout 2400 cursor-agent --print --force --output-format text "$(cat .hermes/plans/PR-1921-prompt.txt)"
echo "CURSOR_EXIT=$?"
echo "=== git status:"; git status --short
echo "=== git log:"; git log --oneline -4
