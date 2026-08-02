# PR-1524 — Optimize Hermes Agent Configuration for Maximum Speed with Gemini Flash (Lite)

Jira Ticket: https://pib-rocks.atlassian.net/browse/PR-1524
Category: Software
Branch: `PR-1524` (DO NOT MERGE TO DEVELOP)

## Goals
Define and seed high-speed Hermes Agent configuration defaults for Gemini 3.5 Flash and Gemini 3.5 Flash-Lite:
- `model`: `gemini-3.5-flash` / `gemini-3.5-flash-lite`
- `reasoning_effort`: `low`
- `max_tokens`: `1024`
- `temperature`: `0.3`
- `context_compress_threshold`: `0.7`

## Implementation Tasks

1. `public_api_client/public_api_client/hermes_agent_client.py` & `pib_hermes_config/pib_hermes_config/__init__.py`:
   - Define speed constants:
     - `DEFAULT_HERMES_MODEL = "gemini-3.5-flash"`
     - `DEFAULT_HERMES_LITE_MODEL = "gemini-3.5-flash-lite"`
     - `DEFAULT_REASONING_EFFORT = "low"`
     - `DEFAULT_MAX_TOKENS = 1024`
     - `DEFAULT_TEMPERATURE = 0.3`
   - In `ensure_profile` and base config seeding, ensure profile `config.yaml` carries `reasoning_effort: low`, `max_tokens: 1024`, `temperature: 0.3`.

2. Test Basis `docs/test-basis/PR-1524-hermes-speed-optimization.md`:
   - Document the 5 test cases from Jira ticket PR-1524.

3. Unit Tests `tests/unit/test_hermes_speed_optimization.py`:
   - Test speed configuration constants and profile seeding.
   - Verify `gemini-3.5-flash-lite` model string support.

## Constraints
- Work strictly on branch `PR-1524`.
- DO NOT MERGE TO DEVELOP.
- Commit all changes and push branch to `origin/PR-1524`.
