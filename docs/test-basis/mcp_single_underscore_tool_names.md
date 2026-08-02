# Test Basis: Simplify FastMCP Tool Names and Synchronize SOUL.md (PR-1533)

**Jira Story:** PR-1533  
**Repository:** `pib-backend`  

## Objectives & Scope

1. **Remove Duplicate `pib_` Function Prefix in FastMCP Server (`pib_mcp_server/server.py`):**
   Clean tool names in `server.py` by dropping redundant `pib_` prefixes:
   - `pib_list_motors` -> `list_motors`
   - `pib_get_state` -> `get_state`
   - `pib_list_poses` -> `list_poses`
   - `pib_list_programs` -> `list_programs`
   - `pib_capture_image` -> `capture_image`
   - `pib_move_motor` -> `move_motor`
   - `pib_apply_pose` -> `apply_pose`
   - `pib_run_program` -> `run_program`
   - `pib_set_led` -> `set_led`
   - `pib_set_relay` -> `set_relay`
   - `pib_soul_append` -> `soul_append`

2. **Synchronize `SOUL.md` Documentation (`pib_hermes_config` & `hermes_agent_client.py`):**
   Update `build_default_soul_text()` to document clean single-prefix names (e.g. `mcp__pib__list_poses` or `mcp_pib_list_poses`) matching the exact registered tool name schema.

3. **Testing & Verification:**
   - Update unit tests in `tests/unit/test_pib_mcp_server.py`, `tests/unit/test_hermes_agent_client.py`, and `tests/unit/test_personality_soul_sync.py`.
   - Re-run profile migration on deployment so existing profiles on the Pi are updated with clean tool names.
