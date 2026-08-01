# Test Basis: Synchronize SOUL.md Documentation with Real FastMCP Tools (PR-1530)

**Jira Story:** PR-1530  
**Repository:** `pib-backend`  

## Objectives & Scope
The `SOUL.md` template currently documents hypothetical tool names (`mcp_pib_get_motor_currents`, `mcp_pib_set_servo_angle`, `mcp_pib_speak`, etc.) which causes the LLM to attempt calling non-existent tools.

Update the `SOUL.md` template in `pib_hermes_config` to accurately document the 11 real tools exported by `pib_mcp_server`:
1. `mcp_pib_pib_list_motors`
2. `mcp_pib_pib_get_state`
3. `mcp_pib_pib_list_poses`
4. `mcp_pib_pib_list_programs`
5. `mcp_pib_pib_capture_image`
6. `mcp_pib_pib_move_motor`
7. `mcp_pib_pib_apply_pose`
8. `mcp_pib_pib_run_program`
9. `mcp_pib_pib_set_led`
10. `mcp_pib_pib_set_relay`
11. `mcp_pib_pib_soul_append`

## Acceptance Criteria
1. `build_default_soul_text()` in `pib_hermes_config` accurately documents the 11 real FastMCP tools.
2. `hermes_agent_client.py` imports and reuses `build_default_soul_text()` without duplicating hardcoded tool names.
3. Unit tests in `tests/unit/test_hermes_agent_client.py` and `tests/unit/test_personality_soul_sync.py` verify the updated tool list.
4. Existing profiles on the robot are updated during deployment so their `SOUL.md` matches the real FastMCP tools.
