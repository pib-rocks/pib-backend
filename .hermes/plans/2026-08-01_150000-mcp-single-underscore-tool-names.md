# Implementation Plan: PR-1533 Simplify FastMCP Tool Names and Synchronize SOUL.md

**Jira Story:** PR-1533  
**Repository:** `pib-backend`  

## Execution Steps

1. **`pib_mcp_server/server.py`:**
   Remove `pib_` prefix from tool function names:
   `list_motors`, `get_state`, `list_poses`, `list_programs`, `capture_image`, `move_motor`, `apply_pose`, `run_program`, `set_led`, `set_relay`, `soul_append`.

2. **`pib_hermes_config/pib_hermes_config/__init__.py`:**
   Update `build_default_soul_text()` to document the clean tools (`mcp__pib__list_poses`, `mcp__pib__get_state`, etc. and single-underscore `mcp_pib_list_poses` aliases).

3. **Unit Tests:**
   Update `tests/unit/test_pib_mcp_server.py`, `tests/unit/test_hermes_agent_client.py`, and `tests/unit/test_personality_soul_sync.py`.

4. **Deployment & Verification:**
   Rebuild containers on the Pi and verify `hermes -p pib_<id> -z "Welche Posen hast du?"` returns the live poses (`Startup/Resting`, `Calibration`).
