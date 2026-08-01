# Implementation Plan: PR-1530 Synchronize SOUL.md with Real FastMCP Tools

**Jira Story:** PR-1530  
**Repository:** `pib-backend`  

## Implementation Steps

1. **Update `pib_hermes_config/pib_hermes_config/__init__.py`:**
   Update `build_default_soul_text()` to list all 11 real FastMCP tools exported by `pib_mcp_server`.

2. **Update Unit Tests:**
   Update assertions in `tests/unit/test_hermes_agent_client.py` and `tests/unit/test_personality_soul_sync.py` to check for `pib_list_motors`, `pib_get_state`, `pib_move_motor`, etc.

3. **Verify:**
   Run local pytest suite to ensure 100% pass rate.
