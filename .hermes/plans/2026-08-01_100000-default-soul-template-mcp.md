# Plan: PR-1516 Default SOUL.md Template with Robot Name and MCP Tools Documentation

**Jira Ticket:** PR-1516  
**Repository:** `pib-backend`  
**Branch:** `PR-1516`  

## Overview
Update `ensure_profile()` in `public_api_client/public_api_client/hermes_agent_client.py` to generate a dynamic default `SOUL.md` whenever a personality is provisioned.

## Technical Tasks
1. Add `build_default_soul_text(personality_name: str, custom_description: Optional[str] = None) -> str` in `hermes_agent_client.py`.
2. Format `SOUL.md`:
   - Line 1: `Du bist der humanoide Roboter {personality_name}.`
   - Include custom description / prompt if provided.
   - Include detailed section `## Verfügbare MCP-Werkzeuge (pib_mcp_server)` describing `get_motor_currents`, `set_servo_angle`, `speak`, `get_bricklets`, `move_head`, `get_head_pose`.
3. Update `ensure_profile()` to write `SOUL.md` into the profile directory.
4. Add unit tests in `tests/unit/test_hermes_agent_client.py` asserting `SOUL.md` content and placeholder replacement.
