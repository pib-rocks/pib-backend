# Test Basis: Default SOUL.md Template with Name Placeholder and MCP Tools Documentation

**Jira Ticket:** PR-1516  
**Repository:** `pib-backend` (`public_api_client/public_api_client/hermes_agent_client.py`)  

## Requirements

1. **Automatic SOUL.md Seeding:**  
   When a new personality profile is provisioned in Hermes Agent (via `ensure_profile()` in `hermes_agent_client.py`), generate a structured `SOUL.md` in `~/.hermes/profiles/<profile_name>/SOUL.md`.

2. **Robot Identity Prefix:**  
   The `SOUL.md` must start with:  
   `Du bist der humanoide Roboter <Name-der-neuen-Persönlichkeit>.`  
   where `<Name-der-neuen-Persönlichkeit>` is dynamically substituted with the name of the personality.

3. **Detailed MCP Tools Documentation:**  
   The `SOUL.md` must contain detailed explanations of all available `pib_mcp_server` tools:
   - `mcp_pib_get_motor_currents`: Auslesen der aktuellen Motor-Ströme (mA).
   - `mcp_pib_set_servo_angle`: Ansteuern einzelner Servo-Gelenke.
   - `mcp_pib_speak`: Sprachausgabe über das Roboter-Audio-System.
   - `mcp_pib_get_bricklets`: Status-Abfrage der verbundenen Tinkerforge Bricklets.
   - `mcp_pib_get_head_pose` / `mcp_pib_move_head`: Abfrage und Bewegung der Kopf-Orientierung.

4. **Goal:**  
   Ensure any newly created Hermes Agent personality immediately knows its robot identity, name, and full capabilities out-of-the-box in conversational chat turns.
