# Test Basis: Automatic MCP Config Seeding

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1508  
**Components:** `setup/setup-pib.sh`, `hermes_agent_client.py`

## Requirement

`mcp_servers.pib` must be automatically configured in `/home/pib/.hermes/config.yaml` during host-side setup (`setup-pib.sh`) and ensured in every Hermes personality profile during provisioning (`hermes_agent_client.py`), so that fresh Pi deployments expose `pib_mcp_server` tools out-of-the-box.

## Acceptance Criteria Traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | `setup-pib.sh` automatically seeds `mcp_servers.pib` in `/home/pib/.hermes/config.yaml`. | Code inspection of `setup/setup-pib.sh` | Planned |
| AC2 | `hermes_agent_client.py` ensures `mcp_servers.pib` is present in profile `config.yaml` during provisioning. | `tests/unit/test_hermes_profile_provisioning.py` | Planned |
| AC3 | Unit tests verify automatic MCP config seeding for both fresh and existing profiles. | `tests/unit/test_hermes_profile_provisioning.py` | Planned |
