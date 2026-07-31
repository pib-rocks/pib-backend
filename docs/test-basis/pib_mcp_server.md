# Test Basis: pib MCP Server

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1506  
**Components:** `pib_mcp_server`, Hermes CLI MCP integration, voice-assistant personality profiles

## Requirement

A dedicated Model Context Protocol (MCP) server `pib_mcp_server` exposes pib's robot capabilities as schema-validated tools for Hermes voice agent profiles, eliminating the need for terminal/shell access or raw prompt instructions.

## Acceptance Criteria Traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | `pib_mcp_server` starts standalone and `hermes mcp test pib` discovers all declared tools. | `tests/unit/test_pib_mcp_server.py` & live `hermes mcp test` | Planned |
| AC2 | Every tool declares a JSON schema; invalid parameters are rejected before reaching hardware. | `tests/unit/test_pib_mcp_server.py::test_invalid_schema_rejection` | Planned |
| AC3 | `pib_move_motor` validates requested position against min/max limits and rejects out-of-range values. | `tests/unit/test_pib_mcp_server.py::test_motor_position_clamping` | Planned |
| AC4 | Actuating tools (move/pose/program/relay) are disabled by default and require explicit enablement. | `tests/unit/test_pib_mcp_server.py::test_actuating_tools_disabled_by_default` | Planned |
| AC5 | Voice agent can move a motor end-to-end through MCP with `terminal` toolset disabled. | Integration test | Planned |
| AC6 | `pib_capture_image` returns a valid image; verified by per-channel std / Laplacian variance. | `tests/unit/test_pib_mcp_server.py::test_capture_image` | Planned |
| AC7 | `pib_soul_append` is exposed as an MCP tool, append-only and size-capped (<= 500 chars). | `tests/unit/test_pib_mcp_server.py::test_soul_append_mcp_tool` | Planned |
| AC8 | Interim `pib-robot-control` skill is updated to point to the MCP server. | In-repo skill docs | Planned |
| AC9 | MCP config can be added/removed per personality profile. | `tests/unit/test_pib_mcp_server.py::test_per_profile_mcp_config` | Planned |
| AC10 | Unit & integration tests exercise the running server over transport. | `tests/unit/test_pib_mcp_server.py` | Planned |
| AC11 | Full pib-backend suite passes; legacy personalities remain functional. | Suite run | Planned |
| AC12 | New requirement documented under `docs/test-basis/` and mapped to tests. | This document | Documented |
| AC13 | Runbook documents MCP registration/deregistration and tool toggling. | `docs/runbooks/pib-mcp-server.md` | Planned |
