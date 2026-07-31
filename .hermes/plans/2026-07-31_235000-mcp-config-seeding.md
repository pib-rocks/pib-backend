# Implementation Plan: PR-1508 Auto-seed mcp_servers.pib

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1508  
**Branch:** `PR-1508`  

## Changes Needed

### 1. `setup/setup-pib.sh`
In `install_hermes_cli()`: After installing Hermes CLI, automatically append/merge `mcp_servers.pib` into `/home/pib/.hermes/config.yaml` using Python/PyYAML:
```bash
sudo -u pib -H python3 -c "
import yaml, os
cfg_path = '/home/pib/.hermes/config.yaml'
if os.path.exists(cfg_path):
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f) or {}
    if 'mcp_servers' not in cfg or 'pib' not in cfg.get('mcp_servers', {}):
        cfg.setdefault('mcp_servers', {})['pib'] = {'command': 'python3', 'args': ['-m', 'pib_mcp_server']}
        with open(cfg_path, 'w') as f:
            yaml.dump(cfg, f)
"
```

### 2. `public_api_client/public_api_client/hermes_agent_client.py`
In `_inherit_base_config(pdir)`:
Ensure `mcp_servers.pib` is merged into the profile's `config.yaml` if it is missing, even when `config.yaml` already exists in the profile.

### 3. Unit Tests
Add test in `tests/unit/test_hermes_profile_provisioning.py` verifying `mcp_servers.pib` is seeded into profile `config.yaml`.
