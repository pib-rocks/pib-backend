# Test Basis: Setup Logs Audit for Package Warnings and Deprecations (PR-1522)

**Jira Story:** PR-1522  
**Repositories:** `pib-backend`, `cerebra`, `pib-sdk`  

## Objective & Scope

Analyze the setup and build log files (`setup-pib.sh`, `docker compose build`, `npm install / ng build`, `pip install`) across the pib robot stack to identify:
1. Deprecated libraries and framework packages.
2. Build/compiler warning messages.
3. Outdated dependencies or security advisories.

## Output Requirements (No Code Modification in PR-1522)
From the audit findings, create 3 separate follow-up tickets for the actual implementation:
1. **Cerebra Ticket:** Deprecated Angular build packages (`@ngtools/webpack`, `@angular-devkit/build-angular`, `bootstrap@4.6.2`, `uuid@8.3.2`, `flag-icon-css`).
2. **pib-sdk Ticket:** Outdated/deprecated Python dependencies and build requirements.
3. **pib-backend Ticket:** System Python / Flask / ROS / pip warnings and deprecated dependencies.
