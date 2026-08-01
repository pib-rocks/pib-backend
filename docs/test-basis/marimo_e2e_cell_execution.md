# Test Basis: Marimo E2E Notebook Creation and Cell Execution Test

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1513  
**Components:** `tests/e2e/test_programs_component_e2e.py`, Marimo iframe UI

## Requirement

Automated Playwright E2E UI test verifying:
1. Creation of a new notebook via the "New notebook" sidebar button.
2. Verification that the new notebook starts empty / clean.
3. Entering a Python script importing `time` and printing the current time.
4. Triggering cell execution in Marimo and verifying the output ("Hello World, current time:").
5. Cleaning up the test notebook afterwards.
