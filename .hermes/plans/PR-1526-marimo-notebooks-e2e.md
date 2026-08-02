# PR-1526 — Robust Marimo Notebooks E2E UI Test with DOM Verification, CRUD & Code Execution

Jira Ticket: https://pib-rocks.atlassian.net/browse/PR-1526
Category: Software
Branch: `PR-1526` (DO NOT MERGE TO DEVELOP)

## Goals
Implement a comprehensive, strict Playwright E2E UI test for the Marimo Notebooks component (`/programs` -> Notebooks tab) that verifies:
1. Deep Iframe DOM Rendering: Asserts `iframe[src*='marimo-server']` is loaded and contains `.marimo-app` or `marimo-code-editor` elements inside the iframe context.
2. Create & Read (C & R): Clicks `"New notebook"` button, verifies `.py` file creation in list and opens it in editor.
3. Update & Code Execution (U & Run): Writes Python code into editor cell, executes it (`Shift+Enter` or Run button), and asserts the actual output string in the rendered DOM output node.
4. Delete (D): Deletes the notebook via UI and verifies removal from file list.

## Implementation Tasks
1. Test Basis `docs/test-basis/PR-1526-marimo-notebooks-e2e.md`:
   - Document all 5 test cases from Jira ticket PR-1526.
2. E2E Test Suite `tests/e2e/test_programs_component_e2e.py`:
   - Expand / update tests to include deep iframe DOM assertions, CRUD lifecycle, and code execution output verification.
3. Run pytest E2E tests against live Pi 5 (`192.168.1.28`).
4. Commit and push branch `PR-1526` to `origin/PR-1526`. DO NOT MERGE TO DEVELOP.
