# Implementation Plan: PR-1513 E2E UI Test for Marimo Notebook Creation and Execution

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1513  
**Branch:** `PR-1513`  

## Objective
Add a new Playwright E2E UI test `test_09_create_notebook_write_time_program_and_run_in_marimo` in `tests/e2e/test_programs_component_e2e.py`:
1. Navigate to `/program/marimo`.
2. Click `[data-test="BTN_New notebook"]` in the right sidebar.
3. Type notebook name `pib_time_demo` into modal and save.
4. Verify notebook is created and selected (`/program/marimo/pib_time_demo.py`).
5. Frame-switch into `app-marimo iframe` and verify empty / initial state.
6. Write a Python snippet importing `time` and printing "Hello World, current time: <timestamp>".
7. Execute cell in Marimo (via run button / shortcut / API).
8. Assert output "Hello World, current time:" appears in cell output.
9. Cleanup test notebook `pib_time_demo.py`.
