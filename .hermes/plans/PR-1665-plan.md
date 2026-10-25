# PR-1665: Jest pose_generator can't resolve 'blockly' in run_all_tests.sh (Pi / Docker)

Ticket: https://pib-rocks.atlassian.net/browse/PR-1665
Repo: pib-backend — /home/pib/opencode/pib-backend, branch `PR-1665` (from develop `1c2e3b60`).
Instruction: implement; ON SUCCESS open a pull-request PR-1665 -> develop.

## Root cause (verified)
Locally (host node) `cd tests/blockly_generator && node node_modules/jest/bin/jest.js pose_generator` PASSES
(8/8). The failure occurs only under `tests/run_all_tests.sh` `run_jest()`, which runs Jest **inside a Docker
container**:
```
run_jest() {
  cd "${SCRIPT_DIR}/blockly_generator"
  ...
  docker run --rm -v "${REPO_ROOT}:/work" -w "/work/tests/blockly_generator" node:18-bookworm \
     node node_modules/jest/bin/jest.js ... 
}
```
In that container, Jest cannot resolve `blockly` for `program-blocks/pose-block.ts` ("Cannot find module 'blockly'
from .../pose-block.ts"). The `jest.config.js` maps `^blockly/(.*)$` but top-level `import * as Blockly from "blockly"`
is not resolved because the container's node resolution misses the host `node_modules/blockly` (root-owned / only
partially visible), or uses incompatible binaries under node:18.

## Goal
Make `run_jest()` (pib-backend tests/run_all_tests.sh) reliably resolve `blockly` so all 5 Blockly suites pass
when run via the full suite on the Pi / in Docker. The repo code itself is already correct (host jest is green).

## Options (choose/implement the robust one; prefer minimal, deterministic)
1. Install deps inside the Jest container before running: e.g. before `node node_modules/jest...` run
   `npm ci` (or at least ensure `node_modules` resolve) inside `-w /work/tests/blockly_generator` with the
   container's node, so `blockly` is present/consistent for node:18. This may need `--unsafe-perm`/ownership care
   since host node_modules is root-owned.
2. Switch the Jest run to the **host** node (like the local green run) instead of the `node:18` Docker container —
   if Docker isn't strictly required. Keep the docker path if it must stay, but make it resolve.
3. Add an explicit `moduleDirectories: ["node_modules", "<rootDir>/node_modules"]` to `jest.config.js` so top-level
   `blockly` resolves regardless of cwd/env, and combine with dep install in the container.

Implement whichever is deterministic and keeps all 5 suites green when run via `run_all_tests.sh` on the Pi.

## Hard constraints
1. Stay on branch `PR-1665`. Do NOT merge/push develop (orchestrator opens PR). You MAY commit (`fix: PR-1665 ...`).
2. Do NOT change the block/generator sources (they pass). Only fix the test runner / resolution so the full suite
   resolves `blockly`.
3. Do NOT commit `.hermes/plans/**`. Do NOT commit if it's not verified (host jest must still be green; and ideally
   the docker path too).
4. Report REAL results.

## Verify (orchestrator)
- Host: `cd tests/blockly_generator && node node_modules/jest/bin/jest.js --config jest.config.js` -> 5 suites pass.
- If feasible: `tests/run_all_tests.sh --skip-docker --skip-robot --include-frontend`... no — run the jest/blockly part
  and confirm pose_generator passes inside the docker path as well.
- `git diff --stat origin/develop...HEAD` shows tests/run_all_tests.sh (and maybe jest.config.js).