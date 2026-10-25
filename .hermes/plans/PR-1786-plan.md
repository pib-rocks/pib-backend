# PR-1786 — Jest cannot run on a fresh clone (truncated vendored node_modules)

Repo: `pib-rocks/pib-backend`, working branch: **`PR-1786`** (from `origin/develop`).
Ticket: Jira **PR-1786** (project pib.rocks), Bug, status *Bereit*. The ticket holds the measured
evidence; this file holds the code-level facts and the required change.

## Measured facts (all verified in this working copy)

```
.gitignore:13  build/
.gitignore:35  build/            <- bare pattern: matches a directory named build at ANY depth

$ git check-ignore -v tests/blockly_generator/node_modules/jest-cli/build/index.js
.gitignore:35:build/    tests/blockly_generator/node_modules/jest-cli/build/index.js

$ git ls-files tests/blockly_generator/node_modules | wc -l   -> 4436
$ find tests/blockly_generator/node_modules -type f | wc -l   -> 7387
$ du -sh tests/blockly_generator/node_modules                 -> 100M   (the missing part is ~8 MiB)

$ git ls-files tests/blockly_generator/node_modules/jest-cli
LICENSE  README.md  bin/jest.js  package.json        <- no build/

# consequence on a fresh clone (measured on a freshly installed pib5edu):
# tests/run_all_tests.sh -> FAILED: Jest (Blockly generators)
Error: Cannot find module '/work/tests/blockly_generator/node_modules/jest-cli/build/index.js'.
   Please verify that the package.json has a valid "main" entry

# while a long-lived working copy (full node_modules) passes:
$ node node_modules/jest/bin/jest.js --config jest.config.js
Test Suites: 12 passed, 12 total / Tests: 64 passed, 64 total
```

Affected packages include `jest-cli`, `jest-config`, `jest-circus`, `jest-watcher`,
`jest-environment-node`, `jest-validate`, `create-jest`, `human-signals` and others - every vendored
package whose compiled output lives in `build/`.

The runner's guard cannot repair this: `run_jest()` only checks
`node_modules/jest/bin/jest.js`, which *is* tracked, so it never installs the missing parts.

## Required change

**1. `.gitignore` - scope the build rule so the vendored tree can be complete.**

The existing rules must stay effective everywhere else (e.g.
`pib_blockly/pib_blockly_server/node_modules/**` and `pib_blockly/pib_blockly_server/dist/**` stay
ignored, and a top-level `build/` for build outputs stays ignored). Add negations for the vendored
test tree, e.g.

```
# vendored test tree must be complete, so compiled package output is tracked there
!tests/blockly_generator/node_modules/**/build/
!tests/blockly_generator/node_modules/**/dist/
```

Verify with `git check-ignore -v <path>` that the negations actually take effect (git cannot
re-include a file whose *parent directory* is excluded - check it, do not assume) and that no other
path changes its ignore state (`git status` in a clean checkout must stay clean afterwards).

**2. Commit the missing files** so a fresh clone has a working Jest: `git add` the previously
ignored files under `tests/blockly_generator/node_modules/` (the ~8 MiB that are present on disk).
Do not "fix" this by deleting the vendored tree - the vendoring is a deliberate offline decision
and that policy question is explicitly out of scope here.

**3. Harden the runner guard as a safety net** in `tests/run_all_tests.sh`, `run_jest()`:

```bash
local jest_cmd='set -e
if [ ! -f node_modules/jest/bin/jest.js ]; then npm install --silent --no-audit --no-fund; fi
node node_modules/jest/bin/jest.js --config jest.config.js'
```

Replace the single file check with a *functional* check - does Jest actually start? - and install
when it does not, e.g. run `node node_modules/jest/bin/jest.js --version` inside the same command
and `npm install` when that fails. Keep the existing behaviour otherwise (same docker fallback, same
`--config jest.config.js`, no new dependencies, no changes to other steps).

## Verification you must run and report verbatim

```bash
bash -n tests/run_all_tests.sh
git check-ignore -v tests/blockly_generator/node_modules/jest-cli/build/index.js   # expect: no rule
git ls-files tests/blockly_generator/node_modules | wc -l                          # expect ~7387
git status --short                                                                 # no unexpected changes

# the decisive proof: a FRESH CLONE of this branch must be able to run Jest
rm -rf /tmp/jest-freshclone && git clone --quiet --branch PR-1786 "$PWD" /tmp/jest-freshclone
cd /tmp/jest-freshclone/tests/blockly_generator
node node_modules/jest/bin/jest.js --config jest.config.js      # expect 12 suites / 64 tests passed
# (reproduce the OLD failure first on origin/develop's state if you want the red/green contrast)
```

The functional check of the runner change (`run_jest` repairing an intentionally broken
`node_modules`) can be shown by moving `node_modules/jest-cli/build` aside in the fresh clone and
running the new command - report what you actually observed.

## Hard constraints

- Files you may touch: `.gitignore`, `tests/run_all_tests.sh`, plus adding the vendored files.
- Stay on branch `PR-1786`; do NOT merge, push, rebase or open a PR.
- Do not remove the vendored tree, do not change the compose files, do not reformat.
- English in the commit message; report real output only.
