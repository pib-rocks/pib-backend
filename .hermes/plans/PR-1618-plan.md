# PR-1618: repo-wide black reformat + pin black (pib-backend)

Ticket: https://pib-rocks.atlassian.net/browse/PR-1618
Branch: PR-1618 (off develop)

## Goal

1. Run `black` over the whole repository once, commit it as a PURE formatting
   commit (no behavioural change).
2. Add `pyproject.toml` with a `[tool.black]` section and pin an exact black
   version in `.github/workflows/black.yml` (currently `psf/black@stable`,
   floating => caused the drift).
3. Make the `Lint` workflow green again.

## Measured baseline (verified)

- `python3 -m black --check .` -> 58 files "would be reformatted", ~1768 diff lines
- Lint workflow red in 100/100 inspected runs (since >= 2026-08-01)
- No `pyproject.toml`, `setup.cfg`, `tox.ini`, `.flake8` exists
- `.github/workflows/black.yml` uses `psf/black@stable` (no pin), `actions/checkout@v3`
- Local black version: 26.5.1

## Decisions (from the ticket's recommendations; treated as decided unless told otherwise)

- Alembic migrations (`pib_api/flask/migrations/versions/`): REFORMAT them
  (ordinary Python, never regenerated in place; excluding creates a second style island).
- Pin: EXACT version (`black==<version>`). Do not float.
- `ros_packages/`: reformat everything black touches. If any file is genuinely
  generated/build-produced, you may exclude it ONLY with a justifying comment in
  `[tool.black]` `exclude =` — but first verify it is actually generated, do not assume.

## Phase 1 — Configuration first (pyproject.toml + workflow pin)

Tasks:
1. Create `pyproject.toml` at repo root with:
   ```toml
   [tool.black]
   line-length = 88
   target-version = ['py311']   # verify what the tests/Pi actually run first
   ```
   Do NOT add an `exclude` unless you find genuinely generated files (Phase 2 check).
2. Edit `.github/workflows/black.yml`: pin the black action to an exact release.
   The action `psf/black@stable` cannot take a version param — instead switch to:
   ```yaml
   # standard GitHub Actions PYTHON setup, then pip install black==26.5.1 and run
   - name: Install black
     run: pip install black==26.5.1
   - name: Run black --check
     run: python -m black --check . --diff
   ```
   (adjust the exact `black==` to what the repo's Python uses; 26.5.1 is what local
   produced the 58-file count with, so it is the fair baseline).
   Keep `on: [ push ]`. Keep it failing on non-zero exit so the job is meaningful.
3. Do NOT run black over the tree yet — Phase 2 does that.

## Phase 2: reformat + single pure-format commit

Tasks:
1. Run `python3 -m black .` at the repo root (formats everything in place).
2. `git diff --stat` (i.e. `git diff --stat` after staging, or `git show --stat HEAD`)
   and confirm ONLY whitespace/line-wrapping changes — the diff must contain NO
   string, import, logic, dependency or comment-content changes.
3. Commit as ONE commit, conventional message, e.g.:
   `style(python): apply black formatting repo-wide (58 files) — formatting only`
4. The commit MUST be separate from the Phase 1 config commit, so blame can be
   ignored via `.git-blame-ignore-revs`.

## Phase 3 — self-check

1. `python3 -m black --check .` -> must exit 0 ("1 file would be left unchanged"
   style: "All done, N files would be left unchanged.").
2. `git show --stat HEAD` and diff review to prove formatting only.
3. Spot-check the failure-table: does `.github/workflows/black.yml` still trigger
   on push and fail on non-zero?

## Constraints (HARD)

- Stay on branch `PR-1618`. Do NOT push. Do NOT merge to develop.
- You MAY commit on `PR-1618` (conventional messages).
- Do NOT change any non-whitespace code. No refactoring, no bug fixes, no imports.
- If `black` fails to parse a file (some files use Python <3.8 syntax that black
  rejects), report the file and the error — do NOT silently drop it.
- Do not run the full backend test suite on the Pi — not needed for pure format,
  and it is slow/costly. It is verified separately.
- Report the REAL final `black --check .` output verbatim (file counts), and the
  exact commit SHA.

## Deliverables from the phase

- Final summary: `black --check .` result (exit 0 + count), the commit SHA(s),
  which files were reformatted (count), and any file black could not handle.