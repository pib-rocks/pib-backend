# PR-1517 — Resolve SQLite Database Locking Under High Concurrency

Jira Ticket: https://pib-rocks.atlassian.net/browse/PR-1517
Category: Software
Branch: `PR-1517` (DO NOT MERGE TO DEVELOP)

## Goals
Fix `sqlite3.OperationalError: database is locked` 500 errors in Flask API under high concurrency by enabling SQLite Write-Ahead Logging (WAL) mode and configuring a 15-second busy timeout.

## Components & Implementation

1. `pib_api/flask/config.py`:
   - Set `SQLALCHEMY_ENGINE_OPTIONS = {"connect_args": {"timeout": 15.0}}`.

2. `pib_api/flask/app/app.py`:
   - Add SQLAlchemy `Engine` connect listener:
     - `PRAGMA journal_mode=WAL`
     - `PRAGMA busy_timeout=15000`

3. Unit & Concurrency Tests:
   - Implement `tests/unit/test_sqlite_concurrency.py`:
     - Test concurrent write and read operations across multiple threads.
     - Verify WAL mode and busy_timeout PRAGMAs are properly set on connections.
     - Ensure no `database is locked` operational errors occur.

## Constraints
- Stay on branch `PR-1517`.
- DO NOT MERGE TO DEVELOP.
- Commit all changes and push branch to `origin/PR-1517`.
