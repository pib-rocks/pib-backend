# Robot Framework tests for pib-backend

Black-box HTTP tests for the pib-backend Flask REST API, written with
[Robot Framework](https://robotframework.org/) and
[robotframework-requests](https://github.com/MarketSquare/robotframework-requests).

The first suite, [`personality_crud.robot`](personality_crud.robot), exercises the full CRUD
lifecycle of the Voice Assistant Personality endpoint (`/voice-assistant/personality`). It was
chosen as the first candidate because it is pure database CRUD: deterministic, self-contained, with
no hardware, external-network, or filesystem side effects. The suite is non-destructive - it creates
its own personality, exercises it, and deletes it, leaving the seeded data intact.

## Prerequisites

1. A running, seeded Flask API instance on `http://localhost:5000`.

   Using Docker Compose (from the repo root):

   ```bash
   docker compose up flask-app
   ```

   Or running it directly (from `pib_api/flask/`):

   ```bash
   flask --app run db upgrade
   flask --app run seed_db
   flask --app run run
   ```

2. The test dependencies installed (ideally in a virtual environment):

   ```bash
   pip install -r tests/robot/requirements.txt
   ```

## Running the tests

From the repo root:

```bash
robot --variable BASE_URL:http://localhost:5000 tests/robot/personality_crud.robot
```

`BASE_URL` defaults to `http://localhost:5000`, so the `--variable` flag is only needed when the API
runs on a different host or port. Robot Framework writes `log.html`, `report.html`, and `output.xml`
to the current directory; use `--outputdir` to redirect them, e.g.:

```bash
robot --outputdir tests/robot/results tests/robot/personality_crud.robot
```

## Layout

- `requirements.txt` - pinned Robot Framework and RequestsLibrary versions.
- `resources/api_keywords.robot` - shared HTTP session setup and reusable keywords.
- `personality_crud.robot` - the personality CRUD suite.

## Extending

The harness in `resources/api_keywords.robot` is reusable for additional API suites. Good next
candidates are `/program`, `/pose`, and `/voice-assistant/chat`.
