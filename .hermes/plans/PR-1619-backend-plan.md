# PR-1619: single build-injected system version (backend + frontend)

Ticket: https://pib-rocks.atlassian.net/browse/PR-1619
Two repos involved:
- pib-backend: version endpoint + backend Dockerfile ARG (this plan covers the BACKEND phase)
- cerebra:      frontend display + frontend Dockerfile + docker-build.yml ARG injection

Decision (from ticket, sharpened): **Option B - direct build injection on the Pi**.
The backend endpoint is served by a build-injected value; NO GHCR build is required
for the endpoint. GHCR is deliberately OUT of scope.

## BACKEND phase (this file, repo pib-backend, branch PR-1619)

### Goal
Add GET /api/version -> { "version": "<APP_VERSION>" } served by a value injected
at Docker build time.

### Verified facts (do not re-derive)
- Backend Dockerfile: ./pib_api/flask/Dockerfile (FROM python:3.10-slim, WORKDIR /app,
  copies flask code to /app, CMD ["flask","--app","run","run"]).
- Blueprint pattern: controllers live in pib_api/flask/controller/*_controller.py, each
  defines `bp = Blueprint(...)` and is registered in pib_api/flask/app/__init__.py with
  app.register_blueprint(<name>.bp, url_prefix=..., name=...).
- Real example: ip_controller.py has `@bp.route("", methods=["GET"])` returning jsonify
  with 200; registered as url_prefix="/host-ip" (reachable at /api/host-ip via nginx
  rewrite ^/api/(.*)$ /$1 break).
- nginx (cerebra repo) proxies location /api -> flask-app:5000 and strips /api.
  So a blueprint registered at url_prefix="/version" is reachable at /api/version.
- app/__init__.py imports controllers (there is an import list at top), then registers.

### Tasks
1. Add ARG APP_VERSION to ./pib_api/flask/Dockerfile. It must NOT have a hardcoded
   default (a GHCR/local build that forgets it should fail loudly). Bake it to a file
   the endpoint reads. Place the bake near the top (after WORKDIR /app) so the file
   exists in the image:
       ARG APP_VERSION
       RUN printf 'APP_VERSION="%s"\n' "$APP_VERSION" > /app/version.py
   IMPORTANT: the image also runs db upgrade/seed at build; adding version.py must not
   disturb that. version.py must be generated BEFORE the flask source is copied/looked
   at? No - put the RUN anywhere before CMD; the file is just read at runtime.
2. Add a new controller pib_api/flask/controller/version_controller.py:
       from flask import Blueprint, jsonify
       bp = Blueprint("version_controller", __name__)
       @bp.route("", methods=["GET"])
       def get_version():
           from app.version import APP_VERSION   # /app is on sys.path as app package? VERIFY import path
           return jsonify({"version": APP_VERSION}), 200
   NOTE the import path: /app contains the flask source (app/ subdir). The generated
   /app/version.py is a MODULE at package root. Check whether "from version import
   APP_VERSION" or "import version" is correct given how app.app is imported. Prefer
   a direct, safe read: `import version` then `version.APP_VERSION` (absolute import),
   falling back to a file read if imports are fragile. Keep it simple and robust.
3. Register the blueprint in pib_api/flask/app/__init__.py:
   - add version_controller to the import list
   - app.register_blueprint(version_controller.bp, url_prefix="/version", name="version")
4. Local/dev default: when building WITHOUT --build-arg APP_VERSION, the ARG is empty.
   Make the endpoint never crash: if version.py missing or APP_VERSION empty, return
   {"version": "unknown"} rather than 500. (GHCR build always passes the real tag.)
5. Update the pib-backend runbook/RELEASE.md note (only if it exists; docs/RELEASE.md):
   document the on-Pi build command:
       docker compose build --build-arg APP_VERSION="$(git tag --points-at HEAD^2)" flask-app
6. DO NOT add any GHCR workflow. DO NOT add a version bump of package.json / setup.py.

### Self-check
- `python3 -m py_compile` on the new controller and edited files.
- Import-safe: `cd pib_api/flask && python3 -c "import version_controller"` should not
  crash (needs deps; if deps missing, at least py_compile).
- Confirm registration compiles (app/__init__.py edit is syntactically valid).

### Constraints (HARD)
- Stay on branch PR-1619. Do NOT push. Do NOT merge to develop.
- Do NOT add GHCR/docker-build workflow to pib-backend (out of scope for the endpoint).
- No behavioural change beyond the new endpoint. No reformat of other files.
- plain python, no new runtime deps.

### Report back (real)
- Files changed, the endpoint code, the Dockerfile ARG/bake snippet.
- How you resolved the version.py import path.
- The exact `docker compose build --build-arg ... flask-app` command for release.
- Real py_compile result.
