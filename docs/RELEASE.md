# pib-backend Release Process

`pib-backend` and `cerebra` share **one** version number and are released **as a
pair**. The release tag is the only place the backend/cerebra version is stated.
The required **pib-sdk** version is pinned separately in `pib_sdk_version.txt` at
the repository root (one line, e.g. `0.7.4`).

> ## Release order: **pib-sdk first**, then **pib-backend**, then **cerebra**
>
> **pib-sdk** is released on its own schedule. Before publishing pib-backend,
> ensure the pinned pib-sdk version already has a **published** GitHub release
> (not a draft or prerelease) so installers and Pi setup can resolve it.
>
> **pib-backend** is released next. At publish time the pairing guard can verify
> that the pinned pib-sdk release exists, because that release must already be
> available.
>
> **cerebra** is last. Its Docker build verifies that the matching pib-backend
> release already exists and **fails** if it does not. pib-backend cannot verify
> cerebra up front — at backend release time cerebra legitimately has no release
> yet — so the guard uses a grace period for cerebra only.

---

## The steps

### 0. Release pib-sdk (when the pin changes)

If `pib_sdk_version.txt` was bumped, publish that version in
[`pib-rocks/pib-sdk`](https://github.com/pib-rocks/pib-sdk) first. Verify:

```bash
# GitHub release v0.7.4 must be Published (not Draft / Pre-release)
pip index versions pib-sdk 2>/dev/null | head
```

### 1. Merge the work into `develop`

Feature branches are squash-merged into `develop` (pib-backend policy). Run the full
test suite on the Pi before continuing:

```bash
# on the Raspberry Pi
cd /home/pib/app/pib-backend && tests/run_all_tests.sh
```

Confirm `pib_sdk_version.txt` on `develop` matches the pib-sdk release you published
in step 0.

### 2. Publish the release **on `develop`**

`release-drafter` keeps a draft release up to date on every push to `develop`
(`commitish: refs/heads/develop` is pinned in `.github/workflows/release-drafter.yml`).
Open it under **Releases**, check notes and version, and press **Publish release**.

> **Pitfall:** a **draft** release creates **no git tag**. Only publishing does.

Verify:

```bash
git fetch origin --tags --force
git tag --points-at origin/develop        # -> v0.6.1
```

### 3. Merge `develop -> main`

```bash
git checkout main
git pull origin main
git merge --no-ff develop -m "Release v0.6.1: merge develop into main"
git push origin main
```

### 4. Release cerebra with the same version

Now follow `docs/RELEASE.md` in [`pib-rocks/cerebra`](https://github.com/pib-rocks/cerebra).
Its `develop -> main` merge triggers the Docker image build, which checks this
repository for the published release from step 2.

---

## Enforcement

The coupling is enforced on both sides:

| Repo | Mechanism | Behaviour |
|---|---|---|
| **cerebra** | `docker-build.yml`, step *Require the same published release in pib-backend* | **Hard block.** No image is built or pushed unless pib-backend has a *published* release with the same tag. |
| **pib-backend** | `.github/workflows/docker-images.yml` | On push to **`develop`** or a **published release**, builds and pushes arm64 service images to ghcr.io. |
| **pib-backend** | `release-pairing-guard.yml` + `setup/release_pairing_guard.py` | **Monitoring.** After a release (and hourly), checks that the pinned **pib-sdk** release exists and that cerebra followed with the same tag. Reports `PENDING` for missing counterparts for 120 minutes, then fails. Draft/prerelease pib-sdk releases fail immediately. |

Why asymmetric for cerebra: because pib-backend is released *before* cerebra, it
cannot verify cerebra up front — at that moment cerebra legitimately has no release
yet. Hence the grace period instead of an immediate failure for a missing cerebra
release. The pib-sdk leg has no such excuse: the pin in `pib_sdk_version.txt` names a
version that must already be published before the backend ships.

A local git `pre-push` hook is deliberately **not** used: hooks are not distributed
with a clone, are bypassable with `--no-verify`, and would need setting up on every
developer machine. The enforcement lives in CI.

### Manual check

```bash
python3 setup/release_pairing_guard.py verify v0.6.1
# or the legacy path:
python3 .github/scripts/verify_release_pairing.py v0.6.1
# exit 0 = pair complete (or still within grace) ; exit 1 = pair incomplete
```

Optional: set the repository secret `RELEASE_GUARD_TOKEN` to a token with read
access to raise the GitHub API rate limit from 60/h to 5000/h. Both repos are
public, so the check also works without it.

---

## Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| `PENDING - pib-sdk has no published release v…` | pib-sdk was not released before the backend | Publish pib-sdk first, or fix `pib_sdk_version.txt` |
| `pib-sdk v… exists but is still a DRAFT` | SDK release was drafted, not published | Publish it in pib-sdk (guard fails immediately) |
| `Release pair INCOMPLETE: cerebra has no release v…` | cerebra was never released after this backend release | Release cerebra with the same tag, or withdraw this release |
| `cerebra v… exists but is still a DRAFT` | cerebra's release was drafted, not published | Publish it in cerebra |
| cerebra build: `pib-backend has no release v…` | Release order was inverted | Publish pib-backend first, then re-run the cerebra workflow |
| Release notes exist but no tag | The release is still a **draft** | Press *Publish release* |
| `required pib_sdk_version.txt is missing` | Pin file absent on the release branch | Add `pib_sdk_version.txt` before publishing |

### Rollback

If cerebra cannot be released after pib-backend already was, either finish the cerebra
release or **withdraw** the pib-backend release (delete the release *and* the tag) so
the two repositories stay in step. Do not leave a published backend release without its
cerebra counterpart — that is exactly the drift this process prevents.

---

## Docker service images (ghcr.io)

CI on **`ubuntu-24.04-arm`** builds every backend compose service image for
**linux/arm64** and pushes them to GitHub Container Registry. The workflow is
`.github/workflows/docker-images.yml`.

| Trigger | Immutable tag | Moving tag | `APP_VERSION` in `flask-app` |
|---|---|---|---|
| Push to **`develop`** | `:<git-sha>` | `:develop` | `develop` |
| **Published release** | `:vX.Y.Z` (release tag) | — | same as release tag |
| **`workflow_dispatch`** | `:<git-sha>` or `:vX.Y.Z` when the input ref is a tag | `:develop` only when the input ref is the **`develop`** branch | `develop` or release tag |

Image names: `ghcr.io/pib-rocks/pib-backend-<service>` (for example
`ghcr.io/pib-rocks/pib-backend-flask-app:develop`).

Each matrix job keeps its own registry build cache at
`ghcr.io/pib-rocks/pib-backend-<service>-cache:buildcache`. The workflow uses
`strategy.fail-fast: true` so a failed service build does not leave a half-published
release set.

### Robot update path

`docker-compose.yaml` names every built service with
`${PIB_IMAGE_PREFIX:-ghcr.io/pib-rocks/pib-backend}-<service>:${PIB_IMAGE_TAG:-develop}`.
The `build:` sections are unchanged: `docker compose build` on the Pi still works
when ghcr.io is unreachable or the tag was never published.

`setup/update_runner.sh` resolves `PIB_IMAGE_TAG` from the update’s
`APP_VERSION` (release tag or the `develop` marker), then:

1. **`docker compose pull --profile all`** when registry pulls are enabled
   (`PIB_USE_REGISTRY_IMAGES` defaults to on).
2. On success, **`up -d --no-build --force-recreate`** (no on-Pi image build).
3. On pull failure or when pulls are disabled, the existing path: refresh Dockerfile
   bases with `build --pull`, build `flask-app` with `--build-arg APP_VERSION`, then
   `up -d --build --force-recreate`.

Rollback (`build_backend_stack rollback`) skips registry pull and base-image pull,
so offline rollback keeps working.

Opt out of registry pulls (force local build):

```bash
export PIB_USE_REGISTRY_IMAGES=0
```

Override image coordinates without editing the compose file:

```bash
export PIB_IMAGE_PREFIX=ghcr.io/pib-rocks/pib-backend
export PIB_IMAGE_TAG=develop   # or v0.6.4 after a release publish
```

### What still builds on the robot

- **Cerebra** (`/home/pib/app/cerebra/docker-compose.yaml`) is unchanged; only
  pib-backend service images come from ghcr.io.
- **Hermes agent sources** are not in the image: `hermes-daemon` still mounts the
  filtered Hermes view from the host at runtime; CI clones
  [NousResearch/hermes-agent](https://github.com/NousResearch/hermes-agent) only to
  satisfy the Dockerfile `hermes` build context.
- **External** images (`matter-server`, `homeassistant`) and **pibsim_webots** are
  not part of this pipeline.

---

## Notes

- CI publishes backend service images; the notes below about on-Pi **`docker compose
  build`** describe the **fallback** path when pulls are disabled or fail.
- On-Pi `flask-app` builds inject the release tag into `GET /api/version` via
  `--build-arg APP_VERSION`. After the `develop -> main` merge (two parents),
  the merge commit's second parent is `develop`, where the release tag lives.
  A fast-forward of `main` has no second parent; the same tag is then on `HEAD`.

```bash
docker compose build --build-arg APP_VERSION="$(git tag --points-at HEAD^2)" flask-app
```

  `setup/update_runner.sh` does this after it fetches both checkouts. It reads
  `git tag --points-at HEAD^2` and, when that commit does not exist, `git tag
  --points-at HEAD`, then runs the command above (`-f` pointing at the backend
  checkout, because the runner does not `cd` into it) before
  `up -d --build --force-recreate`. The same value is exported as `APP_VERSION`
  for that rebuild: `up --build` interpolates `${APP_VERSION:-v0.6.2}` from the
  environment, and without the export it would replace the image just built
  with the compose-file fallback.

  Develop-channel checkouts are not tagged. The runner passes the marker
  `develop` (`APP_VERSION=develop`) so the image does not keep that fallback
  and `/api/version` does not report an old release for new code. A release
  checkout with no tag on either commit is refused rather than built with the
  fallback.
- The per-package `setup.py` versions (`pib_api/client`, `pib_mcp_server`, …) are
  unrelated to the release version and are not touched by this process.
- `pib-sdk` is **not** part of the backend/cerebra version guard on the cerebra
  side; only this repository's pairing guard checks the pin in `pib_sdk_version.txt`.
