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

## Notes

- pib-backend has **no** Docker build pipeline: its containers are built on the Pi
  from source (`docker compose build <service> && docker compose up -d`).
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
