# pib-backend Release Process

`pib-backend` and `cerebra` share **one** version number and are released **as a
pair**. The release tag is the only place the version is stated — there is no
version file to bump.

> ## Release order: pib-backend FIRST, cerebra second
>
> pib-backend is always released first. cerebra is the consumer: its Docker build
> verifies that the matching pib-backend release already exists and **fails** if it
> does not. That only works if the backend goes first.

---

## The steps

### 1. Merge the work into `develop`

Feature branches are squash-merged into `develop` (pib-backend policy). Run the full
test suite on the Pi before continuing:

```bash
# on the Raspberry Pi
cd /home/pib/app/pib-backend && tests/run_all_tests.sh
```

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
| **pib-backend** | `release-pairing-guard.yml` + `.github/scripts/verify_release_pairing.py` | **Monitoring.** After a release (and hourly), checks that cerebra followed. Reports `PENDING` for 120 minutes, then fails. |

Why asymmetric: because pib-backend is released *first*, it cannot verify cerebra up
front — at that moment cerebra legitimately has no release yet. Hence the grace
period instead of an immediate failure.

A local git `pre-push` hook is deliberately **not** used: hooks are not distributed
with a clone, are bypassable with `--no-verify`, and would need setting up on every
developer machine. The enforcement lives in CI.

### Manual check

```bash
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
| `Release pair INCOMPLETE: cerebra has no release v…` | cerebra was never released after this backend release | Release cerebra with the same tag, or withdraw this release |
| `cerebra v… exists but is still a DRAFT` | cerebra's release was drafted, not published | Publish it in cerebra |
| cerebra build: `pib-backend has no release v…` | Release order was inverted | Publish pib-backend first, then re-run the cerebra workflow |
| Release notes exist but no tag | The release is still a **draft** | Press *Publish release* |

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
  the merge commit's second parent is `develop`, where the release tag lives:

```bash
docker compose build --build-arg APP_VERSION="$(git tag --points-at HEAD^2)" flask-app
```
- The per-package `setup.py` versions (`pib_api/client`, `pib_mcp_server`, …) are
  unrelated to the release version and are not touched by this process.
