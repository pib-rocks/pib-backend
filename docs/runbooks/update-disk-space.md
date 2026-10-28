# Update disk space (PR-2110)

The host update runner (`setup/update_runner.sh`) budgets disk for the whole update (pull/build, containers, rollback headroom), reclaims Docker cache before building, and drops obsolete release images after a successful verify.

## Budget

Before container builds start, the runner runs `docker builder prune -af` and `docker image prune -f`, re-measures free space on the backend checkout filesystem, then compares it to a **requirement** (not a fixed 8 GiB floor).

| Variable | Default | Role |
|----------|---------|------|
| `PIB_UPDATE_REQUIRED_FREE_KIB` | (unset) | When set to a positive integer, overrides the computed requirement (KiB). |
| `PIB_UPDATE_IMAGE_SET_KIB` | `20971520` (20 GiB) | Expected size of the backend image set for one release. |
| `PIB_UPDATE_BUILD_CACHE_HEADROOM_KIB` | `4194304` (4 GiB) | Extra space reserved for Docker build cache during the update. |
| `PIB_UPDATE_MIN_FREE_KIB` | `8388608` (8 GiB) | Legacy floor; the computed requirement is `max(MIN_FREE_KIB, IMAGE_SET_KIB) + BUILD_CACHE_HEADROOM_KIB` unless overridden. |

Failure messages include required and available space in MiB and KiB and mention `PIB_UPDATE_REQUIRED_FREE_KIB`.

## Manual reclaim (operator)

If an update fails for disk space, or to free space before retrying:

```bash
docker builder prune -af
docker image prune -f
df -Pk /home/pib/app/pib-backend
```

Adjust the `df` path if `PIB_BACKEND_DIR` is set differently on the device.

After a **successful** update, the runner removes `ghcr.io/pib-rocks/pib-backend-*` images whose tag is neither the installed release nor the immediately previous release (dangling images are pruned as well). Current and previous tags are kept so a single rollback rebuild does not require re-pulling those images.
