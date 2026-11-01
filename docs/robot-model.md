# Robot 3D model on the Flask API

Cerebra loads the pib URDF viewer from the robot itself so the machine in front of the user
matches the firmware, without fetching assets from the public internet.

## Directory

The bundle lives in `ROBOT_MODEL_DIR`:

| Environment | Path |
|-------------|------|
| Docker (`flask-app` in `docker-compose.yaml`) | `/data/robot-model` (persistent host volume under `/home/pib/app/data`) |
| Local dev (default) | `pib_api/flask/robot_model` |

Expected layout after install:

```
version.json
model.json
mapping.json
meshes/*.glb
```

`version.json` carries the release label (`version`) and optional per-file sha256 entries.
The meshes and JSON files are produced by [pib-3d](https://github.com/pib-rocks/pib-3d)
(`tools/build_web_assets.py`).

## HTTP routes

All routes are under `/robot-model`:

| Route | Description |
|-------|-------------|
| `GET /robot-model/manifest.json` | Index: version, build stamp, mesh list with sizes and URLs |
| `GET /robot-model/model.json` | Kinematic tree |
| `GET /robot-model/mapping.json` | Joint mapping |
| `GET /robot-model/meshes/<name>` | One GLB (`name` must match `[A-Za-z0-9._-]+\\.glb`) |

When nothing is installed, JSON routes return `404` with
`{"error":"no robot model installed","hint":"run tools/install_robot_model.py"}`.

Caching: append `?v=<version>` (the value from `version.json`) to enable
`Cache-Control: public, max-age=31536000, immutable`. Without a matching `v`, responses use
`no-cache`. All file responses include an `ETag` (mtime and size); send `If-None-Match` to
receive `304 Not Modified`.

## Install

From the repository root, with a release zip built by pib-3d:

```bash
python3 tools/install_robot_model.py --from /path/to/pib-3d-web-assets.zip
```

Download and verify:

```bash
python3 tools/install_robot_model.py \
  --url https://example.com/pib-3d-web-assets-v0.1.1.zip \
  --sha256 <hex-digest-of-zip>
```

Override the target directory:

```bash
python3 tools/install_robot_model.py --from assets.zip --dest /data/robot-model
```

The installer checks the zip digest (when `--sha256` is given), unpacks beside the target,
validates required files, then swaps the directory atomically. Any failure exits non-zero and
leaves the previous installation untouched.

## Update

Install again with a newer zip (same commands as above). The tool replaces the whole directory
in one rename, so clients should refetch `manifest.json` without `?v=` (or with a new `v`) after
an update.
