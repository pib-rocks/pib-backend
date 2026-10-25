from pathlib import Path
import os

REPO_ROOT = Path(__file__).resolve().parents[1]
COMPOSE_FILE = REPO_ROOT / "docker-compose.yaml"

# CI exercises the bundled STT decode self-check; production containers default it off.
os.environ.setdefault("PIB_STT_SELF_CHECK", "1")
