"""Read-only registry for curated DepthAI model artefacts."""

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Dict, Iterable, Optional

import yaml


@dataclass(frozen=True)
class ModelRecord:
    model_id: str
    task: str
    licence: str
    shaves: int
    size_bytes: int
    blob_path: str
    available: bool
    input_width: int
    input_height: int


class ModelRegistry:
    """Load and validate a model-store manifest without breaking the camera."""

    REQUIRED_FIELDS = {
        "model_id",
        "task",
        "licence",
        "file",
        "sha256",
        "size_bytes",
        "shaves",
        "input_width",
        "input_height",
        "format",
        "openvino_version",
        "notes",
    }

    def __init__(self, store_path=None, logger=None):
        self.store_path = Path(
            store_path or os.environ.get("PIB_MODEL_STORE", "/models")
        )
        self._logger = logger
        self._warned = False
        self._models: Dict[str, ModelRecord] = {}
        self._load()

    def _warn_once(self, message):
        if self._warned:
            return
        self._warned = True
        if self._logger is not None:
            self._logger.warning(message)

    def _load(self):
        manifest_path = self.store_path / "manifest.yaml"
        try:
            with manifest_path.open("r", encoding="utf-8") as manifest_file:
                manifest = yaml.safe_load(manifest_file)
            entries = manifest.get("models") if isinstance(manifest, dict) else None
            if not isinstance(entries, list):
                raise ValueError("manifest has no models list")

            loaded = {}
            for entry in entries:
                if not isinstance(entry, dict) or not self.REQUIRED_FIELDS.issubset(
                    entry
                ):
                    raise ValueError("manifest contains an incomplete model entry")
                model_id = str(entry["model_id"]).strip()
                relative_file = Path(str(entry["file"]))
                if (
                    not model_id
                    or relative_file.is_absolute()
                    or ".." in relative_file.parts
                    or model_id in loaded
                ):
                    raise ValueError("manifest contains an invalid model entry")
                blob_path = self.store_path / relative_file
                size_bytes = int(entry["size_bytes"])
                loaded[model_id] = ModelRecord(
                    model_id=model_id,
                    task=str(entry["task"]),
                    licence=str(entry["licence"]),
                    shaves=int(entry["shaves"]),
                    size_bytes=size_bytes,
                    blob_path=str(blob_path),
                    available=(
                        blob_path.is_file() and blob_path.stat().st_size == size_bytes
                    ),
                    input_width=int(entry["input_width"]),
                    input_height=int(entry["input_height"]),
                )
            unavailable = [
                model.model_id for model in loaded.values() if not model.available
            ]
            if unavailable:
                raise ValueError(
                    "model store is partial; unavailable artefacts: "
                    + ", ".join(unavailable)
                )
            self._models = loaded
        except Exception as exc:
            self._models = {}
            self._warn_once(
                f"Model store unavailable at {manifest_path}: {exc}. "
                "Continuing with an empty registry."
            )

    def get(self, model_id: str) -> Optional[ModelRecord]:
        return self._models.get(model_id)

    def models(self) -> Iterable[ModelRecord]:
        return self._models.values()

    def __len__(self):
        return len(self._models)
