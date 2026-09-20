"""Read-only registry for curated DepthAI model artefacts."""

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

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
    unavailable_reason: str
    input_width: int
    input_height: int
    composite: bool = False
    artifact_ids: Tuple[str, ...] = ()
    publish_topic: str = ""
    selectable: bool = True


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
    COMPOSITE_FIELDS = {
        "model_id",
        "task",
        "composite",
        "artifacts",
        "publish_topic",
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
            composite_entries = []
            invalid_artifacts = []
            for entry in entries:
                if isinstance(entry, dict) and entry.get("composite") is True:
                    composite_entries.append(entry)
                    continue
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
                functional = bool(entry.get("functional", True))
                unavailable_reason = str(entry.get("unavailable_reason", "")).strip()
                if not functional and not unavailable_reason:
                    raise ValueError(
                        f"non-functional model {model_id} has no unavailable_reason"
                    )
                artifact_valid = (
                    blob_path.is_file() and blob_path.stat().st_size == size_bytes
                )
                if not artifact_valid:
                    invalid_artifacts.append(model_id)
                loaded[model_id] = ModelRecord(
                    model_id=model_id,
                    task=str(entry["task"]),
                    licence=str(entry["licence"]),
                    shaves=int(entry["shaves"]),
                    size_bytes=size_bytes,
                    blob_path=str(blob_path),
                    available=functional and artifact_valid,
                    unavailable_reason=unavailable_reason,
                    selectable=bool(entry.get("selectable", True)),
                    input_width=int(entry["input_width"]),
                    input_height=int(entry["input_height"]),
                )

            for entry in composite_entries:
                if not self.COMPOSITE_FIELDS.issubset(entry):
                    raise ValueError("manifest contains an incomplete composite entry")
                model_id = str(entry["model_id"]).strip()
                artifact_ids = tuple(str(item).strip() for item in entry["artifacts"])
                if (
                    not model_id
                    or model_id in loaded
                    or not artifact_ids
                    or any(not artifact_id for artifact_id in artifact_ids)
                ):
                    raise ValueError("manifest contains an invalid composite entry")
                artifacts = [loaded.get(artifact_id) for artifact_id in artifact_ids]
                resolved = [artifact for artifact in artifacts if artifact is not None]
                loaded[model_id] = ModelRecord(
                    model_id=model_id,
                    task=str(entry["task"]),
                    licence="; ".join(
                        dict.fromkeys(artifact.licence for artifact in resolved)
                    ),
                    shaves=sum(artifact.shaves for artifact in resolved),
                    size_bytes=sum(artifact.size_bytes for artifact in resolved),
                    blob_path="",
                    available=(
                        len(resolved) == len(artifact_ids)
                        and all(artifact.available for artifact in resolved)
                    ),
                    unavailable_reason="",
                    input_width=0,
                    input_height=0,
                    composite=True,
                    artifact_ids=artifact_ids,
                    publish_topic=str(entry["publish_topic"]),
                    selectable=bool(entry.get("selectable", True)),
                )
            unavailable = [
                model.model_id for model in loaded.values() if not model.available
            ]
            if unavailable:
                message = "model store has unavailable models: " + ", ".join(
                    unavailable
                )
                # Preserve S4's all-or-empty behavior for legacy manifests.  A
                # composite manifest must remain queryable so callers can report
                # that the chain is unavailable when one dependency is missing.
                if invalid_artifacts and not composite_entries:
                    raise ValueError(message)
                self._warn_once(message)
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

    def selectable_models(self) -> Iterable[ModelRecord]:
        """Models a client may offer or start.

        Composite artefacts (for example the palm detector and the hand
        landmark network that ``hand_tracking`` is built from) stay in the
        registry because the chain is built from them, but they are not
        selectable entries of their own.
        """
        return [model for model in self._models.values() if model.selectable]

    def __len__(self):
        return len(self._models)
