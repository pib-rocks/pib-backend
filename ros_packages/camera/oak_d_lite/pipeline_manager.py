"""Serialized lifecycle manager for one DepthAI camera pipeline."""

from dataclasses import dataclass, field
import threading
import time
from typing import Callable, Dict, Iterable, Optional, Set, Tuple

from .model_registry import ModelRecord, ModelRegistry


@dataclass(frozen=True)
class ActiveModel:
    model: ModelRecord
    shaves: int


@dataclass
class ModelRuntime:
    owners: Set[str] = field(default_factory=set)
    shaves: int = 0
    state: str = "idle"
    message: str = ""
    fps: float = 0.0
    active: bool = False
    packet_count: int = 0
    fps_started_at: float = field(default_factory=time.monotonic)


class PipelineManager:
    """Reference-count model requests and serialize physical rebuilds."""

    def __init__(
        self,
        registry: ModelRegistry,
        rebuild: Callable[[Iterable[ActiveModel]], bool],
        verify_frames: Callable[[float], bool],
        revert_to_color: Callable[[], bool],
        on_change: Optional[Callable[[], None]] = None,
        logger=None,
        attempts: int = 2,
        frame_timeout: float = 3.0,
        backoff: float = 0.25,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.registry = registry
        self._rebuild = rebuild
        self._verify_frames = verify_frames
        self._revert_to_color = revert_to_color
        self._on_change = on_change
        self._logger = logger
        self._attempts = max(1, attempts)
        self._frame_timeout = frame_timeout
        self._backoff = backoff
        self._sleep = sleep
        self._clock = clock
        self._lock = threading.RLock()
        self._runtime: Dict[str, ModelRuntime] = {
            model.model_id: ModelRuntime(shaves=model.shaves)
            for model in registry.models()
        }

    def _notify(self):
        if self._on_change is not None:
            self._on_change()

    def _warn(self, message):
        if self._logger is not None:
            self._logger.warning(message)

    def _active_specs(self):
        specs = []
        for model in self.registry.models():
            runtime = self._runtime[model.model_id]
            if runtime.owners:
                specs.append(ActiveModel(model=model, shaves=runtime.shaves))
        return specs

    def _set_requested_state(self, state, message, active):
        for runtime in self._runtime.values():
            if runtime.owners:
                runtime.state = state
                runtime.message = message
                runtime.active = active
                if not active:
                    runtime.fps = 0.0

    def _rebuild_and_verify(self):
        for attempt in range(self._attempts):
            try:
                rebuilt = self._rebuild(self._active_specs())
                flowing = rebuilt and self._verify_frames(self._frame_timeout)
            except Exception:
                flowing = False
            if flowing:
                return True
            if attempt + 1 < self._attempts:
                self._sleep(self._backoff * (2**attempt))
        return False

    def _handle_failure(self, operation, model_id):
        try:
            self._revert_to_color()
        except Exception:
            pass
        self._set_requested_state(
            "failed",
            f"Pipeline {operation} failed; camera reverted to colour-only",
            False,
        )
        self._warn(
            f"Model {operation} failed for {model_id}; "
            "reverted to colour-only camera pipeline."
        )

    def start(self, model_id: str, shaves: int, owner: str) -> Tuple[bool, str]:
        with self._lock:
            model = self.registry.get(model_id)
            if model is None:
                return False, f"Unknown model: {model_id}"
            if not model.available:
                return False, f"Model artefact is unavailable: {model_id}"
            owner = owner.strip()
            if not owner:
                return False, "owner must not be empty"
            if shaves < 0:
                return False, "shaves must be zero or positive"
            if shaves not in (0, model.shaves):
                return (
                    False,
                    f"Model {model_id} is compiled for {model.shaves} shaves",
                )

            runtime = self._runtime[model_id]
            if owner in runtime.owners:
                if runtime.state in ("starting", "running"):
                    return True, f"Model {model_id} already requested by {owner}"
                added_owner = False
            else:
                added_owner = True

            had_owners = bool(runtime.owners)
            runtime.owners.add(owner)
            if had_owners and runtime.state == "running":
                self._notify()
                return True, f"Model {model_id} already running"

            runtime.shaves = shaves or model.shaves
            self._set_requested_state("starting", "Rebuilding camera pipeline", False)
            self._notify()
            if self._rebuild_and_verify():
                self._set_requested_state("running", "Model is running", True)
                self._notify()
                return True, f"Model {model_id} started"

            if added_owner:
                runtime.owners.discard(owner)
            self._handle_failure("start", model_id)
            runtime.state = "failed"
            runtime.message = "Pipeline start failed; camera reverted to colour-only"
            runtime.active = False
            runtime.fps = 0.0
            self._notify()
            return False, f"Failed to start model {model_id}; camera remains available"

    def stop(self, model_id: str, owner: str) -> Tuple[bool, str]:
        with self._lock:
            model = self.registry.get(model_id)
            if model is None:
                return False, f"Unknown model: {model_id}"
            owner = owner.strip()
            if not owner:
                return False, "owner must not be empty"

            runtime = self._runtime[model_id]
            if owner not in runtime.owners:
                return True, f"Model {model_id} was not requested by {owner}"

            runtime.owners.remove(owner)
            if runtime.owners:
                self._notify()
                return True, f"Model {model_id} remains in use"
            if runtime.state != "running":
                runtime.state = "idle"
                runtime.message = ""
                runtime.active = False
                runtime.fps = 0.0
                self._notify()
                return True, f"Model {model_id} stopped"

            runtime.state = "starting"
            runtime.message = "Rebuilding camera pipeline"
            runtime.active = False
            runtime.fps = 0.0
            self._set_requested_state("starting", "Rebuilding camera pipeline", False)
            self._notify()
            if self._rebuild_and_verify():
                runtime.state = "idle"
                runtime.message = ""
                self._set_requested_state("running", "Model is running", True)
                self._notify()
                return True, f"Model {model_id} stopped"

            runtime.state = "idle"
            runtime.message = ""
            self._handle_failure("stop", model_id)
            self._notify()
            return False, f"Failed to stop model {model_id}; camera remains available"

    def record_packet(self, model_id: str):
        with self._lock:
            runtime = self._runtime.get(model_id)
            if runtime is not None and runtime.active:
                runtime.packet_count += 1

    def refresh_fps(self):
        with self._lock:
            now = self._clock()
            for runtime in self._runtime.values():
                elapsed = now - runtime.fps_started_at
                if elapsed > 0:
                    runtime.fps = (
                        runtime.packet_count / elapsed if runtime.active else 0.0
                    )
                    runtime.packet_count = 0
                    runtime.fps_started_at = now

    def status(self, model_id: str):
        with self._lock:
            runtime = self._runtime[model_id]
            return {
                "active": runtime.active,
                "fps": runtime.fps,
                "shaves": runtime.shaves,
                "state": runtime.state,
                "message": runtime.message,
                "owners": frozenset(runtime.owners),
            }

    def statuses(self):
        with self._lock:
            return {
                model_id: {
                    "active": runtime.active,
                    "fps": runtime.fps,
                    "shaves": runtime.shaves,
                    "state": runtime.state,
                    "message": runtime.message,
                    "owners": frozenset(runtime.owners),
                }
                for model_id, runtime in self._runtime.items()
            }
