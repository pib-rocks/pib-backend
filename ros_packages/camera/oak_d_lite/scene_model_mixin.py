"""Scene-model wiring for the camera node.

Kept out of ``stereo.py`` so the node module stays under its line budget; the
model itself is pure Python in :mod:`oak_d_lite.scene_model`.
"""

from __future__ import annotations

import json

from std_msgs.msg import String

from .scene_model import SceneModel
from .scene_model_projection import (
    intrinsics_from_calibration_matrix,
    point3d_for_detection_bbox,
)

SCENE_MODEL_TOPIC = "/scene_model"
SCENE_MODEL_PUBLISH_HZ = 2.0
SCENE_MODEL_TIMER_PERIOD_SECONDS = 1.0 / SCENE_MODEL_PUBLISH_HZ


class SceneModelMixin:
    """Publishes a persistent scene snapshot and answers ``get_scene``."""

    def _init_scene_model(self) -> None:
        self.scene_model = SceneModel()
        self._oak_device = None
        self.scene_model.set_intrinsics_metadata(None, "pipeline_not_started")
        self.scene_model_publisher_ = self.create_publisher(
            String, SCENE_MODEL_TOPIC, 10
        )
        self.scene_model_timer = self.create_timer(
            SCENE_MODEL_TIMER_PERIOD_SECONDS, self._publish_scene_model
        )
        from std_srvs.srv import Trigger

        self.get_scene_service = self.create_service(
            Trigger, "get_scene", self.get_scene_callback
        )
        # The camera pipeline is started before this method runs, so its
        # calibration is available now; a later rebuild loads it again.
        if getattr(self, "pipeline", None) is not None:
            self._load_scene_model_intrinsics_once()

    def _stamp_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _scene_detection_dicts(self, detections) -> list[dict]:
        intrinsics = self.scene_model.projection_intrinsics()
        depth = getattr(self, "current_depth", None)
        depth_available = bool(getattr(self, "depth_available", False))
        rows = []
        for detection in detections:
            bbox = (
                int(detection.x_min),
                int(detection.y_min),
                int(detection.x_max),
                int(detection.y_max),
            )
            point3d = point3d_for_detection_bbox(
                bbox,
                depth,
                intrinsics,
                depth_available=depth_available,
            )
            rows.append(
                {
                    "label": detection.label,
                    "confidence": float(detection.score),
                    "bbox": bbox,
                    "point3d": point3d,
                }
            )
        return rows

    def _apply_scene_intrinsics_metadata(
        self,
        intrinsics: dict | None,
        unavailable_reason: str | None = None,
    ) -> None:
        scene_model = getattr(self, "scene_model", None)
        if scene_model is not None:
            scene_model.set_intrinsics_metadata(intrinsics, unavailable_reason)

    def _load_scene_model_intrinsics_once(self) -> None:
        """Read RGB intrinsics once per pipeline session from the open OAK device."""
        from . import stereo

        self._oak_device = None
        pipeline = getattr(self, "pipeline", None)
        if pipeline is None:
            self._apply_scene_intrinsics_metadata(None, "pipeline_not_running")
            return
        try:
            device = pipeline.getDefaultDevice()
            self._oak_device = device
            calibration = device.readCalibrationOrDefault()
            published_size = getattr(self, "_published_frame_size", None)
            if callable(published_size):
                frame_width, frame_height = published_size()
            else:
                frame_width = stereo.PUBLISHED_FRAME_WIDTH
                frame_height = stereo.PUBLISHED_FRAME_HEIGHT
            matrix = calibration.getCameraIntrinsics(
                stereo.dai.CameraBoardSocket.CAM_A,
                int(frame_width),
                int(frame_height),
            )
            parsed = intrinsics_from_calibration_matrix(matrix)
            if parsed is None:
                self._apply_scene_intrinsics_metadata(
                    None, "calibration_intrinsics_invalid"
                )
                return
            self._apply_scene_intrinsics_metadata(parsed, None)
        except Exception:
            self._apply_scene_intrinsics_metadata(None, "calibration_read_failed")

    def _clear_scene_model_intrinsics(self) -> None:
        self._oak_device = None
        scene_model = getattr(self, "scene_model", None)
        if scene_model is not None:
            scene_model.set_intrinsics_metadata(None, "pipeline_stopped")

    def _scene_model_json(self) -> str:
        return json.dumps(self.scene_model.snapshot(self._stamp_ms()))

    def _publish_scene_model(self) -> None:
        message = String()
        message.data = self._scene_model_json()
        self.scene_model_publisher_.publish(message)

    def get_scene_callback(self, request, response):
        del request
        response.success = True
        response.message = self._scene_model_json()
        return response
