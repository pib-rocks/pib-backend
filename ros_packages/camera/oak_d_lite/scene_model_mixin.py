"""Scene-model wiring for the camera node.

Kept out of ``stereo.py`` so the node module stays under its line budget; the
model itself is pure Python in :mod:`oak_d_lite.scene_model`.
"""

from __future__ import annotations

import json

from std_msgs.msg import String

from .scene_model import SceneModel

SCENE_MODEL_TOPIC = "/scene_model"
SCENE_MODEL_PUBLISH_HZ = 2.0
SCENE_MODEL_TIMER_PERIOD_SECONDS = 1.0 / SCENE_MODEL_PUBLISH_HZ


class SceneModelMixin:
    """Publishes a persistent scene snapshot and answers ``get_scene``."""

    def _init_scene_model(self) -> None:
        self.scene_model = SceneModel()
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

    def _stamp_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    @staticmethod
    def _scene_detection_dicts(detections) -> list[dict]:
        rows = []
        for detection in detections:
            rows.append(
                {
                    "label": detection.label,
                    "confidence": float(detection.score),
                    "bbox": (
                        int(detection.x_min),
                        int(detection.y_min),
                        int(detection.x_max),
                        int(detection.y_max),
                    ),
                    "point3d": None,
                }
            )
        return rows

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
