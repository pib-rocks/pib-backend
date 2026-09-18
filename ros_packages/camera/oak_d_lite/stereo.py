#!/usr/bin/python3
import base64
from collections import deque
import math
import marshal
import os
import threading
import time
import weakref
import cv2
import depthai as dai
import numpy as np
import rclpy
from datatypes.msg import (
    Detection,
    DetectionArray,
    ModelInfo,
    ModelStatus,
    ModelStatusArray,
)
from datatypes.srv import (
    GetCameraImage,
    GetDepthFrame,
    GetDetections,
    GetDistanceAtPx,
    ListModels,
    StartModel,
    StopModel,
)
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, Int32, Int32MultiArray, String

from .model_registry import ModelRegistry
from .pipeline_manager import PipelineManager
from .imitation import (
    LANDMARK_COUNT as IMITATION_LANDMARK_COUNT,
    build_imitation_script,
    square_box_to_frame,
    square_points_to_frame,
    world_landmark_scalars,
)
from .hand_tracking import (
    HAND_KEYPOINT_NAMES,
    LANDMARK_SCORE_LAYER,
    LANDMARK_VALUE_COUNT,
    LANDMARK_XYZ_LAYERS,
    MANIP_CROP_INSET_PIXELS,
    decode_palm_result,
    fit_manip_crop,
    landmark_score,
    landmarks_in_crop_pixels,
    map_landmarks_to_frame,
)

# Downscaled resolution for Haar cascade face detection (maps back to full frame).
FACE_DETECT_WIDTH = 320
FACE_DETECT_HEIGHT = 180
# Keep ImageManip warp inputs below the full ISP resolution. A 256x256 camera
# branch bounds the single palm manipulation's downscale to 2:1 while remaining
# large enough for the 224x224 landmark crop. Limiting the source branch keeps
# the complete field of view seen by each network; chaining or narrowing
# ImageManip crops would change the pixels on which the models were trained.
HAND_NN_WIDTH = 256
HAND_NN_HEIGHT = 256
IMITATION_SOURCE_WIDTH = 256
IMITATION_SOURCE_HEIGHT = 144
# Device-side queues on the camera branches stay shallow and non-blocking.  The
# host drains them from the 10 Hz timer, far below the camera frame rate, and a
# blocking queue back-pressures the Camera node and stalls every other branch
# sharing it - including the colour output.
BRANCH_INPUT_QUEUE_DEPTH = 1
BRANCH_OUTPUT_QUEUE_DEPTH = 4
COLOR_OUTPUT_QUEUE_DEPTH = 4
STEREO_MODES = {"auto", "on", "off"}
DEFAULT_STEREO_TIMEOUT = 5.0
DEFAULT_HAND_STARTUP_GRACE = 5.0
PIPELINE_START_ATTEMPTS = 3
PIPELINE_START_BACKOFF = 0.25
PIPELINE_STOP_TIMEOUT = 5.0
PIPELINE_STOP_POLL_INTERVAL = 0.05
HAND_STAGE_NAMES = (
    "colour_isp",
    "palm_detector_nn",
    "decoding_nn",
    "decoding_result",
    "image_manip_config",
    "image_manip_roi",
    "hand_landmark_nn",
    "post_processing",
    "publish",
)
IMITATION_STAGE_NAMES = (
    "colour_isp",
    "palm_detector_nn",
    "decoding_nn",
    "decoding_result",
    "image_manip_config",
    "image_manip_roi",
    "hand_landmark_nn",
    "post_processing",
    "publish",
)


class ErrorPublisher(Node):

    def __init__(self):
        super().__init__("error_publisher")
        self.publisher_ = self.create_publisher(String, "camera_topic", 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_image = ""

    def timer_callback(self):
        msg = String()
        msg.data = "Camera not available: "
        self.publisher_.publish(msg)


class CameraNode(Node):
    _device_lifecycle_lock = threading.RLock()
    _device_owner = None

    def __init__(self):
        super().__init__("camera_node")

        self.publisher_ = self.create_publisher(String, "camera_topic", 10)
        self.depth_publisher_ = self.create_publisher(String, "stereo_depth", 10)
        self.face_center_publisher_ = self.create_publisher(
            Float32MultiArray, "face_center", 10
        )
        self.models_status_publisher_ = self.create_publisher(
            ModelStatusArray, "models_status", 10
        )

        cascade_paths = [
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ]

        cascade_path = next((p for p in cascade_paths if os.path.exists(p)), None)

        if cascade_path is None:
            raise RuntimeError("haarcascade_frontalface_default.xml not found")

        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        self.timer_subscription = self.create_subscription(
            Float64, "timer_period_topic", self.timer_period_callback, 10
        )
        self.quality_factor_subscription = self.create_subscription(
            Int32, "quality_factor_topic", self.quality_factor_callback, 10
        )
        self.preview_size_subscription = self.create_subscription(
            Int32MultiArray, "size_topic", self.preview_size_callback, 10
        )

        self.preview_width = 1280
        self.preview_height = 720
        self.quality_factor = 80
        self.current_image = ""
        self.current_frame = None
        self.current_source_size = (0, 0)
        self.current_depth = None
        self.pipeline = None
        self.queue = None
        self.depth_queue = None
        self.nn_queues = {}
        self._pipeline_models = []
        self.depth_available = False
        self.stereo_mode = self._read_stereo_mode()
        self.stereo_timeout = self._read_stereo_timeout()
        self.hand_startup_grace = self._read_hand_startup_grace()
        self._pending_color_packet = None
        self.model_registry = ModelRegistry(logger=self.get_logger())
        self.detection_publishers = {
            model.model_id: self.create_publisher(
                DetectionArray, model.publish_topic, 10
            )
            for model in self.model_registry.models()
            if model.publish_topic
        }
        self.hand_decoder_queue = None
        self.hand_palm_queue = None
        self.hand_roi_queue = None
        self.hand_landmark_queue = None
        self.hand_landmark_config_queue = None
        self.hand_landmark_input_size = 0
        self.hand_source_size = (0, 0)
        self._pending_hand_decoder_packet = None
        self._pending_hands = deque()
        self._hand_warnings = set()
        self.imitation_queue = None
        self._pending_imitation_packet = None
        self.imitation_source_size = (0, 0)
        self._pipeline_lock = threading.RLock()
        self._reset_hand_stage_counters()
        self._reset_imitation_stage_counters()

        self.camera_available = self.init_pipeline()
        self.pipeline_manager = PipelineManager(
            registry=self.model_registry,
            rebuild=self._rebuild_models,
            verify_frames=self._verify_model_frames,
            revert_to_color=self._revert_to_color_only,
            on_change=self.publish_model_statuses,
            logger=self.get_logger(),
        )
        self.last_detections = {}

        if self.camera_available:
            self.get_camera_image_service = self.create_service(
                GetCameraImage, "get_camera_image", self.get_camera_image_callback
            )
            self.get_depth_frame_service = self.create_service(
                GetDepthFrame, "get_depth_frame", self.get_depth_frame_callback
            )
            self.get_distance_at_px_service = self.create_service(
                GetDistanceAtPx, "get_distance_at_px", self.get_distance_at_px_callback
            )
            self.get_logger().info("Camera service initialized.")
        else:
            self.get_logger().error("Camera not available.")

        self.list_models_service = self.create_service(
            ListModels, "list_models", self.list_models_callback
        )
        self.start_model_service = self.create_service(
            StartModel, "start_model", self.start_model_callback
        )
        self.stop_model_service = self.create_service(
            StopModel, "stop_model", self.stop_model_callback
        )
        self.get_detections_service = self.create_service(
            GetDetections, "get_detections", self.get_detections_callback
        )
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.models_status_timer = self.create_timer(1.0, self.publish_model_statuses)

    def _encode_frame(self, frame):
        """JPEG-encode and base64 a frame; returns None on failure."""
        retval, buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality_factor]
        )
        if not retval:
            return None
        return base64.b64encode(buffer).decode("utf-8")

    def get_camera_image_callback(self, request, response):
        # Encode on demand when the service is requested and we have a cached frame.
        if self.current_frame is not None:
            encoded = self._encode_frame(self.current_frame)
            if encoded is not None:
                self.current_image = encoded
        self.get_logger().info(f"LEN IMAGE: {len(self.current_image)}")
        response.image_base64 = self.current_image
        return response

    def _encode_depth_frame(self, depth):
        """Pack uint16 depth (mm) as metadata + base64; returns None on failure."""
        if depth is None:
            return None
        try:
            depth_u16 = np.ascontiguousarray(depth, dtype=np.uint16)
            height, width = depth_u16.shape[:2]
            encoded = base64.b64encode(depth_u16.tobytes()).decode("utf-8")
            return width, height, "16UC1", encoded
        except Exception as e:
            self.get_logger().error(f"Failed to encode depth frame: {e}")
            return None

    def get_depth_frame_callback(self, request, response):
        # Read cached depth from the persistent pipeline; do not reconnect.
        if not self.depth_available:
            response.width = 0
            response.height = 0
            response.encoding = ""
            response.depth_base64 = ""
            return response
        packed = self._encode_depth_frame(self.current_depth)
        if packed is None:
            response.width = 0
            response.height = 0
            response.encoding = ""
            response.depth_base64 = ""
            return response
        response.width, response.height, response.encoding, response.depth_base64 = (
            packed
        )
        return response

    def get_distance_at_px_callback(self, request, response):
        # Pixel lookup against cached depth (mm). 0 means invalid / out of range.
        response.distance_mm = 0.0
        if not self.depth_available or self.current_depth is None:
            return response
        height, width = self.current_depth.shape[:2]
        x, y = int(request.x), int(request.y)
        if x < 0 or y < 0 or x >= width or y >= height:
            return response
        response.distance_mm = float(self.current_depth[y, x])
        return response

    def list_models_callback(self, request, response):
        statuses = self.pipeline_manager.statuses()
        response.models = []
        for model in self.model_registry.models():
            status = statuses[model.model_id]
            info = ModelInfo()
            info.model_id = model.model_id
            info.task = model.task
            info.licence = model.licence
            info.shaves = model.shaves
            info.size_bytes = model.size_bytes
            info.available = model.available
            info.active = status["active"]
            response.models.append(info)
        return response

    def start_model_callback(self, request, response):
        response.success, response.message = self.pipeline_manager.start(
            request.model_id, request.shaves, request.owner
        )
        return response

    def stop_model_callback(self, request, response):
        response.success, response.message = self.pipeline_manager.stop(
            request.model_id, request.owner
        )
        return response

    def _empty_detections(self, model_id):
        detections = DetectionArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.model_id = model_id
        if self.current_frame is None:
            detections.frame_width = 0
            detections.frame_height = 0
        else:
            detections.frame_height, detections.frame_width = self.current_frame.shape[
                :2
            ]
        detections.detections = []
        return detections

    def get_detections_callback(self, request, response):
        response.detections = self.last_detections.get(
            request.model_id, self._empty_detections(request.model_id)
        )
        return response

    def _warn_hand_once(self, message):
        if message not in self._hand_warnings:
            self._hand_warnings.add(message)
            self.get_logger().warning(message)

    def _hand_fingerprint_due(self, name, interval=5.0):
        """Rate-limit diagnostic fingerprints without changing hand processing."""
        now = time.monotonic()
        last_logged = getattr(self, "_hand_fingerprint_last_logged", {}).get(name, 0.0)
        if now - last_logged < interval:
            return False
        if not hasattr(self, "_hand_fingerprint_last_logged"):
            self._hand_fingerprint_last_logged = {}
        self._hand_fingerprint_last_logged[name] = now
        return True

    @staticmethod
    def _hand_tensor_text(values):
        return np.array2string(
            np.asarray(values),
            separator=",",
            threshold=np.inf,
            max_line_width=1000000,
        )

    def _log_palm_fingerprint(self, values, palms):
        if not self._hand_fingerprint_due("palm"):
            return
        records = values.reshape(-1, 8)
        finite = np.all(np.isfinite(records), axis=1)
        positive_size = records[:, 3] > 0
        above_threshold = records[:, 0] >= 0.5
        finite_scores = records[finite, 0]
        best_score = (
            float(np.max(finite_scores)) if finite_scores.size else float("nan")
        )
        self.get_logger().info(
            "HAND_FP DEC "
            f"shape={values.shape} raw={self._hand_tensor_text(values)}; "
            f"decoded_candidates={len(palms)} best_score={best_score:.9g} "
            "score_threshold=0.5 "
            f"dropped_nonfinite={int(np.count_nonzero(~finite))} "
            f"dropped_low_score={int(np.count_nonzero(finite & ~above_threshold))} "
            "dropped_nonpositive_size="
            f"{int(np.count_nonzero(finite & above_threshold & ~positive_size))}"
        )

    def _log_landmark_fingerprint(
        self,
        packet,
        score_tensor,
        landmarks_tensor,
        transformation,
        layer_name=None,
        layer_probe=(),
    ):
        if not self._hand_fingerprint_due("landmark"):
            return
        available_layers = self._packet_layer_names(packet)
        layer_parts = [
            f"{LANDMARK_SCORE_LAYER} shape={score_tensor.shape} "
            f"raw={self._hand_tensor_text(score_tensor)}"
        ]
        if "Identity_2" not in available_layers:
            layer_parts.append("Identity_2 unavailable")
        else:
            try:
                handedness = self._nn_layer(packet, "Identity_2")
                layer_parts.append(
                    f"Identity_2 shape={handedness.shape} "
                    f"raw={self._hand_tensor_text(handedness)}"
                )
            except Exception:
                layer_parts.append("Identity_2 unavailable")
        layer_parts.append(
            f"{layer_name or 'no landmark layer'} "
            f"shape={landmarks_tensor.shape} "
            f"raw={self._hand_tensor_text(landmarks_tensor)}"
        )
        xyz = np.asarray(landmarks_tensor, dtype=np.float32).reshape(-1, 3)
        xy = xyz[:, :2]
        peak = float(np.max(np.abs(xy))) if xy.size else 0.0
        xy_space = "normalized" if peak <= 1.5 else "input_pixels"
        self.get_logger().info(
            "HAND_FP LAND "
            + " / ".join(layer_parts)
            + f"; packet_layers={available_layers} "
            f"landmark_layer_probe={list(layer_probe)} "
            f"input_size={self.hand_landmark_input_size} "
            f"xy_space={xy_space} xy_raw={self._hand_tensor_text(xy)} "
            f"TRANSFORM_available={transformation is not None}"
        )

    def _reset_hand_stage_counters(self):
        self.hand_stage_counters = {stage: 0 for stage in HAND_STAGE_NAMES}
        self._hand_stage_last_logged = dict(self.hand_stage_counters)

    def _reset_imitation_stage_counters(self):
        self.imitation_stage_counters = {stage: 0 for stage in IMITATION_STAGE_NAMES}
        self._imitation_stage_last_logged = dict(self.imitation_stage_counters)

    def _count_hand_stage(self, stage, count=1):
        self.hand_stage_counters[stage] += count

    def _count_imitation_stage(self, stage, count=1):
        self.imitation_stage_counters[stage] += count

    def _log_hand_stage_counters(self):
        if not any(
            active.model.model_id == "hand_tracking"
            for active in getattr(self, "_pipeline_models", ())
        ):
            return
        interval = {
            stage: self.hand_stage_counters[stage] - self._hand_stage_last_logged[stage]
            for stage in HAND_STAGE_NAMES
        }
        last_flowing = "none"
        for stage in HAND_STAGE_NAMES:
            if interval[stage] > 0:
                last_flowing = stage
        raw = " ".join(
            f"{stage}={self.hand_stage_counters[stage]}" for stage in HAND_STAGE_NAMES
        )
        interval_raw = " ".join(
            f"{stage}={interval[stage]}" for stage in HAND_STAGE_NAMES
        )
        source_width, source_height = getattr(self, "hand_source_size", (0, 0))
        self.get_logger().info(
            f"hand_tracking stage packets total: {raw}; "
            f"interval: {interval_raw}; last_flowing={last_flowing}; "
            f"branch={source_width}x{source_height}"
        )
        self._hand_stage_last_logged = dict(self.hand_stage_counters)

    def _log_imitation_stage_counters(self):
        if not any(
            active.model.model_id == "imitation"
            for active in getattr(self, "_pipeline_models", ())
        ):
            return
        interval = {
            stage: (
                self.imitation_stage_counters[stage]
                - self._imitation_stage_last_logged[stage]
            )
            for stage in IMITATION_STAGE_NAMES
        }
        last_flowing = "none"
        for stage in IMITATION_STAGE_NAMES:
            if interval[stage] > 0:
                last_flowing = stage
        raw = " ".join(
            f"{stage}={self.imitation_stage_counters[stage]}"
            for stage in IMITATION_STAGE_NAMES
        )
        interval_raw = " ".join(
            f"{stage}={interval[stage]}" for stage in IMITATION_STAGE_NAMES
        )
        source_width, source_height = self.imitation_source_size
        self.get_logger().info(
            f"imitation stage packets total: {raw}; "
            f"interval: {interval_raw}; last_flowing={last_flowing}; "
            f"branch={source_width}x{source_height}"
        )
        self._imitation_stage_last_logged = dict(self.imitation_stage_counters)

    @staticmethod
    def _nn_layer(packet, name):
        return np.asarray(packet.getTensor(name), dtype=np.float32)

    @staticmethod
    def _packet_layer_names(packet):
        getter = getattr(packet, "getAllLayerNames", None)
        if getter is None:
            return []
        try:
            return sorted(str(name) for name in getter())
        except Exception:
            return []

    def _hand_landmark_score(self, packet):
        """Report the landmark presence score without letting it drop a result.

        This blob emits ``Identity_1`` unactivated, so a hand that fills the
        crop reads about 0.018 and every candidate fails any threshold placed on
        it.  The score is therefore only logged, and a head that is missing or
        malformed leaves the landmarks themselves to decide the outcome.
        """
        try:
            tensor = self._nn_layer(packet, LANDMARK_SCORE_LAYER)
            return tensor, landmark_score(tensor)
        except Exception:
            return np.zeros(0, dtype=np.float32), float("nan")

    def _hand_landmark_tensor(self, packet):
        """Return the first landmark head that actually carries 21 XYZ triples.

        A tensor name that the blob does not expose yields either an exception
        or an empty array, and an empty array maps to zero keypoints without
        raising anything - the landmark result is then silently dropped.  Every
        rejected candidate is reported so the chosen head is visible in the log.
        """
        probe = []
        for name in LANDMARK_XYZ_LAYERS:
            try:
                values = self._nn_layer(packet, name)
            except Exception as exc:
                probe.append(f"{name}=absent({type(exc).__name__})")
                continue
            if values.size == LANDMARK_VALUE_COUNT:
                probe.append(f"{name}=used(size={values.size})")
                return name, values, probe
            probe.append(f"{name}=rejected(size={values.size})")
        return None, np.zeros(0, dtype=np.float32), probe

    @staticmethod
    def _packet_transformation(packet):
        getter = getattr(packet, "getTransformation", None)
        if getter is None:
            return None
        try:
            return getter()
        except Exception:
            return None

    def _map_hand_landmarks(
        self,
        packet,
        tensor,
        palm,
        frame_width,
        frame_height,
        source_width,
        source_height,
        score_tensor=None,
        layer_name=None,
        layer_probe=(),
    ):
        """Map landmark-crop coordinates onto the published preview frame."""
        values = landmarks_in_crop_pixels(tensor, self.hand_landmark_input_size)

        def via_palm_roi():
            return map_landmarks_to_frame(
                values,
                palm,
                frame_width,
                frame_height,
                self.hand_landmark_input_size,
                source_width,
                source_height,
            )

        transformation = self._packet_transformation(packet)
        if score_tensor is not None:
            self._log_landmark_fingerprint(
                packet,
                score_tensor,
                tensor,
                transformation,
                layer_name,
                layer_probe,
            )
        if transformation is None:
            return via_palm_roi()

        try:
            scale_x = frame_width / float(source_width)
            scale_y = frame_height / float(source_height)
            points = []
            for crop_x, crop_y, _ in values:
                crop_point = dai.Point2f(float(crop_x), float(crop_y))
                source_point = transformation.invTransformPoint(crop_point)
                points.append(
                    (
                        min(
                            float(frame_width),
                            max(0.0, float(source_point.x) * scale_x),
                        ),
                        min(
                            float(frame_height),
                            max(0.0, float(source_point.y) * scale_y),
                        ),
                    )
                )
            return points
        except Exception:
            return via_palm_roi()

    def _publish_hand_detections(self, frame_width, frame_height, detections):
        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = "hand_tracking"
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections["hand_tracking"] = message
        publisher = self.detection_publishers.get("hand_tracking")
        if publisher is not None:
            publisher.publish(message)
        self._count_hand_stage("publish")
        self.pipeline_manager.record_packet("hand_tracking")

    def _imitation_detection(self, hand, frame_width, frame_height):
        points = hand.get("landmarks", ())
        if len(points) != IMITATION_LANDMARK_COUNT:
            raise ValueError("imitation result must contain 21 landmarks")
        landmarks = square_points_to_frame(points, frame_width, frame_height)
        region = {
            "box_x": float(hand["box_x"]),
            "box_y": float(hand["box_y"]),
            "box_size": float(hand["box_size"]),
        }
        bbox = square_box_to_frame(region, frame_width, frame_height)
        world_names, world_values = world_landmark_scalars(hand.get("world", ()))

        detection = Detection()
        detection.label = "hand"
        detection.score = float(hand["landmark_score"])
        (
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
        ) = bbox
        detection.keypoint_names = list(HAND_KEYPOINT_NAMES)
        detection.keypoint_x = [float(point[0]) for point in landmarks]
        detection.keypoint_y = [float(point[1]) for point in landmarks]
        detection.keypoint_z = [0.0] * len(HAND_KEYPOINT_NAMES)
        detection.scalar_names = [
            "handedness",
            "palm_score",
            "landmark_score",
        ] + world_names
        detection.scalar_values = [
            float(hand["handedness"]),
            float(hand["palm_score"]),
            float(hand["landmark_score"]),
        ] + world_values
        return detection

    def _publish_imitation_detections(self, frame_width, frame_height, detections):
        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = "imitation"
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections["imitation"] = message
        publisher = self.detection_publishers.get("imitation")
        if publisher is not None:
            publisher.publish(message)
        self.pipeline_manager.record_packet("imitation")

    def _consume_imitation_packet(self, packet):
        payload = marshal.loads(bytes(packet.getData()))
        if not isinstance(payload, dict):
            raise ValueError("imitation Script result must be a dictionary")
        stages = payload.get("stages", {})
        if isinstance(stages, dict):
            for stage in IMITATION_STAGE_NAMES:
                if stage == "colour_isp":
                    continue
                value = stages.get(stage)
                if isinstance(value, int) and value >= 0:
                    self.imitation_stage_counters[stage] = max(
                        self.imitation_stage_counters[stage], value
                    )
        frame_height, frame_width = self.current_frame.shape[:2]
        detections = []
        for hand in payload.get("hands", ()):
            try:
                detections.append(
                    self._imitation_detection(hand, frame_width, frame_height)
                )
            except (KeyError, TypeError, ValueError) as exc:
                self._warn_hand_once(f"Invalid imitation Script result: {exc}")
        self._publish_imitation_detections(frame_width, frame_height, detections)

    def _process_imitation(self):
        if self.imitation_queue is None or self.current_frame is None:
            return
        packet = self._pending_imitation_packet
        self._pending_imitation_packet = None
        for _ in range(32):
            if packet is None:
                packet = self.imitation_queue.tryGet()
            if packet is None:
                break
            try:
                self._consume_imitation_packet(packet)
            except (EOFError, TypeError, ValueError) as exc:
                self._warn_hand_once(f"Invalid imitation payload: {exc}")
            packet = None

    def _log_hand_assembly_fingerprint(self, batch):
        if not batch.get("log_details"):
            return
        reasons = batch.get("drop_reasons", [])
        reason_counts = {
            reason: reasons.count(reason) for reason in sorted(set(reasons))
        }
        self.get_logger().info(
            "HAND_FP ASSEMBLY "
            f"candidates={batch.get('candidates', 0)} "
            f"landmark_results={batch.get('landmark_results', 0)} "
            f"keypoints_built={batch.get('keypoints_built', 0)} "
            f"landmark_layer={batch.get('landmark_layer')} "
            f"appended={len(batch['detections'])} "
            f"dropped={len(reasons)} reasons={reason_counts or {}}"
        )

    def _log_hand_keypoint_fingerprint(self, batch, detail):
        """Report one landmark result and the exact reason it was or was not kept."""
        if not batch.get("log_details"):
            return
        self.get_logger().info(
            "HAND_FP KP "
            f"candidates={batch.get('candidates', 0)} "
            f"landmark_results={batch.get('landmark_results', 0)} "
            f"keypoints_built={batch.get('keypoints_built', 0)} "
            f"detections_pending={len(batch['detections'])} "
            f"{detail}"
        )

    def _hand_detection_message(
        self,
        palm,
        landmarks,
        frame_width,
        frame_height,
        source_width,
        source_height,
    ):
        detection = Detection()
        detection.label = "hand"
        detection.score = float(palm.score)
        (
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
        ) = palm.bbox_pixels(frame_width, frame_height, source_width, source_height)
        detection.keypoint_names = list(HAND_KEYPOINT_NAMES)
        detection.keypoint_x = [float(point[0]) for point in landmarks]
        detection.keypoint_y = [float(point[1]) for point in landmarks]
        detection.keypoint_z = [0.0] * len(HAND_KEYPOINT_NAMES)
        detection.scalar_names = ["z_source"]
        detection.scalar_values = [0.0]
        return detection

    def _queue_landmark_crops(
        self,
        palms,
        frame_width,
        frame_height,
        source_width,
        source_height,
    ):
        batch = {
            "remaining": len(palms),
            "candidates": len(palms),
            "landmark_results": 0,
            "keypoints_built": 0,
            "landmark_layer": None,
            "detections": [],
            "drop_reasons": [],
            "frame_width": frame_width,
            "frame_height": frame_height,
            "source_width": source_width,
            "source_height": source_height,
        }
        for index, palm in enumerate(palms):
            config = self._landmark_crop_config(
                palm,
                source_width,
                source_height,
                reuse_previous=index + 1 < len(palms),
            )
            self.hand_landmark_config_queue.send(config)
            self._count_hand_stage("image_manip_config")
            self._pending_hands.append((palm, batch))

    def _queue_empty_landmark_frame(
        self,
        frame_width,
        frame_height,
        source_width,
        source_height,
    ):
        """Advance the config-gated ROI branch for a frame with no palms."""
        batch = {
            "remaining": 1,
            "candidates": 0,
            "landmark_results": 0,
            "keypoints_built": 0,
            "landmark_layer": None,
            "detections": [],
            "drop_reasons": ["no decoded palm candidates"],
            "frame_width": frame_width,
            "frame_height": frame_height,
            "source_width": source_width,
            "source_height": source_height,
        }
        config = self._landmark_crop_config(
            None,
            source_width,
            source_height,
        )
        self.hand_landmark_config_queue.send(config)
        self._count_hand_stage("image_manip_config")
        # The full-frame landmark result is deliberately discarded. Keeping a
        # sentinel in the pairing queue prevents its packet from being matched
        # with a palm from a later decoder frame.
        self._pending_hands.append((None, batch))

    def _landmark_crop_config(
        self, palm, source_width, source_height, reuse_previous=False
    ):
        """Build a complete dynamic warp config for one decoded palm ROI."""
        if palm is None:
            roi_x, roi_y, roi_width, roi_height, rotation = 0.5, 0.5, 1.0, 1.0, 0.0
        else:
            roi_x, roi_y, roi_width, roi_height = palm.roi_for_frame(
                source_width, source_height
            )
            rotation = palm.rotation
        geometry = (roi_x, roi_y, roi_width, roi_height, rotation)
        if not all(math.isfinite(value) for value in geometry):
            raise ValueError("hand ROI contains non-finite geometry")
        if roi_width <= 0 or roi_height <= 0:
            raise ValueError("hand ROI must have positive dimensions")
        if source_width <= 0 or source_height <= 0:
            raise ValueError("hand ROI source must have positive dimensions")

        crop = fit_manip_crop(
            source_width,
            source_height,
            self.hand_landmark_input_size,
            self.hand_landmark_input_size,
        )
        if palm is None:
            # The sentinel keeps the config-gated branch alive on frames without
            # a palm. Its result is discarded, so it reuses the build-time crop:
            # the complete measured source, inset from the validation boundary.
            roi_x, roi_y = crop.center_x, crop.center_y
            roi_width, roi_height = crop.width, crop.height

        # DepthAI rejects a rotated crop unless its complete bounding box is
        # inside the source image; warp border replication happens only after
        # that validation. Fit the decoded square in actual branch pixels,
        # preserving its aspect and rotation, then normalize it again.
        cos_rotation = abs(math.cos(rotation))
        sin_rotation = abs(math.sin(rotation))
        crop_width = roi_width * source_width
        crop_height = roi_height * source_height
        extent_x = (crop_width * cos_rotation + crop_height * sin_rotation) / 2.0
        extent_y = (crop_width * sin_rotation + crop_height * cos_rotation) / 2.0
        # Keep every corner at least half a pixel inside the frame. Maintaining
        # the decoded center is more important than retaining the full ROI near
        # an edge, so shrink around that center instead of moving off the hand.
        inset = MANIP_CROP_INSET_PIXELS
        center_x = min(
            source_width - inset,
            max(inset, roi_x * source_width),
        )
        center_y = min(
            source_height - inset,
            max(inset, roi_y * source_height),
        )
        available_x = max(inset, min(center_x, source_width - center_x) - inset)
        available_y = max(inset, min(center_y, source_height - center_y) - inset)
        scale = min(
            1.0,
            available_x / extent_x,
            available_y / extent_y,
        )
        crop_width *= scale
        crop_height *= scale
        roi_x = center_x / source_width
        roi_y = center_y / source_height
        roi_width = crop_width / source_width
        roi_height = crop_height / source_height

        rotated = dai.RotatedRect()
        rotated.center.x = roi_x
        rotated.center.y = roi_y
        rotated.size.width = roi_width
        rotated.size.height = roi_height
        rotated.angle = math.degrees(rotation)

        config = dai.ImageManipConfig()
        config.setOutputSize(crop.output_width, crop.output_height)
        config.setFrameType(dai.ImgFrame.Type.BGR888p)
        # roi_for_frame converts decoder coordinates to normalized coordinates
        # of the hand tap as the branch actually delivers it.
        config.addCropRotatedRect(rotated, True)
        # Palm ROIs routinely cross an image edge. Explicit warp border handling
        # keeps those valid instead of making ImageManip skip the frame.
        border_replicate = getattr(config, "setWarpBorderReplicatePixels", None)
        if border_replicate is not None:
            border_replicate()
        config.setReusePreviousImage(reuse_previous)
        return config

    def _drop_landmark_result(self, batch, check, reason):
        """Record and report the exact check that skipped one landmark append."""
        batch.setdefault("drop_reasons", []).append(reason)
        self._log_hand_keypoint_fingerprint(batch, f"skipped={check} reason={reason}")

    def _consume_landmark_result(self, packet, palm, batch):
        """Assemble one landmark result, naming the check that skips the append."""
        batch["landmark_results"] = batch.get("landmark_results", 0) + 1
        if palm is None:
            self._log_hand_keypoint_fingerprint(
                batch, "skipped=sentinel reason=crop carries no paired palm"
            )
            return

        score_tensor, score = self._hand_landmark_score(packet)
        layer_name, landmarks_tensor, layer_probe = self._hand_landmark_tensor(packet)
        batch["landmark_layer"] = layer_name

        if layer_name is None:
            self._log_landmark_fingerprint(
                packet,
                score_tensor,
                landmarks_tensor,
                self._packet_transformation(packet),
                layer_name,
                layer_probe,
            )
            reason = (
                f"no landmark layer carries {LANDMARK_VALUE_COUNT} values; "
                f"probe={list(layer_probe)}"
            )
            self._drop_landmark_result(batch, "layer", reason)
            self._warn_hand_once(f"Hand landmark output unusable: {reason}")
            return

        landmarks = self._map_hand_landmarks(
            packet,
            landmarks_tensor,
            palm,
            batch["frame_width"],
            batch["frame_height"],
            batch["source_width"],
            batch["source_height"],
            score_tensor,
            layer_name,
            layer_probe,
        )
        batch["keypoints_built"] = batch.get("keypoints_built", 0) + len(landmarks)
        if not landmarks:
            self._drop_landmark_result(batch, "mapping", "missing mapped keypoints")
            return

        batch["detections"].append(
            self._hand_detection_message(
                palm,
                landmarks,
                batch["frame_width"],
                batch["frame_height"],
                batch["source_width"],
                batch["source_height"],
            )
        )
        self._log_hand_keypoint_fingerprint(
            batch,
            f"appended=1 score={score:.9g} layer={layer_name} "
            f"keypoints={len(landmarks)}",
        )

    def _process_hand_tracking(self):
        if self.hand_decoder_queue is None or self.current_frame is None:
            return

        while self._pending_hands:
            packet = self.hand_landmark_queue.tryGet()
            if packet is None:
                break
            self._count_hand_stage("hand_landmark_nn")
            palm, batch = self._pending_hands.popleft()
            if "log_details" not in batch:
                batch["log_details"] = self._hand_fingerprint_due("assembly", 1.0)
            try:
                self._consume_landmark_result(packet, palm, batch)
            except (RuntimeError, ValueError) as exc:
                reason = f"invalid landmark output: {type(exc).__name__}: {exc}"
                self._drop_landmark_result(batch, "exception", reason)
                self._warn_hand_once(f"Invalid hand landmark output: {exc}")
            self._count_hand_stage("post_processing")
            batch["remaining"] -= 1
            if batch["remaining"] == 0:
                self._log_hand_assembly_fingerprint(batch)
                self._publish_hand_detections(
                    batch["frame_width"],
                    batch["frame_height"],
                    batch["detections"],
                )

        # Keep decoder and landmark packets paired; accept the next palm frame
        # only after all landmark crops from the previous one have completed.
        if self._pending_hands:
            return
        packet = self._pending_hand_decoder_packet
        self._pending_hand_decoder_packet = None
        if packet is None:
            packet = self.hand_decoder_queue.tryGet()
        if packet is None:
            return
        self._count_hand_stage("decoding_nn")
        frame_height, frame_width = self.current_frame.shape[:2]
        source_width, source_height = self.hand_source_size
        if not source_width or not source_height:
            source_width, source_height = self.current_source_size
        if not source_width or not source_height:
            source_width, source_height = frame_width, frame_height
        try:
            palm_values = self._nn_layer(packet, "result")
            palms = decode_palm_result(palm_values)
            self._log_palm_fingerprint(palm_values, palms)
        except (RuntimeError, ValueError) as exc:
            self._warn_hand_once(f"Invalid palm decoder output: {exc}")
            palms = []
        self._count_hand_stage("decoding_result")
        if not palms:
            self._queue_empty_landmark_frame(
                frame_width,
                frame_height,
                source_width,
                source_height,
            )
            return
        self._queue_landmark_crops(
            palms,
            frame_width,
            frame_height,
            source_width,
            source_height,
        )

    def publish_model_statuses(self):
        statuses = self.pipeline_manager.statuses()
        hand_status = statuses.get("hand_tracking")
        hand_flowing = self._hand_chain_is_flowing()
        if hand_status is not None and hand_flowing:
            if self.pipeline_manager.mark_running("hand_tracking"):
                self.get_logger().info(
                    "hand_tracking recovered after downstream packet flow resumed."
                )
        elif hand_status is not None and hand_status["active"]:
            marked_failed = self.pipeline_manager.mark_failed(
                "hand_tracking",
                "Hand pipeline is not producing startup stage packets",
                startup_grace=self.hand_startup_grace,
            )
            if marked_failed:
                self.get_logger().error(
                    "hand_tracking was marked failed because its physical pipeline "
                    "is missing or has no startup packet flow."
                )
        imitation_status = statuses.get("imitation")
        imitation_flowing = self._imitation_chain_is_flowing()
        if imitation_status is not None and imitation_flowing:
            if self.pipeline_manager.mark_running("imitation"):
                self.get_logger().info(
                    "imitation recovered after device Script packet flow resumed."
                )
        elif imitation_status is not None and imitation_status["active"]:
            marked_failed = self.pipeline_manager.mark_failed(
                "imitation",
                "Imitation pipeline is not producing Script result packets",
                startup_grace=self.hand_startup_grace,
            )
            if marked_failed:
                self.get_logger().error(
                    "imitation was marked failed because its physical pipeline "
                    "is missing or has no startup packet flow."
                )
        self.pipeline_manager.refresh_fps()
        self._log_hand_stage_counters()
        self._log_imitation_stage_counters()
        statuses = self.pipeline_manager.statuses()
        status_array = ModelStatusArray()
        status_array.header.stamp = self.get_clock().now().to_msg()
        status_array.models = []
        for model in self.model_registry.models():
            runtime = statuses[model.model_id]
            status = ModelStatus()
            status.model_id = model.model_id
            status.active = runtime["active"]
            status.fps = float(runtime["fps"])
            status.shaves = runtime["shaves"]
            status.state = runtime["state"]
            status.message = runtime["message"]
            status_array.models.append(status)
        self.models_status_publisher_.publish(status_array)

    def _init_stereo_depth(self):
        """Add StereoDepth outputs to the existing pipeline (no extra start)."""
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        # The OAK-D Lite mono sensors offer only 640x400 and 640x480.  Without an
        # explicit resolution they negotiate a mode the board cannot deliver and
        # the sensor produces zero frames.
        for side, mono in (("left", mono_left), ("right", mono_right)):
            try:
                mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
            except Exception as exc:
                self.get_logger().warning(
                    f"Mono {side} rejected 640x480: {type(exc).__name__}: {exc}"
                )

        stereo = self.pipeline.create(dai.node.StereoDepth)
        try:
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        except Exception:
            pass
        stereo.setLeftRightCheck(True)
        try:
            stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        except Exception as exc:
            self.get_logger().warning(
                f"Stereo median 7x7 unavailable: {type(exc).__name__}: {exc}"
            )
        try:
            stereo.setExtendedDisparity(True)
        except Exception as exc:
            self.get_logger().warning(
                f"Stereo extended disparity unavailable: {type(exc).__name__}: {exc}"
            )

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        self.depth_queue = stereo.depth.createOutputQueue()

    def _read_stereo_mode(self):
        mode = os.environ.get("PIB_CAMERA_STEREO", "auto").strip().lower()
        if mode not in STEREO_MODES:
            self.get_logger().warning(
                f"Invalid PIB_CAMERA_STEREO={mode!r}; using 'off'."
            )
            return "off"
        return mode

    def _read_stereo_timeout(self):
        value = os.environ.get("PIB_CAMERA_STEREO_TIMEOUT", str(DEFAULT_STEREO_TIMEOUT))
        try:
            timeout = float(value)
            if timeout < 0 or not math.isfinite(timeout):
                raise ValueError
            return timeout
        except ValueError:
            self.get_logger().warning(
                "Invalid PIB_CAMERA_STEREO_TIMEOUT; using 5.0 seconds."
            )
            return DEFAULT_STEREO_TIMEOUT

    def _read_hand_startup_grace(self):
        value = os.environ.get(
            "PIB_HAND_STARTUP_GRACE", str(DEFAULT_HAND_STARTUP_GRACE)
        )
        try:
            grace = float(value)
            if grace < 0 or not math.isfinite(grace):
                raise ValueError
            return grace
        except ValueError:
            self.get_logger().warning(
                "Invalid PIB_HAND_STARTUP_GRACE; using 5.0 seconds."
            )
            return DEFAULT_HAND_STARTUP_GRACE

    def _build_pipeline(self, include_stereo):
        """Build one colour pipeline, optionally including the stereo path."""
        self.pipeline = dai.Pipeline()
        self.camRgb = self.pipeline.create(dai.node.Camera)
        self.camRgb.build(dai.CameraBoardSocket.CAM_A)
        self.isp_out = self.camRgb.requestIspOutput()
        self.queue = self.isp_out.createOutputQueue(
            maxSize=COLOR_OUTPUT_QUEUE_DEPTH, blocking=False
        )
        self.depth_queue = None
        self.nn_queues = {}
        self.hand_decoder_queue = None
        self.hand_palm_queue = None
        self.hand_roi_queue = None
        self.hand_landmark_queue = None
        self.hand_landmark_config_queue = None
        self.hand_landmark_input_size = 0
        self.hand_source_size = (0, 0)
        self._pending_hand_decoder_packet = None
        self.imitation_queue = None
        self._pending_imitation_packet = None
        self.imitation_source_size = (0, 0)
        if hasattr(self, "_pending_hands"):
            self._pending_hands.clear()
        else:
            self._pending_hands = deque()
        self._reset_hand_stage_counters()
        self._reset_imitation_stage_counters()

        if include_stereo:
            self._init_stereo_depth()

        for active_model in getattr(self, "_pipeline_models", []):
            model = active_model.model
            if model.model_id == "hand_tracking":
                self._build_hand_pipeline(model)
                continue
            if model.model_id == "imitation":
                self._build_imitation_pipeline(model)
                continue
            neural_network = self.pipeline.create(dai.node.NeuralNetwork)
            neural_network.setBlobPath(model.blob_path)
            neural_network.setNumShavesPerInferenceThread(model.shaves)
            nn_input = self._request_camera_branch(
                (model.input_width, model.input_height)
            )
            self._relax_branch_input(neural_network.input)
            nn_input.link(neural_network.input)
            self.nn_queues[model.model_id] = neural_network.out.createOutputQueue(
                maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
            )

    def _request_camera_branch(self, size):
        """Request a bounded-size colour stream for a model branch."""
        output = self.camRgb.requestOutput(size, type=dai.ImgFrame.Type.BGR888p)
        if output is None:
            raise RuntimeError(
                f"Camera cannot provide a {size[0]}x{size[1]} BGR888p branch output"
            )
        return output

    @staticmethod
    def _size_pair(value):
        """Read a (width, height) pair from a device size object, if it is one."""
        if isinstance(value, (tuple, list)) and len(value) == 2:
            width, height = value
        else:
            width = getattr(value, "width", None)
            height = getattr(value, "height", None)
        if isinstance(width, bool) or isinstance(height, bool):
            return None
        if not isinstance(width, (int, float)) or not isinstance(height, (int, float)):
            return None
        if width <= 0 or height <= 0:
            return None
        return (int(width), int(height))

    def _branch_output_size(self, output, requested):
        """Read the dimensions a camera branch really delivers.

        The camera can hand back a stream whose size differs from the request,
        and every ImageManip crop on that branch has to match the frames that
        arrive rather than the ones that were asked for.
        """
        for getter_name in ("getSize", "getDimensions"):
            getter = getattr(output, getter_name, None)
            if getter is None:
                continue
            try:
                size = self._size_pair(getter())
            except Exception:
                size = None
            if size is not None:
                return size
        return (int(requested[0]), int(requested[1]))

    def _note_hand_branch_size(self, packet):
        """Track the hand branch size a device packet was actually produced from."""
        getter = getattr(packet, "getTransformation", None)
        if getter is None:
            return
        try:
            transformation = getter()
            source_getter = getattr(transformation, "getSourceSize", None)
            size = None if source_getter is None else self._size_pair(source_getter())
        except Exception:
            return
        if size is None or size == self.hand_source_size:
            return
        # Every crop is derived from this size, so a changed branch geometry has
        # to reach the landmark configs before the next frame is cropped.
        self._warn_hand_once(
            "Hand branch delivers "
            f"{size[0]}x{size[1]}, not {self.hand_source_size[0]}x"
            f"{self.hand_source_size[1]}; deriving hand crops from the "
            "delivered size."
        )
        self.hand_source_size = size

    def _configure_hand_manip(
        self,
        manip,
        output_width,
        output_height,
        source_width,
        source_height,
    ):
        """Configure one hand ImageManip from the branch's real dimensions."""
        crop = fit_manip_crop(source_width, source_height, output_width, output_height)
        manip.setMaxOutputFrameSize(crop.output_width * crop.output_height * 3)
        manip.initialConfig.setOutputSize(crop.output_width, crop.output_height)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        # An ImageManip without an explicit initial crop validates the default
        # rect it carries against each incoming frame, and rejects the frame
        # whenever that rect does not fit - including while the landmark manip
        # waits for its first per-frame config.  The rect is centred, inside the
        # measured source, and it is never rotated: only the per-frame landmark
        # config carries an angle, because only the landmark model needs its
        # crop aligned to the palm.
        rotated = dai.RotatedRect()
        rotated.center.x = crop.center_x
        rotated.center.y = crop.center_y
        rotated.size.width = crop.width
        rotated.size.height = crop.height
        rotated.angle = 0.0
        manip.initialConfig.addCropRotatedRect(rotated, True)
        return crop

    def _hand_chain_is_built(self):
        requested = any(
            active.model.model_id == "hand_tracking"
            for active in getattr(self, "_pipeline_models", ())
        )
        if not requested:
            return False
        return all(
            queue is not None
            for queue in (
                self.hand_palm_queue,
                self.hand_decoder_queue,
                self.hand_roi_queue,
                self.hand_landmark_queue,
                self.hand_landmark_config_queue,
            )
        )

    def _hand_chain_is_flowing(self):
        return self._hand_chain_is_built() and all(
            self.hand_stage_counters[stage] > 0
            for stage in (
                "colour_isp",
                "palm_detector_nn",
                "decoding_nn",
                "decoding_result",
                "image_manip_config",
                "image_manip_roi",
                "hand_landmark_nn",
                "post_processing",
                "publish",
            )
        )

    def _imitation_chain_is_built(self):
        requested = any(
            active.model.model_id == "imitation"
            for active in getattr(self, "_pipeline_models", ())
        )
        if not requested:
            return False
        return getattr(self, "imitation_queue", None) is not None

    def _imitation_chain_is_flowing(self):
        return self._imitation_chain_is_built() and all(
            self.imitation_stage_counters[stage] > 0
            for stage in (
                "palm_detector_nn",
                "decoding_nn",
                "decoding_result",
                "publish",
            )
        )

    def _relax_branch_input(self, node_input):
        """Drop frames on a branch input instead of back-pressuring the camera."""
        try:
            node_input.setBlocking(False)
        except Exception:
            self._warn_hand_once("Branch input blocking mode is not configurable.")
        for setter_name in ("setMaxSize", "setQueueSize"):
            setter = getattr(node_input, setter_name, None)
            if setter is None:
                continue
            try:
                setter(BRANCH_INPUT_QUEUE_DEPTH)
                return
            except Exception:
                continue
        self._warn_hand_once("Branch input queue depth is not configurable.")

    def _build_hand_pipeline(self, composite):
        """Add palm resize/detect/decode and dynamic hand ROI landmarks."""
        artifacts = {
            model_id: self.model_registry.get(model_id)
            for model_id in composite.artifact_ids
        }
        palm = artifacts["palm_detection_128x128"]
        decoder = artifacts["palm_detection_128x128_decoding"]
        landmark = artifacts["hand_landmark_224x224"]

        # One downscaled camera stream feeds the palm chain and the landmark
        # manip, both non-blocking. Requesting two identical Camera outputs
        # exceeds the OAK-D Lite camera-output budget once the colour output is
        # present.
        hand_tap = self._request_camera_branch((HAND_NN_WIDTH, HAND_NN_HEIGHT))
        # Every manip crops the frames this branch really carries, so the crops
        # follow the delivered size instead of the requested one.
        self.hand_source_size = self._branch_output_size(
            hand_tap, (HAND_NN_WIDTH, HAND_NN_HEIGHT)
        )
        source_width, source_height = self.hand_source_size

        palm_manip = self.pipeline.create(dai.node.ImageManip)
        self._configure_hand_manip(
            palm_manip,
            palm.input_width,
            palm.input_height,
            source_width,
            source_height,
        )
        self._relax_branch_input(palm_manip.inputImage)
        hand_tap.link(palm_manip.inputImage)

        palm_nn = self.pipeline.create(dai.node.NeuralNetwork)
        palm_nn.setBlobPath(palm.blob_path)
        palm_nn.setNumShavesPerInferenceThread(palm.shaves)
        self._relax_branch_input(palm_nn.input)
        palm_manip.out.link(palm_nn.input)
        self.hand_palm_queue = palm_nn.out.createOutputQueue(
            maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

        decoder_nn = self.pipeline.create(dai.node.NeuralNetwork)
        decoder_nn.setBlobPath(decoder.blob_path)
        decoder_nn.setNumShavesPerInferenceThread(decoder.shaves)
        # Preserve the palm NNData packet so DepthAI can map its named output
        # tensors to the decoder blob's named inputs.
        palm_nn.out.link(decoder_nn.input)
        self.hand_decoder_queue = decoder_nn.out.createOutputQueue(
            maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

        landmark_manip = self.pipeline.create(dai.node.ImageManip)
        landmark_crop = self._configure_hand_manip(
            landmark_manip,
            landmark.input_width,
            landmark.input_height,
            source_width,
            source_height,
        )
        # Landmark tensors are normalized against the size the crop really
        # delivers, so the mapping back to the frame follows it too.
        self.hand_landmark_input_size = landmark_crop.output_width
        # This manip only consumes an image once the host has sent a crop
        # config. Its non-blocking input discards images while idle, so sharing
        # the hand tap cannot back-pressure the palm or colour paths.
        self._relax_branch_input(landmark_manip.inputImage)
        hand_tap.link(landmark_manip.inputImage)
        landmark_manip.inputConfig.setWaitForMessage(True)
        self.hand_landmark_config_queue = landmark_manip.inputConfig.createInputQueue(
            maxSize=16, blocking=False
        )
        self.hand_roi_queue = landmark_manip.out.createOutputQueue(
            maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

        landmark_nn = self.pipeline.create(dai.node.NeuralNetwork)
        landmark_nn.setBlobPath(landmark.blob_path)
        landmark_nn.setNumShavesPerInferenceThread(landmark.shaves)
        landmark_manip.out.link(landmark_nn.input)
        # Stays blocking: _pending_hands expects one landmark result per crop
        # config, and a dropped result would stall the pairing permanently.
        self.hand_landmark_queue = landmark_nn.out.createOutputQueue()

    def _build_imitation_pipeline(self, composite):
        """Add the reference on-device palm/landmark manager Script."""
        artifacts = {
            model_id: self.model_registry.get(model_id)
            for model_id in composite.artifact_ids
        }
        palm = artifacts["palm_detection_sh4"]
        decoder = artifacts["pd_postprocessing_top2_sh1"]
        landmark = artifacts["hand_landmark_full_sh4"]

        imitation_tap = self._request_camera_branch(
            (IMITATION_SOURCE_WIDTH, IMITATION_SOURCE_HEIGHT)
        )
        self.imitation_source_size = self._branch_output_size(
            imitation_tap, (IMITATION_SOURCE_WIDTH, IMITATION_SOURCE_HEIGHT)
        )
        source_width, source_height = self.imitation_source_size

        palm_manip = self.pipeline.create(dai.node.ImageManip)
        self._configure_hand_manip(
            palm_manip,
            palm.input_width,
            palm.input_height,
            source_width,
            source_height,
        )
        palm_manip.inputConfig.setWaitForMessage(True)
        self._relax_branch_input(palm_manip.inputImage)
        imitation_tap.link(palm_manip.inputImage)

        palm_nn = self.pipeline.create(dai.node.NeuralNetwork)
        palm_nn.setBlobPath(palm.blob_path)
        palm_nn.setNumShavesPerInferenceThread(palm.shaves)
        self._relax_branch_input(palm_nn.input)
        palm_manip.out.link(palm_nn.input)

        decoder_nn = self.pipeline.create(dai.node.NeuralNetwork)
        decoder_nn.setBlobPath(decoder.blob_path)
        decoder_nn.setNumShavesPerInferenceThread(decoder.shaves)
        palm_nn.out.link(decoder_nn.input)

        landmark_manip = self.pipeline.create(dai.node.ImageManip)
        self._configure_hand_manip(
            landmark_manip,
            landmark.input_width,
            landmark.input_height,
            source_width,
            source_height,
        )
        landmark_manip.inputConfig.setWaitForMessage(True)
        self._relax_branch_input(landmark_manip.inputImage)
        imitation_tap.link(landmark_manip.inputImage)

        landmark_nn = self.pipeline.create(dai.node.NeuralNetwork)
        landmark_nn.setBlobPath(landmark.blob_path)
        landmark_nn.setNumShavesPerInferenceThread(landmark.shaves)
        landmark_manip.out.link(landmark_nn.input)

        manager = self.pipeline.create(dai.node.Script)
        manager.setScript(build_imitation_script(source_width, source_height))
        processor = getattr(getattr(dai, "ProcessorType", None), "LEON_CSS", None)
        if processor is not None:
            manager.setProcessor(processor)
        manager.outputs["pre_pd_manip_cfg"].link(palm_manip.inputConfig)
        decoder_nn.out.link(manager.inputs["from_post_pd_nn"])
        manager.outputs["pre_lm_manip_cfg"].link(landmark_manip.inputConfig)
        landmark_nn.out.link(manager.inputs["from_lm_nn"])
        self.imitation_queue = manager.outputs["host"].createOutputQueue(
            maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

    def _stop_pipeline(self):
        pipeline_lock = getattr(self, "_pipeline_lock", None)
        if pipeline_lock is None:
            pipeline_lock = threading.RLock()
            self._pipeline_lock = pipeline_lock
        with pipeline_lock, CameraNode._device_lifecycle_lock:
            pipeline = self.pipeline
            if pipeline is not None:
                stop_failed = False
                try:
                    pipeline.stop()
                except Exception as exc:
                    stop_failed = True
                    self.get_logger().warning(
                        f"Camera pipeline stop reported an error: {exc}"
                    )
                deadline = time.monotonic() + PIPELINE_STOP_TIMEOUT
                released = False
                while time.monotonic() < deadline:
                    try:
                        if pipeline.isRunning() is not True:
                            released = True
                            break
                    except Exception:
                        released = not stop_failed
                        break
                    time.sleep(PIPELINE_STOP_POLL_INTERVAL)
                if not released:
                    self.get_logger().warning(
                        "Camera pipeline did not report stopped before timeout."
                    )
                    return False
            owner_ref = CameraNode._device_owner
            if owner_ref is not None and owner_ref() is self:
                CameraNode._device_owner = None
            self.pipeline = None
            self.queue = None
            self.depth_queue = None
            self.nn_queues = {}
            self.hand_decoder_queue = None
            self.hand_palm_queue = None
            self.hand_roi_queue = None
            self.hand_landmark_queue = None
            self.hand_landmark_config_queue = None
            self.hand_landmark_input_size = 0
            self.hand_source_size = (0, 0)
            self._pending_hand_decoder_packet = None
            self.imitation_queue = None
            self._pending_imitation_packet = None
            self.imitation_source_size = (0, 0)
            if hasattr(self, "_pending_hands"):
                self._pending_hands.clear()
            return True

    def _start_pipeline(self, include_stereo):
        """Build and start a fresh pipeline with bounded retries."""
        pipeline_lock = getattr(self, "_pipeline_lock", None)
        if pipeline_lock is None:
            pipeline_lock = threading.RLock()
            self._pipeline_lock = pipeline_lock
        with pipeline_lock, CameraNode._device_lifecycle_lock:
            owner_ref = CameraNode._device_owner
            owner = owner_ref() if owner_ref is not None else None
            if owner is not None and owner is not self:
                self.get_logger().error(
                    "Camera pipeline start refused: this process already has "
                    "another OAK device holder."
                )
                return False
            if owner is self and self.pipeline is not None:
                self.get_logger().error(
                    "Camera pipeline start refused: the previous OAK holder "
                    "has not been released."
                )
                return False
            for attempt in range(PIPELINE_START_ATTEMPTS):
                try:
                    self._build_pipeline(include_stereo)
                    self.pipeline.start()
                    CameraNode._device_owner = weakref.ref(self)
                    return True
                except Exception as exc:
                    self.get_logger().error(
                        "Camera pipeline build/start attempt "
                        f"{attempt + 1}/{PIPELINE_START_ATTEMPTS} failed: {exc}"
                    )
                    self._stop_pipeline()
                    if attempt + 1 < PIPELINE_START_ATTEMPTS:
                        time.sleep(PIPELINE_START_BACKOFF * (2**attempt))
            return False

    def _wait_for_color_frame(self, timeout):
        """Return the first measured colour packet, or None at the deadline."""
        deadline = time.monotonic() + timeout
        while True:
            packet = self.queue.tryGet()
            if packet is not None:
                return packet
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            time.sleep(min(0.05, remaining))

    @staticmethod
    def _wait_for_queue_packet(queue, timeout):
        """Return the first packet from a non-colour queue before timeout."""
        deadline = time.monotonic() + timeout
        while True:
            packet = queue.tryGet()
            if packet is not None:
                return packet
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            time.sleep(min(0.05, remaining))

    def _rebuild_models(self, active_models):
        self._pipeline_models = list(active_models)
        self._stop_pipeline()
        self.camera_available = self.init_pipeline()
        if self.camera_available:
            missing = [
                active.model.model_id
                for active in self._pipeline_models
                if (
                    active.model.model_id == "hand_tracking"
                    and not self._hand_chain_is_built()
                )
                or (
                    active.model.model_id == "imitation"
                    and not self._imitation_chain_is_built()
                )
                or (
                    active.model.model_id not in ("hand_tracking", "imitation")
                    and active.model.model_id not in self.nn_queues
                )
            ]
            if missing:
                self.get_logger().error(
                    "Camera pipeline started without requested model chain(s): "
                    + ", ".join(missing)
                )
                self._stop_pipeline()
                self.camera_available = False
        return self.camera_available

    def _verify_model_frames(self, timeout):
        requested_ids = {
            active.model.model_id for active in getattr(self, "_pipeline_models", ())
        }
        if "hand_tracking" in requested_ids and not self._hand_chain_is_built():
            self.get_logger().error(
                "Cannot verify hand_tracking: the complete hand chain is absent"
            )
            return False
        if "imitation" in requested_ids and not self._imitation_chain_is_built():
            self.get_logger().error(
                "Cannot verify imitation: the Script output queue is absent"
            )
            return False
        if any(
            model_id not in ("hand_tracking", "imitation")
            and model_id not in self.nn_queues
            for model_id in requested_ids
        ):
            return False

        packet = self._wait_for_color_frame(timeout)
        if packet is None:
            return False
        self._pending_color_packet = packet
        if "hand_tracking" in requested_ids:
            palm_packet = self._wait_for_queue_packet(self.hand_palm_queue, timeout)
            if palm_packet is None:
                return False
            self._count_hand_stage("colour_isp")
            self._count_hand_stage("palm_detector_nn")
            packet = self._wait_for_queue_packet(self.hand_decoder_queue, timeout)
            if packet is None:
                return False
            self._pending_hand_decoder_packet = packet
            self._count_hand_stage("decoding_nn")
        if "imitation" in requested_ids:
            packet = self._wait_for_queue_packet(self.imitation_queue, timeout)
            if packet is None:
                return False
            self._pending_imitation_packet = packet
        return True

    def _revert_to_color_only(self):
        self._pipeline_models = []
        self._stop_pipeline()
        self.depth_available = False
        self.current_depth = None
        started = self._start_pipeline(include_stereo=False)
        self.camera_available = started
        if not started:
            return False
        return self._verify_model_frames(self.stereo_timeout)

    def init_pipeline(self) -> bool:
        self.depth_available = False
        self.current_depth = None
        self._pending_color_packet = None

        if self.stereo_mode == "off":
            if not self._start_pipeline(include_stereo=False):
                self.get_logger().error(
                    "Camera not found: colour pipeline failed to start."
                )
                return False
            self.get_logger().warning(
                "Stereo depth disabled - using colour-only pipeline (depth disabled)"
            )
            return True

        stereo_started = self._start_pipeline(include_stereo=True)
        if self.stereo_mode == "on":
            if not stereo_started:
                self.get_logger().error(
                    "Camera not found: stereo pipeline failed to start."
                )
                return False
            self.depth_available = True
            self.get_logger().info(
                "Stereo depth available - full colour + stereo pipeline active "
                "(mode=on)"
            )
            return True

        if stereo_started:
            try:
                first_packet = self._wait_for_color_frame(self.stereo_timeout)
            except Exception:
                first_packet = None
            if first_packet is not None:
                self._pending_color_packet = first_packet
                self.depth_available = True
                self.get_logger().info(
                    "Stereo depth available - full colour + stereo pipeline active "
                    "(mode=auto)"
                )
                return True

        self._stop_pipeline()
        if not self._start_pipeline(include_stereo=False):
            self.get_logger().error(
                "Camera not found: colour pipeline failed to start."
            )
            return False
        self.get_logger().warning(
            "Stereo depth unavailable - falling back to colour-only pipeline "
            "(depth disabled)"
        )
        return True

    def publish_face_center(self, frame):
        # Skip expensive Haar cascade when nobody is listening to face_center.
        if self.face_center_publisher_.get_subscription_count() == 0:
            return

        face_msg = Float32MultiArray()

        if self.face_cascade.empty():
            face_msg.data = [0.0, 0.0]
            self.face_center_publisher_.publish(face_msg)
            return

        full_h, full_w = frame.shape[:2]
        small = cv2.resize(frame, (FACE_DETECT_WIDTH, FACE_DETECT_HEIGHT))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 5)

        if len(faces) > 0:
            x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
            scale_x = full_w / FACE_DETECT_WIDTH
            scale_y = full_h / FACE_DETECT_HEIGHT
            x_full = x * scale_x
            y_full = y * scale_y
            w_full = w * scale_x
            h_full = h * scale_y
            x_center = (x_full + w_full / 2) - (full_w / 2)
            y_center = (full_h / 2) - (y_full + h_full / 2)
            face_msg.data = [float(x_center), float(y_center)]
        else:
            face_msg.data = [0.0, 0.0]

        self.face_center_publisher_.publish(face_msg)

    def _publish_colorized_depth(self, depth):
        if self.depth_publisher_.get_subscription_count() == 0:
            return
        depth_u16 = np.ascontiguousarray(depth, dtype=np.uint16)
        depth_vis = cv2.normalize(depth_u16, None, 0, 255, cv2.NORM_MINMAX)
        colorized = cv2.applyColorMap(depth_vis.astype(np.uint8), cv2.COLORMAP_JET)
        encoded = self._encode_frame(colorized)
        if encoded is None:
            return
        msg = String()
        msg.data = encoded
        self.depth_publisher_.publish(msg)

    def timer_callback(self):
        if self.queue:
            image_rgb = self._pending_color_packet
            self._pending_color_packet = None
            if image_rgb is None:
                image_rgb = self.queue.tryGet()
            if image_rgb is not None:
                frame = image_rgb.getCvFrame()
                self.current_source_size = (frame.shape[1], frame.shape[0])

                if (
                    frame.shape[1] != self.preview_width
                    or frame.shape[0] != self.preview_height
                ):
                    frame = cv2.resize(frame, (self.preview_width, self.preview_height))

                self.current_frame = frame
                if self.hand_decoder_queue is not None:
                    self._count_hand_stage("colour_isp")
                if self.imitation_queue is not None:
                    self._count_imitation_stage("colour_isp")
                self.publish_face_center(frame)

                # Only JPEG/base64 encode when someone is subscribed to camera_topic.
                # get_camera_image encodes on demand from current_frame instead.
                if self.publisher_.get_subscription_count() > 0:
                    encoded = self._encode_frame(frame)
                    if encoded is not None:
                        self.current_image = encoded
                        msg = String()
                        msg.data = encoded
                        self.publisher_.publish(msg)

        for model_id, nn_queue in self.nn_queues.items():
            for _ in range(32):
                if nn_queue.tryGet() is None:
                    break
                self.pipeline_manager.record_packet(model_id)

        for stage, queue in (
            ("palm_detector_nn", self.hand_palm_queue),
            ("image_manip_roi", self.hand_roi_queue),
        ):
            if queue is None:
                continue
            for _ in range(32):
                packet = queue.tryGet()
                if packet is None:
                    break
                if stage == "palm_detector_nn":
                    self._note_hand_branch_size(packet)
                self._count_hand_stage(stage)

        self._process_hand_tracking()
        self._process_imitation()

        if not self.depth_queue:
            return

        depth_packet = self.depth_queue.tryGet()
        if depth_packet is None:
            return

        depth = depth_packet.getFrame()
        self.current_depth = depth
        self._publish_colorized_depth(depth)

    def timer_period_callback(self, msg):
        self.timer_period = msg.data
        self.timer.cancel()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def quality_factor_callback(self, msg):
        self.quality_factor = msg.data

    def preview_size_callback(self, msg):
        self.preview_width, self.preview_height = msg.data

        with self._pipeline_lock:
            self._stop_pipeline()
            self.camera_available = self.init_pipeline()

    def destroy_node(self):
        self._stop_pipeline()
        return super().destroy_node()


def spin_camera(times):
    cnt = times
    if cnt == 0:
        print(
            "Couldn't restart camera due to displayed error/s, publishing error message"
        )
        rclpy.spin(error_publisher)
    else:
        camera_node = None
        try:
            camera_node = CameraNode()
            rclpy.spin(camera_node)
        except Exception as exc:
            error_publisher.timer_callback()
            print(exc)
        finally:
            if camera_node is not None:
                camera_node.destroy_node()
                print("camera_node destroyed")
            cnt = times - 1
            print("Retry starting camera..." + str(cnt))
            spin_camera(cnt)
    return


def main(args=None):
    rclpy.init()
    global error_publisher
    error_publisher = ErrorPublisher()
    print("Starting camera")
    spin_camera(3)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
