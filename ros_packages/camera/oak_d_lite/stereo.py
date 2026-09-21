#!/usr/bin/python3
import base64
from collections import deque
import math
import os
import threading
import time
import weakref
import cv2
import depthai as dai
from depthai_nodes.node import FrameCropper, GatherData, ParsingNeuralNetwork
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
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Float64, Int32, Int32MultiArray, String

from .imu import (
    ClockOffsetEstimator,
    IMU_PUBLISH_RATE_HZ,
    PublishRateThrottle,
    assemble_imu_sample,
    duration_to_nanoseconds,
    host_stamp_nanoseconds,
    measured_rate_hz,
    published_stamp_ns,
)
from .model_registry import ModelRegistry
from .pipeline_manager import PipelineManager
from .imitation import (
    LANDMARK_COUNT as IMITATION_LANDMARK_COUNT,
    PALM_PADDING as IMITATION_PALM_PADDING,
    ProcessDetections,
    box_from_points,
    gathered_hands,
    gathered_result_trace_values,
    world_landmark_scalars,
)
from .imitation_archive import create_landmark_archive, create_palm_archive
from .hand_tracking import (
    HAND_KEYPOINT_NAMES,
    LANDMARK_HANDEDNESS_LAYER,
    LANDMARK_SCORE_LAYER,
    LANDMARK_SCORE_THRESHOLD,
    LANDMARK_VALUE_COUNT,
    LANDMARK_XYZ_LAYERS,
    MANIP_CROP_INSET_PIXELS,
    decode_palm_result,
    fit_manip_crop,
    landmark_score,
    landmarks_in_crop_pixels,
    map_landmarks_to_frame,
    relative_landmark_z,
    PalmRegion,
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
IMITATION_FPS = 8
# The neural branch carries the FULL 16:9 field of view at the size the
# HandTrackerEdge reference uses (internal_frame_height=640 on a 16:9 sensor),
# and this device was measured to deliver a 1152x648 branch alongside the colour
# stream and stereo depth.
#
# The aspect ratio must match the published camera frame. Measured with the
# square 768x768 branch of step 1: a centred hand landed within 8 px of its real
# position, but a hand at the right edge was off by ~190 px in x and its extent
# was squashed in y - the signature of mapping square-normalised coordinates
# onto a 16:9 frame. With a 16:9 branch the mapping back is a pure per-axis
# scale, so that error cannot occur by construction.
IMITATION_SOURCE_WIDTH = 1152
IMITATION_SOURCE_HEIGHT = 648
# The Luxonis hand-pose reference pipeline (PR-1791): palm detector, device-side
# FrameCropper and hand landmarker, timestamp-matched. Same rate limit the example
# uses on RVC2. The branch keeps the 16:9 field of view so the neural branch and
# the published frame agree.
HAND_MP_FPS = 8
HAND_MP_SOURCE_WIDTH = 1152
HAND_MP_SOURCE_HEIGHT = 648
# How many palm detections are buffered while their landmark results arrive.
HAND_MP_PAIR_WINDOW = 32
# Unpaired entries older than this are dropped so the buffer cannot grow.
MAX_HAND_MP_BUFFER = 64
# How many camera frames the FrameCropper may hold while it waits for a crop
# config. One frame is not enough: crops are paired to frames by exact timestamp,
# so a single-slot input has usually dropped the frame a config refers to
# (measured: 7.85 palm detections/s in, 1.00 landmark results/s out). The bound
# still exists so a stretch with no detection cannot exhaust the camera's shared
# frame pool - that is the freeze named in PR-1778.
HAND_MP_CROPPER_QUEUE = 4

# Device-side queues on the camera branches stay shallow and non-blocking.  The
# host drains them from the 10 Hz timer, far below the camera frame rate, and a
# blocking queue back-pressures the Camera node and stalls every other branch
# sharing it - including the colour output.
BRANCH_INPUT_QUEUE_DEPTH = 1
BRANCH_OUTPUT_QUEUE_DEPTH = 4
COLOR_OUTPUT_QUEUE_DEPTH = 4
# Twice the 100 Hz publication rate: a report therefore exists within one sensor
# period (5 ms) of every publication deadline, so a single dropped or late
# report cannot make the throttle skip a whole 10 ms slot. The BMI270 offers up
# to 500 Hz for the raw reports; that headroom is deliberately left unused
# because each report costs USB bandwidth and one host-side Python iteration.
IMU_SENSOR_RATE_HZ = 200
# 20 reports at 200 Hz is 100 ms of device-side buffering, which absorbs ten
# consecutive missed 10 ms poll ticks. A deeper queue would not buy accuracy -
# the stamp comes from the device timestamp, not from the drain instant - it
# would only let a longer stall be replayed as a burst.
IMU_OUTPUT_QUEUE_DEPTH = 20
# Bounds the drain loop so a flooded queue cannot hold the executor. It only has
# to exceed IMU_OUTPUT_QUEUE_DEPTH for one tick to always empty a full queue;
# 256 does so with a wide margin and is still a bounded number of iterations.
IMU_DRAIN_LIMIT = 256
# Publications used to report a measured rate in models_status. 100 samples is
# one second at the new publication rate, which matches the 1 Hz status timer:
# each status message reports the rate of the second it describes.
IMU_RATE_WINDOW = 100
# One poll per publication period. The device queue makes a slower poll harmless
# for throughput, but the report would then wait in the queue, and polling
# faster than the publication period only adds empty wake-ups.
IMU_POLL_PERIOD_SECONDS = 1.0 / IMU_PUBLISH_RATE_HZ
# Expressed as missed publications rather than as a bare second: at 100 Hz a
# one-second silence is a hundred lost samples, far too late to call the stream
# healthy. 50 periods (0.5 s) still tolerates one long colour-frame encode
# stalling the single-threaded executor, which would otherwise make the status
# flap to "stale" while the IMU itself is fine.
IMU_STALE_MISSED_PUBLICATIONS = 50
IMU_STALE_AFTER_SECONDS = IMU_STALE_MISSED_PUBLICATIONS / IMU_PUBLISH_RATE_HZ
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
    """Own and publish the OAK-D Lite camera and its on-board BMI270 IMU.

    On pib the OAK-D Lite is mounted in the head, facing forward. The BMI270 is
    soldered to that camera PCB, so ``oak_imu_frame`` is at the camera module,
    not at the robot base or torso origin.
    """

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
        self.imu_publisher_ = self.create_publisher(Imu, "/imu", 10)

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
        self.imu_queue = None
        self.imu_available = False
        self._imu_last_received_monotonic = None
        self._imu_last_sequence = None
        self._imu_last_device_stamp_ns = None
        self._imu_publish_times = deque(maxlen=IMU_RATE_WINDOW)
        self._imu_throttle = PublishRateThrottle()
        self._imu_clock_offset = ClockOffsetEstimator()
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
        self.hand_mp_pairs = {}
        self._pending_imitation_packet = None
        self.imitation_source_size = (0, 0)
        self.hand_mp_detection_queue = None
        self.hand_mp_landmark_queue = None
        self.hand_mp_source_size = (0, 0)
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
        self.imu_timer = self.create_timer(IMU_POLL_PERIOD_SECONDS, self._process_imu)
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
        for model in self.model_registry.selectable_models():
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

    @staticmethod
    def _queue_state(queue):
        """Return ``"<depth>/<max>"`` for a message queue, or ``"-"`` if absent.

        A pipeline that stops delivering shows up as a queue that stays full or
        stays empty, so the depth at the moment of the freeze separates "the
        host did not drain" from "the device stopped producing".
        """
        if queue is None:
            return "-"
        try:
            return f"{queue.getSize()}/{queue.getMaxSize()}"
        except Exception:
            return "?"

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
        depths = (
            f"colour_queue={self._queue_state(getattr(self, 'queue', None))} "
            f"imitation_queue={self._queue_state(getattr(self, 'imitation_queue', None))} "
            f"pending={1 if getattr(self, '_pending_imitation_packet', None) is not None else 0}"
        )
        self.get_logger().info(
            f"imitation stage packets total: {raw}; "
            f"interval: {interval_raw}; last_flowing={last_flowing}; "
            f"branch={source_width}x{source_height}; {depths}"
        )

        # Mark the moment the pipeline stops, not just the aftermath: without
        # this the counters only ever show that something froze, and the state
        # at that instant - which queue was full - is gone by the time anyone
        # looks. Log once per stall, and re-arm as soon as stages move again.
        if all(interval[stage] == 0 for stage in IMITATION_STAGE_NAMES):
            if not getattr(self, "_imitation_stall_logged", False):
                self._imitation_stall_logged = True
                self.get_logger().warning(
                    f"IMIT_STALL no imitation stage advanced in the last "
                    f"interval; last_flowing={last_flowing}; {depths}"
                )
        else:
            self._imitation_stall_logged = False
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
        """Read the landmark presence score that decides whether a hand is kept.

        A good crop scores near 1.0 and an unusable one scores below 0.02, so
        the same value the reference gates on is usable here.  A missing or
        malformed head returns NaN, which the caller treats as a drop rather
        than letting it pass silently.
        """
        try:
            tensor = self._nn_layer(packet, LANDMARK_SCORE_LAYER)
            return tensor, landmark_score(tensor)
        except Exception:
            return np.zeros(0, dtype=np.float32), float("nan")

    def _hand_landmark_handedness(self, packet):
        """Read the hand's handedness, or NaN when the head is absent."""
        try:
            tensor = self._nn_layer(packet, LANDMARK_HANDEDNESS_LAYER)
            values = np.asarray(tensor, dtype=np.float32).reshape(-1)
            if values.size != 1:
                raise ValueError("handedness must contain one value")
            return float(values[0])
        except Exception:
            return float("nan")

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
        landmarks = hand.get("landmarks", ())
        if len(landmarks) != IMITATION_LANDMARK_COUNT:
            raise ValueError("imitation result must contain 21 landmarks")
        # Decision (a): the published box encloses the 21 landmarks. The
        # detector's own palm box covers the palm only and would leave the
        # fingers outside the rectangle drawn in Cerebra.
        bbox = box_from_points(landmarks, frame_width, frame_height)
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
        self._count_imitation_stage("publish")
        self.pipeline_manager.record_packet("imitation")

    def _publish_hand_mp_detections(self, frame_width, frame_height, detections):
        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = "hand_tracking_mp"
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections["hand_tracking_mp"] = message
        publisher = self.detection_publishers.get("hand_tracking_mp")
        if publisher is not None:
            publisher.publish(message)
        self._count_hand_stage("publish")
        self.pipeline_manager.record_packet("hand_tracking_mp")

    def _consume_imitation_packet(self, packet):
        frame_height, frame_width = self.current_frame.shape[:2]
        palm_count = len(packet.reference_data.detections)
        result_count = len(packet.items)
        self._count_imitation_stage("palm_detector_nn")
        self._count_imitation_stage("decoding_nn")
        self._count_imitation_stage("decoding_result")
        self._count_imitation_stage("image_manip_config", palm_count)
        self._count_imitation_stage("image_manip_roi", result_count)
        self._count_imitation_stage("hand_landmark_nn", result_count)
        self._count_imitation_stage("post_processing", result_count)

        # The trace runs BEFORE the conversion on purpose: when gathered_hands
        # raises, the trace is the only evidence left, and a trace that sits
        # after it disappears exactly when it is needed most.
        for palm_score, landmark_score, crop in gathered_result_trace_values(packet):
            score_text = (
                f"{landmark_score:.9g}" if landmark_score is not None else "unavailable"
            )
            self.get_logger().info(
                "IMIT_TRACE "
                f"palm_score={palm_score:.9g} landmark_score={score_text} "
                f"crop=({crop[0]:.9g},{crop[1]:.9g},"
                f"{crop[2]:.9g},{crop[3]:.9g})"
            )

        hands = gathered_hands(packet, frame_width, frame_height)
        detections = []
        for hand in hands:
            try:
                detections.append(
                    self._imitation_detection(hand, frame_width, frame_height)
                )
            except (KeyError, TypeError, ValueError) as exc:
                self._warn_hand_once(f"Invalid imitation gathered result: {exc}")
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
            except (AttributeError, IndexError, TypeError, ValueError) as exc:
                # NOT _warn_hand_once: that logs a single line for the whole
                # process lifetime. A repeating failure then looks like "no hand
                # detected" while 8 packets per second are silently discarded,
                # which is exactly how this bug hid. Log the type and a running
                # count, throttled so it cannot flood the log.
                self._imitation_error_count = (
                    getattr(self, "_imitation_error_count", 0) + 1
                )
                if self._imitation_error_count % 25 == 1:
                    self.get_logger().error(
                        "IMIT_DROP "
                        f"count={self._imitation_error_count} "
                        f"type={type(exc).__name__} message={exc}"
                    )
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
        landmark_score_value=float("nan"),
        handedness=float("nan"),
        relative_z=(),
    ):
        """Assemble one hand the way the reference reports it.

        The reference carries three things this message has to hold: the
        landmark presence score as the result's confidence, the handedness that
        tells the two hands apart, and the landmark head's third component as a
        hand-relative, unitless depth - all three components of a landmark share
        one scale, as they do in ``rrn_lms``.  ``z_source`` names where the z
        came from, because ``keypoint_z`` is otherwise reserved for depth in
        millimetres.
        """
        detection = Detection()
        detection.label = "hand"
        detection.score = float(landmark_score_value)
        (
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
        ) = palm.bbox_pixels(frame_width, frame_height, source_width, source_height)
        detection.keypoint_names = list(HAND_KEYPOINT_NAMES)
        detection.keypoint_x = [float(point[0]) for point in landmarks]
        detection.keypoint_y = [float(point[1]) for point in landmarks]
        z_values = [float(value) for value in relative_z]
        if len(z_values) != len(HAND_KEYPOINT_NAMES):
            z_values = [0.0] * len(HAND_KEYPOINT_NAMES)
        detection.keypoint_z = z_values
        detection.scalar_names = [
            "handedness",
            "palm_score",
            "landmark_score",
            "z_source",
        ]
        detection.scalar_values = [
            float(handedness),
            float(palm.score),
            float(landmark_score_value),
            1.0,
        ]
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
        if not math.isfinite(score) or score < LANDMARK_SCORE_THRESHOLD:
            self._drop_landmark_result(
                batch,
                "score",
                f"landmark score {score:.6g} below {LANDMARK_SCORE_THRESHOLD}",
            )
            return
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
                score,
                self._hand_landmark_handedness(packet),
                relative_landmark_z(landmarks_tensor, self.hand_landmark_input_size),
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
                    "imitation recovered after gathered result packet flow resumed."
                )
        elif imitation_status is not None and imitation_status["active"]:
            marked_failed = self.pipeline_manager.mark_failed(
                "imitation",
                "Imitation pipeline is not producing gathered result packets",
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
        for model in self.model_registry.selectable_models():
            runtime = statuses[model.model_id]
            status = ModelStatus()
            status.model_id = model.model_id
            status.active = runtime["active"]
            status.fps = float(runtime["fps"])
            status.shaves = runtime["shaves"]
            status.state = runtime["state"]
            status.message = runtime["message"]
            status_array.models.append(status)
        imu_runtime = self._imu_status()
        imu_status = ModelStatus()
        imu_status.model_id = "imu"
        imu_status.active = imu_runtime["state"] != "absent"
        # Report the rate actually observed rather than the configured rate: the
        # rig measured 8.1 Hz while the configuration said 10 Hz, and the gap
        # between configuration and delivery is exactly what this field exists
        # to expose at 100 Hz too.
        imu_status.fps = imu_runtime["fps"]
        imu_status.shaves = 0
        imu_status.state = imu_runtime["state"]
        imu_status.message = imu_runtime["message"]
        status_array.models.append(imu_status)
        self.models_status_publisher_.publish(status_array)

    def _imu_status(self, now=None):
        """Return the IMU extension carried by the existing models_status array."""

        if (
            not getattr(self, "imu_available", False)
            or getattr(self, "imu_queue", None) is None
        ):
            return {
                "state": "absent",
                "message": "OAK device has no usable IMU",
                "fps": 0.0,
            }
        if getattr(self, "_imu_last_received_monotonic", None) is None:
            return {
                "state": "stale",
                "message": "IMU configured but no report received",
                "fps": 0.0,
            }
        now = time.monotonic() if now is None else now
        age = now - self._imu_last_received_monotonic
        if age > IMU_STALE_AFTER_SECONDS:
            return {
                "state": "stale",
                "message": f"No IMU report received for {age:.1f} seconds",
                "fps": 0.0,
            }
        return {
            "state": "present",
            "message": "BMI270 IMU reports are flowing",
            # Rounded for the status channel: the field is consumed by humans and
            # by the UI, and fifteen digits are noise, not precision.
            "fps": round(
                measured_rate_hz(list(getattr(self, "_imu_publish_times", ()))), 2
            ),
        }

    def _init_imu(self):
        """Add the optional BMI270 stream without making camera startup depend on it."""

        self.imu_queue = None
        self.imu_available = False
        try:
            imu = self.pipeline.create(dai.node.IMU)
            # Only the raw outputs are requested. The BMI270 on the OAK-D Lite
            # rejects fused outputs outright ("IMU invalid settings!:
            # ROTATION_VECTOR output is unsupported. BMI270 supports only
            # ACCELEROMETER_RAW and/or GYROSCOPE_RAW outputs."), and that single
            # rejected sensor setting takes the whole pipeline start down.
            for sensor in (
                dai.IMUSensor.ACCELEROMETER_RAW,
                dai.IMUSensor.GYROSCOPE_RAW,
            ):
                # 200 Hz is supported by BMI270 for these reports. The host
                # deterministically drops every second sample to reach the
                # 100 Hz ROS rate.
                imu.enableIMUSensor(sensor, IMU_SENSOR_RATE_HZ)
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(IMU_OUTPUT_QUEUE_DEPTH)
            self.imu_queue = imu.out.createOutputQueue(
                maxSize=IMU_OUTPUT_QUEUE_DEPTH, blocking=False
            )
            self.imu_available = True
            return True
        except Exception as exc:
            self.imu_queue = None
            self.imu_available = False
            self.get_logger().warning(
                "BMI270 IMU unavailable; camera pipeline will continue without it: "
                f"{type(exc).__name__}: {exc}"
            )
            return False

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
        imitation_active = any(
            active.model.model_id == "imitation"
            for active in getattr(self, "_pipeline_models", ())
        )
        if imitation_active:
            self.camRgb.build(
                dai.CameraBoardSocket.CAM_A,
                sensorFps=IMITATION_FPS,
            )
        else:
            self.camRgb.build(dai.CameraBoardSocket.CAM_A)
        self.isp_out = self.camRgb.requestIspOutput()
        self.queue = self.isp_out.createOutputQueue(
            maxSize=COLOR_OUTPUT_QUEUE_DEPTH, blocking=False
        )
        self.depth_queue = None
        self.imu_queue = None
        self.imu_available = False
        self._imu_last_received_monotonic = None
        self._imu_last_sequence = None
        self._imu_last_device_stamp_ns = None
        # A new pipeline is a new device session, so the device clock restarts
        # near zero. Both the throttle deadline and the offset window describe
        # the previous session: keeping the deadline would drop every report
        # until the restarted device clock caught up, and keeping the offsets
        # would stamp with an offset measured against a clock that no longer
        # exists.
        self._imu_throttle = PublishRateThrottle()
        self._imu_clock_offset = ClockOffsetEstimator()
        self._imu_publish_times = deque(maxlen=IMU_RATE_WINDOW)
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
        self.hand_mp_pairs = {}
        self._pending_imitation_packet = None
        self.imitation_source_size = (0, 0)
        self.hand_mp_detection_queue = None
        self.hand_mp_landmark_queue = None
        self.hand_mp_source_size = (0, 0)
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
            if model.model_id == "hand_tracking_mp":
                self._build_hand_mp_pipeline(model)
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

    def _hand_mp_chain_is_built(self):
        requested = any(
            active.model.model_id == "hand_tracking_mp"
            for active in getattr(self, "_pipeline_models", ())
        )
        if not requested:
            return False
        return getattr(self, "hand_mp_landmark_queue", None) is not None

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
        # The reference's models (pib-rocks/imitation).  The post-processing and
        # the host-side decoding below belong together: this blob emits the
        # ``result`` records the decoder parses, and it caps them at top-2 on the
        # device, which is also why the decoder expects exactly two records.
        # The zoo's own three-blob set.  The decoding head is compiled without
        # -ip U8 so its float tensors match the detector's FP16 outputs; with the
        # image-input default it emitted a constant score and unusable geometry.
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
        """Add the official parsed palm, full-frame crop, and landmark graph."""
        artifact_ids = set(composite.artifact_ids)
        required_ids = {"palm_detection_sh4", "hand_landmark_full_sh4"}
        if not required_ids.issubset(artifact_ids):
            raise ValueError("imitation composite is missing its detector or landmark")
        # The composite's decoder is for the legacy raw-NN path. The parsed
        # archive graph decodes palms itself and must not allocate that blob.
        palm = self.model_registry.get("palm_detection_sh4")
        landmark = self.model_registry.get("hand_landmark_full_sh4")
        detection_archive = create_palm_archive(palm.blob_path)
        landmark_archive = create_landmark_archive(landmark.blob_path)

        detector_width = detection_archive.getInputWidth()
        detector_height = detection_archive.getInputHeight()
        landmark_width = landmark_archive.getInputWidth()
        landmark_height = landmark_archive.getInputHeight()

        detector_resize = self.pipeline.create(dai.node.ImageManip)
        detector_resize.setMaxOutputFrameSize(detector_width * detector_height * 3)
        detector_resize.initialConfig.setOutputSize(
            detector_width,
            detector_height,
            mode=dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        detector_resize.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        # The full 16:9 field of view is intentionally stretched to square. This
        # squeezes hands horizontally by about 1.78 and may reduce palm score,
        # but avoids the field-of-view loss of a square camera crop.
        #
        # The neural branches must NOT tap requestIspOutput(): that is the raw,
        # full-resolution ISP stream which also feeds the host queue publishing
        # /camera_topic. Hanging two device consumers on it froze the whole
        # pipeline (last_flowing=none) and took the camera image in Cerebra with
        # it. The Luxonis reference asks the camera for its own sized, rate
        # limited output instead - three consumers on THAT are fine, the running
        # example has two device consumers plus a host node on one output.
        imitation_source = self.camRgb.requestOutput(
            (IMITATION_SOURCE_WIDTH, IMITATION_SOURCE_HEIGHT),
            type=dai.ImgFrame.Type.BGR888p,
            fps=IMITATION_FPS,
        )
        if imitation_source is None:
            raise RuntimeError(
                "Camera cannot provide a "
                f"{IMITATION_SOURCE_WIDTH}x{IMITATION_SOURCE_HEIGHT} "
                "BGR888p branch for the imitation pipeline"
            )
        self.imitation_source_size = (
            IMITATION_SOURCE_WIDTH,
            IMITATION_SOURCE_HEIGHT,
        )
        imitation_source.link(detector_resize.inputImage)

        detection_nn = self.pipeline.create(ParsingNeuralNetwork).build(
            detector_resize.out,
            detection_archive,
        )
        detections_processor = self.pipeline.create(ProcessDetections).build(
            detections_input=detection_nn.out,
            padding=IMITATION_PALM_PADDING,
            target_size=(landmark_width, landmark_height),
        )

        cropper = (
            self.pipeline.create(FrameCropper)
            .fromManipConfigs(
                inputManipConfigs=detections_processor.config_output,
                maxOutputFrameSize=landmark_width * landmark_height * 3,
                waitForConfig=True,
            )
            .build(imitation_source)
        )
        # The crop runs in a device Script node whose ImageManip is built with
        # ``inputConfig.setWaitForMessage(True)``: while no palm detection
        # arrives there is no config, and the node keeps every frame it is
        # handed. Those frames belong to the camera's shared frame pool, so a
        # long stretch without a detection exhausts the pool and stops the WHOLE
        # device - the palm branch, the preview and /camera_topic with it. That
        # is the freeze named in PR-1778: measured 1208 frames at 8 Hz with zero
        # detections, then every stage stood still. Dropping frames instead of
        # blocking keeps the camera alive while nothing is detected, and a held
        # frame could not have produced a crop anyway.
        cropper_input = getattr(
            getattr(cropper, "_cropper_image_manip", None), "inputImage", None
        )
        if cropper_input is None:
            self.get_logger().warning(
                "FrameCropper internals changed: cannot stop it from holding "
                "frames while no detection arrives"
            )
        else:
            cropper_input.setMaxSize(1)
            cropper_input.setBlocking(False)
        pose_nn = self.pipeline.create(ParsingNeuralNetwork).build(
            cropper.out,
            landmark_archive,
        )
        gather_data = self.pipeline.create(GatherData).build(
            cameraFps=IMITATION_FPS,
            inputData=pose_nn.outputs,
            inputReference=detection_nn.out,
        )
        self.imitation_queue = gather_data.out.createOutputQueue(
            maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )
        self.imitation_source_size = (detector_width, detector_height)

    def _build_hand_mp_pipeline(self, composite):
        """Add the Luxonis hand-pose reference pipeline (PR-1791).

        Palm detector through a parsing network (the anchors are decoded host-side,
        so no decoding blob is needed), a host node that turns every palm detection
        into a timestamped crop config, the device-side FrameCropper, and the hand
        landmarker. The landmark stage deliberately runs as a plain NeuralNetwork
        rather than through the parsing network: depthai-nodes' KeypointParser clips
        the landmark components into 0..1, and the third component is the
        hand-relative depth this chain has to publish.
        """
        artifact_ids = set(composite.artifact_ids)
        required = {"palm_detection_128x128", "hand_landmark_224x224"}
        if not required.issubset(artifact_ids):
            raise ValueError(
                "hand_tracking_mp composite is missing its palm detector or landmarker"
            )
        palm = self.model_registry.get("palm_detection_128x128")
        landmark = self.model_registry.get("hand_landmark_224x224")
        detection_archive = create_palm_archive(palm.blob_path)

        detector_width = detection_archive.getInputWidth()
        detector_height = detection_archive.getInputHeight()
        landmark_width = landmark.input_width
        landmark_height = landmark.input_height

        detector_resize = self.pipeline.create(dai.node.ImageManip)
        detector_resize.setMaxOutputFrameSize(detector_width * detector_height * 3)
        detector_resize.initialConfig.setOutputSize(
            detector_width,
            detector_height,
            mode=dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        detector_resize.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)

        # One rate-limited camera branch with the 16:9 field of view, exactly as in
        # the example (there: 768x768, which crops the field of view instead).
        # No explicit fps here. The camera already carries the raw ISP output that
        # publishes /camera_topic, and asking one output for a different rate than
        # that stream leaves the ISP with nothing to deliver: measured with the
        # node's own pipeline, the colour queue stayed empty until the model chain
        # was taken out, while the same branch at the sensor's rate works. The rate
        # of the chain is set by the crop/landmark pairing, not by throttling the
        # camera.
        hand_mp_source = self.camRgb.requestOutput(
            (HAND_MP_SOURCE_WIDTH, HAND_MP_SOURCE_HEIGHT),
            type=dai.ImgFrame.Type.BGR888p,
        )
        if hand_mp_source is None:
            raise RuntimeError(
                "Camera cannot provide a "
                f"{HAND_MP_SOURCE_WIDTH}x{HAND_MP_SOURCE_HEIGHT} BGR888p branch"
            )
        self.hand_mp_source_size = (HAND_MP_SOURCE_WIDTH, HAND_MP_SOURCE_HEIGHT)
        hand_mp_source.link(detector_resize.inputImage)

        detection_nn = self.pipeline.create(ParsingNeuralNetwork).build(
            detector_resize.out, detection_archive
        )
        detections_processor = self.pipeline.create(ProcessDetections).build(
            detections_input=detection_nn.out,
            padding=IMITATION_PALM_PADDING,
            target_size=(landmark_width, landmark_height),
        )

        cropper = (
            self.pipeline.create(FrameCropper)
            .fromManipConfigs(
                inputManipConfigs=detections_processor.config_output,
                maxOutputFrameSize=landmark_width * landmark_height * 3,
                waitForConfig=True,
            )
            .build(hand_mp_source)
        )
        # Frames held by the cropper come out of the camera's shared frame pool;
        # without this a long stretch without a detection stops the whole device
        # (measured for the imitation chain: 1208 frames, then everything stood).
        cropper_input = getattr(
            getattr(cropper, "_cropper_image_manip", None), "inputImage", None
        )
        if cropper_input is None:
            self.get_logger().warning(
                "FrameCropper internals changed: cannot stop it from holding frames"
            )
        else:
            cropper_input.setMaxSize(HAND_MP_CROPPER_QUEUE)
            cropper_input.setBlocking(False)

        # Plain network: the raw NNData is what carries the unclipped z.
        landmark_nn = self.pipeline.create(dai.node.NeuralNetwork)
        landmark_nn.setBlobPath(landmark.blob_path)
        landmark_nn.setNumShavesPerInferenceThread(landmark.shaves)
        cropper.out.link(landmark_nn.input)
        self.hand_landmark_input_size = landmark_width

        # Two host-side streams: the parsed palm detections (for the box and the
        # score) and the raw landmark results. They are paired by timestamp, which
        # is what removes the one-frame-at-a-time round trip.
        self.hand_mp_detection_queue = detection_nn.out.createOutputQueue(
            maxSize=HAND_MP_PAIR_WINDOW, blocking=False
        )
        self.hand_mp_landmark_queue = landmark_nn.out.createOutputQueue(
            maxSize=HAND_MP_PAIR_WINDOW, blocking=False
        )

    @staticmethod
    def _hand_mp_stamp(packet):
        """Normalise a packet timestamp to a (seconds, nanoseconds) key.

        depthai hands out datetime.timedelta for device packets, the replay
        helpers use an object with sec/nanosec; both spellings are read here
        instead of assuming one of them.
        """
        stamp = packet.getTimestamp()
        seconds = getattr(stamp, "sec", None)
        if seconds is None:
            seconds = getattr(stamp, "seconds", 0)
        nanos = getattr(stamp, "nanosec", None)
        if nanos is None:
            nanos = getattr(stamp, "microseconds", 0) * 1000
        return (int(seconds), int(nanos))

    def _process_hand_mp(self):
        """Drain both streams and publish the hands they agree on."""
        # Both queues and the frame are required. Checking only one of them left
        # this called every loop iteration with a None queue whenever the chain
        # was not the active one, and an AttributeError there kills the whole
        # publish path without a traceback.
        if (
            self.hand_mp_landmark_queue is None
            or self.hand_mp_detection_queue is None
            or self.current_frame is None
        ):
            return
        for _ in range(HAND_MP_PAIR_WINDOW):
            packet = self.hand_mp_detection_queue.tryGet()
            if packet is None:
                break
            stamp = self._hand_mp_stamp(packet)
            entry = self.hand_mp_pairs.pop(stamp, None)
            if entry is not None:
                entry["detection"] = packet
            else:
                self.hand_mp_pairs[stamp] = {"detection": packet, "landmark": None}
        for _ in range(HAND_MP_PAIR_WINDOW):
            packet = self.hand_mp_landmark_queue.tryGet()
            if packet is None:
                break
            stamp = self._hand_mp_stamp(packet)
            entry = self.hand_mp_pairs.pop(stamp, None)
            if entry is None:
                self.hand_mp_pairs[stamp] = {"detection": None, "landmark": packet}
                continue
            detection = entry.get("detection")
            if detection is None:
                self.hand_mp_pairs[stamp] = {"detection": None, "landmark": packet}
                continue
            try:
                self._publish_hand_mp_pair(detection, packet)
            except (KeyError, TypeError, ValueError) as exc:
                self._warn_hand_once(f"Invalid hand_mp pair: {exc}")
        # Keep the buffer bounded: drop the oldest unpaired entries.
        while len(self.hand_mp_pairs) > MAX_HAND_MP_BUFFER:
            oldest = min(self.hand_mp_pairs)
            self.hand_mp_pairs.pop(oldest, None)

    def _publish_hand_mp_pair(self, detection_packet, landmark_packet):
        """Turn one matched (palm detection, landmark result) pair into detections."""
        frame_height, frame_width = self.current_frame.shape[:2]
        source_width, source_height = self.hand_mp_source_size
        score_tensor, landmark_score_value = self._hand_landmark_score(landmark_packet)
        layer_name, values, layer_probe = self._hand_landmark_tensor(landmark_packet)
        detections = []
        if (
            layer_name is not None
            and math.isfinite(landmark_score_value)
            and landmark_score_value >= LANDMARK_SCORE_THRESHOLD
        ):
            crop_points = landmarks_in_crop_pixels(
                values, self.hand_landmark_input_size
            )
            relative_z = relative_landmark_z(values, self.hand_landmark_input_size)
            palm = self._hand_mp_palm(detection_packet, frame_width, frame_height)
            mapped = self._map_hand_landmarks(
                landmark_packet,
                values,
                palm,
                frame_width,
                frame_height,
                source_width,
                source_height,
                score_tensor,
                layer_name,
                layer_probe,
            )
            if mapped:
                detections.append(
                    self._hand_mp_detection(
                        palm,
                        mapped,
                        relative_z,
                        landmark_score_value,
                        landmark_packet,
                        frame_width,
                        frame_height,
                        source_width,
                        source_height,
                    )
                )
        self._publish_hand_mp_detections(frame_width, frame_height, detections)

    def _hand_mp_palm(self, detection_packet, frame_width, frame_height):
        """Build the PalmRegion the mapping path needs from the parsed detection.

        The palm parser hands out dai-style detections whose geometry lives in a
        RotatedRect (``getBoundingBox()``), not in x_min/y_min/x_max/y_max: reading
        those attributes silently yielded zeros and an "empty box" for every hand.
        """
        detections = getattr(detection_packet, "detections", None) or []
        if not detections:
            raise ValueError("palm detection packet carries no detection")
        first = detections[0]
        rect = first.getBoundingBox()
        center_x = float(rect.center.x)
        center_y = float(rect.center.y)
        width = float(rect.size.width)
        height = float(rect.size.height)
        rotation = float(getattr(rect, "angle", 0.0) or 0.0)
        box_size = max(width, height)
        if box_size <= 0.0:
            raise ValueError("palm detection has an empty box")
        # The parser fills the detection's confidence (dai's ImgDetection field);
        # there is no "score" attribute, so reading it yielded NaN for every hand.
        palm_score = float("nan")
        for attribute in ("confidence", "score"):
            try:
                palm_score = float(getattr(first, attribute))
                break
            except (AttributeError, TypeError, ValueError):
                continue
        if abs(rotation) < 1e-6:
            # The reference computes the rotation from the wrist and middle-finger
            # anchors; the parser may leave the rect axis aligned.
            keypoints = list(getattr(first, "keypoints", None) or [])
            if len(keypoints) >= 3:
                try:
                    delta_x = float(keypoints[2].x) - float(keypoints[0].x)
                    delta_y = float(keypoints[2].y) - float(keypoints[0].y)
                    rotation = 0.5 * math.pi - math.atan2(-delta_y, delta_x)
                    rotation -= (
                        2 * math.pi * math.floor((rotation + math.pi) / (2 * math.pi))
                    )
                except (AttributeError, TypeError):
                    rotation = 0.0
        # The crop the device makes is the square box plus padding and is
        # letterboxed to the landmark input, so the region is a little larger.
        roi_size = box_size * (1.0 + 2.0 * float(IMITATION_PALM_PADDING))
        return PalmRegion(
            score=palm_score,
            box_x=center_x - 0.5 * box_size * math.cos(rotation),
            box_y=center_y - 0.5 * box_size * math.sin(rotation),
            box_size=box_size,
            roi_x=center_x,
            roi_y=center_y,
            roi_size=roi_size,
            rotation=rotation,
        )

    def _hand_mp_detection(
        self,
        palm,
        mapped,
        relative_z,
        landmark_score_value,
        landmark_packet,
        frame_width,
        frame_height,
        source_width,
        source_height,
    ):
        """Publish one hand with the box enclosing its keypoints, as the example does."""
        xs = [float(point[0]) for point in mapped]
        ys = [float(point[1]) for point in mapped]
        box_x = min(xs) / float(frame_width)
        box_y = min(ys) / float(frame_height)
        box_size = max(
            (max(xs) - min(xs)) / float(frame_width),
            (max(ys) - min(ys)) / float(frame_height),
        )
        enclosing = PalmRegion(
            score=palm.score,
            box_x=box_x,
            box_y=box_y,
            box_size=box_size,
            roi_x=box_x,
            roi_y=box_y,
            roi_size=box_size,
            rotation=0.0,
        )
        # Same reader the three-blob chain uses: it reshapes the head and returns
        # NaN when the blob exposes no usable handedness, where reading values[0]
        # by hand failed on the head's shape and silently published 0.0.
        handedness = float(self._hand_landmark_handedness(landmark_packet))
        if not math.isfinite(handedness):
            self._warn_hand_once(
                "hand_mp: landmark packet carries no usable handedness head"
            )
        return self._hand_detection_message(
            enclosing,
            [(float(p[0]), float(p[1])) for p in mapped],
            frame_width,
            frame_height,
            source_width,
            source_height,
            landmark_score_value=landmark_score_value,
            handedness=handedness,
            relative_z=relative_z,
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
            self.imu_queue = None
            self.imu_available = False
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
            self.hand_mp_pairs = {}
            self._pending_imitation_packet = None
            self.imitation_source_size = (0, 0)
            self.hand_mp_detection_queue = None
            self.hand_mp_landmark_queue = None
            self.hand_mp_source_size = (0, 0)
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
            include_imu = True
            for attempt in range(PIPELINE_START_ATTEMPTS):
                try:
                    self._build_pipeline(include_stereo)
                    if include_imu:
                        include_imu = self._init_imu()
                        if not include_imu:
                            # Discard a graph in which IMU construction failed
                            # after creating a partial node.
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
                    # Some OAK variants expose no IMU. If a graph containing
                    # dai.node.IMU is rejected at start, retry the same camera
                    # graph without it instead of taking down camera topics.
                    include_imu = False
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
                    active.model.model_id == "hand_tracking_mp"
                    and not self._hand_mp_chain_is_built()
                )
                or (
                    active.model.model_id
                    not in ("hand_tracking", "imitation", "hand_tracking_mp")
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
                "Cannot verify imitation: the gathered output queue is absent"
            )
            return False
        if "hand_tracking_mp" in requested_ids and not self._hand_mp_chain_is_built():
            self.get_logger().error(
                "Cannot verify hand_tracking_mp: the reference chain is absent"
            )
            return False
        if any(
            model_id not in ("hand_tracking", "imitation", "hand_tracking_mp")
            and model_id not in self.nn_queues
            for model_id in requested_ids
        ):
            # A composite chain has no nn_queues entry: it is verified through its
            # own stage queues below. Without this, every composite other than the
            # two named ones failed verification before a single frame was read,
            # and the caller reported "no frames arrived".
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
        if "hand_tracking_mp" in requested_ids:
            # The chain is only really running when the landmark stage answers;
            # waiting on the palm branch alone would accept a chain whose crops
            # never come back.
            packet = self._wait_for_queue_packet(self.hand_mp_landmark_queue, timeout)
            if packet is None:
                self.get_logger().error(
                    "hand_tracking_mp chain started but no landmark result arrived"
                )
                return False
            self._count_hand_stage("publish")
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

    def _stereo_requested(self) -> bool:
        """Whether this pipeline should include the stereo depth path.

        ``on`` and ``off`` keep their meaning. ``auto`` now means "depth while
        idle": with a model running, the model branch and the depth path share the
        same camera/ISP budget, and none of the models needs the depth - the
        published z is hand-relative and comes from the landmarker. Depth is back
        as soon as the last model stops.
        """
        if self.stereo_mode == "off":
            return False
        if self.stereo_mode == "on":
            return True
        return not getattr(self, "_pipeline_models", [])

    def init_pipeline(self) -> bool:
        self.depth_available = False
        self.current_depth = None
        self._pending_color_packet = None

        if not self._stereo_requested():
            if not self._start_pipeline(include_stereo=False):
                self.get_logger().error(
                    "Camera not found: colour pipeline failed to start."
                )
                return False
            reason = (
                "mode=off"
                if self.stereo_mode == "off"
                else "a model is running (mode=auto)"
            )
            self.get_logger().warning(
                f"Stereo depth disabled - using colour-only pipeline ({reason})"
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

    @staticmethod
    def _imu_report_xyz(report):
        return (report.x, report.y, report.z)

    @staticmethod
    def _imu_message(sample):
        """Copy an already mapped sample into sensor_msgs/Imu."""

        message = Imu()
        message.header.frame_id = sample.frame_id
        message.header.stamp.sec = sample.stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = sample.stamp_ns % 1_000_000_000
        message.linear_acceleration.x = sample.linear_acceleration.x
        message.linear_acceleration.y = sample.linear_acceleration.y
        message.linear_acceleration.z = sample.linear_acceleration.z
        message.angular_velocity.x = sample.angular_velocity.x
        message.angular_velocity.y = sample.angular_velocity.y
        message.angular_velocity.z = sample.angular_velocity.z
        # The device cannot fuse orientation (see imu.py), so the quaternion
        # stays at identity and orientation_covariance[0] == -1 marks it as
        # unavailable, which is the sensor_msgs/Imu convention consumers check.
        message.orientation.x = 0.0
        message.orientation.y = 0.0
        message.orientation.z = 0.0
        message.orientation.w = 1.0
        message.orientation_covariance = list(sample.orientation_covariance)
        message.angular_velocity_covariance = list(sample.angular_velocity_covariance)
        message.linear_acceleration_covariance = list(
            sample.linear_acceleration_covariance
        )
        return message

    def _process_imu(self):
        """Drain the non-blocking queue and publish at the configured rate."""

        if self.imu_queue is None:
            return
        for _ in range(IMU_DRAIN_LIMIT):
            batch = self.imu_queue.tryGet()
            if batch is None:
                break
            for packet in batch.packets:
                try:
                    acceleration = packet.acceleroMeter
                    angular_velocity = packet.gyroscope
                    # depthai 3.6.1 offers only getTimestamp() and
                    # getTimestampDevice() on the reports; the clock domains are
                    # documented in host_stamp_nanoseconds.
                    host_receipt_ns = host_stamp_nanoseconds(
                        acceleration.getTimestamp(), time.time()
                    )
                    # The device monotonic timestamp carries the measurement
                    # instant: the sensor spaces it uniformly, while host receipt
                    # stamps are quantised by the drain timer. It therefore
                    # drives both the throttle - throttling on receipt stamps
                    # measured 8.1 Hz where 10 Hz was configured - and, shifted
                    # onto the host timeline, the published stamp. It is also
                    # kept as the camera/IMU hardware correlation source.
                    device_stamp_ns = duration_to_nanoseconds(
                        acceleration.getTimestampDevice()
                    )
                    self._imu_last_device_stamp_ns = device_stamp_ns
                    self._imu_last_received_monotonic = time.monotonic()
                    # Every report feeds the offset window, including the ones
                    # the throttle discards: they cost nothing and a wider sample
                    # base can only lower the minimum towards the true offset.
                    offset_ns = self._imu_clock_offset.observe(
                        host_receipt_ns, device_stamp_ns
                    )
                    if not self._imu_throttle.accept(device_stamp_ns):
                        continue
                    stamp_ns = published_stamp_ns(
                        device_stamp_ns, host_receipt_ns, offset_ns
                    )
                    sample = assemble_imu_sample(
                        acceleration.getSequenceNum(),
                        stamp_ns,
                        self._imu_report_xyz(acceleration),
                        self._imu_report_xyz(angular_velocity),
                    )
                    # ROS 2 Header has no sequence field; retain DepthAI's source
                    # sequence here for diagnostics and testable packet mapping.
                    self._imu_last_sequence = sample.sequence
                    self.imu_publisher_.publish(self._imu_message(sample))
                    self._imu_publish_times.append(time.monotonic())
                except (AttributeError, TypeError, ValueError) as exc:
                    self.get_logger().warning(
                        f"Dropping invalid IMU report: {type(exc).__name__}: {exc}"
                    )

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
        try:
            self._process_hand_mp()
        except Exception as exc:  # pragma: no cover - defensive, verified by E2E
            self._warn_hand_once(f"hand_mp processing failed: {exc!r}")
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
