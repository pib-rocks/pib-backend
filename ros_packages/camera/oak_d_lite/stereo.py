#!/usr/bin/python3
import base64
from typing import Optional
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
import json
from urllib.request import urlopen
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Float32MultiArray, Float64, Int32, Int32MultiArray, String

from .mjpeg_server import (
    DEFAULT_STREAM_PORT,
    MjpegFrameBuffer,
    MjpegStreamServer,
)

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
    ProcessGazeEyes,
    box_from_points,
    gathered_hands,
    gathered_result_trace_values,
    world_landmark_scalars,
)
from .imitation_archive import create_landmark_archive, create_palm_archive
from .face_crop import (
    FACE_CROP_EXCLUSIVITY_RULE,
    FACE_CROP_TRANSLATORS,
    FACE_CROP_PADDING,
    FACE_DETECTOR_MODEL_ID,
    GAZE_MODEL_ID,
    HEAD_POSE_MODEL_ID,
    face_crop_classifier_id,
    is_face_crop_composite,
    is_gaze_composite,
    packet_timestamp,
    translate_gaze,
)
from .parsed_detections import published_field, translate_detections
from .qr_detection import (
    QR_MODEL_ID,
    decode_qr_detections,
    validate_qr_blob,
)
from .task_archives import create_archive, labels_for_model
from .hand_tracking import (
    FAST_MODEL_ID,
    HAND_KEYPOINT_NAMES,
    LANDMARK_HANDEDNESS_LAYER,
    LANDMARK_SCORE_LAYER,
    LANDMARK_SCORE_THRESHOLD,
    LANDMARK_VALUE_COUNT,
    LANDMARK_XYZ_LAYERS,
    MANIP_CROP_INSET_PIXELS,
    build_fast_hand_graph,
    decode_palm_result,
    fit_manip_crop,
    landmark_score,
    landmarks_in_crop_pixels,
    map_fast_hand,
    map_landmarks_to_frame,
    parse_fast_script_result,
    relative_landmark_z,
    PalmRegion,
)

from .branch_capture_mixin import BranchCaptureMixin
from .hand_tracking_mixin import HandTrackingMixin
from .pipeline_builders_mixin import PipelineBuildersMixin

# The OAK-D Lite is an RVC2 board with 16 SHAVE cores
# (https://docs.luxonis.com/hardware/platform/rvc/rvc2/). This is the board's
# total, not a model-store value.
TOTAL_SHAVES = 16
# Downscaled resolution for Haar cascade face detection (maps back to full frame).
FACE_DETECT_WIDTH = 320
FACE_DETECT_HEIGHT = 180
# Frame the UI displays. A model colour branch must share this aspect: the
# camera crops any other aspect, and a later per-axis scale cannot put that
# crop back onto the frame.
PUBLISHED_FRAME_WIDTH = 1280
PUBLISHED_FRAME_HEIGHT = 720
COMPRESSED_IMAGE_TOPIC = "/oak/rgb/image_raw/compressed"
COMPRESSED_IMAGE_FRAME_ID = "oak_rgb_camera_optical_frame"
DEFAULT_QUALITY_FACTOR = 80
DEFAULT_TIMER_PERIOD_SECONDS = 0.1
MIN_TIMER_PERIOD_SECONDS = 1.0 / 30.0
MAX_STREAM_FPS = 30
# Full-field branch at that aspect, nine tenths of the preview. Model inputs
# are stretched from this branch; the camera is not asked for their aspect.
# A stretch keeps fractions, so a detection maps back through the window the
# branch actually covers. That window is read from the delivered frame.
COLOUR_BRANCH_WIDTH = 1152
COLOUR_BRANCH_HEIGHT = (
    COLOUR_BRANCH_WIDTH * PUBLISHED_FRAME_HEIGHT // PUBLISHED_FRAME_WIDTH
)
# Opt-in capture of that delivered window beside the preview. Absent unless
# PIB_COLOUR_BRANCH_FIELD names a directory, or the operator creates the
# request file. The running camera node writes both frames itself.
COLOUR_BRANCH_FIELD_REQUEST = "/tmp/pib-colour-branch-field.request"
COLOUR_BRANCH_FIELD_DIR = "/tmp/pib-colour-branch-field"
# The palm warp downscales this branch to 128x128 and stays within 2:1 on the
# long side. The short side follows the published aspect, so the branch is the
# whole frame. A square branch would be a centred crop.
# hand_tracking_fast uses this same output. One manipulation from the 2104x1560
# ISP frame down to the 128 palm input fails with WARP_SWCH_ERR_CACHE_TOO_SMALL.
HAND_NN_WIDTH = 256
HAND_NN_HEIGHT = HAND_NN_WIDTH * PUBLISHED_FRAME_HEIGHT // PUBLISHED_FRAME_WIDTH
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
IMITATION_SOURCE_WIDTH = COLOUR_BRANCH_WIDTH
IMITATION_SOURCE_HEIGHT = COLOUR_BRANCH_HEIGHT
FACE_CROP_SOURCE_WIDTH = COLOUR_BRANCH_WIDTH
FACE_CROP_SOURCE_HEIGHT = COLOUR_BRANCH_HEIGHT
FACE_CROP_PAIR_WINDOW = 32
MAX_FACE_CROP_BUFFER = 64
# Exact timestamp pairing needs more than a single held frame. Four delivered
# full classifier throughput in the measured hand-chain reference while still
# bounding use of the camera's shared frame pool.
FACE_CROPPER_QUEUE = 4
# The Luxonis hand-pose reference pipeline (PR-1791): palm detector, device-side
# FrameCropper and hand landmarker, timestamp-matched. Same rate limit the example
# uses on RVC2. The branch keeps the 16:9 field of view so the neural branch and
# the published frame agree.
HAND_MP_FPS = 8
HAND_MP_SOURCE_WIDTH = COLOUR_BRANCH_WIDTH
HAND_MP_SOURCE_HEIGHT = COLOUR_BRANCH_HEIGHT
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
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_image = ""

    def timer_callback(self):
        return


class CameraNode(
    HandTrackingMixin,
    PipelineBuildersMixin,
    BranchCaptureMixin,
    Node,
):
    """Own and publish the OAK-D Lite camera and its on-board BMI270 IMU.

    On pib the OAK-D Lite is mounted in the head, facing forward. The BMI270 is
    soldered to that camera PCB, so ``oak_imu_frame`` is at the camera module,
    not at the robot base or torso origin.
    """

    _device_lifecycle_lock = threading.RLock()
    _device_owner = None

    def __init__(self):
        super().__init__("camera_node")

        self.compressed_publisher_ = self.create_publisher(
            CompressedImage, COMPRESSED_IMAGE_TOPIC, 10
        )
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

        self.preview_width = PUBLISHED_FRAME_WIDTH
        self.preview_height = PUBLISHED_FRAME_HEIGHT
        self.quality_factor = DEFAULT_QUALITY_FACTOR
        self.timer_period = DEFAULT_TIMER_PERIOD_SECONDS
        self.current_jpeg_bytes = b""
        self.current_image = ""
        self.current_frame = None
        self.current_source_size = (0, 0)
        self.current_depth = None
        self.pipeline = None
        self.video_manip = None
        self.video_encoder = None
        self.mjpeg_buffer = MjpegFrameBuffer()
        self.mjpeg_server = None
        self._load_startup_camera_settings()
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
        self._pending_nn_packets = {}
        self.qr_detector = cv2.QRCodeDetector()
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
        self.face_crop_detection_queue = None
        self.face_crop_result_queue = None
        self.face_crop_pairs = {}
        self.face_crop_source_size = (0, 0)
        self._pending_face_crop_packet = None
        self.face_crop_model_id = None
        self.face_crop_translator = None
        self.hand_mp_detection_queue = None
        self.hand_mp_landmark_queue = None
        self.hand_mp_source_size = (0, 0)
        self.hand_fast_queue = None
        self.hand_fast_source_size = (0, 0)
        self._pending_hand_fast_packet = None
        self._preview_packet = None
        self._colour_branch_field = None
        self._colour_branch_capture_queues = {}
        self._colour_branch_capture_written = False
        self._colour_branch_capture_dir = None
        self._colour_branch_capture = self._colour_branch_capture_requested()
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
        self._start_mjpeg_server()
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
        # Prefer the hardware-encoded JPEG bytes from the OAK VideoEncoder.
        jpeg_bytes = getattr(self, "current_jpeg_bytes", b"")
        if jpeg_bytes:
            self.current_image = base64.b64encode(jpeg_bytes).decode("utf-8")
        elif self.current_frame is not None:
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
        response.total_shaves = TOTAL_SHAVES
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
        # Success is returned only after rebuild and frame verification inside
        # PipelineManager.start. rosbridge abandons the call at
        # ROSBRIDGE_DEFAULT_CALL_SERVICE_TIMEOUT_SECONDS when the call_service
        # message omits timeout, which is shorter than this callback, so the
        # client must send MODEL_LIFECYCLE_CALL_TIMEOUT_SECONDS.
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
        if not self._has_active_frame():
            detections.frame_width = 0
            detections.frame_height = 0
        else:
            detections.frame_width, detections.frame_height = (
                self._active_frame_dimensions()
            )
        detections.detections = []
        return detections

    def get_detections_callback(self, request, response):
        response.detections = self.last_detections.get(
            request.model_id, self._empty_detections(request.model_id)
        )
        return response

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

    def _publish_parsed_detections(self, model_id, packet):
        frame_width, frame_height = self._active_frame_dimensions()
        field = self._object_detection_field(packet, frame_width, frame_height)
        detections = translate_detections(
            packet,
            labels_for_model(model_id),
            frame_width,
            frame_height,
            field,
        )

        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = model_id
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections[model_id] = message
        publisher = self.detection_publishers.get(model_id)
        if publisher is not None:
            publisher.publish(message)

    def _publish_qr_detections(self, packet):
        frame = self._decode_current_frame()
        if frame is None:
            raise ValueError("QR detections cannot be decoded without a cached frame")
        frame_height, frame_width = frame.shape[:2]
        decoded = decode_qr_detections(packet, frame, detector=self.qr_detector)
        detections = []
        for result in decoded:
            detection = Detection()
            detection.label = result.text
            detection.score = result.confidence
            (
                detection.x_min,
                detection.y_min,
                detection.x_max,
                detection.y_max,
            ) = result.box
            detection.keypoint_names = [
                "top_left",
                "top_right",
                "bottom_right",
                "bottom_left",
            ]
            detection.keypoint_x = [point[0] for point in result.corners]
            detection.keypoint_y = [point[1] for point in result.corners]
            detection.keypoint_z = [0.0] * 4
            detection.scalar_names = []
            detection.scalar_values = []
            detections.append(detection)

        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = QR_MODEL_ID
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections[QR_MODEL_ID] = message
        publisher = self.detection_publishers.get(QR_MODEL_ID)
        if publisher is not None:
            publisher.publish(message)

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

    def publish_face_center(self, frame=None):
        # Skip expensive Haar cascade when nobody is listening to face_center.
        if self.face_center_publisher_.get_subscription_count() == 0:
            return
        if frame is None:
            frame = self._decode_current_frame()
        if frame is None:
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
        if self._poll_colour_branch_capture():
            return
        if self.queue:
            image_rgb = self._pending_color_packet
            self._pending_color_packet = None
            if image_rgb is None:
                image_rgb = self.queue.tryGet()
            if image_rgb is not None:
                self._preview_packet = image_rgb
                jpeg_bytes = self._extract_packet_jpeg(image_rgb)
                if jpeg_bytes is not None:
                    self.current_jpeg_bytes = jpeg_bytes
                    self.current_frame = None
                    self.current_source_size = self._published_frame_size()
                    if getattr(self, "mjpeg_buffer", None) is not None:
                        self.mjpeg_buffer.publish(jpeg_bytes)
                    self._publish_compressed_image(jpeg_bytes)
                    if self.hand_decoder_queue is not None:
                        self._count_hand_stage("colour_isp")
                    if self.imitation_queue is not None:
                        self._count_imitation_stage("colour_isp")
                    self.publish_face_center(None)
                else:
                    frame = image_rgb.getCvFrame()
                    self.current_source_size = (frame.shape[1], frame.shape[0])
                    if (
                        frame.shape[1] != self.preview_width
                        or frame.shape[0] != self.preview_height
                    ):
                        frame = cv2.resize(
                            frame, (self.preview_width, self.preview_height)
                        )
                    self.current_frame = frame
                    self.current_jpeg_bytes = b""
                    if self.hand_decoder_queue is not None:
                        self._count_hand_stage("colour_isp")
                    if self.imitation_queue is not None:
                        self._count_imitation_stage("colour_isp")
                    self.publish_face_center(frame)
                    if (
                        getattr(self, "compressed_publisher_", None) is not None
                        and self.compressed_publisher_.get_subscription_count() > 0
                    ):
                        ok, buf = cv2.imencode(
                            ".jpg",
                            frame,
                            [int(cv2.IMWRITE_JPEG_QUALITY), self.quality_factor],
                        )
                        if ok:
                            raw_jpeg = bytes(buf)
                            self.current_jpeg_bytes = raw_jpeg
                            if getattr(self, "mjpeg_buffer", None) is not None:
                                self.mjpeg_buffer.publish(raw_jpeg)
                            self._publish_compressed_image(raw_jpeg)

        self._drain_colour_branch_capture()

        for model_id, nn_queue in self.nn_queues.items():
            packet = self._pending_nn_packets.pop(model_id, None)
            for _ in range(32):
                if packet is None:
                    packet = nn_queue.tryGet()
                if packet is None:
                    break
                if model_id == QR_MODEL_ID:
                    try:
                        self._publish_qr_detections(packet)
                    except (AttributeError, TypeError, ValueError) as exc:
                        self.get_logger().error(
                            f"QR_DROP type={type(exc).__name__} message={exc}"
                        )
                elif model_id in getattr(self, "parsed_model_ids", ()):
                    try:
                        self._publish_parsed_detections(model_id, packet)
                    except (AttributeError, TypeError, ValueError) as exc:
                        error_count = self._parsed_error_counts.get(model_id, 0) + 1
                        self._parsed_error_counts[model_id] = error_count
                        if error_count % 25 == 1:
                            self.get_logger().error(
                                f"PARSED_DROP model={model_id} count={error_count} "
                                f"type={type(exc).__name__} message={exc}"
                            )
                self.pipeline_manager.record_packet(model_id)
                packet = None

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
        try:
            self._process_hand_tracking_fast()
        except Exception as exc:  # pragma: no cover - defensive, verified by E2E
            self._warn_hand_once(f"hand_tracking_fast processing failed: {exc!r}")
        self._process_imitation()
        try:
            self._process_face_crop()
        except Exception as exc:
            self.get_logger().warning(f"Face-crop processing failed: {exc!r}")

        if not self.depth_queue:
            return

        depth_packet = self.depth_queue.tryGet()
        if depth_packet is None:
            return

        depth = depth_packet.getFrame()
        self.current_depth = depth
        self._publish_colorized_depth(depth)

    def timer_period_callback(self, msg):
        requested = float(msg.data)
        if not math.isfinite(requested) or requested <= 0:
            return
        self.timer_period = max(MIN_TIMER_PERIOD_SECONDS, requested)
        self.timer.cancel()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        with self._pipeline_lock:
            self._stop_pipeline()
            self.camera_available = self.init_pipeline()

    def quality_factor_callback(self, msg):
        self.quality_factor = max(1, min(100, int(msg.data)))
        with self._pipeline_lock:
            self._stop_pipeline()
            self.camera_available = self.init_pipeline()

    def preview_size_callback(self, msg):
        self.preview_width, self.preview_height = int(msg.data[0]), int(msg.data[1])

        with self._pipeline_lock:
            self._stop_pipeline()
            self.camera_available = self.init_pipeline()

    def destroy_node(self):
        if getattr(self, "mjpeg_server", None) is not None:
            self.mjpeg_server.stop()
            self.mjpeg_server = None
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
