#!/usr/bin/python3
import base64
from collections import deque
import math
import os
import time
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
from .hand_tracking import (
    HAND_KEYPOINT_NAMES,
    decode_palm_result,
    map_landmarks_to_frame,
)

# Downscaled resolution for Haar cascade face detection (maps back to full frame).
FACE_DETECT_WIDTH = 320
FACE_DETECT_HEIGHT = 180
# Keep ImageManip warp inputs below the full ISP resolution.  In particular,
# dynamic landmark crops from the full OAK-D Lite ISP frame exceed RVC2's warp
# cache; 1280x720 preserves useful hand detail while staying within that limit.
HAND_NN_WIDTH = 1280
HAND_NN_HEIGHT = 720
STEREO_MODES = {"auto", "on", "off"}
DEFAULT_STEREO_TIMEOUT = 5.0
PIPELINE_START_ATTEMPTS = 3
PIPELINE_START_BACKOFF = 0.25


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
        self.hand_landmark_queue = None
        self.hand_landmark_config_queue = None
        self.hand_landmark_input_size = 0
        self.hand_source_size = (0, 0)
        self._pending_hands = deque()
        self._hand_warnings = set()

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

    @staticmethod
    def _nn_layer(packet, name):
        return np.asarray(packet.getLayerFp16(name), dtype=np.float32)

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
        self.pipeline_manager.record_packet("hand_tracking")

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
        detection.score = palm.score
        (
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
        ) = palm.bbox_pixels(frame_width, frame_height, source_width, source_height)
        detection.keypoint_names = list(HAND_KEYPOINT_NAMES)
        detection.keypoint_x = [point[0] for point in landmarks]
        detection.keypoint_y = [point[1] for point in landmarks]
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
            "detections": [],
            "frame_width": frame_width,
            "frame_height": frame_height,
            "source_width": source_width,
            "source_height": source_height,
        }
        for index, palm in enumerate(palms):
            roi_x, roi_y, roi_width, roi_height = palm.roi_for_frame(
                source_width, source_height
            )
            rotated = dai.RotatedRect()
            rotated.center.x = roi_x
            rotated.center.y = roi_y
            rotated.size.width = roi_width
            rotated.size.height = roi_height
            rotated.angle = math.degrees(palm.rotation)
            config = dai.ImageManipConfig()
            config.addCropRotatedRect(rotated, True)
            config.setOutputSize(
                self.hand_landmark_input_size,
                self.hand_landmark_input_size,
                dai.ImageManipConfig.ResizeMode.STRETCH,
            )
            config.setFrameType(dai.ImgFrame.Type.BGR888p)
            config.setReusePreviousImage(index + 1 < len(palms))
            self.hand_landmark_config_queue.send(config)
            self._pending_hands.append((palm, batch))

    def _process_hand_tracking(self):
        if self.hand_decoder_queue is None or self.current_frame is None:
            return

        while self._pending_hands:
            packet = self.hand_landmark_queue.tryGet()
            if packet is None:
                break
            palm, batch = self._pending_hands.popleft()
            try:
                score = self._nn_layer(packet, "Identity_1")
                landmarks_tensor = self._nn_layer(packet, "Identity_dense/BiasAdd/Add")
                if score.size != 1:
                    raise ValueError("landmark confidence must contain one value")
                if score[0] >= 0.5:
                    landmarks = map_landmarks_to_frame(
                        landmarks_tensor,
                        palm,
                        batch["frame_width"],
                        batch["frame_height"],
                        self.hand_landmark_input_size,
                        batch["source_width"],
                        batch["source_height"],
                    )
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
            except (RuntimeError, ValueError) as exc:
                self._warn_hand_once(f"Invalid hand landmark output: {exc}")
            batch["remaining"] -= 1
            if batch["remaining"] == 0:
                self._publish_hand_detections(
                    batch["frame_width"],
                    batch["frame_height"],
                    batch["detections"],
                )

        # Keep decoder and landmark packets paired; accept the next palm frame
        # only after all landmark crops from the previous one have completed.
        if self._pending_hands:
            return
        packet = self.hand_decoder_queue.tryGet()
        if packet is None:
            return
        frame_height, frame_width = self.current_frame.shape[:2]
        source_width, source_height = self.hand_source_size
        if not source_width or not source_height:
            source_width, source_height = self.current_source_size
        if not source_width or not source_height:
            source_width, source_height = frame_width, frame_height
        try:
            palms = decode_palm_result(self._nn_layer(packet, "result"))
        except (RuntimeError, ValueError) as exc:
            self._warn_hand_once(f"Invalid palm decoder output: {exc}")
            palms = []
        if not palms:
            self._publish_hand_detections(frame_width, frame_height, [])
            return
        self._queue_landmark_crops(
            palms,
            frame_width,
            frame_height,
            source_width,
            source_height,
        )

    def publish_model_statuses(self):
        self.pipeline_manager.refresh_fps()
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
        mono_left = self.pipeline.create(dai.node.Camera)
        mono_left.build(dai.CameraBoardSocket.CAM_B)
        mono_right = self.pipeline.create(dai.node.Camera)
        mono_right.build(dai.CameraBoardSocket.CAM_C)

        stereo = self.pipeline.create(dai.node.StereoDepth)
        try:
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        except Exception:
            pass
        stereo.setLeftRightCheck(True)
        try:
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        except Exception:
            self.get_logger().warning(
                "Depth-to-RGB align unavailable; using native depth."
            )

        mono_left_out = mono_left.requestFullResolutionOutput()
        mono_right_out = mono_right.requestFullResolutionOutput()
        mono_left_out.link(stereo.left)
        mono_right_out.link(stereo.right)

        self.depth_queue = stereo.depth.createOutputQueue()

    def _read_stereo_mode(self):
        mode = os.environ.get("PIB_CAMERA_STEREO", "off").strip().lower()
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

    def _build_pipeline(self, include_stereo):
        """Build one colour pipeline, optionally including the stereo path."""
        self.pipeline = dai.Pipeline()
        self.camRgb = self.pipeline.create(dai.node.Camera)
        self.camRgb.build(dai.CameraBoardSocket.CAM_A)
        self.isp_out = self.camRgb.requestIspOutput()
        self.queue = self.isp_out.createOutputQueue()
        self.depth_queue = None
        self.nn_queues = {}
        self.hand_decoder_queue = None
        self.hand_landmark_queue = None
        self.hand_landmark_config_queue = None
        self.hand_landmark_input_size = 0
        self.hand_source_size = (0, 0)
        if hasattr(self, "_pending_hands"):
            self._pending_hands.clear()
        else:
            self._pending_hands = deque()

        if include_stereo:
            self._init_stereo_depth()

        for active_model in getattr(self, "_pipeline_models", []):
            model = active_model.model
            if model.model_id == "hand_tracking":
                self._build_hand_pipeline(model)
                continue
            neural_network = self.pipeline.create(dai.node.NeuralNetwork)
            neural_network.setBlobPath(model.blob_path)
            neural_network.setNumShavesPerInferenceThread(model.shaves)
            nn_input = self.camRgb.requestOutput(
                (model.input_width, model.input_height),
                type=dai.ImgFrame.Type.BGR888p,
            )
            nn_input.link(neural_network.input)
            self.nn_queues[model.model_id] = neural_network.out.createOutputQueue()

    def _build_hand_pipeline(self, composite):
        """Add palm resize/detect/decode and dynamic hand ROI landmarks."""
        artifacts = {
            model_id: self.model_registry.get(model_id)
            for model_id in composite.artifact_ids
        }
        palm = artifacts["palm_detection_128x128"]
        decoder = artifacts["palm_detection_128x128_decoding"]
        landmark = artifacts["hand_landmark_224x224"]
        self.hand_landmark_input_size = landmark.input_width
        self.hand_source_size = (HAND_NN_WIDTH, HAND_NN_HEIGHT)

        hand_input = self.camRgb.requestOutput(
            self.hand_source_size,
            type=dai.ImgFrame.Type.BGR888p,
        )

        palm_manip = self.pipeline.create(dai.node.ImageManip)
        palm_manip.initialConfig.setOutputSize(
            palm.input_width,
            palm.input_height,
            dai.ImageManipConfig.ResizeMode.LETTERBOX,
        )
        palm_manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        hand_input.link(palm_manip.inputImage)

        palm_nn = self.pipeline.create(dai.node.NeuralNetwork)
        palm_nn.setBlobPath(palm.blob_path)
        palm_nn.setNumShavesPerInferenceThread(palm.shaves)
        palm_manip.out.link(palm_nn.input)

        decoder_nn = self.pipeline.create(dai.node.NeuralNetwork)
        decoder_nn.setBlobPath(decoder.blob_path)
        decoder_nn.setNumShavesPerInferenceThread(decoder.shaves)
        palm_nn.out.link(decoder_nn.input)
        self.hand_decoder_queue = decoder_nn.out.createOutputQueue()

        landmark_manip = self.pipeline.create(dai.node.ImageManip)
        landmark_manip.initialConfig.setOutputSize(
            landmark.input_width,
            landmark.input_height,
            dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        landmark_manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        hand_input.link(landmark_manip.inputImage)
        self.hand_landmark_config_queue = landmark_manip.inputConfig.createInputQueue(
            maxSize=16, blocking=False
        )

        landmark_nn = self.pipeline.create(dai.node.NeuralNetwork)
        landmark_nn.setBlobPath(landmark.blob_path)
        landmark_nn.setNumShavesPerInferenceThread(landmark.shaves)
        landmark_manip.out.link(landmark_nn.input)
        self.hand_landmark_queue = landmark_nn.out.createOutputQueue()

    def _stop_pipeline(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None
        self.queue = None
        self.depth_queue = None
        self.nn_queues = {}
        self.hand_decoder_queue = None
        self.hand_landmark_queue = None
        self.hand_landmark_config_queue = None
        self.hand_landmark_input_size = 0
        self.hand_source_size = (0, 0)
        if hasattr(self, "_pending_hands"):
            self._pending_hands.clear()

    def _start_pipeline(self, include_stereo):
        """Build and start a fresh pipeline with bounded, silent retries."""
        for attempt in range(PIPELINE_START_ATTEMPTS):
            try:
                self._build_pipeline(include_stereo)
                self.pipeline.start()
                return True
            except Exception:
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

    def _rebuild_models(self, active_models):
        self._pipeline_models = list(active_models)
        self._stop_pipeline()
        self.camera_available = self.init_pipeline()
        return self.camera_available

    def _verify_model_frames(self, timeout):
        packet = self._wait_for_color_frame(timeout)
        if packet is None:
            return False
        self._pending_color_packet = packet
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

        self._process_hand_tracking()

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

        self._stop_pipeline()
        self.camera_available = self.init_pipeline()


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
