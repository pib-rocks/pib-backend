"""Unit tests for stereo camera CPU optimization (PR-1507)."""

import os
import sys
import threading
import types
import unittest
import weakref
from collections import deque
from datetime import datetime, timedelta, timezone
from unittest.mock import MagicMock, patch
import base64

import cv2

if not hasattr(cv2, "CascadeClassifier"):
    cv2.CascadeClassifier = MagicMock()

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../ros_packages/camera/oak_d_lite")
    ),
)

try:
    import depthai  # noqa: F401
except ImportError:
    sys.modules["depthai"] = types.ModuleType("depthai")


def _usable_rclpy_node():
    """Return a usable ``rclpy.node.Node`` class, or ``None``.

    Importing ``rclpy`` alone cannot decide whether the real client library is
    available: an earlier test module in the same pytest session can leave a
    stub ``rclpy`` in ``sys.modules``. The import then succeeds while the node
    class lacks methods this suite exercises (``create_subscription`` and
    friends), and the failure surfaces later as an ``AttributeError`` from the
    code under test. So the probe checks the API this module depends on and
    treats an incomplete module as absent, which builds the local fake below.
    """
    required = (
        "create_publisher",
        "create_subscription",
        "create_service",
        "create_timer",
        "destroy_node",
    )
    try:
        from rclpy.node import Node
    except ImportError:
        return None
    if all(callable(getattr(Node, name, None)) for name in required):
        return Node
    return None


if _usable_rclpy_node() is None:
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")

    class _FakeNode:
        def __init__(self, *args, **kwargs):
            self._logger = MagicMock()

        def create_publisher(self, *args, **kwargs):
            return MagicMock()

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def create_service(self, *args, **kwargs):
            return MagicMock()

        def create_timer(self, *args, **kwargs):
            return MagicMock()

        def get_logger(self):
            return self._logger

        def destroy_node(self):
            pass

    rclpy_node.Node = _FakeNode
    rclpy.node = rclpy_node
    rclpy.init = MagicMock()
    rclpy.shutdown = MagicMock()
    rclpy.spin = MagicMock()
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

try:
    import std_msgs.msg  # noqa: F401
except ImportError:
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class _Message:
        def __init__(self):
            self.data = None

    std_msgs_msg.Float32MultiArray = _Message
    std_msgs_msg.Float64 = _Message
    std_msgs_msg.Int32 = _Message
    std_msgs_msg.Int32MultiArray = _Message
    std_msgs_msg.String = _Message
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

try:
    import sensor_msgs.msg  # noqa: F401
except ImportError:
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class _Imu:
        def __init__(self):
            vector = lambda: types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.header = types.SimpleNamespace(
                frame_id="", stamp=types.SimpleNamespace(sec=0, nanosec=0)
            )
            self.linear_acceleration = vector()
            self.angular_velocity = vector()
            self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)
            self.orientation_covariance = [0.0] * 9
            self.angular_velocity_covariance = [0.0] * 9
            self.linear_acceleration_covariance = [0.0] * 9

    sensor_msgs_msg.Imu = _Imu
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

try:
    import datatypes.srv as _datatypes_srv
except ImportError:
    _datatypes = types.ModuleType("datatypes")
    _datatypes_srv = types.ModuleType("datatypes.srv")
    _datatypes_msg = types.ModuleType("datatypes.msg")
    _datatypes.msg = _datatypes_msg
    _datatypes.srv = _datatypes_srv
    sys.modules["datatypes"] = _datatypes
    sys.modules["datatypes.srv"] = _datatypes_srv
    sys.modules["datatypes.msg"] = _datatypes_msg

    class _DummySrv:
        class Request:
            pass

        class Response:
            pass

    _datatypes_srv.GetCameraImage = _DummySrv
    _datatypes_srv.GetDepthFrame = _DummySrv
    _datatypes_srv.GetDistanceAtPx = _DummySrv
    _datatypes_srv.GetDetections = _DummySrv
    _datatypes_srv.ListModels = _DummySrv
    _datatypes_srv.StartModel = _DummySrv
    _datatypes_srv.StopModel = _DummySrv

    class _DummyMsg:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    _datatypes_msg.Detection = _DummyMsg
    _datatypes_msg.DetectionArray = _DummyMsg
    _datatypes_msg.ModelInfo = _DummyMsg
    _datatypes_msg.ModelStatus = _DummyMsg
    _datatypes_msg.ModelStatusArray = _DummyMsg
else:
    if not hasattr(_datatypes_srv, "GetDepthFrame"):

        class _DummySrv:
            class Request:
                pass

            class Response:
                pass

        _datatypes_srv.GetDepthFrame = _DummySrv
        _datatypes_srv.GetDistanceAtPx = _DummySrv

import numpy as np

from ros_packages.camera.oak_d_lite.hand_tracking import PalmRegion
from ros_packages.camera.oak_d_lite.imu import (
    ClockOffsetEstimator,
    IMU_CLOCK_OFFSET_WINDOW,
    IMU_PUBLISH_PERIOD_NS,
    IMU_PUBLISH_RATE_HZ,
    PublishRateThrottle,
    assemble_imu_sample,
    estimated_clock_offset_ns,
    host_stamp_nanoseconds,
    measured_rate_hz,
    published_stamp_ns,
)
from ros_packages.camera.oak_d_lite.stereo import (
    BRANCH_INPUT_QUEUE_DEPTH,
    BRANCH_OUTPUT_QUEUE_DEPTH,
    COLOR_OUTPUT_QUEUE_DEPTH,
    CameraNode,
    FACE_DETECT_WIDTH,
    FACE_DETECT_HEIGHT,
    HAND_STAGE_NAMES,
    HAND_NN_HEIGHT,
    HAND_NN_WIDTH,
    IMITATION_DETECTOR_MODEL,
    IMITATION_FPS,
    IMITATION_LANDMARK_MODEL,
    IMITATION_SOURCE_HEIGHT,
    IMITATION_SOURCE_WIDTH,
    IMITATION_STAGE_NAMES,
    IMU_DRAIN_LIMIT,
    IMU_OUTPUT_QUEUE_DEPTH,
    IMU_POLL_PERIOD_SECONDS,
    IMU_RATE_WINDOW,
    IMU_SENSOR_RATE_HZ,
    IMU_STALE_AFTER_SECONDS,
    IMU_STALE_MISSED_PUBLICATIONS,
)


def _hand_artifacts():
    """Registry entries for the three blobs the hand chain is built from."""
    return {
        "palm_detection_128x128": types.SimpleNamespace(
            input_width=128,
            input_height=128,
            blob_path="/palm.blob",
            shaves=4,
        ),
        "palm_detection_128x128_decoding": types.SimpleNamespace(
            blob_path="/decoder.blob", shaves=1
        ),
        "hand_landmark_224x224": types.SimpleNamespace(
            input_width=224,
            input_height=224,
            blob_path="/landmark.blob",
            shaves=4,
        ),
    }


def _imitation_artifacts():
    return {
        "palm_detection_sh4": types.SimpleNamespace(
            input_width=128,
            input_height=128,
            blob_path="/palm-imitation.blob",
            shaves=4,
        ),
        "pd_postprocessing_top2_sh1": types.SimpleNamespace(
            blob_path="/decoder-imitation.blob", shaves=1
        ),
        "hand_landmark_full_sh4": types.SimpleNamespace(
            input_width=224,
            input_height=224,
            blob_path="/landmark-imitation.blob",
            shaves=4,
        ),
    }


class TestStereoCameraOptimization(unittest.TestCase):

    @patch("ros_packages.camera.oak_d_lite.stereo.cv2.CascadeClassifier")
    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_publish_face_center_skips_when_no_subscribers(
        self, mock_exists, mock_dai, mock_cascade
    ):
        with patch.object(CameraNode, "init_pipeline", return_value=True):
            node = CameraNode()
            node.face_center_publisher_ = MagicMock()
            node.face_center_publisher_.get_subscription_count.return_value = 0

            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            node.publish_face_center(frame)

            # CascadeClassifier detectMultiScale should NOT be called
            node.face_cascade.detectMultiScale.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.cv2.CascadeClassifier")
    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_publish_face_center_downscales_frame_when_subscribed(
        self, mock_exists, mock_dai, mock_cascade
    ):
        with patch.object(CameraNode, "init_pipeline", return_value=True):
            node = CameraNode()
            node.face_center_publisher_ = MagicMock()
            node.face_center_publisher_.get_subscription_count.return_value = 1
            node.face_cascade = MagicMock()
            node.face_cascade.empty.return_value = False
            node.face_cascade.detectMultiScale.return_value = []

            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            node.publish_face_center(frame)

            # detectMultiScale should be called on downscaled gray image
            self.assertTrue(node.face_cascade.detectMultiScale.called)
            gray_arg = node.face_cascade.detectMultiScale.call_args[0][0]
            self.assertEqual(gray_arg.shape, (FACE_DETECT_HEIGHT, FACE_DETECT_WIDTH))

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_timer_callback_skips_encoding_when_no_camera_subscribers(
        self, mock_exists, mock_dai
    ):
        with patch.object(CameraNode, "init_pipeline", return_value=True):
            node = CameraNode()
            node.publisher_ = MagicMock()
            node.publisher_.get_subscription_count.return_value = 0
            node.publish_face_center = MagicMock()
            node._encode_frame = MagicMock()

            # Mock DepthAI queue
            mock_img = MagicMock()
            mock_img.getCvFrame.return_value = np.zeros((720, 1280, 3), dtype=np.uint8)
            node.queue = MagicMock()
            node.queue.tryGet.return_value = mock_img

            node.timer_callback()

            # Cached frame updated
            self.assertIsNotNone(node.current_frame)
            # _encode_frame NOT called since no subscribers on camera_topic
            node._encode_frame.assert_not_called()
            node.publisher_.publish.assert_not_called()


class TestStereoDepthInterfaces(unittest.TestCase):

    def _make_node(self):
        with patch.object(CameraNode, "init_pipeline", return_value=True):
            node = CameraNode()
        node.depth_available = True
        node.publisher_ = MagicMock()
        node.publisher_.get_subscription_count.return_value = 0
        node.depth_publisher_ = MagicMock()
        node.depth_publisher_.get_subscription_count.return_value = 0
        node.publish_face_center = MagicMock()
        return node

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_timer_callback_caches_depth_without_publishing(
        self, mock_exists, mock_dai
    ):
        node = self._make_node()
        node.queue = MagicMock()
        node.queue.tryGet.return_value = None

        mock_depth = MagicMock()
        depth = np.array([[0, 1200], [800, 1500]], dtype=np.uint16)
        mock_depth.getFrame.return_value = depth
        node.depth_queue = MagicMock()
        node.depth_queue.tryGet.return_value = mock_depth

        node.timer_callback()

        np.testing.assert_array_equal(node.current_depth, depth)
        node.depth_publisher_.publish.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_get_distance_at_px_reads_cached_depth(self, mock_exists, mock_dai):
        node = self._make_node()
        node.current_depth = np.array([[0, 1200], [800, 1500]], dtype=np.uint16)
        node.pipeline = MagicMock()

        request = MagicMock()
        request.x = 1
        request.y = 0
        response = MagicMock()
        node.get_distance_at_px_callback(request, response)

        self.assertEqual(response.distance_mm, 1200.0)
        node.pipeline.start.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_get_depth_frame_encodes_cached_depth(self, mock_exists, mock_dai):
        node = self._make_node()
        node.current_depth = np.array([[42, 100], [200, 300]], dtype=np.uint16)
        node.pipeline = MagicMock()

        request = MagicMock()
        response = MagicMock()
        node.get_depth_frame_callback(request, response)

        self.assertEqual(response.width, 2)
        self.assertEqual(response.height, 2)
        self.assertEqual(response.encoding, "16UC1")
        decoded = np.frombuffer(
            base64.b64decode(response.depth_base64), dtype=np.uint16
        ).reshape((2, 2))
        np.testing.assert_array_equal(decoded, node.current_depth)
        node.pipeline.start.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_depth_services_return_existing_failure_values_when_unavailable(
        self, mock_exists, mock_dai
    ):
        node = self._make_node()
        node.depth_available = False
        node.current_depth = np.array([[1200]], dtype=np.uint16)

        depth_response = MagicMock()
        node.get_depth_frame_callback(MagicMock(), depth_response)
        self.assertEqual(depth_response.width, 0)
        self.assertEqual(depth_response.height, 0)
        self.assertEqual(depth_response.encoding, "")
        self.assertEqual(depth_response.depth_base64, "")

        distance_response = MagicMock()
        node.get_distance_at_px_callback(MagicMock(x=0, y=0), distance_response)
        self.assertEqual(distance_response.distance_mm, 0.0)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_init_pipeline_starts_once_with_stereo(self, mock_exists, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.get_logger = MagicMock()
        node.stereo_mode = "auto"
        node.stereo_timeout = 5.0
        node.current_depth = None
        node._pending_color_packet = None

        class _StereoType:
            class PresetMode:
                DEFAULT = "DEFAULT"

        pipeline = MagicMock()
        mock_dai.Pipeline.return_value = pipeline
        cam = MagicMock()
        stereo = MagicMock()
        mock_dai.node.Camera = object()
        mock_dai.node.StereoDepth = _StereoType
        mock_dai.CameraBoardSocket.CAM_A = "CAM_A"
        mock_dai.CameraBoardSocket.CAM_B = "CAM_B"
        mock_dai.CameraBoardSocket.CAM_C = "CAM_C"

        created = []

        def create(kind):
            created.append(kind)
            if kind is _StereoType:
                return stereo
            return cam

        pipeline.create.side_effect = create

        ok = CameraNode.init_pipeline(node)

        self.assertTrue(ok)
        pipeline.start.assert_called_once()
        self.assertIn(_StereoType, created)
        stereo.depth.createOutputQueue.assert_called()


class _MonoCameraType:
    pass


class _StereoDepthType:
    class PresetMode:
        DEFAULT = "DEFAULT"


class TestStereoDepthConfiguration(unittest.TestCase):
    """The mono/stereo setup must match the reference proven on depthai 3.6.1."""

    def _init_stereo(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.get_logger = MagicMock()
        node.pipeline = MagicMock()

        mono_left = MagicMock()
        mono_right = MagicMock()
        stereo = MagicMock()
        mock_dai.node.MonoCamera = _MonoCameraType
        mock_dai.node.StereoDepth = _StereoDepthType
        mock_dai.CameraBoardSocket.CAM_A = "CAM_A"
        mock_dai.CameraBoardSocket.CAM_B = "CAM_B"
        mock_dai.CameraBoardSocket.CAM_C = "CAM_C"
        mock_dai.MonoCameraProperties.SensorResolution.THE_480_P = "THE_480_P"
        mock_dai.MedianFilter.KERNEL_7x7 = "KERNEL_7x7"

        monos = iter((mono_left, mono_right))
        node.pipeline.create.side_effect = lambda kind: (
            stereo if kind is _StereoDepthType else next(monos)
        )

        node._init_stereo_depth()
        return node, mono_left, mono_right, stereo

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_mono_cameras_get_an_explicit_640x480_resolution(self, mock_dai):
        _, mono_left, mono_right, _ = self._init_stereo(mock_dai)

        mono_left.setBoardSocket.assert_called_once_with("CAM_B")
        mono_right.setBoardSocket.assert_called_once_with("CAM_C")
        for mono in (mono_left, mono_right):
            mono.setResolution.assert_called_once_with("THE_480_P")
            mono.requestFullResolutionOutput.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_depth_stays_native_instead_of_aligned_to_rgb(self, mock_dai):
        _, _, _, stereo = self._init_stereo(mock_dai)

        stereo.setDepthAlign.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_stereo_uses_lrc_extended_disparity_and_median_7x7(self, mock_dai):
        node, mono_left, mono_right, stereo = self._init_stereo(mock_dai)

        stereo.setLeftRightCheck.assert_called_once_with(True)
        stereo.setExtendedDisparity.assert_called_once_with(True)
        stereo.initialConfig.setMedianFilter.assert_called_once_with("KERNEL_7x7")
        mono_left.out.link.assert_called_once_with(stereo.left)
        mono_right.out.link.assert_called_once_with(stereo.right)
        self.assertIs(node.depth_queue, stereo.depth.createOutputQueue.return_value)
        node.get_logger().warning.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_unsupported_stereo_options_only_warn(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.get_logger = MagicMock()
        node.pipeline = MagicMock()
        mock_dai.node.MonoCamera = _MonoCameraType
        mock_dai.node.StereoDepth = _StereoDepthType

        stereo = MagicMock()
        stereo.setExtendedDisparity.side_effect = RuntimeError("not supported")
        mono = MagicMock()
        mono.setResolution.side_effect = RuntimeError("no such mode")
        monos = iter((mono, MagicMock()))
        node.pipeline.create.side_effect = lambda kind: (
            stereo if kind is _StereoDepthType else next(monos)
        )

        node._init_stereo_depth()

        self.assertEqual(node.get_logger().warning.call_count, 2)
        self.assertIsNotNone(node.depth_queue)


class TestHandPipelineInput(unittest.TestCase):
    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_colour_tap_queue_is_bounded_and_non_blocking(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        mock_dai.Pipeline.return_value = MagicMock()

        node._build_pipeline(include_stereo=False)

        node.isp_out.createOutputQueue.assert_called_once_with(
            maxSize=COLOR_OUTPUT_QUEUE_DEPTH, blocking=False
        )

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_rejected_camera_tap_fails_the_build(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.camRgb = MagicMock()
        node.camRgb.requestOutput.return_value = None

        with self.assertRaises(RuntimeError):
            node._request_camera_branch((HAND_NN_WIDTH, HAND_NN_HEIGHT))

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_landmark_crop_config_contains_rotated_warped_roi(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.hand_landmark_input_size = 224

        class _RotatedRect:
            def __init__(self):
                self.center = types.SimpleNamespace(x=0.0, y=0.0)
                self.size = types.SimpleNamespace(width=0.0, height=0.0)
                self.angle = 0.0

        config = MagicMock()
        mock_dai.RotatedRect.side_effect = _RotatedRect
        mock_dai.ImageManipConfig.return_value = config
        palm = MagicMock(rotation=np.pi / 4)
        palm.roi_for_frame.return_value = (0.5, 0.4, 0.25, 0.5)

        result = node._landmark_crop_config(palm, 640, 480, reuse_previous=True)

        self.assertIs(result, config)
        rotated, normalized = config.addCropRotatedRect.call_args.args
        self.assertTrue(normalized)
        self.assertEqual(rotated.center.x, 0.5)
        self.assertEqual(rotated.center.y, 0.4)
        self.assertEqual(rotated.size.width, 0.25)
        self.assertEqual(rotated.size.height, 0.5)
        self.assertAlmostEqual(rotated.angle, 45.0)
        config.setOutputSize.assert_called_once_with(224, 224)
        config.setFrameType.assert_called_once_with(mock_dai.ImgFrame.Type.BGR888p)
        config.setWarpBorderReplicatePixels.assert_called_once_with()
        config.setReusePreviousImage.assert_called_once_with(True)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_landmark_crop_config_rejects_invalid_geometry(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        palm = MagicMock(rotation=0.0)
        palm.roi_for_frame.return_value = (0.5, 0.5, 0.0, 0.25)

        with self.assertRaisesRegex(ValueError, "positive dimensions"):
            node._landmark_crop_config(palm, 640, 480)

        mock_dai.ImageManipConfig.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_landmark_crop_is_fitted_inside_actual_branch_dimensions(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.hand_landmark_input_size = 224

        class _RotatedRect:
            def __init__(self):
                self.center = types.SimpleNamespace(x=0.0, y=0.0)
                self.size = types.SimpleNamespace(width=0.0, height=0.0)
                self.angle = 0.0

        mock_dai.RotatedRect.side_effect = _RotatedRect
        palm = MagicMock(rotation=np.pi / 6)
        # This decoded ROI extends above and beyond the 640x480 hand branch.
        palm.roi_for_frame.return_value = (0.5, 0.2, 0.8, 1.05)

        node._landmark_crop_config(palm, 640, 480)

        rotated = (
            mock_dai.ImageManipConfig.return_value.addCropRotatedRect.call_args.args[0]
        )
        self.assertGreater(rotated.size.width, 0.0)
        self.assertGreater(rotated.size.height, 0.0)
        center_x = rotated.center.x * 640
        center_y = rotated.center.y * 480
        half_width = rotated.size.width * 640 / 2.0
        half_height = rotated.size.height * 480 / 2.0
        cos_rotation = np.cos(palm.rotation)
        sin_rotation = np.sin(palm.rotation)
        for x_offset in (-half_width, half_width):
            for y_offset in (-half_height, half_height):
                corner_x = center_x + x_offset * cos_rotation - y_offset * sin_rotation
                corner_y = center_y + x_offset * sin_rotation + y_offset * cos_rotation
                self.assertGreater(corner_x, 0.0)
                self.assertLess(corner_x, 640.0)
                self.assertGreater(corner_y, 0.0)
                self.assertLess(corner_y, 480.0)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_no_palm_sentinel_crop_stays_inside_the_source(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.hand_landmark_input_size = 224

        class _RotatedRect:
            def __init__(self):
                self.center = types.SimpleNamespace(x=0.0, y=0.0)
                self.size = types.SimpleNamespace(width=0.0, height=0.0)
                self.angle = 0.0

        mock_dai.RotatedRect.side_effect = _RotatedRect

        node._landmark_crop_config(None, 640, 480)

        rotated = (
            mock_dai.ImageManipConfig.return_value.addCropRotatedRect.call_args.args[0]
        )
        # An exactly full-frame rect sits on the boundary the device validates.
        self.assertLess(rotated.size.width, 1.0)
        self.assertLess(rotated.size.height, 1.0)
        half_width = rotated.size.width * 640 / 2.0
        half_height = rotated.size.height * 480 / 2.0
        self.assertGreater(rotated.center.x * 640 - half_width, 0.0)
        self.assertLess(rotated.center.x * 640 + half_width, 640.0)
        self.assertGreater(rotated.center.y * 480 - half_height, 0.0)
        self.assertLess(rotated.center.y * 480 + half_height, 480.0)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_branch_size_change_is_adopted_for_later_crops(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.hand_source_size = (640, 480)
        node._hand_warnings = set()
        node.get_logger = MagicMock()
        transformation = MagicMock()
        transformation.getSourceSize.return_value = (1280, 720)
        packet = MagicMock()
        packet.getTransformation.return_value = transformation

        node._note_hand_branch_size(packet)

        self.assertEqual(node.hand_source_size, (1280, 720))

        # A device that reports no usable size must not move the crop geometry.
        transformation.getSourceSize.return_value = None
        node._note_hand_branch_size(packet)

        self.assertEqual(node.hand_source_size, (1280, 720))

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_initial_crops_follow_the_delivered_branch_size(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()

        class _RotatedRect:
            def __init__(self):
                self.center = types.SimpleNamespace(x=0.0, y=0.0)
                self.size = types.SimpleNamespace(width=0.0, height=0.0)
                self.angle = 0.0

        mock_dai.RotatedRect.side_effect = _RotatedRect
        artifacts = _hand_artifacts()
        node.model_registry = MagicMock()
        node.model_registry.get.side_effect = artifacts.get
        node.pipeline = MagicMock()
        created = [MagicMock() for _ in range(5)]
        node.pipeline.create.side_effect = created
        node.camRgb = MagicMock()
        hand_tap = MagicMock()
        # The camera answers the 256x256 request with a larger stream.
        hand_tap.getSize.return_value = (1280, 720)
        node.camRgb.requestOutput.return_value = hand_tap

        node._build_hand_pipeline(types.SimpleNamespace(artifact_ids=tuple(artifacts)))

        self.assertEqual(node.hand_source_size, (1280, 720))
        palm_manip = created[0]
        landmark_manip = created[3]
        palm_manip.initialConfig.setOutputSize.assert_called_once_with(128, 128)
        landmark_manip.initialConfig.setOutputSize.assert_called_once_with(224, 224)

        for manip in (palm_manip, landmark_manip):
            rotated, normalized = manip.initialConfig.addCropRotatedRect.call_args.args
            self.assertTrue(normalized)
            # The build-time rect never rotates; only the landmark model needs
            # its crop aligned to the palm, and that comes per frame.
            self.assertEqual(rotated.angle, 0.0)
            half_width = rotated.size.width * 1280 / 2.0
            half_height = rotated.size.height * 720 / 2.0
            self.assertAlmostEqual(rotated.center.x * 1280 - half_width, 0.5)
            self.assertAlmostEqual(rotated.center.x * 1280 + half_width, 1279.5)
            self.assertAlmostEqual(rotated.center.y * 720 - half_height, 0.5)
            self.assertAlmostEqual(rotated.center.y * 720 + half_height, 719.5)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_hand_branch_bounds_the_single_palm_manip_downscale(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()

        class _RotatedRect:
            def __init__(self):
                self.center = types.SimpleNamespace(x=0.0, y=0.0)
                self.size = types.SimpleNamespace(width=0.0, height=0.0)
                self.angle = 0.0

        mock_dai.RotatedRect.side_effect = _RotatedRect
        artifacts = _hand_artifacts()
        node.model_registry = MagicMock()
        node.model_registry.get.side_effect = artifacts.get
        node.pipeline = MagicMock()
        created = [MagicMock() for _ in range(5)]
        node.pipeline.create.side_effect = created
        node.camRgb = MagicMock()
        node.camRgb.requestOutput.return_value = MagicMock()

        node._build_hand_pipeline(types.SimpleNamespace(artifact_ids=tuple(artifacts)))

        self.assertEqual((HAND_NN_WIDTH, HAND_NN_HEIGHT), (256, 256))
        palm_manip, landmark_manip = created[0], created[3]
        palm_manip.setMaxOutputFrameSize.assert_called_once_with(128 * 128 * 3)
        landmark_manip.setMaxOutputFrameSize.assert_called_once_with(224 * 224 * 3)
        rotated = palm_manip.initialConfig.addCropRotatedRect.call_args.args[0]
        self.assertAlmostEqual(
            rotated.size.width * HAND_NN_WIDTH,
            255.0,
        )

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_hand_manips_share_one_non_blocking_camera_tap(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()

        artifacts = _hand_artifacts()
        node.model_registry = MagicMock()
        node.model_registry.get.side_effect = artifacts.get
        node.pipeline = MagicMock()
        created = [MagicMock() for _ in range(5)]
        node.pipeline.create.side_effect = created
        node.camRgb = MagicMock()
        hand_tap = MagicMock()
        node.camRgb.requestOutput.return_value = hand_tap

        node._build_hand_pipeline(types.SimpleNamespace(artifact_ids=tuple(artifacts)))

        palm_manip, palm_nn, decoder_nn, landmark_manip, landmark_nn = created

        # A single downscaled output stays within the camera-output budget. Both
        # consumers are non-blocking, so the config-gated landmark branch cannot
        # back-pressure the palm branch while it is idle.
        expected_request = unittest.mock.call(
            (HAND_NN_WIDTH, HAND_NN_HEIGHT),
            type=mock_dai.ImgFrame.Type.BGR888p,
        )
        self.assertEqual(node.camRgb.requestOutput.call_args_list, [expected_request])
        hand_tap.link.assert_has_calls(
            [
                unittest.mock.call(palm_manip.inputImage),
                unittest.mock.call(landmark_manip.inputImage),
            ]
        )
        palm_manip.out.link.assert_called_once_with(palm_nn.input)
        branch_inputs = [
            palm_manip.inputImage,
            palm_nn.input,
            landmark_manip.inputImage,
        ]
        for branch_input in branch_inputs:
            branch_input.setBlocking.assert_called_once_with(False)
            branch_input.setMaxSize.assert_called_once_with(BRANCH_INPUT_QUEUE_DEPTH)
        palm_nn.setNumShavesPerInferenceThread.assert_called_once_with(4)
        decoder_nn.setNumShavesPerInferenceThread.assert_called_once_with(1)
        landmark_nn.setNumShavesPerInferenceThread.assert_called_once_with(4)
        palm_manip.setMaxOutputFrameSize.assert_called_once_with(128 * 128 * 3)
        landmark_manip.setMaxOutputFrameSize.assert_called_once_with(224 * 224 * 3)
        palm_nn.out.link.assert_called_once_with(decoder_nn.input)
        landmark_manip.inputConfig.setWaitForMessage.assert_called_once_with(True)
        for branch_node in (palm_nn, decoder_nn, landmark_manip):
            branch_node.out.createOutputQueue.assert_called_once_with(
                maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
            )
        # Landmark results stay blocking so _pending_hands keeps its pairing.
        landmark_nn.out.createOutputQueue.assert_called_once_with()
        self.assertEqual(node.hand_source_size, (256, 256))

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_landmarks_use_packet_transformation_to_map_to_preview(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.hand_landmark_input_size = 224
        mock_dai.Point2f.side_effect = lambda x, y: types.SimpleNamespace(x=x, y=y)
        transformation = MagicMock()
        transformation.invTransformPoint.side_effect = (
            lambda point: types.SimpleNamespace(x=point.x + 10, y=point.y + 20)
        )
        packet = MagicMock()
        packet.getTransformation.return_value = transformation
        tensor = np.tile([0.5, 0.25, 0.0], (21, 1))

        points = node._map_hand_landmarks(
            packet, tensor, MagicMock(), 1280, 720, 640, 480
        )

        self.assertEqual(len(points), 21)
        self.assertEqual(points[0], (244.0, 114.0))
        transformation.invTransformPoint.assert_called()


class TestHandStageCounters(unittest.TestCase):
    def _make_node(self):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node._reset_hand_stage_counters()
        node._reset_imitation_stage_counters()
        node.imitation_queue = None
        node.get_logger = MagicMock()
        return node

    def _set_built_hand_chain(self, node):
        node._pipeline_models = [
            types.SimpleNamespace(model=types.SimpleNamespace(model_id="hand_tracking"))
        ]
        node.hand_palm_queue = MagicMock()
        node.hand_decoder_queue = MagicMock()
        node.hand_roi_queue = MagicMock()
        node.hand_landmark_queue = MagicMock()
        node.hand_landmark_config_queue = MagicMock()

    def _set_status_publishing_dependencies(self, node, initial_status, final_status):
        node.pipeline_manager = MagicMock()
        node.pipeline_manager.statuses.side_effect = [
            {"hand_tracking": initial_status},
            {"hand_tracking": final_status},
        ]
        node.model_registry = MagicMock()
        node.model_registry.models.return_value = []
        node.models_status_publisher_ = MagicMock()
        node.get_clock = MagicMock()
        node._log_hand_stage_counters = MagicMock()
        node._log_imitation_stage_counters = MagicMock()

    def test_status_failure_check_passes_configured_startup_grace(self):
        node = self._make_node()
        self._set_built_hand_chain(node)
        node.hand_startup_grace = 7.5
        running = {
            "active": True,
            "state": "running",
            "owners": {"ui"},
        }
        failed = {
            "active": False,
            "state": "failed",
            "owners": {"ui"},
        }
        self._set_status_publishing_dependencies(node, running, failed)
        node.pipeline_manager.mark_failed.return_value = False

        node.publish_model_statuses()

        node.pipeline_manager.mark_failed.assert_called_once_with(
            "hand_tracking",
            "Hand pipeline is not producing startup stage packets",
            startup_grace=7.5,
        )
        node.get_logger().error.assert_not_called()

    def test_stage_counters_recover_failed_hand_status(self):
        node = self._make_node()
        self._set_built_hand_chain(node)
        for stage in HAND_STAGE_NAMES:
            node._count_hand_stage(stage)
        failed = {
            "active": False,
            "state": "failed",
            "owners": {"ui"},
        }
        running = {
            "active": True,
            "state": "running",
            "owners": {"ui"},
        }
        self._set_status_publishing_dependencies(node, failed, running)
        node.pipeline_manager.mark_running.return_value = True

        node.publish_model_statuses()

        node.pipeline_manager.mark_running.assert_called_once_with("hand_tracking")
        node.pipeline_manager.mark_failed.assert_not_called()
        node.get_logger().info.assert_called_once_with(
            "hand_tracking recovered after downstream packet flow resumed."
        )

    def test_hand_startup_grace_is_configurable(self):
        node = self._make_node()
        with patch.dict(os.environ, {"PIB_HAND_STARTUP_GRACE": "7.5"}):
            self.assertEqual(node._read_hand_startup_grace(), 7.5)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_decoder_empty_result_sends_config_and_discards_sentinel_landmarks(
        self, mock_dai
    ):
        node = self._make_node()
        decoder_packet = MagicMock()
        decoder_packet.getTensor.return_value = [0.0] * 80
        landmark_packet = MagicMock()
        node.hand_decoder_queue = MagicMock()
        node.hand_decoder_queue.tryGet.side_effect = [decoder_packet, None]
        node.hand_landmark_queue = MagicMock()
        node.hand_landmark_queue.tryGet.return_value = landmark_packet
        node.hand_landmark_config_queue = MagicMock()
        node.hand_landmark_input_size = 224
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node.current_source_size = (1280, 720)
        node.hand_source_size = (256, 256)
        node._pending_hand_decoder_packet = None
        node._pending_hands = deque()
        node._publish_hand_detections = MagicMock(
            side_effect=lambda *_: node._count_hand_stage("publish")
        )

        node._process_hand_tracking()

        self.assertEqual(node.hand_stage_counters["decoding_nn"], 1)
        self.assertEqual(node.hand_stage_counters["decoding_result"], 1)
        self.assertEqual(node.hand_stage_counters["image_manip_config"], 1)
        self.assertEqual(node.hand_stage_counters["post_processing"], 0)
        node.hand_landmark_config_queue.send.assert_called_once_with(
            mock_dai.ImageManipConfig.return_value
        )
        rotated = (
            mock_dai.ImageManipConfig.return_value.addCropRotatedRect.call_args.args[0]
        )
        self.assertEqual(
            (rotated.center.x, rotated.center.y, rotated.angle), (0.5, 0.5, 0.0)
        )
        # The sentinel covers the complete bounded camera branch, with each edge
        # inset by half a source pixel for ImageManip's crop validation.
        crop_width = rotated.size.width * 256
        crop_height = rotated.size.height * 256
        self.assertAlmostEqual(crop_width, 255.0)
        self.assertAlmostEqual(crop_height, 255.0)
        self.assertAlmostEqual(rotated.center.x * 256 - crop_width / 2.0, 0.5)
        self.assertAlmostEqual(rotated.center.x * 256 + crop_width / 2.0, 255.5)
        self.assertAlmostEqual(rotated.center.y * 256 - crop_height / 2.0, 0.5)
        self.assertAlmostEqual(rotated.center.y * 256 + crop_height / 2.0, 255.5)
        mock_dai.ImageManipConfig.return_value.setOutputSize.assert_called_once_with(
            224, 224
        )

        node._process_hand_tracking()

        self.assertEqual(node.hand_stage_counters["hand_landmark_nn"], 1)
        self.assertEqual(node.hand_stage_counters["post_processing"], 1)
        self.assertEqual(node.hand_stage_counters["publish"], 1)
        node._publish_hand_detections.assert_called_once_with(1280, 720, [])
        landmark_packet.getTensor.assert_not_called()

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_landmark_result_uses_image_tensor_and_packet_transform(self, mock_dai):
        node = self._make_node()
        palm = MagicMock(score=0.9)
        batch = {
            "remaining": 1,
            "detections": [],
            "frame_width": 1280,
            "frame_height": 720,
            "source_width": 640,
            "source_height": 480,
        }
        landmark_packet = MagicMock()
        landmark_packet.getTensor.side_effect = lambda name: {
            "Identity_1": [0.9],
            "Identity_dense/BiasAdd/Add": np.tile([0.5, 0.25, 0.0], (21, 1)),
        }[name]
        transformation = MagicMock()
        transformation.invTransformPoint.return_value = types.SimpleNamespace(
            x=320.0, y=240.0
        )
        landmark_packet.getTransformation.return_value = transformation
        mock_dai.Point2f.side_effect = lambda x, y: types.SimpleNamespace(x=x, y=y)

        node.hand_decoder_queue = MagicMock()
        node.hand_decoder_queue.tryGet.return_value = None
        node.hand_landmark_queue = MagicMock()
        node.hand_landmark_queue.tryGet.return_value = landmark_packet
        node.hand_landmark_input_size = 224
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node.hand_source_size = (640, 480)
        node._pending_hand_decoder_packet = None
        node._pending_hands = deque([(palm, batch)])
        node._hand_detection_message = MagicMock(return_value="detection")
        node._publish_hand_detections = MagicMock()

        node._process_hand_tracking()

        self.assertEqual(
            landmark_packet.getTensor.call_args_list,
            [
                unittest.mock.call("Identity_1"),
                unittest.mock.call("Identity_dense/BiasAdd/Add"),
            ],
        )
        landmark_packet.getTransformation.assert_called_once_with()
        node._publish_hand_detections.assert_called_once_with(1280, 720, ["detection"])

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_landmark_layer_falls_back_when_image_head_is_missing(self, mock_dai):
        node = self._make_node()
        palm = MagicMock(score=0.9)
        batch = {
            "remaining": 1,
            "candidates": 1,
            "detections": [],
            "drop_reasons": [],
            "frame_width": 1280,
            "frame_height": 720,
            "source_width": 640,
            "source_height": 480,
        }
        landmark_packet = MagicMock()
        landmark_packet.getTensor.side_effect = lambda name: {
            "Identity_1": [0.9],
            "Identity_3_dense/BiasAdd/Add": np.tile([0.5, 0.25, 0.0], (21, 1)),
        }[name]
        transformation = MagicMock()
        transformation.invTransformPoint.return_value = types.SimpleNamespace(
            x=320.0, y=240.0
        )
        landmark_packet.getTransformation.return_value = transformation
        mock_dai.Point2f.side_effect = lambda x, y: types.SimpleNamespace(x=x, y=y)
        node.hand_decoder_queue = MagicMock()
        node.hand_decoder_queue.tryGet.return_value = None
        node.hand_landmark_queue = MagicMock()
        node.hand_landmark_queue.tryGet.return_value = landmark_packet
        node.hand_landmark_input_size = 224
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node.hand_source_size = (640, 480)
        node._pending_hand_decoder_packet = None
        node._pending_hands = deque([(palm, batch)])
        node._hand_detection_message = MagicMock(return_value="detection")
        node._publish_hand_detections = MagicMock()

        node._process_hand_tracking()

        self.assertEqual(
            [entry.args[0] for entry in landmark_packet.getTensor.call_args_list],
            [
                "Identity_1",
                "Identity_dense/BiasAdd/Add",
                "Identity",
                "Identity_3_dense/BiasAdd/Add",
            ],
        )
        self.assertEqual(batch["landmark_layer"], "Identity_3_dense/BiasAdd/Add")
        self.assertEqual(batch["drop_reasons"], [])
        node._publish_hand_detections.assert_called_once_with(1280, 720, ["detection"])

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_empty_landmark_head_is_reported_as_the_skipped_check(self, mock_dai):
        node = self._make_node()
        palm = MagicMock(score=0.9)
        batch = {
            "remaining": 1,
            "candidates": 1,
            "detections": [],
            "drop_reasons": [],
            "frame_width": 1280,
            "frame_height": 720,
            "source_width": 640,
            "source_height": 480,
            "log_details": True,
        }
        landmark_packet = MagicMock()
        landmark_packet.getTensor.side_effect = lambda name: {
            "Identity_1": [0.9],
        }.get(name, np.zeros(0, dtype=np.float32))
        landmark_packet.getTransformation.return_value = None
        node.hand_decoder_queue = MagicMock()
        node.hand_decoder_queue.tryGet.return_value = None
        node.hand_landmark_queue = MagicMock()
        node.hand_landmark_queue.tryGet.return_value = landmark_packet
        node.hand_landmark_input_size = 224
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node.hand_source_size = (640, 480)
        node._pending_hand_decoder_packet = None
        node._pending_hands = deque([(palm, batch)])
        node._publish_hand_detections = MagicMock()
        node._hand_warnings = set()

        node._process_hand_tracking()

        self.assertEqual(len(batch["drop_reasons"]), 1)
        self.assertIn("no landmark layer carries 63 values", batch["drop_reasons"][0])
        self.assertIn(
            "Identity_dense/BiasAdd/Add=rejected(size=0)", batch["drop_reasons"][0]
        )
        logged = [call.args[0] for call in node.get_logger().info.call_args_list]
        self.assertTrue(any(line.startswith("HAND_FP KP ") for line in logged))
        self.assertTrue(
            any(
                line.startswith("HAND_FP ASSEMBLY ") and "appended=0" in line
                for line in logged
            )
        )
        node._publish_hand_detections.assert_called_once_with(1280, 720, [])

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_landmark_result_assembles_detection_from_batched_tensors(self, mock_dai):
        node = self._make_node()
        palm = PalmRegion(0.88, 0.5, 0.5, 0.2, 0.5, 0.5, 0.5, 0.0)
        batch = {
            "remaining": 1,
            "detections": [],
            "frame_width": 1280,
            "frame_height": 720,
            "source_width": 256,
            "source_height": 256,
        }
        landmark_packet = MagicMock()
        landmark_packet.getTensor.side_effect = lambda name: {
            "Identity_1": np.array([[0.93]], dtype=np.float32),
            "Identity_dense/BiasAdd/Add": np.tile([0.5, 0.5, 0.0], (1, 21, 1)).reshape(
                1, 63
            ),
        }[name]
        landmark_packet.getTransformation.return_value = None
        node.hand_decoder_queue = MagicMock()
        node.hand_decoder_queue.tryGet.return_value = None
        node.hand_landmark_queue = MagicMock()
        node.hand_landmark_queue.tryGet.return_value = landmark_packet
        node.hand_landmark_input_size = 224
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node.hand_source_size = (256, 256)
        node._pending_hand_decoder_packet = None
        node._pending_hands = deque([(palm, batch)])
        node._publish_hand_detections = MagicMock()

        node._process_hand_tracking()

        node._publish_hand_detections.assert_called_once()
        detections = node._publish_hand_detections.call_args.args[2]
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].label, "hand")
        self.assertAlmostEqual(detections[0].score, 0.88)
        self.assertEqual(len(detections[0].keypoint_x), 21)
        self.assertEqual(len(detections[0].keypoint_y), 21)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_unactivated_landmark_score_still_appends_detection(self, mock_dai):
        node = self._make_node()
        palm = PalmRegion(0.88, 0.5, 0.5, 0.2, 0.5, 0.5, 0.5, 0.0)
        batch = {
            "remaining": 1,
            "candidates": 1,
            "keypoints_built": 0,
            "detections": [],
            "drop_reasons": [],
            "frame_width": 1280,
            "frame_height": 720,
            "source_width": 256,
            "source_height": 256,
            "log_details": True,
        }
        landmark_packet = MagicMock()
        landmark_packet.getTensor.side_effect = lambda name: {
            # Identity_1 as the blob reports it for a hand filling the crop.
            "Identity_1": np.array([[0.01823425]], dtype=np.float32),
            "Identity_dense/BiasAdd/Add": np.tile([0.5, 0.5, 0.0], (1, 21, 1)).reshape(
                1, 63
            ),
        }[name]
        landmark_packet.getTransformation.return_value = None
        node.hand_decoder_queue = MagicMock()
        node.hand_decoder_queue.tryGet.return_value = None
        node.hand_landmark_queue = MagicMock()
        node.hand_landmark_queue.tryGet.return_value = landmark_packet
        node.hand_landmark_input_size = 224
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node.hand_source_size = (256, 256)
        node._pending_hand_decoder_packet = None
        node._pending_hands = deque([(palm, batch)])
        node._publish_hand_detections = MagicMock()

        node._process_hand_tracking()

        self.assertEqual(batch["drop_reasons"], [])
        self.assertEqual(batch["keypoints_built"], 21)
        detections = node._publish_hand_detections.call_args.args[2]
        self.assertEqual(len(detections), 1)
        self.assertEqual(len(detections[0].keypoint_x), 21)
        logged = [call.args[0] for call in node.get_logger().info.call_args_list]
        self.assertTrue(
            any(
                line.startswith("HAND_FP ASSEMBLY ")
                and "appended=1" in line
                and "keypoints_built=21" in line
                for line in logged
            )
        )

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_unreadable_landmark_score_still_appends_detection(self, mock_dai):
        node = self._make_node()
        palm = PalmRegion(0.88, 0.5, 0.5, 0.2, 0.5, 0.5, 0.5, 0.0)
        batch = {
            "remaining": 1,
            "candidates": 1,
            "detections": [],
            "drop_reasons": [],
            "frame_width": 1280,
            "frame_height": 720,
            "source_width": 256,
            "source_height": 256,
        }

        def tensor(name):
            if name == "Identity_1":
                raise RuntimeError("no such layer")
            if name == "Identity_dense/BiasAdd/Add":
                return np.tile([0.5, 0.5, 0.0], (1, 21, 1)).reshape(1, 63)
            raise RuntimeError("no such layer")

        landmark_packet = MagicMock()
        landmark_packet.getTensor.side_effect = tensor
        landmark_packet.getTransformation.return_value = None
        node.hand_decoder_queue = MagicMock()
        node.hand_decoder_queue.tryGet.return_value = None
        node.hand_landmark_queue = MagicMock()
        node.hand_landmark_queue.tryGet.return_value = landmark_packet
        node.hand_landmark_input_size = 224
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node.hand_source_size = (256, 256)
        node._pending_hand_decoder_packet = None
        node._pending_hands = deque([(palm, batch)])
        node._publish_hand_detections = MagicMock()

        node._process_hand_tracking()

        self.assertEqual(batch["drop_reasons"], [])
        detections = node._publish_hand_detections.call_args.args[2]
        self.assertEqual(len(detections), 1)

    def test_counter_log_contains_all_raw_stages_and_last_flowing_stage(self):
        node = self._make_node()
        node._pipeline_models = [
            types.SimpleNamespace(model=types.SimpleNamespace(model_id="hand_tracking"))
        ]
        node._count_hand_stage("colour_isp", 3)
        node._count_hand_stage("palm_detector_nn", 2)

        node._log_hand_stage_counters()

        message = node.get_logger().info.call_args.args[0]
        for stage in HAND_STAGE_NAMES:
            self.assertIn(f"{stage}={node.hand_stage_counters[stage]}", message)
        self.assertIn("last_flowing=palm_detector_nn", message)

    def test_model_verification_requires_and_preserves_decoder_packet(self):
        node = self._make_node()
        self._set_built_hand_chain(node)
        colour_packet = object()
        palm_packet = object()
        decoder_packet = object()
        node._pending_color_packet = None
        node._pending_hand_decoder_packet = None
        node._wait_for_color_frame = MagicMock(return_value=colour_packet)
        node._wait_for_queue_packet = MagicMock(
            side_effect=[palm_packet, decoder_packet]
        )

        self.assertTrue(node._verify_model_frames(3.0))

        self.assertIs(node._pending_color_packet, colour_packet)
        self.assertIs(node._pending_hand_decoder_packet, decoder_packet)
        self.assertEqual(
            node._wait_for_queue_packet.call_args_list,
            [
                unittest.mock.call(node.hand_palm_queue, 3.0),
                unittest.mock.call(node.hand_decoder_queue, 3.0),
            ],
        )
        self.assertFalse(node._hand_chain_is_flowing())
        for stage in (
            "decoding_result",
            "image_manip_config",
            "image_manip_roi",
            "hand_landmark_nn",
            "post_processing",
            "publish",
        ):
            node._count_hand_stage(stage)
        self.assertTrue(node._hand_chain_is_flowing())

    def test_model_verification_fails_when_decoder_has_no_packets(self):
        node = self._make_node()
        self._set_built_hand_chain(node)
        node._pending_color_packet = None
        node._pending_hand_decoder_packet = None
        node._wait_for_color_frame = MagicMock(return_value=object())
        node._wait_for_queue_packet = MagicMock(side_effect=[object(), None])

        self.assertFalse(node._verify_model_frames(3.0))

    def test_model_verification_fails_when_requested_hand_chain_is_absent(self):
        node = self._make_node()
        node._pipeline_models = [
            types.SimpleNamespace(model=types.SimpleNamespace(model_id="hand_tracking"))
        ]
        node.hand_palm_queue = None
        node.hand_decoder_queue = None
        node.hand_roi_queue = None
        node.hand_landmark_queue = None
        node.hand_landmark_config_queue = None
        node.nn_queues = {}
        node._wait_for_color_frame = MagicMock(return_value=object())

        self.assertFalse(node._verify_model_frames(3.0))

        node._wait_for_color_frame.assert_not_called()
        node.get_logger().error.assert_called_once()

    def test_rebuild_rejects_colour_pipeline_missing_requested_hand_chain(self):
        node = self._make_node()
        node._stop_pipeline = MagicMock()
        node.init_pipeline = MagicMock(return_value=True)
        node.nn_queues = {}
        node.hand_palm_queue = None
        node.hand_decoder_queue = None
        node.hand_roi_queue = None
        node.hand_landmark_queue = None
        node.hand_landmark_config_queue = None
        requested = [
            types.SimpleNamespace(model=types.SimpleNamespace(model_id="hand_tracking"))
        ]

        self.assertFalse(node._rebuild_models(requested))

        self.assertFalse(node.camera_available)
        self.assertEqual(node._stop_pipeline.call_count, 2)
        node.get_logger().error.assert_called_once()


class TestImitationPipeline(unittest.TestCase):
    def _make_node(self):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.get_logger = MagicMock()
        node._hand_warnings = set()
        node._reset_imitation_stage_counters()
        return node

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_graph_matches_official_parsed_two_stage_wiring(self, mock_dai):
        node = self._make_node()
        node.pipeline = MagicMock()
        created = [MagicMock() for _ in range(6)]
        node.pipeline.create.side_effect = created
        node.isp_out = MagicMock()
        # The neural branches must ask the camera for their own sized, rate
        # limited output. Tapping isp_out (which feeds /camera_topic) froze the
        # whole pipeline on the robot and blanked the camera view in Cerebra.
        node.camRgb = MagicMock()
        imitation_source = MagicMock()
        node.camRgb.requestOutput.return_value = imitation_source
        detection_archive = MagicMock()
        detection_archive.getInputWidth.return_value = 192
        detection_archive.getInputHeight.return_value = 192
        landmark_archive = MagicMock()
        landmark_archive.getInputWidth.return_value = 224
        landmark_archive.getInputHeight.return_value = 224
        mock_dai.NNArchive.side_effect = [detection_archive, landmark_archive]
        descriptions = [types.SimpleNamespace(platform=None) for _ in range(2)]
        mock_dai.NNModelDescription.side_effect = descriptions

        detector_resize, detection_nn, processor, cropper, pose_nn, gather = created
        detection_nn.build.return_value = detection_nn
        processor.build.return_value = processor
        cropper.fromManipConfigs.return_value = cropper
        cropper.build.return_value = cropper
        pose_nn.build.return_value = pose_nn
        gather.build.return_value = gather

        node._build_imitation_pipeline(types.SimpleNamespace())

        self.assertEqual(
            [call.args[0] for call in mock_dai.NNModelDescription.call_args_list],
            [IMITATION_DETECTOR_MODEL, IMITATION_LANDMARK_MODEL],
        )
        self.assertEqual(
            [description.platform for description in descriptions], ["RVC2"] * 2
        )
        detector_resize.initialConfig.setOutputSize.assert_called_once_with(
            192,
            192,
            mode=mock_dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        node.isp_out.link.assert_not_called()
        node.camRgb.requestOutput.assert_called_once_with(
            (IMITATION_SOURCE_WIDTH, IMITATION_SOURCE_HEIGHT),
            type=mock_dai.ImgFrame.Type.BGR888p,
            fps=IMITATION_FPS,
        )
        imitation_source.link.assert_called_once_with(detector_resize.inputImage)
        detection_nn.build.assert_called_once_with(
            detector_resize.out,
            detection_archive,
        )
        processor.build.assert_called_once_with(
            detections_input=detection_nn.out,
            padding=0.1,
            target_size=(224, 224),
        )
        cropper.fromManipConfigs.assert_called_once_with(
            inputManipConfigs=processor.config_output,
            maxOutputFrameSize=224 * 224 * 3,
            waitForConfig=True,
        )
        # Stage 2 crops from the same camera branch, not from the ISP stream.
        cropper.build.assert_called_once_with(imitation_source)
        pose_nn.build.assert_called_once_with(cropper.out, landmark_archive)
        gather.build.assert_called_once_with(
            cameraFps=IMITATION_FPS,
            inputData=pose_nn.outputs,
            inputReference=detection_nn.out,
        )
        gather.out.createOutputQueue.assert_called_once_with(
            maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_imitation_caps_camera_and_complete_graph_at_eight_fps(self, mock_dai):
        node = self._make_node()
        node._pipeline_models = [
            types.SimpleNamespace(model=types.SimpleNamespace(model_id="imitation"))
        ]
        node._build_imitation_pipeline = MagicMock()
        pipeline = MagicMock()
        camera = MagicMock()
        mock_dai.Pipeline.return_value = pipeline
        pipeline.create.return_value = camera

        node._build_pipeline(include_stereo=False)

        camera.build.assert_called_once_with(
            mock_dai.CameraBoardSocket.CAM_A,
            sensorFps=IMITATION_FPS,
        )
        node._build_imitation_pipeline.assert_called_once()

    def test_lifecycle_verification_preserves_first_gathered_packet(self):
        node = self._make_node()
        node._pipeline_models = [
            types.SimpleNamespace(model=types.SimpleNamespace(model_id="imitation"))
        ]
        node.imitation_queue = MagicMock()
        node.nn_queues = {}
        node._pending_color_packet = None
        node._pending_imitation_packet = None
        colour_packet = object()
        result_packet = object()
        node._wait_for_color_frame = MagicMock(return_value=colour_packet)
        node._wait_for_queue_packet = MagicMock(return_value=result_packet)

        self.assertTrue(node._verify_model_frames(3.0))

        self.assertIs(node._pending_color_packet, colour_packet)
        self.assertIs(node._pending_imitation_packet, result_packet)
        node._wait_for_queue_packet.assert_called_once_with(node.imitation_queue, 3.0)

    def test_gathered_hand_publishes_21_points_world_layout_and_zero_depth(self):
        node = self._make_node()
        hand = {
            "palm_score": 0.87,
            "landmark_score": 0.91,
            "handedness": 0.75,
            "landmarks": [(640.0, 360.0)] * 21,
            "world": [float(index) for index in range(63)],
        }

        detection = node._imitation_detection(hand, 1280, 720)

        self.assertAlmostEqual(detection.score, 0.91)
        self.assertEqual(len(detection.keypoint_x), 21)
        self.assertEqual(list(detection.keypoint_z), [0.0] * 21)
        self.assertGreater(detection.x_max, detection.x_min)
        self.assertGreater(detection.y_max, detection.y_min)
        self.assertEqual(len(detection.scalar_names), 66)
        self.assertEqual(detection.scalar_names[-1], "world_20_z")

    def test_empty_gathered_frame_publishes_empty_detection_array(self):
        node = self._make_node()
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node._publish_imitation_detections = MagicMock()
        packet = types.SimpleNamespace(
            reference_data=types.SimpleNamespace(detections=[]), items=[]
        )

        node._consume_imitation_packet(packet)

        node._publish_imitation_detections.assert_called_once_with(1280, 720, [])
        self.assertEqual(node.imitation_stage_counters["palm_detector_nn"], 1)
        self.assertEqual(node.imitation_stage_counters["decoding_result"], 1)
        self.assertEqual(node.imitation_stage_counters["image_manip_config"], 0)

    @patch("ros_packages.camera.oak_d_lite.stereo.gathered_result_trace_values")
    @patch("ros_packages.camera.oak_d_lite.stereo.gathered_hands")
    def test_result_updates_counters_and_emits_trace(self, mock_hands, mock_traces):
        node = self._make_node()
        node.current_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        node._publish_imitation_detections = MagicMock()
        packet = types.SimpleNamespace(
            reference_data=types.SimpleNamespace(detections=[object()]),
            items=[object()],
        )
        mock_hands.return_value = []
        mock_traces.return_value = [(0.905, 0.997, (0.1, 0.2, 0.3, 0.4))]

        node._consume_imitation_packet(packet)

        for stage in (
            "palm_detector_nn",
            "decoding_nn",
            "decoding_result",
            "image_manip_config",
            "image_manip_roi",
            "hand_landmark_nn",
            "post_processing",
        ):
            self.assertEqual(node.imitation_stage_counters[stage], 1)
        self.assertIn(
            "IMIT_TRACE palm_score=0.905 landmark_score=0.997 "
            "crop=(0.1,0.2,0.3,0.4)",
            node.get_logger().info.call_args.args[0],
        )

    def test_flow_and_counter_log_use_gathered_stages(self):
        node = self._make_node()
        node._pipeline_models = [
            types.SimpleNamespace(model=types.SimpleNamespace(model_id="imitation"))
        ]
        node.imitation_queue = MagicMock()
        node.imitation_source_size = (192, 192)
        for stage in ("palm_detector_nn", "decoding_nn", "decoding_result", "publish"):
            node._count_imitation_stage(stage)

        self.assertTrue(node._imitation_chain_is_flowing())
        node._log_imitation_stage_counters()

        message = node.get_logger().info.call_args.args[0]
        for stage in IMITATION_STAGE_NAMES:
            self.assertIn(f"{stage}={node.imitation_stage_counters[stage]}", message)


class TestStereoModeDecision(unittest.TestCase):

    def _make_node(self, mode):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.get_logger = MagicMock()
        node.stereo_mode = mode
        node.stereo_timeout = 5.0
        node.current_depth = None
        node.pipeline = MagicMock()
        node.queue = MagicMock()
        node.depth_queue = None
        node._pending_color_packet = None
        return node

    def test_stereo_mode_defaults_to_auto(self):
        node = self._make_node("unused")

        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(node._read_stereo_mode(), "auto")

    def test_invalid_stereo_mode_falls_back_to_off(self):
        node = self._make_node("unused")

        with patch.dict(os.environ, {"PIB_CAMERA_STEREO": "invalid"}, clear=True):
            self.assertEqual(node._read_stereo_mode(), "off")

        node.get_logger().warning.assert_called_once_with(
            "Invalid PIB_CAMERA_STEREO='invalid'; using 'off'."
        )

    def test_mode_off_starts_colour_only(self):
        node = self._make_node("off")
        node._start_pipeline = MagicMock(return_value=True)

        self.assertTrue(node.init_pipeline())

        node._start_pipeline.assert_called_once_with(include_stereo=False)
        self.assertFalse(node.depth_available)
        node.get_logger().warning.assert_called_once_with(
            "Stereo depth disabled - using colour-only pipeline (depth disabled)"
        )

    def test_mode_auto_without_frames_falls_back_to_colour_only(self):
        node = self._make_node("auto")
        node._start_pipeline = MagicMock(side_effect=[True, True])
        node._wait_for_color_frame = MagicMock(return_value=None)
        node._stop_pipeline = MagicMock()

        self.assertTrue(node.init_pipeline())

        self.assertEqual(
            node._start_pipeline.call_args_list,
            [
                unittest.mock.call(include_stereo=True),
                unittest.mock.call(include_stereo=False),
            ],
        )
        node._stop_pipeline.assert_called_once()
        self.assertFalse(node.depth_available)
        node.get_logger().warning.assert_called_once_with(
            "Stereo depth unavailable - falling back to colour-only pipeline "
            "(depth disabled)"
        )

    def test_mode_auto_with_frames_keeps_stereo(self):
        node = self._make_node("auto")
        first_packet = MagicMock()
        node._start_pipeline = MagicMock(return_value=True)
        node._wait_for_color_frame = MagicMock(return_value=first_packet)
        node._stop_pipeline = MagicMock()

        self.assertTrue(node.init_pipeline())

        node._start_pipeline.assert_called_once_with(include_stereo=True)
        node._stop_pipeline.assert_not_called()
        self.assertTrue(node.depth_available)
        self.assertIs(node._pending_color_packet, first_packet)
        node.get_logger().info.assert_called_once_with(
            "Stereo depth available - full colour + stereo pipeline active (mode=auto)"
        )

    @patch("ros_packages.camera.oak_d_lite.stereo.time.sleep")
    def test_pipeline_start_retries_are_bounded_with_backoff(self, mock_sleep):
        node = self._make_node("off")

        def build_failing_pipeline(include_stereo):
            node.pipeline = MagicMock()
            node.pipeline.start.side_effect = RuntimeError("start failed")

        node._build_pipeline = MagicMock(side_effect=build_failing_pipeline)

        self.assertFalse(node._start_pipeline(include_stereo=False))

        self.assertEqual(node._build_pipeline.call_count, 3)
        self.assertEqual(
            [call.args[0] for call in mock_sleep.call_args_list], [0.25, 0.5]
        )
        self.assertEqual(node.get_logger().error.call_count, 3)

    @patch("ros_packages.camera.oak_d_lite.stereo.time.sleep")
    def test_stop_waits_for_release_and_clears_process_owner(self, mock_sleep):
        node = self._make_node("off")
        pipeline = node.pipeline
        pipeline.isRunning.side_effect = [True, False]
        CameraNode._device_owner = weakref.ref(node)

        self.assertTrue(node._stop_pipeline())

        pipeline.stop.assert_called_once_with()
        self.assertIsNone(node.pipeline)
        self.assertIsNone(CameraNode._device_owner)
        self.assertEqual(mock_sleep.call_count, 1)

    def test_start_refuses_a_second_process_local_device_holder(self):
        first = self._make_node("off")
        second = self._make_node("off")
        CameraNode._device_owner = weakref.ref(first)
        second._build_pipeline = MagicMock()

        try:
            self.assertFalse(second._start_pipeline(include_stereo=False))
            second._build_pipeline.assert_not_called()
            second.get_logger().error.assert_called_once()
        finally:
            CameraNode._device_owner = None


class _ImuNodeType:
    pass


class TestOakImu(unittest.TestCase):
    def test_pure_sample_assembly_maps_frame_units_stamp_and_sequence(self):
        sample = assemble_imu_sample(
            sequence=42,
            stamp_ns=12_345_678_901,
            acceleration_xyz=(1.0, 2.0, 3.0),
            angular_velocity_xyz=(0.1, 0.2, 0.3),
        )

        self.assertEqual(sample.frame_id, "oak_imu_frame")
        self.assertEqual(sample.sequence, 42)
        self.assertEqual(sample.stamp_ns, 12_345_678_901)
        # DepthAI already supplies m/s² and rad/s; only the optical -> REP-103
        # axis mapping (right/down/forward -> forward/left/up) is applied.
        self.assertEqual(
            (
                sample.linear_acceleration.x,
                sample.linear_acceleration.y,
                sample.linear_acceleration.z,
            ),
            (3.0, -1.0, -2.0),
        )
        self.assertEqual(
            (
                sample.angular_velocity.x,
                sample.angular_velocity.y,
                sample.angular_velocity.z,
            ),
            (0.3, -0.1, -0.2),
        )
        for covariance in (
            sample.orientation_covariance,
            sample.angular_velocity_covariance,
            sample.linear_acceleration_covariance,
        ):
            self.assertEqual(covariance[0], -1.0)
            self.assertEqual(covariance[1:], (0.0,) * 8)

        message = CameraNode._imu_message(sample)
        self.assertEqual(message.header.frame_id, "oak_imu_frame")
        self.assertEqual(message.header.stamp.sec, 12)
        self.assertEqual(message.header.stamp.nanosec, 345_678_901)
        # The device refuses to fuse orientation, so consumers must see the
        # documented "unavailable" marker rather than an invented quaternion.
        self.assertEqual(message.orientation_covariance[0], -1.0)
        self.assertEqual(
            (
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ),
            (0.0, 0.0, 0.0, 1.0),
        )

    def test_throttle_publishes_exactly_one_hundred_of_one_thousand_samples(self):
        # One second of 1 ms device stamps has to leave exactly one sample per
        # 10 ms publication period, on the period boundaries.
        throttle = PublishRateThrottle()
        accepted = [
            stamp_ns
            for stamp_ns in range(0, 1_000_000_000, 1_000_000)
            if throttle.accept(stamp_ns)
        ]

        self.assertEqual(IMU_PUBLISH_RATE_HZ, 100)
        self.assertEqual(IMU_PUBLISH_PERIOD_NS, 10_000_000)
        self.assertEqual(len(accepted), 100)
        self.assertEqual(
            accepted,
            [index * IMU_PUBLISH_PERIOD_NS for index in range(100)],
        )

    def test_throttle_passes_every_second_sample_of_the_sensor_stream(self):
        # The sensor runs at twice the publication rate, so the throttle must
        # publish every second report and drop the other half.
        throttle = PublishRateThrottle()
        sensor_period_ns = 1_000_000_000 // IMU_SENSOR_RATE_HZ
        accepted = [
            index for index in range(20) if throttle.accept(index * sensor_period_ns)
        ]

        self.assertEqual(accepted, list(range(0, 20, 2)))

    def test_offset_estimate_takes_the_window_minimum_not_the_mean(self):
        # Queueing, USB scheduling and the poll timer can only delay a report,
        # so the smallest observation is the least contaminated one. A mean
        # would sit at 1_250 here and push every stamp into the future.
        samples = [1_000, 1_500, 1_500, 1_000]
        self.assertEqual(estimated_clock_offset_ns(samples, window=4), 1_000)
        self.assertNotEqual(
            estimated_clock_offset_ns(samples, window=4),
            sum(samples) / len(samples),
        )

    def test_offset_estimate_forgets_samples_that_left_the_window(self):
        # A minimum never rises on its own; only the window can discard it, and
        # that is what lets the estimate follow a drifting clock.
        samples = [1_000, 5_000, 5_500, 6_000]
        self.assertEqual(estimated_clock_offset_ns(samples, window=4), 1_000)
        self.assertEqual(estimated_clock_offset_ns(samples, window=3), 5_000)
        self.assertEqual(estimated_clock_offset_ns(samples, window=2), 5_500)

    def test_offset_estimate_is_unavailable_for_degenerate_input(self):
        # Fewer than two samples is not a window, so there is no estimate and
        # the caller has to fall back to the receipt time.
        self.assertIsNone(estimated_clock_offset_ns([]))
        self.assertIsNone(estimated_clock_offset_ns([1_234]))
        # Non-finite values are not offsets and must not become the minimum.
        self.assertIsNone(estimated_clock_offset_ns([float("nan"), float("-inf")]))
        self.assertEqual(
            estimated_clock_offset_ns([2_000, float("-inf"), 3_000, None]), 2_000
        )
        # Identical samples are a perfectly valid estimate.
        self.assertEqual(estimated_clock_offset_ns([7_000, 7_000]), 7_000)
        # A negative offset is legitimate: the device clock counts from device
        # boot and may be ahead of the host epoch.
        self.assertEqual(estimated_clock_offset_ns([-5_000, -3_000]), -5_000)
        with self.assertRaises(ValueError):
            estimated_clock_offset_ns([1, 2, 3], window=1)

    def test_offset_estimate_keeps_nanosecond_resolution_at_epoch_magnitude(self):
        # Host epoch nanoseconds are around 1.7e18, where float64 quantises in
        # steps of hundreds of nanoseconds, so the estimate must stay integral.
        base = 1_700_000_000_000_000_000
        estimate = estimated_clock_offset_ns([base + 3, base + 1, base + 2])
        self.assertIsInstance(estimate, int)
        self.assertEqual(estimate, base + 1)

    def test_offset_estimator_tracks_the_minimum_over_its_own_window(self):
        estimator = ClockOffsetEstimator(window=3)
        # host receipt 1_000, device 0 -> offset 1_000, but one sample is not a
        # window yet.
        self.assertIsNone(estimator.observe(1_000, 0))
        self.assertEqual(estimator.observe(2_200, 1_000), 1_000)
        self.assertEqual(estimator.observe(3_400, 2_000), 1_000)
        # The 1_000 sample has now left the three-entry window.
        self.assertEqual(estimator.observe(4_500, 3_000), 1_200)
        self.assertIsNone(ClockOffsetEstimator().observe(10, 0))
        self.assertEqual(IMU_CLOCK_OFFSET_WINDOW, 200)

    def test_uniform_device_stamps_produce_a_uniform_host_timeline(self):
        # Device stamps are uniform, receipt times are not. The published stamps
        # must inherit the device spacing, not the receipt jitter.
        offset_ns = 1_700_000_000_000_000_000
        device_stamps = [index * IMU_PUBLISH_PERIOD_NS for index in range(5)]
        jitter = [0, 7_000_000, 1_000_000, 35_000_000, 2_000_000]
        stamps = [
            published_stamp_ns(device_ns, offset_ns + device_ns + late_ns, offset_ns)
            for device_ns, late_ns in zip(device_stamps, jitter)
        ]

        self.assertEqual(
            [second - first for first, second in zip(stamps, stamps[1:])],
            [IMU_PUBLISH_PERIOD_NS] * 4,
        )

    def test_published_stamp_falls_back_to_receipt_time_without_an_offset(self):
        # 7 s is a device duration since device boot. Publishing it as wall time
        # would claim January 1970; the receipt time is late but real.
        device_stamp_ns = 7_000_000_000
        host_receipt_ns = 1_700_000_000_000_000_000
        self.assertEqual(
            published_stamp_ns(device_stamp_ns, host_receipt_ns, None),
            host_receipt_ns,
        )
        # An offset of zero would publish exactly that device duration, so it is
        # refused as well - as is any offset that does not land the stamp near
        # the receipt time.
        self.assertEqual(
            published_stamp_ns(device_stamp_ns, host_receipt_ns, 0),
            host_receipt_ns,
        )
        self.assertEqual(
            published_stamp_ns(device_stamp_ns, host_receipt_ns, -device_stamp_ns),
            host_receipt_ns,
        )

    def test_published_stamp_rejects_an_offset_from_a_restarted_device_clock(self):
        # The device clock restarts at zero on a new device session. An offset
        # measured against the previous session would place the measurement
        # decades off, so the receipt time is published instead.
        stale_offset_ns = 1_700_000_000_000_000_000
        host_receipt_ns = 1_700_000_600_000_000_000
        self.assertEqual(
            published_stamp_ns(5_000_000, host_receipt_ns, stale_offset_ns),
            host_receipt_ns,
        )
        # Inside the tolerance the device-derived stamp is used, which is the
        # whole point of the estimate.
        fresh_offset_ns = host_receipt_ns - 5_000_000 - 3_000_000
        self.assertEqual(
            published_stamp_ns(5_000_000, host_receipt_ns, fresh_offset_ns),
            host_receipt_ns - 3_000_000,
        )

    def test_published_stamp_is_never_later_than_the_receipt_time(self):
        # The offset is a minimum of (receipt - device), so device + offset
        # cannot exceed the receipt time of the report that set that minimum.
        estimator = ClockOffsetEstimator(window=8)
        latencies = [4_000_000, 1_500_000, 9_000_000, 1_500_000, 2_500_000]
        offset_ns = 1_700_000_000_000_000_000
        for index, latency_ns in enumerate(latencies):
            device_ns = index * IMU_PUBLISH_PERIOD_NS
            host_receipt_ns = offset_ns + device_ns + latency_ns
            estimate = estimator.observe(host_receipt_ns, device_ns)
            self.assertLessEqual(
                published_stamp_ns(device_ns, host_receipt_ns, estimate),
                host_receipt_ns,
            )

    def test_processing_publishes_the_measurement_instant_on_the_host_timeline(self):
        # Reports arrive with 5 ms of extra queueing on the middle one. The
        # published stamps must still be exactly one publication period apart,
        # because the offset estimate removes the receipt jitter.
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()

        def packet(device_seconds):
            acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=9.81)
            acceleration.getSequenceNum = MagicMock(return_value=1)
            # getTimestamp() is a timedelta on depthai 3.6.1, so the node falls
            # back to the host clock it reads itself for the receipt time.
            acceleration.getTimestamp = MagicMock(
                return_value=timedelta(seconds=device_seconds)
            )
            acceleration.getTimestampDevice = MagicMock(
                return_value=timedelta(seconds=device_seconds)
            )
            gyroscope = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            return types.SimpleNamespace(
                acceleroMeter=acceleration, gyroscope=gyroscope
            )

        node.imu_queue = MagicMock()
        node.imu_queue.tryGet.side_effect = [
            types.SimpleNamespace(
                packets=[packet(0.0), packet(0.01), packet(0.02)],
            ),
            None,
        ]
        node.imu_publisher_ = MagicMock()
        node._imu_throttle = PublishRateThrottle()
        node._imu_clock_offset = ClockOffsetEstimator()
        node._imu_publish_times = deque(maxlen=8)
        node._imu_last_received_monotonic = None
        node.get_logger = MagicMock()

        receipts = [1000.100, 1000.115, 1000.120]
        with patch(
            "ros_packages.camera.oak_d_lite.stereo.time.time",
            side_effect=receipts,
        ):
            node._process_imu()

        stamps = []
        for call in node.imu_publisher_.publish.call_args_list:
            stamp = call.args[0].header.stamp
            stamps.append(stamp.sec * 1_000_000_000 + stamp.nanosec)

        self.assertEqual(len(stamps), 3)
        self.assertEqual(
            [second - first for first, second in zip(stamps, stamps[1:])],
            [IMU_PUBLISH_PERIOD_NS] * 2,
        )
        # The second report was received 15 ms after the first but measured
        # 10 ms after it, and the stamp reports the measurement.
        self.assertEqual(stamps[1], 1_000_110_000_000)
        self.assertNotEqual(stamps[1], int(round(receipts[1] * 1_000_000_000)))
        self.assertEqual(node._imu_last_device_stamp_ns, 20_000_000)

    def test_processing_falls_back_to_receipt_time_before_an_offset_exists(self):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        acceleration = types.SimpleNamespace(x=1.0, y=2.0, z=3.0)
        acceleration.getSequenceNum = MagicMock(return_value=17)
        # Measured on depthai 3.6.1: getTimestamp() is a timedelta "related to
        # dai::Clock::now()" and getTimestampDevice() is a device-monotonic
        # timedelta; neither is host wall time. The epoch-valued datetime stands
        # in for the host clock the node reads itself when it determines the
        # receipt time.
        acceleration.getTimestamp = MagicMock(
            return_value=datetime.fromtimestamp(25.123456, tz=timezone.utc)
        )
        acceleration.getTimestampDevice = MagicMock(
            return_value=timedelta(seconds=7, microseconds=654321)
        )
        gyroscope = types.SimpleNamespace(x=0.1, y=0.2, z=0.3)
        packet = types.SimpleNamespace(
            acceleroMeter=acceleration,
            gyroscope=gyroscope,
        )
        node.imu_queue = MagicMock()
        node.imu_queue.tryGet.side_effect = [
            types.SimpleNamespace(packets=[packet]),
            None,
        ]
        node.imu_publisher_ = MagicMock()
        node._imu_throttle = PublishRateThrottle()
        node._imu_clock_offset = ClockOffsetEstimator()
        node._imu_publish_times = deque(maxlen=8)
        node._imu_last_received_monotonic = None
        node.get_logger = MagicMock()

        node._process_imu()

        # A single report is not a window, so no offset exists yet and the
        # receipt time is published - never the 7.654 s device duration.
        message = node.imu_publisher_.publish.call_args.args[0]
        self.assertEqual(
            (message.header.stamp.sec, message.header.stamp.nanosec),
            (25, 123456000),
        )
        self.assertEqual(node._imu_last_sequence, 17)
        self.assertEqual(node._imu_last_device_stamp_ns, 7_654_321_000)
        self.assertEqual(len(node._imu_publish_times), 1)
        node.get_logger().warning.assert_not_called()
        acceleration.getTimestamp.assert_called_once_with()
        acceleration.getTimestampDevice.assert_called_once_with()

    def test_host_stamp_uses_receipt_time_when_the_report_has_no_host_epoch(self):
        # depthai 3.6.1 exposes no getTimestampSystem() on the IMU reports. A
        # device duration must never be reinterpreted as wall time, so the host
        # time of receipt is used whenever the report carries no epoch stamp.
        self.assertEqual(
            host_stamp_nanoseconds(timedelta(seconds=7), 1_700_000_000.5),
            1_700_000_000_500_000_000,
        )
        self.assertEqual(
            host_stamp_nanoseconds(None, 1_700_000_000.5),
            1_700_000_000_500_000_000,
        )
        self.assertEqual(
            host_stamp_nanoseconds(
                datetime.fromtimestamp(25.123456, tz=timezone.utc), 1_700_000_000.5
            ),
            25_123_456_000,
        )

    def test_throttle_follows_the_device_clock_not_the_quantised_host_stamp(self):
        # Measured: the drain timer quantises host receipt stamps, so throttling
        # on them skipped periods and published 8.1 Hz instead of 10 Hz. Two
        # reports one publication period apart on the device clock but only half
        # a period apart in host receipt time must both be published.
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()

        def packet(device_seconds, host_seconds):
            acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=9.81)
            acceleration.getSequenceNum = MagicMock(return_value=1)
            acceleration.getTimestamp = MagicMock(
                return_value=datetime.fromtimestamp(host_seconds, tz=timezone.utc)
            )
            acceleration.getTimestampDevice = MagicMock(
                return_value=timedelta(seconds=device_seconds)
            )
            gyroscope = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            return types.SimpleNamespace(
                acceleroMeter=acceleration, gyroscope=gyroscope
            )

        node.imu_queue = MagicMock()
        node.imu_queue.tryGet.side_effect = [
            types.SimpleNamespace(
                packets=[packet(0.0, 25.000), packet(0.01, 25.005)],
            ),
            None,
        ]
        node.imu_publisher_ = MagicMock()
        node._imu_throttle = PublishRateThrottle()
        node._imu_clock_offset = ClockOffsetEstimator()
        node._imu_publish_times = deque(maxlen=8)
        node._imu_last_received_monotonic = None
        node.get_logger = MagicMock()

        node._process_imu()

        self.assertEqual(node.imu_publisher_.publish.call_count, 2)
        self.assertEqual(node._imu_last_device_stamp_ns, 10_000_000)

    def test_measured_rate_reports_the_observed_rate_not_the_configured_one(self):
        # The rig measured 8.1 Hz while the configuration said 10 Hz, so the
        # status must carry what was observed.
        self.assertAlmostEqual(measured_rate_hz([0.0, 0.1, 0.2, 0.3]), 10.0, places=6)
        self.assertAlmostEqual(
            measured_rate_hz([0.0, 0.1, 0.3, 0.4, 0.6]), 4 / 0.6, places=6
        )
        # One second of publications at the new rate must read as 100 Hz.
        publications = [index / IMU_PUBLISH_RATE_HZ for index in range(101)]
        self.assertAlmostEqual(measured_rate_hz(publications), 100.0, places=6)
        for degenerate in ([], [5.0], [5.0, 5.0]):
            self.assertEqual(measured_rate_hz(degenerate), 0.0)

    def test_imu_status_carries_a_measured_rate_field(self):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.imu_available = False
        node.imu_queue = None
        self.assertEqual(node._imu_status()["fps"], 0.0)

        node.imu_available = True
        node.imu_queue = MagicMock()
        node._imu_last_received_monotonic = 10.0
        # The window the node really keeps, filled at the publication period.
        node._imu_publish_times = deque(
            (index * IMU_POLL_PERIOD_SECONDS for index in range(IMU_RATE_WINDOW)),
            maxlen=IMU_RATE_WINDOW,
        )
        status = node._imu_status(now=10.0)
        self.assertEqual(status["state"], "present")
        self.assertAlmostEqual(status["fps"], 100.0, places=6)
        # The status field must not carry fifteen digits of float noise.
        self.assertEqual(status["fps"], round(status["fps"], 2))

    def test_imu_status_distinguishes_stale_and_present(self):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.imu_available = True
        node.imu_queue = MagicMock()
        node._imu_last_received_monotonic = None

        self.assertEqual(node._imu_status(now=10.0)["state"], "stale")
        # The threshold is a documented multiple of the publication period, not
        # a round second: at 100 Hz a one-second silence is a hundred lost
        # samples.
        self.assertEqual(IMU_STALE_MISSED_PUBLICATIONS, 50)
        self.assertAlmostEqual(IMU_STALE_AFTER_SECONDS, 0.5, places=9)
        node._imu_last_received_monotonic = 10.0 - IMU_STALE_AFTER_SECONDS / 2
        self.assertEqual(node._imu_status(now=10.0)["state"], "present")
        node._imu_last_received_monotonic = 10.0 - IMU_STALE_AFTER_SECONDS * 2
        self.assertEqual(node._imu_status(now=10.0)["state"], "stale")

    def test_imu_rate_constants_describe_one_hundred_hertz_publication(self):
        # The sensor has to run above the publication rate, the poll timer has
        # to wake once per publication period, and the drain limit has to be
        # able to empty a full device queue in a single tick.
        self.assertEqual(IMU_PUBLISH_RATE_HZ, 100)
        self.assertEqual(IMU_SENSOR_RATE_HZ, 200)
        self.assertAlmostEqual(IMU_POLL_PERIOD_SECONDS, 0.01, places=9)
        self.assertGreaterEqual(IMU_SENSOR_RATE_HZ, IMU_PUBLISH_RATE_HZ)
        self.assertGreater(IMU_DRAIN_LIMIT, IMU_OUTPUT_QUEUE_DEPTH)
        # The queue has to hold more than one poll period worth of reports so a
        # late tick does not lose samples.
        self.assertGreater(
            IMU_OUTPUT_QUEUE_DEPTH, IMU_SENSOR_RATE_HZ * IMU_POLL_PERIOD_SECONDS
        )

    def test_rebuilding_the_pipeline_resets_the_imu_clock_state(self):
        # A new pipeline is a new device session with a device clock that starts
        # near zero. A retained throttle deadline would drop every report until
        # the restarted clock caught up, and a retained offset window would
        # stamp against a clock that no longer exists.
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.pipeline = None
        node._pipeline_models = []
        node._imu_throttle = PublishRateThrottle()
        node._imu_throttle.accept(10 * 10**9)
        node._imu_clock_offset = ClockOffsetEstimator()
        node._imu_clock_offset.observe(1_700_000_000_000_000_000, 10 * 10**9)
        node._imu_clock_offset.observe(1_700_000_000_000_000_000, 10 * 10**9)
        node._imu_publish_times = deque([1.0, 2.0], maxlen=IMU_RATE_WINDOW)
        stale_throttle = node._imu_throttle
        self.assertIsNotNone(node._imu_clock_offset.offset_ns())

        with patch("ros_packages.camera.oak_d_lite.stereo.dai"):
            node._build_pipeline(include_stereo=False)

        self.assertIsNot(node._imu_throttle, stale_throttle)
        self.assertTrue(node._imu_throttle.accept(0))
        self.assertIsNone(node._imu_clock_offset.offset_ns())
        self.assertEqual(len(node._imu_publish_times), 0)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_imu_pipeline_uses_supported_reports_and_non_blocking_queue(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.pipeline = MagicMock()
        node.get_logger = MagicMock()
        imu = MagicMock()
        node.pipeline.create.return_value = imu
        mock_dai.node.IMU = _ImuNodeType
        sensors = (
            "ACCELEROMETER_RAW",
            "GYROSCOPE_RAW",
        )
        (
            mock_dai.IMUSensor.ACCELEROMETER_RAW,
            mock_dai.IMUSensor.GYROSCOPE_RAW,
        ) = sensors
        # Measured on the robot: the BMI270 answers a ROTATION_VECTOR request
        # with "IMU invalid settings!" and that single rejected sensor takes the
        # whole pipeline start down, so it must never be requested again.
        mock_dai.IMUSensor.ROTATION_VECTOR = "ROTATION_VECTOR"

        self.assertTrue(node._init_imu())

        self.assertEqual(
            imu.enableIMUSensor.call_args_list,
            [unittest.mock.call(sensor, IMU_SENSOR_RATE_HZ) for sensor in sensors],
        )
        self.assertNotIn(
            "ROTATION_VECTOR",
            [call.args[0] for call in imu.enableIMUSensor.call_args_list],
        )
        imu.setBatchReportThreshold.assert_called_once_with(1)
        imu.setMaxBatchReports.assert_called_once_with(IMU_OUTPUT_QUEUE_DEPTH)
        imu.out.createOutputQueue.assert_called_once_with(
            maxSize=IMU_OUTPUT_QUEUE_DEPTH, blocking=False
        )
        self.assertTrue(node.imu_available)

    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_missing_imu_raises_nothing_and_status_reports_absence(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node.pipeline = MagicMock()
        node.pipeline.create.side_effect = RuntimeError("IMU not detected")
        node.queue = MagicMock()
        node.get_logger = MagicMock()
        node.pipeline_manager = MagicMock()
        node.pipeline_manager.statuses.return_value = {}
        node.model_registry = MagicMock()
        node.model_registry.models.return_value = []
        node.models_status_publisher_ = MagicMock()
        node.get_clock = MagicMock()
        node._log_hand_stage_counters = MagicMock()
        node._log_imitation_stage_counters = MagicMock()

        self.assertFalse(node._init_imu())
        node.publish_model_statuses()

        status_array = node.models_status_publisher_.publish.call_args.args[0]
        self.assertEqual(status_array.models[-1].model_id, "imu")
        self.assertEqual(status_array.models[-1].state, "absent")
        self.assertIsNone(node.imu_queue)
        self.assertFalse(node.imu_available)
        self.assertIsNotNone(node.queue)

    def test_missing_imu_rebuilds_clean_graph_and_starts_camera(self):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()
        node._pipeline_lock = threading.RLock()
        node.get_logger = MagicMock()
        first_pipeline = MagicMock()
        camera_only_pipeline = MagicMock()
        pipelines = iter((first_pipeline, camera_only_pipeline))

        def build_camera(_include_stereo):
            node.pipeline = next(pipelines)
            node.queue = MagicMock()

        def reject_imu():
            node.imu_available = False
            node.imu_queue = None
            return False

        node._build_pipeline = MagicMock(side_effect=build_camera)
        node._init_imu = MagicMock(side_effect=reject_imu)
        CameraNode._device_owner = None
        try:
            self.assertTrue(node._start_pipeline(include_stereo=False))
            self.assertEqual(node._build_pipeline.call_count, 2)
            node._init_imu.assert_called_once_with()
            first_pipeline.start.assert_not_called()
            camera_only_pipeline.start.assert_called_once_with()
            self.assertIsNotNone(node.queue)
            self.assertEqual(node._imu_status()["state"], "absent")
        finally:
            CameraNode._device_owner = None


if __name__ == "__main__":
    unittest.main()
