"""Unit tests for stereo camera CPU optimization (PR-1507)."""

import os
import sys
import types
import unittest
import weakref
from collections import deque
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

try:
    import rclpy  # noqa: F401
except ImportError:
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
        palm_nn.out.link.assert_has_calls(
            [
                unittest.mock.call(decoder_nn.inputs["classificators"]),
                unittest.mock.call(decoder_nn.inputs["regressors"]),
            ]
        )
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


if __name__ == "__main__":
    unittest.main()
