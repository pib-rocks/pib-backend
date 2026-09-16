"""Unit tests for stereo camera CPU optimization (PR-1507)."""

import os
import sys
import types
import unittest
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

from ros_packages.camera.oak_d_lite.stereo import (
    CameraNode,
    FACE_DETECT_WIDTH,
    FACE_DETECT_HEIGHT,
    HAND_NN_HEIGHT,
    HAND_NN_WIDTH,
)


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


class TestHandPipelineInput(unittest.TestCase):
    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    def test_hand_manips_share_bounded_camera_output(self, mock_dai):
        with patch.object(CameraNode, "__init__", lambda self: None):
            node = CameraNode()

        artifacts = {
            "palm_detection_128x128": types.SimpleNamespace(
                input_width=128, input_height=128, blob_path="/palm.blob"
            ),
            "palm_detection_128x128_decoding": types.SimpleNamespace(
                blob_path="/decoder.blob"
            ),
            "hand_landmark_224x224": types.SimpleNamespace(
                input_width=224, input_height=224, blob_path="/landmark.blob"
            ),
        }
        node.model_registry = MagicMock()
        node.model_registry.get.side_effect = artifacts.get
        node.pipeline = MagicMock()
        created = [MagicMock() for _ in range(5)]
        node.pipeline.create.side_effect = created
        node.camRgb = MagicMock()
        hand_input = MagicMock()
        node.camRgb.requestOutput.return_value = hand_input

        node._build_hand_pipeline(
            types.SimpleNamespace(artifact_ids=tuple(artifacts))
        )

        node.camRgb.requestOutput.assert_called_once_with(
            (HAND_NN_WIDTH, HAND_NN_HEIGHT),
            type=mock_dai.ImgFrame.Type.BGR888p,
        )
        hand_input.link.assert_has_calls(
            [
                unittest.mock.call(created[0].inputImage),
                unittest.mock.call(created[3].inputImage),
            ]
        )
        self.assertEqual(node.hand_source_size, (1280, 720))


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


if __name__ == "__main__":
    unittest.main()
