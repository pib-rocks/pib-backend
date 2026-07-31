"""Unit tests for stereo camera CPU optimization (PR-1507)."""

import os
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../ros_packages/camera/oak_d_lite")))

import numpy as np

# Import CameraNode module
from ros_packages.camera.oak_d_lite.stereo import CameraNode, FACE_DETECT_WIDTH, FACE_DETECT_HEIGHT


class TestStereoCameraOptimization(unittest.TestCase):

    @patch("ros_packages.camera.oak_d_lite.stereo.cv2.CascadeClassifier")
    @patch("ros_packages.camera.oak_d_lite.stereo.dai")
    @patch("ros_packages.camera.oak_d_lite.stereo.os.path.exists", return_value=True)
    def test_publish_face_center_skips_when_no_subscribers(self, mock_exists, mock_dai, mock_cascade):
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
    def test_publish_face_center_downscales_frame_when_subscribed(self, mock_exists, mock_dai, mock_cascade):
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
    def test_timer_callback_skips_encoding_when_no_camera_subscribers(self, mock_exists, mock_dai):
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


if __name__ == "__main__":
    unittest.main()
