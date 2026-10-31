"""Unit tests for OAK on-chip MJPEG encoding, /oak/rgb/image_raw/compressed,
HTTP MJPEG streaming server, and startup camera settings (PR-2140).
"""

from __future__ import annotations

import base64
import io
import json
import sys
import types
from unittest.mock import MagicMock, patch
from urllib.error import HTTPError
from pathlib import Path
from urllib.request import urlopen

import numpy as np
import pytest

import os

sys.path.insert(0, os.path.dirname(__file__))
import test_stereo_camera_optimization  # noqa: F401

from ros_packages.camera.oak_d_lite import stereo as camera_stereo
from ros_packages.camera.oak_d_lite.mjpeg_server import (
    MjpegFrameBuffer,
    MjpegStreamServer,
    STREAM_BOUNDARY,
)
from ros_packages.camera.oak_d_lite.stereo import (
    COMPRESSED_IMAGE_FRAME_ID,
    COMPRESSED_IMAGE_TOPIC,
    CameraNode,
)


def _bare_node() -> CameraNode:
    with patch.object(CameraNode, "__init__", lambda self: None):
        node = CameraNode()
    node.get_logger = MagicMock()
    node.get_clock = MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = "stamp-0"
    node.preview_width = 640
    node.preview_height = 480
    node.quality_factor = 80
    node.timer_period = 0.1
    node.current_jpeg_bytes = b""
    node.current_image = ""
    node.current_frame = None
    node.current_source_size = (0, 0)
    node.mjpeg_buffer = MjpegFrameBuffer()
    node.mjpeg_server = None
    node.compressed_publisher_ = MagicMock()
    node.face_center_publisher_ = MagicMock()
    node.face_center_publisher_.get_subscription_count.return_value = 0
    node.hand_decoder_queue = None
    node.imitation_queue = None
    node.nn_queues = {}
    node._pending_nn_packets = {}
    node.hand_palm_queue = None
    node.hand_roi_queue = None
    node.hand_landmark_queue = None
    node.depth_queue = None
    node.hand_mp_landmark_queue = None
    node.hand_mp_detection_queue = None
    node._hand_warnings = set()
    node._pending_color_packet = None
    node._pipeline_lock = camera_stereo.threading.RLock()
    return node


def test_camera_node_Has_no_legacy_camera_topic_publisher():
    """D3 (3B): legacy /camera_topic (std_msgs/String) is completely removed."""
    source = Path(camera_stereo.__file__).read_text(encoding="utf-8")
    assert '"camera_topic"' not in source
    assert "'camera_topic'" not in source


def test_mjpeg_encoder_branch_links_16_9_branch_into_nv12_manip_and_video_encoder():
    """D2 (2A): 16:9 sensor branch -> ImageManip (STRETCH, NV12, 640x480) -> VideoEncoder (MJPEG)."""
    node = _bare_node()
    node.pipeline = MagicMock()
    node.camRgb = MagicMock()
    node.isp_out = MagicMock()
    node.isp_queue = MagicMock()
    source_branch = MagicMock()
    node.camRgb.requestOutput.return_value = source_branch

    manip = MagicMock()
    encoder = MagicMock()
    bitstream_queue = MagicMock()
    encoder.bitstream.createOutputQueue.return_value = bitstream_queue

    class FakeManip:
        pass

    class FakeEncoder:
        pass

    def create_side_effect(kind):
        if kind is FakeManip:
            return manip
        if kind is FakeEncoder:
            return encoder
        return MagicMock()

    node.pipeline.create.side_effect = create_side_effect

    with patch.object(camera_stereo, "dai") as mock_dai:
        mock_dai.node.ImageManip = FakeManip
        mock_dai.node.VideoEncoder = FakeEncoder
        mock_dai.ImgFrame.Type.NV12 = "NV12"
        mock_dai.ImgFrame.Type.BGR888p = "BGR888p"
        mock_dai.ImageManipConfig.ResizeMode.STRETCH = "STRETCH"
        mock_dai.VideoEncoderProperties.Profile.MJPEG = "MJPEG"

        node._build_mjpeg_encoder_branch()

    # 16:9 branch requested even when preview_width x preview_height is 640x480 (4:3)
    node.camRgb.requestOutput.assert_called_once_with(
        (camera_stereo.COLOUR_BRANCH_WIDTH, camera_stereo.COLOUR_BRANCH_HEIGHT),
        type="BGR888p",
        fps=10,
    )
    manip.initialConfig.setOutputSize.assert_called_once_with(640, 480, mode="STRETCH")
    manip.initialConfig.setFrameType.assert_called_once_with("NV12")
    source_branch.link.assert_called_once_with(manip.inputImage)
    encoder.setDefaultProfilePreset.assert_called_once_with(10, "MJPEG")
    encoder.setQuality.assert_called_once_with(80)
    manip.out.link.assert_called_once_with(encoder.input)
    assert node.queue is bitstream_queue


def test_timer_callback_publishes_hardware_jpeg_to_mjpeg_buffer_and_compressed_topic():
    """Hardware JPEG packet updates MjpegFrameBuffer and /oak/rgb/image_raw/compressed without cv2.imencode."""
    node = _bare_node()
    node.compressed_publisher_.get_subscription_count.return_value = 1
    node._encode_frame = MagicMock()

    fake_jpeg = b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\xff\xd9"
    packet = MagicMock()
    packet.getData.return_value = fake_jpeg
    node.queue = MagicMock()
    node.queue.tryGet.return_value = packet

    node.timer_callback()

    # No host OpenCV JPEG encoding called
    node._encode_frame.assert_not_called()
    assert node.current_jpeg_bytes == fake_jpeg
    assert node.mjpeg_buffer.latest_jpeg == fake_jpeg
    node.compressed_publisher_.publish.assert_called_once()
    msg = node.compressed_publisher_.publish.call_args.args[0]
    assert msg.format == "jpeg"
    assert msg.header.frame_id == COMPRESSED_IMAGE_FRAME_ID
    assert bytes(msg.data) == fake_jpeg


def test_get_camera_image_callback_uses_hardware_jpeg_bytes_without_opencv():
    node = _bare_node()
    fake_jpeg = b"\xff\xd8\xff\xe0test-jpeg\xff\xd9"
    node.current_jpeg_bytes = fake_jpeg
    node._encode_frame = MagicMock()

    response = types.SimpleNamespace(image_base64="")
    out = node.get_camera_image_callback(None, response)

    node._encode_frame.assert_not_called()
    assert base64.b64decode(out.image_base64) == fake_jpeg


def test_startup_settings_loaded_from_flask(monkeypatch):
    """D4 (4B): CameraNode loads resX, resY, qualityFactor, refreshRate from Flask on startup."""
    node = _bare_node()
    monkeypatch.setenv("FLASK_API_BASE_URL", "http://flask-app:5000")
    payload = json.dumps(
        {
            "resX": 640,
            "resY": 480,
            "resolution": "SD",
            "qualityFactor": 70,
            "refreshRate": 0.1,
        }
    ).encode("utf-8")

    class FakeResp:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return payload

    with patch.object(camera_stereo, "urlopen", return_value=FakeResp()):
        node._load_startup_camera_settings()

    assert (node.preview_width, node.preview_height) == (640, 480)
    assert node.quality_factor == 70
    assert node.timer_period == pytest.approx(0.1)


def test_timer_period_callback_supports_up_to_30_fps_and_rebuilds_pipeline():
    node = _bare_node()
    node.timer = MagicMock()
    node.create_timer = MagicMock()
    node._stop_pipeline = MagicMock(return_value=True)
    node.init_pipeline = MagicMock(return_value=True)

    node.timer_period_callback(types.SimpleNamespace(data=0.03))

    assert node.timer_period == pytest.approx(1.0 / 30.0)
    assert node._encoder_fps() == 30
    node._stop_pipeline.assert_called_once()
    node.init_pipeline.assert_called_once()


def test_mjpeg_http_server_serves_snapshot_and_multipart_stream():
    """D1 (1A): ThreadingHTTPServer serves /snapshot and multipart/x-mixed-replace /stream."""
    buf = MjpegFrameBuffer()
    server = MjpegStreamServer(frame_buffer=buf, host="127.0.0.1", port=0)
    assert server.start() is True
    port = server.server_port
    try:
        # Before first frame: /snapshot returns 503
        with pytest.raises(HTTPError) as exc_info:
            urlopen(f"http://127.0.0.1:{port}/snapshot", timeout=1.0)
        assert exc_info.value.code == 503

        frame1 = b"\xff\xd8\xff\xe0frame-one\xff\xd9"
        buf.publish(frame1)

        with urlopen(f"http://127.0.0.1:{port}/snapshot", timeout=2.0) as resp:
            assert resp.status == 200
            assert resp.headers.get("Content-Type") == "image/jpeg"
            assert resp.read() == frame1

        with urlopen(f"http://127.0.0.1:{port}/stream", timeout=2.0) as resp:
            assert resp.status == 200
            ctype = resp.headers.get("Content-Type", "")
            assert f"multipart/x-mixed-replace; boundary={STREAM_BOUNDARY}" in ctype
            assert resp.headers.get("X-Accel-Buffering") == "no"
            chunk = resp.read(len(frame1) + 57)
            assert b"--frame\r\nContent-Type: image/jpeg" in chunk
            assert frame1 in chunk
    finally:
        server.stop()
