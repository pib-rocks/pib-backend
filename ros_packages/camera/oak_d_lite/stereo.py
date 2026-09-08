#!/usr/bin/python3
import base64
import os
import cv2
import depthai as dai
import numpy as np
import rclpy
from datatypes.srv import GetCameraImage, GetDepthFrame, GetDistanceAtPx
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, Int32, Int32MultiArray, String

# Downscaled resolution for Haar cascade face detection (maps back to full frame).
FACE_DETECT_WIDTH = 320
FACE_DETECT_HEIGHT = 180


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
        self.current_depth = None
        self.pipeline = None
        self.queue = None
        self.depth_queue = None

        self.camera_available = self.init_pipeline()

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

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

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
        if self.current_depth is None:
            return response
        height, width = self.current_depth.shape[:2]
        x, y = int(request.x), int(request.y)
        if x < 0 or y < 0 or x >= width or y >= height:
            return response
        response.distance_mm = float(self.current_depth[y, x])
        return response

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
        self.get_logger().info("Stereo depth outputs added to pipeline.")

    def init_pipeline(self) -> bool:
        try:
            # depthai v3 API:
            # Camera node replaces deprecated ColorCamera.
            # XLinkOut is completely removed in DepthAI v3.
            # Output queues are created directly from node outputs via requestIspOutput().
            # Pipeline is started via pipeline.start() once (OAK-D-Lite 1-pipeline constraint).
            # Color ISP and StereoDepth share this single pipeline.
            self.pipeline = dai.Pipeline()
            self.camRgb = self.pipeline.create(dai.node.Camera)
            self.camRgb.build(dai.CameraBoardSocket.CAM_A)
            self.isp_out = self.camRgb.requestIspOutput()

            self.queue = self.isp_out.createOutputQueue()
            self.depth_queue = None

            try:
                self._init_stereo_depth()
            except Exception as stereo_exc:
                self.get_logger().warning(f"Stereo depth not available: {stereo_exc}")
                self.depth_queue = None

            self.pipeline.start()

            self.get_logger().info("DepthAI v3 pipeline started successfully.")
            return True

        except Exception as e:
            import traceback

            print("====================================")
            print("CAMERA INIT FAILED")
            traceback.print_exc()
            print("====================================")

            self.get_logger().error(f"Camera not found: {e}")
            self.pipeline = None
            self.queue = None
            self.depth_queue = None
            return False

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
            image_rgb = self.queue.tryGet()
            if image_rgb is not None:
                frame = image_rgb.getCvFrame()

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

        if self.pipeline is not None:
            self.pipeline.stop()

        self.init_pipeline()


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
