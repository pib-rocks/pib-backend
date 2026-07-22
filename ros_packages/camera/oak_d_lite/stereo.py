#!/usr/bin/python3
import base64
import os
import cv2
import depthai as dai
import rclpy
from datatypes.srv import GetCameraImage
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, Int32, Int32MultiArray, String


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

        self.camera_available = self.init_pipeline()

        if self.camera_available:
            self.get_camera_image_service = self.create_service(
                GetCameraImage, "get_camera_image", self.get_camera_image_callback
            )
            self.get_logger().info("Camera service initialized.")
        else:
            self.get_logger().error("Camera not available.")

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def get_camera_image_callback(self, request, response):
        self.get_logger().info(f"LEN IMAGE: {len(self.current_image)}")
        response.image_base64 = self.current_image
        return response

    def init_pipeline(self) -> bool:
        try:
            self.pipeline = dai.Pipeline()

            # depthai v3 API: Camera node replaces deprecated ColorCamera.
            # XLinkOut no longer exists — use requestIspOutput() to get a
            # Node.Output, then createOutputQueue() on it after startPipeline.
            # NOTE: the ColorCamera 'preview' output produced all-black frames
            # on this OAK-D-Lite + depthai 2.25 (auto-exposure never converges
            # on the preview path — verified: 0/236 frames had content, std=0).
            # The 'isp' output works correctly with auto-exposure (std~70), so
            # we stream the full-resolution ISP frame and resize it in software
            # to the requested preview size. See Jira PR-1480.
            self.camRgb = self.pipeline.create(dai.node.Camera)
            self.camRgb.build(dai.CameraBoardSocket.CAM_A)
            self.isp_out = self.camRgb.requestIspOutput()

            # Force USB 2.0 (HIGH) speed to avoid XLink / USB 3.0 re-enumeration
            # bugs on Raspberry Pi 5 which cause X_LINK_DEVICE_ALREADY_IN_USE.
            self.device = dai.Device(dai.UsbSpeed.HIGH)
            self.device.startPipeline(self.pipeline)

            self.queue = self.isp_out.createOutputQueue()
            return True

        except Exception as e:
            import traceback

            print("====================================")
            print("CAMERA INIT FAILED")
            traceback.print_exc()
            print("====================================")

            self.get_logger().error(f"Camera not found: {e}")
            self.device = None
            self.queue = None
            return False

    def publish_face_center(self, frame):
        face_msg = Float32MultiArray()

        if self.face_cascade.empty():
            face_msg.data = [0.0, 0.0]
            self.face_center_publisher_.publish(face_msg)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 5)

        if len(faces) > 0:
            x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
            x_center = (x + w / 2) - (frame.shape[1] / 2)
            y_center = (frame.shape[0] / 2) - (y + h / 2)
            face_msg.data = [float(x_center), float(y_center)]
        else:
            face_msg.data = [0.0, 0.0]

        self.face_center_publisher_.publish(face_msg)

    def timer_callback(self):
        if not self.queue:
            return

        image_rgb = self.queue.tryGet()
        if image_rgb is None:
            return

        frame = image_rgb.getCvFrame()

        # The ISP output is full sensor resolution (1920x1080). Resize to the
        # configured preview size so downstream consumers and the JPEG match the
        # requested dimensions (see PR-1480 — 'preview' output was black).
        if (
            frame.shape[1] != self.preview_width
            or frame.shape[0] != self.preview_height
        ):
            frame = cv2.resize(
                frame, (self.preview_width, self.preview_height)
            )

        self.publish_face_center(frame)

        retval, buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality_factor]
        )
        if not retval:
            return

        jpg_as_text = base64.b64encode(buffer)

        msg = String()
        msg.data = jpg_as_text.decode("utf-8")
        self.current_image = msg.data
        self.publisher_.publish(msg)

    def timer_period_callback(self, msg):
        self.timer_period = msg.data
        self.timer.cancel()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def quality_factor_callback(self, msg):
        self.quality_factor = msg.data

    def preview_size_callback(self, msg):
        self.preview_width, self.preview_height = msg.data

        if self.device is not None:
            self.device.close()

        self.init_pipeline()


def spin_camera(times):
    cnt = times
    if cnt == 0:
        print("Couldn't restart camera due to displayed error/s, publishing error message")
        rclpy.spin(error_publisher)
    else:
        try:
            camera_node = CameraNode()
            rclpy.spin(camera_node)
        except Exception as exc:
            error_publisher.timer_callback()
            print(exc)
        finally:
            if "camera_node" in locals():
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
