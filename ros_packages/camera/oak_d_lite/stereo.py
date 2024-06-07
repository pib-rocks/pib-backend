#!/usr/bin/python3
import base64

import cv2
import depthai as dai
import rclpy
from datatypes.srv import GetCameraImage
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int32, Int32MultiArray


class ErrorPublisher(Node):

    # def __new__(cls, error_message):
    #    print("creating new ErrorPublisher with Error message" + error_message)

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
        # self.get_logger().info('Publishing: "%s"' % msg.data)


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")
        self.publisher_ = self.create_publisher(String, "camera_topic", 10)
        self.timer_subscription = self.create_subscription(
            Float64, "timer_period_topic", self.timer_period_callback, 10
        )
        self.quality_factor_subscription = self.create_subscription(
            Int32, "quality_factor_topic", self.quality_factor_callback, 10
        )
        self.preview_size_subscription = self.create_subscription(
            Int32MultiArray, "size_topic", self.preview_size_callback, 10
        )
        self.get_camera_image_service = self.create_service(
            GetCameraImage, "get_camera_image", self.get_camera_image_callback
        )

        # Initialize default preview size and quality factor
        self.preview_width = 1280
        self.preview_height = 720
        self.quality_factor = 80

        # Initialize pipeline
        self.init_pipeline()

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def get_camera_image_callback(self, request, response):
        self.get_logger().info(f"LEN IMAGE: {len(self.current_image)}")
        response = GetCameraImage.Response(image_base64=self.current_image)
        return response

    def init_pipeline(self):
        self.pipeline = dai.Pipeline()

        # Define a source - color camera
        self.camRgb = self.pipeline.createColorCamera()
        self.camRgb.setPreviewSize(self.preview_width, self.preview_height)
        self.camRgb.setInterleaved(False)

        # Create output
        xoutRgb = self.pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        self.camRgb.preview.link(xoutRgb.input)

        self.device = dai.Device(self.pipeline)

        # Output queue will be used to get the rgb frames from the output defined above
        self.queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    def timer_callback(self):
        image_rgb = self.queue.tryGet()  # non-blocking call
        if image_rgb is None:
            return
        # data is originally represented as a flat 1D array, it needs to be converted into HxWxC form
        frame = image_rgb.getCvFrame()

        # Convert the image to base64
        retval, buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality_factor]
        )
        jpg_as_text = base64.b64encode(buffer)

        msg = String()
        msg.data = jpg_as_text.decode("utf-8")  # convert bytes to string
        self.current_image = msg.data
        self.publisher_.publish(msg)

    def timer_period_callback(self, msg):
        self.timer_period = msg.data
        self.timer.cancel()  # cancel the old timer
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback
        )  # create a new timer with updated period

    def quality_factor_callback(self, msg):
        self.quality_factor = msg.data

    def preview_size_callback(self, msg):
        self.preview_width, self.preview_height = msg.data

        # Reset pipeline with new preview size
        self.device.close()
        self.init_pipeline()


def spin_camera(times):
    cnt = times
    if cnt == 0:
        print(
            "Couldn't restart camera due to displayed error/s, publishing error message"
        )
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
