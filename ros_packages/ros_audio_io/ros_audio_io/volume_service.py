import subprocess

import rclpy
from datatypes.srv import SetVolume
from rclpy.node import Node


class VolumeService(Node):
    def __init__(self):
        super().__init__("volume_service")
        self.service = self.create_service(
            SetVolume, "set_volume", self.set_volume
        )
        self.get_logger().info("Volume service ready (set_volume)")

    def set_volume(self, request, response):
        percent = max(0, min(100, int(request.percent)))
        command = [
            "pactl",
            "set-sink-volume",
            "@DEFAULT_SINK@",
            f"{percent}%",
        ]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
            )
        except OSError as error:
            response.successful = False
            self.get_logger().error(
                f"Failed to set output volume to {percent}%: {error}"
            )
            return response

        response.successful = result.returncode == 0
        if response.successful:
            self.get_logger().info(f"Output volume set to {percent}%")
        else:
            error = result.stderr.strip() or result.stdout.strip()
            self.get_logger().error(
                f"pactl failed to set output volume to {percent}% "
                f"(exit {result.returncode}): {error}"
            )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VolumeService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
