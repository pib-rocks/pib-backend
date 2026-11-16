from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="motors", executable="motor_control"),
            # respawn: the telemetry node talks to the serial buses and must come
            # back on its own if a bus error ever escapes a callback.
            Node(
                package="motors",
                executable="motor_current",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(package="motors", executable="controller_scan"),
            Node(package="motors", executable="relay_control"),
            Node(package="motors", executable="rgb_button_control"),
        ]
    )
