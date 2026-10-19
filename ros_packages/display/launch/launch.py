from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def fail_launch_on_display_failure(event, context):
    # ros2 launch exits 0 after a child failed unless an event handler raises.
    if event.returncode != 0 and not context.is_shutdown:
        raise RuntimeError(f"display exited with exit status {event.returncode}")
    return None


def generate_launch_description():
    display = Node(package="display", executable="display")
    return LaunchDescription(
        [
            display,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=display, on_exit=fail_launch_on_display_failure
                )
            ),
        ]
    )
