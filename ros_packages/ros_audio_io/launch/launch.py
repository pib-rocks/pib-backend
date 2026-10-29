import os

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    mic = os.getenv("MIC_DEVICE", "default").lower()

    ld = LaunchDescription()

    # Stream raw audio
    ld.add_action(
        Node(
            package="ros_audio_io",
            executable="audio_streamer",
            name="audio_streamer",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        )
    )

    # Control the default output sink volume
    ld.add_action(
        Node(
            package="ros_audio_io",
            executable="volume_service",
            name="volume_service",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        )
    )

    # Only do DOA if user asked for the Respeaker
    if mic == "respeaker":
        ld.add_action(
            Node(
                package="ros_audio_io",
                executable="doa_publisher",
                name="doa_publisher",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            )
        )
    else:
        ld.add_action(
            LogInfo(msg=f"[launch] MIC_DEVICE={mic!r}, skipping doa_publisher")
        )

    return ld
