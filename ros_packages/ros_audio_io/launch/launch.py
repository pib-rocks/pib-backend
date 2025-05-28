import os

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    mic = os.getenv('MIC_DEVICE', 'default').lower()

    ld = LaunchDescription()

    # Stream raw audio
    ld.add_action(
        Node(
            package='ros_audio_io',
            executable='audio_streamer',
            name='audio_streamer',
            output='screen',
    ))

    # Only do DOA if user asked for the Respeaker
    if mic == 'respeaker':
        ld.add_action(
            Node(
                package='ros_audio_io',
                executable='doa_publisher',
                name='doa_publisher',
                output='screen',
        ))
    else:
        ld.add_action(
            LogInfo(
                msg=f"[launch] MIC_DEVICE={mic!r}, skipping doa_publisher")
            )

    return ld
