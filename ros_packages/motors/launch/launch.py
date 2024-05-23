import time

from launch_ros.actions import Node
from pib_motors.update_bricklet_uids import *

from launch import LaunchDescription


def check_and_update_uids():
    i = 1
    while True:
        try:
            if no_uids_in_database():
                update_uids()
            print("Updated UIDs")
            break
        except Exception as e:
            print(f"Cannot update uids [{i}], trying again:\n {e}")
            time.sleep(5)
        i += 1


def generate_launch_description():
    check_and_update_uids()
    return LaunchDescription(
        [
            Node(package="motors", executable="motor_control"),
            Node(package="motors", executable="motor_current"),
        ]
    )
