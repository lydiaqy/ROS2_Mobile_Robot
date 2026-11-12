
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_dir = get_package_share_directory(
        "mobile_robot_control"
    )

    config_file = os.path.join(
        package_dir,
        "config",
        "velocity.yaml"
    )

    publisher = Node(
        package="mobile_robot_control",
        executable="velocity_publisher",
        name="velocity_publisher",
        output="screen",
        parameters=[config_file]
    )

    subscriber = Node(
        package="mobile_robot_control",
        executable="velocity_subscriber",
        name="velocity_subscriber",
        output="screen"
    )

    return LaunchDescription([
        publisher,
        subscriber
    ])
