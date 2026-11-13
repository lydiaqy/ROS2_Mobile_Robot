import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Locate the installed robot description package
    description_dir = get_package_share_directory(
        'mobile_robot_description'
    )

    spawn_launch_path = os.path.join(
        description_dir,
        'launch',
        'spawn_robot.launch.py'
    )

    # Reuse the existing Gazebo and robot spawning launch file
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_path)
    )

    # Start the C++ square path controller
    square_controller = Node(
        package='mobile_robot_control',
        executable='square_path_controller',
        name='square_path_controller',
        output='screen',
        parameters=[{
            'side_length': 1.0,
            'max_linear_speed': 0.2,
            'max_angular_speed': 0.4,
        }]
    )

    return LaunchDescription([
        spawn_robot,

        # Initial startup delay; replace with readiness detection later
        TimerAction(
            period=8.0,
            actions=[square_controller]
        ),
    ])