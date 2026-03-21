"""
Lab 08 Launch File

Launches the goal controller, trajectory publisher, and plotter.
Assumes the Crazyflie driver (crazyswarm2) is already running.

Usage:
    # With default (known-good) gains:
    ros2 launch lab08_control lab08.launch.py

    # With student gains:
    ros2 launch lab08_control lab08.launch.py pid_file:=pid_student.yaml

    # With custom waypoints:
    ros2 launch lab08_control lab08.launch.py waypoints_file:=/path/to/waypoints.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lab08_control')
    config_dir = os.path.join(pkg_share, 'config')

    # --- Launch arguments ---
    pid_file_arg = DeclareLaunchArgument(
        'pid_file',
        default_value=os.path.join(config_dir, 'pid_defaults.yaml'),
        description='Path to PID gains YAML file'
    )

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(config_dir, 'waypoints.yaml'),
        description='Path to waypoints YAML file'
    )

    # --- Nodes ---
    goal_controller = Node(
        package='lab08_control',
        executable='goal_controller',
        name='goal_controller',
        parameters=[LaunchConfiguration('pid_file')],
        output='screen',
    )

    trajectory_publisher = Node(
        package='lab08_control',
        executable='trajectory_publisher',
        name='mission_controller',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
        }],
        output='screen',
    )

    plotter = Node(
        package='lab08_control',
        executable='plotter',
        name='trajectory_plotter',
        output='screen',
    )

    return LaunchDescription([
        pid_file_arg,
        waypoints_file_arg,
        goal_controller,
        trajectory_publisher,
        plotter,
    ])
