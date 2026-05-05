"""Bring up only perception (ArUco + semantic + world model + narrator).

Useful for the ArUco-only demo path: drive the robot manually with
teleop_twist_keyboard and confirm that descriptions print as you point the
camera at each marker.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_tour_guide')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(package='robot_tour_guide', executable='aruco_detector',
             name='aruco_detector', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='semantic_perception',
             name='semantic_perception', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='world_model',
             name='world_model', parameters=[params_file], output='screen'),

        # Lightweight executive substitute: announce descriptions when markers are sighted.
        Node(package='robot_tour_guide', executable='landmark_announcer',
             name='landmark_announcer', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='narrator',
             name='narrator', parameters=[params_file], output='screen'),
    ])
