"""Pattern-based tour: ArUco detection + landmark announcement + open-loop
movement following config/pattern.yaml. No Nav2, no map, no AMCL required.

Run:
  ros2 launch robot_tour_guide pattern_tour.launch.py
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
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Single YAML file with parameters for every node.',
        ),

        Node(package='robot_tour_guide', executable='aruco_detector',
             name='aruco_detector', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='landmark_announcer',
             name='landmark_announcer', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='narrator',
             name='narrator', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='pattern_tour',
             name='pattern_tour', parameters=[params_file], output='screen'),
    ])
