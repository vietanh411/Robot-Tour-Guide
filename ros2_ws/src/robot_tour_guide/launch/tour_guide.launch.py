"""Launches the full tour-guide stack on top of an already-running Nav2.

Assumes:
  * The TurtleBot 4 bringup is already running (publishes /scan, /odom,
    /oakd/rgb/preview/image_raw, /oakd/rgb/preview/camera_info, /tf).
  * Nav2 + slam_toolbox (or AMCL with a saved map) are running and providing
    the navigate_to_pose action server and /amcl_pose.

Run:
  ros2 launch robot_tour_guide tour_guide.launch.py
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
    enable_safety = LaunchConfiguration('enable_safety_monitor')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Single YAML file with parameters for every node.',
        ),
        DeclareLaunchArgument(
            'enable_safety_monitor',
            default_value='false',
            description='Run safety_monitor (only enable on the Pi to keep latency low).',
        ),

        Node(package='robot_tour_guide', executable='aruco_detector',
             name='aruco_detector', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='semantic_perception',
             name='semantic_perception', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='world_model',
             name='world_model', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='tour_planner',
             name='tour_planner', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='executive',
             name='executive', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='narrator',
             name='narrator', parameters=[params_file], output='screen'),

        Node(package='robot_tour_guide', executable='safety_monitor',
             name='safety_monitor', parameters=[params_file], output='screen',
             condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(enable_safety)),
    ])
