"""Sim launch: brings up the tour-guide stack against a Gazebo Harmonic
TurtleBot 4 + Nav2 stack the user is expected to start separately.

Run (in three terminals):

  # 1. Sim + bringup (Gazebo Harmonic with TurtleBot 4)
  ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

  # 2. Nav2 + slam_toolbox (or supply a map)
  ros2 launch turtlebot4_navigation nav2.launch.py
  ros2 launch turtlebot4_navigation slam.launch.py

  # 3. This file
  ros2 launch robot_tour_guide tour_guide_sim.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_tour_guide')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for the tour-guide nodes.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'tour_guide.launch.py')
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'enable_safety_monitor': 'false',
            }.items(),
        ),
    ])
