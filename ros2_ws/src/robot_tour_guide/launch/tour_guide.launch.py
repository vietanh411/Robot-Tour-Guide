"""Single launch file for the Robot Tour Guide demo.

Brings up everything needed to run a planned tour through the four landmarks
in REPF B4 and back to the starting point:

  * Nav2 localization (AMCL) against a pre-built map.
  * Nav2 navigation (planner + controller + recovery behaviors).
  * The full tour-guide stack (perception, world model, tour planner,
    executive, narrator, safety monitor).

Robot-side prerequisites that must already be running before this launch:
  * On the Pi: TurtleBot 4 bringup, OAK-D camera, LiDAR motor.
      ssh student@<robot>.cs.nor.ou.edu
      ros2 launch turtlebot4_bringup robot.launch.py
      ros2 launch turtlebot4_bringup oakd.launch.py
      ros2 service call /start_motor std_srvs/srv/Empty "{}"
  * On the desktop: robot-setup.sh (sets ROS_DOMAIN_ID + discovery server).

After launch: in RViz, click "2D Pose Estimate" on the robot's true location
so AMCL can localize. The executive captures that pose as the home point and
returns there once the tour ends.

Usage::

    ros2 launch robot_tour_guide tour_guide.launch.py

Optional arguments::

    map:=/abs/path/to/your_map.yaml      # default = packaged maps/repf_b4_map.yaml
    params_file:=/abs/path/to/params.yaml
    bringup_nav2:=false                  # skip if Nav2 already running elsewhere
    enable_safety_monitor:=false         # disable the LiDAR e-stop reflex
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_tour_guide')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'repf_b4_map.yaml')

    # turtlebot4_navigation may be missing on a non-robot dev machine. Resolve
    # lazily so the launch description still imports; the conditional include
    # only fires when bringup_nav2 is true.
    try:
        tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
    except PackageNotFoundError:
        tb4_nav_share = ''

    map_arg = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    bringup_nav2 = LaunchConfiguration('bringup_nav2')
    enable_safety = LaunchConfiguration('enable_safety_monitor')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Absolute path to the map YAML used by AMCL.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Single YAML with parameters for every tour-guide node.',
        ),
        DeclareLaunchArgument(
            'bringup_nav2',
            default_value='true',
            description='Include turtlebot4_navigation localization + nav2.',
        ),
        DeclareLaunchArgument(
            'enable_safety_monitor',
            default_value='true',
            description='Run safety_monitor (LiDAR forward-arc emergency stop).',
        ),

        # Nav2 localization (map_server + AMCL).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb4_nav_share, 'launch', 'localization.launch.py')
            ),
            launch_arguments={'map': map_arg}.items(),
            condition=IfCondition(bringup_nav2),
        ),

        # Nav2 (planner + controller + recoveries).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb4_nav_share, 'launch', 'nav2.launch.py')
            ),
            condition=IfCondition(bringup_nav2),
        ),

        # ----- Tour-guide stack -------------------------------------------------
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
             condition=IfCondition(enable_safety)),
    ])
