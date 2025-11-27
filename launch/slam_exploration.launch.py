#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Main launch file to start the complete SLAM exploration system.
    This launches Gazebo, slam_toolbox, and RViz together.
    """
    # Get package directory
    pkg_ros2_slam_exploration = get_package_share_directory('ros2_slam_exploration')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch Gazebo with TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros2_slam_exploration, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Launch SLAM toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros2_slam_exploration, 'launch', 'slam.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Launch RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros2_slam_exploration, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])
