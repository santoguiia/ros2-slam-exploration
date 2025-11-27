#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch slam_toolbox for 2D SLAM mapping.
    """
    # Get package directory
    pkg_ros2_slam_exploration = get_package_share_directory('ros2_slam_exploration')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_ros2_slam_exploration, 'config', 'slam_toolbox_params.yaml'
            ),
            description='Full path to the slam_toolbox configuration file'
        ),

        # Start slam_toolbox node in async mode for online SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
    ])
