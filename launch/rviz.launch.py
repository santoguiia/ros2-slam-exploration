#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch RViz2 for visualization.
    """
    # Get package directory
    pkg_ros2_slam_exploration = get_package_share_directory('ros2_slam_exploration')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(
                pkg_ros2_slam_exploration, 'rviz', 'slam_view.rviz'
            ),
            description='Full path to the RViz configuration file'
        ),

        # Start RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
