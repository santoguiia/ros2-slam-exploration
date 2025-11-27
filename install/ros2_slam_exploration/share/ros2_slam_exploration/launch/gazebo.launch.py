#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command # <--- Adicionado Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros2_slam_exploration = get_package_share_directory('ros2_slam_exploration')
    
    # --- CORREÇÃO 2.0: Usando Xacro para limpar o ${namespace} ---
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_file_name = 'turtlebot3_' + turtlebot3_model + '.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name
    )
    
    # Em vez de ler o arquivo direto, usamos o xacro para processar
    # Isso remove o ${namespace} que está causando o erro no RViz
    robot_desc = Command(['xacro ', urdf_path])
    # -------------------------------------------------------------

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_ros2_slam_exploration, 'worlds', 'exploration_world.world'),
            description='Full path to the world model file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Publicador do Estado do Robô (Agora com o URDF processado)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),

        # Spawn do Robô
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items()
        ),
    ])