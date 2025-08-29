# File: ~/ros2_humble/src/moveit/launch/bringup.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define the paths to the necessary files and directories
    urdf_file = os.path.join(
        get_package_share_directory('moveit'),
        'config',
        'ur5.urdf.xacro'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('moveit'),
        'config',
        'moveit.rviz'
    )

    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('gui', default_value='true', description='Flag to enable joint_state_publisher_gui'))

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )
    
    # Joint state broadcaster node
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # Load and start controllers
    load_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller],
            output='screen',
        )
        for controller in ['arm_controller', 'joint_state_broadcaster']
    ]
    
    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        gazebo_launch,
        rviz_node,
    ] + load_controllers)
