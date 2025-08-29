#!/usr/bin/python3

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

# Generate launch description:
def generate_launch_description():
    
    # Define Gazebo world and launch Gazebo:
    gazebo_world_path = os.path.join(
        get_package_share_directory('arm'),
        'world',
        'empty_world.world')  # Use an empty world for simplicity
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': gazebo_world_path}.items(),
    )

    # Load the robot description (URDF):
    urdf_file = os.path.join(
        get_package_share_directory('arm'),
        'urdf',
        'ur5.urdf')
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Spawn the robot in Gazebo:
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'your_robot'],
        output='screen'
    )

    # Robot state publisher (for robot description in ROS):
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Static transform (world to base_link):
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]
    )


    # Return launch description with Gazebo, robot spawn, and necessary nodes:
    return LaunchDescription([
        gazebo_launch,
        spawn_robot,
        robot_state_publisher,
        static_transform,
    ])

