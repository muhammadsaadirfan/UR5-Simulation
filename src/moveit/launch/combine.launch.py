from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():
    # Define package and file paths
    namePackage = 'arm'
    modelFileRelativePath = 'urdf/robot.xacro'
    worldFileRelativePath = 'urdf/empty_world.world'
    rvizConfigFileRelativePath = 'config/robot.rviz'
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    pathRvizConfigFile = os.path.join(get_package_share_directory(namePackage), rvizConfigFileRelativePath)

    # Process the Xacro file to get the robot description
    robot_description = xacro.process_file(pathModelFile).toxml()

    # Define nodes
    gazeboLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'combine.launch.py')),
        launch_arguments={'world': pathWorldFile}.items())

    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'differential_drive_robot'],
        output='screen')

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    nodeRviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', pathRvizConfigFile])

    # Create LaunchDescription
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(nodeRviz)

    return launchDescriptionObject
