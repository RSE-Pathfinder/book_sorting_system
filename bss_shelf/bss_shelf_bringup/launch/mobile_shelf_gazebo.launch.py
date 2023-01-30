import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Constants for paths to different files and folders
    rviz_config_file_path = 'rviz/scout_v2.rviz'
    package_name = 'bss_shelf_bringup'

############ You do not need to change anything below this line #############

    # Set the path to different files and folders.  
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
    # Launch RViz
    start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    # Start Scout Spawning Sequence
    start_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mobile_shelf_spawner.launch.py')
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_spawner_cmd)
    
    return ld