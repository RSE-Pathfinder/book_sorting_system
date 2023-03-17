#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    scout_bringup_dir = get_package_share_directory('scout_demo_node')
    rviz_config_file = os.path.join(get_package_share_directory('bss_shelf_bringup'), 'rviz','cartographer.rviz')

    #Start Scout Base with lidar
    start_base_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(scout_bringup_dir, 'launch', 'scout_base_bringup_launch.py')))
    
    #Start SLAM
    start_cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(scout_bringup_dir, 'launch', 'scout_cartographer.launch.py')))

    start_rviz_cmd = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    )

    ld = LaunchDescription()
    ld.add_action(start_base_bringup_cmd)
    ld.add_action(start_cartographer_cmd)
    # ld.add_action(start_rviz_cmd)

    return ld