#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    declare_slam_param_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("scout_demo_node"),
                                   'param', 'slam_online_async.yaml'),
        description='Full path to the ROS 2 parameters file to use for slam_toolbox node'
    )

    #Start Scout Node
    scout_node = launch_ros.actions.Node(
                    package='scout_demo_node',
                    executable='scout_node',
                    name='scout_base_node')
    
    #Start SLAM Node
    start_async_slam_toolbox_node = launch_ros.actions.Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    #Launch Lidar
    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'scout_lidar_launch.py')))

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam_param_file)
    # ld.add_action(scout_node)
    ld.add_action(start_lidar_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld