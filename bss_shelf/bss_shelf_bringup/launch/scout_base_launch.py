import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    scout_node = launch_ros.actions.Node(
                    package='scout_demo_node',
                    executable='scout_node',
                    name='scout')
    
    ld = LaunchDescription()
    ld.add_action(scout_node)

    return ld