import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launch_dir = get_package_share_directory('scout_demo_node')

    start_base_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch', 'scout_base_launch.py')))

    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch', 'scout_lidar_launch.py')))
    
    ld = LaunchDescription()
    ld.add_action(start_base_cmd)
    ld.add_action(start_lidar_cmd)

    return ld