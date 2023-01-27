import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    urdf_file_name = 'models/bookver5/model.sdf'

    # print("urdf_file_name : {}".format(urdf_file_name))
    
    personalPath = FindPackageShare(package='bss_gazebo').find('bss_gazebo')
    fullPath = os.path.join(personalPath,urdf_file_name)
    
    print("\n\n")
    print(personalPath)
    print(fullPath)
    print("\n\n")

    return LaunchDescription([
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-file", fullPath, "-entity", "book"]
            ),
    ])