import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    #Change the sdf path here
    urdf_file_name = 'models/bookver5/model.sdf'

    #Set position and orientation of sdf
    position = [0.70, -0.22, 1.23]
    orientation = [0.00, 0.66, 0.00]

    
    personalPath = FindPackageShare(package='bss_gazebo').find('bss_gazebo')
    fullPath = os.path.join(personalPath,urdf_file_name)
    
    #Debugger, sometimes the variable fullPath does not give absolute path which results in errors
    #print("\n\n")
    #print(personalPath)
    #print(fullPath)
    #print("\n\n")

    return LaunchDescription([
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-file", fullPath, "-entity", "book",
            "-x", str(position[0]), "-y", str(position[1]), "-z", str(position[2]),
            "-R",str(orientation[0]), "-P",str(orientation[1]),"-Y",str(orientation[2])
            ]
            )
    ])