import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    param_file = LaunchConfiguration('param_file')

    param_declare = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(get_package_share_directory('scout_demo_node'), 'param', 'ydlidar.yaml'),
        description='FPath to the ROS2 parameters file to use.'
        )

    start_lidar_node = Node(
        executable='ydlidar_driver_node',
        name='lidar',
        package='scout_demo_node',
        namespace='scout',
        parameters=[param_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        param_declare,
        start_lidar_node
    ])