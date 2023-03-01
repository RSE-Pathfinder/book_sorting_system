from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='laser_static_tf',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_footprint','base_laser'],
                    namespace='scout_base_node'
                    )
    rviz2_node = Node(package='rviz2',
                    node_executable='rviz2',
                    node_name='rviz2',
                    arguments=['-d', rviz_config_file],
                    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        # rviz2_node
    ])