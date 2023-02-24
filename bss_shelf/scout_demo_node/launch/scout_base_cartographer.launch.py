import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    pkg_share = get_package_share_directory('scout_demo_node')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='scout_demo_node',
            executable='scout_node',
            name='scout'),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'param/ekf.yaml'),
            ]
        ),
  ])